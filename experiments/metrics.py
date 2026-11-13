"""Quality metrics: compare the tracker's log with the simulator's ground truth.

This is the only module that reads both the tracker output (its JSON-lines log file) and the
ground truth (the ``truth`` UDP messages). It imports nothing from ``tracker`` or
``simulator``. Run it next to a simulator and a tracker:

    python -m experiments.metrics --tracks runs/tracks.jsonl --duration 60
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from shared.config import last_frame_of, load_config
from shared.protocol import UDP_HOST, ProtocolError, Truth, decode

Position = tuple[float, float, float]
FramePositions = dict[int, dict[int, Position]]

# A track farther than this from every bee counts as matching no bee (a bit more than one
# body length of a bumblebee).
MATCH_GATE_M = 0.02
# Wall-clock wait after the truth ends so the tracker can finish writing its log. It does not
# change which frames are compared.
LOG_GRACE_S = 1.0
# Percentile reported as the "bad case" position error.
ERROR_PERCENTILE = 95.0
# Largest UDP datagram we accept when listening for truth.
MAX_DATAGRAM_BYTES = 65535
# Socket timeout so that a wait for datagrams stays interruptible.
SOCKET_TIMEOUT_S = 0.2
# Conversions for the printed report.
MM_PER_M = 1000.0
PERCENT = 100.0


@dataclass(frozen=True)
class Metrics:
    """Quality numbers of one run.

    Attributes:
        frames: Number of frames evaluated.
        mean_error_m: Mean distance of matched track-bee pairs in metres (nan if none).
        p95_error_m: 95th percentile of that distance in metres (nan if none).
        id_swaps: Number of times a bee was matched to a different track id than before.
        ghost_tracks: Confirmed track ids that matched no bee in over half of their frames.
        track_count: Distinct confirmed track ids seen.
        tracked_share: Bee id -> share of its frames in which it was matched (0..1).
    """

    frames: int
    mean_error_m: float
    p95_error_m: float
    id_swaps: int
    ghost_tracks: int
    track_count: int
    tracked_share: dict[int, float]


@dataclass
class _Tally:
    """Running counts while walking over the frames."""

    errors_m: list[float]
    swaps: int
    last_track_of_bee: dict[int, int]
    bee_frames: dict[int, int]
    bee_matched: dict[int, int]
    track_frames: dict[int, int]
    track_unmatched: dict[int, int]


def read_track_log(path: Path) -> FramePositions:
    """Read the tracker log and keep the confirmed tracks.

    Args:
        path: JSON-lines file written by the tracker, one line per processed frame.

    Returns:
        Frame number -> {track id -> position (x, y, z) in metres, arena frame}. A frame
        without a confirmed track maps to an empty dict.

    Raises:
        OSError: If the file cannot be read.
        ValueError: If a line is not valid JSON of the expected shape.
    """
    frames: FramePositions = {}
    with path.open(encoding="utf-8") as log_file:
        for line in log_file:
            if not line.strip():
                continue
            record = json.loads(line)
            frames[int(record["frame"])] = {
                int(track["id"]): _as_position(track["pos"])
                for track in record["tracks"]
                if track["confirmed"]
            }
    return frames


def _as_position(values: list[float]) -> Position:
    """Convert a JSON list of three numbers to a position tuple.

    Args:
        values: [x, y, z] in metres.

    Returns:
        The same numbers as a tuple.
    """
    return (float(values[0]), float(values[1]), float(values[2]))


def _match_frame(
    bees: dict[int, Position], tracks: dict[int, Position], match_gate_m: float
) -> list[tuple[int, int, float]]:
    """Match tracks to bees in one frame, one-to-one, minimising total distance.

    Args:
        bees: Bee id -> true position in metres.
        tracks: Track id -> estimated position in metres.
        match_gate_m: Pairs farther apart than this are not matches, in metres.

    Returns:
        List of (bee id, track id, distance in metres) for the accepted pairs.
    """
    if not bees or not tracks:
        return []
    bee_ids = list(bees)
    track_ids = list(tracks)
    bee_xyz = np.array([bees[i] for i in bee_ids])
    track_xyz = np.array([tracks[i] for i in track_ids])
    # Euclidean distance of every bee-track pair: the cost the assignment minimises.
    distance_m = np.linalg.norm(bee_xyz[:, None, :] - track_xyz[None, :, :], axis=2)
    rows, cols = linear_sum_assignment(distance_m)
    return [
        (bee_ids[r], track_ids[c], float(distance_m[r, c]))
        for r, c in zip(rows, cols)
        if distance_m[r, c] <= match_gate_m
    ]


def _count_frame(
    tally: _Tally,
    bees: dict[int, Position],
    tracks: dict[int, Position],
    pairs: list[tuple[int, int, float]],
) -> None:
    """Add one frame's matching result to the running counts.

    Args:
        tally: Counts to update in place.
        bees: Bee id -> true position in metres.
        tracks: Track id -> estimated position in metres.
        pairs: Accepted (bee id, track id, distance in metres) pairs of this frame.
    """
    matched_tracks = {track_id for _, track_id, _ in pairs}
    for bee_id in bees:
        tally.bee_frames[bee_id] = tally.bee_frames.get(bee_id, 0) + 1
    for track_id in tracks:
        tally.track_frames[track_id] = tally.track_frames.get(track_id, 0) + 1
        if track_id not in matched_tracks:
            tally.track_unmatched[track_id] = tally.track_unmatched.get(track_id, 0) + 1
    for bee_id, track_id, distance_m in pairs:
        tally.errors_m.append(distance_m)
        tally.bee_matched[bee_id] = tally.bee_matched.get(bee_id, 0) + 1
        previous = tally.last_track_of_bee.get(bee_id)
        if previous is not None and previous != track_id:
            tally.swaps += 1
        tally.last_track_of_bee[bee_id] = track_id


def _error_statistics(errors_m: list[float]) -> tuple[float, float]:
    """Mean and 95th percentile of the position errors.

    Args:
        errors_m: Distances of matched pairs in metres.

    Returns:
        (mean, 95th percentile) in metres; both nan when the list is empty.
    """
    if not errors_m:
        return math.nan, math.nan
    return float(np.mean(errors_m)), float(np.percentile(errors_m, ERROR_PERCENTILE))


def _count_ghosts(tally: _Tally) -> int:
    """Count tracks that matched no bee in more than half of their frames.

    Args:
        tally: Counts after all frames were processed.

    Returns:
        Number of ghost track ids.
    """
    return sum(
        1
        for track_id, seen in tally.track_frames.items()
        if 2 * tally.track_unmatched.get(track_id, 0) > seen
    )


def compute_metrics(
    truth: FramePositions, tracks: FramePositions, match_gate_m: float
) -> Metrics:
    """Compare tracker output with ground truth.

    Only truth frames up to the last frame of the track log are evaluated, because the tracker
    may still be behind when the truth ends. A truth frame without a log line counts as "no
    tracks".

    Args:
        truth: Frame number -> {bee id -> true position in metres, arena frame}.
        tracks: Frame number -> {track id -> position in metres, arena frame}, confirmed
            tracks only.
        match_gate_m: Largest bee-track distance that still counts as a match, in metres.

    Returns:
        The quality numbers.
    """
    last_frame = max(tracks, default=-1)
    tally = _Tally([], 0, {}, {}, {}, {}, {})
    frames = sorted(f for f in truth if f <= last_frame)
    for frame in frames:
        bees = truth[frame]
        frame_tracks = tracks.get(frame, {})
        _count_frame(tally, bees, frame_tracks, _match_frame(bees, frame_tracks, match_gate_m))
    mean_m, p95_m = _error_statistics(tally.errors_m)
    share = {
        bee_id: tally.bee_matched.get(bee_id, 0) / seen
        for bee_id, seen in sorted(tally.bee_frames.items())
    }
    return Metrics(
        frames=len(frames),
        mean_error_m=mean_m,
        p95_error_m=p95_m,
        id_swaps=tally.swaps,
        ghost_tracks=_count_ghosts(tally),
        track_count=len(tally.track_frames),
        tracked_share=share,
    )


def format_metrics(metrics: Metrics) -> str:
    """Build the printed report.

    Args:
        metrics: Result of `compute_metrics`.

    Returns:
        Several lines of text; errors in millimetres, shares in percent.
    """
    lines = [
        f"frames evaluated:   {metrics.frames}",
        f"mean error:         {metrics.mean_error_m * MM_PER_M:.2f} mm",
        f"95th pct error:     {metrics.p95_error_m * MM_PER_M:.2f} mm",
        f"id swaps:           {metrics.id_swaps}",
        f"ghost tracks:       {metrics.ghost_tracks}",
        f"confirmed tracks:   {metrics.track_count}",
    ]
    if metrics.tracked_share:
        shares = metrics.tracked_share
        # Mean of the per-bee shares, so that every bee weighs the same.
        mean_share = sum(shares.values()) / len(shares)
        worst_id = min(shares, key=lambda bee_id: shares[bee_id])
        lines.append(f"mean tracked share: {mean_share * PERCENT:.1f} %")
        lines.append(
            f"lowest tracked:     {shares[worst_id] * PERCENT:.1f} % (bee {worst_id})"
        )
    return "\n".join(lines)


def _collect_truth(port: int, last_frame: int | None) -> FramePositions:
    """Listen for truth messages and collect the bee positions per frame.

    Stops on Ctrl+C, or once the last frame has arrived and the socket falls silent. The
    silence is awaited because the truth of one frame can come in several datagrams.

    Args:
        port: UDP port to bind on the loopback host.
        last_frame: Number of the last frame to collect, or None to run until Ctrl+C.

    Returns:
        Frame number -> {bee id -> position in metres, arena frame}.
    """
    frames: FramePositions = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((UDP_HOST, port))
        sock.settimeout(SOCKET_TIMEOUT_S)
        try:
            while not (_receive_into(sock, frames, last_frame) and _is_silent(sock)):
                pass
        except KeyboardInterrupt:
            pass
    return frames


def _receive_into(sock: socket.socket, frames: FramePositions, last_frame: int | None) -> bool:
    """Receive one truth message, store its bees and tell whether the last frame is in.

    Args:
        sock: Bound, timeout-enabled UDP socket.
        frames: Collected positions, extended in place. Frames after last_frame are not kept.
        last_frame: Number of the last frame to collect, or None for no end.

    Returns:
        True once a message of the last frame or a later one has arrived.
    """
    message = _receive_truth(sock)
    if message is None:
        return last_frame is not None and last_frame in frames
    if last_frame is None or message.frame <= last_frame:
        frames.setdefault(message.frame, {}).update({bee.id: bee.pos for bee in message.bees})
    return last_frame is not None and message.frame >= last_frame


def _is_silent(sock: socket.socket) -> bool:
    """Tell whether no further datagram arrives within the socket timeout.

    Args:
        sock: Bound, timeout-enabled UDP socket.

    Returns:
        True on a timeout; False if a datagram was waiting (it stays in the queue).
    """
    try:
        sock.recv(MAX_DATAGRAM_BYTES, socket.MSG_PEEK)
    except TimeoutError:
        return True
    return False


def _receive_truth(sock: socket.socket) -> Truth | None:
    """Receive one datagram and return it if it is a valid truth message.

    Args:
        sock: Bound, timeout-enabled UDP socket.

    Returns:
        The truth message, or None on timeout, a malformed datagram or another message type.
    """
    try:
        datagram, _ = sock.recvfrom(MAX_DATAGRAM_BYTES)
        message = decode(datagram)
    except (TimeoutError, ProtocolError):
        return None
    return message if isinstance(message, Truth) else None


def main() -> None:
    """Collect truth, read the tracker log, print the quality report.

    Raises:
        SystemExit: On invalid command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tracks", type=Path, required=True, help="tracker log file")
    parser.add_argument("--config", type=Path, default=Path("config/default.yaml"))
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    parser.add_argument("--match-gate", type=float, default=MATCH_GATE_M, help="metres")
    args = parser.parse_args()
    config = load_config(args.config)
    last_frame = None if args.duration is None else last_frame_of(config, args.duration)
    truth = _collect_truth(config.ports.truth, last_frame)
    _wait_for_log()
    tracks = read_track_log(args.tracks)
    print(format_metrics(compute_metrics(truth, tracks, args.match_gate)))


def _wait_for_log() -> None:
    """Wait on the wall clock so the tracker can finish writing its log."""

    time.sleep(LOG_GRACE_S)


if __name__ == "__main__":
    main()
