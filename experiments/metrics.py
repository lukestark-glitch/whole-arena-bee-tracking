"""Quality metrics: compare the tracker's log with the simulator's ground truth.

This is the only module that reads both the tracker output (its JSON-lines log file) and the
ground truth (the ``truth`` UDP messages). It imports nothing from ``tracker`` or
``simulator``. Run it next to a simulator and a tracker:

    python -m experiments.metrics --tracks runs/tracks.jsonl --duration 60

Besides the tracking quality it reports, when a bee was selected in the tracker, how long the
selected bee was inside the cinema camera's frame and inside its depth of field. For that it
uses the robot axes that come with the ``truth`` messages and the robot file (``--robot``).
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

from shared.config import Robot, last_frame_of, load_config, load_robot
from shared.geometry import cinema_view, robot_camera_pose
from shared.protocol import UDP_HOST, ProtocolError, RobotAxes, Truth, decode

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


@dataclass(frozen=True)
class RobotMetrics:
    """How well the cinema camera followed the selected bee.

    Attributes:
        frames: Frames in which a bee was selected and the robot axes are known.
        in_frame_share: Share of those frames with the bee inside the image (0..1, nan if
            no frames).
        in_focus_share: Share of those frames with the bee inside the image and within the
            depth of field (0..1, nan if no frames).
    """

    frames: int
    in_frame_share: float
    in_focus_share: float


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


def read_selection(path: Path) -> dict[int, int]:
    """Read which track was selected in each frame from the tracker log.

    Args:
        path: JSON-lines file written by the tracker.

    Returns:
        Frame number -> selected track id, only for frames with a selection. Lines without
        the ``selected`` key (older logs) or with null count as "nothing selected".

    Raises:
        OSError: If the file cannot be read.
        ValueError: If a line is not valid JSON.
    """
    selection: dict[int, int] = {}
    with path.open(encoding="utf-8") as log_file:
        for line in log_file:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("selected") is not None:
                selection[int(record["frame"])] = int(record["selected"])
    return selection


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


def _followed_bee_in_view(
    robot: Robot,
    axes: RobotAxes,
    bees: dict[int, Position],
    track_pos: Position | None,
    match_gate_m: float,
) -> tuple[bool, bool]:
    """Tell whether the bee behind the selected track is in frame and in focus.

    Args:
        robot: Robot description (rail, camera mount, cinema camera).
        axes: Robot axes of this frame (rail in metres, tilt in radians, focus in metres).
        bees: Bee id -> true position in metres, arena frame.
        track_pos: Position of the selected track in metres, arena frame; None if the track
            is not in the log of this frame.
        match_gate_m: Largest track-bee distance that still identifies the bee, in metres.

    Returns:
        (in frame, in focus). Both are False when the bee cannot be identified.
    """
    if track_pos is None or not bees:
        return False, False
    distance_m = {i: math.dist(pos, track_pos) for i, pos in bees.items()}
    bee_id = min(distance_m, key=lambda i: distance_m[i])
    if distance_m[bee_id] > match_gate_m:
        return False, False
    pose = robot_camera_pose(robot, axes.rail_m, axes.tilt_rad)
    in_frame, depth_m = cinema_view(pose, robot.cinema, np.array([bees[bee_id]]))
    # The depth of field is centred on the focus plane, so half of it lies on each side.
    half_depth_m = robot.cinema.depth_of_field_m / 2
    in_focus = bool(in_frame[0]) and abs(float(depth_m[0]) - axes.focus_m) <= half_depth_m
    return bool(in_frame[0]), in_focus


def compute_robot_metrics(
    truth: FramePositions,
    axes: dict[int, RobotAxes],
    tracks: FramePositions,
    selection: dict[int, int],
    robot: Robot,
    match_gate_m: float,
) -> RobotMetrics:
    """Measure how long the selected bee is in the cinema frame and in focus.

    Args:
        truth: Frame number -> {bee id -> true position in metres, arena frame}.
        axes: Frame number -> robot axes from the truth messages.
        tracks: Frame number -> {track id -> position in metres, arena frame}.
        selection: Frame number -> selected track id.
        robot: Robot description.
        match_gate_m: Largest track-bee distance that still identifies the bee, in metres.

    Returns:
        Shares over the frames that have a selection, truth and robot axes.
    """
    frames = sorted(f for f in selection if f in truth and f in axes)
    in_frame_count = 0
    in_focus_count = 0
    for frame in frames:
        track_pos = tracks.get(frame, {}).get(selection[frame])
        in_frame, in_focus = _followed_bee_in_view(
            robot, axes[frame], truth[frame], track_pos, match_gate_m
        )
        in_frame_count += in_frame
        in_focus_count += in_focus
    if not frames:
        return RobotMetrics(0, math.nan, math.nan)
    return RobotMetrics(len(frames), in_frame_count / len(frames), in_focus_count / len(frames))


def format_robot_metrics(metrics: RobotMetrics) -> str:
    """Build the printed robot report.

    Args:
        metrics: Result of `compute_robot_metrics`.

    Returns:
        Three lines of text; shares in percent.
    """
    return "\n".join(
        [
            f"frames with target: {metrics.frames}",
            f"time in frame:      {metrics.in_frame_share * PERCENT:.1f} %",
            f"time in focus:      {metrics.in_focus_share * PERCENT:.1f} %",
        ]
    )


def _collect_truth(
    port: int, last_frame: int | None
) -> tuple[FramePositions, dict[int, RobotAxes]]:
    """Listen for truth messages and collect the bee positions and robot axes per frame.

    Stops on Ctrl+C, or once the last frame has arrived and the socket falls silent. The
    silence is awaited because the truth of one frame can come in several datagrams.

    Args:
        port: UDP port to bind on the loopback host.
        last_frame: Number of the last frame to collect, or None to run until Ctrl+C.

    Returns:
        (frame number -> {bee id -> position in metres, arena frame},
        frame number -> robot axes, for frames whose truth carried them).
    """
    frames: FramePositions = {}
    axes: dict[int, RobotAxes] = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((UDP_HOST, port))
        sock.settimeout(SOCKET_TIMEOUT_S)
        try:
            while not (_receive_into(sock, frames, axes, last_frame) and _is_silent(sock)):
                pass
        except KeyboardInterrupt:
            pass
    return frames, axes


def _receive_into(
    sock: socket.socket,
    frames: FramePositions,
    axes: dict[int, RobotAxes],
    last_frame: int | None,
) -> bool:
    """Receive one truth message, store its bees and tell whether the last frame is in.

    Args:
        sock: Bound, timeout-enabled UDP socket.
        frames: Collected positions, extended in place. Frames after last_frame are not kept.
        axes: Collected robot axes per frame, extended in place under the same rule.
        last_frame: Number of the last frame to collect, or None for no end.

    Returns:
        True once a message of the last frame or a later one has arrived.
    """
    message = _receive_truth(sock)
    if message is None:
        return last_frame is not None and last_frame in frames
    if last_frame is None or message.frame <= last_frame:
        frames.setdefault(message.frame, {}).update({bee.id: bee.pos for bee in message.bees})
        if message.robot is not None:
            axes[message.frame] = message.robot
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
    parser.add_argument("--robot", type=Path, default=Path("config/robot.yaml"))
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    parser.add_argument("--match-gate", type=float, default=MATCH_GATE_M, help="metres")
    args = parser.parse_args()
    config = load_config(args.config)
    last_frame = None if args.duration is None else last_frame_of(config, args.duration)
    truth, axes = _collect_truth(config.ports.truth, last_frame)
    _wait_for_log()
    tracks = read_track_log(args.tracks)
    print(format_metrics(compute_metrics(truth, tracks, args.match_gate)))
    robot_metrics = compute_robot_metrics(
        truth, axes, tracks, read_selection(args.tracks), load_robot(args.robot), args.match_gate
    )
    if robot_metrics.frames > 0:
        print(format_robot_metrics(robot_metrics))


def _wait_for_log() -> None:
    """Wait on the wall clock so the tracker can finish writing its log."""

    time.sleep(LOG_GRACE_S)


if __name__ == "__main__":
    main()
