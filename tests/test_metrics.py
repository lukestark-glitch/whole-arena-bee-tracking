"""Tests for experiments.metrics: matching, swaps, ghosts, log reading and truth collection."""

from __future__ import annotations

import json
import math
import socket
import threading
import time
import warnings
from pathlib import Path

from experiments.metrics import (
    MATCH_GATE_M,
    Metrics,
    _collect_truth,
    compute_metrics,
    format_metrics,
    read_track_log,
)
from shared.protocol import UDP_HOST, Truth, TruthBee, encode_truth

FRAMES = 10
BEE_A = (0.0, 0.0, 0.0)
BEE_B = (1.0, 0.0, 0.0)


def _truth(frames: int = FRAMES) -> dict:
    """Two stationary bees, 1 m apart, for the given number of frames."""
    return {f: {0: BEE_A, 1: BEE_B} for f in range(frames)}


def _shift(pos: tuple[float, float, float], dx_m: float) -> tuple[float, float, float]:
    """Move a position along x by dx_m metres."""
    return (pos[0] + dx_m, pos[1], pos[2])


def test_perfect_tracking() -> None:
    """Perfect tracks give zero error, no swaps, no ghosts, full shares."""
    tracks = {f: {10: BEE_A, 11: BEE_B} for f in range(FRAMES)}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert m.frames == FRAMES
    assert m.mean_error_m == 0.0 and m.p95_error_m == 0.0
    assert m.id_swaps == 0 and m.ghost_tracks == 0 and m.track_count == 2
    assert m.tracked_share == {0: 1.0, 1: 1.0}


def test_constant_offset() -> None:
    """A constant 3 mm offset gives 3 mm mean and 95th percentile error."""
    tracks = {f: {10: _shift(BEE_A, 0.003), 11: _shift(BEE_B, 0.003)} for f in range(FRAMES)}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert math.isclose(m.mean_error_m, 0.003, abs_tol=1e-9)
    assert math.isclose(m.p95_error_m, 0.003, abs_tol=1e-9)


def test_ids_exchange_bees() -> None:
    """Two track ids exchanging bees half-way count as two swaps."""
    tracks = {}
    for f in range(FRAMES):
        a, b = (10, 11) if f < FRAMES // 2 else (11, 10)
        tracks[f] = {a: BEE_A, b: BEE_B}
    assert compute_metrics(_truth(), tracks, MATCH_GATE_M).id_swaps == 2


def test_replaced_after_gap() -> None:
    """A new id after a gap is one swap and lowers the tracked share."""
    tracks = {}
    for f in range(FRAMES):
        if f < 4:
            tracks[f] = {10: BEE_A, 11: BEE_B}
        elif f < 6:
            tracks[f] = {11: BEE_B}
        else:
            tracks[f] = {12: BEE_A, 11: BEE_B}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert m.id_swaps == 1
    assert m.tracked_share[0] < 1.0 and m.tracked_share[1] == 1.0


def test_ghost_track() -> None:
    """A track far from every bee in all its frames is one ghost."""
    far = (5.0, 5.0, 5.0)
    tracks = {f: {10: BEE_A, 11: BEE_B, 12: far} for f in range(FRAMES)}
    assert compute_metrics(_truth(), tracks, MATCH_GATE_M).ghost_tracks == 1


def test_briefly_far_track_is_no_ghost() -> None:
    """A track that is far for only 2 of 10 frames is not a ghost."""
    tracks = {}
    for f in range(FRAMES):
        tracks[f] = {10: (5.0, 5.0, 5.0) if f < 2 else BEE_A, 11: BEE_B}
    assert compute_metrics(_truth(), tracks, MATCH_GATE_M).ghost_tracks == 0


def test_track_beyond_gate_does_not_track() -> None:
    """A track beyond the gate does not count as tracking the bee."""
    tracks = {f: {10: _shift(BEE_A, 2 * MATCH_GATE_M), 11: BEE_B} for f in range(FRAMES)}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert m.tracked_share[0] == 0.0 and m.tracked_share[1] == 1.0


def test_frames_after_last_log_frame_are_ignored() -> None:
    """Truth frames after the last log frame are not evaluated."""
    tracks = {f: {10: BEE_A, 11: BEE_B} for f in range(5)}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert m.frames == 5
    assert m.tracked_share == {0: 1.0, 1: 1.0}


def test_truth_frame_without_log_line_is_untracked() -> None:
    """A truth frame missing from the log counts as no tracks."""
    tracks = {f: {10: BEE_A, 11: BEE_B} for f in range(FRAMES) if f != 3}
    m = compute_metrics(_truth(), tracks, MATCH_GATE_M)
    assert m.frames == FRAMES
    assert math.isclose(m.tracked_share[0], 0.9)


def test_no_matches_gives_nan_without_warnings() -> None:
    """With no matches at all the errors are nan and nothing is raised or warned."""
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        m = compute_metrics(_truth(), {f: {} for f in range(FRAMES)}, MATCH_GATE_M)
    assert math.isnan(m.mean_error_m) and math.isnan(m.p95_error_m)
    assert m.tracked_share == {0: 0.0, 1: 0.0}


def test_read_track_log(tmp_path: Path) -> None:
    """Only confirmed tracks are read and an empty frame maps to an empty dict."""
    def track(track_id: int, confirmed: bool) -> dict:
        return {"id": track_id, "pos": [0.1, 0.2, 0.3], "vel": [0, 0, 0], "acc": [0, 0, 0],
                "state": "flying", "confirmed": confirmed, "cameras": 5}

    path = tmp_path / "tracks.jsonl"
    lines = [
        {"frame": 1, "t": 0.1, "tracks": [track(4, True), track(5, False)]},
        {"frame": 2, "t": 0.2, "tracks": []},
    ]
    path.write_text("\n".join(json.dumps(line) for line in lines) + "\n", encoding="utf-8")
    assert read_track_log(path) == {1: {4: (0.1, 0.2, 0.3)}, 2: {}}


def test_format_metrics_contains_key_numbers() -> None:
    """The report shows errors in mm, counts and shares in percent."""
    m = Metrics(10, 0.003, 0.0045, 2, 1, 3, {0: 1.0, 7: 0.5})
    text = format_metrics(m)
    for expected in ("3.00 mm", "4.50 mm", "2", "75.0 %", "50.0 % (bee 7)"):
        assert expected in text


def test_collect_truth_over_udp() -> None:
    """Truth frames up to the last frame are collected over real UDP."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind((UDP_HOST, 0))
        port = probe.getsockname()[1]
    result: list[dict] = []
    thread = threading.Thread(target=lambda: result.append(_collect_truth(port, 1)))
    thread.start()
    time.sleep(0.3)
    bee = TruthBee(id=0, pos=(0.1, 0.1, 0.1), vel=(0.0, 0.0, 0.0), state="flying")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for frame, t_s in enumerate((0.0, 0.5, 1.0)):  # frame 2 is after the last frame
            for datagram in encode_truth(Truth(frame=frame, t=t_s, bees=(bee,))):
                sender.sendto(datagram, (UDP_HOST, port))
    thread.join(timeout=5.0)
    assert not thread.is_alive()
    assert result[0] == {0: {0: (0.1, 0.1, 0.1)}, 1: {0: (0.1, 0.1, 0.1)}}
