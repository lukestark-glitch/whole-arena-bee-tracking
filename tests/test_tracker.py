"""Tests for the tracker as a whole: import rules and tracking of simulated bees.

The end-to-end tests run simulator and tracker in one process, without UDP: the
simulator's datagrams are decoded and handed to the tracker frame by frame, and the
result is measured with `experiments.metrics`.
"""

import ast
import dataclasses
from pathlib import Path
from typing import Any

from experiments.metrics import MATCH_GATE_M, Metrics, compute_metrics
from shared.config import Config, frame_rate_hz, last_frame_of, load_config
from shared.protocol import Detections, Truth, decode
from simulator.main import create_simulation, step_simulation
from tracker.main import create_tracker, process_frame
from tracker.receiver import FrameAssembler

REPO = Path(__file__).parent.parent
CONFIG = load_config(REPO / "config" / "default.yaml")
FORBIDDEN_PACKAGES = {"simulator", "robot_emulator"}
# Modules that read the wall clock; the tracker may only use simulation time.
CLOCK_MODULES = {"time", "datetime"}
RUN_FRAMES = 600  # 10 s of simulation time


def _imported_packages(path: Path) -> set[str]:
    """Return the top-level packages a Python file imports."""
    packages: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            packages.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            packages.add(node.module.split(".")[0])
    return packages


def _run_offline(config: Config, seed: int, frames: int) -> tuple[list[dict[str, Any]], Metrics]:
    """Simulate and track in one process; return the track records and the metrics."""
    simulation = create_simulation(config, seed, 1.0)
    period_s = 1.0 / frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames * period_s
    )
    tracker = create_tracker(config)
    records: list[dict[str, Any]] = []
    truth: dict[int, dict[int, tuple[float, float, float]]] = {}
    for _ in range(frames):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if isinstance(message, Truth):
                truth.setdefault(message.frame, {}).update({b.id: b.pos for b in message.bees})
            elif isinstance(message, Detections):
                records.extend(process_frame(tracker, f) for f in assembler.add(message))
    tracks = {
        record["frame"]: {t["id"]: tuple(t["pos"]) for t in record["tracks"] if t["confirmed"]}
        for record in records
    }
    return records, compute_metrics(truth, tracks, MATCH_GATE_M)


def test_tracker_does_not_import_the_simulator() -> None:
    """No module in tracker/ imports from simulator/ or robot_emulator/."""
    for path in (REPO / "tracker").glob("*.py"):
        assert not _imported_packages(path) & FORBIDDEN_PACKAGES, path.name


def test_tracker_does_not_use_the_wall_clock() -> None:
    """No module in tracker/ imports a clock module."""
    for path in (REPO / "tracker").glob("*.py"):
        assert not _imported_packages(path) & CLOCK_MODULES, path.name


def test_last_frame_of_duration() -> None:
    """A 60 s run at 60 Hz ends with frame 3599; a part of a period counts as started."""
    assert last_frame_of(CONFIG, 60.0) == 3599
    assert last_frame_of(CONFIG, 0.02) == 1


def test_tracks_ten_bees_with_default_noise() -> None:
    """Ten bees are tracked for 10 s with small error, few swaps and no ghosts."""
    records, metrics = _run_offline(CONFIG, seed=1, frames=RUN_FRAMES)
    assert len(records) == RUN_FRAMES
    assert metrics.mean_error_m < 0.002
    assert metrics.ghost_tracks == 0
    assert sum(metrics.tracked_share.values()) / len(metrics.tracked_share) > 0.9


def test_same_input_gives_the_same_tracks() -> None:
    """The tracker is deterministic: the same detections give identical records."""
    first, _ = _run_offline(CONFIG, seed=3, frames=120)
    second, _ = _run_offline(CONFIG, seed=3, frames=120)
    assert first == second


def test_tracking_without_noise_is_nearly_exact() -> None:
    """Without pixel noise and dropout the mean error stays below 0.5 mm (filter lag only)."""
    quiet = dataclasses.replace(
        CONFIG.noise, pixel_sigma_px=0.0, dropout_probability=0.0
    )
    _, metrics = _run_offline(dataclasses.replace(CONFIG, noise=quiet), seed=1, frames=300)
    assert metrics.mean_error_m < 0.0005
