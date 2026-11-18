"""Stage 7 batch runner: every experiment, every seed, in one process each, into CSV files.

Each run puts simulator, tracker (with ``auto_select``) and robot emulator into one process,
as ``tests/test_robot_loop.py`` does, so a run depends only on its configuration and seed and
not on how busy the computer is. Datagrams are handed over in the order of their send time;
a datagram whose send time falls after the start of the next frame is delivered only after
that frame is simulated, so node delay and jitter act as they do over the network.

Truth datagrams are passed to ``experiments.metrics`` unread; that module computes every
number (D95). Results go to ``experiments/results/``: ``runs.csv`` (one row per run),
``summary.csv`` (mean over the seeds), ``criterion.csv`` (the camera layouts against D93),
``coverage.json`` and SVG plots. Run everything with:

    python -m experiments.batch

``--estimate`` only prints the expected run time; ``--seeds`` and ``--duration`` shorten it.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import itertools
import json
import math
import os
import statistics
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from multiprocessing import Pool
from pathlib import Path
from typing import Any

from experiments.coverage import GRID_SPACING_M, coverage_of, coverage_svg
from experiments.metrics import (
    MATCH_GATE_M,
    FramePositions,
    add_truth_datagram,
    compute_metrics,
    compute_prediction_error,
    compute_robot_metrics,
)
from experiments.plots import write_plots
from shared.config import Config, Robot, frame_rate_hz, last_frame_of, load_config, load_robot
from shared.protocol import Detections, RobotPose, decode, encode_target
from shared.wand import wand_duration_s
from simulator.main import apply_control, apply_target, create_simulation, step_simulation
from tracker.main import apply_robot_pose, create_tracker, process_frame, target_message
from tracker.receiver import FrameAssembler, frames_from_datagram
from tracker.wand_run import Calibration, calibrate

RESULTS = Path("experiments/results")
DEFAULT_CONFIG = Path("config/default.yaml")
DEFAULT_ROBOT = Path("config/robot.yaml")
CAMERA_CONFIGS = {
    16: DEFAULT_CONFIG,
    12: Path("config/cameras-12.yaml"),
    10: Path("config/cameras-10.yaml"),
    8: Path("config/cameras-8.yaml"),
}
SEEDS = (1, 2, 3, 4, 5)
# Simulated seconds per run. 20 s hold about 1200 frames and several flights per bee, and
# keep the whole batch under about 30 minutes on a laptop with 8 cores.
DURATION_S = 20.0
# The flights, not the start, are measured: tracks need a few frames to be confirmed, and
# trip bees start resting at the entrance. Frames before this time are not evaluated.
WARMUP_S = 2.0
# Prediction leads: one frame, and 100 ms as a stand-in for the robot's unknown delay.
PREDICTION_LEADS_S = (1.0 / 60.0, 0.1)
# Bees of the base case and of the camera comparison.
BASE_BEES = 10
CAMERA_BEES = 5
# Marker threshold that no camera reaches: switches the wall-marker correction off.
NO_MARKER_CHECK_PX = 1e9
# Slack when comparing send times with frame boundaries, in seconds.
TIME_EPSILON_S = 1e-9
# Cost of one simulated second of the base case per process, used only for the printed
# estimate. Measured 2026-11-18 on an 8-core Apple M-series laptop with 7 processes, where
# the full batch took 17 minutes; one process alone needs about 1.1 s.
SECONDS_PER_SIM_SECOND = 2.1
# Metrics whose spread over the seeds the summary also reports (standard deviation).
SPREAD_METRICS = ("p95_error_mm", "id_swaps", "in_frame_pct", "in_focus_pct")
# Worker processes: leave one core free for the rest of the computer.
WORKERS = max(1, (os.cpu_count() or 2) - 1)
MM_PER_M = 1000.0
PERCENT = 100.0
SECONDS_PER_MINUTE = 60.0

# Robot limit sets: the cautious file values, the earlier values of D74, and a hypothetical
# fast drive. Rail speed stays 3 m/s in all of them. (rail acc m/s^2, tilt rad/s, tilt
# rad/s^2, focus m/s, focus m/s^2)
ROBOT_LIMITS = {
    "cautious (file)": None,
    "moderate (D74)": (15.0, 12.0, 90.0, 1.0, 20.0),
    "fast (hypothetical)": (30.0, 20.0, 300.0, 2.0, 40.0),
}


@dataclass(frozen=True)
class Case:
    """One experimental condition; every field not set keeps the base value.

    Attributes:
        experiment: Name of the experiment the case belongs to.
        label: Short description of the condition.
        cameras: Number of cameras: selects the configuration file.
        bees: Number of bees.
        flight_model: ``random`` or ``trips``.
        pixel_sigma_px: Detection noise, or None for the file value.
        dropout_probability: Missed detections, or None for the file value.
        node_delay_s: Mean node delay in seconds, or None for the file value.
        node_delay_jitter_s: Standard deviation of the node delay in seconds, or None.
        stopped_nodes: Number of nodes that send nothing (the first ones of the file).
        calibration: ``true`` (cameras where the file says), ``none`` (miscalibrated, no
            correction), ``markers`` (miscalibrated, wall-marker correction) or ``wand``
            (miscalibrated, wand calibration and wall-marker correction).
        robot_limits: Key of ROBOT_LIMITS.
    """

    experiment: str
    label: str
    cameras: int = 16
    bees: int = BASE_BEES
    flight_model: str = "random"
    pixel_sigma_px: float | None = None
    dropout_probability: float | None = None
    node_delay_s: float | None = None
    node_delay_jitter_s: float | None = None
    stopped_nodes: int = 0
    calibration: str = "true"
    robot_limits: str = "cautious (file)"


def cases() -> list[Case]:
    """List every condition of the Stage 7 experiments.

    Returns:
        The cases, base case first in each experiment.
    """
    out = []
    for model in ("random", "trips"):
        out += [Case("bees", f"{n} bees, {model}", bees=n, flight_model=model)
                for n in (1, 3, 5, 10, 20)]
    for label, sigma, dropout in (("low", 0.25, 0.0), ("nominal", None, None),
                                  ("high", 1.0, 0.1)):
        out.append(Case("noise", f"noise {label}", pixel_sigma_px=sigma,
                        dropout_probability=dropout))
    out += [Case("nodes", f"{n} of 8 nodes stopped", stopped_nodes=n) for n in (0, 1, 2)]
    for cameras in (16, 12, 10, 8):
        for model in ("random", "trips"):
            out.append(Case("cameras", f"{cameras} cameras, {model}", cameras=cameras,
                            bees=CAMERA_BEES, flight_model=model))
    for model in ("random", "trips"):
        out.append(Case("cameras", f"8 cameras, 1 of 4 nodes stopped, {model}", cameras=8,
                        bees=CAMERA_BEES, flight_model=model, stopped_nodes=1))
    for delay_ms, jitter_ms in ((5, 1), (8, 2), (15, 5), (30, 10)):
        out.append(Case("delay", f"delay {delay_ms} ± {jitter_ms} ms",
                        node_delay_s=delay_ms / 1000, node_delay_jitter_s=jitter_ms / 1000))
    out += [Case("calibration", f"calibration: {mode}", calibration=mode)
            for mode in ("true", "none", "markers", "wand")]
    out += [Case("robot", f"robot limits {name}", robot_limits=name) for name in ROBOT_LIMITS]
    return out


def case_config(case: Case) -> tuple[Config, Robot]:
    """Build the configuration and the robot file of a case.

    Args:
        case: The condition.

    Returns:
        The configuration and the robot description the tracker is given (nominal values).
    """
    config = load_config(CAMERA_CONFIGS[case.cameras])
    noise_changes = {name: getattr(case, name) for name in
                     ("pixel_sigma_px", "dropout_probability", "node_delay_s",
                      "node_delay_jitter_s") if getattr(case, name) is not None}
    config = replace(
        config,
        noise=replace(config.noise, **noise_changes),
        bees=replace(config.bees, count=case.bees, flight_model=case.flight_model),
    )
    if case.calibration == "none":
        config = replace(config, calibration=replace(
            config.calibration, marker_threshold_px=NO_MARKER_CHECK_PX))
    return config, _robot_with_limits(load_robot(DEFAULT_ROBOT), ROBOT_LIMITS[case.robot_limits])


def _robot_with_limits(robot: Robot, limits: tuple[float, ...] | None) -> Robot:
    """Replace the acceleration and speed limits of a robot.

    Args:
        robot: Robot from the file.
        limits: (rail acc m/s^2, tilt rad/s, tilt rad/s^2, focus m/s, focus m/s^2), or None
            to keep the file values.

    Returns:
        The robot with the new limits; rail speed unchanged.
    """
    if limits is None:
        return robot
    rail_acc, tilt_speed, tilt_acc, focus_speed, focus_acc = limits
    return replace(
        robot,
        rail=replace(robot.rail, max_acceleration=rail_acc),
        tilt=replace(robot.tilt, max_speed=tilt_speed, max_acceleration=tilt_acc),
        focus=replace(robot.focus, max_speed=focus_speed, max_acceleration=focus_acc),
    )


def wand_calibration(config: Config, robot: Robot, seed: int) -> Calibration:
    """Simulate a wand run with miscalibrated cameras and compute the calibration from it.

    Args:
        config: Configuration (cameras, wand schedule).
        robot: Robot file values.
        seed: Seed; the camera errors depend on it, so the bee run must use the same.

    Returns:
        The calibration the tracker would load with ``--calibration``.
    """
    simulation = create_simulation(config, robot, seed, 1.0, miscalibrated=True, wand=True)
    period_s = 1.0 / frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames * period_s
    )
    last_frame = last_frame_of(config, wand_duration_s(config.calibration))
    frames, poses = [], []
    # A few extra frames push the last frame of the run out of the assembler.
    for _ in range(last_frame + 2 + int(config.tracker.frame_timeout_frames)):
        for item in step_simulation(simulation):
            if item.port == config.ports.detections:
                frames.extend(frames_from_datagram(assembler, item.datagram))
            elif item.port == config.ports.robot_pose:
                poses.append(decode(item.datagram))
    return calibrate(config, robot, [f for f in frames if f.frame <= last_frame], poses)


@dataclass
class _Log:
    """What one run collected for the metrics."""

    truth: FramePositions = field(default_factory=dict)
    axes: dict = field(default_factory=dict)
    tracks: FramePositions = field(default_factory=dict)
    velocities: FramePositions = field(default_factory=dict)
    selection: dict[int, int] = field(default_factory=dict)


def run_case(case: Case, seed: int, duration_s: float) -> dict[str, Any]:
    """Run one case with one seed and compute its metrics.

    Args:
        case: The condition.
        seed: Seed of the run.
        duration_s: Simulated seconds.

    Returns:
        One CSV row: the case fields, the seed and every metric (mm, %, counts).
    """
    started_s = time.perf_counter()
    config, robot = case_config(case)
    miscalibrated = case.calibration != "true"
    calibration = wand_calibration(config, robot, seed) if case.calibration == "wand" else None
    simulation = create_simulation(config, robot, seed, 1.0, miscalibrated=miscalibrated)
    for node in config.nodes[: case.stopped_nodes]:
        apply_control(simulation, "node_stopped", {"node_id": node.id, "stopped": True})
    tracker = create_tracker(config, robot, auto_select=True, calibration=calibration)
    period_s = 1.0 / frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames * period_s
    )
    log = _Log()
    deliver = _deliverer(config, simulation, tracker, assembler, log)
    queue: list = []
    order = itertools.count()
    for frame in range(last_frame_of(config, duration_s) + 1):
        for item in step_simulation(simulation):
            heapq.heappush(queue, (item.send_time_s, next(order), item))
        next_frame_s = (frame + 1) * period_s
        while queue and queue[0][0] <= next_frame_s + TIME_EPSILON_S:
            deliver(heapq.heappop(queue)[2])
    # The robot numbers need the robot as it really is: with --miscalibrated its rail start
    # differs from the file the tracker was given.
    return _row(case, seed, duration_s, config, simulation.controller.robot, log, period_s,
                time.perf_counter() - started_s)


def _deliverer(
    config: Config, simulation: Any, tracker: Any, assembler: FrameAssembler, log: _Log
) -> Callable[[Any], None]:
    """Return the function that hands one datagram to its receiver.

    Args:
        config: Configuration (ports).
        simulation: The simulation; receives `target`.
        tracker: The tracker; receives detections and `robot_pose`.
        assembler: Frame assembler of the tracker.
        log: Collected data, extended in place.

    Returns:
        A function of one `Outgoing` item.
    """
    ports = config.ports

    def deliver(item: Any) -> None:
        """Hand one datagram to the metrics, the tracker or the robot.

        Args:
            item: An `Outgoing` datagram of the simulation.
        """
        if item.port == ports.truth:
            add_truth_datagram(log.truth, log.axes, item.datagram)
            return
        if item.port == ports.robot_pose:
            apply_robot_pose(tracker, item.datagram)
            return
        for frame in frames_from_datagram(assembler, item.datagram):
            record = process_frame(tracker, frame)
            confirmed = [t for t in record["tracks"] if t["confirmed"]]
            log.tracks[record["frame"]] = {t["id"]: tuple(t["pos"]) for t in confirmed}
            log.velocities[record["frame"]] = {t["id"]: tuple(t["vel"]) for t in confirmed}
            if record["selected"] is not None:
                log.selection[record["frame"]] = record["selected"]
            target = target_message(tracker)
            if target is not None:
                apply_target(simulation, encode_target(target))

    return deliver


def _after_warmup(frames: dict, first_frame: int) -> dict:
    """Keep the entries of the frames from ``first_frame`` on.

    Args:
        frames: Frame number -> anything.
        first_frame: First frame to keep.

    Returns:
        A new dict without the earlier frames.
    """
    return {frame: value for frame, value in frames.items() if frame >= first_frame}


def _row(
    case: Case,
    seed: int,
    duration_s: float,
    config: Config,
    robot: Robot,
    log: _Log,
    period_s: float,
    wall_s: float,
) -> dict[str, Any]:
    """Compute every metric of a run and flatten it into a CSV row.

    Args:
        case: The condition.
        seed: Seed of the run.
        duration_s: Simulated seconds.
        config: Configuration of the run.
        robot: The true robot of the simulation (geometry and limits).
        log: Collected data.
        period_s: Frame period in seconds.
        wall_s: Wall-clock time the run took, in seconds.

    Returns:
        The row; lengths in millimetres, shares in percent.
    """
    first = round(WARMUP_S / period_s)
    truth = _after_warmup(log.truth, first)
    tracks = _after_warmup(log.tracks, first)
    metrics = compute_metrics(truth, tracks, MATCH_GATE_M)
    robot_metrics = compute_robot_metrics(truth, log.axes, tracks,
                                          _after_warmup(log.selection, first), robot, MATCH_GATE_M)
    shares = list(metrics.tracked_share.values())
    minutes = (duration_s - WARMUP_S) / SECONDS_PER_MINUTE
    row = {**asdict(case), "seed": seed, "nodes": len(config.nodes),
           "duration_s": duration_s,
           "mean_error_mm": metrics.mean_error_m * MM_PER_M,
           "p95_error_mm": metrics.p95_error_m * MM_PER_M,
           "id_swaps": metrics.id_swaps,
           "id_swaps_per_bee_min": metrics.id_swaps / (case.bees * minutes),
           "ghost_tracks": metrics.ghost_tracks,
           "ghost_share_pct": metrics.ghost_share * PERCENT,
           "confirmed_tracks": metrics.track_count,
           "tracked_pct": (sum(shares) / len(shares) if shares else math.nan) * PERCENT,
           "lowest_tracked_pct": (min(shares) if shares else math.nan) * PERCENT}
    for lead_s in PREDICTION_LEADS_S:
        error = compute_prediction_error(truth, tracks, _after_warmup(log.velocities, first),
                                         round(lead_s / period_s), period_s, MATCH_GATE_M)
        name = f"prediction_{round(lead_s * 1000)}ms"
        row[f"{name}_mean_mm"] = error.mean_m * MM_PER_M
        row[f"{name}_p95_mm"] = error.p95_m * MM_PER_M
    row |= {"robot_frames": robot_metrics.frames,
            "in_frame_pct": robot_metrics.in_frame_share * PERCENT,
            "in_focus_pct": robot_metrics.in_focus_share * PERCENT,
            "follow_offset_mm": robot_metrics.mean_offset_m * MM_PER_M,
            "focus_error_mm": robot_metrics.mean_focus_error_m * MM_PER_M,
            "wall_s": round(wall_s, 1)}
    return row


def estimate_s(all_cases: list[Case], seeds: tuple[int, ...], duration_s: float) -> float:
    """Estimate the wall-clock time of the batch.

    The cost of a run grows with the number of bees and cameras; a wand run adds its own
    length.

    Args:
        all_cases: The cases.
        seeds: The seeds.
        duration_s: Simulated seconds per run.

    Returns:
        Estimated seconds with WORKERS processes.
    """
    total_s = 0.0
    for case in all_cases:
        # Cost model: a fixed part per frame plus a part per bee, scaled by the cameras.
        weight = (0.5 + 0.5 * case.bees / BASE_BEES) * case.cameras / 16
        wand_s = wand_duration_s(load_config(DEFAULT_CONFIG).calibration)
        run_s = duration_s + (wand_s if case.calibration == "wand" else 0.0)
        total_s += weight * run_s * SECONDS_PER_SIM_SECOND * len(seeds)
    return total_s / WORKERS


def summarise(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Average the numeric columns of the rows over the seeds of each case.

    Args:
        rows: One row per run.

    Returns:
        One row per (experiment, label) with the mean of every metric, the worst seed for
        ghost tracks, the standard deviation over the seeds of SPREAD_METRICS and the number
        of seeds.
    """
    out = []
    # Cases in the order they were run.
    for key in dict.fromkeys((r["experiment"], r["label"]) for r in rows):
        group = [r for r in rows if (r["experiment"], r["label"]) == key]
        summary = {name: group[0][name] for name in asdict(Case("", "")) | {"nodes": 0}}
        summary["seeds"] = len(group)
        for name, value in group[0].items():
            if name in summary or name == "seed" or isinstance(value, str):
                continue
            values = [r[name] for r in group if not _is_nan(r[name])]
            summary[name] = round(sum(values) / len(values), 3) if values else math.nan
        summary["ghost_tracks_max"] = max(r["ghost_tracks"] for r in group)
        for name in SPREAD_METRICS:
            values = [r[name] for r in group if not _is_nan(r[name])]
            summary[f"{name}_sd"] = round(statistics.pstdev(values), 3) if values else math.nan
        out.append(summary)
    return out


def _is_nan(value: Any) -> bool:
    """Tell whether a value is a float nan."""
    return isinstance(value, float) and math.isnan(value)


def criterion(summary: list[dict[str, Any]], coverage: dict[int, dict]) -> list[dict[str, Any]]:
    """Check every camera layout at CAMERA_BEES bees against the D93 criterion.

    Args:
        summary: Rows of `summarise`.
        coverage: Number of cameras -> coverage summary (share_by_level).

    Returns:
        One row per layout and flight model with each test and the verdict.
    """
    rows = {(r["cameras"], r["flight_model"]): r for r in summary
            if r["experiment"] == "cameras" and r["stopped_nodes"] == 0}
    out = []
    for (cameras, model), r in sorted(rows.items(), key=lambda kv: (-kv[0][0], kv[0][1])):
        ref = rows[(16, model)]
        tests = {
            "seen_by_3_pct": coverage[cameras]["share_by_level"][3] * PERCENT >= 95.0,
            "mean_error_le_1mm": r["mean_error_mm"] <= 1.0,
            "p95_error_le_2mm": r["p95_error_mm"] <= 2.0,
            "no_ghost_track": r["ghost_tracks_max"] == 0,
            "tracked_ge_95pct": r["tracked_pct"] >= 95.0,
            "id_swaps_le_1_5x": r["id_swaps"] <= 1.5 * max(ref["id_swaps"], 1.0),
            "in_frame_within_5": r["in_frame_pct"] >= ref["in_frame_pct"] - 5.0,
            "in_focus_within_5": r["in_focus_pct"] >= ref["in_focus_pct"] - 5.0,
        }
        failed = [name for name, ok in tests.items() if not ok]
        out.append({"cameras": cameras, "flight_model": model, **tests,
                    "verdict": "good enough" if not failed else "not good enough: " + ", ".join(failed)})
    return out


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    """Write rows with the same keys as a CSV file.

    Args:
        path: Output file.
        rows: Rows; the first one names the columns.
    """
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _run_job(job: tuple[Case, int, float]) -> dict[str, Any]:
    """Run one (case, seed, duration) job; a top-level function so worker processes can."""
    return run_case(*job)


def main() -> None:
    """Print the run-time estimate, run every case and seed, and write the result files."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seeds", type=int, default=len(SEEDS), help="use seeds 1..N")
    parser.add_argument("--duration", type=float, default=DURATION_S, help="simulated s per run")
    parser.add_argument("--estimate", action="store_true", help="only print the estimate")
    parser.add_argument("--out", type=Path, default=RESULTS)
    args = parser.parse_args()
    seeds = SEEDS[: args.seeds]
    all_cases = cases()
    jobs = [(case, seed, args.duration) for case in all_cases for seed in seeds]
    print(f"{len(all_cases)} cases x {len(seeds)} seeds = {len(jobs)} runs of "
          f"{args.duration:.0f} s on {WORKERS} processes; estimate "
          f"{estimate_s(all_cases, seeds, args.duration) / SECONDS_PER_MINUTE:.0f} min")
    if args.estimate:
        return
    args.out.mkdir(parents=True, exist_ok=True)
    coverage = {}
    for cameras, path in CAMERA_CONFIGS.items():
        summary, points_m, counts = coverage_of(path, GRID_SPACING_M)
        coverage[cameras] = asdict(summary)
        (args.out / f"coverage-{cameras}.svg").write_text(
            coverage_svg(summary, points_m, counts, GRID_SPACING_M), encoding="utf-8")
    (args.out / "coverage.json").write_text(
        json.dumps(list(coverage.values()), indent=1), encoding="utf-8")
    started_s = time.perf_counter()
    with Pool(WORKERS) as pool:
        rows = []
        for row in pool.imap(_run_job, jobs):
            rows.append(row)
            print(f"[{len(rows)}/{len(jobs)}] {row['label']} seed {row['seed']}: "
                  f"{row['mean_error_mm']:.2f} mm, {row['id_swaps']} swaps, "
                  f"in frame {row['in_frame_pct']:.0f} %", flush=True)
    summary = summarise(rows)
    write_csv(args.out / "runs.csv", rows)
    write_csv(args.out / "summary.csv", summary)
    write_csv(args.out / "criterion.csv", criterion(summary, coverage))
    write_plots(summary, coverage, args.out)
    print(f"done in {(time.perf_counter() - started_s) / SECONDS_PER_MINUTE:.1f} min; "
          f"results in {args.out}")


if __name__ == "__main__":
    main()
