"""Calibration end to end in one process: wand run, calibration file, tracking, marker check."""

import dataclasses
import math
from functools import cache
from pathlib import Path

import numpy as np
import pytest

from shared.config import frame_rate_hz, load_config, load_robot
from shared.protocol import Detections, RobotPose, Truth, decode
from shared.wand import wand_duration_s
from simulator.main import Simulation, apply_control, create_simulation, step_simulation
from tracker.calibrate import pose_difference
from tracker.main import Tracker, create_tracker, process_frame
from tracker.receiver import FrameAssembler
from tracker.wand_run import Calibration, calibrate, load_calibration, save_calibration

REPO = Path(__file__).parent.parent
CONFIG = load_config(REPO / "config" / "default.yaml")
ROBOT = load_robot(REPO / "config" / "robot.yaml")
RATE_HZ = frame_rate_hz(CONFIG)
TRACKING_FRAMES = 600  # 10 s of bees
MATCH_GATE_M = 0.02


def _assembler() -> FrameAssembler:
    """Build a frame assembler for the configured cameras."""
    return FrameAssembler(
        [camera.id for camera in CONFIG.cameras], CONFIG.tracker.frame_timeout_frames / RATE_HZ
    )


@cache
def _wand_run() -> tuple[Simulation, Calibration]:
    """Play a wand run on a miscalibrated simulator and calibrate from it (computed once)."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0, miscalibrated=True, wand=True)
    assembler = _assembler()
    frames, poses = [], []
    for _ in range(round(wand_duration_s(CONFIG.calibration) * RATE_HZ)):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if isinstance(message, Detections):
                frames.extend(assembler.add(message))
            elif isinstance(message, RobotPose):
                poses.append(message)
    return simulation, calibrate(CONFIG, ROBOT, frames, poses)


def _mean_error_m(miscalibrated: bool, calibration: Calibration | None) -> float:
    """Track ten bees for 10 s and return the mean distance of each bee to its nearest track."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0, miscalibrated=miscalibrated)
    tracker = create_tracker(CONFIG, ROBOT, calibration=calibration)
    # The marker check would repair wrong cameras by itself; here the calibration is on test.
    tracker.calibration = dataclasses.replace(CONFIG.calibration, markers=())
    assembler = _assembler()
    truth: dict[int, np.ndarray] = {}
    errors_m = []
    for _ in range(TRACKING_FRAMES):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if isinstance(message, Truth):
                truth[message.frame] = np.array([bee.pos for bee in message.bees])
            elif isinstance(message, Detections):
                for frame in assembler.add(message):
                    record = process_frame(tracker, frame)
                    tracks = np.array([t["pos"] for t in record["tracks"] if t["confirmed"]])
                    if len(tracks):
                        gaps = np.linalg.norm(truth[frame.frame][:, None] - tracks[None], axis=2)
                        nearest = gaps.min(axis=1)
                        errors_m.extend(nearest[nearest < MATCH_GATE_M])
    return float(np.mean(errors_m)) if errors_m else math.inf


def test_wand_run_recovers_cameras_and_robot() -> None:
    """With pixel noise the cameras come out within 1 mm and 0.1 degree, the robot within 1 mm."""
    simulation, calibration = _wand_run()
    for camera_id, true_pose in simulation.poses.items():
        distance_m, angle_rad = pose_difference(true_pose, calibration.cameras[camera_id])
        assert distance_m < 0.001 and math.degrees(angle_rad) < 0.1
    true_start = np.array(simulation.controller.robot.rail_start_m)
    assert np.linalg.norm(np.array(calibration.rail_start_m) - true_start) < 0.001
    assert calibration.lens_offset_m == pytest.approx(ROBOT.lens_offset_m, abs=0.001)
    assert calibration.focus_scale == pytest.approx(1.0, abs=0.005)


def test_tracking_with_own_calibration_matches_the_baseline() -> None:
    """On miscalibrated cameras the self-computed calibration is within 20 % of the baseline."""
    baseline_m = _mean_error_m(miscalibrated=False, calibration=None)
    calibrated_m = _mean_error_m(miscalibrated=True, calibration=_wand_run()[1])
    uncalibrated_m = _mean_error_m(miscalibrated=True, calibration=None)
    assert calibrated_m < 1.2 * baseline_m
    assert uncalibrated_m > 3.0 * baseline_m


def test_calibration_file_round_trip(tmp_path: Path) -> None:
    """A saved calibration loads with the same values; another file is refused."""
    calibration = _wand_run()[1]
    path = tmp_path / "calibration.json"
    save_calibration(path, calibration)
    loaded = load_calibration(path)
    assert loaded.rail_start_m == calibration.rail_start_m
    assert loaded.focus_scale == calibration.focus_scale
    for camera_id, pose in calibration.cameras.items():
        assert np.array_equal(loaded.cameras[camera_id].position_m, pose.position_m)
        assert np.array_equal(loaded.cameras[camera_id].rotation, pose.rotation)
    path.write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError):
        load_calibration(path)


def _track(simulation: Simulation, tracker: Tracker, frames: int) -> None:
    """Feed simulated frames to a tracker."""
    assembler = _assembler()
    for _ in range(frames):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if isinstance(message, Detections):
                for frame in assembler.add(message):
                    process_frame(tracker, frame)


def test_displaced_camera_is_reported_and_corrected() -> None:
    """A camera turned by 0.02 rad is found by the marker check and gets its true pose back."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    tracker = create_tracker(CONFIG, ROBOT)
    _track(simulation, tracker, 30)
    assert tracker.corrections == {}
    apply_control(simulation, "camera_displaced", {"camera_id": "cam-06", "angle_rad": 0.02})
    _track(simulation, tracker, 61)
    assert list(tracker.corrections) == ["cam-06"]
    correction = tracker.corrections["cam-06"]
    assert correction["error_px"] > CONFIG.calibration.marker_threshold_px > correction["after_px"]
    camera = next(c for c in tracker.cameras if c.id == "cam-06")
    _, angle_rad = pose_difference(camera.pose, simulation.poses["cam-06"])
    assert math.degrees(angle_rad) < 0.2
