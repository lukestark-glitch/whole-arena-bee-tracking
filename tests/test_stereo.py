"""Tests for the robot stereo camera emulation."""

from dataclasses import replace
from pathlib import Path

import numpy as np

from robot_emulator.stereo import observe_stereo
from shared.config import load_robot
from shared.geometry import focal_length_px, robot_camera_pose

BODY_M = 0.015
ROBOT = load_robot(Path("config/robot.yaml"))
POSE = robot_camera_pose(ROBOT, 0.45, 0.0)
QUIET = replace(ROBOT.stereo, pixel_sigma_px=0.0, size_sigma_px=0.0, disparity_sigma_px=0.0)


def _ahead(distance_m: float) -> np.ndarray:
    """Return one bee on the optical axis at the given distance, shape (1, 3)."""
    return (POSE.position_m + np.array([0.0, distance_m, 0.0]))[None, :]


def test_zero_noise_on_axis() -> None:
    """A bee on the axis gives one blob at the image centre with exact values."""
    blobs = observe_stereo(POSE, QUIET, _ahead(0.25), BODY_M, np.random.default_rng(0))
    assert len(blobs) == 1
    assert abs(blobs[0].u_px - 640) < 1e-6 and abs(blobs[0].v_px - 400) < 1e-6
    assert abs(blobs[0].distance_m - 0.25) < 1e-9
    assert abs(blobs[0].size_px - focal_length_px(QUIET.model) * BODY_M / 0.25) < 1e-6


def test_not_seen_cases() -> None:
    """Bees behind the camera or closer than a body length give no blob."""
    rng = np.random.default_rng(0)
    assert observe_stereo(POSE, QUIET, _ahead(-0.25), BODY_M, rng) == []
    assert observe_stereo(POSE, QUIET, _ahead(0.01), BODY_M, rng) == []
    assert observe_stereo(POSE, QUIET, np.zeros((0, 3)), BODY_M, rng) == []


def test_nearer_bee_is_larger() -> None:
    """A nearer bee gives a larger blob than a farther one."""
    both = np.vstack([_ahead(0.2), _ahead(0.5)])
    near, far = observe_stereo(POSE, QUIET, both, BODY_M, np.random.default_rng(0))
    assert near.size_px > far.size_px


def test_noise_statistics() -> None:
    """Distance and pixel noise match their expected standard deviations."""
    rng = np.random.default_rng(1)
    blobs = [observe_stereo(POSE, ROBOT.stereo, _ahead(0.25), BODY_M, rng)[0] for _ in range(2000)]
    focal_px = focal_length_px(ROBOT.stereo.model)
    expected_m = 0.25**2 * ROBOT.stereo.disparity_sigma_px / (focal_px * ROBOT.stereo.baseline_m)
    assert abs(np.std([b.distance_m - 0.25 for b in blobs]) / expected_m - 1) < 0.2
    assert abs(np.std([b.u_px - 640 for b in blobs]) / ROBOT.stereo.pixel_sigma_px - 1) < 0.2


def test_same_seed_same_blobs() -> None:
    """The same seed gives identical blobs."""
    a = observe_stereo(POSE, ROBOT.stereo, _ahead(0.3), BODY_M, np.random.default_rng(5))
    b = observe_stereo(POSE, ROBOT.stereo, _ahead(0.3), BODY_M, np.random.default_rng(5))
    assert a == b
