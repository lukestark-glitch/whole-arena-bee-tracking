"""Tests for the robot calibration fit."""

import dataclasses
import math

import numpy as np
import pytest

from shared.config import Robot, load_robot
from tracker.robot_calibration import MIN_SAMPLES, RobotSample, fit_robot

ROBOT_PATH = "config/robot.yaml"
TRUE_START_M = (0.008, -0.156, 0.155)
TRUE_LENS_OFFSET_M = 0.02
SAMPLE_COUNT = 15


def _true_robot() -> Robot:
    """Robot with the true rail start and lens offset."""
    return dataclasses.replace(
        load_robot(ROBOT_PATH),
        rail_start_m=TRUE_START_M,
        lens_offset_m=TRUE_LENS_OFFSET_M,
    )


def _points(rng: np.random.Generator) -> np.ndarray:
    """Seeded random wand points inside the small test volume."""
    low = np.array([0.1, 0.02, 0.02])
    high = np.array([0.8, 0.28, 0.28])
    return rng.uniform(low, high, size=(SAMPLE_COUNT, 3))


def _samples(
    points: np.ndarray, scale: float = 1.0, offset_m: float = -TRUE_LENS_OFFSET_M
) -> list[RobotSample]:
    """Samples as the true robot would report them for the given points."""
    start = TRUE_START_M
    out = []
    for p in points:
        dy, dz = p[1] - start[1], p[2] - start[2]
        out.append(
            RobotSample(
                point_m=(float(p[0]), float(p[1]), float(p[2])),
                rail_m=float(p[0] - start[0]),
                tilt_rad=math.atan2(dz, dy),
                focus_m=scale * math.hypot(dy, dz) + offset_m,
            )
        )
    return out


def test_noise_free_fit_recovers_truth() -> None:
    """Without noise the fit recovers the true robot exactly."""
    base = load_robot(ROBOT_PATH)
    samples = _samples(_points(np.random.default_rng(1)))
    fitted, scale = fit_robot(base, samples)
    assert np.allclose(fitted.rail_start_m, TRUE_START_M, atol=1e-6)
    assert fitted.lens_offset_m == pytest.approx(TRUE_LENS_OFFSET_M, abs=1e-6)
    assert scale == pytest.approx(1.0, abs=1e-6)
    expected = dataclasses.replace(
        base, rail_start_m=fitted.rail_start_m, lens_offset_m=fitted.lens_offset_m
    )
    assert fitted == expected


def test_noisy_points_stay_within_tolerance() -> None:
    """0.5 mm noise on the wand points keeps the fit within millimetres."""
    rng = np.random.default_rng(2)
    points = _points(rng)
    samples = _samples(points)
    noisy = points + rng.normal(0.0, 0.0005, size=points.shape)
    samples = [
        dataclasses.replace(s, point_m=(float(q[0]), float(q[1]), float(q[2])))
        for s, q in zip(samples, noisy)
    ]
    fitted, _ = fit_robot(load_robot(ROBOT_PATH), samples)
    assert np.allclose(fitted.rail_start_m, TRUE_START_M, atol=1e-3)
    assert fitted.lens_offset_m == pytest.approx(TRUE_LENS_OFFSET_M, abs=2e-3)


def test_focus_scale_is_returned() -> None:
    """A lens that over-reads distance by 5 percent gives scale 1.05."""
    samples = _samples(_points(np.random.default_rng(3)), scale=1.05, offset_m=-0.02)
    _, scale = fit_robot(load_robot(ROBOT_PATH), samples)
    assert scale == pytest.approx(1.05, abs=1e-6)


def test_too_few_samples_raises() -> None:
    """Fewer than MIN_SAMPLES samples is rejected."""
    samples = _samples(_points(np.random.default_rng(4)))[: MIN_SAMPLES - 1]
    with pytest.raises(ValueError, match="at least"):
        fit_robot(load_robot(ROBOT_PATH), samples)
