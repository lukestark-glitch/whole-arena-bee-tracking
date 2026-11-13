"""Tests for the axis motion model in `robot_emulator/motion.py`."""

from __future__ import annotations

from pathlib import Path

from robot_emulator.motion import Axis, step_axis
from shared.config import AxisLimits, load_robot

DT_S = 0.002
ROBOT_YAML = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"
LIMITS = AxisLimits(lower=0.0, upper=0.9, max_speed=2.0, max_acceleration=20.0)


def _run(
    limits: AxisLimits, start: float, target: float, duration_s: float
) -> tuple[Axis, float, float, float]:
    """Run a step move; return the axis, peak speed, peak acceleration and peak position."""
    axis = Axis(position=start, velocity=0.0)
    peak_v = peak_a = 0.0
    peak_pos = start
    for _ in range(round(duration_s / DT_S)):
        v_before = axis.velocity
        step_axis(axis, limits, target, 0.0, DT_S)
        peak_v = max(peak_v, abs(axis.velocity))
        peak_a = max(peak_a, abs(axis.velocity - v_before) / DT_S)
        peak_pos = max(peak_pos, axis.position)
    return axis, peak_v, peak_a, peak_pos


def test_limits_never_exceeded_during_step_move() -> None:
    """Speed and acceleration stay within limits during a large step."""
    _, peak_v, peak_a, _ = _run(LIMITS, 0.0, 0.9, 3.0)
    assert peak_v <= LIMITS.max_speed + 1e-9
    assert peak_a <= LIMITS.max_acceleration + 1e-6


def test_step_target_reached_and_settles() -> None:
    """The axis reaches a step target, stops, and does not overshoot."""
    axis, _, _, peak_pos = _run(LIMITS, 0.0, 0.6, 3.0)
    assert abs(axis.position - 0.6) < 1e-4
    assert abs(axis.velocity) < 1e-3
    assert peak_pos - 0.6 < 1e-3


def test_moving_target_followed_with_feed_forward() -> None:
    """A target moving at constant velocity is followed with almost no lag."""
    speed = 0.5
    axis = Axis(position=0.0, velocity=0.0)
    steps = round(1.5 / DT_S)
    for i in range(1, steps + 1):
        step_axis(axis, LIMITS, speed * i * DT_S, speed, DT_S)
    # One control step of lag is inherent: the axis moves after the target does.
    assert abs(speed * steps * DT_S - axis.position) < 1e-3 + 1e-9


def test_position_stays_within_travel() -> None:
    """A target outside the travel never drives the axis past its end stops."""
    for target in (5.0, -5.0):
        axis = Axis(position=0.45, velocity=0.0)
        for _ in range(round(3.0 / DT_S)):
            step_axis(axis, LIMITS, target, 0.0, DT_S)
            assert LIMITS.lower <= axis.position <= LIMITS.upper
        assert abs(axis.position - min(max(target, 0.0), 0.9)) < 1e-3


def test_edited_robot_yaml_limits_are_respected(tmp_path: Path) -> None:
    """Lower rail limits in robot.yaml directly lower the peak speed and acceleration."""
    text = ROBOT_YAML.read_text()
    edited = text.replace("max_speed_m_s: 2.0", "max_speed_m_s: 0.5", 1)
    edited = edited.replace("max_acceleration_m_s2: 20.0", "max_acceleration_m_s2: 3.0", 1)
    assert edited != text
    assert "max_speed_m_s: 0.5" in edited
    assert "max_acceleration_m_s2: 3.0" in edited
    path = tmp_path / "robot.yaml"
    path.write_text(edited)
    robot = load_robot(path)

    _, peak_v, peak_a, _ = _run(robot.rail, 0.0, 0.9, 6.0)
    assert peak_v <= 0.5 + 1e-9
    assert peak_v > 0.45
    assert peak_a <= 3.0 + 1e-6
