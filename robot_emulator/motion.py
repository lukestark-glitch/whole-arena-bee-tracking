"""Motion of one robot axis with speed and acceleration limits.

The rail, tilt and focus of the robot emulator are all modelled as the same
kind of axis: a position and a velocity that follow a target without ever
exceeding the configured maximum speed or acceleration. Used by
`robot_emulator/controller.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from shared.config import AxisLimits


@dataclass
class Axis:
    """State of one axis.

    Attributes:
        position: Position in m (rail, focus) or rad (tilt).
        velocity: Velocity in m/s (rail, focus) or rad/s (tilt).
    """

    position: float
    velocity: float


def _approach_speed(error: float, max_acceleration: float, dt_s: float) -> float:
    """Return the speed at which to approach a target.

    Args:
        error: Remaining distance to the target, absolute value, in m or rad.
        max_acceleration: Maximum deceleration in m/s^2 or rad/s^2.
        dt_s: Control step in s.

    Returns:
        Approach speed in m/s or rad/s.
    """
    # Braking curve of a drive that slows by a*dt per step: stopping from
    # v = n*a*dt takes d = a*dt^2 * n(n+1)/2, solved here for n. It tends to
    # v = sqrt(2 a d) for small steps, but does not overshoot at finite steps.
    step_dv = max_acceleration * dt_s
    steps_to_stop = (math.sqrt(1.0 + 8.0 * error / (step_dv * dt_s)) - 1.0) / 2.0
    # Whole steps only: a fractional step would leave speed at the target.
    braking_speed = math.floor(steps_to_stop) * step_dv
    # Never ask for more than the distance covered in one step, so no overshoot.
    return min(braking_speed, error / dt_s)


def _clip(value: float, limit: float) -> float:
    """Clip a value to the symmetric range [-limit, +limit]."""
    return max(-limit, min(limit, value))


def step_axis(
    axis: Axis,
    limits: AxisLimits,
    target_position: float,
    target_velocity: float,
    dt_s: float,
) -> None:
    """Advance an axis in place by one control step.

    Args:
        axis: Axis state, modified in place (m, m/s or rad, rad/s).
        limits: Travel, speed and acceleration limits of this axis.
        target_position: Wanted position at the end of the step in m or rad; clipped to
            the travel.
        target_velocity: Velocity of the target in m/s or rad/s, used as
            feed-forward so a moving target is followed without lag.
        dt_s: Control step in s, greater than zero.
    """
    target = max(limits.lower, min(limits.upper, target_position))
    # A target beyond the travel does not move for the axis: no feed-forward into the stop.
    if target != target_position:
        target_velocity = 0.0
    # The target position is for the end of the step. The feed-forward alone already
    # covers target_velocity * dt_s of the way, so only the rest is error.
    error = target - (axis.position + target_velocity * dt_s)
    direction = math.copysign(1.0, error)
    approach = _approach_speed(abs(error), limits.max_acceleration, dt_s)
    desired = _clip(target_velocity + direction * approach, limits.max_speed)

    # Limit the change of velocity per step to what the drive can deliver.
    acceleration = _clip((desired - axis.velocity) / dt_s, limits.max_acceleration)
    axis.velocity += acceleration * dt_s
    axis.position += axis.velocity * dt_s

    # End stop: the axis cannot move past its travel and loses its speed.
    if axis.position < limits.lower or axis.position > limits.upper:
        axis.position = max(limits.lower, min(limits.upper, axis.position))
        axis.velocity = 0.0
