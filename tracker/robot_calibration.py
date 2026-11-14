"""Robot calibration of the tracker.

Fits the robot's rail start and the mapping from distance to focus from wand
positions and the robot's own axis readings. All functions are pure. Used by
the calibration command in ``tracker/calibrate.py``. Units are metres and
radians; positions are in the arena frame. This module must not import from
``simulator/`` or ``robot_emulator/`` and never reads the wall clock.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from shared.config import Robot

# docs/robot-interface.md asks for 10 to 20 places; fewer cannot pin down
# the three rail-start unknowns plus the two focus-line parameters reliably.
MIN_SAMPLES = 10


@dataclass(frozen=True)
class RobotSample:
    """One moment in which the robot was aimed and focused at a known point.

    Attributes:
        point_m: Wand position in the arena frame, metres.
        rail_m: Rail carriage position, metres along +x.
        tilt_rad: Tilt angle, radians (0 = along +y, positive = up).
        focus_m: Focus distance reported by the lens, metres.
    """

    point_m: tuple[float, float, float]
    rail_m: float
    tilt_rad: float
    focus_m: float


def _range_and_angle(
    start_m: np.ndarray, point_m: tuple[float, float, float]
) -> tuple[float, float]:
    """Distance and tilt angle from the tilt axis to a point.

    Args:
        start_m: Tilt axis position at rail position 0, arena frame, metres.
        point_m: Target point, arena frame, metres.

    Returns:
        (range_m, tilt_rad) in the y-z plane of the tilt axis.
    """
    dy_m = point_m[1] - start_m[1]
    dz_m = point_m[2] - start_m[2]
    return math.hypot(dy_m, dz_m), math.atan2(dz_m, dy_m)


def _residuals(start_m: np.ndarray, samples: list[RobotSample]) -> np.ndarray:
    """Rail and tilt residuals of all samples for a candidate rail start.

    Args:
        start_m: Candidate rail start, arena frame, metres.
        samples: Calibration samples.

    Returns:
        Array of 2 residuals per sample, metres. The tilt error is multiplied
        by the range so that an angle error becomes a distance at the point
        and both residual kinds share one unit.
    """
    out: list[float] = []
    for s in samples:
        range_m, tilt_rad = _range_and_angle(start_m, s.point_m)
        out.append(s.point_m[0] - start_m[0] - s.rail_m)
        out.append((tilt_rad - s.tilt_rad) * range_m)
    return np.array(out)


def _fit_focus_line(
    start_m: np.ndarray, samples: list[RobotSample]
) -> tuple[float, float]:
    """Fit the straight line focus = offset + scale * range.

    Args:
        start_m: Fitted rail start, arena frame, metres.
        samples: Calibration samples.

    Returns:
        (offset_m, scale), the intercept in metres and the dimensionless slope.
    """
    ranges_m = [_range_and_angle(start_m, s.point_m)[0] for s in samples]
    focus_m = [s.focus_m for s in samples]
    scale, offset_m = np.polyfit(ranges_m, focus_m, 1)
    return float(offset_m), float(scale)


def fit_robot(robot: Robot, samples: list[RobotSample]) -> tuple[Robot, float]:
    """Fit the robot's rail start, lens offset and focus scale.

    Args:
        robot: Robot whose ``rail_start_m`` is the starting guess; all other
            fields are kept unchanged.
        samples: At least ``MIN_SAMPLES`` calibration samples.

    Returns:
        (robot with fitted ``rail_start_m`` and ``lens_offset_m``, focus scale).
        The scale is 1.0 for a lens that reports true distances.

    Raises:
        ValueError: If fewer than ``MIN_SAMPLES`` samples are given.
    """
    if len(samples) < MIN_SAMPLES:
        raise ValueError(
            f"Robot calibration needs at least {MIN_SAMPLES} samples, "
            f"got {len(samples)}."
        )
    result = least_squares(
        _residuals, np.array(robot.rail_start_m, dtype=float), args=(samples,)
    )
    offset_m, scale = _fit_focus_line(result.x, samples)
    fitted = dataclasses.replace(
        robot,
        rail_start_m=(float(result.x[0]), float(result.x[1]), float(result.x[2])),
        # focus = scale * (range - lens offset), so the line's intercept is
        # minus scale times the lens offset.
        lens_offset_m=-offset_m / scale,
    )
    return fitted, scale
