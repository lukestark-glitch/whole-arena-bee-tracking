"""Bee motion model of the simulator.

Moves simulated bees through the arena: smooth random flight, bouncing or landing at
the walls, sitting and walking on the walls, and take-off. The bees hold the ground
truth, so this module lives in ``simulator/`` and the tracker never imports it.

All quantities are in the arena frame (right-handed, x along the long side, y away from
the robot, z up), in metres, seconds and radians.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from shared.config import Arena, Bees

SITTING = "sitting"
WALKING = "walking"
FLYING = "flying"

# Per-axis variance factor of the Ornstein-Uhlenbeck flight: the 3D mean square speed is
# 3 * sigma^2 * tau / 2 per unit noise, so sigma^2 = 2 * v_rms^2 / (3 * tau).
_AXIS_COUNT = 3


@dataclass
class Bee:
    """State of one simulated bee.

    Attributes:
        id: Bee identifier, unique within a run.
        position_m: Position, shape (3,), arena frame, metres.
        velocity_m_s: Velocity, shape (3,), arena frame, metres per second.
        state: One of SITTING, WALKING or FLYING.
    """

    id: int
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    state: str


def _flight_sigma_m_s_per_sqrt_s(settings: Bees) -> float:
    """Return the noise strength of the flight velocity process.

    With sigma = v_rms * sqrt(2 / (3 * tau)) the long-run RMS speed equals v_rms.

    Args:
        settings: Bee settings.

    Returns:
        Noise strength sigma in m/s per sqrt(s).
    """
    tau_s = settings.flight_direction_time_s
    return settings.flight_speed_m_s * np.sqrt(2.0 / (_AXIS_COUNT * tau_s))


def create_bees(settings: Bees, arena: Arena, rng: np.random.Generator) -> list[Bee]:
    """Create flying bees at random positions with stationary flight velocities.

    Args:
        settings: Bee settings.
        arena: Arena whose box (arena frame, metres) holds the bees.
        rng: Seeded random generator.

    Returns:
        ``settings.count`` bees with ids 0..count-1, all FLYING.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    axis_std_m_s = settings.flight_speed_m_s / np.sqrt(_AXIS_COUNT)
    bees = []
    for bee_id in range(settings.count):
        position_m = rng.uniform(0.0, 1.0, _AXIS_COUNT) * size_m
        velocity_m_s = rng.normal(0.0, axis_std_m_s, _AXIS_COUNT)
        velocity_m_s = _limit_speed(velocity_m_s, settings.max_speed_m_s)
        bees.append(Bee(bee_id, position_m, velocity_m_s, FLYING))
    return bees


def step_bees(
    bees: list[Bee],
    settings: Bees,
    arena: Arena,
    dt_s: float,
    rng: np.random.Generator,
) -> None:
    """Advance every bee by one frame, in place, in list order.

    Args:
        bees: Bees to move.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        dt_s: Frame duration in seconds.
        rng: Seeded random generator.
    """
    for bee in bees:
        if bee.state == FLYING:
            _step_flying(bee, settings, arena, dt_s, rng)
        elif bee.state == SITTING:
            _step_sitting(bee, settings, arena, dt_s, rng)
        else:
            _step_walking(bee, settings, arena, dt_s, rng)


def _limit_speed(velocity_m_s: np.ndarray, max_speed_m_s: float) -> np.ndarray:
    """Scale a velocity down so its norm does not exceed the speed limit.

    Args:
        velocity_m_s: Velocity, shape (3,), metres per second.
        max_speed_m_s: Speed limit in metres per second.

    Returns:
        The velocity, scaled down if needed.
    """
    speed_m_s = float(np.linalg.norm(velocity_m_s))
    if speed_m_s > max_speed_m_s:
        return velocity_m_s * (max_speed_m_s / speed_m_s)
    return velocity_m_s


def _step_flying(
    bee: Bee, settings: Bees, arena: Arena, dt_s: float, rng: np.random.Generator
) -> None:
    """Advance a flying bee: smooth velocity change, move, then wall interaction.

    Args:
        bee: Flying bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        dt_s: Frame duration in seconds.
        rng: Seeded random generator.
    """
    tau_s = settings.flight_direction_time_s
    sigma = _flight_sigma_m_s_per_sqrt_s(settings)
    # Euler-Maruyama step of an Ornstein-Uhlenbeck process: relaxation to zero plus noise
    # that grows with sqrt(dt), so the statistics do not depend on the frame rate.
    noise = rng.normal(0.0, 1.0, _AXIS_COUNT)
    velocity_m_s = bee.velocity_m_s - bee.velocity_m_s * dt_s / tau_s
    velocity_m_s = velocity_m_s + sigma * np.sqrt(dt_s) * noise
    bee.velocity_m_s = _limit_speed(velocity_m_s, settings.max_speed_m_s)
    bee.position_m = bee.position_m + bee.velocity_m_s * dt_s
    _handle_walls(bee, settings, arena, rng)


def _handle_walls(
    bee: Bee, settings: Bees, arena: Arena, rng: np.random.Generator
) -> None:
    """Land or bounce a flying bee on every axis where it left the box.

    Args:
        bee: Flying bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        rng: Seeded random generator.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    for axis in range(_AXIS_COUNT):
        if 0.0 <= bee.position_m[axis] <= size_m[axis]:
            continue
        if rng.random() < settings.landing_probability:
            _land(bee, axis, size_m)
            return
        _bounce(bee, axis, size_m)
    bee.position_m = np.clip(bee.position_m, 0.0, size_m)


def _land(bee: Bee, axis: int, size_m: np.ndarray) -> None:
    """Put a bee onto the wall it hit and make it sit.

    Args:
        bee: Bee, modified in place.
        axis: Axis (0, 1, 2) of the wall that was hit.
        size_m: Box size, arena frame, metres.
    """
    bee.position_m = np.clip(bee.position_m, 0.0, size_m)
    bee.velocity_m_s = np.zeros(_AXIS_COUNT)
    bee.state = SITTING


def _bounce(bee: Bee, axis: int, size_m: np.ndarray) -> None:
    """Mirror a bee back inside the box on one axis and reverse that velocity.

    Args:
        bee: Bee, modified in place.
        axis: Axis (0, 1, 2) on which the bee left the box.
        size_m: Box size, arena frame, metres.
    """
    # Mirroring about the wall keeps the distance travelled past it, as a reflection would.
    if bee.position_m[axis] < 0.0:
        bee.position_m[axis] = -bee.position_m[axis]
    else:
        bee.position_m[axis] = 2.0 * size_m[axis] - bee.position_m[axis]
    bee.velocity_m_s[axis] = -bee.velocity_m_s[axis]


def _step_sitting(
    bee: Bee, settings: Bees, arena: Arena, dt_s: float, rng: np.random.Generator
) -> None:
    """Advance a sitting bee: it stays put, may take off or start to walk.

    Args:
        bee: Sitting bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        dt_s: Frame duration in seconds.
        rng: Seeded random generator.
    """
    draw = rng.random()
    if draw < settings.takeoff_rate_hz * dt_s:
        _take_off(bee, settings, arena)
    elif draw < (settings.takeoff_rate_hz + settings.sit_to_walk_rate_hz) * dt_s:
        bee.state = WALKING


def _wall_of(position_m: np.ndarray, size_m: np.ndarray) -> tuple[int, int]:
    """Find the wall a bee is on.

    Args:
        position_m: Position, shape (3,), arena frame, metres.
        size_m: Box size, arena frame, metres.

    Returns:
        Tuple (axis, side) with side 0 for the wall at 0 and 1 for the wall at the box size.

    Raises:
        ValueError: If the position is on no wall.
    """
    for axis in range(_AXIS_COUNT):
        if position_m[axis] == 0.0:
            return axis, 0
        if position_m[axis] == size_m[axis]:
            return axis, 1
    raise ValueError(f"Position {position_m} is not on a wall")


def _wall_normal(axis: int, side: int) -> np.ndarray:
    """Return the unit normal of a wall pointing into the arena.

    Args:
        axis: Axis (0, 1, 2) perpendicular to the wall.
        side: 0 for the wall at coordinate 0, 1 for the wall at the box size.

    Returns:
        Unit vector, shape (3,), arena frame.
    """
    normal = np.zeros(_AXIS_COUNT)
    normal[axis] = 1.0 if side == 0 else -1.0
    return normal


def _take_off(bee: Bee, settings: Bees, arena: Arena) -> None:
    """Make a bee fly away from its wall at the typical flight speed.

    Args:
        bee: Sitting or walking bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
    """
    axis, side = _wall_of(bee.position_m, np.asarray(arena.size_m, dtype=float))
    takeoff_speed_m_s = min(settings.flight_speed_m_s, settings.max_speed_m_s)
    bee.velocity_m_s = _wall_normal(axis, side) * takeoff_speed_m_s
    bee.state = FLYING


def _step_walking(
    bee: Bee, settings: Bees, arena: Arena, dt_s: float, rng: np.random.Generator
) -> None:
    """Advance a walking bee: it may take off, sit down, or walk along its wall.

    Args:
        bee: Walking bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        dt_s: Frame duration in seconds.
        rng: Seeded random generator.
    """
    draw = rng.random()
    if draw < settings.takeoff_rate_hz * dt_s:
        _take_off(bee, settings, arena)
    elif draw < (settings.takeoff_rate_hz + settings.walk_to_sit_rate_hz) * dt_s:
        bee.velocity_m_s = np.zeros(_AXIS_COUNT)
        bee.state = SITTING
    else:
        _walk(bee, settings, arena, dt_s, rng)


def _walk(
    bee: Bee, settings: Bees, arena: Arena, dt_s: float, rng: np.random.Generator
) -> None:
    """Turn the heading randomly and move a bee within its wall plane.

    Args:
        bee: Walking bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        dt_s: Frame duration in seconds.
        rng: Seeded random generator.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    axis, _ = _wall_of(bee.position_m, size_m)
    in_plane = [a for a in range(_AXIS_COUNT) if a != axis]
    heading_rad = _heading_rad(bee.velocity_m_s, in_plane, rng)
    # A random walk in heading: the std of the turn grows with sqrt(time).
    heading_rad += rng.normal() * settings.walk_turn_rad_per_sqrt_s * np.sqrt(dt_s)
    velocity_m_s = np.zeros(_AXIS_COUNT)
    velocity_m_s[in_plane[0]] = settings.walk_speed_m_s * np.cos(heading_rad)
    velocity_m_s[in_plane[1]] = settings.walk_speed_m_s * np.sin(heading_rad)
    wall_coordinate_m = bee.position_m[axis]
    position_m = np.clip(bee.position_m + velocity_m_s * dt_s, 0.0, size_m)
    position_m[axis] = wall_coordinate_m
    bee.position_m = position_m
    bee.velocity_m_s = velocity_m_s


def _heading_rad(
    velocity_m_s: np.ndarray, in_plane: list[int], rng: np.random.Generator
) -> float:
    """Return the walking heading in the wall plane.

    Args:
        velocity_m_s: Current velocity, shape (3,), arena frame, metres per second.
        in_plane: The two axes spanning the wall plane.
        rng: Seeded random generator, used only when the bee has no velocity yet.

    Returns:
        Angle in radians from the first in-plane axis towards the second.
    """
    first_m_s = velocity_m_s[in_plane[0]]
    second_m_s = velocity_m_s[in_plane[1]]
    if first_m_s == 0.0 and second_m_s == 0.0:
        return float(rng.uniform(0.0, 2.0 * np.pi))
    return float(np.arctan2(second_m_s, first_m_s))
