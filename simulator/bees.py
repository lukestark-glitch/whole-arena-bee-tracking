"""Bee motion model of the simulator.

Moves simulated bees through the arena: smooth random flight, bouncing or landing at
the walls, sitting and walking on the walls, and take-off. With the ``trips`` flight model
each bee instead flies between the colony entrance and the feeder, drawn by the light
(D87 to D89). The bees hold the ground truth, so this module lives in ``simulator/`` and
the tracker never imports it.

All quantities are in the arena frame (right-handed, x along the long side, y away from
the robot, z up), in metres, seconds and radians.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from shared.config import TRIPS_FLIGHT, Arena, Bees, Trips

SITTING = "sitting"
WALKING = "walking"
FLYING = "flying"

# Phases of a trip in the trips flight model (D88).
TO_FEEDER = "to_feeder"
FEEDING = "feeding"
TO_ENTRANCE = "to_entrance"
RESTING = "resting"

# Number of axes of the arena frame (x, y, z).
_AXIS_COUNT = 3
# Index of the vertical axis; the aim point spreads vertically along it.
_Z_AXIS = 2


@dataclass
class Trip:
    """Where a bee of the trips flight model is in its cycle.

    Attributes:
        phase: TO_FEEDER, FEEDING, TO_ENTRANCE or RESTING.
        time_left_s: Time left at the feeder or entrance, in s; unused while flying.
        aim_offset_m: Offset of the aim point from the goal, shape (3,), arena frame,
            metres. Its part along the goal wall's normal is ignored.
        speed_m_s: Cruising speed of the current flight, in m/s.
    """

    phase: str
    time_left_s: float
    aim_offset_m: np.ndarray
    speed_m_s: float


@dataclass
class Bee:
    """State of one simulated bee.

    Attributes:
        id: Bee identifier, unique within a run.
        position_m: Position, shape (3,), arena frame, metres.
        velocity_m_s: Velocity, shape (3,), arena frame, metres per second.
        state: One of SITTING, WALKING or FLYING.
        trip: Trip state with the trips flight model, None with random flight.
    """

    id: int
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    state: str
    trip: Trip | None = None


def _flight_sigma_m_s_per_sqrt_s(rms_speed_m_s: float, tau_s: float) -> float:
    """Return the noise strength of the flight velocity process.

    Each axis of an Ornstein-Uhlenbeck process settles at a variance of
    sigma^2 * tau / 2, so the mean square speed over three axes is 3 * sigma^2 * tau / 2.
    With sigma = v_rms * sqrt(2 / (3 * tau)) the long-run RMS speed equals v_rms.

    Args:
        rms_speed_m_s: Long-run RMS deviation from the mean velocity, in m/s.
        tau_s: Relaxation time of the velocity, in s.

    Returns:
        Noise strength sigma in m/s per sqrt(s).
    """
    return rms_speed_m_s * np.sqrt(2.0 / (_AXIS_COUNT * tau_s))


def create_bee(bee_id: int, settings: Bees, arena: Arena, rng: np.random.Generator) -> Bee:
    """Create one flying bee at a random position with a stationary flight velocity.

    Args:
        bee_id: Identifier of the new bee.
        settings: Bee settings.
        arena: Arena whose box (arena frame, metres) holds the bee.
        rng: Seeded random generator.

    Returns:
        A FLYING bee; position in the arena frame (m), velocity in m/s. With the trips
        flight model a bee resting at the entrance instead.
    """
    if settings.flight_model == TRIPS_FLIGHT:
        return _resting_bee(bee_id, settings.trips, arena, rng)
    size_m = np.asarray(arena.size_m, dtype=float)
    axis_std_m_s = settings.flight_speed_m_s / np.sqrt(_AXIS_COUNT)
    position_m = rng.uniform(0.0, 1.0, _AXIS_COUNT) * size_m
    velocity_m_s = rng.normal(0.0, axis_std_m_s, _AXIS_COUNT)
    velocity_m_s = _limit_speed(velocity_m_s, settings.max_speed_m_s)
    return Bee(bee_id, position_m, velocity_m_s, FLYING)


def create_bees(settings: Bees, arena: Arena, rng: np.random.Generator) -> list[Bee]:
    """Create flying bees at random positions with stationary flight velocities.

    Args:
        settings: Bee settings.
        arena: Arena whose box (arena frame, metres) holds the bees.
        rng: Seeded random generator.

    Returns:
        ``settings.count`` bees with ids 0..count-1, all FLYING (all resting at the
        entrance with the trips flight model).
    """
    return [create_bee(bee_id, settings, arena, rng) for bee_id in range(settings.count)]


def _resting_bee(bee_id: int, trips: Trips, arena: Arena, rng: np.random.Generator) -> Bee:
    """Create a bee of the trips model that sits near the entrance.

    Args:
        bee_id: Identifier of the new bee.
        trips: Trip settings.
        arena: Arena box, arena frame, metres.
        rng: Seeded random generator.

    Returns:
        A SITTING bee on the entrance wall in phase RESTING.
    """
    # A random share of the mean rest time staggers the first departures.
    trip = Trip(RESTING, rng.exponential(trips.rest_time_s), _aim_offset(trips, rng), 0.0)
    position_m = _aim_point(trips.entrance_m, trip.aim_offset_m, arena)
    return Bee(bee_id, position_m, np.zeros(_AXIS_COUNT), SITTING, trip)


def _aim_offset(trips: Trips, rng: np.random.Generator) -> np.ndarray:
    """Draw the offset of a flight's aim point from its goal.

    Args:
        trips: Trip settings.
        rng: Seeded random generator.

    Returns:
        Offset, shape (3,), arena frame, metres: lateral spread on x and y, vertical on z.
    """
    spreads_m = np.array([trips.lateral_spread_m] * _AXIS_COUNT)
    spreads_m[_Z_AXIS] = trips.vertical_spread_m
    return rng.normal(0.0, 1.0, _AXIS_COUNT) * spreads_m


def _aim_point(goal_m: tuple[float, float, float], offset_m: np.ndarray, arena: Arena) -> np.ndarray:
    """Return the point on the goal's wall that a bee aims at.

    Args:
        goal_m: Feeder or entrance, on a wall, arena frame, metres.
        offset_m: Offset from the goal, shape (3,), metres.
        arena: Arena box, arena frame, metres.

    Returns:
        Point on the goal's wall inside the box, shape (3,), arena frame, metres.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    goal = np.asarray(goal_m, dtype=float)
    axis, _ = _wall_of(goal, size_m)
    # The offset slides along the wall only, so the aim point stays on the wall even after
    # the feeder was moved to another wall.
    in_wall_m = offset_m.copy()
    in_wall_m[axis] = 0.0
    return np.clip(goal + in_wall_m, 0.0, size_m)


def _goal_m(trip: Trip, trips: Trips) -> tuple[float, float, float]:
    """Return the goal of a trip phase: the feeder while flying to it or feeding.

    Args:
        trip: Trip state.
        trips: Trip settings.

    Returns:
        Feeder or entrance, arena frame, metres.
    """
    return trips.feeder_m if trip.phase in (TO_FEEDER, FEEDING) else trips.entrance_m


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
    desired_m_s, pull_m_s2, rms_m_s = _flight_drive(bee, settings, arena)
    sigma = _flight_sigma_m_s_per_sqrt_s(rms_m_s, tau_s)
    # Euler-Maruyama step of an Ornstein-Uhlenbeck process: relaxation to the desired
    # velocity plus noise that grows with sqrt(dt), so the statistics do not depend on the
    # frame rate. With random flight the desired velocity and the pull are zero (D89).
    noise = rng.normal(0.0, 1.0, _AXIS_COUNT)
    velocity_m_s = bee.velocity_m_s - (bee.velocity_m_s - desired_m_s) * dt_s / tau_s
    velocity_m_s = velocity_m_s + pull_m_s2 * dt_s + sigma * np.sqrt(dt_s) * noise
    bee.velocity_m_s = _limit_speed(velocity_m_s, settings.max_speed_m_s)
    bee.position_m = bee.position_m + bee.velocity_m_s * dt_s
    _handle_walls(bee, settings, arena, rng)


def _flight_drive(
    bee: Bee, settings: Bees, arena: Arena
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return what drives a flying bee: desired velocity, light pull and velocity noise.

    Args:
        bee: Flying bee.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.

    Returns:
        Desired velocity (m/s, shape (3,)), constant pull (m/s^2, shape (3,)) and the RMS
        deviation from the desired velocity (m/s). Random flight: zero, zero and the
        typical flight speed.
    """
    if bee.trip is None:
        return np.zeros(_AXIS_COUNT), np.zeros(_AXIS_COUNT), settings.flight_speed_m_s
    trips = settings.trips
    aim_m = _aim_point(_goal_m(bee.trip, trips), bee.trip.aim_offset_m, arena)
    to_aim_m = aim_m - bee.position_m
    distance_m = float(np.linalg.norm(to_aim_m))
    desired_m_s = np.zeros(_AXIS_COUNT)
    if distance_m > 0.0:
        desired_m_s = to_aim_m / distance_m * bee.trip.speed_m_s
    pull_m_s2 = np.asarray(trips.light_direction) * trips.light_strength_m_s2
    return desired_m_s, pull_m_s2, trips.velocity_jitter_m_s


def _handle_walls(
    bee: Bee, settings: Bees, arena: Arena, rng: np.random.Generator
) -> None:
    """Land or bounce a flying bee on every axis where it left the box.

    With the trips model the bee lands only on its goal's wall and bounces off the others.

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
        if bee.trip is not None:
            if _is_goal_wall(bee, axis, settings.trips, size_m):
                _arrive(bee, axis, settings.trips, size_m, rng)
                return
            _bounce(bee, axis, size_m)
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


def _is_goal_wall(bee: Bee, axis: int, trips: Trips, size_m: np.ndarray) -> bool:
    """Tell whether a bee that left the box on one axis has reached its goal's wall.

    Args:
        bee: Flying bee of the trips model, just outside the box on ``axis``.
        axis: Axis (0, 1, 2) on which the bee left the box.
        trips: Trip settings.
        size_m: Box size, arena frame, metres.

    Returns:
        True if the crossed wall holds the goal of the current flight.
    """
    goal_axis, goal_side = _wall_of(np.asarray(_goal_m(bee.trip, trips), dtype=float), size_m)
    side = 0 if bee.position_m[axis] < 0.0 else 1
    return (axis, side) == (goal_axis, goal_side)


def _arrive(
    bee: Bee, axis: int, trips: Trips, size_m: np.ndarray, rng: np.random.Generator
) -> None:
    """Land a bee of the trips model on its goal's wall and start feeding or resting.

    Args:
        bee: Flying bee of the trips model, modified in place.
        axis: Axis (0, 1, 2) of the goal's wall.
        trips: Trip settings.
        size_m: Box size, arena frame, metres.
        rng: Seeded random generator.
    """
    _land(bee, axis, size_m)
    if bee.trip.phase == TO_FEEDER:
        bee.trip.phase, mean_s = FEEDING, trips.feeding_time_s
    else:
        bee.trip.phase, mean_s = RESTING, trips.rest_time_s
    bee.trip.time_left_s = rng.exponential(mean_s)


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
    rate_hz = _takeoff_rate_hz(bee, settings)
    if _leaves_wall(bee, draw, rate_hz, dt_s):
        _take_off(bee, settings, arena, rng)
    elif draw < (rate_hz + settings.sit_to_walk_rate_hz) * dt_s:
        bee.state = WALKING


def _takeoff_rate_hz(bee: Bee, settings: Bees) -> float:
    """Return the random take-off rate: zero with the trips model, whose timer decides.

    Args:
        bee: Sitting or walking bee.
        settings: Bee settings.

    Returns:
        Take-off rate in 1/s.
    """
    return settings.takeoff_rate_hz if bee.trip is None else 0.0


def _leaves_wall(bee: Bee, draw: float, rate_hz: float, dt_s: float) -> bool:
    """Decide whether a sitting or walking bee takes off in this frame.

    Args:
        bee: Sitting or walking bee; the trip timer counts down, modified in place.
        draw: Uniform random number from [0, 1) drawn for this frame.
        rate_hz: Random take-off rate, in 1/s.
        dt_s: Frame duration in seconds.

    Returns:
        True when the bee takes off: at random with random flight, when the time at the
        feeder or entrance is over with the trips model.
    """
    if bee.trip is None:
        return draw < rate_hz * dt_s
    bee.trip.time_left_s -= dt_s
    return bee.trip.time_left_s <= 0.0


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


def _take_off(bee: Bee, settings: Bees, arena: Arena, rng: np.random.Generator) -> None:
    """Make a bee fly away from its wall.

    With random flight it leaves at the typical flight speed. With the trips model it
    starts the next flight, towards the other goal, at a cruising speed drawn for the trip.

    Args:
        bee: Sitting or walking bee, modified in place.
        settings: Bee settings.
        arena: Arena box, arena frame, metres.
        rng: Seeded random generator, used only by the trips model.
    """
    axis, side = _wall_of(bee.position_m, np.asarray(arena.size_m, dtype=float))
    speed_m_s = settings.flight_speed_m_s
    if bee.trip is not None:
        trips = settings.trips
        bee.trip.phase = TO_ENTRANCE if bee.trip.phase == FEEDING else TO_FEEDER
        bee.trip.aim_offset_m = _aim_offset(trips, rng)
        bee.trip.speed_m_s = rng.uniform(trips.min_cruise_speed_m_s, trips.max_cruise_speed_m_s)
        speed_m_s = bee.trip.speed_m_s
    bee.velocity_m_s = _wall_normal(axis, side) * min(speed_m_s, settings.max_speed_m_s)
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
    rate_hz = _takeoff_rate_hz(bee, settings)
    if _leaves_wall(bee, draw, rate_hz, dt_s):
        _take_off(bee, settings, arena, rng)
    elif draw < (rate_hz + settings.walk_to_sit_rate_hz) * dt_s:
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
    bee.position_m = bee.position_m + velocity_m_s * dt_s
    bee.velocity_m_s = velocity_m_s
    for in_plane_axis in in_plane:
        _turn_back_at_edge(bee, in_plane_axis, size_m)


def _turn_back_at_edge(bee: Bee, axis: int, size_m: np.ndarray) -> None:
    """Mirror a walking bee back onto its wall where it crossed the wall's edge.

    Turning back keeps the reported velocity true to the motion; a bee clamped at the
    edge would stand still while still reporting its walking speed.

    Args:
        bee: Walking bee, modified in place.
        axis: In-plane axis to check.
        size_m: Arena size, shape (3,), arena frame, metres.
    """
    coordinate_m = bee.position_m[axis]
    if 0.0 <= coordinate_m <= size_m[axis]:
        return
    edge_m = 0.0 if coordinate_m < 0.0 else size_m[axis]
    bee.position_m[axis] = 2.0 * edge_m - coordinate_m
    bee.velocity_m_s[axis] = -bee.velocity_m_s[axis]


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
