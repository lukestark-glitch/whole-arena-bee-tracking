"""Tests for the bee motion model: containment, speed limit, states and determinism."""

from dataclasses import replace

import numpy as np
import pytest

from shared.config import RANDOM_FLIGHT, TRIPS_FLIGHT, Arena, Bees, Trips
from simulator.bees import (
    FEEDING,
    FLYING,
    RESTING,
    SITTING,
    WALKING,
    Bee,
    create_bees,
    step_bees,
)

DT_S = 1.0 / 60.0
ARENA = Arena((0.9, 0.3, 0.3))
TRIPS = Trips(
    light_direction=(-1.0, 0.0, 0.0),
    light_strength_m_s2=0.3,
    feeder_m=(0.0, 0.15, 0.10),
    entrance_m=(0.9, 0.15, 0.10),
    lateral_spread_m=0.05,
    vertical_spread_m=0.04,
    min_cruise_speed_m_s=0.3,
    max_cruise_speed_m_s=0.8,
    velocity_jitter_m_s=0.15,
    feeding_time_s=4.0,
    rest_time_s=6.0,
)
SETTINGS = Bees(
    count=10,
    body_length_m=0.015,
    max_speed_m_s=5.0,
    flight_speed_m_s=1.0,
    flight_direction_time_s=0.5,
    walk_speed_m_s=0.03,
    walk_turn_rad_per_sqrt_s=1.0,
    landing_probability=0.2,
    takeoff_rate_hz=0.2,
    sit_to_walk_rate_hz=0.3,
    walk_to_sit_rate_hz=0.3,
    flight_model=RANDOM_FLIGHT,
    trips=TRIPS,
)
TRIP_SETTINGS = replace(SETTINGS, flight_model=TRIPS_FLIGHT)
SIZE_M = np.array(ARENA.size_m)


def _on_wall(bee: Bee) -> bool:
    """Return True if any coordinate of the bee is exactly on a wall."""
    return bool(np.any((bee.position_m == 0.0) | (bee.position_m == SIZE_M)))


def _run(settings: Bees, seed: int, steps: int) -> list[list[Bee]]:
    """Run the model and return a deep snapshot of the bees after each step."""
    rng = np.random.default_rng(seed)
    bees = create_bees(settings, ARENA, rng)
    snapshots = []
    for _ in range(steps):
        step_bees(bees, settings, ARENA, DT_S, rng)
        snapshots.append(
            [Bee(b.id, b.position_m.copy(), b.velocity_m_s.copy(), b.state) for b in bees]
        )
    return snapshots


def test_create_bees_count_ids_state_and_inside() -> None:
    """Created bees have ids 0..9, fly, and lie inside the box."""
    bees = create_bees(SETTINGS, ARENA, np.random.default_rng(1))
    assert len(bees) == 10
    assert [b.id for b in bees] == list(range(10))
    assert all(b.state == FLYING for b in bees)
    assert all(np.all(b.position_m >= 0.0) and np.all(b.position_m <= SIZE_M) for b in bees)


def test_long_run_invariants() -> None:
    """Over 60 s bees stay in the box, respect the speed limit and the state rules."""
    states = set()
    for snapshot in _run(SETTINGS, 2, 3600):
        for bee in snapshot:
            states.add(bee.state)
            assert np.all(bee.position_m >= 0.0) and np.all(bee.position_m <= SIZE_M)
            speed_m_s = np.linalg.norm(bee.velocity_m_s)
            assert speed_m_s <= SETTINGS.max_speed_m_s + 1e-9
            if bee.state != FLYING:
                assert _on_wall(bee)
            if bee.state == SITTING:
                assert speed_m_s == 0.0
            if bee.state == WALKING:
                assert speed_m_s <= SETTINGS.walk_speed_m_s + 1e-9
    assert states == {FLYING, SITTING, WALKING}


def test_speed_limit_with_low_maximum() -> None:
    """A low max_speed_m_s is never exceeded."""
    settings = replace(SETTINGS, max_speed_m_s=0.5)
    for snapshot in _run(settings, 3, 3600):
        for bee in snapshot:
            assert np.linalg.norm(bee.velocity_m_s) <= 0.5 + 1e-9


def test_same_seed_is_identical_and_other_seed_differs() -> None:
    """The same seed reproduces the run exactly; a different seed does not."""
    first = _run(SETTINGS, 4, 600)[-1]
    second = _run(SETTINGS, 4, 600)[-1]
    other = _run(SETTINGS, 5, 600)[-1]
    for a, b in zip(first, second):
        assert np.array_equal(a.position_m, b.position_m)
        assert np.array_equal(a.velocity_m_s, b.velocity_m_s)
        assert a.state == b.state
    assert any(not np.array_equal(a.position_m, c.position_m) for a, c in zip(first, other))


def test_flight_rms_speed_is_near_flight_speed() -> None:
    """The RMS speed of flying bees over 60 s is between 0.5 and 1.5 m/s."""
    speeds_sq = [
        float(np.sum(bee.velocity_m_s**2))
        for snapshot in _run(SETTINGS, 6, 3600)
        for bee in snapshot
        if bee.state == FLYING
    ]
    assert 0.5 < np.sqrt(np.mean(speeds_sq)) < 1.5


def test_walking_bee_velocity_matches_motion() -> None:
    """A walking bee moves by velocity times dt, except in a step where it turns back."""
    rng = np.random.default_rng(3)
    bees = create_bees(SETTINGS, ARENA, rng)
    checked = 0
    for _ in range(3600):
        before = {bee.id: bee.position_m.copy() for bee in bees if bee.state == WALKING}
        step_bees(bees, SETTINGS, ARENA, DT_S, rng)
        for bee in bees:
            if bee.state == WALKING and bee.id in before:
                moved_m = np.linalg.norm(bee.position_m - before[bee.id])
                assert moved_m <= SETTINGS.walk_speed_m_s * DT_S + 1e-12
                assert np.linalg.norm(bee.velocity_m_s) == pytest.approx(SETTINGS.walk_speed_m_s)
                checked += 1
    assert checked > 0


def test_random_flight_ignores_the_trip_settings() -> None:
    """With random flight the trip settings, light included, change nothing."""
    other = replace(SETTINGS, trips=replace(TRIPS, light_strength_m_s2=5.0, feeder_m=(0.3, 0.0, 0.1)))
    for first, second in zip(_run(SETTINGS, 4, 600), _run(other, 4, 600)):
        for a, b in zip(first, second):
            assert np.array_equal(a.position_m, b.position_m)


def _trip_phases(settings: Bees, seed: int, steps: int) -> tuple[list[list[str]], list[Bee]]:
    """Run the trips model; return each bee's phase sequence and a final snapshot."""
    rng = np.random.default_rng(seed)
    bees = create_bees(settings, ARENA, rng)
    phases: list[list[str]] = [[] for _ in bees]
    for _ in range(steps):
        step_bees(bees, settings, ARENA, DT_S, rng)
        for bee, seen in zip(bees, phases):
            assert np.all(bee.position_m >= 0.0) and np.all(bee.position_m <= SIZE_M)
            assert np.linalg.norm(bee.velocity_m_s) <= settings.max_speed_m_s + 1e-12
            if not seen or seen[-1] != bee.trip.phase:
                seen.append(bee.trip.phase)
    return phases, bees


def test_trips_start_resting_at_the_entrance() -> None:
    """Trip bees start sitting on the entrance wall."""
    bees = create_bees(TRIP_SETTINGS, ARENA, np.random.default_rng(1))
    assert all(b.state == SITTING and b.trip.phase == RESTING for b in bees)
    assert all(b.position_m[0] == ARENA.size_m[0] for b in bees)


def test_trips_visit_feeder_and_entrance_in_turn() -> None:
    """Over 120 s bees feed and rest in turn, inside the box and under the speed limit."""
    phases, _ = _trip_phases(TRIP_SETTINGS, 3, 7200)
    cycle = ["resting", "to_feeder", "feeding", "to_entrance"]
    for seen in phases:
        assert seen.count(FEEDING) >= 2
        assert all(seen[i + 1] == cycle[(cycle.index(seen[i]) + 1) % 4] for i in range(len(seen) - 1))


def test_trips_fly_mostly_along_x() -> None:
    """Flying trip bees move mainly along the arena's long axis."""
    rng = np.random.default_rng(5)
    bees = create_bees(TRIP_SETTINGS, ARENA, rng)
    along_m, across_m = 0.0, 0.0
    for _ in range(3600):
        step_bees(bees, TRIP_SETTINGS, ARENA, DT_S, rng)
        for bee in bees:
            if bee.state == FLYING:
                along_m += abs(bee.velocity_m_s[0])
                across_m += float(np.linalg.norm(bee.velocity_m_s[1:]))
    assert along_m > 2.0 * across_m


def test_trips_feed_at_a_moved_feeder() -> None:
    """A feeder on the floor is reached too: feeding bees sit on the floor."""
    settings = replace(TRIP_SETTINGS, trips=replace(TRIPS, feeder_m=(0.45, 0.15, 0.0)))
    rng = np.random.default_rng(6)
    bees = create_bees(settings, ARENA, rng)
    feeding_heights_m = []
    for _ in range(3600):
        step_bees(bees, settings, ARENA, DT_S, rng)
        feeding_heights_m += [b.position_m[2] for b in bees if b.trip.phase == FEEDING]
    assert feeding_heights_m and max(feeding_heights_m) == 0.0
