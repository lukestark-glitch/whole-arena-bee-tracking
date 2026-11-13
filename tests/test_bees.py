"""Tests for the bee motion model: containment, speed limit, states and determinism."""

from dataclasses import replace

import numpy as np

from shared.config import Arena, Bees
from simulator.bees import FLYING, SITTING, WALKING, Bee, create_bees, step_bees

DT_S = 1.0 / 60.0
ARENA = Arena((0.9, 0.3, 0.3))
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
)
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
