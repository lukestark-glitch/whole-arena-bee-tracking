"""Tests for tracker.tracks: filter accuracy, assignment, probation, removal and motion."""

import numpy as np

from shared.config import TrackerSettings
from tracker.tracks import (
    FLYING,
    NEW_POINT,
    SITTING,
    Track,
    assign,
    create_track_set,
    extrapolate,
    predict_tracks,
    update_tracks,
)

SETTINGS = TrackerSettings(
    frame_timeout_frames=2,
    claim_gate_m=0.02,
    ray_tolerance_m=0.004,
    min_cameras=3,
    min_cameras_tracked=2,
    coverage_share=0.5,
    arena_margin_m=0.01,
    assignment_gate_m=0.05,
    confirm_frames=5,
    max_missed_frames=15,
    jerk_sigma_m_s3=300.0,
    measurement_sigma_m=0.002,
    initial_speed_sigma_m_s=2.0,
    initial_acceleration_sigma_m_s2=20.0,
    flying_above_m_s=0.15,
    sitting_below_m_s=0.05,
)
DT_S = 1.0 / 60.0


def step(track_set, frame_index, points, ids=None, counts=None):
    """Run one frame: predict to its time, then update with the given points."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    n = len(points)
    ids = np.full(n, NEW_POINT) if ids is None else np.asarray(ids, dtype=int)
    counts = np.full(n, 3) if counts is None else np.asarray(counts, dtype=int)
    t_s = frame_index * DT_S
    predict_tracks(track_set, t_s, SETTINGS)
    update_tracks(track_set, points, counts, ids, t_s, SETTINGS)


def run_line(velocity_m_s, frames, noise_sigma_m=0.0, rng=None):
    """Follow one point moving at constant velocity from the origin; return the track set."""
    track_set = create_track_set()
    velocity = np.asarray(velocity_m_s)
    for k in range(frames):
        point = velocity * k * DT_S
        if noise_sigma_m > 0.0:
            point = point + rng.normal(0.0, noise_sigma_m, 3)
        step(track_set, k, [point])
    return track_set


def test_exact_straight_path_gives_velocity_and_zero_acceleration():
    """Noise-free constant velocity is recovered closely after 2 s."""
    velocity = np.array([0.8, -0.3, 0.1])
    track = run_line(velocity, 120).tracks[0]
    assert np.allclose(track.state[3:6], velocity, atol=0.01)
    assert np.allclose(track.state[6:9], 0.0, atol=0.5)


def test_noisy_straight_path_velocity_within_tolerance():
    """With 1 mm position noise the velocity error stays below 0.1 m/s."""
    velocity = np.array([0.8, -0.3, 0.1])
    rng = np.random.default_rng(7)
    track = run_line(velocity, 120, 0.001, rng).tracks[0]
    assert np.allclose(track.state[3:6], velocity, atol=0.1)


def test_crossing_paths_keep_their_ids():
    """Two bees passing 8 cm apart keep their ids although point order changes."""
    track_set = create_track_set()
    for k in range(61):
        t_s = k * DT_S
        a = [-0.5 + t_s, 0.0, 1.0]
        b = [0.5 - t_s, 0.08, 1.0]
        step(track_set, k, [a, b] if k % 2 == 0 else [b, a])
    assert [t.id for t in track_set.tracks] == [0, 1]
    assert abs(track_set.tracks[0].state[1] - 0.0) < 0.01
    assert abs(track_set.tracks[1].state[1] - 0.08) < 0.01


def test_assign_is_globally_optimal():
    """A case where greedy nearest-first would leave a track unmatched."""
    tracks = np.array([[0.0, 0, 0], [0.04, 0, 0]])
    points = np.array([[0.03, 0, 0], [0.07, 0, 0]])
    assert sorted(assign(tracks, points, 0.05)) == [(0, 0), (1, 1)]


def test_assign_drops_pairs_beyond_gate():
    """A pair farther apart than the gate is not returned."""
    assert assign(np.array([[0.0, 0, 0]]), np.array([[0.2, 0, 0]]), 0.05) == []


def test_assign_handles_empty_inputs():
    """No tracks or no points give no pairs."""
    assert assign(np.zeros((0, 3)), np.zeros((2, 3)), 0.05) == []
    assert assign(np.zeros((2, 3)), np.zeros((0, 3)), 0.05) == []


def test_probation_needs_five_consecutive_frames():
    """A point seen in 4 frames is unconfirmed, in the 5th it is confirmed."""
    track_set = create_track_set()
    for k in range(4):
        step(track_set, k, [[0.5, 0.5, 0.5]])
    assert not track_set.tracks[0].confirmed
    step(track_set, 4, [[0.5, 0.5, 0.5]])
    assert track_set.tracks[0].confirmed


def test_point_seen_once_leaves_no_track():
    """An unconfirmed track is removed at the first frame without support."""
    track_set = create_track_set()
    step(track_set, 0, [[0.5, 0.5, 0.5]])
    step(track_set, 1, [])
    assert track_set.tracks == []


def test_confirmed_track_survives_fifteen_misses_and_dies_at_sixteenth():
    """A confirmed track is removed when missed exceeds max_missed_frames."""
    track_set = run_line(np.zeros(3), 5)
    for k in range(5, 20):
        step(track_set, k, [])
    assert len(track_set.tracks) == 1
    step(track_set, 20, [])
    assert track_set.tracks == []


def test_gap_follows_prediction():
    """During a gap the constant-velocity path continues within 1 cm."""
    velocity = np.array([1.0, 0.0, 0.0])
    track_set = run_line(velocity, 60)
    for k in range(60, 65):
        step(track_set, k, [])
    track = track_set.tracks[0]
    assert np.linalg.norm(track.state[:3] - velocity * 64 * DT_S) < 0.01


def test_labelled_point_updates_its_track_even_if_another_is_nearer():
    """A point carrying a track id goes to that track, not to the nearest one."""
    track_set = create_track_set()
    step(track_set, 0, [[0.0, 0, 0], [0.1, 0, 0]])
    step(track_set, 1, [[0.1, 0, 0], [0.0, 0, 0]], ids=[0, NEW_POINT], counts=[4, 2])
    track = track_set.tracks[0]
    assert track.id == 0 and track.hits == 2 and track.camera_count == 4
    assert track.state[0] > 0.03
    # Track 1 got no point and was still on probation, so it is gone; the unlabelled point is new.
    assert [t.id for t in track_set.tracks] == [0, 2]


def test_ids_are_never_reused():
    """After a track is removed, a new one gets a fresh id."""
    track_set = create_track_set()
    step(track_set, 0, [[0.5, 0.5, 0.5]])
    step(track_set, 1, [])
    step(track_set, 2, [[0.5, 0.5, 0.5]])
    assert [t.id for t in track_set.tracks] == [1]


def _track_with_velocity(speed_m_s, motion):
    """Make a track whose state has the given speed along x and the given motion."""
    track_set = create_track_set()
    step(track_set, 0, [[0.0, 0.0, 0.0]])
    track = track_set.tracks[0]
    track.state[3] = speed_m_s
    track.motion = motion
    return track_set


def _motion_after(track_set, speed_m_s):
    """Set the speed, then update with a point exactly at the state so velocity is unchanged."""
    track = track_set.tracks[0]
    track.state[3] = speed_m_s
    update_tracks(track_set, track.state[:3].reshape(1, 3), np.array([3]), np.array([track.id]), track.t_s, SETTINGS)
    return track_set.tracks[0].motion


def test_motion_hysteresis():
    """Speed thresholds switch state only when crossed, not when between them."""
    track_set = _track_with_velocity(0.0, SITTING)
    assert _motion_after(track_set, 0.1) == SITTING
    assert _motion_after(track_set, 0.2) == FLYING
    assert _motion_after(track_set, 0.1) == FLYING
    assert _motion_after(track_set, 0.01) == SITTING
    assert _motion_after(track_set, 0.1) == SITTING


def test_extrapolate_matches_formula_and_leaves_track_unchanged():
    """Extrapolation equals p + v dt + a dt^2/2 and does not modify the track."""
    state = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1.0, -1.0, 2.0])
    track = Track(0, state.copy(), np.eye(9), 1.0, True, 5, 0, SITTING, 3)
    result = extrapolate(track, 1.5)
    assert np.allclose(result, state[:3] + state[3:6] * 0.5 + state[6:] * 0.125)
    assert np.array_equal(track.state, state) and track.t_s == 1.0


def test_covariance_stays_symmetric_positive_definite():
    """After 600 frames the covariance is symmetric and positive definite."""
    rng = np.random.default_rng(3)
    covariance = run_line(np.array([0.5, 0.2, -0.1]), 600, 0.002, rng).tracks[0].covariance
    assert np.allclose(covariance, covariance.T)
    assert np.all(np.linalg.eigvalsh((covariance + covariance.T) / 2.0) > 0.0)
