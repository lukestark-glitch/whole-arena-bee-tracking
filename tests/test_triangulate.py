"""Tests for tracker/triangulate.py: ray geometry, guided step, new points, ghosts, speed."""

import dataclasses
import itertools
import time

import numpy as np

from shared.config import Arena, CameraModel, TrackerSettings
from shared.geometry import camera_pose, project
from tracker.triangulate import (
    NEW_POINT,
    NO_POINT,
    Rays,
    TrackingCamera,
    closest_point_to_rays,
    frame_rays,
    guided_points,
    new_points,
    point_ray_distances,
    triangulate,
)

MODEL = CameraModel("m", 1456, 1088, 1.919862, 60.0)
ARENA = Arena((0.9, 0.3, 0.3))
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
    acceleration_smoothing_s=0.2,
    prediction_lead_s=0.1,
    node_silent_after_s=0.5,
)
CENTRE = (0.45, 0.15, 0.15)


def _camera(index: int, position_m, look_at_m) -> TrackingCamera:
    """Build a camera with the test model."""
    return TrackingCamera(f"c{index}", camera_pose(position_m, look_at_m), MODEL)


def _corner_cameras() -> list[TrackingCamera]:
    """Eight cameras on the corners of the arena box, looking at its centre."""
    corners = itertools.product((0.0, 0.9), (0.0, 0.3), (0.0, 0.3))
    return [_camera(i, c, CENTRE) for i, c in enumerate(corners)]


def _rays(cameras, points_m, noise_px=0.0, rng=None) -> Rays:
    """Project points into every camera and convert the pixels back to rays."""
    detections = {}
    for camera in cameras:
        pixels_px = project(camera.pose, camera.model, np.asarray(points_m, dtype=float))
        if noise_px:
            pixels_px = pixels_px + rng.normal(0.0, noise_px, pixels_px.shape)
        detections[camera.id] = tuple((u, v, 1.0) for u, v in pixels_px)
    return frame_rays(detections, cameras)


def _sorted_by_x(points_m: np.ndarray) -> np.ndarray:
    """Sort points by x so results can be compared with the truth."""
    return points_m[np.argsort(points_m[:, 0])]


def _ghost_setup():
    """Three cameras and two bees in the plane z = 0.15 (concept paper figure)."""
    look_at = (0.36, 0.17, 0.15)
    cameras = [
        _camera(0, (0.06, 0.25, 0.15), look_at),
        _camera(1, (0.66, 0.25, 0.15), look_at),
        _camera(2, (0.36, 0.05, 0.15), look_at),
    ]
    bees = np.array([[0.26, 0.17, 0.15], [0.46, 0.17, 0.15]])
    return cameras, bees, _rays(cameras, bees)


def _no_tracks() -> tuple[np.ndarray, np.ndarray]:
    """Empty track ids and predictions."""
    return np.zeros(0, dtype=int), np.zeros((0, 3))


def test_closest_point_recovers_the_point() -> None:
    """Noise-free rays from four cameras meet at the true point."""
    cameras = _corner_cameras()[:4]
    point = np.array([[0.3, 0.1, 0.2]])
    rays = _rays(cameras, point)
    found = closest_point_to_rays(rays.origins_m, rays.directions)
    assert np.allclose(found, point[0], atol=1e-9)


def test_point_ray_distances() -> None:
    """Distance of a point to a ray is perpendicular; inf behind the origin."""
    origins = np.array([[0.0, 0.0, 0.0]])
    directions = np.array([[1.0, 0.0, 0.0]])
    points = np.array([[2.0, 0.3, 0.0], [-1.0, 0.0, 0.0]])
    distances = point_ray_distances(points, origins, directions)
    assert distances.shape == (2, 1)
    assert np.isclose(distances[0, 0], 0.3)
    assert np.isinf(distances[1, 0])


def test_ghosts_are_rejected() -> None:
    """Three cameras see two bees: exactly two points with three cameras each."""
    cameras, bees, rays = _ghost_setup()
    found = new_points(rays, np.zeros(6, dtype=bool), cameras, ARENA, SETTINGS)
    assert len(found.positions_m) == 2
    assert np.allclose(_sorted_by_x(found.positions_m), bees, atol=1e-6)
    assert list(found.camera_counts) == [3, 3]
    full = triangulate(rays, *_no_tracks(), cameras, ARENA, SETTINGS)
    assert np.allclose(_sorted_by_x(full.positions_m), bees, atol=1e-6)


def test_ghosts_lose_against_real_points_with_two_cameras() -> None:
    """With min_cameras=2 and no coverage rule a detection still supports one point only."""
    cameras, bees, rays = _ghost_setup()
    loose = dataclasses.replace(SETTINGS, min_cameras=2, coverage_share=0.0)
    found = new_points(rays, np.zeros(6, dtype=bool), cameras, ARENA, loose)
    assert len(found.positions_m) == 2
    assert np.allclose(_sorted_by_x(found.positions_m), bees, atol=1e-6)


def test_five_points_eight_cameras() -> None:
    """Five separated points are all found, exactly and with 0.5 px pixel noise."""
    cameras = _corner_cameras()
    truth = np.array(
        [[0.15, 0.08, 0.1], [0.3, 0.22, 0.2], [0.5, 0.1, 0.25], [0.65, 0.2, 0.08], [0.8, 0.12, 0.2]]
    )
    exact = triangulate(_rays(cameras, truth), *_no_tracks(), cameras, ARENA, SETTINGS)
    assert np.allclose(_sorted_by_x(exact.positions_m), truth, atol=1e-6)
    rng = np.random.default_rng(1)
    noisy_rays = _rays(cameras, truth, noise_px=0.5, rng=rng)
    noisy = triangulate(noisy_rays, *_no_tracks(), cameras, ARENA, SETTINGS)
    assert len(noisy.positions_m) == 5
    assert np.allclose(_sorted_by_x(noisy.positions_m), truth, atol=2e-3)


def test_guided_step_uses_predictions() -> None:
    """Predictions 5 mm off give points with the right ids; nothing is left for new points."""
    cameras = _corner_cameras()
    truth = np.array([[0.2, 0.1, 0.1], [0.45, 0.2, 0.2], [0.7, 0.1, 0.2]])
    rays = _rays(cameras, truth)
    ids = np.array([7, 3, 9])
    predicted = truth + np.array([0.005, 0.0, 0.0])
    points, used = guided_points(rays, ids, predicted, cameras, SETTINGS)
    assert list(points.track_ids) == [7, 3, 9]
    assert np.allclose(points.positions_m, truth, atol=1e-6)
    assert used.all()
    assert len(new_points(rays, used, cameras, ARENA, SETTINGS).positions_m) == 0


def test_far_prediction_claims_nothing() -> None:
    """A prediction 10 cm from the bee gets no point; the bee becomes a new point."""
    cameras = _corner_cameras()
    truth = np.array([CENTRE])
    rays = _rays(cameras, truth)
    ids = np.array([4])
    predicted = truth + np.array([0.1, 0.0, 0.0])
    guided, used = guided_points(rays, ids, predicted, cameras, SETTINGS)
    assert len(guided.positions_m) == 0 and not used.any()
    full = triangulate(rays, ids, predicted, cameras, ARENA, SETTINGS)
    assert list(full.track_ids) == [NEW_POINT]
    assert np.allclose(full.positions_m, truth, atol=1e-6)


def test_wrong_ray_is_dropped() -> None:
    """An extra ray 1 cm from the true point does not move the guided point by 1 mm."""
    cameras = _corner_cameras()
    truth = np.array(CENTRE)
    rays = _rays(cameras, truth[None])
    to_truth = truth - rays.origins_m[0]
    side = np.cross(to_truth, [1.0, 0.0, 0.0])
    side /= np.linalg.norm(side)
    predicted = truth + 0.01 * side
    bad = predicted - rays.origins_m[0]
    bad /= np.linalg.norm(bad)
    with_bad = Rays(
        np.vstack([rays.origins_m, rays.origins_m[0]]),
        np.vstack([rays.directions, bad]),
        np.append(rays.camera_index, 0),
        np.append(rays.detection_index, 1),
    )
    points, _ = guided_points(with_bad, np.array([1]), predicted[None], cameras, SETTINGS)
    assert len(points.positions_m) == 1
    assert np.allclose(points.positions_m[0], truth, atol=1e-3)


def test_two_cameras_give_no_new_point() -> None:
    """A single ray pair from two cameras is not enough with min_cameras=3."""
    cameras = _corner_cameras()[:2]
    rays = _rays(cameras, np.array([CENTRE]))
    found = triangulate(rays, *_no_tracks(), cameras, ARENA, SETTINGS)
    assert len(found.positions_m) == 0


def test_empty_inputs() -> None:
    """No rays, or no tracks and no rays, give empty points with the right shapes."""
    cameras = _corner_cameras()
    empty = frame_rays({}, cameras)
    found = triangulate(empty, np.array([1]), np.array([CENTRE]), cameras, ARENA, SETTINGS)
    assert found.positions_m.shape == (0, 3)
    assert found.camera_counts.shape == (0,)
    assert found.track_ids.shape == (0,)
    rays = _rays(cameras, np.array([CENTRE]))
    assert len(triangulate(rays, *_no_tracks(), cameras, ARENA, SETTINGS).positions_m) == 1


def test_speed_of_one_frame() -> None:
    """Sixteen cameras and forty bees are triangulated in under 150 ms per frame (aim: 5 ms)."""
    corners = list(itertools.product((0.0, 0.9), (0.0, 0.3), (0.0, 0.3)))
    edges = [(x, y, z) for x in (0.3, 0.6) for y in (0.0, 0.3) for z in (0.0, 0.3)]
    positions = corners + [e for e in edges if e[1] in (0.0, 0.3) and e[2] in (0.0, 0.3)]
    cameras = [_camera(i, p, CENTRE) for i, p in enumerate(positions[:16])]
    rng = np.random.default_rng(3)
    points = rng.uniform([0.1, 0.05, 0.05], [0.8, 0.25, 0.25], (40, 3))
    rays = _rays(cameras, points)
    start_s = time.perf_counter()
    for _ in range(20):
        found = triangulate(rays, *_no_tracks(), cameras, ARENA, SETTINGS)
    per_frame_ms = (time.perf_counter() - start_s) / 20 * 1000.0
    print(f"triangulate: {per_frame_ms:.1f} ms per frame, {len(found.positions_m)} points")
    assert len(found.positions_m) > 0
    assert per_frame_ms < 150.0


def test_guided_point_needs_the_usual_share_of_cameras() -> None:
    """A track that claims only 2 of 8 covering cameras gets no point; its rays stay free."""
    cameras = _corner_cameras()
    truth = np.array([CENTRE])
    rays = _rays(cameras, truth)
    two = Rays(
        rays.origins_m[:2],
        rays.directions[:2],
        rays.camera_index[:2],
        rays.detection_index[:2],
    )
    guided, used = guided_points(two, np.array([4]), truth, cameras, SETTINGS)
    assert len(guided.positions_m) == 0 and not used.any()
    guided, used = guided_points(rays, np.array([4]), truth, cameras, SETTINGS)
    assert list(guided.track_ids) == [4] and used.all()


def test_ray_point_maps_rays_of_new_points() -> None:
    """Every ray of a new point maps to its row; rays of no point map to NO_POINT."""
    cameras = _corner_cameras()
    truth = np.array([CENTRE, (0.2, 0.1, 0.1)])
    rays = _rays(cameras, truth)
    found = triangulate(rays, *_no_tracks(), cameras, ARENA, SETTINGS)
    assert len(found.positions_m) == 2
    assert found.ray_point.shape == (len(rays.camera_index),)
    for row, position_m in enumerate(found.positions_m):
        supported = rays.detection_index[found.ray_point == row]
        expected = int(np.argmin(np.linalg.norm(truth - position_m, axis=1)))
        assert list(supported) == [expected] * len(cameras)
    extra = Rays(*(np.concatenate([a, a[:1]]) for a in (
        rays.origins_m, rays.directions, rays.camera_index, rays.detection_index)))
    extra.directions[-1] = -extra.directions[-1]
    assert triangulate(extra, *_no_tracks(), cameras, ARENA, SETTINGS).ray_point[-1] == NO_POINT


def test_ray_point_in_concatenated_result() -> None:
    """With one guided and one new point the row numbers of both are right."""
    cameras = _corner_cameras()
    truth = np.array([CENTRE, (0.2, 0.1, 0.1)])
    rays = _rays(cameras, truth)
    found = triangulate(rays, np.array([7]), truth[:1], cameras, ARENA, SETTINGS)
    assert list(found.track_ids) == [7, NEW_POINT]
    for row in (0, 1):
        assert np.all(rays.detection_index[found.ray_point == row] == row)
        assert np.sum(found.ray_point == row) == len(cameras)
    assert not np.any(found.ray_point == NO_POINT)


def test_detection_index_with_two_detections() -> None:
    """A camera with two detections numbers its rays 0 and 1; others start at 0."""
    cameras = _corner_cameras()[:2]
    detections = {"c0": ((1.0, 2.0, 1.0), (3.0, 4.0, 1.0)), "c1": ((5.0, 6.0, 1.0),)}
    rays = frame_rays(detections, cameras)
    assert list(rays.camera_index) == [0, 0, 1]
    assert list(rays.detection_index) == [0, 1, 0]
