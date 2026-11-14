"""Tests for the wand calibration solver (tracker/calibrate.py)."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from shared.config import load_config
from shared.geometry import CameraPose, is_visible, project
from tracker.calibrate import WandData, pose_difference, solve_camera_pose, solve_cameras
from tracker.triangulate import TrackingCamera, tracking_cameras

CONFIG = load_config("config/default.yaml")
START_SHIFT_M = 0.01  # nominal-pose error: 1 cm
START_TILT_RAD = np.radians(2.0)  # nominal-pose error: 2 degrees
N_WAND_POINTS = 150
FRAMES_PER_REFERENCE = 3


def _perturb(pose: CameraPose, shift_m: float, tilt_rad: float, rng: np.random.Generator) -> CameraPose:
    """Move a pose by exactly shift_m in a random direction and tilt_rad about a random axis."""
    direction = rng.normal(size=3)
    axis = rng.normal(size=3)
    rot = Rotation.from_rotvec(tilt_rad * axis / np.linalg.norm(axis)).as_matrix()
    return CameraPose(pose.position_m + shift_m * direction / np.linalg.norm(direction), pose.rotation @ rot)


def _start_cameras(true_cams: list[TrackingCamera], rng: np.random.Generator) -> list[TrackingCamera]:
    """Cameras with every pose moved off by 1 cm and 2 degrees."""
    return [TrackingCamera(c.id, _perturb(c.pose, START_SHIFT_M, START_TILT_RAD, rng), c.model) for c in true_cams]


def _wand_data(true_cams: list[TrackingCamera], rng: np.random.Generator, noise_px: float) -> tuple[WandData, np.ndarray]:
    """Wand run: reference points (known, repeated) plus random unknown points; returns data and true points."""
    reference = np.array(CONFIG.calibration.reference_points_m, dtype=float)
    random_pts = rng.uniform(0.0, 1.0, size=(N_WAND_POINTS, 3)) * np.array(CONFIG.arena.size_m)
    true_points = np.vstack([np.repeat(reference, FRAMES_PER_REFERENCE, axis=0), random_pts])
    known = np.full_like(true_points, np.nan)
    n_known = len(reference) * FRAMES_PER_REFERENCE
    known[:n_known] = true_points[:n_known]
    frames, cams, pixels = [], [], []
    for c, cam in enumerate(true_cams):
        vis = is_visible(cam.pose, cam.model, true_points)
        k = np.flatnonzero(vis)
        pix = project(cam.pose, cam.model, true_points[k]) + rng.normal(scale=noise_px, size=(len(k), 2))
        frames.append(k)
        cams.append(np.full(len(k), c))
        pixels.append(pix)
    data = WandData(np.concatenate(frames), np.concatenate(cams), np.vstack(pixels), known)
    return data, true_points


def _check_cameras(solved, true_cams, max_m: float, max_deg: float) -> None:
    """Assert every solved pose is within the given distance and angle of the truth."""
    for s, t in zip(solved, true_cams):
        dist_m, ang_rad = pose_difference(s.pose, t.pose)
        assert dist_m < max_m, (s.id, dist_m)
        assert np.degrees(ang_rad) < max_deg, (s.id, np.degrees(ang_rad))


def test_noise_free_recovery() -> None:
    """Noise-free wand data recovers every camera within 1 mm and 0.1 degree."""
    rng = np.random.default_rng(1)
    true_cams = tracking_cameras(CONFIG)
    start = _start_cameras(true_cams, rng)
    for s, t in zip(start, true_cams):
        dist_m, ang_rad = pose_difference(s.pose, t.pose)
        assert dist_m == pytest.approx(START_SHIFT_M, rel=1e-6)
        assert ang_rad == pytest.approx(START_TILT_RAD, rel=1e-6)
    data, true_points = _wand_data(true_cams, rng, 0.0)
    solved, points = solve_cameras(start, data)
    _check_cameras(solved, true_cams, 1e-3, 0.1)
    assert np.abs(points - true_points).max() < 1e-3
    assert [c.id for c in solved] == [c.id for c in true_cams]


def test_noisy_recovery() -> None:
    """With 0.5 px noise every camera is within 5 mm and 0.5 degree."""
    rng = np.random.default_rng(2)
    true_cams = tracking_cameras(CONFIG)
    data, _ = _wand_data(true_cams, rng, 0.5)
    solved, _ = solve_cameras(_start_cameras(true_cams, rng), data)
    _check_cameras(solved, true_cams, 5e-3, 0.5)


def test_solve_camera_pose_from_markers() -> None:
    """One camera off by 1 degree is recovered from 8 known points."""
    rng = np.random.default_rng(3)
    true_cam = tracking_cameras(CONFIG)[0]
    points = rng.uniform(0.0, 1.0, size=(60, 3)) * np.array(CONFIG.arena.size_m)
    points = points[is_visible(true_cam.pose, true_cam.model, points)][:8]
    pixels = project(true_cam.pose, true_cam.model, points)
    start = TrackingCamera(true_cam.id, _perturb(true_cam.pose, 0.0, np.radians(1.0), rng), true_cam.model)
    pose = solve_camera_pose(start, points, pixels)
    dist_m, ang_rad = pose_difference(pose, true_cam.pose)
    assert dist_m < 2e-3
    assert np.degrees(ang_rad) < 0.1


def test_solve_camera_pose_needs_four_points() -> None:
    """Fewer than 4 points raises ValueError."""
    cam = tracking_cameras(CONFIG)[0]
    points = np.array(CONFIG.arena.size_m) * np.array([[0.5, 0.5, 0.5]] * 3)
    with pytest.raises(ValueError):
        solve_camera_pose(cam, points, project(cam.pose, cam.model, points))


def test_camera_without_observations_raises() -> None:
    """A camera that saw nothing raises ValueError."""
    rng = np.random.default_rng(4)
    true_cams = tracking_cameras(CONFIG)
    data, _ = _wand_data(true_cams, rng, 0.0)
    keep = data.camera_index != 0
    data = WandData(data.frame_index[keep], data.camera_index[keep], data.pixels_px[keep], data.known_points_m)
    with pytest.raises(ValueError):
        solve_cameras(true_cams, data)
