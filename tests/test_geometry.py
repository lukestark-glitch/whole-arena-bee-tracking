"""Tests for the shared fisheye camera geometry."""

import numpy as np
import pytest

from shared.config import CameraModel
from shared.geometry import (
    camera_pose,
    focal_length_px,
    is_visible,
    pixel_to_ray,
    project,
)

MODEL = CameraModel(
    id="pi_gs_imx296",
    width_px=1456,
    height_px=1088,
    fov_horizontal_rad=1.919862,
    rate_hz=60.0,
)
TARGET_M = (0.45, 0.15, 0.15)
POSES = [camera_pose((0.0, 0.0, 0.0), TARGET_M), camera_pose((0.9, 0.3, 0.3), TARGET_M)]
CENTRE_PX = np.array([MODEL.width_px / 2.0, MODEL.height_px / 2.0])


def _camera_to_arena(pose, vectors_camera: np.ndarray) -> np.ndarray:
    """Rotate camera-frame row vectors into the arena frame."""
    return np.asarray(vectors_camera, dtype=float) @ pose.rotation


@pytest.mark.parametrize("pose", POSES)
def test_round_trip_point_lies_on_ray(pose) -> None:
    """Projecting then back-projecting gives a ray through the original point."""
    rng = np.random.default_rng(1)
    theta_rad = rng.uniform(0.0, 0.7, 200)
    phi_rad = rng.uniform(0.0, 2.0 * np.pi, 200)
    directions_camera = np.column_stack(
        [np.sin(theta_rad) * np.cos(phi_rad), np.sin(theta_rad) * np.sin(phi_rad), np.cos(theta_rad)]
    )
    points_m = pose.position_m + _camera_to_arena(pose, directions_camera)
    rays = pixel_to_ray(pose, MODEL, project(pose, MODEL, points_m))
    offsets_m = points_m - pose.position_m
    along_m = np.sum(offsets_m * rays, axis=1)
    distance_m = np.linalg.norm(offsets_m - along_m[:, None] * rays, axis=1)
    assert np.all(distance_m < 1e-6)
    assert np.allclose(np.linalg.norm(rays, axis=1), 1.0)


@pytest.mark.parametrize("pose", POSES)
def test_look_at_projects_to_centre(pose) -> None:
    """The look-at point lands on the image centre and the centre ray is the view axis."""
    pixel = project(pose, MODEL, np.array([TARGET_M]))[0]
    assert np.allclose(pixel, (728.0, 544.0), atol=1e-9)
    view = np.array(TARGET_M) - pose.position_m
    view /= np.linalg.norm(view)
    assert np.allclose(pixel_to_ray(pose, MODEL, CENTRE_PX[None, :])[0], view)


def test_half_fov_lands_on_horizontal_edge() -> None:
    """A point at half the horizontal field of view to the right lands at u = width."""
    pose = POSES[0]
    half_rad = MODEL.fov_horizontal_rad / 2.0
    direction_camera = np.array([[np.sin(half_rad), 0.0, np.cos(half_rad)]])
    point_m = pose.position_m + _camera_to_arena(pose, direction_camera)
    u_px, v_px = project(pose, MODEL, point_m)[0]
    assert abs(u_px - MODEL.width_px) < 1e-6
    assert abs(v_px - CENTRE_PX[1]) < 1e-6
    assert focal_length_px(MODEL) == pytest.approx(MODEL.width_px / MODEL.fov_horizontal_rad)


def test_image_orientation() -> None:
    """Higher z is up in the image; the camera frame is right-handed (x right, y down)."""
    pose = POSES[0]
    forward = pose.rotation[2]
    up_m = pose.position_m + forward + np.array([0.0, 0.0, 0.05])
    assert project(pose, MODEL, up_m[None, :])[0, 1] < CENTRE_PX[1]
    # Looking toward +x/+y with z up, the right-hand side is toward +x and -y.
    right_m = np.array([forward[1], -forward[0], 0.0])
    right_m /= np.linalg.norm(right_m)
    assert project(pose, MODEL, (pose.position_m + forward + 0.05 * right_m)[None, :])[0, 0] > CENTRE_PX[0]
    assert project(pose, MODEL, (pose.position_m + forward - 0.05 * right_m)[None, :])[0, 0] < CENTRE_PX[0]


def test_visibility() -> None:
    """Visible means in front of the camera and inside the image."""
    pose = POSES[0]
    forward = pose.rotation[2]
    down = pose.rotation[1]
    behind = pose.position_m - forward
    ahead = pose.position_m + forward
    steep_rad = 1.2  # Far beyond the vertical half field of view (about 0.72 rad).
    above = pose.position_m + np.cos(steep_rad) * forward - np.sin(steep_rad) * down
    visible = is_visible(pose, MODEL, np.array([behind, ahead, above]))
    assert visible.tolist() == [False, True, False]


@pytest.mark.parametrize("pose", POSES)
def test_rotation_is_proper(pose) -> None:
    """The rotation matrix is orthonormal with determinant +1."""
    assert np.allclose(pose.rotation @ pose.rotation.T, np.eye(3))
    assert np.linalg.det(pose.rotation) == pytest.approx(1.0)


def test_camera_pose_rejects_degenerate_views() -> None:
    """Coincident points and a vertical view direction raise ValueError."""
    with pytest.raises(ValueError):
        camera_pose((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))
    with pytest.raises(ValueError):
        camera_pose((1.0, 2.0, 0.0), (1.0, 2.0, 1.0))
