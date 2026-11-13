"""Camera geometry shared by the simulator and the tracker.

There is exactly one camera model in the project: an equidistant fisheye lens. The
simulator uses `project` to turn bee positions into pixels, and the tracker uses
`pixel_to_ray` to turn reported pixels back into rays. Keeping both directions here
guarantees they are exact inverses of each other.

Frames: the arena frame is right-handed (x along the long side, y away from the robot,
z up, metres). The camera frame is right-handed with x right, y down and z along the
optical axis. Pixels have their origin top-left, u to the right and v down.
"""

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from shared.config import CameraModel

# Arena "up" axis; it fixes the camera roll so that image "up" is arena +z.
_ARENA_UP = np.array([0.0, 0.0, 1.0])

# Below this norm a direction is treated as zero or parallel to another one.
_MIN_NORM = 1e-9

# Visibility needs the ray to be in front of the camera: angle to the axis below 90 deg.
_MAX_VISIBLE_THETA_RAD = np.pi / 2.0


@dataclass(frozen=True)
class CameraPose:
    """Position and orientation of a camera in the arena frame.

    Attributes:
        position_m: Camera centre, shape (3,), arena frame, metres.
        rotation: Shape (3, 3). Rows are the camera axes (x right, y down, z optical
            axis) expressed in the arena frame, so
            ``p_camera = rotation @ (p_arena - position_m)``.
    """

    position_m: np.ndarray
    rotation: np.ndarray


def camera_pose(position_m: Sequence[float], look_at_m: Sequence[float]) -> CameraPose:
    """Build a camera pose whose optical axis points at a target, with arena +z up.

    Args:
        position_m: Camera centre, arena frame, metres.
        look_at_m: Point the optical axis passes through, arena frame, metres.

    Returns:
        The camera pose.

    Raises:
        ValueError: If position and target coincide, or the view direction is parallel
            to the arena z axis (roll would be undefined).
    """
    position = np.asarray(position_m, dtype=float)
    forward = np.asarray(look_at_m, dtype=float) - position
    if np.linalg.norm(forward) < _MIN_NORM:
        raise ValueError("position_m and look_at_m must differ")
    forward /= np.linalg.norm(forward)
    # Right is perpendicular to both the view direction and "up".
    right = np.cross(forward, _ARENA_UP)
    if np.linalg.norm(right) < _MIN_NORM:
        raise ValueError("view direction must not be parallel to the arena z axis")
    right /= np.linalg.norm(right)
    # Completes a right-handed set: right x down = forward.
    down = np.cross(forward, right)
    return CameraPose(position_m=position, rotation=np.stack([right, down, forward]))


def focal_length_px(model: CameraModel) -> float:
    """Return the equidistant focal length in pixels.

    The horizontal image edge (half the width) sits at half the horizontal field of
    view, so ``f = (width / 2) / (fov / 2)``.

    Args:
        model: Camera model.

    Returns:
        Focal length in pixels per radian.
    """
    return (model.width_px / 2.0) / (model.fov_horizontal_rad / 2.0)


def _to_camera_frame(pose: CameraPose, points_m: np.ndarray) -> np.ndarray:
    """Express arena-frame points in the camera frame.

    Args:
        pose: Camera pose.
        points_m: Shape (N, 3), arena frame, metres.

    Returns:
        Shape (N, 3), camera frame, metres.
    """
    return (np.asarray(points_m, dtype=float) - pose.position_m) @ pose.rotation.T


def _angle_from_axis_rad(points_camera_m: np.ndarray) -> np.ndarray:
    """Return the angle between each point's direction and the optical axis.

    Args:
        points_camera_m: Shape (N, 3), camera frame, metres.

    Returns:
        Shape (N,), radians in [0, pi]. arctan2 stays finite on the axis (theta = 0).
    """
    radial_m = np.hypot(points_camera_m[:, 0], points_camera_m[:, 1])
    return np.arctan2(radial_m, points_camera_m[:, 2])


def project(pose: CameraPose, model: CameraModel, points_m: np.ndarray) -> np.ndarray:
    """Project arena points to pixels with the equidistant fisheye model.

    A ray at angle theta from the optical axis lands at radius ``r = f * theta`` from the
    principal point, in the direction of the point's (x, y) in the camera frame. No
    visibility test is made.

    Args:
        pose: Camera pose.
        model: Camera model.
        points_m: Shape (N, 3), arena frame, metres.

    Returns:
        Shape (N, 2) array of (u, v) in pixels.
    """
    points_camera_m = _to_camera_frame(pose, points_m)
    theta_rad = _angle_from_axis_rad(points_camera_m)
    radial_m = np.hypot(points_camera_m[:, 0], points_camera_m[:, 1])
    radius_px = focal_length_px(model) * theta_rad
    # On the axis radial_m is 0 and so is the offset; avoid dividing by zero there.
    scale_px_per_m = np.divide(
        radius_px, radial_m, out=np.zeros_like(radius_px), where=radial_m > 0.0
    )
    offset_px = points_camera_m[:, :2] * scale_px_per_m[:, None]
    centre_px = np.array([model.width_px / 2.0, model.height_px / 2.0])
    return centre_px + offset_px


def pixel_to_ray(
    pose: CameraPose, model: CameraModel, pixels_px: np.ndarray
) -> np.ndarray:
    """Convert pixels to unit ray directions; exact inverse of `project`.

    Args:
        pose: Camera pose. Rays start at ``pose.position_m``.
        model: Camera model.
        pixels_px: Shape (N, 2) array of (u, v) in pixels.

    Returns:
        Shape (N, 3) unit vectors, arena frame.
    """
    centre_px = np.array([model.width_px / 2.0, model.height_px / 2.0])
    offset_px = np.asarray(pixels_px, dtype=float) - centre_px
    theta_rad = np.hypot(offset_px[:, 0], offset_px[:, 1]) / focal_length_px(model)
    # sin(theta) / r equals sinc(theta / pi) / f, which stays finite at the centre.
    xy_scale = np.sinc(theta_rad / np.pi) / focal_length_px(model)
    direction_camera = np.column_stack(
        [offset_px * xy_scale[:, None], np.cos(theta_rad)]
    )
    # Rotation rows are camera axes, so the inverse transform is multiplication by R.
    return direction_camera @ pose.rotation


def is_visible(pose: CameraPose, model: CameraModel, points_m: np.ndarray) -> np.ndarray:
    """Test whether points are in front of the camera and inside the image.

    Args:
        pose: Camera pose.
        model: Camera model.
        points_m: Shape (N, 3), arena frame, metres.

    Returns:
        Shape (N,) boolean array, True where visible.
    """
    theta_rad = _angle_from_axis_rad(_to_camera_frame(pose, points_m))
    pixels_px = project(pose, model, points_m)
    in_image = (
        (pixels_px[:, 0] >= 0.0)
        & (pixels_px[:, 0] < model.width_px)
        & (pixels_px[:, 1] >= 0.0)
        & (pixels_px[:, 1] < model.height_px)
    )
    return (theta_rad < _MAX_VISIBLE_THETA_RAD) & in_image
