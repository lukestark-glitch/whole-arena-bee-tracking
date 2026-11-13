"""What the robot's stereo camera reports in the emulator.

The emulator computes blobs from true bee positions and adds measurement noise. This
module is part of `robot_emulator/` and is used by the simulator to feed
`robot_emulator/controller.py`. It receives positions as a plain array and imports
nothing from `simulator/`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from shared.config import StereoCamera
from shared.geometry import CameraPose, focal_length_px, is_visible, project

# Smallest blob diameter reported (pixels); keeps noisy sizes positive for far bees.
MIN_SIZE_PX = 0.5


@dataclass(frozen=True)
class Blob:
    """One bee as seen by the robot's stereo camera.

    Attributes:
        u_px: Image column, pixels, origin top-left, u to the right.
        v_px: Image row, pixels, origin top-left, v down.
        size_px: Blob diameter in pixels.
        distance_m: Measured distance from the camera centre, metres.
    """

    u_px: float
    v_px: float
    size_px: float
    distance_m: float


def _seen_mask(
    pose: CameraPose, stereo: StereoCamera, positions_m: np.ndarray, body_length_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """Find the bees inside the field of view and not closer than one body length.

    Args:
        pose: Camera pose in the arena frame.
        stereo: Stereo camera parameters.
        positions_m: Bee positions, shape (N, 3), arena frame, metres.
        body_length_m: Bee body length, metres.

    Returns:
        Boolean mask of seen bees, shape (N,), and true distances, shape (N,), metres.
    """
    distance_m = np.linalg.norm(positions_m - pose.position_m, axis=1)
    seen = is_visible(pose, stereo.model, positions_m) & (distance_m >= body_length_m)
    return seen, distance_m


def observe_stereo(
    pose: CameraPose,
    stereo: StereoCamera,
    positions_m: np.ndarray,
    body_length_m: float,
    rng: np.random.Generator,
) -> list[Blob]:
    """Compute the noisy blobs the stereo camera reports for the given bees.

    Args:
        pose: Camera pose in the arena frame.
        stereo: Stereo camera parameters (image model, baseline, noise).
        positions_m: True bee positions, shape (N, 3), arena frame, metres. May be empty.
        body_length_m: Bee body length, metres.
        rng: Seeded generator; draws pixel, size and distance noise in that order.

    Returns:
        Blobs in the order of the input bees; bees that are not seen or whose noisy
        pixel falls outside the image are left out.
    """
    if len(positions_m) == 0:
        return []
    seen, distance_all_m = _seen_mask(pose, stereo, positions_m, body_length_m)
    if not seen.any():
        return []
    points_m = positions_m[seen]
    distance_m = distance_all_m[seen]
    count = len(points_m)
    focal_px = focal_length_px(stereo.model)

    pixels_px = project(pose, stereo.model, points_m)
    pixels_px = pixels_px + rng.normal(0.0, stereo.pixel_sigma_px, (count, 2))
    # Equidistant lens: pixels = focal length * angle, and a bee subtends about
    # body_length / distance radians.
    size_px = focal_px * body_length_m / distance_m
    size_px = np.maximum(size_px + rng.normal(0.0, stereo.size_sigma_px, count), MIN_SIZE_PX)
    # Stereo depth = focal length * baseline / disparity, so a disparity error grows
    # with the square of the distance.
    sigma_distance_m = distance_m**2 * stereo.disparity_sigma_px / (focal_px * stereo.baseline_m)
    measured_m = distance_m + rng.normal(0.0, 1.0, count) * sigma_distance_m

    inside = (
        (pixels_px[:, 0] >= 0)
        & (pixels_px[:, 0] < stereo.model.width_px)
        & (pixels_px[:, 1] >= 0)
        & (pixels_px[:, 1] < stereo.model.height_px)
    )
    return [
        Blob(float(p[0]), float(p[1]), float(s), float(d))
        for p, s, d, keep in zip(pixels_px, size_px, measured_m, inside)
        if keep
    ]
