"""Virtual tracking cameras: what each camera reports about the bees in one frame.

Part of the simulator. Cameras deliver blob detections (points), not images. The
camera node code in `simulator/nodes.py` packs the result into `detections` messages.
"""

import numpy as np

from shared.config import CameraModel, Noise
from shared.geometry import CameraPose, focal_length_px, is_visible, project
from simulator.bees import Bee


def observe_bees(
    pose: CameraPose,
    model: CameraModel,
    bees: list[Bee],
    body_length_m: float,
    noise: Noise,
    rng: np.random.Generator,
) -> tuple[tuple[float, float, float], ...]:
    """Report the bees one camera sees in one frame.

    Args:
        pose: Camera pose in the arena frame (m).
        model: Camera model (image size in px, field of view in rad).
        bees: All bees, positions in the arena frame (m).
        body_length_m: Bee body length (m).
        noise: Noise settings (pixel sigma in px, dropout probability 0..1).
        rng: The simulation's random generator.

    Returns:
        One (u_px, v_px, size_px) entry per detected bee, in bee order. Empty when
        nothing is visible or detected.
    """
    if not bees:
        return ()
    all_points_m = np.array([bee.position_m for bee in bees])
    all_distances_m = np.linalg.norm(all_points_m - pose.position_m, axis=1)
    # A bee closer than its own length covers the lens and gives no usable blob.
    seen = is_visible(pose, model, all_points_m) & (all_distances_m >= body_length_m)
    if not seen.any():
        return ()
    points_m = all_points_m[seen]
    distances_m = all_distances_m[seen]
    pixels_px = project(pose, model, points_m)
    # Equidistant lens: pixels = focal length x angle, and the bee subtends about
    # body_length / distance radians.
    sizes_px = focal_length_px(model) * body_length_m / distances_m
    pixels_px = pixels_px + rng.normal(0.0, noise.pixel_sigma_px, size=pixels_px.shape)
    kept = rng.random(len(points_m)) >= noise.dropout_probability
    # Noise can push a point at the image border outside; a sensor cannot report that.
    kept &= (pixels_px[:, 0] >= 0.0) & (pixels_px[:, 0] < model.width_px)
    kept &= (pixels_px[:, 1] >= 0.0) & (pixels_px[:, 1] < model.height_px)
    return tuple(
        (float(u_px), float(v_px), float(size_px))
        for (u_px, v_px), size_px, keep in zip(pixels_px, sizes_px, kept)
        if keep
    )
