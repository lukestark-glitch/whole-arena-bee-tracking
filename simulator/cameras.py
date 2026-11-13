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
    points_m = all_points_m[is_visible(pose, model, all_points_m)]
    if len(points_m) == 0:
        return ()
    pixels_px = project(pose, model, points_m)
    distances_m = np.linalg.norm(points_m - pose.position_m, axis=1)
    # Equidistant lens: pixels = focal length x angle, and the bee subtends about
    # body_length / distance radians.
    sizes_px = focal_length_px(model) * body_length_m / distances_m
    pixels_px = pixels_px + rng.normal(0.0, noise.pixel_sigma_px, size=pixels_px.shape)
    kept = rng.random(len(points_m)) >= noise.dropout_probability
    return tuple(
        (float(u_px), float(v_px), float(size_px))
        for (u_px, v_px), size_px, keep in zip(pixels_px, sizes_px, kept)
        if keep
    )
