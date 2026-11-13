"""Virtual tracking cameras: what each camera reports about the bees in one frame.

Part of the simulator. Cameras deliver blob detections (points), not images. The
camera node code in `simulator/nodes.py` packs the result into `detections` messages.
"""

import numpy as np
from scipy.sparse.csgraph import connected_components

from shared.config import CameraModel, Noise
from shared.geometry import CameraPose, focal_length_px, is_visible, project
from simulator.bees import Bee


def _merge_overlapping(
    pixels_px: np.ndarray, sizes_px: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Merge blobs whose discs touch, as a real blob detector would.

    Overlap is transitive: a chain of touching blobs becomes one blob.

    Args:
        pixels_px: Blob centres, shape (N, 2), image frame (u right, v down), pixels.
        sizes_px: Blob diameters, shape (N,), pixels.

    Returns:
        Merged centres, shape (M, 2), and sizes, shape (M,), ordered by the lowest
        input index of each group.
    """
    gaps_px = np.linalg.norm(pixels_px[:, None, :] - pixels_px[None, :, :], axis=2)
    # Two discs touch when their centres are closer than the sum of the radii.
    touching = gaps_px < (sizes_px[:, None] + sizes_px[None, :]) / 2.0
    # Every blob touches itself. In most frames nothing else touches, and skipping the
    # grouping then keeps the simulator well inside its frame time.
    if np.count_nonzero(touching) == len(sizes_px):
        return pixels_px, sizes_px
    count, labels = connected_components(touching, directed=False)
    areas_px2 = sizes_px**2
    # Labels are numbered in order of first appearance, so group g is ordered by lowest
    # index. The area-weighted mean puts the merged blob where most of the light is;
    # adding the areas keeps the total blob area.
    group_areas_px2 = np.bincount(labels, weights=areas_px2, minlength=count)
    merged_pixels_px = np.column_stack(
        [
            np.bincount(labels, weights=areas_px2 * pixels_px[:, axis], minlength=count)
            for axis in range(pixels_px.shape[1])
        ]
    ) / group_areas_px2[:, None]
    return merged_pixels_px, np.sqrt(group_areas_px2)


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
        One (u_px, v_px, size_px) entry per detected blob (overlapping bees merged), in
        order of the lowest bee index of each blob. Empty when
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
    pixels_px, sizes_px = _merge_overlapping(pixels_px, sizes_px)
    pixels_px = pixels_px + rng.normal(0.0, noise.pixel_sigma_px, size=pixels_px.shape)
    kept = rng.random(len(pixels_px)) >= noise.dropout_probability
    # Noise can push a point at the image border outside; a sensor cannot report that.
    kept &= (pixels_px[:, 0] >= 0.0) & (pixels_px[:, 0] < model.width_px)
    kept &= (pixels_px[:, 1] >= 0.0) & (pixels_px[:, 1] < model.height_px)
    return tuple(
        (float(u_px), float(v_px), float(size_px))
        for (u_px, v_px), size_px, keep in zip(pixels_px, sizes_px, kept)
        if keep
    )
