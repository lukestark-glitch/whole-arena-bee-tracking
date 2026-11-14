"""Calibration tools of the simulator.

Covers where the LED wand is during a wand run, the deliberate error of the true
cameras, and what a camera sees of the wall markers. Used by `simulator/main.py`.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from shared.config import Arena, CalibrationSettings, CameraModel, Marker
from shared.geometry import CameraPose, is_visible, project
from shared.wand import DONE, REFERENCE, STOP, SWEEP, wand_phase

# Per-axis sweep frequencies [Hz]. They share no common period within a run, so the
# path fills the volume instead of retracing one closed curve.
SWEEP_FREQUENCIES_HZ = (0.23, 0.37, 0.61)

# Time spacing [s] between the sample times of the stop points; spreads the stops
# over the volume.
STOP_SPACING_S = 1.7


def sweep_point(settings: CalibrationSettings, arena: Arena, sweep_t_s: float) -> np.ndarray:
    """Return the wand position on the sweep path.

    Args:
        settings: Calibration settings (uses `sweep_margin_m`).
        arena: Arena box.
        sweep_t_s: Time since the sweep began, in seconds.

    Returns:
        Position (3,) in metres, arena frame.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    centre_m = size_m / 2.0
    half_m = size_m / 2.0 - settings.sweep_margin_m
    freq_hz = np.asarray(SWEEP_FREQUENCIES_HZ)
    # A sine per axis around the arena centre, scaled to stay inside the margin.
    return centre_m + half_m * np.sin(2.0 * np.pi * freq_hz * sweep_t_s)


def stop_points(settings: CalibrationSettings, arena: Arena) -> np.ndarray:
    """Return the wand positions of all stops.

    Args:
        settings: Calibration settings (uses `stop_count`).
        arena: Arena box.

    Returns:
        Array (stop_count, 3) in metres, arena frame.
    """
    return np.array(
        [sweep_point(settings, arena, k * STOP_SPACING_S) for k in range(settings.stop_count)]
    ).reshape(settings.stop_count, 3)


def wand_position(settings: CalibrationSettings, arena: Arena, t_s: float) -> np.ndarray:
    """Return the wand position at a simulation time.

    Args:
        settings: Calibration settings.
        arena: Arena box.
        t_s: Simulation time since the wand run began, in seconds.

    Returns:
        Position (3,) in metres, arena frame.
    """
    phase, index, since_s = wand_phase(settings, t_s)
    if phase == REFERENCE:
        return np.asarray(settings.reference_points_m[index], dtype=float)
    if phase == SWEEP:
        return sweep_point(settings, arena, since_s)
    if phase == STOP:
        return stop_points(settings, arena)[index]
    # DONE: the wand stays where the last stop left it.
    return stop_points(settings, arena)[-1]


def _random_unit_vector(rng: np.random.Generator) -> np.ndarray:
    """Draw a uniformly random direction.

    Args:
        rng: Seeded random generator.

    Returns:
        Unit vector (3,).
    """
    v = rng.normal(size=3)
    return v / np.linalg.norm(v)


def perturb_pose(
    pose: CameraPose, distance_m: float, angle_rad: float, rng: np.random.Generator
) -> CameraPose:
    """Return a copy of a pose with a deliberate position and orientation error.

    Args:
        pose: True pose (arena frame).
        distance_m: Exact position shift, in metres.
        angle_rad: Exact rotation about a random axis, in radians.
        rng: Seeded random generator (direction drawn first, then axis).

    Returns:
        The perturbed pose.
    """
    direction = _random_unit_vector(rng)
    axis = _random_unit_vector(rng)
    # A rotation vector (axis times angle) gives a rotation of exactly angle_rad.
    delta = Rotation.from_rotvec(axis * angle_rad).as_matrix()
    return CameraPose(
        position_m=pose.position_m + direction * distance_m,
        rotation=pose.rotation @ delta,
    )


def observe_markers(
    pose: CameraPose,
    model: CameraModel,
    markers: tuple[Marker, ...],
    pixel_sigma_px: float,
    rng: np.random.Generator,
) -> tuple[tuple[int, float, float], ...]:
    """Return what a camera sees of the wall markers.

    Args:
        pose: Camera pose (arena frame).
        model: Camera model (image size).
        markers: Markers with arena-frame positions in metres.
        pixel_sigma_px: Standard deviation of the pixel noise, in pixels.
        rng: Seeded random generator; noise is drawn only for visible markers.

    Returns:
        Tuples (marker_id, u_px, v_px) in marker order. Markers that are not
        visible or whose noisy pixel falls outside the image are left out.
    """
    if not markers:
        return ()
    points_m = np.array([m.position_m for m in markers], dtype=float)
    visible = is_visible(pose, model, points_m)
    if not visible.any():
        return ()
    pixels_px = project(pose, model, points_m[visible])
    pixels_px = pixels_px + rng.normal(0.0, pixel_sigma_px, pixels_px.shape)
    seen = [m for m, v in zip(markers, visible) if v]
    result = []
    for marker, (u_px, v_px) in zip(seen, pixels_px):
        # Noise can push a marker at the image edge out of the frame.
        if 0.0 <= u_px < model.width_px and 0.0 <= v_px < model.height_px:
            result.append((int(marker.id), float(u_px), float(v_px)))
    return tuple(result)
