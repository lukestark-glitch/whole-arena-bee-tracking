"""Calibration of the tracking cameras from a wand run and from wall markers.

Part of the tracker. A single moving point (an LED wand) is seen by all cameras;
bundle adjustment then refines the camera poses and the wand positions together.
Three wand positions that are known in the arena frame fix its position,
orientation and scale. The functions here are pure: a later change adds the
command that collects a wand run.

Frames and units: arena frame in metres, pixels with origin top-left, angles in
radians.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from shared.geometry import CameraPose, pixel_to_ray, project
from tracker.triangulate import TrackingCamera, closest_point_to_rays

POSE_PARAMS = 6  # 3 position offsets + 3 rotation-vector components per camera
POINT_PARAMS = 3  # x, y, z per unknown wand frame
MIN_OBSERVATIONS_PER_FRAME = 2  # two rays are needed to triangulate a point
MIN_KNOWN_FRAMES = 3  # three different reference points fix position, orientation and scale
MIN_MARKERS = 4  # fewer known points leave the six pose values under-determined
MAX_EVALUATIONS = 100  # noisy data never reaches the tight tolerances; stop after convergence stalls
SOLVER_TOLERANCE = 1e-12  # tight, so the result is limited by the data and not the solver


@dataclass(frozen=True)
class WandData:
    """Observations of the wand: observation i says camera `camera_index[i]` saw wand frame `frame_index[i]` at `pixels_px[i]`.

    Attributes:
        frame_index: (M,) int, wand frame of each observation, 0..K-1.
        camera_index: (M,) int, index into the camera list.
        pixels_px: (M, 2) observed pixel (u, v).
        known_points_m: (K, 3) known wand position in the arena frame (m) for
            reference frames, nan for all other frames.
    """

    frame_index: np.ndarray
    camera_index: np.ndarray
    pixels_px: np.ndarray
    known_points_m: np.ndarray


def _pose_from(start: CameraPose, params: np.ndarray) -> CameraPose:
    """Build a pose from 6 offsets relative to a start pose.

    Args:
        start: Start pose.
        params: (6,) position offset in m (arena frame) then rotation vector in rad.

    Returns:
        The pose with position start + d and rotation R(w) @ start rotation.
    """
    delta = Rotation.from_rotvec(params[3:]).as_matrix()
    return CameraPose(start.position_m + params[:3], delta @ start.rotation)


def pose_difference(a: CameraPose, b: CameraPose) -> tuple[float, float]:
    """Compare two poses.

    Args:
        a: First pose.
        b: Second pose.

    Returns:
        (distance between the centres in m, rotation angle between the orientations in rad).
    """
    distance_m = float(np.linalg.norm(a.position_m - b.position_m))
    # Rotation angle of the relative rotation: trace(R) = 1 + 2 cos(angle).
    cos_angle = (np.trace(a.rotation @ b.rotation.T) - 1.0) / 2.0
    return distance_m, float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))


def _validate(cameras: list[TrackingCamera], data: WandData) -> np.ndarray:
    """Check that the data can be solved.

    Args:
        cameras: Cameras of the run.
        data: Wand observations.

    Returns:
        (K,) bool, True for frames whose point is unknown.

    Raises:
        ValueError: If fewer than 3 different reference points are known, a camera has
            no observation, or an unknown frame has fewer than 2 observations.
    """
    unknown = np.isnan(data.known_points_m).any(axis=1)
    # Many frames of the same point count once: two points leave a rotation open.
    if len(np.unique(data.known_points_m[~unknown], axis=0)) < MIN_KNOWN_FRAMES:
        raise ValueError(f"need at least {MIN_KNOWN_FRAMES} different known reference points")
    seen = np.bincount(data.camera_index, minlength=len(cameras))
    for i in np.flatnonzero(seen == 0):
        raise ValueError(f"camera {cameras[i].id} has no observations")
    per_frame = np.bincount(data.frame_index, minlength=len(unknown))
    bad = np.flatnonzero(unknown & (per_frame < MIN_OBSERVATIONS_PER_FRAME))
    if bad.size:
        raise ValueError(f"unknown wand frame {bad[0]} has fewer than {MIN_OBSERVATIONS_PER_FRAME} observations")
    return unknown


def _initial_points(cameras: list[TrackingCamera], data: WandData, unknown: np.ndarray) -> np.ndarray:
    """Triangulate every wand frame with the start poses.

    Args:
        cameras: Cameras with start poses.
        data: Wand observations.
        unknown: (K,) bool, frames to estimate.

    Returns:
        (K, 3) points in m: triangulated for unknown frames, known ones unchanged.
    """
    points = data.known_points_m.copy()
    rays = np.zeros((len(data.frame_index), 3))
    for c, cam in enumerate(cameras):
        sel = data.camera_index == c
        rays[sel] = pixel_to_ray(cam.pose, cam.model, data.pixels_px[sel])
    origins = np.array([cameras[c].pose.position_m for c in data.camera_index])
    for k in np.flatnonzero(unknown):
        sel = data.frame_index == k
        points[k] = closest_point_to_rays(origins[sel], rays[sel])
    return points


def _sparsity(data: WandData, n_cameras: int, slot: np.ndarray) -> lil_matrix:
    """Jacobian sparsity: an observation depends on its camera and its frame only.

    Args:
        data: Wand observations.
        n_cameras: Number of cameras.
        slot: (K,) index of each frame among the unknown frames, -1 if known.

    Returns:
        (2M, 6C + 3U) sparsity pattern.
    """
    n_obs = len(data.frame_index)
    pattern = lil_matrix((2 * n_obs, POSE_PARAMS * n_cameras + POINT_PARAMS * int((slot >= 0).sum())), dtype=int)
    for i, (c, k) in enumerate(zip(data.camera_index, data.frame_index)):
        cols = list(range(POSE_PARAMS * c, POSE_PARAMS * (c + 1)))
        if slot[k] >= 0:
            start = POSE_PARAMS * n_cameras + POINT_PARAMS * slot[k]
            cols += list(range(start, start + POINT_PARAMS))
        pattern[2 * i : 2 * i + 2, cols] = 1
    return pattern


def _residuals(
    x: np.ndarray,
    cameras: list[TrackingCamera],
    data: WandData,
    groups: list[np.ndarray],
    slot: np.ndarray,
    known_m: np.ndarray,
) -> np.ndarray:
    """Reprojection error (projected minus observed) of all observations.

    Args:
        x: Unknowns: 6 per camera, then 3 per unknown frame.
        cameras: Cameras with start poses.
        data: Wand observations.
        groups: Per camera, indices of its observations.
        slot: (K,) index among unknown frames, -1 if known.
        known_m: (K, 3) points with known ones filled in (others ignored).

    Returns:
        (2M,) residuals in px, ordered (u, v) per observation.
    """
    n_cam = len(cameras)
    points = known_m.copy()
    unknown_k = np.flatnonzero(slot >= 0)
    points[unknown_k] = x[POSE_PARAMS * n_cam :].reshape(-1, POINT_PARAMS)[slot[unknown_k]]
    out = np.zeros((len(data.frame_index), 2))
    for c, idx in enumerate(groups):
        pose = _pose_from(cameras[c].pose, x[POSE_PARAMS * c : POSE_PARAMS * (c + 1)])
        out[idx] = project(pose, cameras[c].model, points[data.frame_index[idx]]) - data.pixels_px[idx]
    return out.ravel()


def solve_cameras(cameras: list[TrackingCamera], data: WandData) -> tuple[list[TrackingCamera], np.ndarray]:
    """Estimate all camera poses and the unknown wand points by bundle adjustment.

    Args:
        cameras: Cameras with nominal start poses.
        data: Wand observations; known points fix the arena frame.

    Returns:
        The cameras with estimated poses (same order, ids, models) and the (K, 3)
        wand points in m (known ones unchanged).

    Raises:
        ValueError: If the data is insufficient (see `_validate`).
    """
    unknown = _validate(cameras, data)
    points0 = _initial_points(cameras, data, unknown)
    slot = np.full(len(unknown), -1)
    slot[unknown] = np.arange(int(unknown.sum()))
    groups = [np.flatnonzero(data.camera_index == c) for c in range(len(cameras))]
    x0 = np.concatenate([np.zeros(POSE_PARAMS * len(cameras)), points0[unknown].ravel()])
    result = least_squares(
        _residuals,
        x0,
        args=(cameras, data, groups, slot, data.known_points_m),
        jac_sparsity=_sparsity(data, len(cameras), slot),
        method="trf",
        x_scale="jac",
        xtol=SOLVER_TOLERANCE,
        ftol=SOLVER_TOLERANCE,
        gtol=SOLVER_TOLERANCE,
        max_nfev=MAX_EVALUATIONS,
    )
    solved = [
        TrackingCamera(cam.id, _pose_from(cam.pose, result.x[POSE_PARAMS * c : POSE_PARAMS * (c + 1)]), cam.model)
        for c, cam in enumerate(cameras)
    ]
    points = data.known_points_m.copy()
    points[unknown] = result.x[POSE_PARAMS * len(cameras) :].reshape(-1, POINT_PARAMS)
    return solved, points


def solve_camera_pose(camera: TrackingCamera, points_m: np.ndarray, pixels_px: np.ndarray) -> CameraPose:
    """Re-estimate one camera from known 3D points (for example wall markers).

    Args:
        camera: Camera whose current pose is the start.
        points_m: (N, 3) known points in the arena frame, m; N >= 4.
        pixels_px: (N, 2) their observed pixels.

    Returns:
        The estimated pose.

    Raises:
        ValueError: If fewer than 4 points are given.
    """
    if len(points_m) < MIN_MARKERS:
        raise ValueError(f"need at least {MIN_MARKERS} known points, got {len(points_m)}")

    def residuals(params: np.ndarray) -> np.ndarray:
        pose = _pose_from(camera.pose, params)
        return (project(pose, camera.model, points_m) - pixels_px).ravel()

    result = least_squares(
        residuals,
        np.zeros(POSE_PARAMS),
        x_scale="jac",
        xtol=SOLVER_TOLERANCE,
        ftol=SOLVER_TOLERANCE,
        gtol=SOLVER_TOLERANCE,
    )
    return _pose_from(camera.pose, result.x)


def marker_error_px(
    camera: TrackingCamera,
    seen: tuple[tuple[int, float, float], ...],
    marker_positions_m: dict[int, tuple[float, float, float]],
) -> tuple[float, np.ndarray, np.ndarray]:
    """Compare where a camera sees the wall markers with where it should see them.

    Args:
        camera: The camera with the pose the tracker currently uses.
        seen: Reported markers as (marker_id, u_px, v_px). Unknown ids are ignored.
        marker_positions_m: Marker id to known position, arena frame (m).

    Returns:
        The mean distance between seen and expected pixels (px; nan when no known marker
        was seen), the (N, 3) positions and the (N, 2) seen pixels of the markers used.
    """
    known = [entry for entry in seen if entry[0] in marker_positions_m]
    points_m = np.array([marker_positions_m[marker_id] for marker_id, _, _ in known])
    pixels_px = np.array([(u_px, v_px) for _, u_px, v_px in known])
    if not known:
        return float("nan"), points_m.reshape(0, 3), pixels_px.reshape(0, 2)
    expected_px = project(camera.pose, camera.model, points_m)
    error_px = float(np.mean(np.linalg.norm(expected_px - pixels_px, axis=1)))
    return error_px, points_m, pixels_px
