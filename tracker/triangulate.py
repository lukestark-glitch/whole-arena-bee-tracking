"""Triangulation of one frame of camera detections into 3D points.

Every detection (a pixel in one camera) becomes a ray in the arena frame (metres, right-handed,
z up). Points are found in two steps: a guided step that lets each existing track claim at most
one ray per camera around its predicted position, and a new-point step that searches the
rays left over for places where several cameras agree. Every ray supports at most one point,
which is what removes "ghost" crossings of rays from different bees.

The tracker (association and filtering) consumes the `Points` produced here. This module is
pure geometry: it uses no randomness, no wall clock and no ground truth.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from shared.config import Arena, CameraModel, Config, TrackerSettings
from shared.geometry import CameraPose, camera_pose, is_visible, pixel_to_ray

NEW_POINT = -1  # `track_ids` value of a point that was not found for an existing track.

# Cost given to a track/ray pair outside the claim gate. It only has to be larger than any gate
# so that the assignment avoids such pairs; pairs that still get chosen are filtered afterwards.
_OUT_OF_GATE_COST_M = 1e3

# Two rays with 1 - cos^2(angle) below this are treated as parallel (no closest approach).
_PARALLEL_EPS = 1e-9

# A point needs at least two rays; with one ray the least-squares system is singular.
_MIN_RAYS_FOR_POINT = 2

# Seeds of a real point cluster: a point seen by m cameras gives m (m - 1) / 2 seed midpoints,
# all within this many ray tolerances of each other. Isolated seeds are crossings of rays of
# different bees (ghosts) and are dropped before the expensive support search.
_CLUSTER_RADIUS_TOLERANCES = 1.0

# Seeds are processed in chunks of this size so that the (seeds x rays) distance matrices
# stay in the processor cache.
_SEED_CHUNK = 256

# Fit, re-select the support set around the fitted point, fit again (two fits in total).
_SUPPORT_ROUNDS = 2


@dataclass(frozen=True)
class TrackingCamera:
    """A fixed camera as the tracker sees it.

    Attributes:
        id: Camera id from the configuration.
        pose: Position and orientation in the arena frame.
        model: Intrinsic camera model (image size, field of view, range).
    """

    id: str
    pose: CameraPose
    model: CameraModel


@dataclass(frozen=True)
class Rays:
    """All detections of one frame as rays in the arena frame.

    Attributes:
        origins_m: Ray start points (camera positions), shape (N, 3), metres.
        directions: Unit direction vectors, shape (N, 3).
        camera_index: Index of the originating camera in the camera list, shape (N,).
    """

    origins_m: np.ndarray
    directions: np.ndarray
    camera_index: np.ndarray


@dataclass(frozen=True)
class Points:
    """Triangulated points of one frame.

    Attributes:
        positions_m: Point positions in the arena frame, shape (P, 3), metres.
        camera_counts: Number of cameras (rays) supporting each point, shape (P,).
        track_ids: Id of the track each point was found for, `NEW_POINT` for a new point,
            shape (P,).
    """

    positions_m: np.ndarray
    camera_counts: np.ndarray
    track_ids: np.ndarray


def tracking_cameras(config: Config) -> list[TrackingCamera]:
    """Build the camera list used for triangulation.

    Args:
        config: Loaded configuration.

    Returns:
        One `TrackingCamera` per configured camera, in configuration order.
    """
    return [
        TrackingCamera(
            cam.id,
            camera_pose(cam.position_m, cam.look_at_m),
            config.camera_models[cam.model],
        )
        for cam in config.cameras
    ]


def frame_rays(
    detections: dict[str, tuple[tuple[float, float, float], ...]],
    cameras: list[TrackingCamera],
) -> Rays:
    """Turn the detections of one frame into rays.

    Args:
        detections: Camera id -> entries (u_px, v_px, size_px). The size is not used here.
            Cameras missing from the dict contribute nothing.
        cameras: Camera list; a ray's `camera_index` indexes into it.

    Returns:
        All rays of the frame in arena-frame coordinates, grouped by camera.
    """
    origins, directions, indices = [], [], []
    for index, camera in enumerate(cameras):
        entries = detections.get(camera.id, ())
        if not entries:
            continue
        pixels_px = np.array([(u_px, v_px) for u_px, v_px, _ in entries], dtype=float)
        directions.append(pixel_to_ray(camera.pose, camera.model, pixels_px))
        origins.append(np.tile(camera.pose.position_m, (len(entries), 1)))
        indices.append(np.full(len(entries), index, dtype=int))
    if not indices:
        return _empty_rays()
    return Rays(np.concatenate(origins), np.concatenate(directions), np.concatenate(indices))


def closest_point_to_rays(origins_m: np.ndarray, directions: np.ndarray) -> np.ndarray:
    """Least-squares point closest to a set of lines.

    The squared distance of p to a line is |(I - d d^T)(p - o)|^2; setting the gradient of the
    sum to zero gives sum(I - d d^T) p = sum((I - d d^T) o).

    Args:
        origins_m: Line points, shape (N, 3), metres, arena frame.
        directions: Unit line directions, shape (N, 3).

    Returns:
        The point, shape (3,), metres, arena frame.

    Raises:
        numpy.linalg.LinAlgError: If the lines are all parallel (singular system).
    """
    along = np.einsum("ni,ni->n", directions, origins_m)
    normal = len(directions) * np.eye(3) - directions.T @ directions
    rhs = origins_m.sum(axis=0) - directions.T @ along
    return np.linalg.solve(normal, rhs)


def point_ray_distances(
    points_m: np.ndarray, origins_m: np.ndarray, directions: np.ndarray
) -> np.ndarray:
    """Perpendicular distance of every point to every ray.

    Uses |p - o|^2 - ((p - o) . d)^2 (Pythagoras) expanded into matrix products, which avoids
    a (P, N, 3) intermediate array.

    Args:
        points_m: Points, shape (P, 3), metres, arena frame.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit ray directions, shape (N, 3).

    Returns:
        Distances, shape (P, N), metres; `inf` where the point lies behind the ray origin.
    """
    along = points_m @ directions.T
    along -= np.einsum("ni,ni->n", origins_m, directions)
    squared = points_m @ origins_m.T
    squared *= -2.0
    squared += np.einsum("pi,pi->p", points_m, points_m)[:, None]
    squared += np.einsum("ni,ni->n", origins_m, origins_m)[None, :]
    squared -= along**2
    np.maximum(squared, 0.0, out=squared)
    distances = np.sqrt(squared, out=squared)
    distances[along <= 0.0] = np.inf
    return distances


def triangulate(
    rays: Rays,
    track_ids: np.ndarray,
    predicted_m: np.ndarray,
    cameras: list[TrackingCamera],
    arena: Arena,
    settings: TrackerSettings,
) -> Points:
    """Triangulate one frame: guided step for known tracks, then new points.

    Args:
        rays: Rays of the frame.
        track_ids: Ids of the existing tracks, shape (T,).
        predicted_m: Predicted track positions at this frame's time, shape (T, 3), metres.
        cameras: Camera list that `rays.camera_index` refers to.
        arena: Arena box (used to reject points outside it).
        settings: Tracker settings.

    Returns:
        Guided points first (in track order), then new points in acceptance order.
    """
    guided, used = guided_points(rays, track_ids, predicted_m, cameras, settings)
    new = new_points(rays, used, cameras, arena, settings)
    return Points(
        np.concatenate([guided.positions_m, new.positions_m]),
        np.concatenate([guided.camera_counts, new.camera_counts]),
        np.concatenate([guided.track_ids, new.track_ids]),
    )


def guided_points(
    rays: Rays,
    track_ids: np.ndarray,
    predicted_m: np.ndarray,
    cameras: list[TrackingCamera],
    settings: TrackerSettings,
) -> tuple[Points, np.ndarray]:
    """Find a point for each existing track from the rays near its prediction.

    A track whose prediction is off (a bee that turned sharply) still claims the few
    rays that happen to run along the error, and those alone give a poor point. Such a
    track therefore gets no point here unless the usual share of the cameras covering
    the place agrees; its rays stay free, the new-point step finds the bee with full
    support, and the track picks that point up by distance.

    Args:
        rays: Rays of the frame.
        track_ids: Ids of the existing tracks, shape (T,).
        predicted_m: Predicted track positions, shape (T, 3), metres, arena frame.
        cameras: Camera list that `rays.camera_index` refers to.
        settings: Tracker settings (claim gate, ray tolerance, minimum cameras).

    Returns:
        The guided points (in track order) and a boolean mask of shape (N,) of the rays used.
    """
    used = np.zeros(len(rays.camera_index), dtype=bool)
    if used.size == 0 or len(track_ids) == 0:
        return _empty_points(), used
    claim_track, claim_ray = _claim_rays(rays, predicted_m, settings.claim_gate_m)
    fits = []
    for row in range(len(track_ids)):
        claimed = claim_ray[claim_track == row]
        if len(claimed) < settings.min_cameras_tracked:
            continue
        fit = _fit_claimed(rays.origins_m[claimed], rays.directions[claimed], settings)
        if fit is not None:
            fits.append((track_ids[row], fit[0], claimed[fit[1]]))
    if not fits:
        return _empty_points(), used
    required = _required_cameras(
        np.array([point_m for _, point_m, _ in fits]),
        cameras,
        settings,
        settings.min_cameras_tracked,
    )
    kept = [fit for fit, needed in zip(fits, required) if len(fit[2]) >= needed]
    for _, _, ray_indices in kept:
        used[ray_indices] = True
    return (
        _make_points(
            [point_m for _, point_m, _ in kept],
            [len(ray_indices) for _, _, ray_indices in kept],
            [track_id for track_id, _, _ in kept],
        ),
        used,
    )


def new_points(
    rays: Rays,
    used: np.ndarray,
    cameras: list[TrackingCamera],
    arena: Arena,
    settings: TrackerSettings,
) -> Points:
    """Find new points among the rays that the guided step left unused.

    Args:
        rays: Rays of the frame.
        used: Boolean mask, shape (N,), of rays already used; these are ignored.
        cameras: Camera list that `rays.camera_index` refers to.
        arena: Arena box.
        settings: Tracker settings.

    Returns:
        New points (track id `NEW_POINT`) in acceptance order.
    """
    free = np.flatnonzero(~used)
    if len(free) < _MIN_RAYS_FOR_POINT:
        return _empty_points()
    origins, directions = rays.origins_m[free], rays.directions[free]
    camera_index = rays.camera_index[free]
    box = _arena_box(arena, settings)
    seeds = _pair_seeds(origins, directions, camera_index, settings.ray_tolerance_m, box)
    seeds = _thin_seeds(seeds, settings)
    if len(seeds) == 0:
        return _empty_points()
    candidates = _support_candidates(
        seeds,
        origins,
        directions,
        camera_index,
        len(cameras),
        settings.ray_tolerance_m,
        max(settings.min_cameras, _MIN_RAYS_FOR_POINT),
    )
    ordered = _order_candidates(*candidates, origins, directions, cameras, settings)
    return _accept_greedily(ordered, origins, directions, box, settings)


def _empty_rays() -> Rays:
    """Return a `Rays` without any ray."""
    return Rays(np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0, dtype=int))


def _empty_points() -> Points:
    """Return a `Points` without any point."""
    return _make_points([], [], [])


def _make_points(positions: list, counts: list, ids: list) -> Points:
    """Pack lists of point data into a `Points` with the right shapes and dtypes.

    Args:
        positions: Points, each of shape (3,), metres.
        counts: Supporting camera count per point.
        ids: Track id per point.

    Returns:
        The packed points; shapes (P, 3), (P,), (P,).
    """
    return Points(
        np.array(positions, dtype=float).reshape(-1, 3),
        np.array(counts, dtype=int),
        np.array(ids, dtype=int),
    )


def _arena_box(arena: Arena, settings: TrackerSettings) -> np.ndarray:
    """Return the arena box enlarged by the margin, so points at the wall are not rejected.

    Args:
        arena: Arena box.
        settings: Tracker settings (`arena_margin_m`).

    Returns:
        Array of shape (2, 3): lower and upper corner, metres, arena frame.
    """
    size_m = np.asarray(arena.size_m, dtype=float)
    return np.stack([np.full(3, -settings.arena_margin_m), size_m + settings.arena_margin_m])


def _inside(points_m: np.ndarray, box: np.ndarray) -> np.ndarray:
    """Test which points lie inside a box.

    Args:
        points_m: Points, shape (..., 3), metres.
        box: Lower and upper corner, shape (2, 3), metres.

    Returns:
        Boolean array of shape (...).
    """
    return np.all((points_m >= box[0]) & (points_m <= box[1]), axis=-1)


def _claim_rays(
    rays: Rays, predicted_m: np.ndarray, claim_gate_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """Let each prediction claim at most one ray per camera.

    Per camera the optimal one-to-one assignment (minimum total distance) is used, so that
    two neighbouring tracks cannot claim the same ray.

    Args:
        rays: Rays of the frame.
        predicted_m: Predicted positions, shape (T, 3), metres.
        claim_gate_m: Largest distance between a prediction and a claimed ray, metres.

    Returns:
        Arrays (track row, ray index) of the claims, both of shape (K,).
    """
    rows, claimed = [], []
    for camera in np.unique(rays.camera_index):
        ray_idx = np.flatnonzero(rays.camera_index == camera)
        distances = point_ray_distances(
            predicted_m, rays.origins_m[ray_idx], rays.directions[ray_idx]
        )
        in_gate = distances <= claim_gate_m
        cost = np.where(in_gate, distances, _OUT_OF_GATE_COST_M)
        track_row, ray_col = linear_sum_assignment(cost)
        ok = in_gate[track_row, ray_col]
        rows.append(track_row[ok])
        claimed.append(ray_idx[ray_col[ok]])
    return np.concatenate(rows), np.concatenate(claimed)


def _fit_claimed(
    origins_m: np.ndarray, directions: np.ndarray, settings: TrackerSettings
) -> tuple[np.ndarray, np.ndarray] | None:
    """Fit a point to the rays a track claimed, dropping the worst ray while it is an outlier.

    Args:
        origins_m: Claimed ray origins, shape (K, 3), metres.
        directions: Claimed unit directions, shape (K, 3).
        settings: Tracker settings.

    Returns:
        The point (3,) and the indices of the rays kept, or None if no consistent point with
        at least `min_cameras_tracked` rays exists.
    """
    kept = np.arange(len(directions))
    while len(kept) >= max(settings.min_cameras_tracked, _MIN_RAYS_FOR_POINT):
        point_m = closest_point_to_rays(origins_m[kept], directions[kept])
        distances = point_ray_distances(point_m[None], origins_m[kept], directions[kept])[0]
        worst = int(np.argmax(distances))
        if distances[worst] <= settings.ray_tolerance_m:
            return point_m, kept
        kept = np.delete(kept, worst)
    return None


def _pair_seeds(
    origins_m: np.ndarray,
    directions: np.ndarray,
    camera_index: np.ndarray,
    ray_tolerance_m: float,
    box: np.ndarray,
) -> np.ndarray:
    """Midpoints of the closest approach of all ray pairs from different cameras.

    Minimising |w + t1 d1 - t2 d2|^2 over the ray parameters t1 and t2 (w = o1 - o2, unit
    directions d1, d2) gives, with b = d1.d2, d = d1.w and e = d2.w,
    t1 = (b e - d) / (1 - b^2) and t2 = (e - b d) / (1 - b^2). All pairs are evaluated as
    (N, N) matrices built from matrix products. A pair is kept when the closest points lie in
    front of both origins, both rays are within tolerance of the midpoint (the closest
    points are at most twice the tolerance apart), and the midpoint is inside the box.

    Args:
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        camera_index: Camera of each ray, shape (N,).
        ray_tolerance_m: Largest allowed ray-to-point distance, metres.
        box: Enlarged arena box, shape (2, 3), metres.

    Returns:
        Seed points, shape (S, 3), metres.
    """
    b = directions @ directions.T
    d_dot_o = directions @ origins_m.T
    own = np.diag(d_dot_o)
    d = own[:, None] - d_dot_o
    e = d_dot_o.T - own[None, :]
    denominator = 1.0 - b * b
    parallel = denominator < _PARALLEL_EPS
    denominator = np.where(parallel, 1.0, denominator)
    t1 = (b * e - d) / denominator
    t2 = (e - b * d) / denominator
    squared_norm = np.einsum("ni,ni->n", origins_m, origins_m)
    w_squared = squared_norm[:, None] + squared_norm[None, :] - 2.0 * origins_m @ origins_m.T
    gap_squared = w_squared + t1**2 + t2**2 + 2.0 * t1 * d - 2.0 * t2 * e - 2.0 * t1 * t2 * b
    ok = (
        np.triu(camera_index[:, None] != camera_index[None, :], k=1)
        & ~parallel
        & (t1 > 0.0)
        & (t2 > 0.0)
        & (gap_squared <= (2.0 * ray_tolerance_m) ** 2)
    )
    first, second = np.nonzero(ok)
    mid_m = (
        origins_m[first]
        + t1[first, second, None] * directions[first]
        + origins_m[second]
        + t2[first, second, None] * directions[second]
    ) / 2.0
    return mid_m[_inside(mid_m, box)]


def _thin_seeds(seeds: np.ndarray, settings: TrackerSettings) -> np.ndarray:
    """Keep one seed per cluster cell and drop seeds that cannot belong to a real point.

    A point seen by `min_cameras` cameras yields at least min_cameras (min_cameras - 1) / 2
    seeds close together, so seeds with fewer close neighbours are ghosts. Of the rest, one
    seed per cell of one ray tolerance is enough, because the support search that follows
    re-centres every seed.

    Args:
        seeds: Seed points, shape (S, 3), metres.
        settings: Tracker settings.

    Returns:
        The remaining seeds, shape (K, 3), metres.
    """
    if len(seeds) < settings.min_cameras * (settings.min_cameras - 1) // 2:
        return seeds[:0]
    radius_m = _CLUSTER_RADIUS_TOLERANCES * settings.ray_tolerance_m
    needed = settings.min_cameras * (settings.min_cameras - 1) // 2
    nearest_m, _ = cKDTree(seeds).query(seeds, k=needed, workers=-1)
    kth_neighbour_m = nearest_m.reshape(len(seeds), -1)[:, -1]
    clustered = seeds[kth_neighbour_m <= radius_m]
    if len(clustered) == 0:
        return clustered
    cells = np.floor(clustered / settings.ray_tolerance_m).astype(np.int64)
    cells -= cells.min(axis=0)
    span = cells.max(axis=0) + 1
    keys = np.ravel_multi_index(tuple(cells.T), tuple(span))
    _, first = np.unique(keys, return_index=True)
    return clustered[first]


def _nearest_per_camera(
    points_m: np.ndarray,
    origins_m: np.ndarray,
    directions: np.ndarray,
    camera_index: np.ndarray,
    camera_count: int,
    ray_tolerance_m: float,
) -> np.ndarray:
    """For each point and camera, find the nearest ray within the tolerance.

    Points are processed in chunks so the (points x rays) distance matrix stays in cache.

    Args:
        points_m: Points, shape (S, 3), metres.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        camera_index: Camera of each ray, shape (N,).
        camera_count: Number of cameras.
        ray_tolerance_m: Largest allowed ray-to-point distance, metres.

    Returns:
        Ray index per point and camera, shape (S, camera_count); -1 where there is none.
    """
    chunks = [
        _nearest_chunk(
            points_m[start : start + _SEED_CHUNK],
            origins_m,
            directions,
            camera_index,
            camera_count,
            ray_tolerance_m,
        )
        for start in range(0, len(points_m), _SEED_CHUNK)
    ]
    return np.concatenate(chunks) if chunks else np.zeros((0, camera_count), dtype=int)


def _nearest_chunk(
    points_m: np.ndarray,
    origins_m: np.ndarray,
    directions: np.ndarray,
    camera_index: np.ndarray,
    camera_count: int,
    ray_tolerance_m: float,
) -> np.ndarray:
    """Nearest ray within tolerance per point and camera, for a chunk of points.

    Only the few (point, ray) pairs within tolerance are kept; sorting them by distance and
    taking the first per (point, camera) avoids a dense per-camera search.

    Args:
        points_m: Points, shape (S, 3), metres.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        camera_index: Camera of each ray, shape (N,).
        camera_count: Number of cameras.
        ray_tolerance_m: Largest allowed ray-to-point distance, metres.

    Returns:
        Ray index per point and camera, shape (S, camera_count); -1 where there is none.
    """
    distances = point_ray_distances(points_m, origins_m, directions)
    rows, rays = np.nonzero(distances <= ray_tolerance_m)
    order = np.lexsort((distances[rows, rays], camera_index[rays], rows))
    rows, rays = rows[order], rays[order]
    cameras = camera_index[rays]
    first = np.ones(len(rows), dtype=bool)
    first[1:] = (rows[1:] != rows[:-1]) | (cameras[1:] != cameras[:-1])
    support = np.full((len(points_m), camera_count), -1, dtype=int)
    support[rows[first], cameras[first]] = rays[first]
    return support


def _fit_supports(
    support: np.ndarray, origins_m: np.ndarray, directions: np.ndarray
) -> np.ndarray:
    """Least-squares point for each support set (batched `closest_point_to_rays`).

    Args:
        support: Ray index per candidate and camera, shape (S, C); -1 where there is none.
            Every row needs at least two rays from non-parallel lines.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).

    Returns:
        Points, shape (S, 3), metres.
    """
    weight = (support >= 0).astype(float)
    safe = np.maximum(support, 0)
    d, o = directions[safe], origins_m[safe]
    along = np.einsum("sci,sci->sc", d, o)
    normal = weight.sum(axis=1)[:, None, None] * np.eye(3) - np.einsum(
        "sc,sci,scj->sij", weight, d, d
    )
    rhs = np.einsum("sc,sci->si", weight, o - along[..., None] * d)
    return np.linalg.solve(normal, rhs[..., None])[..., 0]


def _support_candidates(
    seeds: np.ndarray,
    origins_m: np.ndarray,
    directions: np.ndarray,
    camera_index: np.ndarray,
    camera_count: int,
    ray_tolerance_m: float,
    min_cameras: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Grow each seed into a candidate point with a supporting ray per camera.

    The support set is chosen around the seed, a point is fitted, the support set is chosen
    again around that point (the seed is only a midpoint of two rays) and fitted once more.

    Args:
        seeds: Seed points, shape (S, 3), metres.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        camera_index: Camera of each ray, shape (N,).
        camera_count: Number of cameras.
        ray_tolerance_m: Largest allowed ray-to-point distance, metres.
        min_cameras: Candidates supported by fewer cameras are dropped (ghost seeds), after
            every round, to keep the later rounds cheap.

    Returns:
        Candidate points (K, 3) in metres and their support sets (K, camera_count), with
        identical support sets kept once.
    """
    points_m = seeds
    for _ in range(_SUPPORT_ROUNDS):
        support = _nearest_per_camera(
            points_m, origins_m, directions, camera_index, camera_count, ray_tolerance_m
        )
        support = support[(support >= 0).sum(axis=1) >= min_cameras]
        points_m = _fit_supports(support, origins_m, directions)
    support, first = np.unique(support, axis=0, return_index=True)
    return points_m[first], support


def _order_candidates(
    points_m: np.ndarray,
    support: np.ndarray,
    origins_m: np.ndarray,
    directions: np.ndarray,
    cameras: list[TrackingCamera],
    settings: TrackerSettings,
) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Sort candidates: most cameras first, ties by smallest mean ray distance.

    Candidates with fewer cameras than they require are dropped; removing used rays later can
    only make their support smaller.

    Args:
        points_m: Candidate points, shape (K, 3), metres.
        support: Support sets, shape (K, C), -1 where there is no ray.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        cameras: Camera list.
        settings: Tracker settings.

    Returns:
        List of (point, support row, required camera count) in the order they are tried.
    """
    counts = (support >= 0).sum(axis=1)
    safe = np.maximum(support, 0)
    offset = points_m[:, None, :] - origins_m[safe]
    along = np.einsum("kci,kci->kc", offset, directions[safe])
    perpendicular = np.linalg.norm(offset - along[..., None] * directions[safe], axis=2)
    mean_distance_m = np.where(support >= 0, perpendicular, 0.0).sum(axis=1) / counts
    required = _required_cameras(points_m, cameras, settings, settings.min_cameras)
    keep = np.flatnonzero(counts >= required)
    order = keep[np.lexsort((mean_distance_m[keep], -counts[keep]))]
    return [(points_m[k], support[k], int(required[k])) for k in order]


def _required_cameras(
    points_m: np.ndarray,
    cameras: list[TrackingCamera],
    settings: TrackerSettings,
    min_cameras: int,
) -> np.ndarray:
    """Number of cameras that must agree on each point: more where more cameras cover it.

    Visibility is evaluated at the candidate point; the small shift when rays are removed
    later (millimetres) does not change which cameras cover the place.

    Args:
        points_m: Candidate points, shape (K, 3), metres, arena frame.
        cameras: Camera list.
        settings: Tracker settings.
        min_cameras: Smallest number allowed, whatever the coverage.

    Returns:
        max(min_cameras, ceil(coverage_share * cameras that can see the point)), shape (K,).
    """
    visible = sum(is_visible(c.pose, c.model, points_m).astype(int) for c in cameras)
    shared = np.ceil(settings.coverage_share * visible).astype(int)
    return np.maximum(min_cameras, shared)


def _accept_greedily(
    ordered: list[tuple[np.ndarray, np.ndarray, int]],
    origins_m: np.ndarray,
    directions: np.ndarray,
    box: np.ndarray,
    settings: TrackerSettings,
) -> Points:
    """Accept candidates in order; every ray supports at most one accepted point.

    Args:
        ordered: Candidates as (point, support row, required cameras), best first.
        origins_m: Ray origins, shape (N, 3), metres.
        directions: Unit directions, shape (N, 3).
        box: Enlarged arena box, shape (2, 3), metres.
        settings: Tracker settings.

    Returns:
        The accepted points in acceptance order.
    """
    taken = np.zeros(len(directions), dtype=bool)
    positions, counts = [], []
    for point_m, support, required in ordered:
        rays = support[support >= 0]
        free = rays[~taken[rays]]
        if len(free) < max(required, _MIN_RAYS_FOR_POINT):
            continue
        if len(free) < len(rays):
            point_m = closest_point_to_rays(origins_m[free], directions[free])
        distances = point_ray_distances(point_m[None], origins_m[free], directions[free])[0]
        if distances.max() > settings.ray_tolerance_m or not _inside(point_m, box):
            continue
        taken[free] = True
        positions.append(point_m)
        counts.append(len(free))
    return _make_points(positions, counts, [NEW_POINT] * len(positions))
