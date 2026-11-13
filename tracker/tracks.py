"""Track management for the tracking server.

Holds one constant-acceleration Kalman filter per bee, assigns the triangulated
points of each frame to tracks, and decides when a track is confirmed or removed.
It sits after triangulation and before the control interface: points come in,
tracks with stable ids go out. All times are simulation time supplied by the caller.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

from shared.config import TrackerSettings

SITTING = "sitting"
FLYING = "flying"
NEW_POINT = -1  # value in track_ids for a point that belongs to no track yet

_AXES = 3  # x, y, z
_STATE_DIM = 3 * _AXES  # position, velocity, acceleration per axis
_VELOCITY = slice(_AXES, 2 * _AXES)
_POSITION = slice(0, _AXES)
# Cost used for pairs outside the gate so the solver avoids them; they are dropped afterwards.
_FORBIDDEN_COST_M = 1e6


@dataclass
class Track:
    """The filtered state of one bee.

    Attributes:
        id: Unique track id, never reused.
        state: Shape (9,): [x, y, z, vx, vy, vz, ax, ay, az] in the arena frame,
            in m, m/s and m/s^2.
        covariance: Shape (9, 9) covariance of the state.
        t_s: Simulation time in s the state is valid for.
        confirmed: False while the track is on probation.
        hits: Consecutive frames with a supporting point.
        missed: Consecutive frames without a supporting point.
        motion: SITTING or FLYING.
        camera_count: Cameras that supported the last point (0 when missed).
    """

    id: int
    state: np.ndarray
    covariance: np.ndarray
    t_s: float
    confirmed: bool
    hits: int
    missed: int
    motion: str
    camera_count: int


@dataclass
class TrackSet:
    """All live tracks, ordered by id.

    Attributes:
        tracks: The live tracks.
        next_id: The id the next new track receives.
    """

    tracks: list[Track]
    next_id: int


def create_track_set() -> TrackSet:
    """Create an empty track set.

    Returns:
        A TrackSet with no tracks whose first id will be 0.
    """
    return TrackSet(tracks=[], next_id=0)


def _transition_matrix(dt_s: float) -> np.ndarray:
    """Build the 9x9 constant-acceleration transition matrix.

    Args:
        dt_s: Time step in s.

    Returns:
        Shape (9, 9) matrix F acting on [positions, velocities, accelerations].
    """
    per_axis = np.array([[1.0, dt_s, dt_s**2 / 2.0], [0.0, 1.0, dt_s], [0.0, 0.0, 1.0]])
    return np.kron(per_axis, np.eye(_AXES))


def _process_noise(dt_s: float, jerk_sigma_m_s3: float) -> np.ndarray:
    """Build the 9x9 process noise for a jerk that is constant over the step.

    This is the discrete white-noise jerk model (Bar-Shalom): Q = G G^T sigma^2
    with G = [dt^3/6, dt^2/2, dt]^T per axis.

    Args:
        dt_s: Time step in s.
        jerk_sigma_m_s3: Standard deviation of the jerk in m/s^3.

    Returns:
        Shape (9, 9) process noise covariance.
    """
    g = np.array([dt_s**3 / 6.0, dt_s**2 / 2.0, dt_s])
    return np.kron(np.outer(g, g), np.eye(_AXES)) * jerk_sigma_m_s3**2


def predict_tracks(track_set: TrackSet, t_s: float, settings: TrackerSettings) -> None:
    """Predict every track to time t_s, in place.

    Args:
        track_set: Tracks to predict.
        t_s: Target simulation time in s. Tracks already at or after t_s are left alone.
        settings: Tracker settings (uses jerk_sigma_m_s3).
    """
    for track in track_set.tracks:
        dt_s = t_s - track.t_s
        if dt_s <= 0.0:
            continue
        f = _transition_matrix(dt_s)
        q = _process_noise(dt_s, settings.jerk_sigma_m_s3)
        track.state = f @ track.state
        track.covariance = f @ track.covariance @ f.T + q
        track.t_s = t_s


def assign(predicted_m: np.ndarray, points_m: np.ndarray, gate_m: float) -> list[tuple[int, int]]:
    """Match tracks to points so the total Euclidean distance is smallest.

    Args:
        predicted_m: Shape (T, 3) predicted track positions in m, arena frame.
        points_m: Shape (P, 3) point positions in m, arena frame.
        gate_m: Largest allowed track-to-point distance in m.

    Returns:
        Pairs (track index, point index); pairs farther apart than gate_m are omitted.
    """
    if len(predicted_m) == 0 or len(points_m) == 0:
        return []
    distance_m = cdist(predicted_m, points_m)
    cost = np.where(distance_m <= gate_m, distance_m, _FORBIDDEN_COST_M)
    rows, cols = linear_sum_assignment(cost)
    return [(int(r), int(c)) for r, c in zip(rows, cols) if distance_m[r, c] <= gate_m]


def _new_track(track_id: int, position_m: np.ndarray, t_s: float, settings: TrackerSettings) -> Track:
    """Start a probationary track at a point.

    Args:
        track_id: Id of the new track.
        position_m: Shape (3,) point position in m, arena frame.
        t_s: Simulation time in s.
        settings: Tracker settings (initial uncertainties).

    Returns:
        A SITTING, unconfirmed track with zero velocity and acceleration.
    """
    state = np.zeros(_STATE_DIM)
    state[_POSITION] = position_m
    sigmas = np.repeat(
        [
            settings.measurement_sigma_m,
            settings.initial_speed_sigma_m_s,
            settings.initial_acceleration_sigma_m_s2,
        ],
        _AXES,
    )
    return Track(track_id, state, np.diag(sigmas**2), t_s, False, 1, 0, SITTING, 0)


def _kalman_update(track: Track, position_m: np.ndarray, settings: TrackerSettings) -> None:
    """Correct a track with a measured position, in place.

    Uses the Joseph form so the covariance stays symmetric and positive definite.

    Args:
        track: Track already predicted to the measurement time.
        position_m: Shape (3,) measured position in m, arena frame.
        settings: Tracker settings (measurement_sigma_m).
    """
    h = np.zeros((_AXES, _STATE_DIM))
    h[:, _POSITION] = np.eye(_AXES)
    r = np.eye(_AXES) * settings.measurement_sigma_m**2
    p = track.covariance
    gain = p @ h.T @ np.linalg.inv(h @ p @ h.T + r)
    track.state = track.state + gain @ (position_m - h @ track.state)
    i_kh = np.eye(_STATE_DIM) - gain @ h
    track.covariance = i_kh @ p @ i_kh.T + gain @ r @ gain.T


def _apply_point(track: Track, position_m: np.ndarray, camera_count: int, settings: TrackerSettings) -> None:
    """Update a track that received a point this frame.

    Args:
        track: Track predicted to the current time.
        position_m: Shape (3,) point position in m, arena frame.
        camera_count: Number of cameras that supported the point.
        settings: Tracker settings (measurement_sigma_m, confirm_frames).
    """
    _kalman_update(track, position_m, settings)
    track.hits += 1
    track.missed = 0
    track.camera_count = camera_count
    if track.hits >= settings.confirm_frames:
        track.confirmed = True


def _apply_miss(track: Track) -> None:
    """Record that a track got no point this frame; its state stays the prediction.

    Args:
        track: Track predicted to the current time.
    """
    track.missed += 1
    track.hits = 0
    track.camera_count = 0


def _update_motion(track: Track, settings: TrackerSettings) -> None:
    """Switch between SITTING and FLYING with hysteresis on the speed.

    Two thresholds stop a bee near the boundary from flickering between states.

    Args:
        track: Track whose motion label is updated.
        settings: Tracker settings (flying_above_m_s, sitting_below_m_s).
    """
    speed_m_s = float(np.linalg.norm(track.state[_VELOCITY]))
    if track.motion == SITTING and speed_m_s > settings.flying_above_m_s:
        track.motion = FLYING
    elif track.motion == FLYING and speed_m_s < settings.sitting_below_m_s:
        track.motion = SITTING


def _match_labelled(track_set: TrackSet, track_ids: np.ndarray) -> dict[int, int]:
    """Pair points that carry the id of an existing track with that track.

    Args:
        track_set: Current tracks.
        track_ids: Shape (P,) track id per point, or NEW_POINT.

    Returns:
        Map from track id to point index; the first point claiming a track wins.
    """
    live_ids = {track.id for track in track_set.tracks}
    matches: dict[int, int] = {}
    for point_index, track_id in enumerate(track_ids):
        if int(track_id) in live_ids:
            matches.setdefault(int(track_id), point_index)
    return matches


def _match_by_distance(
    tracks: list[Track], positions_m: np.ndarray, point_indices: list[int], gate_m: float
) -> dict[int, int]:
    """Assign leftover points to leftover tracks by distance.

    Args:
        tracks: Tracks without a labelled point.
        positions_m: Shape (P, 3) all point positions in m.
        point_indices: Indices of the points still free.
        gate_m: Largest allowed distance in m.

    Returns:
        Map from track id to point index.
    """
    predicted_m = np.array([track.state[_POSITION] for track in tracks]).reshape(-1, _AXES)
    pairs = assign(predicted_m, positions_m[point_indices], gate_m)
    return {tracks[t].id: point_indices[p] for t, p in pairs}


def _is_expired(track: Track, settings: TrackerSettings) -> bool:
    """Decide whether a track must be removed.

    Args:
        track: Track after this frame's update.
        settings: Tracker settings (max_missed_frames).

    Returns:
        True for an unconfirmed track that missed a frame, or a confirmed track
        that missed more than max_missed_frames in a row.
    """
    if track.missed == 0:
        return False
    return not track.confirmed or track.missed > settings.max_missed_frames


def update_tracks(
    track_set: TrackSet,
    positions_m: np.ndarray,
    camera_counts: np.ndarray,
    track_ids: np.ndarray,
    t_s: float,
    settings: TrackerSettings,
) -> None:
    """Fold one frame of points into the tracks, in place.

    Call after predict_tracks(track_set, t_s, settings).

    Args:
        track_set: Tracks, already predicted to t_s.
        positions_m: Shape (P, 3) point positions in m, arena frame.
        camera_counts: Shape (P,) cameras that supported each point.
        track_ids: Shape (P,) id of the track each point belongs to, or NEW_POINT.
        t_s: Simulation time in s of this frame.
        settings: Tracker settings.
    """
    matches = _match_labelled(track_set, track_ids)
    free_points = [i for i in range(len(positions_m)) if i not in matches.values()]
    free_tracks = [track for track in track_set.tracks if track.id not in matches]
    matches.update(_match_by_distance(free_tracks, positions_m, free_points, settings.assignment_gate_m))
    for track in track_set.tracks:
        if track.id in matches:
            index = matches[track.id]
            _apply_point(track, positions_m[index], int(camera_counts[index]), settings)
        else:
            _apply_miss(track)
    used = set(matches.values())
    for index in range(len(positions_m)):
        if index not in used:
            new = _new_track(track_set.next_id, positions_m[index], t_s, settings)
            new.camera_count = int(camera_counts[index])
            track_set.tracks.append(new)
            track_set.next_id += 1
    track_set.tracks = [t for t in track_set.tracks if not _is_expired(t, settings)]
    for track in track_set.tracks:
        _update_motion(track, settings)


def extrapolate(track: Track, t_s: float) -> np.ndarray:
    """Predict a track's position at a time without changing the track.

    Uses constant acceleration: p + v dt + a dt^2 / 2.

    Args:
        track: Track to extrapolate.
        t_s: Target simulation time in s.

    Returns:
        Shape (3,) position in m, arena frame.
    """
    dt_s = t_s - track.t_s
    p = track.state[_POSITION]
    v = track.state[_VELOCITY]
    a = track.state[2 * _AXES :]
    return p + v * dt_s + a * dt_s**2 / 2.0
