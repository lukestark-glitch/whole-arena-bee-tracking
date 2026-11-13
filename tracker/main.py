"""Tracking server entry point: turns `detections` into tracks and writes the track log.

Run with `python -m tracker.main`. For every frame the receiver releases, the tracker
predicts all tracks to the frame's time, triangulates the detections (known tracks
first, then new points), updates the tracks and writes one JSON line to the log.
`process_frame` is pure (no socket, no clock) so tests can drive it directly; `run` adds
the UDP socket and the log file.

While a track is selected, every frame also yields a `target` message for the robot. Its
distance and the list of other bees in the robot's view are computed from the last
`robot_pose` the robot sent.

The tracker also serves the control window: a browser page that gets a `scene` message on
connect and `tracks` snapshots over a WebSocket, and answers with `select` messages that
say which track is selected. The message builders here are pure as well.
"""

import argparse
import asyncio
import json
import socket
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from websockets.asyncio.server import ServerConnection, broadcast, serve
from websockets.exceptions import ConnectionClosed

from shared.config import (
    Arena,
    Config,
    Robot,
    TrackerSettings,
    frame_rate_hz,
    last_frame_of,
    load_config,
    load_robot,
)
from shared.geometry import is_visible, project, robot_camera_pose
from shared.protocol import (
    METRE_DECIMALS,
    PIXEL_DECIMALS,
    RATE_DECIMALS,
    SECOND_DECIMALS,
    UDP_HOST,
    OtherBee,
    ProtocolError,
    RobotPose,
    Target,
    decode,
    encode_target,
)
from shared.web import page_response
from tracker.receiver import TIME_ROUNDING_S, Frame, FrameAssembler, frames_from_datagram
from tracker.tracks import (
    Track,
    TrackSet,
    create_track_set,
    extrapolate,
    predict_tracks,
    update_tracks,
)
from tracker.triangulate import (
    NO_POINT,
    Points,
    Rays,
    TrackingCamera,
    frame_rays,
    tracking_cameras,
    triangulate,
)

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
DEFAULT_ROBOT_PATH = Path("config/robot.yaml")
DEFAULT_LOG_PATH = Path("logs/tracks.jsonl")
# Room for about one second of detections, so a short stall of the tracker loses nothing.
RECEIVE_BUFFER_BYTES = 4 * 1024 * 1024
# The state vector holds position, velocity and acceleration, three values each.
_AXES = 3
PAGE_PATH = Path(__file__).parent / "web" / "index.html"
# The control window gets this many snapshots per second of simulation time (the tracker
# may process more frames; a browser cannot draw more).
SNAPSHOT_RATE_HZ = 30
# `track_id` of a detection that no track used.
UNUSED_DETECTION = -1
# Most other bees listed in one `target`. Eight entries keep the datagram well below the
# 1400 bytes a `target` may have; the bees nearest to the target in the image are kept.
MAX_OTHERS = 8


@dataclass
class Tracker:
    """Everything the tracking server keeps between frames.

    Attributes:
        cameras: Tracking cameras with pose and lens model, in config order.
        arena: Arena box, arena frame, metres.
        settings: Tuning values.
        track_set: The current tracks.
        selected_id: Id of the track selected in the control window, or None.
        last_frame: Number of the last processed frame, or None before the first.
        detections: Camera id to `[u_px, v_px, size_px, track_id]` per detection of the
            last frame; `track_id` is -1 for a detection no track used.
        robot: Robot description (geometry and stereo camera), for `target`.
        auto_select: Select the oldest confirmed track whenever none is selected, for
            runs without a control window.
        robot_pose: The newest `robot_pose`, or None before the first.
    """

    cameras: list[TrackingCamera]
    arena: Arena
    settings: TrackerSettings
    track_set: TrackSet
    robot: Robot
    auto_select: bool = False
    robot_pose: RobotPose | None = None
    selected_id: int | None = None
    last_frame: int | None = None
    detections: dict[str, list[list[float]]] = field(default_factory=dict)


def create_tracker(config: Config, robot: Robot, auto_select: bool = False) -> Tracker:
    """Build a tracker without tracks from the configuration.

    Args:
        config: Configuration. Camera poses are the nominal ones from the file.
        robot: Robot description.
        auto_select: Select the oldest confirmed track whenever none is selected.

    Returns:
        The tracker.
    """
    return Tracker(
        tracking_cameras(config), config.arena, config.tracker, create_track_set(), robot,
        auto_select,
    )


def _rounded(values: np.ndarray, decimals: int) -> list[float]:
    """Round a vector for the log.

    Args:
        values: Vector, shape (3,).
        decimals: Decimals to keep.

    Returns:
        The rounded components as plain floats.
    """
    return [round(float(value), decimals) for value in values]


def _track_record(track: Track) -> dict[str, Any]:
    """Describe one track for the log.

    Args:
        track: The track; state in the arena frame (m, m/s, m/s^2).

    Returns:
        JSON-compatible fields of the track.
    """
    return {
        "id": track.id,
        "pos": _rounded(track.state[:_AXES], METRE_DECIMALS),
        "vel": _rounded(track.state[_AXES : 2 * _AXES], RATE_DECIMALS),
        "acc": _rounded(track.state[2 * _AXES :], RATE_DECIMALS),
        "state": track.motion,
        "confirmed": track.confirmed,
        "cameras": track.camera_count,
    }


def _detection_records(
    frame: Frame,
    rays: Rays,
    points: Points,
    point_track_ids: np.ndarray,
    cameras: list[TrackingCamera],
) -> dict[str, list[list[float]]]:
    """Label every detection of a frame with the track that used it.

    Args:
        frame: The frame; detections as (u_px, v_px, size_px).
        rays: Rays of the frame (one per detection).
        points: Points of the frame; `ray_point` says which point each ray supports.
        point_track_ids: Track id per point, shape (P,).
        cameras: Camera list that `rays.camera_index` refers to.

    Returns:
        Camera id to `[u_px, v_px, size_px, track_id]` per detection, `UNUSED_DETECTION` as
        id where no track used the detection.
    """
    records = {
        camera_id: [[u_px, v_px, size_px, UNUSED_DETECTION] for u_px, v_px, size_px in entries]
        for camera_id, entries in frame.detections.items()
    }
    for ray, point in enumerate(points.ray_point):
        if point != NO_POINT:
            camera_id = cameras[rays.camera_index[ray]].id
            records[camera_id][rays.detection_index[ray]][3] = int(point_track_ids[point])
    return records


def _restart_tracks(tracker: Tracker) -> None:
    """Drop all tracks and the selection after the sender restarted; ids keep counting up.

    Args:
        tracker: Tracker, modified in place.
    """
    tracker.track_set = TrackSet(tracks=[], next_id=tracker.track_set.next_id)
    tracker.selected_id = None
    # Poses of the old run carry later times and would shadow those of the new run.
    tracker.robot_pose = None


def _update_selection(tracker: Tracker) -> None:
    """Clear the selection of a removed track; pick one when `auto_select` is on.

    Args:
        tracker: Tracker, modified in place.
    """
    tracks = tracker.track_set.tracks
    if tracker.selected_id not in {track.id for track in tracks}:
        tracker.selected_id = None
    confirmed = [track.id for track in tracks if track.confirmed]
    if tracker.auto_select and tracker.selected_id is None and confirmed:
        tracker.selected_id = min(confirmed)


def process_frame(tracker: Tracker, frame: Frame) -> dict[str, Any]:
    """Advance the tracker by one frame of detections.

    A frame number lower than the previous one means the sender restarted: the tracks and
    the selection are dropped first.

    Args:
        tracker: Tracker, modified in place.
        frame: Detections of one frame with its exposure time (simulation s).

    Returns:
        The log record of the frame: frame number, time, selected track id and all tracks.
    """
    if tracker.last_frame is not None and frame.frame < tracker.last_frame:
        _restart_tracks(tracker)
    tracker.last_frame = frame.frame
    tracks = tracker.track_set
    predict_tracks(tracks, frame.t_s, tracker.settings)
    track_ids = np.array([track.id for track in tracks.tracks], dtype=int)
    predicted_m = np.array([track.state[:_AXES] for track in tracks.tracks]).reshape(-1, _AXES)
    rays = frame_rays(frame.detections, tracker.cameras)
    points = triangulate(
        rays, track_ids, predicted_m, tracker.cameras, tracker.arena, tracker.settings
    )
    point_track_ids = update_tracks(
        tracks,
        points.positions_m,
        points.camera_counts,
        points.track_ids,
        frame.t_s,
        tracker.settings,
    )
    tracker.detections = _detection_records(frame, rays, points, point_track_ids, tracker.cameras)
    _update_selection(tracker)
    return {
        "frame": frame.frame,
        "t": round(frame.t_s, SECOND_DECIMALS),
        "selected": tracker.selected_id,
        "tracks": [_track_record(track) for track in tracks.tracks],
    }


def _confidence(tracker: Tracker, track: Track) -> float:
    """Share of the cameras covering the track's position that confirm it in this frame.

    Args:
        tracker: Tracker (tracking cameras).
        track: The track; position in the arena frame (m).

    Returns:
        A value from 0 to 1; 0 when no camera covers the position.
    """
    position_m = track.state[:_AXES].reshape(1, _AXES)
    expected = sum(
        bool(is_visible(camera.pose, camera.model, position_m)[0]) for camera in tracker.cameras
    )
    return min(1.0, track.camera_count / expected) if expected else 0.0


def _others_in_view(tracker: Tracker, selected: Track, pose: RobotPose) -> tuple[OtherBee, ...]:
    """List the other confirmed bees the robot's stereo camera should see.

    Args:
        tracker: Tracker (tracks and robot description).
        selected: The selected track.
        pose: The newest robot pose.

    Returns:
        At most MAX_OTHERS bees, those nearest to the selected bee in the stereo image first.
    """
    others = [t for t in tracker.track_set.tracks if t.confirmed and t.id != selected.id]
    if not others:
        return ()
    camera = robot_camera_pose(tracker.robot, pose.rail_m, pose.tilt_rad)
    model = tracker.robot.stereo.model
    positions_m = np.array([track.state[:_AXES] for track in [selected] + others])
    pixels_px = project(camera, model, positions_m)
    visible = is_visible(camera, model, positions_m)
    offsets_px = np.linalg.norm(pixels_px[1:] - pixels_px[0], axis=1)
    order = [i for i in np.argsort(offsets_px, kind="stable") if visible[i + 1]]
    return tuple(
        OtherBee(
            others[i].id,
            tuple(float(x) for x in others[i].state[:_AXES]),
            tuple(float(x) for x in others[i].state[_AXES : 2 * _AXES]),
        )
        for i in order[:MAX_OTHERS]
    )


def target_message(tracker: Tracker) -> Target | None:
    """Build the `target` message for the selected track.

    Args:
        tracker: Tracker after `process_frame`.

    Returns:
        The message, or None when no track is selected or no `robot_pose` has arrived yet
        (the distance cannot be computed without one).
    """
    pose = tracker.robot_pose
    selected = [t for t in tracker.track_set.tracks if t.id == tracker.selected_id]
    if not selected or pose is None:
        return None
    track = selected[0]
    lens_m = robot_camera_pose(tracker.robot, pose.rail_m, pose.tilt_rad).position_m
    return Target(
        t=track.t_s,
        track_id=track.id,
        pos=tuple(float(x) for x in track.state[:_AXES]),
        vel=tuple(float(x) for x in track.state[_AXES : 2 * _AXES]),
        # The smoothed acceleration: the raw filter value is too noisy to extrapolate with.
        acc=tuple(float(x) for x in track.smoothed_acceleration),
        distance_m=float(np.linalg.norm(track.state[:_AXES] - lens_m)),
        state=track.motion,
        confidence=_confidence(tracker, track),
        others=_others_in_view(tracker, track, pose),
    )


def apply_robot_pose(tracker: Tracker, datagram: bytes) -> None:
    """Keep the newest `robot_pose`; anything else in the datagram is ignored.

    Args:
        tracker: Tracker, modified in place.
        datagram: One UDP datagram from the robot.
    """
    try:
        message = decode(datagram)
    except ProtocolError:
        return
    newest = tracker.robot_pose
    # UDP may reorder; the protocol orders messages by t, not by arrival.
    if isinstance(message, RobotPose) and (newest is None or message.t > newest.t):
        tracker.robot_pose = message


def scene_message(config: Config) -> str:
    """Build the `scene` message the control window gets on connect.

    Args:
        config: Configuration; positions in metres, arena frame.

    Returns:
        The JSON text with arena size, prediction lead, cameras, nodes and the port on
        which the simulator streams the cinema view.
    """
    cameras = [
        {
            "id": camera.id,
            "position_m": list(camera.position_m),
            "look_at_m": list(camera.look_at_m),
            "fov_horizontal_rad": config.camera_models[camera.model].fov_horizontal_rad,
            "width_px": config.camera_models[camera.model].width_px,
            "height_px": config.camera_models[camera.model].height_px,
        }
        for camera in config.cameras
    ]
    nodes = [{"id": node.id, "cameras": list(node.camera_ids)} for node in config.nodes]
    return json.dumps(
        {
            "type": "scene",
            "arena_size_m": list(config.arena.size_m),
            "prediction_lead_s": config.tracker.prediction_lead_s,
            "min_cameras": config.tracker.min_cameras,
            "cinema_port": config.ports.simulator_window,
            "cameras": cameras,
            "nodes": nodes,
        },
        separators=(",", ":"),
    )


def _snapshot_track(track: Track, t_s: float, lead_s: float) -> dict[str, Any]:
    """Describe one track for the control window.

    Args:
        track: The track, predicted to the frame time.
        t_s: Frame time, simulation s.
        lead_s: How far ahead `pred` lies, in s.

    Returns:
        The log fields of the track, with the smoothed acceleration as `acc`, plus `pred`,
        the position `lead_s` after `t_s` in m.
    """
    record = _track_record(track)
    record["acc"] = _rounded(track.smoothed_acceleration, RATE_DECIMALS)
    record["pred"] = _rounded(extrapolate(track, t_s + lead_s), METRE_DECIMALS)
    return record


def _health(config: Config, t_s: float, last_report_s: dict[str, float]) -> dict[str, Any]:
    """Tell which cameras and nodes have gone quiet.

    Args:
        config: Configuration (cameras, nodes, silence limit).
        t_s: Frame time, simulation s.
        last_report_s: Camera id to time of its newest message, simulation s.

    Returns:
        `camera_age_s` (time since each camera reported; a camera never heard is as old as
        `t_s`) and `silent_nodes` (nodes whose cameras are all at least the limit old).
    """
    # A camera never heard counts as last heard at time zero, so its age is the frame time.
    # A frame released by timeout is older than the newest report, hence the lower bound.
    age_s = {
        camera.id: max(0.0, t_s - last_report_s.get(camera.id, 0.0)) for camera in config.cameras
    }
    limit_s = config.tracker.node_silent_after_s
    silent = [
        node.id
        for node in config.nodes
        if all(age_s[camera_id] >= limit_s for camera_id in node.camera_ids)
    ]
    return {
        "silent_nodes": silent,
        "camera_age_s": {c: round(age, SECOND_DECIMALS) for c, age in age_s.items()},
    }


def tracks_message(
    tracker: Tracker, frame: Frame, config: Config, last_report_s: dict[str, float]
) -> str:
    """Build the `tracks` snapshot sent to the control window after a frame.

    Args:
        tracker: Tracker after `process_frame(tracker, frame)`.
        frame: The frame just processed.
        config: Configuration.
        last_report_s: Camera id to the time of its newest message, simulation s.

    Returns:
        The JSON text with all tracks, the labelled detections, the selection and health.
    """
    lead_s = config.tracker.prediction_lead_s
    detections = {
        camera_id: [
            [round(u, PIXEL_DECIMALS), round(v, PIXEL_DECIMALS), round(size, PIXEL_DECIMALS), track_id]
            for u, v, size, track_id in entries
        ]
        for camera_id, entries in tracker.detections.items()
    }
    return json.dumps(
        {
            "type": "tracks",
            "frame": frame.frame,
            "t": round(frame.t_s, SECOND_DECIMALS),
            "selected": tracker.selected_id,
            "tracks": [
                _snapshot_track(track, frame.t_s, lead_s) for track in tracker.track_set.tracks
            ],
            "detections": detections,
            "health": _health(config, frame.t_s, last_report_s),
        },
        separators=(",", ":"),
    )


def apply_selection(tracker: Tracker, raw: str | bytes) -> None:
    """Apply a `select` message from the control window.

    Args:
        tracker: Tracker whose `selected_id` is set.
        raw: JSON text `{"type":"select","track_id":4}`; `track_id` may be null.

    Raises:
        ValueError: If the text is not valid JSON, not a select message, or the id is not an
            integer (a bool does not count) of an existing track.
    """
    message = json.loads(raw)
    if not isinstance(message, dict) or message.get("type") != "select":
        raise ValueError("not a select message")
    if "track_id" not in message:
        raise ValueError("select message without track_id")
    track_id = message["track_id"]
    if track_id is not None:
        if isinstance(track_id, bool) or not isinstance(track_id, int):
            raise ValueError("track_id must be an integer or null")
        if track_id not in {track.id for track in tracker.track_set.tracks}:
            raise ValueError(f"no track with id {track_id}")
    tracker.selected_id = track_id


@dataclass
class _Session:
    """State of one `run`: the tracker, its inputs and the open control windows.

    Attributes:
        config: Configuration.
        tracker: The tracker.
        assembler: Groups detections messages into frames.
        log: Open track log file.
        target_socket: UDP socket the `target` messages leave from.
        duration_s: Simulation duration to run for, or None.
        connections: Open control window connections.
        done: Set when the last frame of the duration was processed.
        last_sent: The frame last broadcast as (frame number, time in s), or None.
        last_handled: The frame processed last, or None before the first frame.
    """

    config: Config
    tracker: Tracker
    assembler: FrameAssembler
    log: Any
    target_socket: socket.socket
    duration_s: float | None
    connections: set[ServerConnection] = field(default_factory=set)
    done: asyncio.Event = field(default_factory=asyncio.Event)
    last_sent: tuple[int, float] | None = None
    last_handled: Frame | None = None


def _snapshot_due(session: _Session, frame: Frame) -> bool:
    """Decide whether this frame is broadcast to the control window.

    Args:
        session: The running session.
        frame: The frame just processed.

    Returns:
        True when at least 1 / SNAPSHOT_RATE_HZ of simulation time has passed since the last
        broadcast (less a rounding slack), or the frame number went backwards.
    """
    if session.last_sent is None:
        return True
    last_frame, last_t_s = session.last_sent
    period_s = 1.0 / SNAPSHOT_RATE_HZ - TIME_ROUNDING_S
    return frame.frame < last_frame or frame.t_s - last_t_s >= period_s


def _snapshot(session: _Session, frame: Frame) -> str:
    """Build the `tracks` message for a frame from the session's current state.

    Args:
        session: The running session.
        frame: The frame the tracker processed last.

    Returns:
        The message as JSON text.
    """
    return tracks_message(
        session.tracker, frame, session.config, session.assembler.last_report_s()
    )


def _handle_frame(session: _Session, frame: Frame) -> None:
    """Track one frame, log it, send `target` and broadcast a snapshot when it is due.

    Args:
        session: The running session.
        frame: A frame released by the assembler.
    """
    record = process_frame(session.tracker, frame)
    session.log.write(json.dumps(record, separators=(",", ":")) + "\n")
    # Each line is flushed so that the metrics can read the log while it grows.
    session.log.flush()
    target = target_message(session.tracker)
    if target is not None:
        session.target_socket.sendto(
            encode_target(target), (UDP_HOST, session.config.ports.target)
        )
    session.last_handled = frame
    if _snapshot_due(session, frame):
        broadcast(session.connections, _snapshot(session, frame))
        session.last_sent = (frame.frame, frame.t_s)
    if session.duration_s is not None and frame.frame >= last_frame_of(
        session.config, session.duration_s
    ):
        session.done.set()


def _resend_snapshot(session: _Session) -> None:
    """Broadcast the last frame again, so a window sees a change without new detections.

    While the simulation is paused no frame arrives; a new window or a new selection
    would otherwise show nothing until it continues.

    Args:
        session: The running session.
    """
    if session.last_handled is not None:
        broadcast(session.connections, _snapshot(session, session.last_handled))


class _DetectionsProtocol(asyncio.DatagramProtocol):
    """Receives detections datagrams on the event loop and tracks the frames in them."""

    def __init__(self, session: _Session) -> None:
        """Create the protocol.

        Args:
            session: The running session.
        """
        self._session = session

    def datagram_received(self, data: bytes, addr: Any) -> None:
        """Handle one datagram: assemble frames and track each released frame.

        Args:
            data: The datagram bytes.
            addr: Sender address (unused).
        """
        for frame in frames_from_datagram(self._session.assembler, data):
            _handle_frame(self._session, frame)


class _RobotPoseProtocol(asyncio.DatagramProtocol):
    """Receives `robot_pose` datagrams on the event loop."""

    def __init__(self, session: _Session) -> None:
        """Create the protocol.

        Args:
            session: The running session.
        """
        self._session = session

    def datagram_received(self, data: bytes, addr: Any) -> None:
        """Hand one datagram to the tracker.

        Args:
            data: The datagram bytes.
            addr: Sender address (unused).
        """
        apply_robot_pose(self._session.tracker, data)


async def _control_window(session: _Session, connection: ServerConnection) -> None:
    """Serve one control window: send the scene, then apply its `select` messages.

    Args:
        session: The running session.
        connection: The WebSocket connection of the window.
    """
    try:
        await connection.send(scene_message(session.config))
        session.connections.add(connection)
        _resend_snapshot(session)
        async for raw in connection:
            before = session.tracker.selected_id
            try:
                apply_selection(session.tracker, raw)
            except (ValueError, RecursionError) as error:
                # Deeply nested JSON ends in a RecursionError, not a ValueError.
                print(f"ignored control window message: {error}", file=sys.stderr)
            # Only a real change is worth a snapshot; a flood of repeats must not keep the
            # loop from reading detections.
            if session.tracker.selected_id != before:
                _resend_snapshot(session)
    except ConnectionClosed:
        pass
    finally:
        session.connections.discard(connection)


def _receive_socket(port: int) -> socket.socket:
    """Open a UDP socket for incoming messages.

    Args:
        port: UDP port to bind on the local host.

    Returns:
        The bound socket with a large receive buffer.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
    sock.bind((UDP_HOST, port))
    return sock


async def _serve(session: _Session) -> None:
    """Receive detections and robot poses and serve the control window until the run is over.

    Args:
        session: The running session.
    """
    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        lambda: _DetectionsProtocol(session),
        sock=_receive_socket(session.config.ports.detections),
    )
    pose_transport, _ = await loop.create_datagram_endpoint(
        lambda: _RobotPoseProtocol(session),
        sock=_receive_socket(session.config.ports.robot_pose),
    )
    try:
        async with serve(
            lambda connection: _control_window(session, connection),
            UDP_HOST,
            session.config.ports.control_window,
            process_request=lambda connection, request: page_response(
                connection, request, PAGE_PATH
            ),
        ):
            await session.done.wait()
    finally:
        transport.close()
        pose_transport.close()


def run(
    config: Config,
    robot: Robot,
    log_path: Path,
    duration_s: float | None,
    auto_select: bool,
) -> None:
    """Receive detections over UDP, write the track log, send `target`, serve the window.

    Args:
        config: Configuration.
        robot: Robot description.
        log_path: File that gets one JSON line per processed frame.
        duration_s: Stop after the last frame before this simulation time (s), or None
            to run until interrupted.
        auto_select: Select the oldest confirmed track whenever none is selected.
    """
    rate_hz = frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames / rate_hz
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)
    tracker = create_tracker(config, robot, auto_select)
    with (
        open(log_path, "w", encoding="utf-8") as log,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as target_socket,
    ):
        session = _Session(config, tracker, assembler, log, target_socket, duration_s)
        asyncio.run(_serve(session))


def main() -> None:
    """Parse the command line and run the tracker."""
    parser = argparse.ArgumentParser(description="Bee tracking server")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--robot", type=Path, default=DEFAULT_ROBOT_PATH, help="robot file")
    parser.add_argument(
        "--auto-select",
        action="store_true",
        help="select the oldest confirmed track whenever none is selected",
    )
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG_PATH, help="track log file")
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    args = parser.parse_args()
    try:
        run(
            load_config(args.config), load_robot(args.robot), args.log, args.duration,
            args.auto_select,
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
