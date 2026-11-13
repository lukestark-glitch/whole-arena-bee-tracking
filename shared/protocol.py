"""Build and parse the UDP messages described in docs/protocol.md.

This is the only module that builds or parses messages. The simulator, the robot
emulator, the tracker and the metrics code all use it, so the wire format lives in
exactly one place.

Each message is one JSON object in one UTF-8 datagram, with a ``"type"`` and a
``"v"`` field. Units are metres, seconds and radians; positions are in the arena
frame (right-handed, x along the long side, y away from the robot, z up). Image
coordinates are pixels with the origin top-left, u to the right and v down.
"""

import json
from dataclasses import dataclass
from typing import Any

# Version written to and required in every message.
PROTOCOL_VERSION = 1

# A datagram must stay below this many bytes so that it fits one Ethernet frame
# (1500 byte MTU minus IP and UDP headers, with margin) and is never fragmented.
MAX_DATAGRAM_BYTES = 1400

# docs/protocol.md fixes the transport to UDP on localhost.
UDP_HOST = "127.0.0.1"

# Sender rounding keeps datagrams small; sub-pixel and sub-0.1 mm detail is noise.
PIXEL_DECIMALS = 2  # u, v and blob size, in pixels
METRE_DECIMALS = 4  # positions and distances, in metres
SECOND_DECIMALS = 6  # simulation time, in seconds
RATE_DECIMALS = 4  # velocity (m/s), acceleration (m/s^2) and tilt (rad)
CONFIDENCE_DECIMALS = 2  # dimensionless, 0 to 1

Vec3 = tuple[float, float, float]


class ProtocolError(ValueError):
    """Raised when a message cannot be encoded or a datagram cannot be decoded."""


@dataclass(frozen=True)
class Detections:
    """Image points seen by one camera in one frame.

    Attributes:
        node_id: Camera node that sent the message.
        camera_id: Camera that produced the points.
        frame: Frame counter of the camera.
        t: Simulation time of the frame, in seconds.
        bees: Bee blobs as (u_px, v_px, size_px); origin top-left, u right, v down.
        markers: Marker points as (marker_id, u_px, v_px).
    """

    node_id: str
    camera_id: str
    frame: int
    t: float
    bees: tuple[tuple[float, float, float], ...]
    markers: tuple[tuple[int, float, float], ...] = ()


@dataclass(frozen=True)
class OtherBee:
    """A tracked bee other than the target.

    Attributes:
        track_id: Track identifier.
        pos: Position in the arena frame, in metres.
        vel: Velocity in the arena frame, in m/s.
    """

    track_id: int
    pos: Vec3
    vel: Vec3


@dataclass(frozen=True)
class Target:
    """Where the selected bee is, sent from the tracker to the robot.

    Attributes:
        t: Simulation time, in seconds.
        track_id: Track identifier of the target bee.
        pos: Position in the arena frame, in metres.
        vel: Velocity in the arena frame, in m/s.
        acc: Acceleration in the arena frame, in m/s^2.
        distance_m: Distance from the robot camera to the bee, in metres.
        state: "sitting" or "flying".
        confidence: Track confidence, 0 to 1.
        others: Other tracked bees expected in the robot's view.
    """

    t: float
    track_id: int
    pos: Vec3
    vel: Vec3
    acc: Vec3
    distance_m: float
    state: str
    confidence: float
    others: tuple[OtherBee, ...]


@dataclass(frozen=True)
class RobotPose:
    """Measured robot axes, sent from the robot to the tracker.

    Attributes:
        t: Simulation time, in seconds.
        rail_m: Position along the rail, in metres.
        tilt_rad: Camera tilt, in radians.
        focus_m: Focus distance, in metres.
    """

    t: float
    rail_m: float
    tilt_rad: float
    focus_m: float


@dataclass(frozen=True)
class TruthBee:
    """Ground-truth state of one bee.

    Attributes:
        id: Bee identifier.
        pos: Position in the arena frame, in metres.
        vel: Velocity in the arena frame, in m/s.
        state: "sitting", "walking" or "flying".
    """

    id: int
    pos: Vec3
    vel: Vec3
    state: str


@dataclass(frozen=True)
class RobotAxes:
    """Ground-truth robot axes.

    Attributes:
        rail_m: Position along the rail, in metres.
        tilt_rad: Camera tilt, in radians.
        focus_m: Focus distance, in metres.
    """

    rail_m: float
    tilt_rad: float
    focus_m: float


@dataclass(frozen=True)
class Truth:
    """Ground truth of one frame; only the metrics code may read it.

    Attributes:
        frame: Frame counter.
        t: Simulation time, in seconds.
        bees: Ground-truth bees.
        robot: Ground-truth robot axes, or None when not reported.
    """

    frame: int
    t: float
    bees: tuple[TruthBee, ...]
    robot: RobotAxes | None = None


Message = Detections | Target | RobotPose | Truth


def _vec(values: Vec3, decimals: int) -> list[float]:
    """Round a 3-vector for the wire.

    Args:
        values: Vector in any unit.
        decimals: Number of decimals to keep.

    Returns:
        The rounded components as a list.
    """
    return [round(value, decimals) for value in values]


def _dumps(fields: Any) -> bytes:
    """Serialise a message dictionary, or a part of one, as compact UTF-8 JSON.

    Args:
        fields: JSON-compatible message fields or a single list entry.

    Returns:
        The datagram payload.
    """
    return json.dumps(fields, separators=(",", ":")).encode("utf-8")


def _fits(fields: dict[str, Any], lists: dict[str, list[Any]]) -> bool:
    """Tell whether a message with these lists stays below the datagram limit.

    Args:
        fields: Scalar message fields.
        lists: Named lists of JSON-compatible entries.

    Returns:
        True if the encoded datagram is shorter than MAX_DATAGRAM_BYTES.
    """
    return len(_dumps({**fields, **lists})) < MAX_DATAGRAM_BYTES


def _pack(fields: dict[str, Any], lists: dict[str, list[Any]]) -> list[bytes]:
    """Spread list entries greedily over as few datagrams as possible.

    Every datagram repeats ``fields``. The first list is always present, even if
    empty; later lists appear only when a datagram holds entries of them.

    Args:
        fields: Scalar message fields, repeated in every datagram.
        lists: Named lists of JSON-compatible entries, packed in order.

    Returns:
        At least one datagram, each shorter than MAX_DATAGRAM_BYTES.

    Raises:
        ProtocolError: If a single entry does not fit in one datagram.
    """
    first_key = next(iter(lists))
    # Almost every message fits in one datagram; trying that first avoids encoding
    # the message once per entry in the loop below.
    whole = {key: entries for key, entries in lists.items() if entries or key == first_key}
    if _fits(fields, whole):
        return [_dumps({**fields, **whole})]
    datagrams: list[bytes] = []
    current: dict[str, list[Any]] = {first_key: []}
    empty_bytes = len(_dumps({**fields, **current}))
    size_bytes = empty_bytes
    for key, entries in lists.items():
        for entry in entries:
            if size_bytes + _added_bytes(current, key, entry) >= MAX_DATAGRAM_BYTES:
                datagrams.append(_dumps({**fields, **current}))
                current = {first_key: []}
                size_bytes = empty_bytes
                if size_bytes + _added_bytes(current, key, entry) >= MAX_DATAGRAM_BYTES:
                    raise ProtocolError(f"a single {key} entry exceeds the datagram limit")
            size_bytes += _added_bytes(current, key, entry)
            current.setdefault(key, []).append(entry)
    datagrams.append(_dumps({**fields, **current}))
    return datagrams


def _added_bytes(current: dict[str, list[Any]], key: str, entry: Any) -> int:
    """Return by how many bytes a datagram grows when an entry is appended to a list.

    Compact JSON is additive, so the size can be tracked without encoding the whole
    message again for every entry.

    Args:
        current: Lists already in the datagram.
        key: Name of the list the entry goes to.
        entry: JSON-compatible entry.

    Returns:
        Bytes of the entry plus its separator, plus the list itself if it is new.
    """
    entry_bytes = len(_dumps(entry))
    if key not in current:
        # A new list adds `,"key":[]`, one byte less than the object `{"key":[]}`.
        return entry_bytes + len(_dumps({key: []})) - 1
    # Every entry after the first is preceded by a comma.
    return entry_bytes + (1 if current[key] else 0)


def encode_detections(message: Detections) -> list[bytes]:
    """Encode detections, splitting into several datagrams when too large.

    Args:
        message: Detections with pixel values in pixels and time in seconds.

    Returns:
        One or more datagrams; each repeats node, camera, frame and time.

    Raises:
        ProtocolError: If a single entry does not fit in one datagram.
    """
    fields = {
        "type": "detections",
        "v": PROTOCOL_VERSION,
        "node_id": message.node_id,
        "camera_id": message.camera_id,
        "frame": message.frame,
        "t": round(message.t, SECOND_DECIMALS),
    }
    bees = [[round(x, PIXEL_DECIMALS) for x in bee] for bee in message.bees]
    markers = [
        [marker_id, round(u, PIXEL_DECIMALS), round(v, PIXEL_DECIMALS)]
        for marker_id, u, v in message.markers
    ]
    return _pack(fields, {"bees": bees, "markers": markers})


def encode_truth(message: Truth) -> list[bytes]:
    """Encode ground truth, splitting into several datagrams when too large.

    Args:
        message: Truth with positions in metres (arena frame) and time in seconds.

    Returns:
        One or more datagrams; each repeats frame, time and the robot axes.

    Raises:
        ProtocolError: If a single bee does not fit in one datagram.
    """
    fields: dict[str, Any] = {
        "type": "truth",
        "v": PROTOCOL_VERSION,
        "frame": message.frame,
        "t": round(message.t, SECOND_DECIMALS),
    }
    if message.robot is not None:
        fields["robot"] = {
            "rail_m": round(message.robot.rail_m, METRE_DECIMALS),
            "tilt_rad": round(message.robot.tilt_rad, RATE_DECIMALS),
            "focus_m": round(message.robot.focus_m, METRE_DECIMALS),
        }
    bees = [
        {
            "id": bee.id,
            "pos": _vec(bee.pos, METRE_DECIMALS),
            "vel": _vec(bee.vel, RATE_DECIMALS),
            "state": bee.state,
        }
        for bee in message.bees
    ]
    return _pack(fields, {"bees": bees})


def _check_size(datagram: bytes, name: str) -> bytes:
    """Reject a datagram that is not below the size limit.

    Args:
        datagram: Encoded message.
        name: Message type, for the error text.

    Returns:
        The same datagram.

    Raises:
        ProtocolError: If the datagram is not shorter than MAX_DATAGRAM_BYTES.
    """
    if len(datagram) >= MAX_DATAGRAM_BYTES:
        raise ProtocolError(f"{name} message is {len(datagram)} bytes; it must not be split")
    return datagram


def encode_target(message: Target) -> bytes:
    """Encode a target message.

    Args:
        message: Target with positions in metres (arena frame) and time in seconds.

    Returns:
        One datagram.

    Raises:
        ProtocolError: If the datagram is not below MAX_DATAGRAM_BYTES.
    """
    fields = {
        "type": "target",
        "v": PROTOCOL_VERSION,
        "t": round(message.t, SECOND_DECIMALS),
        "track_id": message.track_id,
        "pos": _vec(message.pos, METRE_DECIMALS),
        "vel": _vec(message.vel, RATE_DECIMALS),
        "acc": _vec(message.acc, RATE_DECIMALS),
        "distance_m": round(message.distance_m, METRE_DECIMALS),
        "state": message.state,
        "confidence": round(message.confidence, CONFIDENCE_DECIMALS),
        "others": [
            {
                "track_id": o.track_id,
                "pos": _vec(o.pos, METRE_DECIMALS),
                "vel": _vec(o.vel, RATE_DECIMALS),
            }
            for o in message.others
        ],
    }
    return _check_size(_dumps(fields), "target")


def encode_robot_pose(message: RobotPose) -> bytes:
    """Encode a robot pose message.

    Args:
        message: Pose with rail in metres, tilt in radians, focus in metres.

    Returns:
        One datagram.

    Raises:
        ProtocolError: If the datagram is not below MAX_DATAGRAM_BYTES.
    """
    fields = {
        "type": "robot_pose",
        "v": PROTOCOL_VERSION,
        "t": round(message.t, SECOND_DECIMALS),
        "rail_m": round(message.rail_m, METRE_DECIMALS),
        "tilt_rad": round(message.tilt_rad, RATE_DECIMALS),
        "focus_m": round(message.focus_m, METRE_DECIMALS),
    }
    return _check_size(_dumps(fields), "robot_pose")


def _vec3(values: list[float]) -> Vec3:
    """Convert a decoded JSON list into a 3-vector.

    Args:
        values: List of three numbers.

    Returns:
        The vector as a tuple.

    Raises:
        ValueError: If the list does not have three entries.
    """
    x, y, z = values
    return (x, y, z)


def _decode_detections(d: dict[str, Any]) -> Detections:
    """Build Detections from decoded JSON.

    Args:
        d: Decoded message dictionary.

    Returns:
        The message.
    """
    return Detections(
        node_id=d["node_id"],
        camera_id=d["camera_id"],
        frame=d["frame"],
        t=d["t"],
        bees=tuple((u, v, size) for u, v, size in d["bees"]),
        markers=tuple((m, u, v) for m, u, v in d.get("markers", ())),
    )


def _decode_target(d: dict[str, Any]) -> Target:
    """Build Target from decoded JSON.

    Args:
        d: Decoded message dictionary.

    Returns:
        The message.
    """
    return Target(
        t=d["t"],
        track_id=d["track_id"],
        pos=_vec3(d["pos"]),
        vel=_vec3(d["vel"]),
        acc=_vec3(d["acc"]),
        distance_m=d["distance_m"],
        state=d["state"],
        confidence=d["confidence"],
        others=tuple(
            OtherBee(o["track_id"], _vec3(o["pos"]), _vec3(o["vel"])) for o in d["others"]
        ),
    )


def _decode_robot_pose(d: dict[str, Any]) -> RobotPose:
    """Build RobotPose from decoded JSON.

    Args:
        d: Decoded message dictionary.

    Returns:
        The message.
    """
    return RobotPose(d["t"], d["rail_m"], d["tilt_rad"], d["focus_m"])


def _decode_truth(d: dict[str, Any]) -> Truth:
    """Build Truth from decoded JSON.

    Args:
        d: Decoded message dictionary.

    Returns:
        The message.
    """
    robot = d.get("robot")
    return Truth(
        frame=d["frame"],
        t=d["t"],
        bees=tuple(
            TruthBee(b["id"], _vec3(b["pos"]), _vec3(b["vel"]), b["state"]) for b in d["bees"]
        ),
        robot=None if robot is None else RobotAxes(robot["rail_m"], robot["tilt_rad"], robot["focus_m"]),
    )


_DECODERS = {
    "detections": _decode_detections,
    "target": _decode_target,
    "robot_pose": _decode_robot_pose,
    "truth": _decode_truth,
}


def decode(datagram: bytes) -> Message:
    """Parse one datagram into a message.

    Args:
        datagram: UTF-8 JSON payload of one UDP datagram.

    Returns:
        The parsed message; lists are tuples.

    Raises:
        ProtocolError: If the JSON is invalid, the type is unknown, the version is
            wrong, or a field is missing or malformed.
    """
    try:
        d = json.loads(datagram.decode("utf-8"))
    except ValueError as error:  # includes UnicodeDecodeError and JSONDecodeError
        raise ProtocolError(f"invalid JSON: {error}") from error
    if not isinstance(d, dict):
        raise ProtocolError("message must be a JSON object")
    if d.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version: {d.get('v')!r}")
    message_type = d.get("type")
    decoder = _DECODERS.get(message_type) if isinstance(message_type, str) else None
    if decoder is None:
        raise ProtocolError(f"unknown message type: {d.get('type')!r}")
    try:
        return decoder(d)
    except KeyError as error:
        raise ProtocolError(f"missing field {error} in {d['type']} message") from error
    except (TypeError, ValueError) as error:
        raise ProtocolError(f"malformed {d['type']} message: {error}") from error
