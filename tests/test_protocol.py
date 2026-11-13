"""Tests for shared.protocol: round trips, rounding, splitting and decode errors."""

import json

import pytest

from shared.protocol import (
    MAX_DATAGRAM_BYTES,
    Detections,
    OtherBee,
    ProtocolError,
    RobotAxes,
    RobotPose,
    Target,
    Truth,
    TruthBee,
    decode,
    encode_detections,
    encode_robot_pose,
    encode_target,
    encode_truth,
)

DETECTIONS = Detections(
    "node-03", "cam-06", 1042, 17.366667,
    bees=((512.31, 300.87, 14.2), (901.05, 644.40, 9.8)),
    markers=((12, 88.10, 40.55),),
)
TARGET = Target(
    17.383333, 4, (0.4120, 0.1503, 0.1711), (0.80, -0.12, 0.05), (0.0, 0.0, 0.0),
    0.2140, "flying", 0.93, (OtherBee(7, (0.4301, 0.1100, 0.1650)),),
)
POSE = RobotPose(17.38, 0.3920, 0.1100, 0.2150)
TRUTH = Truth(
    1042, 17.366667,
    (TruthBee(0, (0.41, 0.15, 0.17), (0.8, -0.1, 0.0), "flying"),),
    RobotAxes(0.392, 0.11, 0.215),
)


def _many_truth_bees(count: int) -> tuple[TruthBee, ...]:
    """Build `count` distinct truth bees."""
    return tuple(
        TruthBee(i, (round(0.1 * i, 4), 0.2, 0.3), (0.5, -0.5, 0.0), "walking") for i in range(count)
    )


def test_detections_round_trip() -> None:
    """Detections decode back to the encoded message in one datagram."""
    datagrams = encode_detections(DETECTIONS)
    assert len(datagrams) == 1
    assert decode(datagrams[0]) == DETECTIONS


def test_target_round_trip() -> None:
    """A target decodes back to the encoded message."""
    assert decode(encode_target(TARGET)) == TARGET


def test_robot_pose_round_trip() -> None:
    """A robot pose decodes back to the encoded message."""
    assert decode(encode_robot_pose(POSE)) == POSE


def test_truth_round_trip() -> None:
    """Truth with a robot decodes back to the encoded message in one datagram."""
    datagrams = encode_truth(TRUTH)
    assert len(datagrams) == 1
    assert decode(datagrams[0]) == TRUTH


def test_truth_without_robot_round_trip() -> None:
    """Truth without robot axes round-trips and omits the robot key."""
    message = Truth(1, 0.5, TRUTH.bees)
    (datagram,) = encode_truth(message)
    assert "robot" not in json.loads(datagram)
    assert decode(datagram) == message


def test_empty_bees_still_sent_and_markers_omitted() -> None:
    """Empty detections give one datagram without a markers key."""
    message = Detections("n", "c", 1, 0.1, bees=())
    datagrams = encode_detections(message)
    assert len(datagrams) == 1
    fields = json.loads(datagrams[0])
    assert fields["bees"] == []
    assert "markers" not in fields
    assert decode(datagrams[0]) == message


def test_wire_format_is_compact_with_version() -> None:
    """Datagrams use compact separators and carry type and version."""
    fields = json.loads(encode_robot_pose(POSE))
    assert fields["type"] == "robot_pose" and fields["v"] == 1
    assert b" " not in encode_robot_pose(POSE)


def test_rounding() -> None:
    """Values are rounded to the decimals fixed by the protocol."""
    message = Detections("n", "c", 1, 17.3666666666, bees=((512.314159, 1.0, 2.0),))
    decoded = decode(encode_detections(message)[0])
    assert decoded.bees[0][0] == 512.31
    assert decoded.t == 17.366667
    target = Target(
        0.0, 1, (0.41203456, 0.0, 0.0), (0.123456, 0.0, 0.0), (0.0, 0.0, 0.0),
        0.123456, "flying", 0.934, (),
    )
    decoded_target = decode(encode_target(target))
    assert decoded_target.pos[0] == 0.412
    assert decoded_target.vel[0] == 0.1235
    assert decoded_target.distance_m == 0.1235
    assert decoded_target.confidence == 0.93


def test_detections_split() -> None:
    """Large detections are split; parts repeat the header and keep order."""
    bees = tuple((float(i) + 0.5, 100.25, 10.5) for i in range(500))
    markers = tuple((i, 10.5, 20.25) for i in range(100))
    message = Detections("node-03", "cam-06", 7, 1.5, bees, markers)
    datagrams = encode_detections(message)
    assert len(datagrams) > 1
    parts = [decode(d) for d in datagrams]
    assert all(len(d) < MAX_DATAGRAM_BYTES for d in datagrams)
    assert all((p.node_id, p.camera_id, p.frame, p.t) == ("node-03", "cam-06", 7, 1.5) for p in parts)
    assert tuple(b for p in parts for b in p.bees) == bees
    assert tuple(m for p in parts for m in p.markers) == markers


def test_truth_split() -> None:
    """Large truth is split; every part repeats frame, time and robot."""
    message = Truth(3, 2.5, _many_truth_bees(100), RobotAxes(0.392, 0.11, 0.215))
    datagrams = encode_truth(message)
    assert len(datagrams) > 1
    parts = [decode(d) for d in datagrams]
    assert all(len(d) < MAX_DATAGRAM_BYTES for d in datagrams)
    assert all(p.frame == 3 and p.t == 2.5 and p.robot == message.robot for p in parts)
    assert tuple(b for p in parts for b in p.bees) == message.bees


def test_oversized_target_raises() -> None:
    """A target that does not fit in one datagram is an error, never split."""
    others = tuple(OtherBee(i, (0.1, 0.2, 0.3)) for i in range(3000))
    big = Target(0.0, 1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.1, "flying", 0.9, others)
    with pytest.raises(ProtocolError):
        encode_target(big)


def test_decode_invalid_json() -> None:
    """Garbage bytes raise ProtocolError."""
    with pytest.raises(ProtocolError, match="invalid JSON"):
        decode(b"{not json")


def test_decode_unknown_type() -> None:
    """An unknown type raises ProtocolError."""
    with pytest.raises(ProtocolError, match="unknown message type"):
        decode(b'{"type":"nope","v":1}')


def test_decode_wrong_version() -> None:
    """A wrong protocol version raises ProtocolError."""
    with pytest.raises(ProtocolError, match="version"):
        decode(b'{"type":"robot_pose","v":2,"t":0,"rail_m":0,"tilt_rad":0,"focus_m":0}')


def test_decode_missing_field() -> None:
    """A missing field raises ProtocolError naming the field."""
    with pytest.raises(ProtocolError, match="focus_m"):
        decode(b'{"type":"robot_pose","v":1,"t":0,"rail_m":0,"tilt_rad":0}')
