"""Tests for the virtual camera nodes."""

import numpy as np

from shared.config import Node, Noise
from shared.protocol import Detections, decode
from simulator.nodes import node_messages

NODE = Node("node-1", ("cam-a", "cam-b"))
PORT = 9999
SEEN = {"cam-a": ((10.0, 20.0, 3.0), (30.0, 40.0, 4.0)), "cam-b": ()}


def test_two_cameras_two_messages() -> None:
    """Each camera gets one datagram with the shared send time and the port."""
    noise = Noise(0.0, 0.0, 0.01, 0.005)
    out = node_messages(NODE, SEEN, {}, 7, 0.5, noise, PORT, np.random.default_rng(0))
    assert len(out) == 2
    assert out[0].send_time_s == out[1].send_time_s
    assert all(o.port == PORT for o in out)


def test_decoded_content() -> None:
    """Decoded messages carry node, camera, frame, time and bees; empty cameras too."""
    noise = Noise(0.0, 0.0, 0.0, 0.0)
    out = node_messages(NODE, SEEN, {}, 7, 0.5, noise, PORT, np.random.default_rng(0))
    first, second = (decode(o.datagram) for o in out)
    assert isinstance(first, Detections) and isinstance(second, Detections)
    assert (first.node_id, first.camera_id, first.frame, first.t) == ("node-1", "cam-a", 7, 0.5)
    assert first.bees == SEEN["cam-a"]
    assert second.camera_id == "cam-b"
    assert second.bees == ()


def test_zero_jitter_delay() -> None:
    """With no jitter the send time is the frame time plus the node delay."""
    noise = Noise(0.0, 0.0, 0.02, 0.0)
    out = node_messages(NODE, SEEN, {}, 1, 1.0, noise, PORT, np.random.default_rng(0))
    assert out[0].send_time_s == 1.0 + 0.02


def test_delay_never_negative() -> None:
    """The delay is clamped at zero even for large jitter."""
    noise = Noise(0.0, 0.0, 0.0, 1.0)
    rng = np.random.default_rng(1)
    for _ in range(1000):
        out = node_messages(NODE, SEEN, {}, 1, 2.0, noise, PORT, rng)
        assert out[0].send_time_s >= 2.0


def test_markers_are_sent_with_their_camera() -> None:
    """Markers of a camera travel in that camera's message; other cameras send none."""
    noise = Noise(0.0, 0.0, 0.0, 0.0)
    markers = {NODE.camera_ids[0]: ((3, 10.0, 20.0),)}
    out = node_messages(NODE, SEEN, markers, 1, 1.0, noise, PORT, np.random.default_rng(0))
    first, second = (decode(o.datagram) for o in out)
    assert first.markers == ((3, 10.0, 20.0),) and second.markers == ()
