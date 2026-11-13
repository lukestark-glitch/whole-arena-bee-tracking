"""Virtual camera nodes: turn per-camera detections into delayed UDP datagrams.

Part of the simulator. A node serves several cameras and adds one processing delay
per frame; `simulator/main.py` sends each datagram at its `send_time_s`.
"""

from dataclasses import dataclass

import numpy as np

from shared.config import Node, Noise
from shared.protocol import Detections, encode_detections


@dataclass(frozen=True)
class Outgoing:
    """One datagram waiting to be sent.

    Attributes:
        send_time_s: Simulation time at which the datagram leaves (s).
        port: Destination UDP port.
        datagram: Encoded message bytes.
    """

    send_time_s: float
    port: int
    datagram: bytes


def node_messages(
    node: Node,
    bees_by_camera: dict[str, tuple[tuple[float, float, float], ...]],
    frame: int,
    t_s: float,
    noise: Noise,
    port: int,
    rng: np.random.Generator,
) -> list[Outgoing]:
    """Build the datagrams one camera node sends for one frame.

    Args:
        node: The node and its camera ids.
        bees_by_camera: Detections (u_px, v_px, size_px) per camera id.
        frame: Frame number.
        t_s: Simulation time of the frame (s).
        noise: Noise settings (node delay and jitter in s).
        port: Destination UDP port.
        rng: The simulation's random generator.

    Returns:
        One Outgoing per datagram, all leaving at t_s plus the node's delay.
    """
    # Both cameras of a node share its processing delay; it cannot be negative.
    delay_s = max(0.0, float(rng.normal(noise.node_delay_s, noise.node_delay_jitter_s)))
    outgoing: list[Outgoing] = []
    for camera_id in node.camera_ids:
        message = Detections(
            node_id=node.id,
            camera_id=camera_id,
            frame=frame,
            t=t_s,
            bees=bees_by_camera[camera_id],
        )
        outgoing.extend(
            Outgoing(t_s + delay_s, port, datagram) for datagram in encode_detections(message)
        )
    return outgoing
