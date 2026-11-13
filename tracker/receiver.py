"""Receive camera `detections` datagrams and group them into frames.

Each camera sends one message per frame over UDP. Messages can be lost, late or
reordered, so `FrameAssembler` collects them per frame number and releases a frame
when every camera has reported or when a newer message proves the wait is over.
Only the simulation time `t` carried in the messages drives the timeout; the tracker
never reads a wall clock. The released `Frame` objects feed the tracking core.
"""

from __future__ import annotations

import socket
from collections.abc import Iterator, Sequence
from dataclasses import dataclass, field

from shared.protocol import Detections, ProtocolError, decode

# Large enough for the biggest datagram a UDP socket can deliver (IPv4 limit).
RECV_BUFFER_BYTES = 65535
# Senders round `t` to microseconds, so the difference of two times can come out this
# much too small; without the slack a timeout of two frame periods would often take three.
TIME_ROUNDING_S = 1e-6


@dataclass(frozen=True)
class Frame:
    """All detections of one exposure, grouped by camera.

    Attributes:
        frame: Frame counter shared by all cameras.
        t_s: Exposure time in simulation seconds.
        detections: Camera id to (u_px, v_px, size_px) entries. Only cameras that
            reported are present.
    """

    frame: int
    t_s: float
    detections: dict[str, tuple[tuple[float, float, float], ...]]


@dataclass
class _Pending:
    """A frame that is still waiting for cameras."""

    t_s: float
    detections: dict[str, tuple[tuple[float, float, float], ...]] = field(default_factory=dict)


class FrameAssembler:
    """Groups detections messages into frames and decides when a frame is ready."""

    def __init__(self, camera_ids: Sequence[str], timeout_s: float) -> None:
        """Create an assembler.

        Args:
            camera_ids: Ids of all cameras expected to report every frame.
            timeout_s: Simulation time in seconds after which a message newer than a
                pending frame forces that frame out incomplete.
        """
        self._camera_ids = frozenset(camera_ids)
        self._timeout_s = timeout_s
        self._pending: dict[int, _Pending] = {}
        self._last_released = -1  # Frame numbers start at 0, so -1 means none yet.

    def add(self, message: Detections) -> list[Frame]:
        """Add one message and return the frames that are now ready.

        Args:
            message: A decoded detections message.

        Returns:
            Released frames, oldest first. Usually empty or one frame.
        """
        if message.camera_id not in self._camera_ids or message.frame <= self._last_released:
            return []
        pending = self._pending.setdefault(message.frame, _Pending(t_s=message.t))
        # Parts of a large message share camera and frame; join them in arrival order.
        previous = pending.detections.get(message.camera_id, ())
        pending.detections[message.camera_id] = previous + message.bees
        return self._release_ready(message.t)

    def _release_ready(self, now_s: float) -> list[Frame]:
        """Release every frame that is complete or timed out, plus all older ones.

        Args:
            now_s: Simulation time of the newest message, in seconds.

        Returns:
            Released frames in increasing frame order.
        """
        newest_ready = -1
        for number, pending in self._pending.items():
            complete = self._camera_ids <= pending.detections.keys()
            timed_out = now_s - pending.t_s >= self._timeout_s - TIME_ROUNDING_S
            if complete or timed_out:
                newest_ready = max(newest_ready, number)
        # Releasing in order means a ready frame also flushes all older ones.
        released = [
            Frame(number, self._pending[number].t_s, self._pending[number].detections)
            for number in sorted(self._pending)
            if number <= newest_ready
        ]
        for frame in released:
            del self._pending[frame.frame]
        if released:
            self._last_released = released[-1].frame
        return released


def receive_frames(sock: socket.socket, assembler: FrameAssembler) -> Iterator[Frame]:
    """Read datagrams forever and yield frames as they become ready.

    Datagrams that cannot be decoded, or that are not detections, are skipped.

    Args:
        sock: A bound UDP socket, created and configured by the caller.
        assembler: Assembler that groups the messages into frames.

    Yields:
        Released frames, oldest first.
    """
    while True:
        try:
            message = decode(sock.recv(RECV_BUFFER_BYTES))
        except ProtocolError:
            continue
        if isinstance(message, Detections):
            yield from assembler.add(message)
