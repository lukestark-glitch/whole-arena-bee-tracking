"""Tests for the simulation loop and the UDP runner."""

import dataclasses
import math
import socket
from itertools import islice
from pathlib import Path

import numpy as np

from experiments.listen_detections import summarise
from shared.config import Config, frame_rate_hz, load_config
from shared.protocol import UDP_HOST, Detections, Truth, decode
from simulator.main import run, simulate
from simulator.nodes import Outgoing

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
ROUNDING_M = 1e-4  # allowance for the encoder's rounding
TIME_ROUNDING_S = 1e-6  # the encoder rounds t to microseconds


def _frames(config: Config, seed: int, count: int) -> list[list[Outgoing]]:
    """Simulate a number of frames with a given seed."""
    return list(islice(simulate(config, np.random.default_rng(seed)), count))


def test_same_seed_identical() -> None:
    """The same seed gives identical datagrams, ports and send times."""
    assert _frames(CONFIG, 5, 120) == _frames(CONFIG, 5, 120)


def test_different_seed_differs() -> None:
    """A different seed gives different output."""
    assert _frames(CONFIG, 5, 120) != _frames(CONFIG, 6, 120)


def test_frame_contents() -> None:
    """Every frame has one truth with 10 bees and one detections per camera."""
    camera_ids = {c.id for c in CONFIG.cameras}
    for frame, outgoing in enumerate(_frames(CONFIG, 1, 120)):
        messages = [decode(o.datagram) for o in outgoing]
        truths = [m for m in messages if isinstance(m, Truth)]
        detections = [m for m in messages if isinstance(m, Detections)]
        assert len(truths) == 1 and len(truths[0].bees) == 10
        assert sorted(d.camera_id for d in detections) == sorted(camera_ids)
        expected_t = round(frame / 60, 6)
        assert all(m.t == expected_t for m in messages)
        assert all(o.send_time_s >= expected_t - TIME_ROUNDING_S for o in outgoing)


def test_long_run_stays_in_arena_and_finite() -> None:
    """Over 60 s bees stay in the arena, pixels are finite, every camera sees a bee."""
    size_m = CONFIG.arena.size_m
    seeing: set[str] = set()
    for frame, outgoing in enumerate(_frames(CONFIG, 2, 3600)):
        for message in (decode(o.datagram) for o in outgoing):
            if isinstance(message, Truth):
                for bee in message.bees:
                    assert all(
                        -ROUNDING_M <= p <= s + ROUNDING_M for p, s in zip(bee.pos, size_m)
                    )
            elif isinstance(message, Detections):
                assert all(math.isfinite(x) for b in message.bees for x in b)
                if frame < 600 and message.bees:
                    seeing.add(message.camera_id)
    assert seeing == {c.id for c in CONFIG.cameras}


def _bound_socket() -> socket.socket:
    """Open a UDP socket on a free loopback port with a 2 s timeout."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, 0))
    sock.settimeout(2.0)
    return sock


def test_end_to_end_udp() -> None:
    """Running 0.1 s sends 6 truth and 96 detections datagrams."""
    with _bound_socket() as truth_sock, _bound_socket() as det_sock:
        ports = dataclasses.replace(
            CONFIG.ports,
            truth=truth_sock.getsockname()[1],
            detections=det_sock.getsockname()[1],
        )
        run(dataclasses.replace(CONFIG, ports=ports), seed=1, duration_s=0.1, time_scale=1.0, fast=True)
        assert len([truth_sock.recv(65535) for _ in range(6)]) == 6
        assert len([det_sock.recv(65535) for _ in range(96)]) == 96


def test_summarise_one_second() -> None:
    """One simulated second of detections summarises to 16 cameras, 60 frames each."""
    rate = int(frame_rate_hz(CONFIG))
    messages = [
        m
        for outgoing in _frames(CONFIG, 1, rate)
        for m in (decode(o.datagram) for o in outgoing)
        if isinstance(m, Detections)
    ]
    line = summarise(0, messages)
    assert "cameras=16" in line and "frames per camera=60..60" in line
