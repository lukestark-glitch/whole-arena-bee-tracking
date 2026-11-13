"""Tests for the simulation loop and the UDP runner."""

import dataclasses
import math
import socket
from pathlib import Path

import numpy as np
import pytest

from experiments.listen_detections import summarise
from shared.config import Config, frame_rate_hz, load_config
from shared.protocol import UDP_HOST, Detections, Truth, decode
from simulator.main import (
    Simulation,
    apply_control,
    create_simulation,
    run,
    step_simulation,
)
from simulator.nodes import Outgoing

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
ROUNDING_M = 1e-4  # allowance for the encoder's rounding
TIME_ROUNDING_S = 1e-6  # the encoder rounds t to microseconds


def _frames(config: Config, seed: int, count: int) -> list[list[Outgoing]]:
    """Simulate a number of frames with a given seed."""
    simulation = create_simulation(config, seed, 1.0)
    return [step_simulation(simulation) for _ in range(count)]


def _frames_with_controls(
    seed: int, count: int, controls: dict[int, tuple[str, object]]
) -> list[list[Outgoing]]:
    """Simulate frames, applying a control before the frame whose number is its key."""
    simulation = create_simulation(CONFIG, seed, 1.0)
    frames = []
    for frame in range(count):
        if frame in controls:
            apply_control(simulation, *controls[frame])
        frames.append(step_simulation(simulation))
    return frames


def _truth_ids(outgoing: list[Outgoing]) -> list[int]:
    """Bee ids of the truth message in one frame."""
    truths = [m for m in (decode(o.datagram) for o in outgoing) if isinstance(m, Truth)]
    return [b.id for b in truths[0].bees]


def _step_one(simulation: Simulation) -> list[object]:
    """Step once and decode the datagrams."""
    return [decode(o.datagram) for o in step_simulation(simulation)]


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


def _free_tcp_port() -> int:
    """Return a TCP port that is free right now."""
    with socket.socket() as sock:
        sock.bind((UDP_HOST, 0))
        return sock.getsockname()[1]


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
            simulator_window=_free_tcp_port(),
        )
        config = dataclasses.replace(CONFIG, ports=ports)
        run(config, seed=1, duration_s=0.1, time_scale=1.0, fast=True)
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


def test_same_controls_same_output() -> None:
    """Same seed and same controls at the same frames give identical output."""
    controls = {30: ("bee_count", 14), 60: ("pixel_sigma_px", 1.0)}
    first = _frames_with_controls(5, 120, controls)
    assert first == _frames_with_controls(5, 120, controls)
    assert first != _frames(CONFIG, 5, 120)


def test_bee_count_changes_and_ids_are_not_reused() -> None:
    """Adding and removing bees changes the truth and a removed id never returns."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    apply_control(simulation, "bee_count", 12)
    assert _truth_ids(step_simulation(simulation)) == list(range(12))
    apply_control(simulation, "bee_count", 4)
    assert _truth_ids(step_simulation(simulation)) == [0, 1, 2, 3]
    apply_control(simulation, "bee_count", 6)
    assert _truth_ids(step_simulation(simulation)) == [0, 1, 2, 3, 12, 13]


def test_max_speed_control() -> None:
    """After a new speed limit no bee is faster than it."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    apply_control(simulation, "max_speed_m_s", 0.3)
    step_simulation(simulation)
    assert all(np.linalg.norm(b.velocity_m_s) <= 0.3 + 1e-12 for b in simulation.bees)


def test_full_dropout_control() -> None:
    """With dropout probability 1 no detections report a bee."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    apply_control(simulation, "dropout_probability", 1)
    for _ in range(5):
        detections = [m for m in _step_one(simulation) if isinstance(m, Detections)]
        assert detections and all(not d.bees for d in detections)


def test_seed_control_restarts() -> None:
    """A seed control makes the following frames equal to a fresh simulation."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    for _ in range(50):
        step_simulation(simulation)
    apply_control(simulation, "seed", 7)
    apply_control(simulation, "paused", True)
    assert simulation.paused and simulation.seed == 7
    fresh = create_simulation(CONFIG, 7, 1.0)
    assert [step_simulation(simulation) for _ in range(20)] == [
        step_simulation(fresh) for _ in range(20)
    ]


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("unknown", 1),
        ("bee_count", -1),
        ("dropout_probability", 2),
        ("time_scale", 0),
        ("max_speed_m_s", "fast"),
        ("pixel_sigma_px", True),
        ("bee_count", True),
        ("paused", 1),
    ],
)
def test_invalid_controls(name: str, value: object) -> None:
    """Invalid controls raise ValueError and change nothing."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    with pytest.raises(ValueError):
        apply_control(simulation, name, value)
    assert simulation.config == CONFIG


def test_controls_reject_values_that_would_stall_or_overflow() -> None:
    """Huge counts, huge speeds, tiny time scales and oversized integers are refused."""
    simulation = create_simulation(CONFIG, 1, 1.0)
    for name, value in [
        ("bee_count", 10**8),
        ("max_speed_m_s", 1e308),
        ("time_scale", 1e-9),
        ("pixel_sigma_px", 10**400),
    ]:
        with pytest.raises(ValueError):
            apply_control(simulation, name, value)
