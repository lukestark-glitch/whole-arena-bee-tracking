"""Tests for the simulation loop and the UDP runner."""

import dataclasses
import math
import socket
from pathlib import Path

import numpy as np
import pytest

from experiments.listen_detections import summarise
from shared.config import Config, frame_rate_hz, load_config, load_robot
from shared.protocol import (
    UDP_HOST,
    Detections,
    RobotPose,
    Target,
    Truth,
    decode,
    encode_target,
)
from shared.wand import STOP, wand_duration_s, wand_phase
from simulator.main import (
    Simulation,
    apply_control,
    apply_target,
    create_simulation,
    run,
    step_simulation,
)
from simulator.nodes import Outgoing

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
ROBOT = load_robot(Path(__file__).parent.parent / "config" / "robot.yaml")
ROUNDING_M = 1e-4  # allowance for the encoder's rounding
TIME_ROUNDING_S = 1e-6  # the encoder rounds t to microseconds


def _frames(config: Config, seed: int, count: int) -> list[list[Outgoing]]:
    """Simulate a number of frames with a given seed."""
    simulation = create_simulation(config, ROBOT, seed, 1.0)
    return [step_simulation(simulation) for _ in range(count)]


def _frames_with_controls(
    seed: int, count: int, controls: dict[int, tuple[str, object]]
) -> list[list[Outgoing]]:
    """Simulate frames, applying a control before the frame whose number is its key."""
    simulation = create_simulation(CONFIG, ROBOT, seed, 1.0)
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
    """Every frame has one truth with 10 bees and robot axes, and one detections per camera."""
    camera_ids = {c.id for c in CONFIG.cameras}
    for frame, outgoing in enumerate(_frames(CONFIG, 1, 120)):
        messages = [decode(o.datagram) for o in outgoing]
        truths = [m for m in messages if isinstance(m, Truth)]
        detections = [m for m in messages if isinstance(m, Detections)]
        assert len(truths) == 1 and len(truths[0].bees) == 10
        assert truths[0].robot is not None
        assert sorted(d.camera_id for d in detections) == sorted(camera_ids)
        expected_t = round(frame / 60, 6)
        assert all(m.t == expected_t for m in truths + detections)
        assert all(o.send_time_s >= expected_t - TIME_ROUNDING_S for o in outgoing)


def test_robot_pose_at_control_rate() -> None:
    """Two seconds of frames carry one robot_pose per control step, in time order."""
    poses = [
        decode(o.datagram)
        for outgoing in _frames(CONFIG, 1, 120)
        for o in outgoing
        if o.port == CONFIG.ports.robot_pose
    ]
    assert len(poses) == round(2.0 * ROBOT.control.rate_hz)
    assert all(isinstance(pose, RobotPose) for pose in poses)
    assert [pose.t for pose in poses] == sorted(pose.t for pose in poses)
    assert poses[-1].t == 2.0


def test_robot_limit_control() -> None:
    """A robot limit from the page replaces the value of the robot file; a bad one is refused."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, "rail_max_speed_m_s", 0.7)
    assert simulation.controller.robot.rail.max_speed == 0.7
    assert simulation.controller.robot.tilt == ROBOT.tilt
    with pytest.raises(ValueError):
        apply_control(simulation, "tilt_max_acceleration_rad_s2", 0)


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
            # Port 0 lets the system pick a free port to listen on for `target`.
            target=0,
        )
        config = dataclasses.replace(CONFIG, ports=ports)
        run(config, ROBOT, seed=1, duration_s=0.1, time_scale=1.0, fast=True)
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
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, "bee_count", 12)
    assert _truth_ids(step_simulation(simulation)) == list(range(12))
    apply_control(simulation, "bee_count", 4)
    assert _truth_ids(step_simulation(simulation)) == [0, 1, 2, 3]
    apply_control(simulation, "bee_count", 6)
    assert _truth_ids(step_simulation(simulation)) == [0, 1, 2, 3, 12, 13]


def test_max_speed_control() -> None:
    """After a new speed limit no bee is faster than it."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, "max_speed_m_s", 0.3)
    step_simulation(simulation)
    assert all(np.linalg.norm(b.velocity_m_s) <= 0.3 + 1e-12 for b in simulation.bees)


def test_full_dropout_control() -> None:
    """With dropout probability 1 no detections report a bee."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, "dropout_probability", 1)
    for _ in range(5):
        detections = [m for m in _step_one(simulation) if isinstance(m, Detections)]
        assert detections and all(not d.bees for d in detections)


def test_seed_control_restarts() -> None:
    """A seed control makes the following frames equal to a fresh simulation."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    for _ in range(50):
        step_simulation(simulation)
    apply_control(simulation, "seed", 7)
    apply_control(simulation, "paused", True)
    assert simulation.paused and simulation.seed == 7
    fresh = create_simulation(CONFIG, ROBOT, 7, 1.0)
    assert [step_simulation(simulation) for _ in range(20)] == [
        step_simulation(fresh) for _ in range(20)
    ]


def _stop(node_id: str, stopped: bool) -> tuple[str, object]:
    """Build a `node_stopped` control."""
    return ("node_stopped", {"node_id": node_id, "stopped": stopped})


def _node_ids(outgoing: list[Outgoing]) -> list[str]:
    """Node ids of the detections messages in one frame."""
    return [m.node_id for m in (decode(o.datagram) for o in outgoing) if isinstance(m, Detections)]


def test_stopped_node_sends_nothing_and_changes_nothing_else() -> None:
    """A stopped node is silent; the truth and the other nodes stay byte-identical."""
    reference = _frames(CONFIG, 5, 120)
    stopped = _frames_with_controls(5, 120, {0: _stop("node-03", True)})
    for frame_ref, frame_stopped in zip(reference, stopped, strict=True):
        assert "node-03" not in _node_ids(frame_stopped)
        assert len(_node_ids(frame_stopped)) == 14
        kept = [o for o in frame_ref if "node-03" not in _node_ids([o])]
        assert frame_stopped == kept


def test_node_restart_brings_messages_back() -> None:
    """Starting a stopped node again makes it send again."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, *_stop("node-03", True))
    assert "node-03" not in _node_ids(step_simulation(simulation))
    apply_control(simulation, *_stop("node-03", False))
    assert _node_ids(step_simulation(simulation)).count("node-03") == 2


def test_stopped_node_survives_seed_control() -> None:
    """A restart with a new seed keeps the stopped nodes."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    apply_control(simulation, *_stop("node-03", True))
    apply_control(simulation, "seed", 7)
    assert simulation.stopped_nodes == frozenset({"node-03"})
    assert "node-03" not in _node_ids(step_simulation(simulation))


@pytest.mark.parametrize(
    "value",
    [
        {"node_id": "node-99", "stopped": True},
        {"stopped": True},
        {"node_id": "node-03"},
        {"node_id": "node-03", "stopped": 1},
        "node-03",
    ],
)
def test_invalid_node_stopped(value: object) -> None:
    """Unknown nodes, missing keys, non-bool flags and plain strings are refused."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    with pytest.raises(ValueError):
        apply_control(simulation, "node_stopped", value)
    assert simulation.stopped_nodes == frozenset()


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
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    with pytest.raises(ValueError):
        apply_control(simulation, name, value)
    assert simulation.config == CONFIG


def test_controls_reject_values_that_would_stall_or_overflow() -> None:
    """Huge counts, huge speeds, tiny time scales and oversized integers are refused."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    for name, value in [
        ("bee_count", 10**8),
        ("max_speed_m_s", 1e308),
        ("time_scale", 1e-9),
        ("pixel_sigma_px", 10**400),
    ]:
        with pytest.raises(ValueError):
            apply_control(simulation, name, value)


def test_target_of_an_earlier_run_is_ignored() -> None:
    """After a restart a target with a later time than the present does not reach the robot."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    target = Target(
        5.0, 1, (0.4, 0.1, 0.1), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.3, "flying", 1.0, ()
    )
    apply_target(simulation, encode_target(target))
    assert simulation.controller.target is None
    apply_target(simulation, encode_target(dataclasses.replace(target, t=0.0)))
    assert simulation.controller.target is not None


def _detections_of(simulation: Simulation) -> list[Detections]:
    """Step one frame and return its detections messages."""
    return [m for m in _step_one(simulation) if isinstance(m, Detections)]


def test_wand_run_has_one_point_and_the_robot_aims_at_it() -> None:
    """In a wand run the truth holds only the wand, and the robot points at it at a stop."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0, wand=True)
    settings = CONFIG.calibration
    last_stop_s = wand_duration_s(settings) - settings.stop_dwell_s / 4
    truth = None
    for _ in range(round(last_stop_s * 60)):
        truth = next(m for m in _step_one(simulation) if isinstance(m, Truth))
        assert len(truth.bees) == 1
    assert wand_phase(settings, truth.t)[0] == STOP
    wand = truth.bees[0].pos
    start = ROBOT.rail_start_m
    assert truth.robot.rail_m == pytest.approx(wand[0] - start[0], abs=1e-4)
    tilt_rad = math.atan2(wand[2] - start[2], wand[1] - start[1])
    assert truth.robot.tilt_rad == pytest.approx(tilt_rad, abs=1e-4)
    assert truth.robot.focus_m == pytest.approx(
        math.hypot(wand[1] - start[1], wand[2] - start[2]), abs=1e-4
    )


def test_miscalibrated_cameras_are_off_by_the_configured_error() -> None:
    """Every true camera is 1 cm and 2 degrees away from its nominal pose; the robot is shifted."""
    nominal = create_simulation(CONFIG, ROBOT, 1, 1.0)
    off = create_simulation(CONFIG, ROBOT, 1, 1.0, miscalibrated=True)
    settings = CONFIG.calibration
    for camera_id, pose in nominal.poses.items():
        moved_m = np.linalg.norm(off.poses[camera_id].position_m - pose.position_m)
        assert moved_m == pytest.approx(settings.camera_error_position_m)
    shift = np.array(off.controller.robot.rail_start_m) - np.array(ROBOT.rail_start_m)
    assert shift == pytest.approx(np.array(settings.robot_error_m))
    # The bees are the same in both: the errors come from a generator of their own.
    assert all(
        np.array_equal(a.position_m, b.position_m) for a, b in zip(nominal.bees, off.bees)
    )


def test_markers_are_reported_once_per_second() -> None:
    """Frame 0 and frame 60 carry wall markers of every camera, the frames between none."""
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    with_markers = []
    for frame in range(61):
        if any(d.markers for d in _detections_of(simulation)):
            with_markers.append(frame)
    assert with_markers == [0, 60]


def test_displaced_camera_sees_the_markers_elsewhere() -> None:
    """Displacing a camera shifts its marker pixels; a bad value is refused."""
    quiet = dataclasses.replace(CONFIG, noise=dataclasses.replace(CONFIG.noise, pixel_sigma_px=0.0))
    before = create_simulation(quiet, ROBOT, 1, 1.0)
    after = create_simulation(quiet, ROBOT, 1, 1.0)
    apply_control(after, "camera_displaced", {"camera_id": "cam-06", "angle_rad": 0.02})
    seen = {d.camera_id: d.markers for d in _detections_of(before)}
    moved = {d.camera_id: d.markers for d in _detections_of(after)}
    assert seen["cam-01"] == moved["cam-01"]
    assert seen["cam-06"] != moved["cam-06"]
    with pytest.raises(ValueError):
        apply_control(after, "camera_displaced", {"camera_id": "cam-99", "angle_rad": 0.02})
    with pytest.raises(ValueError):
        apply_control(after, "camera_displaced", {"camera_id": "cam-06", "angle_rad": 1.0})
