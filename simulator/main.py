"""Simulator entry point: runs bees, cameras and nodes and sends UDP datagrams.

Run with `python -m simulator.main`. `Simulation` and `step_simulation` are pure (no
clock, no sockets) so tests can use them; `run` adds the UDP socket, the wall-clock
pacing and the simulator window: one port that serves the page and a WebSocket over
which the page receives `scene` and `state` messages and sends `control` messages.
The page changes the simulation only through `apply_control`.

The robot emulator runs inside the simulation so that it shares simulation time: every
frame, `step_simulation` lets the robot's stereo camera look at the true bees and runs
the robot's control steps. `run` also receives the `target` datagrams for the robot and
relays the cinema view, which the simulator window renders, to the control window.

Two options serve the calibration (Stage 6). `--miscalibrated` moves every true camera and
the true robot away from the values in the config files, so the tracker has something to
find out. `--wand` replaces the bees by the calibration wand, and the robot is aimed at
the wand as an operator would do it.
"""

import argparse
import asyncio
import json
import math
import socket
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from websockets.asyncio.server import ServerConnection, broadcast, serve
from websockets.http11 import Request, Response

from robot_emulator.controller import (
    Controller,
    apply_exposure,
    create_controller,
    has_target,
    pose_message,
    receive_target,
    step_controller,
)
from robot_emulator.motion import step_axis
from robot_emulator.stereo import observe_stereo
from shared.config import Config, Robot, frame_rate_hz, load_config, load_robot
from shared.geometry import CameraPose, camera_pose, robot_camera_pose
from shared.protocol import (
    UDP_HOST,
    ProtocolError,
    RobotAxes,
    Target,
    Truth,
    TruthBee,
    decode,
    encode_robot_pose,
    encode_truth,
)
from shared.web import page_response
from shared.wand import wand_duration_s
from simulator.bees import FLYING, Bee, create_bee, create_bees, step_bees
from simulator.cameras import observe_bees
from simulator.nodes import Outgoing, node_messages
from simulator.wand import observe_markers, perturb_pose, wand_position

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
DEFAULT_ROBOT_PATH = Path("config/robot.yaml")
# WebSocket path on which the control window receives the cinema view.
CINEMA_PATH = "/cinema"
PAGE_PATH = Path(__file__).parent / "web" / "index.html"
# How often the page gets a `state` message, in wall-clock time. 30 Hz looks smooth and
# keeps the message load small.
SNAPSHOT_RATE_HZ = 30
# Positions in `state` messages are rounded to 0.1 mm, far below what the page can draw.
POSITION_DECIMALS = 4
# Simulation time in `state` messages is rounded to microseconds, like the UDP messages.
TIME_DECIMALS = 6
# Limits for controls from the page. They keep a wrong value from stalling the frame loop
# (too many bees, a time scale near zero) or from producing numbers JSON cannot carry.
MAX_BEE_COUNT = 200
MAX_SPEED_LIMIT_M_S = 20.0
MIN_TIME_SCALE = 0.01
MAX_TIME_SCALE = 100.0
# Seeds are kept to 32 bits so that every browser and JSON parser carries them exactly.
MAX_SEED = 2**32 - 1
# The event loop cannot sleep much shorter than a millisecond; shorter waits are skipped,
# otherwise a high time scale would be slowed down by its own pacing.
MIN_SLEEP_S = 0.001
# While paused the frame loop looks for a resume every 20 ms of wall time.
PAUSE_POLL_S = 0.02
# Id of the single "bee" that stands for the calibration wand.
WAND_ID = 0
# Largest angle the page may displace a camera by (rad); a real knock is far smaller.
MAX_DISPLACEMENT_RAD = 0.2
# Slack when comparing times that were computed as count / rate (s).
TIME_EPSILON_S = 1e-9
# Robot limits the page may change: control name -> (axis, limit, largest allowed value).
# Units: m/s and m/s^2 for the rail, rad/s and rad/s^2 for the tilt. The upper bounds only
# keep absurd values out; they are far above any real drive.
ROBOT_LIMIT_CONTROLS = {
    "rail_max_speed_m_s": ("rail", "max_speed", 20.0),
    "rail_max_acceleration_m_s2": ("rail", "max_acceleration", 500.0),
    "tilt_max_speed_rad_s": ("tilt", "max_speed", 100.0),
    "tilt_max_acceleration_rad_s2": ("tilt", "max_acceleration", 5000.0),
}


def _truth_messages(
    config: Config, bees: list[Bee], controller: Controller, frame: int, t_s: float
) -> list[Outgoing]:
    """Build the truth datagrams of one frame.

    Args:
        config: Configuration (ports).
        bees: Current bees, arena frame (m, m/s).
        controller: The robot; its axes are reported as they are at `t_s`.
        frame: Frame number.
        t_s: Simulation time (s).

    Returns:
        Outgoing truth datagrams, leaving at t_s.
    """
    message = Truth(
        frame=frame,
        t=t_s,
        bees=tuple(
            TruthBee(
                id=bee.id,
                pos=tuple(float(x) for x in bee.position_m),
                vel=tuple(float(x) for x in bee.velocity_m_s),
                state=bee.state,
            )
            for bee in bees
        ),
        robot=RobotAxes(
            controller.rail.position, controller.tilt.position, controller.focus.position
        ),
    )
    return [Outgoing(t_s, config.ports.truth, d) for d in encode_truth(message)]


@dataclass
class Simulation:
    """All state of a running simulation.

    Attributes:
        config: Current settings; replaced (not mutated) by controls.
        seed: Seed of the current random generator.
        rng: The one random generator all randomness comes from.
        poses: Camera poses by camera id, arena frame.
        bees: Bees with their ground truth, arena frame (m, m/s).
        frame: Number of the next frame to produce.
        next_bee_id: Id of the next bee to be created; ids are never reused.
        paused: True while the frame loop must not step.
        time_scale: Simulation seconds per wall-clock second.
        stopped_nodes: Ids of camera nodes that send nothing (as if their computer failed).
        controller: The emulated robot; `controller.robot` holds its current limits.
        robot_rng: Random generator of the robot's stereo camera, spawned from `rng`.
        marker_rng: Random generator of the marker pixel noise, spawned from `rng`.
        control_steps: Robot control steps done since frame 0.
        exposures: Stereo exposures done since frame 0.
        miscalibrated: True cameras and robot differ from the config files.
        wand: The calibration wand takes the place of the bees.
    """

    config: Config
    seed: int
    rng: np.random.Generator
    poses: dict[str, CameraPose]
    bees: list[Bee]
    frame: int
    next_bee_id: int
    paused: bool
    time_scale: float
    controller: Controller
    robot_rng: np.random.Generator
    marker_rng: np.random.Generator
    control_steps: int = 0
    exposures: int = 0
    miscalibrated: bool = False
    wand: bool = False
    stopped_nodes: frozenset[str] = frozenset()


def _true_poses(
    config: Config, miscalibrated: bool, rng: np.random.Generator
) -> dict[str, CameraPose]:
    """Build the true camera poses: nominal, or each moved by the configured error.

    Args:
        config: Configuration (nominal cameras, error sizes in m and rad).
        miscalibrated: Whether to apply the error.
        rng: Generator for the directions of the errors.

    Returns:
        Camera id to true pose, arena frame.
    """
    settings = config.calibration
    poses = {c.id: camera_pose(c.position_m, c.look_at_m) for c in config.cameras}
    if not miscalibrated:
        return poses
    return {
        camera_id: perturb_pose(
            pose, settings.camera_error_position_m, settings.camera_error_angle_rad, rng
        )
        for camera_id, pose in poses.items()
    }


def _true_robot(config: Config, robot: Robot, miscalibrated: bool) -> Robot:
    """Build the true robot: as in the robot file, or with its rail start shifted.

    Args:
        config: Configuration (`calibration.robot_error_m`, arena frame, m).
        robot: Robot description from the robot file.
        miscalibrated: Whether to apply the shift.

    Returns:
        The robot the emulator simulates.
    """
    if not miscalibrated:
        return robot
    start_m = np.array(robot.rail_start_m) + np.array(config.calibration.robot_error_m)
    return replace(robot, rail_start_m=tuple(float(x) for x in start_m))


def create_simulation(
    config: Config,
    robot: Robot,
    seed: int,
    time_scale: float,
    miscalibrated: bool = False,
    wand: bool = False,
) -> Simulation:
    """Create a simulation at frame 0.

    Args:
        config: Configuration.
        robot: Robot description from the robot file.
        seed: Seed of the random generator.
        time_scale: Simulation seconds per wall-clock second.
        miscalibrated: Move the true cameras and the true robot away from the config files
            by the errors in `config.calibration`.
        wand: Simulate the calibration wand instead of bees.

    Returns:
        A running (not paused) simulation with `config.bees.count` bees, or with the wand.
    """
    rng = np.random.default_rng(seed)
    # The robot's draws depend on when `target` arrives over UDP, and neither the camera
    # errors nor the marker noise should change the bees. Generators of their own, spawned
    # from the one seed, keep the bees and their detections as they were before.
    robot_rng, setup_rng, marker_rng = rng.spawn(3)
    poses = _true_poses(config, miscalibrated, setup_rng)
    if wand:
        start_m = wand_position(config.calibration, config.arena, 0.0)
        bees = [Bee(WAND_ID, start_m, np.zeros(3), FLYING)]
    else:
        bees = create_bees(config.bees, config.arena, rng)
    controller = create_controller(_true_robot(config, robot, miscalibrated))
    return Simulation(
        config, seed, rng, poses, bees, 0, len(bees), False, time_scale,
        controller, robot_rng, marker_rng, miscalibrated=miscalibrated, wand=wand,
    )


def _observe_markers(simulation: Simulation) -> dict[str, tuple[tuple[int, float, float], ...]]:
    """Let every camera report the wall markers when a marker report is due.

    Args:
        simulation: Simulation; its marker generator is used for the pixel noise.

    Returns:
        Camera id to (marker_id, u_px, v_px); empty in frames without a marker report.
    """
    config = simulation.config
    settings = config.calibration
    frames_per_report = max(1, round(frame_rate_hz(config) / settings.marker_rate_hz))
    if not settings.markers or simulation.frame % frames_per_report:
        return {}
    return {
        c.id: observe_markers(
            simulation.poses[c.id],
            config.camera_models[c.model],
            settings.markers,
            config.noise.pixel_sigma_px,
            simulation.marker_rng,
        )
        for c in config.cameras
    }


def _aim_robot(simulation: Simulation, frame_t_s: float, period_s: float) -> list[Outgoing]:
    """Aim and focus the robot at the wand, as the operator does during calibration.

    Args:
        simulation: Simulation in wand mode; the robot's axes are modified in place.
        frame_t_s: Start of the frame (s).
        period_s: Frame period (s).

    Returns:
        One `robot_pose` datagram per control step, leaving at the end of its step.
    """
    controller = simulation.controller
    robot = controller.robot
    step_s = 1.0 / robot.control.rate_hz
    offset_m = simulation.bees[0].position_m - np.array(robot.rail_start_m)
    range_m = math.hypot(offset_m[1], offset_m[2])
    # Same geometry as the controller: rail under the point, tilt towards it, focus on it.
    commands = (
        (controller.rail, robot.rail, offset_m[0]),
        (controller.tilt, robot.tilt, math.atan2(offset_m[2], offset_m[1])),
        (controller.focus, robot.focus, range_m - robot.lens_offset_m),
    )
    out = []
    while (simulation.control_steps + 1) * step_s <= frame_t_s + period_s + TIME_EPSILON_S:
        for axis, limits, position in commands:
            step_axis(axis, limits, position, 0.0, step_s)
        simulation.control_steps += 1
        end_s = simulation.control_steps * step_s
        datagram = encode_robot_pose(pose_message(controller, end_s))
        out.append(Outgoing(end_s, simulation.config.ports.robot_pose, datagram))
    return out



def _expose_stereo(simulation: Simulation, frame_t_s: float, t_s: float) -> None:
    """Let the robot's stereo camera look at the bees and hand the blobs to the robot.

    Args:
        simulation: Simulation; the robot and its random generator are modified.
        frame_t_s: Time the bee positions are valid for (s).
        t_s: Time of the exposure (s), at or after `frame_t_s`.
    """
    controller = simulation.controller
    bees = simulation.bees
    positions_m = np.array([bee.position_m for bee in bees]).reshape(-1, 3)
    velocities_m_s = np.array([bee.velocity_m_s for bee in bees]).reshape(-1, 3)
    pose = robot_camera_pose(controller.robot, controller.rail.position, controller.tilt.position)
    blobs = observe_stereo(
        pose,
        controller.robot.stereo,
        # Bees move once per frame; between frames they are carried on with their velocity.
        positions_m + velocities_m_s * (t_s - frame_t_s),
        simulation.config.bees.body_length_m,
        simulation.robot_rng,
    )
    apply_exposure(controller, t_s, blobs)


def _step_robot(simulation: Simulation, frame_t_s: float, period_s: float) -> list[Outgoing]:
    """Run the robot's control steps that end within one frame period.

    A stereo exposure that is due is taken at the start of a control step, so exposures
    are aligned to the control rate.

    Args:
        simulation: Simulation; the robot is modified in place.
        frame_t_s: Start of the frame (s).
        period_s: Frame period (s).

    Returns:
        One `robot_pose` datagram per control step, leaving at the end of its step.
    """
    controller = simulation.controller
    step_s = 1.0 / controller.robot.control.rate_hz
    exposure_s = 1.0 / controller.robot.stereo.model.rate_hz
    out = []
    while (simulation.control_steps + 1) * step_s <= frame_t_s + period_s + TIME_EPSILON_S:
        start_s = simulation.control_steps * step_s
        if simulation.exposures * exposure_s <= start_s + TIME_EPSILON_S:
            _expose_stereo(simulation, frame_t_s, start_s)
            simulation.exposures += 1
        end_s = start_s + step_s
        step_controller(controller, end_s, step_s)
        simulation.control_steps += 1
        datagram = encode_robot_pose(pose_message(controller, end_s))
        out.append(Outgoing(end_s, simulation.config.ports.robot_pose, datagram))
    return out


def step_simulation(simulation: Simulation) -> list[Outgoing]:
    """Produce the datagrams of the next frame, then advance robot and bees by one frame.

    Args:
        simulation: Simulation, modified in place (bees, robot, random generators, frame).

    Returns:
        The datagrams sorted by send time (stable).
    """
    config = simulation.config
    rate_hz = frame_rate_hz(config)
    t_s = simulation.frame / rate_hz
    out = _truth_messages(config, simulation.bees, simulation.controller, simulation.frame, t_s)
    seen = {
        c.id: observe_bees(
            simulation.poses[c.id],
            config.camera_models[c.model],
            simulation.bees,
            config.bees.body_length_m,
            config.noise,
            simulation.rng,
        )
        for c in config.cameras
    }
    markers = _observe_markers(simulation)
    for node in config.nodes:
        messages = node_messages(
            node, seen, markers, simulation.frame, t_s, config.noise,
            config.ports.detections, simulation.rng,
        )
        # A stopped node is still simulated and only its datagrams are dropped: the random
        # draws then stay the same, so the truth and the other nodes do not change.
        if node.id not in simulation.stopped_nodes:
            out.extend(messages)
    if simulation.wand:
        out.extend(_aim_robot(simulation, t_s, 1.0 / rate_hz))
        simulation.bees[0].position_m = wand_position(
            config.calibration, config.arena, (simulation.frame + 1) / rate_hz
        )
    else:
        out.extend(_step_robot(simulation, t_s, 1.0 / rate_hz))
        step_bees(simulation.bees, config.bees, config.arena, 1.0 / rate_hz, simulation.rng)
    simulation.frame += 1
    return sorted(out, key=lambda o: o.send_time_s)


def _number(name: str, value: object) -> float:
    """Check that a control value is a finite number (not a bool).

    Args:
        name: Control name, for the error message.
        value: Value received.

    Returns:
        The value as a float.

    Raises:
        ValueError: If the value is not a finite number.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number, got {value!r}")
    try:
        number = float(value)
    except OverflowError as error:
        # JSON allows integers of any length; one too large for a float is not a setting.
        raise ValueError(f"{name} is too large") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return number


def _integer(name: str, value: object, minimum: int, maximum: int) -> int:
    """Check that a control value is an integer inside a range.

    Args:
        name: Control name, for the error message.
        value: Value received.
        minimum: Smallest allowed value.
        maximum: Largest allowed value.

    Returns:
        The value.

    Raises:
        ValueError: If the value is not an int (bool excluded) or is outside the range.
    """
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer from {minimum} to {maximum}, got {value!r}")
    return value


def _set_bee_count(simulation: Simulation, value: object) -> None:
    """Add bees at the end of the list or remove bees from its end.

    Args:
        simulation: Simulation, modified in place.
        value: New number of bees (integer from 0 to MAX_BEE_COUNT).

    Raises:
        ValueError: If the value is not such an integer.
    """
    count = _integer("bee_count", value, 0, MAX_BEE_COUNT)
    if simulation.wand:
        raise ValueError("bee_count cannot be changed during a wand run")
    config = simulation.config
    del simulation.bees[count:]
    while len(simulation.bees) < count:
        bee = create_bee(simulation.next_bee_id, config.bees, config.arena, simulation.rng)
        simulation.next_bee_id += 1
        simulation.bees.append(bee)
    simulation.config = replace(config, bees=replace(config.bees, count=count))


def _set_max_speed(simulation: Simulation, value: object) -> None:
    """Set the bee speed limit (m/s, above 0 and at most MAX_SPEED_LIMIT_M_S).

    Args:
        simulation: Simulation, modified in place.
        value: New limit.

    Raises:
        ValueError: If the value is not a number in that range.
    """
    speed_m_s = _number("max_speed_m_s", value)
    if not 0.0 < speed_m_s <= MAX_SPEED_LIMIT_M_S:
        raise ValueError(
            f"max_speed_m_s must be above 0 and at most {MAX_SPEED_LIMIT_M_S}, got {value!r}"
        )
    simulation.config = replace(
        simulation.config, bees=replace(simulation.config.bees, max_speed_m_s=speed_m_s)
    )


def _set_pixel_sigma(simulation: Simulation, value: object) -> None:
    """Set the detection pixel noise (px, >= 0).

    Args:
        simulation: Simulation, modified in place.
        value: New standard deviation.

    Raises:
        ValueError: If the value is not a number >= 0.
    """
    sigma_px = _number("pixel_sigma_px", value)
    if sigma_px < 0.0:
        raise ValueError(f"pixel_sigma_px must be >= 0, got {value!r}")
    simulation.config = replace(
        simulation.config, noise=replace(simulation.config.noise, pixel_sigma_px=sigma_px)
    )


def _set_dropout(simulation: Simulation, value: object) -> None:
    """Set the detection dropout probability (0..1).

    Args:
        simulation: Simulation, modified in place.
        value: New probability.

    Raises:
        ValueError: If the value is not a number in 0..1.
    """
    probability = _number("dropout_probability", value)
    if not 0.0 <= probability <= 1.0:
        raise ValueError(f"dropout_probability must be in 0..1, got {value!r}")
    simulation.config = replace(
        simulation.config,
        noise=replace(simulation.config.noise, dropout_probability=probability),
    )


def _set_paused(simulation: Simulation, value: object) -> None:
    """Pause or resume the simulation.

    Args:
        simulation: Simulation, modified in place.
        value: True to pause.

    Raises:
        ValueError: If the value is not a bool.
    """
    if not isinstance(value, bool):
        raise ValueError(f"paused must be true or false, got {value!r}")
    simulation.paused = value


def _set_time_scale(simulation: Simulation, value: object) -> None:
    """Set the simulation seconds per wall-clock second.

    Args:
        simulation: Simulation, modified in place.
        value: New time scale, from MIN_TIME_SCALE to MAX_TIME_SCALE.

    Raises:
        ValueError: If the value is not a number in that range.
    """
    time_scale = _number("time_scale", value)
    if not MIN_TIME_SCALE <= time_scale <= MAX_TIME_SCALE:
        raise ValueError(
            f"time_scale must be from {MIN_TIME_SCALE} to {MAX_TIME_SCALE}, got {value!r}"
        )
    simulation.time_scale = time_scale


def _restart(simulation: Simulation, value: object) -> None:
    """Restart in place with a new seed, keeping the current settings.

    The true camera poses stay as they are: a restart does not move cameras.

    Args:
        simulation: Simulation, modified in place; `paused` and `time_scale` are kept.
        value: New seed (integer from 0 to MAX_SEED).

    Raises:
        ValueError: If the value is not such an integer.
    """
    seed = _integer("seed", value, 0, MAX_SEED)
    # The robot in the simulation is already the true one; it must not be shifted again.
    robot = simulation.controller.robot
    fresh = create_simulation(
        simulation.config, robot, seed, simulation.time_scale, simulation.miscalibrated,
        simulation.wand,
    )
    fresh.controller = create_controller(robot)
    simulation.seed = fresh.seed
    simulation.rng = fresh.rng
    simulation.bees = fresh.bees
    simulation.frame = fresh.frame
    simulation.next_bee_id = fresh.next_bee_id
    simulation.controller = fresh.controller
    simulation.robot_rng = fresh.robot_rng
    simulation.marker_rng = fresh.marker_rng
    simulation.control_steps = fresh.control_steps
    simulation.exposures = fresh.exposures


def _set_node_stopped(simulation: Simulation, value: object) -> None:
    """Stop or restart one camera node.

    Args:
        simulation: Simulation, modified in place.
        value: Object `{"node_id": "node-03", "stopped": true}`.

    Raises:
        ValueError: If the value is not such an object or the node does not exist.
    """
    if not isinstance(value, dict) or set(value) != {"node_id", "stopped"}:
        raise ValueError(f'node_stopped must be {{"node_id": ..., "stopped": ...}}, got {value!r}')
    node_id, stopped = value["node_id"], value["stopped"]
    if not isinstance(stopped, bool):
        raise ValueError(f"node_stopped.stopped must be true or false, got {stopped!r}")
    if not any(node.id == node_id for node in simulation.config.nodes):
        raise ValueError(f"node_stopped.node_id is not a node: {node_id!r}")
    if stopped:
        simulation.stopped_nodes = simulation.stopped_nodes | {node_id}
    else:
        simulation.stopped_nodes = simulation.stopped_nodes - {node_id}


def _displace_camera(simulation: Simulation, value: object) -> None:
    """Turn one true camera by a small angle, as if it had been knocked.

    Args:
        simulation: Simulation, modified in place.
        value: Object `{"camera_id": "cam-06", "angle_rad": 0.02}`.

    Raises:
        ValueError: If the value is not such an object, the camera does not exist or the
            angle is not a number within MAX_DISPLACEMENT_RAD.
    """
    if not isinstance(value, dict) or set(value) != {"camera_id", "angle_rad"}:
        raise ValueError(f'camera_displaced must be {{"camera_id": ..., "angle_rad": ...}}')
    camera_id = value["camera_id"]
    angle_rad = _number("camera_displaced.angle_rad", value["angle_rad"])
    if not isinstance(camera_id, str) or camera_id not in simulation.poses:
        raise ValueError(f"camera_displaced.camera_id is not a camera: {camera_id!r}")
    if abs(angle_rad) > MAX_DISPLACEMENT_RAD:
        raise ValueError(f"camera_displaced.angle_rad must be within {MAX_DISPLACEMENT_RAD}")
    pose = simulation.poses[camera_id]
    cos, sin = math.cos(angle_rad), math.sin(angle_rad)
    # Rotation about the camera's own y axis (image "down"): the view pans sideways.
    pan = np.array([[cos, 0.0, sin], [0.0, 1.0, 0.0], [-sin, 0.0, cos]])
    simulation.poses[camera_id] = CameraPose(pose.position_m, pan @ pose.rotation)


def _robot_limit_setter(name: str) -> Callable[[Simulation, object], None]:
    """Build the control that changes one speed or acceleration limit of the robot.

    Args:
        name: Control name, a key of ROBOT_LIMIT_CONTROLS.

    Returns:
        The control function; it raises ValueError for a value that is not a number above 0
        and at most the largest allowed value.
    """
    axis_name, limit_name, largest = ROBOT_LIMIT_CONTROLS[name]

    def set_limit(simulation: Simulation, value: object) -> None:
        """Replace the limit in the robot description of the running robot."""
        limit = _number(name, value)
        if not 0.0 < limit <= largest:
            raise ValueError(f"{name} must be above 0 and at most {largest}, got {value!r}")
        robot = simulation.controller.robot
        axis = replace(getattr(robot, axis_name), **{limit_name: limit})
        simulation.controller.robot = replace(robot, **{axis_name: axis})

    return set_limit


_CONTROLS: dict[str, Callable[[Simulation, object], None]] = {
    "bee_count": _set_bee_count,
    "max_speed_m_s": _set_max_speed,
    "pixel_sigma_px": _set_pixel_sigma,
    "dropout_probability": _set_dropout,
    "paused": _set_paused,
    "time_scale": _set_time_scale,
    "seed": _restart,
    "node_stopped": _set_node_stopped,
    "camera_displaced": _displace_camera,
    **{name: _robot_limit_setter(name) for name in ROBOT_LIMIT_CONTROLS},
}


def apply_control(simulation: Simulation, name: str, value: object) -> None:
    """Apply one control from the page.

    Args:
        simulation: Simulation, modified in place. Call only between frames.
        name: Control name: bee_count, max_speed_m_s, pixel_sigma_px,
            dropout_probability, paused, time_scale, seed, node_stopped,
            camera_displaced or one of ROBOT_LIMIT_CONTROLS.
        value: New value; its type and range depend on the control.

    Raises:
        ValueError: If the name is unknown or the value is invalid.
    """
    if not isinstance(name, str) or name not in _CONTROLS:
        raise ValueError(f"unknown control {name!r}")
    _CONTROLS[name](simulation, value)


def scene_message(config: Config, robot: Robot) -> str:
    """Build the `scene` message: arena box, camera poses, nodes and robot geometry.

    Args:
        config: Configuration (arena size in m, cameras in the arena frame).
        robot: Robot description (rail in the arena frame, cinema camera).

    Returns:
        JSON text of the message.
    """
    cameras = [
        {
            "id": c.id,
            "position_m": list(c.position_m),
            "look_at_m": list(c.look_at_m),
            "fov_horizontal_rad": config.camera_models[c.model].fov_horizontal_rad,
        }
        for c in config.cameras
    ]
    nodes = [{"id": n.id, "cameras": list(n.camera_ids)} for n in config.nodes]
    message = {
        "type": "scene",
        "arena_size_m": list(config.arena.size_m),
        "cameras": cameras,
        "nodes": nodes,
        "body_length_m": config.bees.body_length_m,
        "robot": {
            "rail_start_m": list(robot.rail_start_m),
            "rail_length_m": robot.rail.upper,
            "lens_offset_m": robot.lens_offset_m,
            "cinema_fov_horizontal_rad": robot.cinema.fov_horizontal_rad,
            "cinema_aspect_ratio": robot.cinema.aspect_ratio,
            "depth_of_field_m": robot.cinema.depth_of_field_m,
        },
    }
    return json.dumps(message, separators=(",", ":"))


def state_message(simulation: Simulation) -> str:
    """Build the `state` message: bees and robot as they are now and the current settings.

    Args:
        simulation: Simulation; `frame` and `t` describe the next frame to be produced.

    Returns:
        JSON text of the message; positions in the arena frame, rounded, metres.
    """
    config = simulation.config
    controller = simulation.controller
    t_s = simulation.frame / frame_rate_hz(config)
    message = {
        "type": "state",
        "frame": simulation.frame,
        "t": round(t_s, TIME_DECIMALS),
        "seed": simulation.seed,
        "paused": simulation.paused,
        "wand": simulation.wand,
        "miscalibrated": simulation.miscalibrated,
        "time_scale": simulation.time_scale,
        "stopped_nodes": sorted(simulation.stopped_nodes),
        "settings": {
            "bee_count": config.bees.count,
            "max_speed_m_s": config.bees.max_speed_m_s,
            "pixel_sigma_px": config.noise.pixel_sigma_px,
            "dropout_probability": config.noise.dropout_probability,
            **{
                name: getattr(getattr(controller.robot, axis_name), limit_name)
                for name, (axis_name, limit_name, _) in ROBOT_LIMIT_CONTROLS.items()
            },
        },
        "robot": {
            "rail_m": round(controller.rail.position, POSITION_DECIMALS),
            "tilt_rad": round(controller.tilt.position, POSITION_DECIMALS),
            "focus_m": round(controller.focus.position, POSITION_DECIMALS),
            "has_target": has_target(controller, t_s),
        },
        "bees": [
            {
                "id": bee.id,
                "pos": [round(float(x), POSITION_DECIMALS) for x in bee.position_m],
                "vel": [round(float(x), POSITION_DECIMALS) for x in bee.velocity_m_s],
                "state": bee.state,
            }
            for bee in simulation.bees
        ],
    }
    return json.dumps(message, separators=(",", ":"))


def _apply_message(simulation: Simulation, raw: str | bytes) -> None:
    """Parse one text from the page and apply it as a control.

    Args:
        simulation: Simulation, modified in place.
        raw: WebSocket message, JSON `{"type": "control", "name": ..., "value": ...}`.

    Raises:
        ValueError: If the text is not such a message or the control is invalid.
    """
    try:
        message = json.loads(raw)
    except ValueError as error:
        raise ValueError(f"not JSON: {error}") from error
    if not isinstance(message, dict) or message.get("type") != "control":
        raise ValueError("not a control message")
    apply_control(simulation, message.get("name"), message.get("value"))


def apply_target(simulation: Simulation, datagram: bytes) -> None:
    """Hand a `target` datagram to the robot; anything else is ignored.

    Args:
        simulation: Simulation; the robot is modified in place.
        datagram: One UDP datagram from the tracker.
    """
    try:
        message = decode(datagram)
    except ProtocolError:
        return
    now_s = simulation.frame / frame_rate_hz(simulation.config)
    # A target from the future is a leftover of the run before a restart; taking it in
    # would shadow every target of the new run.
    if isinstance(message, Target) and message.t <= now_s:
        receive_target(simulation.controller, message)


class _TargetProtocol(asyncio.DatagramProtocol):
    """Receives `target` datagrams on the event loop, between two frames."""

    def __init__(self, simulation: Simulation) -> None:
        """Create the protocol.

        Args:
            simulation: The running simulation.
        """
        self._simulation = simulation

    def datagram_received(self, data: bytes, addr: object) -> None:
        """Hand one datagram to the robot.

        Args:
            data: The datagram bytes.
            addr: Sender address (unused).
        """
        apply_target(self._simulation, data)


def _serve_page(connection: ServerConnection, request: Request) -> Response | None:
    """Answer a plain HTTP GET with the simulator page; let WebSocket upgrades through.

    Args:
        connection: The connection (used to build the response).
        request: The HTTP request.

    Returns:
        A response for a plain GET, None for a WebSocket upgrade.
    """
    return page_response(connection, request, PAGE_PATH)


async def _sleep_until(deadline_s: float) -> None:
    """Sleep until a monotonic clock time, unless it is less than MIN_SLEEP_S away.

    Args:
        deadline_s: Target time on `time.monotonic` (wall-clock s).
    """
    remaining_s = deadline_s - time.monotonic()
    if remaining_s >= MIN_SLEEP_S:
        await asyncio.sleep(remaining_s)


def _apply_pending(simulation: Simulation, pending: list[str | bytes]) -> None:
    """Apply and clear the control messages received since the last frame.

    Args:
        simulation: Simulation, modified in place.
        pending: Raw WebSocket messages in order of arrival; emptied here.
    """
    for raw in pending:
        try:
            _apply_message(simulation, raw)
        except ValueError as error:
            print(f"ignored message from page: {error}", file=sys.stderr)
    pending.clear()


async def _frame_loop(
    simulation: Simulation,
    pending: list[str | bytes],
    sock: socket.socket,
    duration_s: float | None,
    fast: bool,
) -> None:
    """Produce frames and send their datagrams until the duration is over.

    Each datagram leaves at frame start + (send time - frame time) / time scale. The
    frame start moves on incrementally, so a changed time scale or a pause needs no
    global offset.

    Args:
        simulation: Simulation, modified in place.
        pending: Control messages from the pages, applied only between frames so that a
            frame is never sent half with old and half with new settings.
        sock: UDP socket.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        fast: Send as fast as possible, no pacing.
    """
    period_s = 1.0 / frame_rate_hz(simulation.config)
    frame_start_s = time.monotonic()
    while duration_s is None or simulation.frame * period_s < duration_s:
        _apply_pending(simulation, pending)
        if simulation.paused:
            await asyncio.sleep(PAUSE_POLL_S)
            frame_start_s = time.monotonic()
            continue
        t_s = simulation.frame * period_s
        for item in step_simulation(simulation):
            if not fast:
                await _sleep_until(frame_start_s + (item.send_time_s - t_s) / simulation.time_scale)
            sock.sendto(item.datagram, (UDP_HOST, item.port))
        # A late frame (slow step, sleeping laptop) must not be made up for with a burst.
        frame_start_s = max(frame_start_s + period_s / simulation.time_scale, time.monotonic())
        # Give the window server a turn in every frame, also when no wait was needed.
        await asyncio.sleep(0)


async def _snapshot_loop(simulation: Simulation, pages: set[ServerConnection]) -> None:
    """Broadcast `state` to all pages at the snapshot rate of the wall clock, also while paused.

    Args:
        simulation: Simulation, read only.
        pages: Connected pages; a slow or closed page is skipped by `broadcast`.
    """
    while True:
        broadcast(pages, state_message(simulation))
        await asyncio.sleep(1.0 / SNAPSHOT_RATE_HZ)


async def _serve(
    config: Config,
    robot: Robot,
    seed: int,
    duration_s: float | None,
    time_scale: float,
    fast: bool,
    miscalibrated: bool,
    wand: bool,
) -> None:
    """Run the simulation with its window server until the duration is over.

    Args:
        config: Configuration.
        robot: Robot description.
        miscalibrated: Move the true cameras and robot away from the config files.
        wand: Simulate the calibration wand instead of bees.
        seed: Seed of the random generator.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        time_scale: Initial simulation seconds per wall-clock second.
        fast: Send as fast as possible, no pacing.
    """
    simulation = create_simulation(config, robot, seed, time_scale, miscalibrated, wand)
    pages: set[ServerConnection] = set()
    viewers: set[ServerConnection] = set()
    pending: list[str | bytes] = []

    async def handle_viewer(connection: ServerConnection) -> None:
        """Keep a cinema viewer (the control window) until it disconnects."""
        viewers.add(connection)
        try:
            await connection.wait_closed()
        finally:
            viewers.discard(connection)

    async def handle_page(connection: ServerConnection) -> None:
        """Send the scene, then queue the page's controls and relay its cinema images."""
        if connection.request.path == CINEMA_PATH:
            await handle_viewer(connection)
            return
        await connection.send(scene_message(simulation.config, simulation.controller.robot))
        pages.add(connection)
        try:
            async for raw in connection:
                # Text is a control; binary is one image of the cinema view.
                if isinstance(raw, bytes):
                    broadcast(viewers, raw)
                else:
                    pending.append(raw)
        finally:
            pages.discard(connection)

    targets, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: _TargetProtocol(simulation), local_addr=(UDP_HOST, config.ports.target)
    )
    try:
        async with serve(
            handle_page, UDP_HOST, config.ports.simulator_window, process_request=_serve_page
        ):
            snapshots = asyncio.create_task(_snapshot_loop(simulation, pages))
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    await _frame_loop(simulation, pending, sock, duration_s, fast)
            finally:
                snapshots.cancel()
    finally:
        targets.close()


def run(
    config: Config,
    robot: Robot,
    seed: int,
    duration_s: float | None,
    time_scale: float,
    fast: bool,
    miscalibrated: bool = False,
    wand: bool = False,
) -> None:
    """Run the simulator: send all datagrams over UDP and serve the simulator window.

    Args:
        config: Configuration.
        robot: Robot description.
        miscalibrated: Move the true cameras and robot away from the config files.
        wand: Simulate the calibration wand instead of bees.
        seed: Seed of the random generator.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        time_scale: Initial simulation seconds per wall-clock second (1.0 = real time).
        fast: Send as fast as possible, no pacing.
    """
    asyncio.run(_serve(config, robot, seed, duration_s, time_scale, fast, miscalibrated, wand))


def main() -> None:
    """Parse the command line and run the simulator."""
    parser = argparse.ArgumentParser(description="Bee tracking simulator")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--robot", type=Path, default=DEFAULT_ROBOT_PATH, help="robot file")
    parser.add_argument("--seed", type=int, default=None, help="default: seed from config")
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument("--fast", action="store_true", help="no pacing")
    parser.add_argument(
        "--miscalibrated",
        action="store_true",
        help="true cameras and robot differ from the config files (calibration section)",
    )
    parser.add_argument(
        "--wand",
        action="store_true",
        help="calibration wand instead of bees; stops after the wand run unless --duration",
    )
    args = parser.parse_args()
    if args.time_scale <= 0:
        parser.error("--time-scale must be greater than 0")
    config = load_config(args.config)
    seed = config.seed if args.seed is None else args.seed
    duration_s = args.duration
    if args.wand and duration_s is None:
        duration_s = wand_duration_s(config.calibration)
    try:
        run(
            config, load_robot(args.robot), seed, duration_s, args.time_scale, args.fast,
            args.miscalibrated, args.wand,
        )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
