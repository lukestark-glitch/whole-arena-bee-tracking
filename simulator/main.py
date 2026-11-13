"""Simulator entry point: runs bees, cameras and nodes and sends UDP datagrams.

Run with `python -m simulator.main`. `Simulation` and `step_simulation` are pure (no
clock, no sockets) so tests can use them; `run` adds the UDP socket, the wall-clock
pacing and the simulator window: one port that serves the page and a WebSocket over
which the page receives `scene` and `state` messages and sends `control` messages.
The page changes the simulation only through `apply_control`.
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
from http import HTTPStatus
from pathlib import Path

import numpy as np
from websockets.asyncio.server import ServerConnection, broadcast, serve
from websockets.datastructures import Headers
from websockets.http11 import Request, Response

from shared.config import Config, frame_rate_hz, load_config
from shared.geometry import CameraPose, camera_pose
from shared.protocol import UDP_HOST, Truth, TruthBee, encode_truth
from simulator.bees import Bee, create_bee, create_bees, step_bees
from simulator.cameras import observe_bees
from simulator.nodes import Outgoing, node_messages

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
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
# While paused the frame loop looks for a resume every 20 ms of wall time.
PAUSE_POLL_S = 0.02


def _truth_messages(config: Config, bees: list[Bee], frame: int, t_s: float) -> list[Outgoing]:
    """Build the truth datagrams of one frame.

    Args:
        config: Configuration (ports).
        bees: Current bees, arena frame (m, m/s).
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
        robot=None,
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


def create_simulation(config: Config, seed: int, time_scale: float) -> Simulation:
    """Create a simulation at frame 0.

    Args:
        config: Configuration.
        seed: Seed of the random generator.
        time_scale: Simulation seconds per wall-clock second.

    Returns:
        A running (not paused) simulation with `config.bees.count` bees.
    """
    rng = np.random.default_rng(seed)
    poses = {c.id: camera_pose(c.position_m, c.look_at_m) for c in config.cameras}
    bees = create_bees(config.bees, config.arena, rng)
    return Simulation(config, seed, rng, poses, bees, 0, len(bees), False, time_scale)


def step_simulation(simulation: Simulation) -> list[Outgoing]:
    """Produce the datagrams of the next frame, then advance the bees by one frame.

    Args:
        simulation: Simulation, modified in place (bees, random generator, frame).

    Returns:
        The datagrams sorted by send time (stable).
    """
    config = simulation.config
    rate_hz = frame_rate_hz(config)
    t_s = simulation.frame / rate_hz
    out = _truth_messages(config, simulation.bees, simulation.frame, t_s)
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
    for node in config.nodes:
        out.extend(
            node_messages(
                node, seen, simulation.frame, t_s, config.noise, config.ports.detections,
                simulation.rng,
            )
        )
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

    Args:
        simulation: Simulation, modified in place; `paused` and `time_scale` are kept.
        value: New seed (integer from 0 to MAX_SEED).

    Raises:
        ValueError: If the value is not such an integer.
    """
    seed = _integer("seed", value, 0, MAX_SEED)
    fresh = create_simulation(simulation.config, seed, simulation.time_scale)
    simulation.seed = fresh.seed
    simulation.rng = fresh.rng
    simulation.bees = fresh.bees
    simulation.frame = fresh.frame
    simulation.next_bee_id = fresh.next_bee_id


_CONTROLS: dict[str, Callable[[Simulation, object], None]] = {
    "bee_count": _set_bee_count,
    "max_speed_m_s": _set_max_speed,
    "pixel_sigma_px": _set_pixel_sigma,
    "dropout_probability": _set_dropout,
    "paused": _set_paused,
    "time_scale": _set_time_scale,
    "seed": _restart,
}


def apply_control(simulation: Simulation, name: str, value: object) -> None:
    """Apply one control from the page.

    Args:
        simulation: Simulation, modified in place. Call only between frames.
        name: Control name: bee_count, max_speed_m_s, pixel_sigma_px,
            dropout_probability, paused, time_scale or seed.
        value: New value; its type and range depend on the control.

    Raises:
        ValueError: If the name is unknown or the value is invalid.
    """
    if not isinstance(name, str) or name not in _CONTROLS:
        raise ValueError(f"unknown control {name!r}")
    _CONTROLS[name](simulation, value)


def scene_message(config: Config) -> str:
    """Build the `scene` message: the arena box and the camera poses.

    Args:
        config: Configuration (arena size in m, cameras in the arena frame).

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
    message = {"type": "scene", "arena_size_m": list(config.arena.size_m), "cameras": cameras}
    return json.dumps(message, separators=(",", ":"))


def state_message(simulation: Simulation) -> str:
    """Build the `state` message: bees as they are now and the current settings.

    Args:
        simulation: Simulation; `frame` and `t` describe the next frame to be produced.

    Returns:
        JSON text of the message; positions in the arena frame, rounded, metres.
    """
    config = simulation.config
    message = {
        "type": "state",
        "frame": simulation.frame,
        "t": round(simulation.frame / frame_rate_hz(config), TIME_DECIMALS),
        "seed": simulation.seed,
        "paused": simulation.paused,
        "time_scale": simulation.time_scale,
        "settings": {
            "bee_count": config.bees.count,
            "max_speed_m_s": config.bees.max_speed_m_s,
            "pixel_sigma_px": config.noise.pixel_sigma_px,
            "dropout_probability": config.noise.dropout_probability,
        },
        "bees": [
            {
                "id": bee.id,
                "pos": [round(float(x), POSITION_DECIMALS) for x in bee.position_m],
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


def _serve_page(connection: ServerConnection, request: Request) -> Response | None:
    """Answer a plain HTTP GET with the page; let WebSocket upgrades through.

    Args:
        connection: The connection (used to build the response).
        request: The HTTP request.

    Returns:
        A response for a plain GET, None for a WebSocket upgrade.
    """
    if "Upgrade" in request.headers:
        return None
    if not PAGE_PATH.is_file():
        return connection.respond(HTTPStatus.NOT_FOUND, "Simulator page not found\n")
    body = PAGE_PATH.read_bytes()
    headers = Headers(
        [("Content-Type", "text/html; charset=utf-8"), ("Content-Length", str(len(body)))]
    )
    return Response(HTTPStatus.OK.value, HTTPStatus.OK.phrase, headers, body)


async def _sleep_until(deadline_s: float) -> None:
    """Sleep until a monotonic clock time, yielding to the event loop at least once.

    Args:
        deadline_s: Target time on `time.monotonic` (wall-clock s).
    """
    await asyncio.sleep(max(0.0, deadline_s - time.monotonic()))


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
        if fast:
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
    config: Config, seed: int, duration_s: float | None, time_scale: float, fast: bool
) -> None:
    """Run the simulation with its window server until the duration is over.

    Args:
        config: Configuration.
        seed: Seed of the random generator.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        time_scale: Initial simulation seconds per wall-clock second.
        fast: Send as fast as possible, no pacing.
    """
    simulation = create_simulation(config, seed, time_scale)
    pages: set[ServerConnection] = set()
    pending: list[str | bytes] = []

    async def handle_page(connection: ServerConnection) -> None:
        """Send the scene, then queue the controls the page sends for the frame loop."""
        await connection.send(scene_message(simulation.config))
        pages.add(connection)
        try:
            async for raw in connection:
                pending.append(raw)
        finally:
            pages.discard(connection)

    async with serve(
        handle_page, UDP_HOST, config.ports.simulator_window, process_request=_serve_page
    ):
        snapshots = asyncio.create_task(_snapshot_loop(simulation, pages))
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                await _frame_loop(simulation, pending, sock, duration_s, fast)
        finally:
            snapshots.cancel()


def run(
    config: Config, seed: int, duration_s: float | None, time_scale: float, fast: bool
) -> None:
    """Run the simulator: send all datagrams over UDP and serve the simulator window.

    Args:
        config: Configuration.
        seed: Seed of the random generator.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        time_scale: Initial simulation seconds per wall-clock second (1.0 = real time).
        fast: Send as fast as possible, no pacing.
    """
    asyncio.run(_serve(config, seed, duration_s, time_scale, fast))


def main() -> None:
    """Parse the command line and run the simulator."""
    parser = argparse.ArgumentParser(description="Bee tracking simulator")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--seed", type=int, default=None, help="default: seed from config")
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument("--fast", action="store_true", help="no pacing")
    args = parser.parse_args()
    if args.time_scale <= 0:
        parser.error("--time-scale must be greater than 0")
    config = load_config(args.config)
    seed = config.seed if args.seed is None else args.seed
    try:
        run(config, seed, args.duration, args.time_scale, args.fast)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
