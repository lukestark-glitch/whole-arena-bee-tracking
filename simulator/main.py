"""Simulator entry point: runs bees, cameras and nodes and sends UDP datagrams.

Run with `python -m simulator.main`. The generator `simulate` is pure (no clock, no
sockets) so tests can use it; `run` adds the socket and the wall-clock pacing.
"""

import argparse
import socket
import time
from collections.abc import Iterator
from pathlib import Path

import numpy as np

from shared.config import Config, frame_rate_hz, load_config
from shared.geometry import camera_pose
from shared.protocol import UDP_HOST, Truth, TruthBee, encode_truth
from simulator.bees import Bee, create_bees, step_bees
from simulator.cameras import observe_bees
from simulator.nodes import Outgoing, node_messages

DEFAULT_CONFIG_PATH = Path("config/default.yaml")


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


def simulate(config: Config, rng: np.random.Generator) -> Iterator[list[Outgoing]]:
    """Generate the datagrams of every frame, endlessly.

    Args:
        config: Configuration.
        rng: The simulation's random generator.

    Yields:
        For each frame, the datagrams sorted by send time (stable).
    """
    poses = {c.id: camera_pose(c.position_m, c.look_at_m) for c in config.cameras}
    bees = create_bees(config.bees, config.arena, rng)
    rate_hz = frame_rate_hz(config)
    frame = 0
    while True:
        t_s = frame / rate_hz
        out = _truth_messages(config, bees, frame, t_s)
        seen = {
            c.id: observe_bees(
                poses[c.id],
                config.camera_models[c.model],
                bees,
                config.bees.body_length_m,
                config.noise,
                rng,
            )
            for c in config.cameras
        }
        for node in config.nodes:
            out.extend(
                node_messages(node, seen, frame, t_s, config.noise, config.ports.detections, rng)
            )
        yield sorted(out, key=lambda o: o.send_time_s)
        step_bees(bees, config.bees, config.arena, 1.0 / rate_hz, rng)
        frame += 1


def _wait_until(deadline_s: float) -> None:
    """Sleep until a monotonic clock time.

    Args:
        deadline_s: Target time on `time.monotonic` (wall-clock s).
    """
    delay_s = deadline_s - time.monotonic()
    if delay_s > 0:
        time.sleep(delay_s)


def run(
    config: Config, seed: int, duration_s: float | None, time_scale: float, fast: bool
) -> None:
    """Run the simulator and send all datagrams over UDP.

    Args:
        config: Configuration.
        seed: Seed of the random generator.
        duration_s: Simulation time to run (s), or None to run until interrupted.
        time_scale: Simulation seconds per wall-clock second (1.0 = real time).
        fast: Send as fast as possible, no pacing.
    """
    rng = np.random.default_rng(seed)
    rate_hz = frame_rate_hz(config)
    start_s = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for frame, outgoing in enumerate(simulate(config, rng)):
            if duration_s is not None and frame / rate_hz >= duration_s:
                return
            for item in outgoing:
                if not fast:
                    _wait_until(start_s + item.send_time_s / time_scale)
                sock.sendto(item.datagram, (UDP_HOST, item.port))


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
