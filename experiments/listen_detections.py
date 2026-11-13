"""Listener that prints one summary line per simulated second of detections.

A manual check for the simulator: run it, then start `python -m simulator.main`.
It only reads the `detections` port and imports nothing from `simulator`.
"""

import argparse
import socket
from pathlib import Path

from shared.config import load_config
from shared.protocol import UDP_HOST, Detections, decode

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
RECEIVE_BUFFER_BYTES = 65535  # largest possible UDP payload


def summarise(second: int, messages: list[Detections]) -> str:
    """Build the summary line for one second of detections.

    Args:
        second: Whole second of simulation time (s).
        messages: All detections messages with t inside that second.

    Returns:
        One line with camera count, frames per camera and mean bees per camera frame.
    """
    frames: dict[str, set[int]] = {}
    for message in messages:
        frames.setdefault(message.camera_id, set()).add(message.frame)
    counts = [len(frames[c]) for c in frames]
    mean_bees = sum(len(m.bees) for m in messages) / len(messages) if messages else 0.0
    low, high = (min(counts), max(counts)) if counts else (0, 0)
    return (
        f"t={second} s  cameras={len(frames)}  frames per camera={low}..{high}  "
        f"bees per camera and frame={mean_bees:.1f}"
    )


def _listen(port: int) -> None:
    """Receive detections and print a line whenever a simulated second is complete.

    Args:
        port: UDP port to bind on the loopback address.
    """
    pending: dict[int, list[Detections]] = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((UDP_HOST, port))
        while True:
            message = decode(sock.recv(RECEIVE_BUFFER_BYTES))
            if not isinstance(message, Detections):
                continue
            pending.setdefault(int(message.t), []).append(message)
            # Delayed datagrams of a second can arrive after the next one starts, so
            # a second is printed only once a later second has been seen.
            for second in sorted(s for s in pending if s < int(message.t)):
                print(summarise(second, pending.pop(second)), flush=True)


def main() -> None:
    """Parse the command line and listen until interrupted."""
    parser = argparse.ArgumentParser(description="Print detection summaries")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = parser.parse_args()
    try:
        _listen(load_config(args.config).ports.detections)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
