"""Tracking server entry point: turns `detections` into tracks and writes the track log.

Run with `python -m tracker.main`. For every frame the receiver releases, the tracker
predicts all tracks to the frame's time, triangulates the detections (known tracks
first, then new points), updates the tracks and writes one JSON line to the log.
`process_frame` is pure (no socket, no clock) so tests can drive it directly; `run` adds
the UDP socket and the log file.
"""

import argparse
import json
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from shared.config import (
    Arena,
    Config,
    TrackerSettings,
    frame_rate_hz,
    last_frame_of,
    load_config,
)
from shared.protocol import METRE_DECIMALS, RATE_DECIMALS, SECOND_DECIMALS, UDP_HOST
from tracker.receiver import Frame, FrameAssembler, receive_frames
from tracker.tracks import Track, TrackSet, create_track_set, predict_tracks, update_tracks
from tracker.triangulate import TrackingCamera, frame_rays, tracking_cameras, triangulate

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
DEFAULT_LOG_PATH = Path("logs/tracks.jsonl")
# Room for about one second of detections, so a short stall of the tracker loses nothing.
RECEIVE_BUFFER_BYTES = 4 * 1024 * 1024
# The state vector holds position, velocity and acceleration, three values each.
_AXES = 3


@dataclass
class Tracker:
    """Everything the tracking server keeps between frames.

    Attributes:
        cameras: Tracking cameras with pose and lens model, in config order.
        arena: Arena box, arena frame, metres.
        settings: Tuning values.
        track_set: The current tracks.
    """

    cameras: list[TrackingCamera]
    arena: Arena
    settings: TrackerSettings
    track_set: TrackSet


def create_tracker(config: Config) -> Tracker:
    """Build a tracker without tracks from the configuration.

    Args:
        config: Configuration. Camera poses are the nominal ones from the file.

    Returns:
        The tracker.
    """
    return Tracker(tracking_cameras(config), config.arena, config.tracker, create_track_set())


def _rounded(values: np.ndarray, decimals: int) -> list[float]:
    """Round a vector for the log.

    Args:
        values: Vector, shape (3,).
        decimals: Decimals to keep.

    Returns:
        The rounded components as plain floats.
    """
    return [round(float(value), decimals) for value in values]


def _track_record(track: Track) -> dict[str, Any]:
    """Describe one track for the log.

    Args:
        track: The track; state in the arena frame (m, m/s, m/s^2).

    Returns:
        JSON-compatible fields of the track.
    """
    return {
        "id": track.id,
        "pos": _rounded(track.state[:_AXES], METRE_DECIMALS),
        "vel": _rounded(track.state[_AXES : 2 * _AXES], RATE_DECIMALS),
        "acc": _rounded(track.state[2 * _AXES :], RATE_DECIMALS),
        "state": track.motion,
        "confirmed": track.confirmed,
        "cameras": track.camera_count,
    }


def process_frame(tracker: Tracker, frame: Frame) -> dict[str, Any]:
    """Advance the tracker by one frame of detections.

    Args:
        tracker: Tracker, modified in place.
        frame: Detections of one frame with its exposure time (simulation s).

    Returns:
        The log record of the frame: frame number, time and all tracks.
    """
    tracks = tracker.track_set
    predict_tracks(tracks, frame.t_s, tracker.settings)
    track_ids = np.array([track.id for track in tracks.tracks], dtype=int)
    predicted_m = np.array([track.state[:_AXES] for track in tracks.tracks]).reshape(-1, _AXES)
    points = triangulate(
        frame_rays(frame.detections, tracker.cameras),
        track_ids,
        predicted_m,
        tracker.cameras,
        tracker.arena,
        tracker.settings,
    )
    update_tracks(
        tracks,
        points.positions_m,
        points.camera_counts,
        points.track_ids,
        frame.t_s,
        tracker.settings,
    )
    return {
        "frame": frame.frame,
        "t": round(frame.t_s, SECOND_DECIMALS),
        "tracks": [_track_record(track) for track in tracks.tracks],
    }


def run(config: Config, log_path: Path, duration_s: float | None) -> None:
    """Receive detections over UDP and write the track log until the duration is over.

    Args:
        config: Configuration.
        log_path: File that gets one JSON line per processed frame.
        duration_s: Stop after the last frame before this simulation time (s), or None
            to run until interrupted.
    """
    rate_hz = frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames / rate_hz
    )
    tracker = create_tracker(config)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, open(
        log_path, "w", encoding="utf-8"
    ) as log:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
        sock.bind((UDP_HOST, config.ports.detections))
        for frame in receive_frames(sock, assembler):
            log.write(json.dumps(process_frame(tracker, frame), separators=(",", ":")) + "\n")
            # Each line is flushed so that the metrics can read the log while it grows.
            log.flush()
            if duration_s is not None and frame.frame >= last_frame_of(config, duration_s):
                return


def main() -> None:
    """Parse the command line and run the tracker."""
    parser = argparse.ArgumentParser(description="Bee tracking server")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG_PATH, help="track log file")
    parser.add_argument("--duration", type=float, default=None, help="simulation seconds")
    args = parser.parse_args()
    try:
        run(load_config(args.config), args.log, args.duration)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
