"""Calibration from a wand run: collect the run, solve, and keep the result in a file.

Run with `python -m tracker.wand_run` while the simulator plays a wand run
(`python -m simulator.main --wand`). The command listens to the same `detections` and
`robot_pose` messages as the tracker, then

1. estimates all camera poses from the wand (`tracker/calibrate.py`), starting from the
   nominal poses of the config,
2. triangulates the wand at the stop points with the new camera poses and fits the robot's
   rail start and focus mapping from the robot's axis readings there
   (`tracker/robot_calibration.py`),
3. writes both to a calibration file that `tracker/main.py` loads with `--calibration`.

Which frame belongs to which part of the run follows from its time and the schedule in
`shared/wand.py`. `calibrate` is pure, so tests can feed it frames directly.
"""

import argparse
import json
import math
import selectors
import socket
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from shared.config import Config, Robot, frame_rate_hz, last_frame_of, load_config, load_robot
from shared.geometry import CameraPose, pixel_to_ray
from shared.protocol import UDP_HOST, ProtocolError, RobotPose, decode
from shared.wand import DONE, REFERENCE, STOP, wand_duration_s, wand_phase
from tracker.calibrate import WandData, pose_difference, solve_cameras
from tracker.receiver import Frame, FrameAssembler, frames_from_datagram
from tracker.robot_calibration import RobotSample, fit_robot
from tracker.triangulate import TrackingCamera, closest_point_to_rays, tracking_cameras

DEFAULT_CONFIG_PATH = Path("config/default.yaml")
DEFAULT_ROBOT_PATH = Path("config/robot.yaml")
DEFAULT_CALIBRATION_PATH = Path("logs/calibration.json")
# Room for the detections of a run sent without pacing.
RECEIVE_BUFFER_BYTES = 4 * 1024 * 1024
MAX_DATAGRAM_BYTES = 65535
# Two rays are needed to triangulate the wand.
MIN_CAMERAS = 2
# The robot needs up to half a second to arrive at a stop (rail limits of the robot file);
# only the last part of each stop is used.
SETTLED_SHARE = 0.7
MM_PER_M = 1000.0


@dataclass(frozen=True)
class Calibration:
    """Result of a calibration.

    Attributes:
        cameras: Camera id to estimated pose, arena frame.
        rail_start_m: Fitted tilt axis of the robot at rail position 0, arena frame (m).
        lens_offset_m: Fitted lens reference point ahead of the tilt axis (m).
        focus_scale: Fitted slope of the robot's focus reading over the true distance.
    """

    cameras: dict[str, CameraPose]
    rail_start_m: tuple[float, float, float]
    lens_offset_m: float
    focus_scale: float


def _wand_pixels(frame: Frame, cameras: list[TrackingCamera]) -> list[tuple[int, float, float]]:
    """List where each camera saw the wand in one frame.

    Args:
        frame: One frame of a wand run.
        cameras: Camera list.

    Returns:
        (camera index, u_px, v_px) for every camera with exactly one detection; a camera
        with several detections cannot tell which one is the wand and is left out.
    """
    seen = []
    for index, camera in enumerate(cameras):
        entries = frame.detections.get(camera.id, ())
        if len(entries) == 1:
            seen.append((index, entries[0][0], entries[0][1]))
    return seen


def wand_data(config: Config, cameras: list[TrackingCamera], frames: list[Frame]) -> WandData:
    """Turn the frames of a wand run into the observations for the camera solver.

    Every `sweep_frame_step`-th frame is used. Frames on a reference point get that
    point as their known position; frames in which fewer than two cameras saw the wand
    are left out.

    Args:
        config: Configuration (wand schedule, reference points).
        cameras: Camera list; observations refer to it by index.
        frames: Frames of the run with their times (simulation s).

    Returns:
        The observations.
    """
    settings = config.calibration
    frame_index, camera_index, pixels, known = [], [], [], []
    for frame in frames:
        phase, point, _ = wand_phase(settings, frame.t_s)
        seen = _wand_pixels(frame, cameras)
        if frame.frame % settings.sweep_frame_step or phase == DONE or len(seen) < MIN_CAMERAS:
            continue
        for index, u_px, v_px in seen:
            frame_index.append(len(known))
            camera_index.append(index)
            pixels.append((u_px, v_px))
        reference = settings.reference_points_m[point] if phase == REFERENCE else (math.nan,) * 3
        known.append(reference)
    return WandData(
        np.array(frame_index, dtype=int),
        np.array(camera_index, dtype=int),
        np.array(pixels, dtype=float).reshape(-1, 2),
        np.array(known, dtype=float).reshape(-1, 3),
    )


def _triangulate_wand(frame: Frame, cameras: list[TrackingCamera]) -> np.ndarray | None:
    """Triangulate the wand in one frame.

    Args:
        frame: One frame of a wand run.
        cameras: Calibrated cameras.

    Returns:
        The wand position, shape (3,), arena frame (m), or None when fewer than two
        cameras saw it.
    """
    seen = _wand_pixels(frame, cameras)
    if len(seen) < MIN_CAMERAS:
        return None
    origins_m = np.array([cameras[index].pose.position_m for index, _, _ in seen])
    directions = np.array(
        [
            pixel_to_ray(cameras[index].pose, cameras[index].model, np.array([[u_px, v_px]]))[0]
            for index, u_px, v_px in seen
        ]
    )
    return closest_point_to_rays(origins_m, directions)


def _settled_stop(config: Config, t_s: float) -> int | None:
    """Tell which stop point a time belongs to, once the robot has had time to arrive.

    Args:
        config: Configuration (wand schedule).
        t_s: Simulation time (s).

    Returns:
        The index of the stop point, or None outside the settled part of a stop.
    """
    settings = config.calibration
    phase, index, since_s = wand_phase(settings, t_s)
    if phase != STOP or since_s < SETTLED_SHARE * settings.stop_dwell_s:
        return None
    return index


def robot_samples(
    config: Config, cameras: list[TrackingCamera], frames: list[Frame], poses: list[RobotPose]
) -> list[RobotSample]:
    """Pair the wand position at every stop point with the robot's axes there.

    Args:
        config: Configuration (wand schedule).
        cameras: Calibrated cameras, used to triangulate the wand.
        frames: Frames of the run.
        poses: `robot_pose` messages of the run.

    Returns:
        One sample per stop point for which both the wand and the robot were observed:
        the mean wand position and the mean axis readings over the settled part of the stop.
    """
    points: dict[int, list[np.ndarray]] = {}
    axes: dict[int, list[tuple[float, float, float]]] = {}
    for frame in frames:
        stop = _settled_stop(config, frame.t_s)
        point_m = None if stop is None else _triangulate_wand(frame, cameras)
        if point_m is not None:
            points.setdefault(stop, []).append(point_m)
    for pose in poses:
        stop = _settled_stop(config, pose.t)
        if stop is not None:
            axes.setdefault(stop, []).append((pose.rail_m, pose.tilt_rad, pose.focus_m))
    samples = []
    for stop in sorted(points.keys() & axes.keys()):
        point_m = np.mean(points[stop], axis=0)
        rail_m, tilt_rad, focus_m = np.mean(axes[stop], axis=0)
        samples.append(
            RobotSample(tuple(float(x) for x in point_m), float(rail_m), float(tilt_rad), float(focus_m))
        )
    return samples


def calibrate(
    config: Config, robot: Robot, frames: list[Frame], poses: list[RobotPose]
) -> Calibration:
    """Compute the calibration from a recorded wand run.

    Args:
        config: Configuration with the nominal cameras and the wand schedule.
        robot: Robot description from the robot file (start values of the fit).
        frames: All frames of the run.
        poses: All `robot_pose` messages of the run.

    Returns:
        The camera poses, the robot's rail start, lens offset and focus scale.

    Raises:
        ValueError: If the run does not hold enough observations (see `solve_cameras`
            and `fit_robot`).
    """
    nominal = tracking_cameras(config)
    cameras, _ = solve_cameras(nominal, wand_data(config, nominal, frames))
    fitted, focus_scale = fit_robot(robot, robot_samples(config, cameras, frames, poses))
    return Calibration(
        {camera.id: camera.pose for camera in cameras},
        fitted.rail_start_m,
        fitted.lens_offset_m,
        focus_scale,
    )


def apply_calibration(
    cameras: list[TrackingCamera], robot: Robot, calibration: Calibration
) -> tuple[list[TrackingCamera], Robot]:
    """Replace nominal camera poses and robot position by the calibrated ones.

    Args:
        cameras: Nominal cameras from the config.
        robot: Robot description from the robot file.
        calibration: A calibration for the same cameras.

    Returns:
        The cameras with calibrated poses (same order) and the robot with the fitted rail
        start and lens offset.

    Raises:
        ValueError: If the calibration lacks one of the cameras.
    """
    missing = [camera.id for camera in cameras if camera.id not in calibration.cameras]
    if missing:
        raise ValueError(f"calibration has no pose for {', '.join(missing)}")
    calibrated = [replace(camera, pose=calibration.cameras[camera.id]) for camera in cameras]
    fitted = replace(
        robot, rail_start_m=calibration.rail_start_m, lens_offset_m=calibration.lens_offset_m
    )
    return calibrated, fitted


def save_calibration(path: Path, calibration: Calibration) -> None:
    """Write a calibration as JSON.

    Args:
        path: File to write; its folder is created.
        calibration: The calibration (metres, arena frame).
    """
    cameras = {
        camera_id: {"position_m": pose.position_m.tolist(), "rotation": pose.rotation.tolist()}
        for camera_id, pose in calibration.cameras.items()
    }
    content = {
        "cameras": cameras,
        "robot": {
            "rail_start_m": list(calibration.rail_start_m),
            "lens_offset_m": calibration.lens_offset_m,
            "focus_scale": calibration.focus_scale,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(content, indent=1), encoding="utf-8")


def load_calibration(path: Path) -> Calibration:
    """Read a calibration written by `save_calibration`.

    Args:
        path: The JSON file.

    Returns:
        The calibration.

    Raises:
        ValueError: If the file is not a calibration file.
    """
    try:
        content = json.loads(path.read_text(encoding="utf-8"))
        cameras = {
            camera_id: CameraPose(
                np.array(entry["position_m"], dtype=float).reshape(3),
                np.array(entry["rotation"], dtype=float).reshape(3, 3),
            )
            for camera_id, entry in content["cameras"].items()
        }
        robot = content["robot"]
        return Calibration(
            cameras,
            tuple(float(x) for x in robot["rail_start_m"]),
            float(robot["lens_offset_m"]),
            float(robot["focus_scale"]),
        )
    except (KeyError, TypeError, AttributeError, ValueError) as error:
        raise ValueError(f"{path} is not a calibration file: {error!r}") from error


def format_calibration(config: Config, robot: Robot, calibration: Calibration) -> str:
    """Describe how far the calibration moved every camera and the robot.

    Args:
        config: Configuration with the nominal cameras.
        robot: Robot description from the robot file.
        calibration: The calibration.

    Returns:
        One line per camera (shift in mm, rotation in degrees) and one for the robot.
    """
    lines = []
    for camera in tracking_cameras(config):
        distance_m, angle_rad = pose_difference(camera.pose, calibration.cameras[camera.id])
        lines.append(
            f"{camera.id}: moved {distance_m * MM_PER_M:.2f} mm, "
            f"turned {math.degrees(angle_rad):.2f} deg from nominal"
        )
    shift_m = np.array(calibration.rail_start_m) - np.array(robot.rail_start_m)
    lines.append(
        f"robot: rail start shifted by {np.round(shift_m * MM_PER_M, 2).tolist()} mm, "
        f"lens offset {calibration.lens_offset_m * MM_PER_M:.2f} mm, "
        f"focus scale {calibration.focus_scale:.4f}"
    )
    return "\n".join(lines)


def _bound_socket(port: int) -> socket.socket:
    """Open a UDP socket for incoming messages.

    Args:
        port: UDP port to bind on the local host.

    Returns:
        The bound socket with a large receive buffer.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
    sock.bind((UDP_HOST, port))
    return sock


def _robot_pose(datagram: bytes) -> RobotPose | None:
    """Decode a datagram if it is a `robot_pose`.

    Args:
        datagram: One UDP datagram.

    Returns:
        The message, or None for anything else.
    """
    try:
        message = decode(datagram)
    except ProtocolError:
        return None
    return message if isinstance(message, RobotPose) else None


def collect_run(config: Config) -> tuple[list[Frame], list[RobotPose]]:
    """Listen until the wand run is over and return what was received.

    Args:
        config: Configuration (ports, cameras, wand schedule).

    Returns:
        All frames up to the last frame of the run, and all `robot_pose` messages.

    The run is over when its last frame has come out of the assembler. If that frame
    stays incomplete (a lost datagram), a `robot_pose` from the end of the run ends the
    listening instead, once no further datagram is waiting.
    """
    period_s = 1.0 / frame_rate_hz(config)
    assembler = FrameAssembler(
        [camera.id for camera in config.cameras], config.tracker.frame_timeout_frames * period_s
    )
    duration_s = wand_duration_s(config.calibration)
    last_frame = last_frame_of(config, duration_s)
    robot_done = False
    frames: list[Frame] = []
    poses: list[RobotPose] = []
    with (
        _bound_socket(config.ports.detections) as detections,
        _bound_socket(config.ports.robot_pose) as robot_poses,
        selectors.DefaultSelector() as selector,
    ):
        selector.register(detections, selectors.EVENT_READ)
        selector.register(robot_poses, selectors.EVENT_READ)
        while not frames or frames[-1].frame < last_frame:
            # After the robot's last pose only what is already waiting is still read.
            events = selector.select(timeout=0 if robot_done else None)
            if robot_done and not events:
                break
            for key, _ in events:
                datagram = key.fileobj.recv(MAX_DATAGRAM_BYTES)
                if key.fileobj is detections:
                    frames.extend(frames_from_datagram(assembler, datagram))
                elif (pose := _robot_pose(datagram)) is not None:
                    poses.append(pose)
                    robot_done = robot_done or pose.t >= duration_s - period_s
    return frames, poses


def main() -> None:
    """Collect a wand run, compute the calibration, write it and print what changed."""
    parser = argparse.ArgumentParser(description="Calibration from a wand run")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--robot", type=Path, default=DEFAULT_ROBOT_PATH, help="robot file")
    parser.add_argument(
        "--out", type=Path, default=DEFAULT_CALIBRATION_PATH, help="calibration file to write"
    )
    args = parser.parse_args()
    config = load_config(args.config)
    robot = load_robot(args.robot)
    print("waiting for the wand run (start the simulator with --wand) ...")
    try:
        frames, poses = collect_run(config)
    except KeyboardInterrupt:
        return
    calibration = calibrate(config, robot, frames, poses)
    save_calibration(args.out, calibration)
    print(format_calibration(config, robot, calibration))
    print(f"written to {args.out}")


if __name__ == "__main__":
    main()
