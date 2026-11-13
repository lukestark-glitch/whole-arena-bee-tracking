"""Control software of the emulated robot: follow the bee the tracker names.

This module plays the part of the real robot's software as specified in
`docs/robot-interface.md`. It receives `target` messages, extrapolates them to the
present, looks for the bee in a small window of the stereo image, and commands rail, tilt
and focus with feed-forward from the bee's velocity. It never decides on its own which bee
to follow: a blob that fails the selection rule is ignored and the prediction is followed.

It knows only what a real robot would know: `target`, its stereo blobs and its own axes.
The simulator calls `apply_exposure` for every stereo exposure and `step_controller` at
the control rate, and sends the `robot_pose` built by `pose_message`.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from robot_emulator.motion import Axis, step_axis
from robot_emulator.stereo import Blob
from shared.config import AxisLimits, Robot, RobotControl
from shared.geometry import pixel_to_ray, project, robot_camera_pose
from shared.protocol import RobotPose, Target

_AXES = 3  # x, y, z


@dataclass
class Controller:
    """State of the robot: its axes and what it knows about the target.

    Attributes:
        robot: Robot description (limits, cameras, control settings).
        rail: Rail axis (m, m/s), position from the rail start.
        tilt: Tilt axis (rad, rad/s), 0 = horizontal, positive = up.
        focus: Focus axis (m, m/s), distance of the focus plane from the lens reference.
        target: The newest `target` message, or None before the first one.
        locked: True once a stereo blob was accepted for the current track; focus then
            follows the stereo measurement instead of the preset `distance_m`.
        correction_m: Shape (3,), arena frame, metres: position measured by the stereo
            camera minus the predicted position, from the last accepted blob.
    """

    robot: Robot
    rail: Axis
    tilt: Axis
    focus: Axis
    target: Target | None = None
    locked: bool = False
    correction_m: np.ndarray = field(default_factory=lambda: np.zeros(_AXES))


def _middle(limits: AxisLimits) -> Axis:
    """Create an axis at rest in the middle of its travel.

    Args:
        limits: Travel of the axis (m or rad).

    Returns:
        The axis.
    """
    return Axis((limits.lower + limits.upper) / 2.0, 0.0)


def create_controller(robot: Robot) -> Controller:
    """Create a robot at rest without a target.

    Args:
        robot: Robot description.

    Returns:
        The controller, every axis in the middle of its travel (the real home position
        is not known yet).
    """
    return Controller(robot, _middle(robot.rail), _middle(robot.tilt), _middle(robot.focus))


def receive_target(controller: Controller, target: Target) -> None:
    """Take in a `target` message; an older one than the current is dropped.

    On a change of track the stereo lock and correction are dropped, so focus presets
    from `distance_m` while the camera swings over.

    Args:
        controller: Controller, modified in place.
        target: Decoded message; position in the arena frame (m), time in simulation s.
    """
    current = controller.target
    # UDP may reorder; the protocol orders messages by t, not by arrival.
    if current is not None and target.t <= current.t:
        return
    if current is None or current.track_id != target.track_id:
        controller.locked = False
        controller.correction_m = np.zeros(_AXES)
    controller.target = target


def has_target(controller: Controller, t_s: float) -> bool:
    """Tell whether the robot has a target that is fresh enough to follow.

    Args:
        controller: Controller.
        t_s: Current simulation time (s).

    Returns:
        False before the first `target` and after a silence of `target_timeout_s`.
    """
    target = controller.target
    return target is not None and t_s - target.t <= controller.robot.control.target_timeout_s


def predict_target(target: Target, t_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Extrapolate a target to a later time with constant acceleration.

    Args:
        target: The message; valid for its time `t`.
        t_s: Time to predict for (simulation s).

    Returns:
        Position (m) and velocity (m/s), each shape (3,), arena frame.
    """
    dt_s = t_s - target.t
    velocity_m_s = np.array(target.vel)
    acceleration_m_s2 = np.array(target.acc)
    # Constant acceleration: p + v dt + a dt^2 / 2, and v + a dt.
    position_m = np.array(target.pos) + velocity_m_s * dt_s + acceleration_m_s2 * dt_s**2 / 2.0
    return position_m, velocity_m_s + acceleration_m_s2 * dt_s


def select_blob(
    blobs: list[Blob],
    predicted_px: np.ndarray,
    expected_distance_m: float,
    others_px: np.ndarray,
    control: RobotControl,
) -> Blob | None:
    """Apply the selection rule of `docs/robot-interface.md` inside the search window.

    Only the blob nearest to the predicted point is considered. It is accepted when its
    measured distance fits and no other known bee explains it better. Blob size plays no
    part, so a larger bee crossing the window cannot take over.

    Args:
        blobs: Blobs of one stereo exposure.
        predicted_px: Shape (2,), predicted image position (u, v) of the target (px).
        expected_distance_m: Expected distance of the target from the camera (m).
        others_px: Shape (M, 2), predicted image positions of the other known bees (px).
        control: Search window half side (px) and distance tolerance (m).

    Returns:
        The accepted blob, or None when no blob qualifies.
    """
    in_window = [
        blob
        for blob in blobs
        if abs(blob.u_px - predicted_px[0]) <= control.search_window_px
        and abs(blob.v_px - predicted_px[1]) <= control.search_window_px
    ]
    if not in_window:
        return None
    nearest = min(in_window, key=lambda blob: _pixel_distance_px(blob, predicted_px))
    if abs(nearest.distance_m - expected_distance_m) > control.distance_tolerance_m:
        return None
    # The blob belongs to another bee when that bee is expected nearer to it than the target.
    own_px = _pixel_distance_px(nearest, predicted_px)
    if any(_pixel_distance_px(nearest, other_px) < own_px for other_px in others_px):
        return None
    return nearest


def _pixel_distance_px(blob: Blob, point_px: np.ndarray) -> float:
    """Image distance between a blob and a point.

    Args:
        blob: A stereo blob.
        point_px: Shape (2,), (u, v) in px.

    Returns:
        The distance in px.
    """
    return math.hypot(blob.u_px - point_px[0], blob.v_px - point_px[1])


def apply_exposure(controller: Controller, t_s: float, blobs: list[Blob]) -> None:
    """Use one stereo exposure to refine where the target is.

    The accepted blob gives a measured position (its viewing ray times its measured
    distance). The difference to the prediction is kept as `correction_m`. When no blob
    qualifies the old correction stays: the robot follows the prediction and keeps the
    last offset in centring and focus.

    Args:
        controller: Controller, modified in place.
        t_s: Time of the exposure (simulation s).
        blobs: Blobs the stereo camera reports for this exposure.
    """
    if not has_target(controller, t_s):
        return
    target = controller.target
    model = controller.robot.stereo.model
    pose = robot_camera_pose(controller.robot, controller.rail.position, controller.tilt.position)
    predicted_m, _ = predict_target(target, t_s)
    points_m = np.array([predicted_m] + [other.pos for other in target.others])
    pixels_px = project(pose, model, points_m)
    # The distance is recomputed for the predicted position and the present pose; it is
    # the same quantity as `distance_m`, which was valid at the target's time.
    expected_m = float(np.linalg.norm(predicted_m - pose.position_m))
    blob = select_blob(blobs, pixels_px[0], expected_m, pixels_px[1:], controller.robot.control)
    if blob is None:
        return
    ray = pixel_to_ray(pose, model, np.array([[blob.u_px, blob.v_px]]))[0]
    controller.correction_m = pose.position_m + ray * blob.distance_m - predicted_m
    controller.locked = True


def _commands(controller: Controller, t_s: float) -> tuple[tuple[float, float], ...]:
    """Compute position and feed-forward velocity for rail, tilt and focus.

    Args:
        controller: Controller with a fresh target.
        t_s: Time the commands are for (simulation s).

    Returns:
        (position, velocity) for rail (m, m/s), tilt (rad, rad/s) and focus (m, m/s).
    """
    robot = controller.robot
    predicted_m, velocity_m_s = predict_target(controller.target, t_s)
    aim_m = predicted_m + controller.correction_m - np.array(robot.rail_start_m)
    along_m, up_m = aim_m[1], aim_m[2]
    range_m = math.hypot(along_m, up_m)
    rail = (aim_m[0], velocity_m_s[0])
    # Rate of atan2(up, along): (along * d(up)/dt - up * d(along)/dt) / range^2.
    tilt_rate_rad_s = (along_m * velocity_m_s[2] - up_m * velocity_m_s[1]) / range_m**2
    tilt = (math.atan2(up_m, along_m), tilt_rate_rad_s)
    if not controller.locked:
        # Focus preset: no stereo measurement of this bee yet, so trust the tracker.
        return rail, tilt, (controller.target.distance_m, 0.0)
    # Rate of the range: the velocity component along the line of sight.
    range_rate_m_s = (along_m * velocity_m_s[1] + up_m * velocity_m_s[2]) / range_m
    return rail, tilt, (range_m - robot.lens_offset_m, range_rate_m_s)


def step_controller(controller: Controller, t_s: float, dt_s: float) -> None:
    """Advance the axes by one control step that ends at `t_s`.

    With a fresh target every axis is commanded to where the bee will be at `t_s`, with
    the bee's velocity as feed-forward. Without one the axes brake and hold position.

    Args:
        controller: Controller, modified in place.
        t_s: Simulation time at the end of the step (s).
        dt_s: Length of the step (s).
    """
    robot = controller.robot
    axes = ((controller.rail, robot.rail), (controller.tilt, robot.tilt), (controller.focus, robot.focus))
    if has_target(controller, t_s):
        commands = _commands(controller, t_s)
    else:
        commands = tuple((axis.position, 0.0) for axis, _ in axes)
    for (axis, limits), (position, velocity) in zip(axes, commands):
        step_axis(axis, limits, position, velocity, dt_s)


def pose_message(controller: Controller, t_s: float) -> RobotPose:
    """Build the `robot_pose` message with the true axis positions.

    Args:
        controller: Controller.
        t_s: Time the positions are valid for (simulation s).

    Returns:
        The message (m, rad, m).
    """
    return RobotPose(
        t=t_s,
        rail_m=controller.rail.position,
        tilt_rad=controller.tilt.position,
        focus_m=controller.focus.position,
    )
