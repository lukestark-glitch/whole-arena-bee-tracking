"""Tests of the robot controller: selection rule, following, fallback and hold."""

from dataclasses import replace
from pathlib import Path

import numpy as np

from robot_emulator.controller import (
    Controller,
    apply_exposure,
    create_controller,
    pose_message,
    receive_target,
    select_blob,
    step_controller,
)
from robot_emulator.stereo import Blob, observe_stereo
from shared.config import Robot, load_robot
from shared.geometry import cinema_view, robot_camera_pose
from shared.protocol import OtherBee, Target

ROBOT = load_robot(Path("config/robot.yaml"))
BODY_LENGTH_M = 0.015
FRAME_S = 1.0 / 60.0
STEPS_PER_FRAME = 8
STEP_S = FRAME_S / STEPS_PER_FRAME
CENTRE_PX = np.array([640.0, 400.0])
NO_OTHERS_PX = np.zeros((0, 2))


def _target(t_s: float, pos: np.ndarray, vel: np.ndarray, others: tuple = ()) -> Target:
    """Build a target message of track 1 for a bee at `pos` (m) moving with `vel` (m/s)."""
    return Target(
        t=t_s,
        track_id=1,
        pos=tuple(pos),
        vel=tuple(vel),
        acc=(0.0, 0.0, 0.0),
        distance_m=0.3,
        state="flying",
        confidence=1.0,
        others=others,
    )


def _run(
    robot: Robot,
    seconds: float,
    bee_at,
    intruder_at=None,
    report_intruder: bool = False,
) -> tuple[Controller, list[float]]:
    """Run the robot against a perfect tracker that is one frame late.

    Args:
        robot: Robot description.
        seconds: Simulation time to run (s).
        bee_at: Function t_s -> (position, velocity) of the followed bee.
        intruder_at: The same for a second bee the robot must not follow, or None.
        report_intruder: Whether the tracker lists the intruder in `others`.

    Returns:
        The controller and, per frame, the sideways miss |rail camera x - bee x| in m.
    """
    controller = create_controller(robot)
    rng = np.random.default_rng(3)
    misses_m = []
    for frame in range(int(seconds / FRAME_S)):
        t_s = frame * FRAME_S
        pos, _ = bee_at(t_s)
        positions = [pos]
        if frame > 0:
            late_pos, late_vel = bee_at(t_s - FRAME_S)
            others = ()
            if intruder_at is not None and report_intruder:
                others = (OtherBee(2, tuple(intruder_at(t_s - FRAME_S)[0])),)
            receive_target(controller, _target(t_s - FRAME_S, late_pos, late_vel, others))
        if intruder_at is not None:
            positions.append(intruder_at(t_s)[0])
        pose = robot_camera_pose(robot, controller.rail.position, controller.tilt.position)
        blobs = observe_stereo(pose, robot.stereo, np.array(positions), BODY_LENGTH_M, rng)
        apply_exposure(controller, t_s, blobs)
        misses_m.append(abs(pose.position_m[0] - pos[0]))
        for step in range(STEPS_PER_FRAME):
            step_controller(controller, t_s + (step + 1) * STEP_S, STEP_S)
    return controller, misses_m


def _slow_bee(t_s: float) -> tuple[np.ndarray, np.ndarray]:
    """A bee flying along +x at 0.1 m/s, 0.2 m into the arena, slightly above the rail."""
    velocity = np.array([0.1, 0.0, 0.0])
    return np.array([0.3, 0.2, 0.2]) + velocity * t_s, velocity


def _crossing_bee(t_s: float) -> tuple[np.ndarray, np.ndarray]:
    """A nearer (so larger) bee flying along -x through the line of sight to the slow bee."""
    velocity = np.array([-0.2, 0.0, 0.0])
    return np.array([0.75, 0.05, 0.1786]) + velocity * t_s, velocity


def test_selection_takes_the_blob_at_the_prediction() -> None:
    """A blob near the predicted point with the expected distance is accepted."""
    blob = Blob(645.0, 398.0, 30.0, 0.31)
    assert select_blob([blob], CENTRE_PX, 0.3, NO_OTHERS_PX, ROBOT.control) is blob


def test_selection_ignores_blobs_outside_the_window() -> None:
    """A blob farther than the window half side from the prediction is not looked at."""
    blob = Blob(640.0 + ROBOT.control.search_window_px + 1.0, 400.0, 30.0, 0.3)
    assert select_blob([blob], CENTRE_PX, 0.3, NO_OTHERS_PX, ROBOT.control) is None


def test_selection_rejects_a_wrong_distance_and_does_not_take_the_next_blob() -> None:
    """When the nearest blob has the wrong distance, no blob is accepted at all."""
    near_wrong = Blob(642.0, 400.0, 60.0, 0.15)
    far_right = Blob(670.0, 400.0, 30.0, 0.3)
    assert select_blob([near_wrong, far_right], CENTRE_PX, 0.3, NO_OTHERS_PX, ROBOT.control) is None


def test_selection_rejects_a_blob_explained_by_another_bee() -> None:
    """A blob that lies nearer to a known other bee than to the prediction is rejected."""
    blob = Blob(660.0, 400.0, 30.0, 0.3)
    others_px = np.array([[665.0, 400.0]])
    assert select_blob([blob], CENTRE_PX, 0.3, others_px, ROBOT.control) is None


def test_robot_swings_to_the_bee_and_follows_it() -> None:
    """After the swing the bee stays in the cinema frame and in focus."""
    controller, misses_m = _run(ROBOT, 3.0, _slow_bee)
    assert max(misses_m[60:]) < 0.002
    pos, _ = _slow_bee(3.0)
    pose = robot_camera_pose(ROBOT, controller.rail.position, controller.tilt.position)
    in_frame, depth_m = cinema_view(pose, ROBOT.cinema, np.array([pos]))
    assert in_frame[0]
    assert abs(depth_m[0] - controller.focus.position) < ROBOT.cinema.depth_of_field_m


def test_larger_bee_crossing_the_window_is_not_followed() -> None:
    """A larger bee crossing the search window does not pull the robot off its bee."""
    for report_intruder in (False, True):
        _, misses_m = _run(ROBOT, 3.0, _slow_bee, _crossing_bee, report_intruder)
        assert max(misses_m[60:]) < 0.002


def test_crossing_bee_is_the_largest_blob_in_the_window() -> None:
    """The crossing scenario would fool the old rule: the intruder is the larger blob."""
    t_s = 1.5  # the moment both bees have the same x
    pos, _ = _slow_bee(t_s)
    pose = robot_camera_pose(ROBOT, pos[0] - ROBOT.rail_start_m[0], np.arctan2(0.05, 0.35))
    quiet = replace(ROBOT.stereo, pixel_sigma_px=0.0, size_sigma_px=0.0, disparity_sigma_px=0.0)
    positions = np.array([pos, _crossing_bee(t_s)[0]])
    own, intruder = observe_stereo(pose, quiet, positions, BODY_LENGTH_M, np.random.default_rng(0))
    assert intruder.size_px > own.size_px
    assert abs(intruder.u_px - own.u_px) < ROBOT.control.search_window_px
    assert abs(intruder.v_px - own.v_px) < ROBOT.control.search_window_px


def test_focus_presets_from_distance_before_the_first_blob() -> None:
    """Without an accepted blob the focus is driven to `distance_m` of the target."""
    controller = create_controller(ROBOT)
    receive_target(controller, _target(0.0, np.array([0.45, 0.15, 0.15]), np.zeros(3)))
    for step in range(200):
        step_controller(controller, (step + 1) * STEP_S, STEP_S)
    assert abs(controller.focus.position - 0.3) < 1e-3


def test_robot_holds_position_after_the_target_falls_silent() -> None:
    """Half a second after the last target the axes stand still."""
    controller = create_controller(ROBOT)
    receive_target(controller, _target(0.0, np.array([0.8, 0.15, 0.25]), np.array([0.5, 0.0, 0.0])))
    steps = int(1.0 / STEP_S)
    for step in range(steps):
        step_controller(controller, (step + 1) * STEP_S, STEP_S)
    assert controller.rail.velocity == 0.0
    assert controller.tilt.velocity == 0.0
    before = pose_message(controller, 1.0)
    step_controller(controller, 1.0 + STEP_S, STEP_S)
    assert pose_message(controller, 1.0) == before


def test_older_target_is_dropped_and_a_new_track_resets_the_lock() -> None:
    """Targets are ordered by time; a change of track drops the stereo lock."""
    controller = create_controller(ROBOT)
    newest = _target(1.0, np.array([0.4, 0.1, 0.1]), np.zeros(3))
    receive_target(controller, newest)
    receive_target(controller, _target(0.5, np.array([0.1, 0.1, 0.1]), np.zeros(3)))
    assert controller.target is newest
    controller.locked = True
    receive_target(controller, replace(newest, t=1.1, track_id=2))
    assert not controller.locked
