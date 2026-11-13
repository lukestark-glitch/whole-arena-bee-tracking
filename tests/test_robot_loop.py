"""Closed loop in one process: simulator, tracker and robot emulator talk through messages."""

from pathlib import Path

from experiments.metrics import RobotMetrics, compute_robot_metrics
from shared.config import frame_rate_hz, load_config, load_robot
from shared.protocol import Detections, RobotPose, Truth, decode, encode_target
from simulator.main import apply_target, create_simulation, step_simulation
from tracker.main import apply_robot_pose, create_tracker, process_frame, target_message
from tracker.receiver import FrameAssembler

REPO = Path(__file__).parent.parent
CONFIG = load_config(REPO / "config" / "default.yaml")
ROBOT = load_robot(REPO / "config" / "robot.yaml")
RUN_FRAMES = 600  # 10 s of simulation time
MATCH_GATE_M = 0.02


def _run_closed_loop(seed: int) -> tuple[RobotMetrics, set[int]]:
    """Run simulator, tracker (auto-select) and robot; every message passes the protocol.

    Returns:
        The robot metrics of the run and the set of track ids that were ever selected.
    """
    simulation = create_simulation(CONFIG, ROBOT, seed, 1.0)
    period_s = 1.0 / frame_rate_hz(CONFIG)
    assembler = FrameAssembler(
        [camera.id for camera in CONFIG.cameras], CONFIG.tracker.frame_timeout_frames * period_s
    )
    tracker = create_tracker(CONFIG, ROBOT, auto_select=True)
    truth, axes, tracks, selection = {}, {}, {}, {}
    for _ in range(RUN_FRAMES):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if isinstance(message, Truth):
                truth.setdefault(message.frame, {}).update({b.id: b.pos for b in message.bees})
                axes[message.frame] = message.robot
            elif isinstance(message, RobotPose):
                apply_robot_pose(tracker, item.datagram)
            elif isinstance(message, Detections):
                for frame in assembler.add(message):
                    record = process_frame(tracker, frame)
                    tracks[record["frame"]] = {
                        t["id"]: tuple(t["pos"]) for t in record["tracks"] if t["confirmed"]
                    }
                    if record["selected"] is not None:
                        selection[record["frame"]] = record["selected"]
                    target = target_message(tracker)
                    if target is not None:
                        apply_target(simulation, encode_target(target))
    metrics = compute_robot_metrics(truth, axes, tracks, selection, ROBOT, MATCH_GATE_M)
    return metrics, set(selection.values())


def test_robot_follows_the_selected_bee() -> None:
    """With a selected bee the cinema camera frames it most of the time, among ten bees."""
    metrics, selected_ids = _run_closed_loop(1)
    assert len(selected_ids) == 1
    assert metrics.frames > RUN_FRAMES // 2
    assert metrics.in_frame_share > 0.5
    assert metrics.in_focus_share > 0.1


def test_closed_loop_repeats_with_the_same_seed() -> None:
    """The same seed gives the same robot metrics."""
    assert _run_closed_loop(2)[0] == _run_closed_loop(2)[0]
