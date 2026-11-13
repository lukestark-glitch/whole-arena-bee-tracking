"""Tests for frame assembly, restart detection and datagram decoding in tracker/receiver.py."""

from shared.protocol import (
    Detections,
    RobotPose,
    encode_detections,
    encode_robot_pose,
)
from tracker.receiver import FrameAssembler, frames_from_datagram

PERIOD_S = 1 / 60
TIMEOUT_S = 2 * PERIOD_S
CAMERAS = ["c1", "c2", "c3"]


def msg(camera: str, frame: int, bees=((1.0, 2.0, 3.0),), t: float | None = None) -> Detections:
    """Build a detections message with t derived from the frame number."""
    return Detections(
        node_id="n1",
        camera_id=camera,
        frame=frame,
        t=frame * PERIOD_S if t is None else t,
        bees=tuple(bees),
        markers=(),
    )


def make() -> FrameAssembler:
    """Create an assembler for the three test cameras."""
    return FrameAssembler(CAMERAS, TIMEOUT_S)


def test_complete_frame_released_with_all_detections() -> None:
    """A frame is released when the last camera reports, with all detections."""
    a = make()
    assert a.add(msg("c1", 1, [(1.0, 1.0, 1.0)])) == []
    assert a.add(msg("c2", 1, [(2.0, 2.0, 2.0)])) == []
    frames = a.add(msg("c3", 1, [(3.0, 3.0, 3.0)]))
    assert len(frames) == 1
    assert frames[0].frame == 1
    assert frames[0].t_s == PERIOD_S
    assert frames[0].detections == {
        "c1": ((1.0, 1.0, 1.0),),
        "c2": ((2.0, 2.0, 2.0),),
        "c3": ((3.0, 3.0, 3.0),),
    }


def test_cameras_in_any_order() -> None:
    """The arrival order of cameras does not matter."""
    a = make()
    assert a.add(msg("c3", 1)) == []
    assert a.add(msg("c1", 1)) == []
    assert len(a.add(msg("c2", 1))) == 1


def test_missing_camera_released_after_timeout_only() -> None:
    """An incomplete frame is released at t + timeout and not before."""
    a = make()
    a.add(msg("c1", 1))
    a.add(msg("c2", 1))
    assert a.add(msg("c1", 2)) == []  # exactly one period newer
    frames = a.add(msg("c1", 3))  # exactly two periods newer
    assert [f.frame for f in frames] == [1]
    assert set(frames[0].detections) == {"c1", "c2"}


def test_frames_released_in_order() -> None:
    """A complete frame 5 flushes the incomplete frame 4 first."""
    a = make()
    a.add(msg("c1", 4))
    a.add(msg("c2", 4))
    a.add(msg("c1", 5))
    a.add(msg("c2", 5))
    frames = a.add(msg("c3", 5))
    assert [f.frame for f in frames] == [4, 5]
    assert set(frames[0].detections) == {"c1", "c2"}
    assert set(frames[1].detections) == set(CAMERAS)


def test_late_message_dropped() -> None:
    """A message for an already released frame is dropped."""
    a = make()
    for camera in CAMERAS:
        a.add(msg(camera, 3))
    assert a.add(msg("c1", 3)) == []
    assert a.add(msg("c1", 2)) == []
    assert a.add(msg("c2", 4)) == []  # frame 4 must start fresh, with one camera only


def test_unknown_camera_dropped() -> None:
    """A message from an unknown camera is ignored."""
    a = make()
    for camera in CAMERAS[:2]:
        a.add(msg(camera, 1))
    assert a.add(msg("zz", 1)) == []
    assert len(a.add(msg("c3", 1))) == 1


def test_parts_joined_in_order() -> None:
    """Two parts of the same camera and frame are concatenated in arrival order."""
    a = make()
    a.add(msg("c1", 1, [(1.0, 1.0, 1.0)]))
    a.add(msg("c1", 1, [(2.0, 2.0, 2.0)]))
    a.add(msg("c2", 1))
    frames = a.add(msg("c3", 1))
    assert frames[0].detections["c1"] == ((1.0, 1.0, 1.0), (2.0, 2.0, 2.0))


def test_empty_bees_counts_as_reported() -> None:
    """A camera that sees nothing still completes the frame."""
    a = make()
    a.add(msg("c1", 1, []))
    a.add(msg("c2", 1))
    frames = a.add(msg("c3", 1))
    assert len(frames) == 1
    assert frames[0].detections["c1"] == ()


def test_frames_from_datagram() -> None:
    """A valid datagram is assembled; garbage and non-detections give no frames."""
    a = make()
    pose = RobotPose(t=0.0, rail_m=0.0, tilt_rad=0.0, focus_m=0.1)
    assert frames_from_datagram(a, b"garbage") == []
    assert frames_from_datagram(a, encode_robot_pose(pose)) == []
    frames: list = []
    for camera in CAMERAS:
        for datagram in encode_detections(msg(camera, 1)):
            frames.extend(frames_from_datagram(a, datagram))
    assert [f.frame for f in frames] == [1]
    assert set(frames[0].detections) == set(CAMERAS)


def _release_frames(a: FrameAssembler, numbers) -> list[int]:
    """Send all cameras for each frame number; return the released frame numbers."""
    released: list[int] = []
    for number in numbers:
        for camera in CAMERAS:
            released.extend(f.frame for f in a.add(msg(camera, number)))
    return released


def test_restart_starts_a_new_run() -> None:
    """After frames up to 500, frame 0 starts a new run and frames 0, 1, 2 come out again."""
    a = make()
    _release_frames(a, range(501))
    assert _release_frames(a, range(3)) == [0, 1, 2]


def test_straggler_is_not_a_restart() -> None:
    """A message 3 frames late is dropped and does not reset the assembler."""
    a = make()
    _release_frames(a, range(100))
    assert a.add(msg("c1", 96)) == []
    assert a.add(msg("c1", 100)) == []
    assert _release_frames(a, [101]) == [100, 101]


def test_last_report_per_camera() -> None:
    """last_report_s holds the newest t per camera, omits silent ones, clears on restart."""
    a = make()
    a.add(msg("c1", 1))
    a.add(msg("c1", 5))
    a.add(msg("c2", 2))
    assert a.last_report_s() == {"c1": 5 * PERIOD_S, "c2": 2 * PERIOD_S}
    _release_frames(a, range(200))
    a.add(msg("c3", 0))
    assert a.last_report_s() == {"c3": 0.0}


def test_timeout_is_not_delayed_by_rounded_times() -> None:
    """With times rounded to microseconds a frame still times out after two periods."""
    assembler = FrameAssembler(["cam-01", "cam-02"], 2.0 / 60.0)
    late_by = []
    for frame in range(300):
        message = Detections("node-01", "cam-01", frame, round(frame / 60.0, 6), ())
        late_by.extend(frame - released.frame for released in assembler.add(message))
    assert late_by and set(late_by) == {2}
