"""Tests for the control window: message builders, selection and the WebSocket session."""

import asyncio
import dataclasses
import json
import socket
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import pytest
from websockets.asyncio.client import ClientConnection, connect

import tracker.main as main
from shared.config import Config, frame_rate_hz, last_frame_of, load_config, load_robot
from shared.protocol import Detections, RobotPose, decode, encode_detections
from simulator.main import create_simulation, step_simulation
from tracker.main import (
    Tracker,
    apply_selection,
    create_tracker,
    process_frame,
    scene_message,
    tracks_message,
)
from tracker.receiver import Frame, FrameAssembler

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
ROBOT = load_robot(Path(__file__).parent.parent / "config" / "robot.yaml")
HOST = "127.0.0.1"
TIMEOUT_S = 10.0  # generous, so a busy computer does not fail the test
CONNECT_ATTEMPTS = 100
CONNECT_PAUSE_S = 0.05
RUN_DURATION_S = 100.0  # far beyond what the tests send; they end the run with a late frame
SENT_FRAMES = 150
SILENT_FROM_FRAME = 10  # the silent node stops reporting at this frame
SILENT_TEST_FRAMES = 70  # about 1 s of simulation time, longer than node_silent_after_s


def _feed(frames: int, silent_cameras: frozenset[str] = frozenset()) -> tuple[Tracker, Frame, dict]:
    """Track simulated frames in process; some cameras go silent after a few frames.

    Returns:
        The tracker, the last processed frame and the assembler's last report times.
    """
    simulation = create_simulation(CONFIG, ROBOT, 1, 1.0)
    period_s = 1.0 / frame_rate_hz(CONFIG)
    assembler = FrameAssembler(
        [camera.id for camera in CONFIG.cameras], CONFIG.tracker.frame_timeout_frames * period_s
    )
    tracker = create_tracker(CONFIG, ROBOT)
    last = None
    for _ in range(frames):
        for item in step_simulation(simulation):
            message = decode(item.datagram)
            if not isinstance(message, Detections):
                continue
            if message.camera_id in silent_cameras and message.frame >= SILENT_FROM_FRAME:
                continue
            for frame in assembler.add(message):
                process_frame(tracker, frame)
                last = frame
    return tracker, last, assembler.last_report_s()


def test_scene_message() -> None:
    """The scene lists the cameras with image size and the nodes with their cameras."""
    message = json.loads(scene_message(CONFIG, ROBOT))
    assert message["robot"] == {
        "rail_start_m": list(ROBOT.rail_start_m),
        "rail_length_m": ROBOT.rail.upper,
        "lens_offset_m": ROBOT.lens_offset_m,
    }
    assert message["type"] == "scene" and message["arena_size_m"] == list(CONFIG.arena.size_m)
    assert message["prediction_lead_s"] == CONFIG.tracker.prediction_lead_s
    assert len(message["cameras"]) == 16 and len(message["nodes"]) == 8
    assert set(message["cameras"][0]) == {
        "id", "position_m", "look_at_m", "fov_horizontal_rad", "width_px", "height_px"
    }
    assert set(message["nodes"][0]) == {"id", "cameras"}


def test_tracks_message() -> None:
    """Confirmed tracks are complete, detections are labelled, and no node is silent."""
    tracker, frame, last_report_s = _feed(120)
    message = json.loads(tracks_message(tracker, frame, CONFIG, last_report_s))
    assert message["type"] == "tracks" and message["frame"] == frame.frame
    confirmed = [t for t in message["tracks"] if t["confirmed"]]
    assert len(confirmed) >= 8
    for track in confirmed:
        assert set(track) == {"id", "pos", "vel", "acc", "pred", "state", "confirmed", "cameras"}
        assert len(track["pos"]) == len(track["pred"]) == len(track["acc"]) == 3
    assert len(message["detections"]) == 16
    entries = [e for rows in message["detections"].values() for e in rows]
    assert all(len(e) == 4 for e in entries)
    assert sum(e[3] >= 0 for e in entries) > len(entries) / 2
    assert message["health"]["silent_nodes"] == []
    assert len(message["health"]["camera_age_s"]) == 16
    assert message["selected"] is None
    assert message["robot"] is None
    tracker.robot_pose = RobotPose(t=1.0, rail_m=0.3, tilt_rad=0.1, focus_m=0.25)
    message = json.loads(tracks_message(tracker, frame, CONFIG, last_report_s))
    assert message["robot"] == {"rail_m": 0.3, "tilt_rad": 0.1, "focus_m": 0.25}


def test_silent_node_is_reported() -> None:
    """A node whose two cameras stopped reporting for over 0.5 s is listed as silent."""
    node = CONFIG.nodes[2]
    tracker, frame, last_report_s = _feed(SILENT_TEST_FRAMES, frozenset(node.camera_ids))
    health = json.loads(tracks_message(tracker, frame, CONFIG, last_report_s))["health"]
    assert health["silent_nodes"] == [node.id]
    assert health["camera_age_s"][node.camera_ids[0]] >= CONFIG.tracker.node_silent_after_s


def test_apply_selection() -> None:
    """A valid id and null are accepted; everything else raises ValueError."""
    tracker, _, _ = _feed(60)
    track_id = tracker.track_set.tracks[0].id
    apply_selection(tracker, json.dumps({"type": "select", "track_id": track_id}))
    assert tracker.selected_id == track_id
    apply_selection(tracker, json.dumps({"type": "select", "track_id": None}).encode())
    assert tracker.selected_id is None
    apply_selection(tracker, json.dumps({"type": "select", "track_id": track_id}))
    bad = [
        "not json",
        "[1]",
        json.dumps({"type": "other", "track_id": track_id}),
        json.dumps({"type": "select"}),
        json.dumps({"type": "select", "track_id": True}),
        json.dumps({"type": "select", "track_id": "3"}),
        json.dumps({"type": "select", "track_id": 1.5}),
        json.dumps({"type": "select", "track_id": 10**6}),
        b"\xff",
    ]
    for raw in bad:
        with pytest.raises(ValueError):
            apply_selection(tracker, raw)
    assert tracker.selected_id == track_id


def test_selection_is_cleared_when_the_track_disappears() -> None:
    """After enough empty frames the selected track is gone and the selection with it."""
    tracker, frame, _ = _feed(60)
    track_id = next(t.id for t in tracker.track_set.tracks if t.confirmed)
    apply_selection(tracker, json.dumps({"type": "select", "track_id": track_id}))
    number, period_s = frame.frame, 1.0 / frame_rate_hz(CONFIG)
    for step in range(1, CONFIG.tracker.max_missed_frames + 3):
        process_frame(tracker, Frame(number + step, frame.t_s + step * period_s, {}))
    assert tracker.selected_id is None


def _free_port(kind: int) -> int:
    """Return a port of the given socket kind that is free right now."""
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind((HOST, 0))
        return sock.getsockname()[1]


def _finish_datagrams(config: Config) -> list[bytes]:
    """Datagrams of one late empty frame from every camera; it ends the run."""
    number = last_frame_of(config, RUN_DURATION_S)
    t_s = number / frame_rate_hz(config)
    datagrams: list[bytes] = []
    for camera in config.cameras:
        message = Detections("node-xx", camera.id, number, t_s, (), ())
        datagrams.extend(encode_detections(message))
    return datagrams


@contextmanager
def _running(tmp_path: Path) -> Iterator[Config]:
    """Run the tracker in a thread on free ports; end it with a late frame and join."""
    ports = dataclasses.replace(
        CONFIG.ports,
        detections=_free_port(socket.SOCK_DGRAM),
        control_window=_free_port(socket.SOCK_STREAM),
        robot_pose=_free_port(socket.SOCK_DGRAM),
    )
    config = dataclasses.replace(CONFIG, ports=ports)
    thread = threading.Thread(
        target=main.run, args=(config, ROBOT, tmp_path / "tracks.jsonl", RUN_DURATION_S, False),
        daemon=True,
    )
    thread.start()
    try:
        yield config
    finally:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for datagram in _finish_datagrams(config):
                sock.sendto(datagram, (HOST, config.ports.detections))
        thread.join(TIMEOUT_S)
        assert not thread.is_alive()


async def _connect(port: int) -> ClientConnection:
    """Open a WebSocket to the tracker, retrying until the server is up."""
    for _ in range(CONNECT_ATTEMPTS):
        try:
            return await connect(f"ws://{HOST}:{port}")
        except OSError:
            await asyncio.sleep(CONNECT_PAUSE_S)
    raise AssertionError("server did not come up")


async def _next_tracks(ws: ClientConnection) -> dict:
    """Receive messages until a `tracks` message arrives."""
    while True:
        message = json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
        if message["type"] == "tracks":
            return message


async def _send_frames(config: Config, frames: int) -> None:
    """Send simulated detections datagrams to the tracker, yielding between frames."""
    simulation = create_simulation(config, ROBOT, 1, 1.0)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _ in range(frames):
            for item in step_simulation(simulation):
                if item.port == config.ports.detections:
                    sock.sendto(item.datagram, (HOST, item.port))
            await asyncio.sleep(0.001)


async def _session(config: Config) -> None:
    """Scene, increasing frames, selection, garbage, and a clean close at the end."""
    ws = await _connect(config.ports.control_window)
    async with ws:
        scene = json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
        assert scene["type"] == "scene" and len(scene["cameras"]) == 16
        sender = asyncio.create_task(_send_frames(config, SENT_FRAMES))
        message = await _next_tracks(ws)
        while not any(t["confirmed"] for t in message["tracks"]):
            message = await _next_tracks(ws)
        later = await _next_tracks(ws)
        assert later["frame"] > message["frame"]
        track_id = next(t["id"] for t in message["tracks"] if t["confirmed"])
        await ws.send(json.dumps({"type": "select", "track_id": track_id}))
        await ws.send("garbage")
        while (await _next_tracks(ws))["selected"] != track_id:
            pass
        await ws.send(json.dumps({"type": "select", "track_id": None}))
        await sender
        for datagram in _finish_datagrams(config):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.sendto(datagram, (HOST, config.ports.detections))
        # Keep reading: a client that stops reading never sees the close.
        await asyncio.wait_for(_drain(ws), TIMEOUT_S)


async def _drain(ws: ClientConnection) -> None:
    """Read and discard messages until the server closes the connection."""
    async for _ in ws:
        pass


def test_websocket_session(tmp_path: Path) -> None:
    """The window gets the scene and growing frames; select works; garbage is ignored."""
    with _running(tmp_path) as config:
        asyncio.run(_session(config))


def _get(port: int) -> tuple[int, bytes]:
    """HTTP GET of the root path, retrying until the server is up."""
    for _ in range(CONNECT_ATTEMPTS):
        try:
            with urllib.request.urlopen(f"http://{HOST}:{port}/", timeout=TIMEOUT_S) as reply:
                return reply.status, reply.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()
        except urllib.error.URLError:
            threading.Event().wait(CONNECT_PAUSE_S)
    raise AssertionError("server did not come up")


def test_page_is_served(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A plain GET returns the page file with status 200."""
    page = tmp_path / "index.html"
    page.write_text("<html>control</html>", encoding="utf-8")
    monkeypatch.setattr(main, "PAGE_PATH", page)
    with _running(tmp_path) as config:
        status, body = _get(config.ports.control_window)
    assert status == 200 and body == b"<html>control</html>"


def test_missing_page_is_404(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Without a page file a plain GET returns 404."""
    monkeypatch.setattr(main, "PAGE_PATH", tmp_path / "missing.html")
    with _running(tmp_path) as config:
        status, _ = _get(config.ports.control_window)
    assert status == 404


def test_camera_age_is_never_negative() -> None:
    """A report newer than the frame (frame released by timeout) gives age 0, not below."""
    tracker, frame, last_report_s = _feed(30)
    newer = {camera_id: t_s + 0.05 for camera_id, t_s in last_report_s.items()}
    health = json.loads(tracks_message(tracker, frame, CONFIG, newer))["health"]
    assert set(health["camera_age_s"].values()) == {0.0}


def test_scene_carries_the_camera_minimum() -> None:
    """The page gets the warning threshold from the tracker settings, not a copy."""
    assert json.loads(scene_message(CONFIG, ROBOT))["min_cameras"] == CONFIG.tracker.min_cameras
