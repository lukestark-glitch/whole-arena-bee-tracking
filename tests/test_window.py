"""Tests for the simulator window: messages, WebSocket protocol and page serving."""

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

import simulator.main as main
from shared.config import load_config
from shared.protocol import UDP_HOST
from simulator.main import create_simulation, scene_message, state_message

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
RUN_DURATION_S = 5.0  # simulation time of the background run; ended early by the tests
FINISH_TIME_SCALE = 100.0  # makes the rest of the run take a few milliseconds
TIMEOUT_S = 5.0
CONNECT_ATTEMPTS = 100
CONNECT_PAUSE_S = 0.05
PAUSED_STATES_TO_READ = 5


def _free_tcp_port() -> int:
    """Return a TCP port that is free right now."""
    with socket.socket() as sock:
        sock.bind((UDP_HOST, 0))
        return sock.getsockname()[1]


def _control(name: str, value: object) -> str:
    """Build a control message."""
    return json.dumps({"type": "control", "name": name, "value": value})


async def _send_finish(port: int) -> None:
    """Ask a running simulator to run through its remaining frames quickly."""
    async with connect(f"ws://{UDP_HOST}:{port}") as ws:
        await ws.send(_control("paused", False))
        await ws.send(_control("time_scale", FINISH_TIME_SCALE))
        await ws.send(_control("bee_count", 0))
        await asyncio.wait_for(ws.wait_closed(), TIMEOUT_S)


@contextmanager
def _running() -> Iterator[int]:
    """Run the simulator in a thread and yield its window port; join at the end."""
    port = _free_tcp_port()
    config = dataclasses.replace(
        CONFIG, ports=dataclasses.replace(CONFIG.ports, simulator_window=port)
    )
    thread = threading.Thread(
        target=main.run, args=(config, 1, RUN_DURATION_S, 1.0, False), daemon=True
    )
    thread.start()
    try:
        yield port
    finally:
        asyncio.run(_send_finish(port))
        thread.join(TIMEOUT_S)
        assert not thread.is_alive()


def _get(port: int) -> tuple[int, bytes]:
    """HTTP GET of the root path, retrying until the server is up."""
    for _ in range(CONNECT_ATTEMPTS):
        try:
            with urllib.request.urlopen(f"http://{UDP_HOST}:{port}/", timeout=TIMEOUT_S) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()
        except urllib.error.URLError:
            threading.Event().wait(CONNECT_PAUSE_S)
    raise AssertionError("server did not come up")


async def _connect(port: int) -> ClientConnection:
    """Open a WebSocket to the simulator, retrying until the server is up."""
    for _ in range(CONNECT_ATTEMPTS):
        try:
            return await connect(f"ws://{UDP_HOST}:{port}")
        except OSError:
            await asyncio.sleep(CONNECT_PAUSE_S)
    raise AssertionError("server did not come up")


async def _next_state(ws: ClientConnection) -> dict:
    """Receive messages until a `state` message arrives."""
    while True:
        message = json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
        if message["type"] == "state":
            return message


def test_scene_message() -> None:
    """The scene lists the arena size and every camera with its field of view."""
    message = json.loads(scene_message(CONFIG))
    assert message["type"] == "scene"
    assert message["arena_size_m"] == list(CONFIG.arena.size_m)
    assert len(message["cameras"]) == 16
    first = message["cameras"][0]
    assert set(first) == {"id", "position_m", "look_at_m", "fov_horizontal_rad"}


def test_state_message() -> None:
    """The state holds frame, time, settings and one entry per bee."""
    simulation = create_simulation(CONFIG, 3, 2.0)
    message = json.loads(state_message(simulation))
    assert message["type"] == "state" and message["frame"] == 0 and message["t"] == 0.0
    assert message["seed"] == 3 and message["paused"] is False and message["time_scale"] == 2.0
    assert message["settings"] == {
        "bee_count": 10,
        "max_speed_m_s": CONFIG.bees.max_speed_m_s,
        "pixel_sigma_px": CONFIG.noise.pixel_sigma_px,
        "dropout_probability": CONFIG.noise.dropout_probability,
    }
    assert [b["id"] for b in message["bees"]] == list(range(10))
    assert len(message["bees"][0]["pos"]) == 3 and message["bees"][0]["state"] == "flying"


def test_websocket_session() -> None:
    """Scene first, then advancing states; pause stops them; garbage is ignored."""

    async def session(port: int) -> None:
        ws = await _connect(port)
        async with ws:
            scene = json.loads(await asyncio.wait_for(ws.recv(), TIMEOUT_S))
            assert scene["type"] == "scene" and len(scene["cameras"]) == 16
            first = await _next_state(ws)
            while (await _next_state(ws))["frame"] <= first["frame"]:
                pass
            for garbage in ("not json", "[1]", _control("bee_count", -1), _control("nope", 1)):
                await ws.send(garbage)
            await ws.send(_control("paused", True))
            state = await _next_state(ws)
            while not state["paused"]:
                state = await _next_state(ws)
            for _ in range(PAUSED_STATES_TO_READ):
                assert (await _next_state(ws))["frame"] == state["frame"]

    with _running() as port:
        asyncio.run(session(port))


@pytest.mark.parametrize("page_exists", [True, False])
def test_http_page(page_exists: bool, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A plain GET returns the page, or 404 when the page file is missing."""
    page = tmp_path / "index.html"
    if page_exists:
        page.write_text("<!doctype html><title>x</title>", encoding="utf-8")
    monkeypatch.setattr(main, "PAGE_PATH", page)
    with _running() as port:
        status, body = _get(port)
    assert status == (200 if page_exists else 404)
    assert (body == page.read_bytes()) if page_exists else body
