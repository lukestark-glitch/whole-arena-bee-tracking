"""Tests for shared.config: loading the default file and each validation rule."""

from pathlib import Path
from typing import Any, Callable

import pytest
import yaml

from shared.config import ConfigError, load_config

DEFAULT_PATH = Path(__file__).parent.parent / "config" / "default.yaml"
ARENA_CENTRE_M = (0.45, 0.15, 0.15)


def _load_broken(tmp_path: Path, mutate: Callable[[dict[str, Any]], None]) -> None:
    """Load the default YAML, apply ``mutate`` to it, and run load_config."""
    raw = yaml.safe_load(DEFAULT_PATH.read_text(encoding="utf-8"))
    mutate(raw)
    path = tmp_path / "broken.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    load_config(path)


def test_default_config_counts() -> None:
    """The default file has 16 cameras and 8 nodes of two cameras each."""
    config = load_config(DEFAULT_PATH)
    assert len(config.cameras) == 16
    assert len(config.nodes) == 8
    assert all(len(n.camera_ids) == 2 for n in config.nodes)


def test_default_config_values() -> None:
    """Arena, model, noise and port values match the file."""
    config = load_config(DEFAULT_PATH)
    assert config.seed == 1
    assert config.arena.size_m == (0.9, 0.3, 0.3)
    model = config.camera_models["pi_gs_imx296"]
    assert (model.width_px, model.height_px, model.rate_hz) == (1456, 1088, 60)
    assert model.fov_horizontal_rad == pytest.approx(1.919862)
    assert config.noise.pixel_sigma_px == 0.5
    ports = config.ports
    assert (ports.detections, ports.target, ports.robot_pose, ports.truth) == (
        5005, 5006, 5007, 5008,
    )


def test_default_cameras_look_at_centre() -> None:
    """Every default camera looks at the arena centre."""
    config = load_config(DEFAULT_PATH)
    assert all(c.look_at_m == ARENA_CENTRE_M for c in config.cameras)


def test_default_has_eight_corner_cameras() -> None:
    """Exactly eight cameras sit on corners of the arena box."""
    config = load_config(DEFAULT_PATH)
    size = config.arena.size_m
    corners = [
        c for c in config.cameras
        if all(p in (0.0, s) for p, s in zip(c.position_m, size))
    ]
    assert len(corners) == 8


def test_duplicate_camera_id(tmp_path: Path) -> None:
    """A repeated camera id is rejected and named."""
    with pytest.raises(ConfigError, match="duplicate camera id 'cam-01'"):
        _load_broken(tmp_path, lambda r: r["cameras"][1].update(id="cam-01"))


def test_duplicate_node_id(tmp_path: Path) -> None:
    """A repeated node id is rejected and named."""
    with pytest.raises(ConfigError, match="duplicate node id 'node-01'"):
        _load_broken(tmp_path, lambda r: r["nodes"][1].update(id="node-01"))


def test_unknown_camera_model(tmp_path: Path) -> None:
    """A camera with an undefined model is rejected and named."""
    with pytest.raises(ConfigError, match="camera 'cam-03'.*'nope'"):
        _load_broken(tmp_path, lambda r: r["cameras"][2].update(model="nope"))


def test_node_lists_unknown_camera(tmp_path: Path) -> None:
    """A node naming a nonexistent camera is rejected and named."""
    with pytest.raises(ConfigError, match="node 'node-01'.*'cam-99'"):
        _load_broken(
            tmp_path, lambda r: r["nodes"][0].update(cameras=["cam-01", "cam-99"])
        )


def test_camera_in_no_node(tmp_path: Path) -> None:
    """A camera that no node holds is rejected and named."""
    with pytest.raises(ConfigError, match="camera 'cam-02' belongs to no node"):
        _load_broken(tmp_path, lambda r: r["nodes"][0].update(cameras=["cam-01"]))


def test_camera_in_two_nodes(tmp_path: Path) -> None:
    """A camera held by two nodes is rejected and named."""
    with pytest.raises(ConfigError, match="camera 'cam-01' belongs to more than one"):
        _load_broken(tmp_path, lambda r: r["nodes"][1].update(cameras=["cam-01", "cam-04"]))


def test_node_with_three_cameras(tmp_path: Path) -> None:
    """A node with more cameras than the port limit is rejected and named."""
    with pytest.raises(ConfigError, match="node 'node-01' has 3 cameras"):
        _load_broken(
            tmp_path,
            lambda r: r["nodes"][0].update(cameras=["cam-01", "cam-02", "cam-03"]),
        )


def test_missing_key_names_key_and_section(tmp_path: Path) -> None:
    """A missing key is reported with its name and section."""
    with pytest.raises(ConfigError, match="missing key 'rate_hz'.*camera_models"):
        _load_broken(
            tmp_path, lambda r: r["camera_models"]["pi_gs_imx296"].pop("rate_hz")
        )
