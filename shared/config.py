"""Load and validate the YAML configuration files in ``config/``.

This is the only module that reads configuration. It turns a YAML file into
frozen dataclasses that the simulator and the tracker use. All positions are in
the arena frame: right-handed, x along the long side, y away from the robot,
z up, in metres.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# A Raspberry Pi 5 has two camera ports, so a node can serve at most two cameras.
MAX_CAMERAS_PER_NODE = 2


class ConfigError(ValueError):
    """Raised when a configuration file is missing keys or is inconsistent."""


@dataclass(frozen=True)
class Arena:
    """Flight arena box from the origin to ``size_m`` (arena frame, metres)."""

    size_m: tuple[float, float, float]


@dataclass(frozen=True)
class CameraModel:
    """Intrinsic description of a camera type."""

    id: str
    width_px: int
    height_px: int
    fov_horizontal_rad: float
    rate_hz: float


@dataclass(frozen=True)
class Camera:
    """A fixed camera; ``model`` is a camera model id, positions in metres."""

    id: str
    model: str
    position_m: tuple[float, float, float]
    look_at_m: tuple[float, float, float]


@dataclass(frozen=True)
class Node:
    """A camera node (one computer) and the ids of the cameras it holds."""

    id: str
    camera_ids: tuple[str, ...]


@dataclass(frozen=True)
class Noise:
    """Detection noise and node latency, all in seconds or pixels."""

    pixel_sigma_px: float
    dropout_probability: float
    node_delay_s: float
    node_delay_jitter_s: float


@dataclass(frozen=True)
class Ports:
    """UDP port numbers for each message stream."""

    detections: int
    target: int
    robot_pose: int
    truth: int


@dataclass(frozen=True)
class Config:
    """Complete validated configuration."""

    seed: int
    arena: Arena
    camera_models: dict[str, CameraModel]
    cameras: tuple[Camera, ...]
    nodes: tuple[Node, ...]
    noise: Noise
    ports: Ports


def _require(mapping: Any, key: str, section: str) -> Any:
    """Return ``mapping[key]`` or raise a ConfigError naming key and section.

    Args:
        mapping: Parsed YAML section, expected to be a dict.
        key: Key that must be present.
        section: Human-readable section name used in the error message.

    Returns:
        The value stored under ``key``.

    Raises:
        ConfigError: If ``mapping`` is not a dict or lacks ``key``.
    """
    if not isinstance(mapping, dict) or key not in mapping:
        raise ConfigError(f"missing key '{key}' in section '{section}'")
    return mapping[key]


def _vec3(values: Any, name: str) -> tuple[float, float, float]:
    """Convert a three-element YAML list to a float tuple.

    Args:
        values: Parsed YAML value, expected to be a list of three numbers.
        name: Description used in the error message.

    Returns:
        The three numbers as floats (metres in the arena frame).

    Raises:
        ConfigError: If ``values`` is not a list of three numbers.
    """
    if not isinstance(values, list) or len(values) != 3:
        raise ConfigError(f"{name} must be a list of three numbers")
    return (float(values[0]), float(values[1]), float(values[2]))


def _parse_camera_model(model_id: str, raw: Any) -> CameraModel:
    """Build a CameraModel from its YAML section.

    Args:
        model_id: Key of the model in ``camera_models``.
        raw: Parsed YAML mapping for that model.

    Returns:
        The camera model.

    Raises:
        ConfigError: If a key is missing.
    """
    section = f"camera_models.{model_id}"
    return CameraModel(
        id=model_id,
        width_px=int(_require(raw, "width_px", section)),
        height_px=int(_require(raw, "height_px", section)),
        fov_horizontal_rad=float(_require(raw, "fov_horizontal_rad", section)),
        rate_hz=float(_require(raw, "rate_hz", section)),
    )


def _parse_camera(raw: Any) -> Camera:
    """Build a Camera from one entry of the ``cameras`` list.

    Args:
        raw: Parsed YAML mapping for one camera.

    Returns:
        The camera, with positions in metres in the arena frame.

    Raises:
        ConfigError: If a key is missing or a vector is malformed.
    """
    cam_id = str(_require(raw, "id", "cameras"))
    section = f"cameras.{cam_id}"
    return Camera(
        id=cam_id,
        model=str(_require(raw, "model", section)),
        position_m=_vec3(_require(raw, "position_m", section), f"{cam_id} position_m"),
        look_at_m=_vec3(_require(raw, "look_at_m", section), f"{cam_id} look_at_m"),
    )


def _parse_node(raw: Any) -> Node:
    """Build a Node from one entry of the ``nodes`` list.

    Args:
        raw: Parsed YAML mapping for one node.

    Returns:
        The node.

    Raises:
        ConfigError: If a key is missing.
    """
    node_id = str(_require(raw, "id", "nodes"))
    cameras = _require(raw, "cameras", f"nodes.{node_id}")
    return Node(id=node_id, camera_ids=tuple(str(c) for c in cameras))


def _check_unique_ids(items: tuple[Camera, ...] | tuple[Node, ...], kind: str) -> None:
    """Reject duplicate ids in a sequence of cameras or nodes.

    Args:
        items: Cameras or nodes to check.
        kind: Word used in the message, "camera" or "node".

    Raises:
        ConfigError: Naming the first duplicated id.
    """
    seen: set[str] = set()
    for item in items:
        if item.id in seen:
            raise ConfigError(f"duplicate {kind} id '{item.id}'")
        seen.add(item.id)


def _check_camera_models(
    cameras: tuple[Camera, ...], models: dict[str, CameraModel]
) -> None:
    """Reject cameras that use a model that is not defined.

    Args:
        cameras: Cameras to check.
        models: Defined camera models by id.

    Raises:
        ConfigError: Naming the camera and the unknown model.
    """
    for camera in cameras:
        if camera.model not in models:
            raise ConfigError(
                f"camera '{camera.id}' uses unknown camera model '{camera.model}'"
            )


def _check_node_cameras(nodes: tuple[Node, ...], cameras: tuple[Camera, ...]) -> None:
    """Check each node's camera list: known ids and at most the port limit.

    Args:
        nodes: Nodes to check.
        cameras: All defined cameras.

    Raises:
        ConfigError: Naming the offending node and camera.
    """
    known = {camera.id for camera in cameras}
    for node in nodes:
        if len(node.camera_ids) > MAX_CAMERAS_PER_NODE:
            raise ConfigError(
                f"node '{node.id}' has {len(node.camera_ids)} cameras, "
                f"at most {MAX_CAMERAS_PER_NODE} allowed"
            )
        for camera_id in node.camera_ids:
            if camera_id not in known:
                raise ConfigError(
                    f"node '{node.id}' lists unknown camera '{camera_id}'"
                )


def _check_camera_membership(
    nodes: tuple[Node, ...], cameras: tuple[Camera, ...]
) -> None:
    """Require every camera to belong to exactly one node.

    Args:
        nodes: All nodes.
        cameras: All cameras.

    Raises:
        ConfigError: Naming a camera in no node or in several nodes.
    """
    for camera in cameras:
        # Count every listing, so a camera named twice by one node is caught too.
        holders = [n.id for n in nodes for c in n.camera_ids if c == camera.id]
        if not holders:
            raise ConfigError(f"camera '{camera.id}' belongs to no node")
        if len(holders) > 1:
            raise ConfigError(
                f"camera '{camera.id}' is listed more than once in the nodes: "
                f"{', '.join(holders)}"
            )


def _validate(config: Config) -> None:
    """Run all consistency checks on a parsed configuration.

    Args:
        config: Configuration to check.

    Raises:
        ConfigError: On the first inconsistency found.
    """
    _check_unique_ids(config.cameras, "camera")
    _check_unique_ids(config.nodes, "node")
    _check_camera_models(config.cameras, config.camera_models)
    _check_node_cameras(config.nodes, config.cameras)
    _check_camera_membership(config.nodes, config.cameras)


def _parse_noise(raw: Any) -> Noise:
    """Build Noise from the ``noise`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The noise settings (pixels and seconds).

    Raises:
        ConfigError: If a key is missing.
    """
    return Noise(
        pixel_sigma_px=float(_require(raw, "pixel_sigma_px", "noise")),
        dropout_probability=float(_require(raw, "dropout_probability", "noise")),
        node_delay_s=float(_require(raw, "node_delay_s", "noise")),
        node_delay_jitter_s=float(_require(raw, "node_delay_jitter_s", "noise")),
    )


def _parse_ports(raw: Any) -> Ports:
    """Build Ports from the ``ports`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The UDP ports.

    Raises:
        ConfigError: If a key is missing.
    """
    return Ports(
        detections=int(_require(raw, "detections", "ports")),
        target=int(_require(raw, "target", "ports")),
        robot_pose=int(_require(raw, "robot_pose", "ports")),
        truth=int(_require(raw, "truth", "ports")),
    )


def _parse(raw: Any) -> Config:
    """Build a Config from the parsed YAML document, without consistency checks.

    Args:
        raw: Parsed YAML document, expected to be a mapping.

    Returns:
        The configuration. Lengths are in metres, angles in radians.

    Raises:
        ConfigError: If a key is missing or a vector is malformed.
    """
    arena_raw = _require(raw, "arena", "top level")
    models_raw = _require(raw, "camera_models", "top level")
    return Config(
        seed=int(_require(raw, "seed", "top level")),
        arena=Arena(size_m=_vec3(_require(arena_raw, "size_m", "arena"), "arena size_m")),
        camera_models={k: _parse_camera_model(k, v) for k, v in models_raw.items()},
        cameras=tuple(_parse_camera(c) for c in _require(raw, "cameras", "top level")),
        nodes=tuple(_parse_node(n) for n in _require(raw, "nodes", "top level")),
        noise=_parse_noise(_require(raw, "noise", "top level")),
        ports=_parse_ports(_require(raw, "ports", "top level")),
    )


def load_config(path: Path) -> Config:
    """Read, parse and validate a configuration file.

    Args:
        path: Path to a YAML file such as ``config/default.yaml``.

    Returns:
        The validated configuration. Lengths are in metres, angles in radians.

    Raises:
        ConfigError: If a key is missing, a value has the wrong type or the content
            is inconsistent.
    """
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    try:
        config = _parse(raw)
    except ConfigError:
        raise
    except (TypeError, ValueError, AttributeError) as error:
        # A wrong value type (text where a number belongs, a list where a mapping
        # belongs) must reach the user as a config problem, not as a bare traceback.
        raise ConfigError(f"invalid value in {path}: {error}") from error
    _validate(config)
    return config
