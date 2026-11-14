"""Load and validate the YAML configuration files in ``config/``.

This is the only module that reads configuration. It turns ``config/default.yaml``
(`load_config`) and the robot file ``config/robot.yaml`` (`load_robot`) into
frozen dataclasses that the simulator, the robot emulator and the tracker use. All
positions are in the arena frame: right-handed, x along the long side, y away from the
robot, z up, in metres.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, TypeVar

import yaml

_Parsed = TypeVar("_Parsed")

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
class Bees:
    """Number of simulated bees and the parameters of their motion.

    Attributes:
        count: Number of bees in the arena.
        body_length_m: Body length, sets the blob size in the images, in metres.
        max_speed_m_s: Speed no bee exceeds, in m/s.
        flight_speed_m_s: Typical (root mean square) flight speed, in m/s.
        flight_direction_time_s: Time over which a flying bee keeps its direction, in s.
        walk_speed_m_s: Speed of a walking bee, in m/s.
        walk_turn_rad_per_sqrt_s: Random heading change of a walking bee, standard
            deviation after one second, in rad.
        landing_probability: Chance that a flying bee lands when it touches a wall.
        takeoff_rate_hz: Rate at which a sitting or walking bee takes off, in 1/s.
        sit_to_walk_rate_hz: Rate at which a sitting bee starts to walk, in 1/s.
        walk_to_sit_rate_hz: Rate at which a walking bee stops, in 1/s.
    """

    count: int
    body_length_m: float
    max_speed_m_s: float
    flight_speed_m_s: float
    flight_direction_time_s: float
    walk_speed_m_s: float
    walk_turn_rad_per_sqrt_s: float
    landing_probability: float
    takeoff_rate_hz: float
    sit_to_walk_rate_hz: float
    walk_to_sit_rate_hz: float


@dataclass(frozen=True)
class TrackerSettings:
    """Tuning values of the tracking server.

    Attributes:
        frame_timeout_frames: An incomplete frame is processed once a message arrives
            that is this many frame periods newer.
        claim_gate_m: A track claims a detection whose ray passes within this distance of
            the track's predicted position, in metres.
        ray_tolerance_m: Rays that pass within this distance of a point support it, in
            metres.
        min_cameras: Cameras that must agree on a new point.
        min_cameras_tracked: Cameras that must agree on the point of an existing track.
        coverage_share: A new point also needs this share of the cameras that should see
            its location.
        arena_margin_m: A new point may lie this far outside the arena box, in metres.
        assignment_gate_m: Largest distance between a predicted track and a new point
            that may be assigned to it, in metres.
        confirm_frames: Consecutive frames with support before a track is confirmed.
        max_missed_frames: Consecutive frames without support before a confirmed track
            is removed.
        jerk_sigma_m_s3: Process noise of the filter: standard deviation of the jerk, in
            m/s^3.
        measurement_sigma_m: Standard deviation of a triangulated point per axis, in
            metres.
        initial_speed_sigma_m_s: Velocity uncertainty of a new track per axis, in m/s.
        initial_acceleration_sigma_m_s2: Acceleration uncertainty of a new track per
            axis, in m/s^2.
        flying_above_m_s: A sitting track becomes flying above this speed, in m/s.
        sitting_below_m_s: A flying track becomes sitting below this speed, in m/s.
        acceleration_smoothing_s: Time constant of the smoothed acceleration shown for a
            track, in seconds.
        prediction_lead_s: How far ahead the predicted position shown in the control
            window lies, in seconds.
        node_silent_after_s: A node counts as stopped when none of its cameras has
            reported for this long, in seconds of simulation time.
    """

    frame_timeout_frames: float
    claim_gate_m: float
    ray_tolerance_m: float
    min_cameras: int
    min_cameras_tracked: int
    coverage_share: float
    arena_margin_m: float
    assignment_gate_m: float
    confirm_frames: int
    max_missed_frames: int
    jerk_sigma_m_s3: float
    measurement_sigma_m: float
    initial_speed_sigma_m_s: float
    initial_acceleration_sigma_m_s2: float
    flying_above_m_s: float
    sitting_below_m_s: float
    acceleration_smoothing_s: float
    prediction_lead_s: float
    node_silent_after_s: float


# Tracker settings that are whole numbers; all others are floats.
_TRACKER_INT_FIELDS = ("min_cameras", "min_cameras_tracked", "confirm_frames", "max_missed_frames")


@dataclass(frozen=True)
class Marker:
    """A wall marker with a known position.

    Attributes:
        id: Marker number, unique.
        position_m: Position in the arena frame (m).
    """

    id: int
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class CalibrationSettings:
    """The wand run, the wall markers and the deliberate errors of the simulation.

    Attributes:
        camera_error_position_m: Distance of each true camera from its nominal position
            when the simulator runs miscalibrated (m).
        camera_error_angle_rad: Rotation of each true camera against its nominal
            orientation when the simulator runs miscalibrated (rad).
        robot_error_m: Shift of the true rail start against the robot file when the
            simulator runs miscalibrated, arena frame (m).
        reference_points_m: Three known points the wand rests on first, arena frame (m).
        reference_dwell_s: Rest on each reference point (s).
        sweep_duration_s: Duration of the sweep through the arena (s).
        sweep_margin_m: Distance the sweep keeps from the walls (m).
        sweep_frame_step: The camera solver uses every n-th frame of the sweep.
        stop_count: Number of stop points for the robot calibration.
        stop_dwell_s: Rest on each stop point (s).
        marker_rate_hz: How often the cameras report the wall markers (Hz).
        marker_threshold_px: Mean marker error above which a camera counts as displaced (px).
        min_markers: Markers a camera must see to be checked and corrected.
        markers: The wall markers.
    """

    camera_error_position_m: float
    camera_error_angle_rad: float
    robot_error_m: tuple[float, float, float]
    reference_points_m: tuple[tuple[float, float, float], ...]
    reference_dwell_s: float
    sweep_duration_s: float
    sweep_margin_m: float
    sweep_frame_step: int
    stop_count: int
    stop_dwell_s: float
    marker_rate_hz: float
    marker_threshold_px: float
    min_markers: int
    markers: tuple[Marker, ...]


# Known points needed to estimate the six pose values of one camera.
MIN_MARKERS_FOR_POSE = 4
# Reference points needed to fix position, orientation and scale of the arena frame.
REFERENCE_POINT_COUNT = 3


@dataclass(frozen=True)
class Ports:
    """Port numbers: one UDP port per message stream, one TCP port per browser window."""

    detections: int
    target: int
    robot_pose: int
    truth: int
    simulator_window: int
    control_window: int


@dataclass(frozen=True)
class Config:
    """Complete validated configuration."""

    seed: int
    arena: Arena
    camera_models: dict[str, CameraModel]
    cameras: tuple[Camera, ...]
    nodes: tuple[Node, ...]
    noise: Noise
    bees: Bees
    tracker: TrackerSettings
    calibration: CalibrationSettings
    ports: Ports


@dataclass(frozen=True)
class AxisLimits:
    """Travel and motion limits of one robot axis.

    The unit depends on the axis: metres for rail and focus, radians for tilt.

    Attributes:
        lower: Smallest position (m or rad).
        upper: Largest position (m or rad).
        max_speed: Largest speed (m/s or rad/s).
        max_acceleration: Largest acceleration (m/s^2 or rad/s^2).
    """

    lower: float
    upper: float
    max_speed: float
    max_acceleration: float


@dataclass(frozen=True)
class Cinema:
    """The robot's cinema camera.

    Attributes:
        fov_horizontal_rad: Horizontal field of view (rad).
        aspect_ratio: Image width divided by image height.
        depth_of_field_m: Total depth around the focus plane that counts as sharp (m).
    """

    fov_horizontal_rad: float
    aspect_ratio: float
    depth_of_field_m: float


@dataclass(frozen=True)
class StereoCamera:
    """The robot's stereo camera pair, treated as one rectified image.

    Attributes:
        model: Image size (px), field of view (rad) and rate (Hz).
        baseline_m: Distance between the two lenses (m).
        pixel_sigma_px: Noise of a blob position per image axis (px).
        size_sigma_px: Noise of a blob diameter (px).
        disparity_sigma_px: Stereo matching noise (px); it becomes the distance noise.
    """

    model: CameraModel
    baseline_m: float
    pixel_sigma_px: float
    size_sigma_px: float
    disparity_sigma_px: float


@dataclass(frozen=True)
class RobotControl:
    """Settings of the robot's control software.

    Attributes:
        rate_hz: Rate of the axis control and of `robot_pose` (Hz).
        search_window_px: Half side of the square search window in the stereo image (px).
        distance_tolerance_m: Largest difference between a blob's measured distance and
            the expected one (m).
        target_timeout_s: Silence of `target` after which the robot holds position (s).
    """

    rate_hz: float
    search_window_px: float
    distance_tolerance_m: float
    target_timeout_s: float


@dataclass(frozen=True)
class Robot:
    """Complete validated robot description from ``config/robot.yaml``.

    Attributes:
        rail_start_m: Tilt axis at rail position 0, arena frame (m). The rail runs along +x.
        lens_offset_m: Lens reference point ahead of the tilt axis along the optical
            axis (m).
        rail: Rail limits (m, m/s, m/s^2); position counts from ``rail_start_m``.
        tilt: Tilt limits (rad, rad/s, rad/s^2); 0 = horizontal, positive = up.
        focus: Focus limits as distance of the focus plane (m, m/s, m/s^2).
        cinema: Cinema camera.
        stereo: Stereo camera.
        control: Control software settings.
    """

    rail_start_m: tuple[float, float, float]
    lens_offset_m: float
    rail: AxisLimits
    tilt: AxisLimits
    focus: AxisLimits
    cinema: Cinema
    stereo: StereoCamera
    control: RobotControl


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


def _check_frame_rate(cameras: tuple[Camera, ...], models: dict[str, CameraModel]) -> None:
    """Require all cameras to run at the same frame rate.

    One trigger source pulses every camera, so a second rate cannot exist.

    Args:
        cameras: All cameras.
        models: Defined camera models by id.

    Raises:
        ConfigError: Naming the first camera whose rate differs.
    """
    rate_hz = models[cameras[0].model].rate_hz
    for camera in cameras:
        if models[camera.model].rate_hz != rate_hz:
            raise ConfigError(
                f"camera '{camera.id}' runs at {models[camera.model].rate_hz} Hz, "
                f"but all cameras share one trigger at {rate_hz} Hz"
            )


def frame_rate_hz(config: Config) -> float:
    """Return the common frame rate of all cameras.

    Args:
        config: A validated configuration.

    Returns:
        Trigger rate in Hz.
    """
    return config.camera_models[config.cameras[0].model].rate_hz


def last_frame_of(config: Config, duration_s: float) -> int:
    """Return the number of the last frame that starts before a duration ends.

    Simulator, tracker and metrics all end a run of a given duration with this frame.

    Args:
        config: A validated configuration.
        duration_s: Duration in simulation seconds.

    Returns:
        The largest frame number n with n / frame rate < duration_s.
    """
    return math.ceil(duration_s * frame_rate_hz(config)) - 1


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
    _check_frame_rate(config.cameras, config.camera_models)
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


def _parse_bees(raw: Any) -> Bees:
    """Build Bees from the ``bees`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The bee settings (metres, seconds, radians).

    Raises:
        ConfigError: If a key is missing.
    """
    names = [field.name for field in fields(Bees)]
    values = {name: float(_require(raw, name, "bees")) for name in names}
    return Bees(**{**values, "count": int(values["count"])})


def _parse_tracker(raw: Any) -> TrackerSettings:
    """Build TrackerSettings from the ``tracker`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The tracker settings (metres, seconds, frames).

    Raises:
        ConfigError: If a key is missing.
    """
    names = [field.name for field in fields(TrackerSettings)]
    values = {name: float(_require(raw, name, "tracker")) for name in names}
    whole = {name: int(values[name]) for name in _TRACKER_INT_FIELDS}
    return TrackerSettings(**{**values, **whole})


def _parse_calibration(raw: Any) -> CalibrationSettings:
    """Build CalibrationSettings from the ``calibration`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The calibration settings (metres, seconds, radians, pixels).

    Raises:
        ConfigError: If a key is missing, there are not exactly three reference points, a
            count or rate is not positive, `min_markers` is below 4, or two markers share
            an id.
    """
    section = "calibration"
    names = (
        "camera_error_position_m", "camera_error_angle_rad", "reference_dwell_s",
        "sweep_duration_s", "sweep_margin_m", "stop_dwell_s", "marker_rate_hz",
        "marker_threshold_px",
    )
    numbers = dict(zip(names, _floats(raw, names, section)))
    whole = {
        name: int(_require(raw, name, section))
        for name in ("sweep_frame_step", "stop_count", "min_markers")
    }
    references = tuple(
        _vec3(point, "calibration reference point")
        for point in _require(raw, "reference_points_m", section)
    )
    markers = tuple(
        Marker(int(_require(m, "id", "marker")), _vec3(_require(m, "position_m", "marker"), "marker"))
        for m in _require(raw, "markers", section)
    )
    if len(references) != REFERENCE_POINT_COUNT:
        raise ConfigError(f"calibration needs exactly {REFERENCE_POINT_COUNT} reference points")
    if min(whole.values()) < 1 or numbers["marker_rate_hz"] <= 0.0:
        raise ConfigError("calibration counts, the frame step and marker_rate_hz must be above 0")
    if whole["min_markers"] < MIN_MARKERS_FOR_POSE:
        raise ConfigError(f"calibration min_markers must be at least {MIN_MARKERS_FOR_POSE}")
    if len({marker.id for marker in markers}) != len(markers):
        raise ConfigError("calibration marker ids must be unique")
    return CalibrationSettings(
        robot_error_m=_vec3(_require(raw, "robot_error_m", section), "robot_error_m"),
        reference_points_m=references,
        markers=markers,
        **numbers,
        **whole,
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
        simulator_window=int(_require(raw, "simulator_window", "ports")),
        control_window=int(_require(raw, "control_window", "ports")),
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
        bees=_parse_bees(_require(raw, "bees", "top level")),
        tracker=_parse_tracker(_require(raw, "tracker", "top level")),
        calibration=_parse_calibration(_require(raw, "calibration", "top level")),
        ports=_parse_ports(_require(raw, "ports", "top level")),
    )


def _floats(raw: Any, names: tuple[str, ...], section: str) -> list[float]:
    """Read several numbers from one section.

    Args:
        raw: Parsed YAML mapping.
        names: Keys to read, in order.
        section: Section name used in the error message.

    Returns:
        The values as floats, in the order of ``names``.

    Raises:
        ConfigError: If a key is missing.
    """
    return [float(_require(raw, name, section)) for name in names]


def _parse_axis(raw: Any, section: str, lower: str | None, upper: str, unit: str) -> AxisLimits:
    """Build the limits of one robot axis.

    Args:
        raw: Parsed YAML mapping of the axis.
        section: Section name: rail, tilt or focus.
        lower: Key of the smallest position, or None when the axis starts at 0.
        upper: Key of the largest position.
        unit: Unit suffix of the speed and acceleration keys: ``m`` or ``rad``.

    Returns:
        The limits, in the unit of the axis.

    Raises:
        ConfigError: If a key is missing, the range is empty or a limit is not positive.
    """
    low = 0.0 if lower is None else float(_require(raw, lower, section))
    high, speed, acceleration = _floats(
        raw, (upper, f"max_speed_{unit}_s", f"max_acceleration_{unit}_s2"), section
    )
    if not low < high:
        raise ConfigError(f"section '{section}': the range of travel is empty")
    if speed <= 0.0 or acceleration <= 0.0:
        raise ConfigError(f"section '{section}': speed and acceleration limits must be above 0")
    return AxisLimits(low, high, speed, acceleration)


def _parse_stereo(raw: Any) -> StereoCamera:
    """Build StereoCamera from the ``stereo`` section.

    Args:
        raw: Parsed YAML mapping.

    Returns:
        The stereo camera (pixels, radians, metres).

    Raises:
        ConfigError: If a key is missing.
    """
    model = CameraModel(
        id="stereo",
        width_px=int(_require(raw, "width_px", "stereo")),
        height_px=int(_require(raw, "height_px", "stereo")),
        fov_horizontal_rad=float(_require(raw, "fov_horizontal_rad", "stereo")),
        rate_hz=float(_require(raw, "rate_hz", "stereo")),
    )
    names = ("baseline_m", "pixel_sigma_px", "size_sigma_px", "disparity_sigma_px")
    return StereoCamera(model, *_floats(raw, names, "stereo"))


def _parse_robot(raw: Any) -> Robot:
    """Build a Robot from the parsed YAML document.

    Args:
        raw: Parsed YAML document, expected to be a mapping.

    Returns:
        The robot description. Lengths in metres, angles in radians.

    Raises:
        ConfigError: If a key is missing or a value is out of range.
    """
    rail_raw = _require(raw, "rail", "top level")
    cinema_names = ("fov_horizontal_rad", "aspect_ratio", "depth_of_field_m")
    control_names = ("rate_hz", "search_window_px", "distance_tolerance_m", "target_timeout_s")
    robot = Robot(
        rail_start_m=_vec3(_require(rail_raw, "start_m", "rail"), "rail start_m"),
        lens_offset_m=float(_require(_require(raw, "lens", "top level"), "offset_m", "lens")),
        rail=_parse_axis(rail_raw, "rail", None, "length_m", "m"),
        tilt=_parse_axis(_require(raw, "tilt", "top level"), "tilt", "min_rad", "max_rad", "rad"),
        focus=_parse_axis(_require(raw, "focus", "top level"), "focus", "min_m", "max_m", "m"),
        cinema=Cinema(*_floats(_require(raw, "cinema", "top level"), cinema_names, "cinema")),
        stereo=_parse_stereo(_require(raw, "stereo", "top level")),
        control=RobotControl(
            *_floats(_require(raw, "control", "top level"), control_names, "control")
        ),
    )
    if robot.control.rate_hz <= 0.0 or robot.stereo.model.rate_hz <= 0.0:
        raise ConfigError("control rate_hz and stereo rate_hz must be above 0")
    return robot


def _load(path: Path, parse: Callable[[Any], _Parsed]) -> _Parsed:
    """Read a YAML file and parse it, turning wrong value types into ConfigError.

    Args:
        path: Path to the YAML file.
        parse: Builds the result from the parsed YAML document.

    Returns:
        What ``parse`` returns.

    Raises:
        ConfigError: If a key is missing or a value has the wrong type.
    """
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    try:
        return parse(raw)
    except ConfigError:
        raise
    except (TypeError, ValueError, AttributeError) as error:
        # A wrong value type (text where a number belongs, a list where a mapping
        # belongs) must reach the user as a config problem, not as a bare traceback.
        raise ConfigError(f"invalid value in {path}: {error}") from error


def load_robot(path: Path) -> Robot:
    """Read, parse and validate a robot file.

    Args:
        path: Path to a YAML file such as ``config/robot.yaml``.

    Returns:
        The validated robot description. Lengths in metres, angles in radians.

    Raises:
        ConfigError: If a key is missing, a value has the wrong type or is out of range.
    """
    return _load(path, _parse_robot)


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
    config = _load(path, _parse)
    _validate(config)
    return config
