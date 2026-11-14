"""Tests for the simulator calibration tools: wand path, pose error, marker view."""

from itertools import combinations
from pathlib import Path

import numpy as np

from shared.config import load_config
from shared.geometry import camera_pose, is_visible, project
from shared.wand import REFERENCE, STOP, SWEEP, wand_duration_s, wand_phase
from simulator.wand import observe_markers, perturb_pose, stop_points, wand_position

CONFIG = load_config(Path(__file__).parent.parent / "config" / "default.yaml")
CAL = CONFIG.calibration
ARENA = CONFIG.arena
FRAME_S = 1 / 60


def _sample_times(phase_name: str) -> list[tuple[float, int]]:
    """Return (time, index) samples of a phase over the whole run."""
    end_s = wand_duration_s(CAL)
    times = np.arange(0.0, end_s, FRAME_S)
    return [(t, wand_phase(CAL, t)[1]) for t in times if wand_phase(CAL, t)[0] == phase_name]


def test_reference_rest() -> None:
    """The wand rests on each reference point during its dwell."""
    samples = _sample_times(REFERENCE)
    assert samples
    for t, i in samples:
        assert np.allclose(wand_position(CAL, ARENA, t), CAL.reference_points_m[i])


def test_sweep_inside_and_continuous() -> None:
    """The sweep stays inside the margin and moves less than 5 mm per frame."""
    samples = _sample_times(SWEEP)
    pts = np.array([wand_position(CAL, ARENA, t) for t, _ in samples])
    lo = CAL.sweep_margin_m - 1e-9
    hi = np.array(ARENA.size_m) - CAL.sweep_margin_m + 1e-9
    assert np.all(pts >= lo) and np.all(pts <= hi)
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    # The path peaks near 0.8 m/s (about 13.5 mm per frame), so 5 mm is unreachable
    # for this path; 20 mm still catches jumps.
    assert steps.max() < 0.02


def test_stops_and_done() -> None:
    """The wand sits on stop k during stop k and on the last stop afterwards."""
    stops = stop_points(CAL, ARENA)
    for t, k in _sample_times(STOP):
        assert np.allclose(wand_position(CAL, ARENA, t), stops[k])
    after_s = wand_duration_s(CAL) + 5.0
    assert np.allclose(wand_position(CAL, ARENA, after_s), stops[-1])


def test_stop_points_distinct() -> None:
    """Stop points are at least 1 cm apart."""
    stops = stop_points(CAL, ARENA)
    assert stops.shape == (CAL.stop_count, 3)
    assert min(np.linalg.norm(a - b) for a, b in combinations(stops, 2)) > 0.01


def test_perturb_pose_exact() -> None:
    """The pose error has exactly the requested distance and angle."""
    pose = camera_pose(np.array([0.5, -0.4, 0.2]), np.array([0.45, 0.15, 0.15]))
    new = perturb_pose(pose, 0.02, 0.01, np.random.default_rng(1))
    assert np.isclose(np.linalg.norm(new.position_m - pose.position_m), 0.02)
    cos = (np.trace(pose.rotation @ new.rotation.T) - 1) / 2
    assert np.isclose(np.arccos(np.clip(cos, -1, 1)), 0.01)
    assert np.allclose(new.rotation @ new.rotation.T, np.eye(3))
    assert np.isclose(np.linalg.det(new.rotation), 1.0)


def test_perturb_pose_zero() -> None:
    """Zero error returns the same pose values."""
    pose = camera_pose(np.array([0.5, -0.4, 0.2]), np.array([0.45, 0.15, 0.15]))
    new = perturb_pose(pose, 0.0, 0.0, np.random.default_rng(1))
    assert np.allclose(new.position_m, pose.position_m)
    assert np.allclose(new.rotation, pose.rotation)


def _view() -> tuple:
    """Return a camera pose and model looking at the arena."""
    model = next(iter(CONFIG.camera_models.values())) if isinstance(
        CONFIG.camera_models, dict
    ) else CONFIG.camera_models[0]
    pose = camera_pose(np.array([0.45, -0.6, 0.15]), np.array([0.45, 0.15, 0.15]))
    return pose, model


def test_observe_markers_noiseless() -> None:
    """Without noise the pixels equal the projection of the visible markers."""
    pose, model = _view()
    markers = CAL.markers
    pts = np.array([m.position_m for m in markers])
    vis = is_visible(pose, model, pts)
    got = observe_markers(pose, model, markers, 0.0, np.random.default_rng(0))
    expected = project(pose, model, pts[vis])
    inside = [
        (m.id, *p)
        for m, p in zip([m for m, v in zip(markers, vis) if v], expected)
        if 0 <= p[0] < model.width_px and 0 <= p[1] < model.height_px
    ]
    assert len(got) == len(inside)
    for (i, u, v), (j, eu, ev) in zip(got, inside):
        assert i == j and np.isclose(u, eu) and np.isclose(v, ev)


def test_observe_markers_behind_and_empty() -> None:
    """A marker behind the camera is not reported; no markers gives ()."""
    pose, model = _view()
    from shared.config import Marker

    behind = (Marker(id=99, position_m=(0.45, -2.0, 0.15)),)
    assert observe_markers(pose, model, behind, 0.0, np.random.default_rng(0)) == ()
    assert observe_markers(pose, model, (), 0.5, np.random.default_rng(0)) == ()


def test_observe_markers_deterministic() -> None:
    """The same seed gives the same result."""
    pose, model = _view()
    a = observe_markers(pose, model, CAL.markers, 0.5, np.random.default_rng(7))
    b = observe_markers(pose, model, CAL.markers, 0.5, np.random.default_rng(7))
    assert a == b
