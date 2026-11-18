"""Tests for the Stage 7 tools: coverage, camera-layout files and the batch runner."""

from pathlib import Path

import numpy as np
import yaml

from experiments.batch import CAMERA_CONFIGS, Case, cases, criterion, run_case
from experiments.coverage import GRID_SPACING_M, coverage_of, grid_points

REPO = Path(__file__).parent.parent


def test_grid_points_fill_the_box() -> None:
    """Grid cell centres lie inside the box, one per cell."""
    points_m = grid_points((0.9, 0.3, 0.3), 0.1)
    assert points_m.shape == (9 * 3 * 3, 3)
    assert np.all(points_m > 0.0) and np.all(points_m < [0.9, 0.3, 0.3])


def test_every_layout_sees_the_box_with_three_cameras() -> None:
    """All camera layouts see every grid point with three or more cameras; fewer cameras
    give a larger expected error."""
    errors = {}
    for cameras, path in CAMERA_CONFIGS.items():
        summary, _, _ = coverage_of(REPO / path, GRID_SPACING_M)
        assert summary.cameras == cameras and summary.share_by_level[3] == 1.0
        errors[cameras] = summary.median_error_m
    assert errors[8] > errors[16]


def test_layout_files_differ_from_default_only_in_cameras_and_nodes() -> None:
    """The variant files repeat config/default.yaml outside the cameras and nodes."""
    default = yaml.safe_load((REPO / "config" / "default.yaml").read_text())
    for path in CAMERA_CONFIGS.values():
        variant = yaml.safe_load((REPO / path).read_text())
        for key in default.keys() - {"cameras", "nodes"}:
            assert variant[key] == default[key], (path, key)


def test_divided_arena_differs_from_default_only_in_its_walls() -> None:
    """config/arena-wall.yaml is the default with a walls file."""
    default = yaml.safe_load((REPO / "config" / "default.yaml").read_text())
    divided = yaml.safe_load((REPO / "config" / "arena-wall.yaml").read_text())
    assert divided["arena"]["walls_stl"] == "arenas/divided-box.stl"
    divided["arena"]["walls_stl"] = None
    assert divided == default


def test_partition_hides_part_of_the_volume() -> None:
    """The partition leaves points seen by fewer than six cameras, but none by fewer than
    two."""
    summary, _, _ = coverage_of(REPO / "config" / "arena-wall.yaml", GRID_SPACING_M)
    assert summary.share_by_level[2] == 1.0 and summary.share_by_level[6] < 0.9


def test_case_labels_are_unique() -> None:
    """Every experiment case has its own label."""
    labels = [(case.experiment, case.label) for case in cases()]
    assert len(labels) == len(set(labels))


def test_run_case_repeats_with_the_same_seed() -> None:
    """A short run gives the same numbers twice and tracks every bee."""
    case = Case("test", "short", bees=3)
    first, second = (run_case(case, 1, 3.0) for _ in range(2))
    first.pop("wall_s"), second.pop("wall_s")
    assert first == second
    assert first["tracked_pct"] > 90.0 and first["ghost_tracks"] == 0


def test_criterion_names_the_failed_point() -> None:
    """A layout with a ghost track is not good enough, and the verdict says why."""
    base = {"experiment": "cameras", "stopped_nodes": 0, "flight_model": "random",
            "mean_error_mm": 0.5, "p95_error_mm": 1.0, "ghost_tracks_max": 0,
            "tracked_pct": 99.0, "id_swaps": 4.0, "in_frame_pct": 50.0, "in_focus_pct": 30.0}
    rows = [{**base, "cameras": 16}, {**base, "cameras": 8, "ghost_tracks_max": 1}]
    coverage = {16: {"share_by_level": {3: 1.0}}, 8: {"share_by_level": {3: 1.0}}}
    verdicts = {row["cameras"]: row["verdict"] for row in criterion(rows, coverage)}
    assert verdicts[16] == "good enough"
    assert verdicts[8] == "not good enough: no_ghost_track"
