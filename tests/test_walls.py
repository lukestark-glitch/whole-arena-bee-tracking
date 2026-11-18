"""Tests for opaque arena walls: line of sight, bees hidden from cameras, tracker coverage."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from shared.config import ConfigError, load_config
from shared.geometry import camera_pose
from shared.walls import hidden, load_walls
from simulator.bees import FLYING, Bee
from simulator.cameras import observe_bees
from tracker.triangulate import frame_rays, tracking_cameras, triangulate

REPO = Path(__file__).parent.parent
CONFIG = load_config(REPO / "config" / "arena-wall.yaml")
WALLS = load_walls(CONFIG.arena.walls_stl)
ORIGIN_M = np.array([0.0, 0.0, 0.0])


def test_points_behind_the_wall_are_hidden() -> None:
    """The partition at x = 0.45 m hides what lies behind it, not what lies before it or
    beside it in the gap at the back, nor a point on its front face."""
    points_m = np.array([
        [0.70, 0.10, 0.15],   # behind the wall
        [0.20, 0.10, 0.15],   # in front of the wall
        [0.50, 0.29, 0.15],   # seen through the gap at the back
        [0.448, 0.10, 0.15],  # on the front face
    ])
    assert hidden(WALLS, ORIGIN_M, points_m).tolist() == [True, False, False, False]


def test_no_walls_hide_nothing() -> None:
    """Without walls no point is hidden."""
    assert not hidden(None, ORIGIN_M, np.array([[0.7, 0.1, 0.15]])).any()


def test_blocked_camera_does_not_detect_a_bee_behind_the_wall() -> None:
    """A bee behind the partition is missing in the camera it hides from and present in a
    camera on the same side as the bee."""
    bee = Bee(0, np.array([0.70, 0.10, 0.15]), np.zeros(3), FLYING)
    cameras = {camera.id: camera for camera in CONFIG.cameras}
    quiet = replace(CONFIG.noise, pixel_sigma_px=0.0, dropout_probability=0.0)

    def detections(camera_id: str) -> tuple:
        camera = cameras[camera_id]
        pose = camera_pose(camera.position_m, camera.look_at_m)
        return observe_bees(pose, CONFIG.camera_models[camera.model], [bee],
                            CONFIG.bees.body_length_m, quiet, np.random.default_rng(1), WALLS)

    assert detections("cam-01") == ()      # at x = 0, the wall is in between
    assert len(detections("cam-04")) == 1  # at x = 0.9, same side as the bee


def test_walls_path_is_relative_to_the_config_and_must_exist(tmp_path: Path) -> None:
    """The STL path is resolved next to the config file; a missing file is refused."""
    assert CONFIG.arena.walls_stl == REPO / "config" / "arenas" / "divided-box.stl"
    broken = tmp_path / "broken.yaml"
    broken.write_text((REPO / "config" / "arena-wall.yaml").read_text())
    with pytest.raises(ConfigError):
        load_config(broken)


def test_tracker_counts_only_cameras_the_wall_leaves_free() -> None:
    """Behind the partition only 8 cameras see a bee, of 14 whose field of view covers it.
    With two detections missed, the coverage rule still accepts the point when it knows the
    wall (4 of 8 needed) and rejects it without (7 of 14 needed)."""
    cameras = tracking_cameras(CONFIG)
    quiet = replace(CONFIG.noise, pixel_sigma_px=0.0, dropout_probability=0.0)
    bee = Bee(0, np.array([0.70, 0.10, 0.15]), np.zeros(3), FLYING)
    detections = {
        camera.id: observe_bees(camera.pose, camera.model, [bee], CONFIG.bees.body_length_m,
                                quiet, np.random.default_rng(1), WALLS)
        for camera in cameras
    }
    seeing = [camera_id for camera_id, found in detections.items() if found]
    assert len(seeing) == 8
    for camera_id in seeing[:2]:
        detections[camera_id] = ()
    rays = frame_rays(detections, cameras)
    no_tracks = (np.zeros(0, dtype=int), np.zeros((0, 3)))
    with_walls = triangulate(rays, *no_tracks, cameras, CONFIG.arena, CONFIG.tracker)
    without = triangulate(rays, *no_tracks, cameras, replace(CONFIG.arena, walls_stl=None),
                          CONFIG.tracker)
    assert len(with_walls.positions_m) == 1 and len(without.positions_m) == 0
