"""Tests for the virtual camera observation."""

import numpy as np

from shared.config import CameraModel, Noise
from shared.geometry import camera_pose, focal_length_px, is_visible, project
from simulator.bees import Bee
from simulator.cameras import observe_bees

MODEL = CameraModel("m", 640, 480, 2.0, 60.0)
POSE = camera_pose(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
BODY_LENGTH_M = 0.015
QUIET = Noise(0.0, 0.0, 0.0, 0.0)


def _bee(i: int, x_m: float, y_m: float = 0.0, z_m: float = 0.0) -> Bee:
    """Make a stationary bee at a position (m)."""
    return Bee(i, np.array([x_m, y_m, z_m]), np.zeros(3), "sitting")


def test_noise_free_matches_projection() -> None:
    """Without noise the output equals the projection of the visible bees."""
    bees = [_bee(0, 0.5, 0.1), _bee(1, -0.5), _bee(2, 0.4, -0.1, 0.05)]
    got = observe_bees(POSE, MODEL, bees, BODY_LENGTH_M, QUIET, np.random.default_rng(0))
    points = np.array([b.position_m for b in bees])
    mask = is_visible(POSE, MODEL, points)
    expected = project(POSE, MODEL, points[mask])
    assert len(got) == 2
    np.testing.assert_allclose(np.array(got)[:, :2], expected)


def test_blob_size() -> None:
    """Blob size follows focal length times body length over distance."""
    got = observe_bees(POSE, MODEL, [_bee(0, 0.5)], BODY_LENGTH_M, QUIET, np.random.default_rng(0))
    assert got[0][2] == focal_length_px(MODEL) * 0.015 / 0.5


def test_bee_behind_not_reported() -> None:
    """A bee behind the camera is not reported."""
    got = observe_bees(POSE, MODEL, [_bee(0, -0.5)], BODY_LENGTH_M, QUIET, np.random.default_rng(0))
    assert got == ()


def test_full_dropout() -> None:
    """Dropout probability 1 removes every detection."""
    noise = Noise(0.0, 1.0, 0.0, 0.0)
    got = observe_bees(POSE, MODEL, [_bee(0, 0.5)], BODY_LENGTH_M, noise, np.random.default_rng(0))
    assert got == ()


def test_pixel_noise_std() -> None:
    """Pixel error has a standard deviation close to the configured sigma."""
    noise = Noise(0.5, 0.0, 0.0, 0.0)
    rng = np.random.default_rng(3)
    bees = [_bee(0, 0.5)]
    clean = observe_bees(POSE, MODEL, bees, BODY_LENGTH_M, QUIET, rng)[0]
    errors = [
        observe_bees(POSE, MODEL, bees, BODY_LENGTH_M, noise, rng)[0][0] - clean[0]
        for _ in range(5000)
    ]
    assert 0.4 < float(np.std(errors)) < 0.6


def test_bee_on_the_lens_not_reported() -> None:
    """A bee closer than its body length, or at the camera position, gives no blob."""
    bees = [_bee(0, 0.0), _bee(1, 0.01), _bee(2, 0.5)]
    got = observe_bees(POSE, MODEL, bees, BODY_LENGTH_M, QUIET, np.random.default_rng(0))
    assert len(got) == 1
    assert np.isfinite(got[0]).all()


def test_noisy_points_stay_inside_image() -> None:
    """Noise never moves a reported point outside the image."""
    edge_bee = [_bee(0, 0.5, 0.0, 0.5 * np.tan(239.9 / focal_length_px(MODEL)))]
    noisy = Noise(2.0, 0.0, 0.0, 0.0)
    rng = np.random.default_rng(0)
    for _ in range(200):
        for u_px, v_px, _ in observe_bees(POSE, MODEL, edge_bee, BODY_LENGTH_M, noisy, rng):
            assert 0.0 <= u_px < MODEL.width_px
            assert 0.0 <= v_px < MODEL.height_px
