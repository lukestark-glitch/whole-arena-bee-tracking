"""Camera coverage of the arena: how many cameras see each point and how well it is located.

For a grid of points inside the arena box it counts the tracking cameras that see each
point (``shared.geometry.is_visible``) and estimates the position error a triangulation
would have there from the pixel noise and the ray geometry. The tracker's ghost-point rule
needs three cameras, so the share of the volume seen by three or more is the key number.
Results go to a JSON file and one SVG picture per configuration. Run:

    python -m experiments.coverage --configs config/default.yaml config/cameras-8.yaml

It reads only the configuration, never a run, so it needs no simulator.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from shared.config import Config, load_config
from shared.geometry import camera_pose, focal_length_px, is_visible

# Grid spacing of the evaluated points, in metres: 9000 points in the default box, fine
# enough to see the corners and quick to compute.
GRID_SPACING_M = 0.015
# Camera counts reported as shares of the volume; 6 shows the margin above the 3 the
# ghost-point rule needs.
COVERAGE_LEVELS = (2, 3, 4, 6)
# Heights of the horizontal slices drawn in the SVG pictures, as shares of the box height.
SLICE_HEIGHTS = (0.1, 0.5, 0.9)
# Size of one grid cell in the SVG, in SVG pixels.
SVG_CELL_PX = 6
# Space between slices and for the labels in the SVG, in SVG pixels.
SVG_GAP_PX = 30
# Fill colours for 0, 1, ... and 8 or more cameras: red below the 3 cameras the ghost-point
# rule needs, then light to dark green.
COUNT_COLOURS = ("#d7301f", "#fc8d59", "#fdd49e", "#d9f0a3", "#addd8e", "#78c679", "#41ab5d",
                 "#238443", "#005a32")
MM_PER_M = 1000.0
PERCENT = 100.0
DEFAULT_OUT = Path("experiments/results")


@dataclass(frozen=True)
class Coverage:
    """Coverage of one camera layout.

    Attributes:
        config: Path of the configuration file.
        cameras: Number of tracking cameras.
        nodes: Number of camera nodes.
        points: Number of grid points evaluated.
        share_by_level: Minimum camera count -> share of the points seen by at least that
            many cameras (0..1).
        median_error_m: Median expected position error over the points seen by at least two
            cameras, in metres.
        p95_error_m: 95th percentile of that error, in metres.
    """

    config: str
    cameras: int
    nodes: int
    points: int
    share_by_level: dict[int, float]
    median_error_m: float
    p95_error_m: float


def grid_points(size_m: tuple[float, float, float], spacing_m: float) -> np.ndarray:
    """Return the centres of a regular grid of cells filling the box.

    Args:
        size_m: Box size, arena frame, metres.
        spacing_m: Cell size in metres.

    Returns:
        Shape (N, 3), arena frame, metres.
    """
    axes = [np.arange(spacing_m / 2, size, spacing_m) for size in size_m]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def visibility(config: Config, points_m: np.ndarray) -> np.ndarray:
    """Tell which camera sees which point.

    Args:
        config: Configuration with cameras and camera models.
        points_m: Shape (N, 3), arena frame, metres.

    Returns:
        Shape (C, N) booleans, one row per camera in config order.
    """
    return np.array([
        is_visible(camera_pose(c.position_m, c.look_at_m), config.camera_models[c.model], points_m)
        for c in config.cameras
    ])


def expected_errors_m(config: Config, points_m: np.ndarray, seen: np.ndarray) -> np.ndarray:
    """Estimate the RMS triangulation error at each point from pixel noise and ray angles.

    Each camera places the point on its ray with a sideways standard deviation of distance
    times the angular noise (pixel noise over the focal length). The least-squares point of
    several rays then has the inverse covariance sum_i (I - u_i u_i^T) / sigma_i^2, the
    standard linearised result (Hartley and Zisserman, Multiple View Geometry, ch. 12).

    Args:
        config: Configuration (cameras, camera models, pixel noise).
        points_m: Shape (N, 3), arena frame, metres.
        seen: Shape (C, N) from `visibility`.

    Returns:
        Shape (N,) RMS position error in metres; inf where fewer than two cameras see it.
    """
    information = np.zeros((len(points_m), 3, 3))
    for camera, row in zip(config.cameras, seen):
        model = config.camera_models[camera.model]
        sigma_rad = config.noise.pixel_sigma_px / focal_length_px(model)
        offset_m = points_m[row] - np.asarray(camera.position_m)
        distance_m = np.linalg.norm(offset_m, axis=1)
        u = offset_m / distance_m[:, None]
        across = np.eye(3)[None] - u[:, :, None] * u[:, None, :]
        information[row] += across / ((distance_m * sigma_rad) ** 2)[:, None, None]
    errors_m = np.full(len(points_m), np.inf)
    usable = seen.sum(axis=0) >= 2
    errors_m[usable] = np.sqrt(np.trace(np.linalg.inv(information[usable]), axis1=1, axis2=2))
    return errors_m


def coverage_of(path: Path, spacing_m: float) -> tuple[Coverage, np.ndarray, np.ndarray]:
    """Compute the coverage of one configuration file.

    Args:
        path: Configuration file.
        spacing_m: Grid spacing in metres.

    Returns:
        The summary, the grid points (N, 3) and the camera count per point (N,).
    """
    config = load_config(path)
    points_m = grid_points(config.arena.size_m, spacing_m)
    seen = visibility(config, points_m)
    counts = seen.sum(axis=0)
    errors_m = expected_errors_m(config, points_m, seen)
    located = errors_m[np.isfinite(errors_m)]
    summary = Coverage(
        config=str(path),
        cameras=len(config.cameras),
        nodes=len(config.nodes),
        points=len(points_m),
        share_by_level={level: float(np.mean(counts >= level)) for level in COVERAGE_LEVELS},
        median_error_m=float(np.median(located)),
        p95_error_m=float(np.percentile(located, 95.0)),
    )
    return summary, points_m, counts


def coverage_svg(
    summary: Coverage, points_m: np.ndarray, counts: np.ndarray, spacing_m: float
) -> str:
    """Draw horizontal slices of the camera count as an SVG picture.

    Args:
        summary: Coverage of the layout (for the title).
        points_m: Grid points, shape (N, 3), arena frame, metres.
        counts: Camera count per point, shape (N,).
        spacing_m: Grid spacing in metres.

    Returns:
        SVG text: one map per slice height, x to the right, y up the page.
    """
    index = np.round(points_m / spacing_m - 0.5).astype(int)
    nx, ny, nz = index.max(axis=0) + 1
    width_px = nx * SVG_CELL_PX
    height_px = ny * SVG_CELL_PX
    parts = []
    for slot, share in enumerate(SLICE_HEIGHTS):
        layer = min(int(share * nz), nz - 1)
        top_px = SVG_GAP_PX + slot * (height_px + SVG_GAP_PX)
        z_m = (layer + 0.5) * spacing_m
        parts.append(f'<text x="0" y="{top_px - 6}" font-size="12">z = {z_m:.2f} m</text>')
        for (i, j, k), count in zip(index, counts):
            if k != layer:
                continue
            colour = COUNT_COLOURS[min(int(count), len(COUNT_COLOURS) - 1)]
            y_px = top_px + (ny - 1 - j) * SVG_CELL_PX
            parts.append(f'<rect x="{i * SVG_CELL_PX}" y="{y_px}" width="{SVG_CELL_PX}" '
                         f'height="{SVG_CELL_PX}" fill="{colour}"/>')
    total_px = SVG_GAP_PX + len(SLICE_HEIGHTS) * (height_px + SVG_GAP_PX)
    shares = ", ".join(f"≥{level}: {share * PERCENT:.0f} %"
                       for level, share in summary.share_by_level.items())
    last = len(COUNT_COLOURS) - 1
    legend = " ".join(f'<tspan fill="{colour}">■</tspan> {n}{"+" if n == last else ""}'
                      for n, colour in enumerate(COUNT_COLOURS))
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{max(width_px, 320)}" '
            f'height="{total_px + 20}" font-family="sans-serif">'
            f'<text x="0" y="14" font-size="13">{summary.cameras} cameras ({shares})</text>'
            + "".join(parts)
            + f'<text x="0" y="{total_px + 12}" font-size="12">cameras: {legend}</text></svg>')


def main() -> None:
    """Compute the coverage of the given configurations and write JSON and SVG files."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--configs", type=Path, nargs="+", default=[Path("config/default.yaml")])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output directory")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    summaries = []
    for path in args.configs:
        summary, points_m, counts = coverage_of(path, GRID_SPACING_M)
        summaries.append(asdict(summary))
        (args.out / f"coverage-{path.stem}.svg").write_text(
            coverage_svg(summary, points_m, counts, GRID_SPACING_M), encoding="utf-8")
        shares = "  ".join(f">={level}: {share * PERCENT:5.1f} %"
                           for level, share in summary.share_by_level.items())
        print(f"{path}: {summary.cameras} cameras  {shares}  "
              f"error median {summary.median_error_m * MM_PER_M:.3f} mm, "
              f"p95 {summary.p95_error_m * MM_PER_M:.3f} mm")
    (args.out / "coverage.json").write_text(json.dumps(summaries, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
