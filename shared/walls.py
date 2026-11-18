"""Opaque walls inside the arena box: which points a camera cannot see behind them.

The walls come from the STL file named by ``arena.walls_stl`` (D98); the box itself is glass
and hides nothing. The simulator uses this to hide bees from cameras, the tracker to know
which cameras can cover a place (it is configuration, not ground truth), and
`experiments/coverage.py` to count cameras. Positions are in the arena frame, in metres.
"""

from functools import cache
from pathlib import Path

import numpy as np
import trimesh

# A wall hit closer than this to the bee does not count: a bee sitting on a wall is seen
# from the wall's front side, and rounding must not hide it.
SURFACE_TOLERANCE_M = 1e-4
# Below this determinant a line of sight runs parallel to a triangle (metres cubed).
_PARALLEL_EPSILON = 1e-15


@cache
def load_walls(path: Path | None) -> trimesh.Trimesh | None:
    """Load the walls of an arena; each file is read once.

    Args:
        path: STL file (metres, arena frame), or None for an empty box.

    Returns:
        The wall mesh, or None.
    """
    if path is None:
        return None
    return trimesh.load_mesh(path, file_type="stl")


def hidden(
    walls: trimesh.Trimesh | None, origin_m: np.ndarray, points_m: np.ndarray
) -> np.ndarray:
    """Tell which points a wall hides from a viewpoint.

    Every line of sight is tested against every wall triangle with the Moller-Trumbore
    ray-triangle test (Moller and Trumbore, 1997). trimesh's own ray test would need the
    `rtree` package; wall meshes have few triangles, so the plain test is fast enough.

    Args:
        walls: Wall mesh, or None for no walls.
        origin_m: Viewpoint (camera centre), shape (3,), arena frame, metres.
        points_m: Points, shape (N, 3), arena frame, metres.

    Returns:
        Shape (N,) booleans, True where a wall lies on the line of sight before the point.
    """
    if walls is None or len(points_m) == 0:
        return np.zeros(len(points_m), dtype=bool)
    corners = walls.triangles                      # (T, 3, 3)
    edge_1 = corners[:, 1] - corners[:, 0]         # (T, 3)
    edge_2 = corners[:, 2] - corners[:, 0]
    segments_m = points_m - origin_m               # (N, 3): origin to point, so t in [0, 1]
    p_vec = np.cross(segments_m[:, None, :], edge_2[None, :, :])        # (N, T, 3)
    det = np.einsum("tk,ntk->nt", edge_1, p_vec)
    # Lines parallel to a triangle never cross it; avoid dividing by zero for them.
    parallel = np.abs(det) < _PARALLEL_EPSILON
    inv_det = 1.0 / np.where(parallel, 1.0, det)
    to_origin = origin_m[None, :] - corners[:, 0]  # (T, 3)
    u = np.einsum("tk,ntk->nt", to_origin, p_vec) * inv_det
    q_vec = np.cross(to_origin, edge_1)             # (T, 3)
    v = np.einsum("nk,tk->nt", segments_m, q_vec) * inv_det
    t = np.einsum("tk,tk->t", edge_2, q_vec)[None, :] * inv_det
    # The share of the segment before the point that the tolerance leaves out.
    end = 1.0 - SURFACE_TOLERANCE_M / np.linalg.norm(segments_m, axis=1)[:, None]
    crossing = ~parallel & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > 0.0) & (t < end)
    return crossing.any(axis=1)
