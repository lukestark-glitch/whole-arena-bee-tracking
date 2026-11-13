# Changelog

One entry per stage or notable change, newest first. Record measured results here when a
stage asks for it.

## 2026-11-13: Stage 4, control window

- Control window at <http://127.0.0.1:8766/>: tracks with ID, trail and predicted position,
  selection by click, live values of the selected bee, a panel per camera with its
  detections and their IDs, health panel.
- Tracker: snapshots over WebSocket, selection, node health, fresh start when the
  simulator restarts.
- Simulator window: stop and restart single camera nodes.
- Config: port `control_window`, three new tracker settings (decisions D39 to D45).

## 2026-11-13: Stage 3, tracking core

- `tracker/`: receiver, triangulation (known tracks first, then new points with ghost
  rejection), one constant-acceleration Kalman filter per bee, track log.
- `experiments/metrics.py`: position error, ID swaps, ghost tracks, tracked share.
- Config: new `tracker` section (decisions D32 to D38).
- Fixed: a time scale above 1 ran too slowly because of sub-millisecond waits.

**Baseline** (60 s, 10 bees, default noise, seed 1, default configuration):

| Metric | Value |
| --- | --- |
| Frames evaluated | 3600 |
| Mean position error | 0.44 mm |
| 95th percentile position error | 1.11 mm |
| ID swaps | 23 |
| Ghost tracks | 0 |
| Confirmed tracks (10 bees) | 14 |
| Mean share of time tracked | 99.6 % |
| Lowest share of time tracked | 97.2 % (bee 8) |

Reading: the position error is below the 1 mm estimate of the concept paper. The weak
point is identity. Tracks break when a bee turns sharply at a wall, and two bees closer
than about one body length merge into one detection in most cameras and are tracked as
one until they separate.

## 2026-11-13: Stage 2, simulator window

- Simulator window at <http://127.0.0.1:8765/>: arena, bees coloured by state, cameras with
  view cones.
- Controls that change the running simulation: number of bees, maximum speed, pixel noise,
  dropout, pause, time scale, seed with restart.
- Bees whose blobs overlap in a camera image are merged into one detection (D27).
- Config: new port `simulator_window` (decisions D27 to D31).

## 2026-11-13: Stage 1, simulator core

- `shared/geometry.py`: camera pose, equidistant fisheye projection, pixel to ray,
  visibility test.
- `simulator/`: bee model (sitting, walking, flying), virtual cameras with noise, dropout
  and blob size, camera nodes with delay, fixed-step loop with real-time pacing.
- `python -m simulator.main` sends `detections` and `truth`.
  `python -m experiments.listen_detections` shows what arrives.
- Config: new `bees` section. All cameras must share one frame rate (decisions D19 to D25).

## 2026-11-13: Stage 0, foundation

- The project installs with `pip install -e ".[dev]"`.
- `config/default.yaml`: arena, 16 cameras on 8 nodes, camera model, noise, ports, seed.
- `shared/config.py` loads and validates the configuration.
- `shared/protocol.py` encodes and decodes `detections`, `target`, `robot_pose` and `truth`,
  with rounding and splitting below 1400 bytes.
- `docs/protocol.md`: splitting rule and rounding table made complete, `truth.robot` is
  optional (decisions D14 to D18).

## 2026-11-13: Readable concept paper

- Added the concept paper as Markdown (`docs/concept-paper.md`) with its figures as SVG
  files, so it can be read directly on GitHub. Removed the HTML source.

## 2026-11-13: Project foundation

- Added the concept paper, the implementation plan, the rules for Claude Code and the
  documentation skeleton.
- No code yet.
