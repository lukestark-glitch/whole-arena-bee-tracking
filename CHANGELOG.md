# Changelog

One entry per stage or notable change, newest first. Record measured results here when a
stage asks for it.

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
