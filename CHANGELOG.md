# Changelog

One entry per stage or notable change, newest first. Record measured results here when a
stage asks for it.

## 2026-11-14: Stage 5, robot

- `config/robot.yaml`: every robot value with unit and source; `--robot PATH` selects
  another file (simulator, tracker, metrics).
- `robot_emulator/`: axes with speed and acceleration limits, stereo camera with noise,
  controller with the selection rule, fallback to the prediction, feed-forward and focus
  preset. It runs inside the simulator and speaks `target` and `robot_pose`.
- Tracker: sends `target` for the selected track, reads `robot_pose`, option
  `--auto-select`, field `selected` in the track log.
- Protocol: entries of `others` in `target` also carry `vel`.
- Simulator window: robot model, cinema view, sliders for the limits of rail and tilt.
  Control window: cinema view as an image stream.
- Metrics: time in frame and time in focus of the selected bee.
- `CLAUDE.md`: a push notification is sent when a stage's pull request is open.
- Decisions D47 to D59.

**Robot result** (60 s, 10 bees, seed 1, default configuration and `config/robot.yaml`,
tracker with `--auto-select`; cinema picture 24 mm wide, depth of field 2 mm):

| Run | Time in frame | Time in focus |
| --- | --- | --- |
| Three programs in real time, run 1 | 36.3 % | 19.0 % |
| Three programs in real time, run 2 | 55.1 % | 23.9 % |
| One process (`tests/test_robot_loop.py` set-up), `target` one frame late | 80.3 % | 34.4 % |
| Same, two frames late | 70.6 % | 32.5 % |
| Same, three frames late | 60.3 % | 31.0 % |

Tracking itself is unchanged (0.44 mm mean error, 3600 frames).

Reading: the result depends strongly on how old `target` is when the robot uses it, about
10 points of time in frame per frame of delay. The real-time runs differ from each other
because that delay depends on how busy the computer is (both were measured on a loaded
machine). The one-process numbers repeat exactly. Nearly all robot values are still
assumptions, so the numbers show the mechanism, not the real robot.

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
