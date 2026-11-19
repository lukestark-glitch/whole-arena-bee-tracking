# Changelog

One entry per stage or notable change, newest first. Record measured results here when a
stage asks for it.

## 2026-11-19: README

- README: short "About this project" section and an up-to-date status.

## 2026-11-18: Results paper

- `docs/results-paper.pdf` and `docs/results-paper.html`: what was built, how to run it, and
  all Stage 7 results with screenshots. Numbers come from `experiments/results/`.

## 2026-11-18: Stage 7 experiments

- `python -m experiments.batch` runs every Stage 7 experiment deterministically in one
  process per run and writes `experiments/results/` (CSV, JSON, SVG plots). That is 41
  cases with seeds 1 to 5, 20 s each, about 20 minutes (D95).
- `experiments/coverage.py`: cameras per point and expected error for a layout.
  `experiments/plots.py`: SVG bar charts in plain Python.
- `experiments/metrics.py`: ghost share, prediction error (17 ms and 100 ms) and the robot's
  follow error (offset from the optical axis and focus error) (D96).
- New layouts `config/cameras-8.yaml`, `cameras-10.yaml` and `cameras-12.yaml` in the
  current box (D94). The "good enough" criterion was committed before the runs (D93).
- Findings in `experiments/REPORT_DATA.md`. 8 cameras see the whole box with at least 4
  cameras and keep the mean error at 0.52 mm (random) and 0.79 mm (trips), but they fail the
  criterion: more ID swaps, a p95 of 2.33 mm with trips, less time in focus, and fragile
  when one of the 4 nodes fails. 10 and 12 cameras fail it too, so 16 stays the smallest
  layout that passes. Without calibration the tracker loses every bee. The wall-marker check
  alone brings it back to 0.47 mm mean error.
- Paper figures of both windows in `docs/paper-figures/`.
- Arena walls from an STL file (`arena.walls_stl`, new dependency `trimesh`, approved, D98).
  Example: `config/arena-wall.yaml`, the box with an opaque partition. A bee behind a wall
  is not detected by the cameras the wall blocks. The tracker leaves those cameras out of
  its coverage rule (D99). Without that, the divided box gave 74 ID swaps and up to 12
  ghost tracks. With it, the cost is small with random flight and moderate with trips (p95
  2.48 mm).

## 2026-11-18: Realistic values, measured

- `config/robot.yaml`: the cinema field of view (0.096 rad, 24 mm wide at 0.25 m) and depth
  of field (2 mm) are back at the Stage 5 values; they are realistic, not widened for
  viewing. The unknown limits take the cautious end of a NEMA 23 drive: rail 10 m/s²
  (was 15), tilt 8 rad/s and 50 rad/s² (was 12 and 90), focus 0.5 m/s and 10 m/s² (was 1
  and 20). The rail stays at 3 m/s, now tagged `stated`. Every other robot value stays
  `assumed` with its derivation (D92).
- `config/default.yaml`: no value changed. Noise, dropout, node delay, frame rate, bee size,
  speed and count already were the documented values of `docs/camera-node.md` and the
  concept paper; the noise section now says so. The config has no trigger jitter.
- No Stage 7 experiment command exists yet, so only the robot was re-measured.

**Robot result** (60 s, 10 bees, seed 1, tracker with `--auto-select`). The robot numbers
depend on assumed robot values (limits, field of view, depth of field) and change when
Jeremy gives real ones. "One process" is the deterministic set-up of
`tests/test_robot_loop.py`. "Real time" is the three-program command of `docs/setup.md`,
which varies from run to run.

| Flight model | Robot values | Time in frame, one process | Time in focus, one process | Time in frame, real time | Time in focus, real time |
| --- | --- | --- | --- | --- | --- |
| random | before (widened view, faster limits) | 80.7 % | 41.1 % | 75.5 % | 40.2 % |
| random | after (realistic) | 52.1 % | 30.8 % | 48.5 % | 30.3 % |
| random | new limits, old widened view | 65.3 % | 35.8 % | | |
| trips | before | 99.0 % | 92.2 % | 98.9 % | 92.2 % |
| trips | after | 97.1 % | 73.9 % | 96.9 % | 71.3 % |
| trips | new limits, old widened view | 98.4 % | 91.5 % | | |

With random flight the slower limits cost about 15 points of time in frame and the narrower
picture about 13 more. With trips the selected bee spends much of the time feeding or
resting, so it stays framed; the 2 mm depth of field costs about 18 points in focus.

**Tracking result** (same real-time runs; does not depend on the robot values):

| Flight model | Mean error | 95th percentile | ID swaps | Ghost tracks |
| --- | --- | --- | --- | --- |
| random | 0.44 mm | 1.11 mm | 23 | 0 |
| trips | 0.55 mm | 1.30 mm | 60 | 1 |

Trips raise the ID swaps, as the plan expected from bees crowding at feeder and entrance.

## 2026-11-18: Flight behaviour models

- New flight model `trips` (`bees.flight_model` in `config/default.yaml`, default stays
  `random`): bees rest near the colony entrance, fly to the feeder, feed, fly back, drawn by
  a light along the arena (D87 to D89, D91). With `random` a fixed seed gives exactly the
  same bees as before (checked over 10 minutes of simulation time).
- Simulator window: a "Flight" card with the flight model selector, the light strength
  and the feeder's wall and position. With `trips` the 3D view shows feeder, entrance and
  light (D90).
- Fixed a blank line that split the control table in `docs/architecture.md`.

## 2026-11-18: Plan for flight behaviour models

- `docs/implementation-plan.md`: new optional section "Flight behaviour models" after
  Stage 7. It plans a second bee motion model (`trips`: light, feeder, entrance, trips along
  the arena), selected in the config, with the current random model as the default. It also
  lists the effects on tracking to be measured in Stage 7. New "Open questions" list.
- Decisions D87 to D89. Planning only, no code changes.

## 2026-11-14: Stage 5 polish 3

- Fixed: the robot stopped following a selected bee while the simulator window was open on
  a busy computer. The simulator read one `target` per turn of its event loop; once it ran
  late a backlog built up and every target was older than the timeout. It now reads all
  waiting targets before each frame (D80). With the page open in a software-rendering
  browser the robot had a target in 37 to 58 % of the snapshots before and in 100 % after.
- Fixed: with a bee selected in the control window the robot moved for a second and then
  stood still. On a busy computer the tracker needed longer than one frame period per
  frame, fell behind without limit, and every `target` was older than the robot's timeout.
  The tracker now reads its sockets empty and skips old frames when it is behind (D84),
  and its filter is faster (D85). The control window shows "Skipped frames".
- The robot model reports a build failure in the console instead of failing silently (D81).
- Robot accent colour is teal instead of purple (D82).
- Tracking cameras: rounded board, lens ring with raised barrel, ribbon cable with a bend,
  bracket arm and foot, status LED.
- Control window: "Show prediction lines" switch (D83).

**Robot limits** (one process, 60 s, 10 bees, seed 1, `target` one frame late):

| Robot file | Time in frame | Time in focus |
| --- | --- | --- |
| Before PR 11 (rail 2 m/s, 20 m/s²; tilt 10 rad/s, 200 rad/s²) | 91.5 % | 43.6 % |
| Now (rail 3 m/s, 15 m/s²; tilt 12 rad/s, 90 rad/s²) | 80.7 % | 41.1 % |
| Now, but rail 20 m/s² | 82.2 % | 42.0 % |
| Now, but tilt 200 rad/s² | 89.9 % | 43.0 % |

The code gives the same numbers before and after PR 11 with the same robot file. The new
limits cost about 11 points of time in frame, almost all of it from the tilt acceleration.
The limits were left as they are.

## 2026-11-14: Stage 5 polish 2

- Fixed: robot and tracking cameras were missing in both windows. Parts with and without an
  index cannot be merged in three.js; the merge returned nothing and the scene handler
  stopped. All parts are now made non-indexed first (D75).
- Fixed: the simulator window drew one bee with still wings, and no robot or cinema view.
  The `state` message had no bee velocity, so the page stopped at the first bee (D76).
- Fixed: the camera panels of the control window did not update. The missing robot model
  stopped the handler of every `tracks` message before the panels were marked for redrawing.
- Control window: confirmed tracks are drawn as bees with flying or folded wings (D77).
- Both windows: tracking cameras as Raspberry Pi camera modules with a status LED (D78), a
  "Show camera IDs" switch (D79), scroll bars in the style of the panels.

## 2026-11-14: Stage 5 polish

- Softer 3D models for bee (bands, fur colours, flapping wings, heading from velocity),
  robot (belt rail, carriage, NEMA 23 motors with gearboxes, camera, lens, stereo pair),
  arena (base plate, glass walls, frame) and tracking cameras. Both windows share them
  (`shared/web/models.js`); the control window now shows the same robot model.
- Shared style sheet and card layout for both windows.
- Control window: "Show trails" switch.
- `config/robot.yaml`: rail 3 m/s and 15 m/s², tilt 12 rad/s and 90 rad/s². The robot
  result below was not re-measured.
- Decisions D70 to D74.

## 2026-11-14: Stage 6, calibration

- Simulator: `--miscalibrated` (true cameras and robot differ from the config files),
  `--wand` (calibration wand run), wall markers in `detections`, control to displace a camera.
- `tracker/calibrate.py`: bundle adjustment of all camera poses from the wand with three
  reference points; pose of one camera from the wall markers.
- `tracker/robot_calibration.py`: rail start, lens offset and focus scale from 15 wand stops.
- `tracker/wand_run.py`: command that collects a wand run and writes `logs/calibration.json`;
  the tracker loads it with `--calibration`.
- Marker check in the tracker: a displaced camera is reported in the control window and
  corrected.
- Control window: robot drawn with the simulator's model (Stage 5 follow-up).
- Config: new `calibration` section (decisions D60 to D69).

**Calibration result** (seed 1, default configuration, cameras off by 1 cm and 2°, 0.5 px noise):

| Measure | Value |
| --- | --- |
| Camera position after calibration, worst camera | 0.13 mm |
| Camera orientation after calibration, worst camera | 0.02° |
| Robot rail start | within 0.3 mm |
| Mean position error, 60 s, 10 bees, own calibration | 0.45 mm (baseline 0.44 mm) |
| 95th percentile position error | 1.12 mm (baseline 1.11 mm) |

The 60 s run used three programs over UDP; the camera numbers are from the same wand run
in one process, where the true poses can be compared.

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
tracker with `--auto-select`; cinema picture 24 mm wide, depth of field 2 mm; both were
doubled in `config/robot.yaml` afterwards for viewing, the numbers below are not re-measured):

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
