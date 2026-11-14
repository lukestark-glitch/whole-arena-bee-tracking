# Implementation plan

The work is split into stages. Each stage ends with something that can be seen or measured.
Work on one stage at a time and follow `CLAUDE.md`.

| Stage | Name | Status |
| --- | --- | --- |
| 0 | Foundation | done |
| 1 | Simulator core | done |
| 2 | Simulator window | done |
| 3 | Tracking core | done |
| 4 | Control window | done |
| 5 | Robot | done |
| 6 | Calibration | done |
| 7 | Experiments and other arena shapes | not started |
| 8 | Optional: rendered camera images and a real detector | not planned yet |

Status values: `not started`, `in progress`, `done`.

Every stage ends with the documentation checklist in `CLAUDE.md`. It is not repeated below.

## Target layout

```
config/default.yaml        arena, cameras, nodes, camera models, noise, ports
shared/                    config.py, protocol.py, geometry.py
simulator/                 bees.py, cameras.py, nodes.py, main.py, web/
robot_emulator/            motion.py, stereo.py, controller.py
tracker/                   receiver.py, triangulate.py, tracks.py, main.py, web/
experiments/               metrics.py, run scripts
tests/
docs/
```

Create a file only in the stage that needs it.

---

## Stage 0: Foundation

**Goal.** A project that installs, loads a configuration and can send and read every message.

**Build**

- `pyproject.toml` with the allowed dependencies, `.gitignore` entries as needed.
- `config/default.yaml`: arena box of 0.90 × 0.30 × 0.30 m; 16 cameras (8 corners, 2 along
  each long edge) aimed at the arena centre; 8 nodes with two neighbouring cameras each; one
  camera model `pi_gs_imx296` (1456 × 1088 px, 110° horizontal field of view, 60 Hz);
  noise and dropout values; ports; random seed.
- `shared/config.py`: load the YAML into dataclasses and validate it (every camera belongs
  to exactly one node, every node has at most two cameras, ids are unique).
- `shared/protocol.py`: one `encode_*` and one `decode` function for the messages in
  `docs/protocol.md`, including rounding and the 1400-byte limit.

**Done when**

- `pip install -e ".[dev]"` works in a fresh virtual environment.
- Tests: the default config loads; an invalid config fails with a clear message; every
  message type survives an encode/decode round trip; an oversized `detections` message is
  split.
- `docs/setup.md` describes installation and how to run the tests.

---

## Stage 1: Simulator core

**Goal.** A simulator without a window that sends realistic `detections` and `truth`.

**Build**

- `shared/geometry.py`: camera pose from position and look-at point; equidistant fisheye
  projection (3D point → pixel) and its inverse (pixel → ray in the arena frame); a
  visibility test (in front of the camera and inside the image).
- `simulator/bees.py`: bee state (position, velocity, state `sitting`/`walking`/`flying`),
  one step function per frame. Flying bees change direction smoothly at random, stay inside
  the arena, and do not exceed the configured maximum speed (default 5 m/s). Sitting and
  walking bees stay on a surface. State changes happen at random with configured rates.
- `simulator/cameras.py`: for each camera and frame, project all bees, keep the visible
  ones, add Gaussian pixel noise, drop each detection with the configured probability,
  compute the blob size from distance.
- `simulator/nodes.py`: group cameras by node, apply the node delay, send `detections`.
- `simulator/main.py`: the fixed-step loop, real-time pacing with a time-scale factor,
  `truth` output, command-line options for config file, seed, duration and "as fast as
  possible".

**Done when**

- Tests: projecting a point and turning the pixel back into a ray gives a ray through the
  point (error below 1e-6 m at 1 m); points behind a camera are not visible; bees never
  leave the arena over a 60 s run; the same seed gives identical output.
- Running the simulator and a small listener script prints plausible `detections` for all
  16 cameras at 60 Hz of simulation time.

---

## Stage 2: Simulator window

**Goal.** See the simulated world and change it while it runs.

**Build**

- `simulator/web/`: one HTML page with three.js. Shows the arena, bees (coloured by state),
  cameras with view cones.
- WebSocket in `simulator/main.py`: state snapshots to the page (at most 30 per second of
  wall time), control messages back.
- Controls: number of bees, maximum speed, pixel noise, dropout probability, pause, time
  scale, seed with restart.
- `simulator/cameras.py`: bees whose blobs overlap in a camera image are merged into one
  detection, as `docs/camera-node.md` describes.

**Done when**

- Opening the page shows bees moving in the arena; every control changes the simulation
  without a restart (except seed).
- The page holds no simulation logic. With the page closed, the simulator output is
  identical for the same seed and settings.
- Test: two bees that overlap in a camera image give one detection, two bees apart give two.

---

## Stage 3: Tracking core

**Goal.** The tracker turns `detections` into tracks with stable IDs, and the result is
measured against the truth.

**Build**

- `tracker/receiver.py`: receive `detections`, group by `frame`, release a frame when all
  cameras have reported or after a timeout (default 2 frame periods).
- `tracker/triangulate.py`:
  - Pixels to rays with `shared/geometry.py`. Camera poses come from the config file in
    this stage.
  - Track-guided step: for every existing track, predict its position, project it into each
    camera and claim the nearest detection within a gate. Triangulate from the claimed rays.
  - New candidates from the unclaimed detections: pair rays from different cameras that
    pass within the distance tolerance, then collect further supporting rays.
  - Accept candidates in order of support. A detection supports only one point. A candidate
    needs at least three cameras, and more where more cameras cover that location.
  - The 3D point is the least-squares point closest to all supporting rays.
- `tracker/tracks.py`: constant-acceleration Kalman filter per track; assignment of points
  to tracks with `scipy.optimize.linear_sum_assignment` and a distance gate; a new track is
  tentative until seen in N consecutive frames; a track is removed after M frames without
  support; `sitting`/`flying` state from speed with hysteresis.
- `tracker/main.py`: the loop, a track log file (one JSON line per frame), command-line
  options.
- `experiments/metrics.py`: match tracks to true bees per frame and report mean and 95th
  percentile position error, number of ID swaps, number of ghost tracks, and share of time
  each bee was tracked.

**Done when**

- Tests: triangulating noise-free rays returns the point; two bees and three cameras in the
  layout of the paper's ghost figure produce two points, not four; a filter fed a straight
  constant-speed path recovers the speed; assignment keeps IDs when two paths cross at a
  distance above the gate.
- A 60 s run with 10 bees and default noise prints the metrics. Record the numbers in
  `CHANGELOG.md` as the baseline.
- `tracker/` imports nothing from `simulator/` or `robot_emulator/` (checked by a test).

---

## Stage 4: Control window

**Goal.** See what the tracker believes, select a bee, and watch its values live.

**Build**

- `tracker/web/`: one HTML page with three.js. Shows tracks with ID, a trail of the last
  seconds and the predicted position; a panel per tracking camera with its detections and
  the ID each was assigned to; a health panel.
- WebSocket in `tracker/main.py`: track snapshots to the page, the selected ID back.
- Selection by click. Panel for the selected bee: speed, smoothed acceleration, state,
  number of supporting cameras.
- Health: cameras per track, nodes that have stopped reporting.
- Simulator: a control in the simulator window that stops and restarts a single node. A
  stopped node sends nothing for both of its cameras.

**Done when**

- With simulator and tracker running, both windows side by side show matching motion.
- Clicking a bee selects it and the live values update.
- Stopping a node in the simulator shows up in the health panel within one second.

---

## Stage 5: Robot

**Goal.** The simulated camera robot follows the selected bee and does not jump to others.

**Build**

- `config/robot.yaml`: every robot parameter in its own file, so real values can replace
  the assumptions without touching code: rail position and length, lens reference point,
  tilt axis, limits for speed and acceleration of rail, tilt and focus, cinema field of view
  and depth of field, stereo camera field of view, baseline and noise, control rate. Each
  value has a comment with its unit and its source: `video` (stated in Jeremy Fielding's
  videos), `assumed`, or `measured` (with date and who measured it). The values known from
  the videos are listed in `docs/robot-interface.md`.
- `shared/config.py`: load `config/robot.yaml`; a command-line option selects another robot
  file, so several robot variants can be compared.
- Simulator window: sliders for the speed and acceleration limits of rail and tilt, for
  trying values live. The file stays the source of the values a run starts with.
- `robot_emulator/motion.py`: rail, tilt and focus as limited second-order axes.
- `robot_emulator/stereo.py`: what the robot's stereo camera sees: blobs with image
  position, size and measured distance, with noise.
- `robot_emulator/controller.py`: receive `target`, extrapolate to now, project into the
  stereo image, apply the selection rule in the search window (nearest to the prediction,
  distance within tolerance, not one of `others`), fall back to the prediction when no blob
  qualifies, command the axes with feed-forward from the target velocity, send `robot_pose`.
  Focus presets from `distance_m` on a target change and follows the stereo distance after.
- Tracker: send `target` for the selected track, compute `distance_m` and `others` from
  the last `robot_pose`.
- Simulator window: robot model that moves. Cinema view rendered with a narrow field of
  view and blur by distance from the focus plane, streamed to the control window.
- Metrics: share of time the selected bee is inside the cinema frame, and inside the depth
  of field.

**Done when**

- Selecting a bee makes the robot swing to it and follow it.
- Changing a value in `config/robot.yaml` changes the behaviour on the next start, with no
  code change. A test loads a robot file with changed limits and checks the axes obey them.
- A test scenario in which a larger bee crosses the search window does not change the
  followed bee.
- A 60 s run prints time in frame and time in focus. Record the numbers in `CHANGELOG.md`.

**Stage 5 follow-ups**

- Done: the control window uses the same robot model as the simulator window
  (`shared/web/models.js`).
- Done: softer 3D models (bee, robot, arena, tracking cameras), shared style sheet and card
  layout for both windows, a "Show trails" switch in the control window.
- Done: robot limits for a 3 m/s rail and NEMA 23 steppers with gearboxes.
- Done: fixed the windows after the model change (robot and cameras missing, only one bee
  drawn, camera panels frozen); tracks drawn as bees; Raspberry Pi camera models with a
  status LED; "Show camera IDs" switch in both windows; styled scroll bars.
- Not re-measured: the robot result in `CHANGELOG.md` (rerun `experiments` with the new
  `config/robot.yaml`).

---

## Stage 6: Calibration

**Goal.** The tracker works from a calibration it computed itself.

**Build**

- Simulator: a calibration mode that moves a single point (the LED wand) through the arena,
  and optional wall markers at configured positions.
- `tracker/calibrate.py`: estimate all camera poses from the wand observations with
  `scipy.optimize.least_squares`, starting from the nominal poses in the config with a
  deliberate error. Fix scale and frame with three known reference points.
- Robot calibration: from 10 to 20 wand positions with recorded rail, tilt and focus
  values, fit the robot's pose in the arena frame and the distance-to-focus mapping.
- Wall markers: compare observed with expected marker positions per camera, flag a camera
  whose error exceeds a threshold, and re-estimate that camera's pose.
- Simulator control to displace one camera by a small angle.

**Done when**

- Tests: camera poses perturbed by 2° and 1 cm are recovered to within 0.1° and 1 mm from
  noise-free wand data.
- The Stage 3 run with self-computed calibration is within 20 % of the baseline position
  error.
- Displacing a camera is reported in the control window and corrected.

---

## Stage 7: Experiments and other arena shapes

**Goal.** Numbers that answer the questions in the concept paper.

**Build**

- `experiments/coverage.py`: for a grid of points, the number of cameras that see each
  point and the expected position error. Output as a file and a simple plot page.
- Batch runner: run a list of configurations without windows and collect the metrics in
  one table (CSV).
- Experiment configurations: 8, 12 and 16 cameras; 5, 10, 20 and 40 bees; one node failed;
  node delay of 5, 15 and 30 ms.
- Arena from a triangle mesh (STL file) and an occlusion test (ray against mesh) in the
  visibility check. Needs the `trimesh` dependency: record the decision first.
- `docs/results.md`: the tables with a short reading of each.

**Done when**

- One command reproduces every table in `docs/results.md`.
- A non-box arena loads and bees hidden behind a wall are not detected by the blocked
  cameras.

---

## Stage 8 (optional, not planned yet)

Render a grey-scale image per tracking camera and run a detector that could run on a
Raspberry Pi (background subtraction and blob detection as described in
`docs/camera-node.md`). The detector replaces the point generator in `simulator/cameras.py`
behind the same output. Decide after Stage 7 whether this is worth building.
