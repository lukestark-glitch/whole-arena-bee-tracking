# Code map

Every module, what it is for, and its public functions. This is the hand-written overview.
The full function reference is generated from the docstrings (see `docs/setup.md`).

Update this file in every stage. A module that is not listed here should not exist.

Status: Stages 0 to 5 are built.

## How to read this file

Each package gets a section. Each module gets one row.

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| _example:_ `shared/protocol.py` | _Build and parse the UDP messages_ | _`encode_detections`, `decode`_ | _simulator, tracker_ |

## `shared/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `shared/config.py` | Load and validate `config/*.yaml` into dataclasses | `load_config`, `load_robot`, `frame_rate_hz`, `last_frame_of`, `ConfigError`, `Config`, `Arena`, `CameraModel`, `Camera`, `Node`, `Noise`, `Bees`, `TrackerSettings`, `Ports`, `Robot`, `AxisLimits`, `Cinema`, `StereoCamera`, `RobotControl` | simulator, robot_emulator, tracker, experiments |
| `shared/protocol.py` | Build and parse the UDP messages of `docs/protocol.md` | `encode_detections`, `encode_target`, `encode_robot_pose`, `encode_truth`, `decode`, `ProtocolError`, `Detections`, `Target`, `OtherBee`, `RobotPose`, `Truth`, `TruthBee`, `RobotAxes`, `UDP_HOST` | simulator, tracker, experiments |
| `shared/web.py` | Serve a window's page on the port of its WebSocket | `page_response` | simulator, tracker |
| `shared/geometry.py` | Equidistant fisheye camera model: pose, projection, rays, visibility. Pose of the robot's cameras from its axes, and what the cinema camera frames | `camera_pose`, `focal_length_px`, `project`, `pixel_to_ray`, `is_visible`, `robot_camera_pose`, `cinema_view`, `CameraPose` | simulator, robot_emulator, tracker, experiments |

## `config/`

| File | Content |
| --- | --- |
| `config/default.yaml` | Arena box, 16 cameras on 8 nodes, camera model `pi_gs_imx296`, noise, bees, tracker settings, ports, seed |
| `config/robot.yaml` | The camera robot: rail, tilt, focus with limits, lens reference point, cinema camera, stereo camera, control settings. Every value with unit and source |

## `tests/`

| Module | Checks |
| --- | --- |
| `tests/test_config.py` | The default config and the robot file load; each validation rule fails with a clear message |
| `tests/test_protocol.py` | Round trip of every message type, rounding, splitting at 1400 bytes, decode errors |
| `tests/test_geometry.py` | Pixel-to-ray round trip below 1e-6 m at 1 m, image orientation, visibility |
| `tests/test_bees.py` | Bees stay in the arena for 60 s, speed limit, surface states, same seed gives same run |
| `tests/test_cameras.py` | Projection without noise, blob size, merging of overlapping blobs, dropout, noise level |
| `tests/test_nodes.py` | Shared node delay, message content, empty cameras still report |
| `tests/test_simulator.py` | Identical output for the same seed and controls, every control, 16 detections and 1 truth per frame, UDP end to end |
| `tests/test_window.py` | `scene` and `state` messages, WebSocket controls, page served over HTTP |
| `tests/test_receiver.py` | Complete and timed-out frames, release order, late and split messages |
| `tests/test_triangulate.py` | Noise-free rays give the point, the ghost figure gives two points, guided step, coverage rule |
| `tests/test_tracks.py` | Filter recovers a constant speed, IDs survive crossing paths, probation, removal, hysteresis |
| `tests/test_metrics.py` | Every metric on hand-made cases, log reading, truth over UDP |
| `tests/test_tracker.py` | `tracker/` imports no simulator code and no clock; ten simulated bees tracked end to end; restart of the sender |
| `tests/test_motion.py` | Axes keep their speed and acceleration limits, reach a target, follow a moving one; a robot file with changed limits is obeyed |
| `tests/test_stereo.py` | Blob position, size and distance without noise, noise levels, same seed gives same blobs |
| `tests/test_controller.py` | Selection rule, swing and follow, a larger bee crossing the search window is not followed, focus preset, hold after silence |
| `tests/test_robot_loop.py` | Simulator, tracker and robot in one process: the selected bee is framed; same seed gives the same result |
| `tests/test_control_window.py` | `scene` and `tracks` messages, silent nodes, selection, WebSocket and page end to end |

## `simulator/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `simulator/bees.py` | Bee motion: sitting, walking, flying, one step per frame | `create_bees`, `create_bee`, `step_bees`, `Bee`, `SITTING`, `WALKING`, `FLYING` | `simulator/main.py`, `simulator/cameras.py` |
| `simulator/cameras.py` | What one virtual camera reports for a frame: visible bees, overlapping blobs merged, with noise, dropout and blob size | `observe_bees` | `simulator/main.py` |
| `simulator/nodes.py` | Camera node: one shared delay for its two cameras, builds the `detections` datagrams | `node_messages`, `Outgoing` | `simulator/main.py` |
| `simulator/main.py` | Simulation state and step (bees, cameras, nodes and the robot), controls, `truth` output, `target` input, pacing, WebSocket and page server, relay of the cinema view, command line | `Simulation`, `create_simulation`, `step_simulation`, `apply_control` (including the stop of a node and the robot limits), `apply_target`, `scene_message`, `state_message`, `run`, `main` | command line |
| `simulator/web/index.html` | Simulator window: draws arena, bees, cameras and the robot with three.js, draws the cinema view and sends it as images, sends control changes. No simulation logic | (browser page) | served by `simulator/main.py` |

## `robot_emulator/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `robot_emulator/motion.py` | One robot axis with speed and acceleration limits | `step_axis`, `Axis` | `robot_emulator/controller.py` |
| `robot_emulator/stereo.py` | What the robot's stereo camera reports: blobs with image position, size and measured distance, with noise | `observe_stereo`, `Blob` | `simulator/main.py`, `robot_emulator/controller.py` |
| `robot_emulator/controller.py` | The robot's software: take `target`, extrapolate, selection rule in the search window, axis commands with feed-forward, `robot_pose` | `create_controller`, `receive_target`, `has_target`, `predict_target`, `select_blob`, `apply_exposure`, `step_controller`, `pose_message`, `Controller` | `simulator/main.py` |

## `tracker/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `tracker/receiver.py` | Group `detections` by frame; release a frame when all cameras reported or after the timeout; notice a restart of the sender | `FrameAssembler`, `Frame`, `frames_from_datagram` | `tracker/main.py` |
| `tracker/triangulate.py` | Pixels to rays, points for known tracks first, then new points with ghost rejection | `triangulate`, `guided_points`, `new_points`, `frame_rays`, `closest_point_to_rays`, `point_ray_distances`, `tracking_cameras`, `TrackingCamera`, `Rays`, `Points`, `NEW_POINT` | `tracker/main.py` |
| `tracker/tracks.py` | One Kalman filter per bee, assignment of points to tracks, probation, removal, sitting/flying | `predict_tracks`, `update_tracks`, `assign`, `extrapolate`, `create_track_set`, `Track`, `TrackSet`, `SITTING`, `FLYING` | `tracker/main.py` |
| `tracker/main.py` | The loop: one frame in, one line of the track log and one `target` out; `robot_pose` in; snapshots for the control window, selection, node health; command line | `Tracker`, `create_tracker`, `process_frame`, `target_message`, `apply_robot_pose`, `scene_message`, `tracks_message`, `apply_selection`, `run`, `main` | command line |
| `tracker/web/index.html` | Control window: tracks with ID, trail and prediction, selection by click, camera panels, health, cinema view. No logic that affects results | (browser page) | served by `tracker/main.py` |

## `experiments/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `experiments/listen_detections.py` | Listener that prints one summary line of the `detections` per simulation second | `summarise`, `main` | command line |
| `experiments/metrics.py` | Compare the track log with `truth`: position error, ID swaps, ghost tracks, tracked share; time of the selected bee in the cinema frame and in focus. The only module that reads both | `compute_metrics`, `compute_robot_metrics`, `read_track_log`, `read_selection`, `format_metrics`, `format_robot_metrics`, `Metrics`, `RobotMetrics`, `main` | command line |

Batch runs are not built yet (Stage 7).
