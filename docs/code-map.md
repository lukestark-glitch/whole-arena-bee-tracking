# Code map

Every module, what it is for, and its public functions. This is the hand-written overview.
The full function reference is generated from the docstrings (see `docs/setup.md`).

Update this file in every stage. A module that is not listed here should not exist.

Status: Stages 0 to 4 are built.

## How to read this file

Each package gets a section. Each module gets one row.

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| _example:_ `shared/protocol.py` | _Build and parse the UDP messages_ | _`encode_detections`, `decode`_ | _simulator, tracker_ |

## `shared/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `shared/config.py` | Load and validate `config/*.yaml` into dataclasses | `load_config`, `frame_rate_hz`, `last_frame_of`, `ConfigError`, `Config`, `Arena`, `CameraModel`, `Camera`, `Node`, `Noise`, `Bees`, `TrackerSettings`, `Ports` | simulator, tracker, experiments |
| `shared/protocol.py` | Build and parse the UDP messages of `docs/protocol.md` | `encode_detections`, `encode_target`, `encode_robot_pose`, `encode_truth`, `decode`, `ProtocolError`, `Detections`, `Target`, `OtherBee`, `RobotPose`, `Truth`, `TruthBee`, `RobotAxes`, `UDP_HOST` | simulator, tracker, experiments |
| `shared/web.py` | Serve a window's page on the port of its WebSocket | `page_response` | simulator, tracker |
| `shared/geometry.py` | Equidistant fisheye camera model: pose, projection, rays, visibility | `camera_pose`, `focal_length_px`, `project`, `pixel_to_ray`, `is_visible`, `CameraPose` | simulator, tracker |

## `config/`

| File | Content |
| --- | --- |
| `config/default.yaml` | Arena box, 16 cameras on 8 nodes, camera model `pi_gs_imx296`, noise, bees, tracker settings, ports, seed |

## `tests/`

| Module | Checks |
| --- | --- |
| `tests/test_config.py` | The default config loads; each validation rule fails with a clear message |
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
| `tests/test_control_window.py` | `scene` and `tracks` messages, silent nodes, selection, WebSocket and page end to end |

## `simulator/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `simulator/bees.py` | Bee motion: sitting, walking, flying, one step per frame | `create_bees`, `create_bee`, `step_bees`, `Bee`, `SITTING`, `WALKING`, `FLYING` | `simulator/main.py`, `simulator/cameras.py` |
| `simulator/cameras.py` | What one virtual camera reports for a frame: visible bees, overlapping blobs merged, with noise, dropout and blob size | `observe_bees` | `simulator/main.py` |
| `simulator/nodes.py` | Camera node: one shared delay for its two cameras, builds the `detections` datagrams | `node_messages`, `Outgoing` | `simulator/main.py` |
| `simulator/main.py` | Simulation state and step, controls, `truth` output, pacing, WebSocket and page server, command line | `Simulation`, `create_simulation`, `step_simulation`, `apply_control` (including the stop of a node), `scene_message`, `state_message`, `run`, `main` | command line |
| `simulator/web/index.html` | Simulator window: draws arena, bees and cameras with three.js, sends control changes. No simulation logic | (browser page) | served by `simulator/main.py` |

## `robot_emulator/`

Not built yet (Stage 5).

## `tracker/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `tracker/receiver.py` | Group `detections` by frame; release a frame when all cameras reported or after the timeout; notice a restart of the sender | `FrameAssembler`, `Frame`, `frames_from_datagram` | `tracker/main.py` |
| `tracker/triangulate.py` | Pixels to rays, points for known tracks first, then new points with ghost rejection | `triangulate`, `guided_points`, `new_points`, `frame_rays`, `closest_point_to_rays`, `point_ray_distances`, `tracking_cameras`, `TrackingCamera`, `Rays`, `Points`, `NEW_POINT` | `tracker/main.py` |
| `tracker/tracks.py` | One Kalman filter per bee, assignment of points to tracks, probation, removal, sitting/flying | `predict_tracks`, `update_tracks`, `assign`, `extrapolate`, `create_track_set`, `Track`, `TrackSet`, `SITTING`, `FLYING` | `tracker/main.py` |
| `tracker/main.py` | The loop: one frame in, one line of the track log out; snapshots for the control window, selection, node health; command line | `Tracker`, `create_tracker`, `process_frame`, `scene_message`, `tracks_message`, `apply_selection`, `run`, `main` | command line |
| `tracker/web/index.html` | Control window: tracks with ID, trail and prediction, selection by click, camera panels, health. No logic that affects results | (browser page) | served by `tracker/main.py` |

## `experiments/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `experiments/listen_detections.py` | Listener that prints one summary line of the `detections` per simulation second | `summarise`, `main` | command line |
| `experiments/metrics.py` | Compare the track log with `truth`: position error, ID swaps, ghost tracks, tracked share. The only module that reads both | `compute_metrics`, `read_track_log`, `format_metrics`, `Metrics`, `main` | command line |

Batch runs are not built yet (Stage 7).
