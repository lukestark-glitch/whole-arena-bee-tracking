# Code map

Every module, what it is for, and its public functions. This is the hand-written overview.
The full function reference is generated from the docstrings (see `docs/setup.md`).

Update this file in every stage. A module that is not listed here should not exist.

Status: Stages 0 and 1 are built.

## How to read this file

Each package gets a section. Each module gets one row.

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| _example:_ `shared/protocol.py` | _Build and parse the UDP messages_ | _`encode_detections`, `decode`_ | _simulator, tracker_ |

## `shared/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `shared/config.py` | Load and validate `config/*.yaml` into dataclasses | `load_config`, `frame_rate_hz`, `ConfigError`, `Config`, `Arena`, `CameraModel`, `Camera`, `Node`, `Noise`, `Bees`, `Ports` | simulator, experiments, tracker (from Stage 3) |
| `shared/protocol.py` | Build and parse the UDP messages of `docs/protocol.md` | `encode_detections`, `encode_target`, `encode_robot_pose`, `encode_truth`, `decode`, `ProtocolError`, `Detections`, `Target`, `OtherBee`, `RobotPose`, `Truth`, `TruthBee`, `RobotAxes`, `UDP_HOST` | simulator, experiments, tracker (from Stage 3) |
| `shared/geometry.py` | Equidistant fisheye camera model: pose, projection, rays, visibility | `camera_pose`, `focal_length_px`, `project`, `pixel_to_ray`, `is_visible`, `CameraPose` | simulator, tracker (from Stage 3) |

## `config/`

| File | Content |
| --- | --- |
| `config/default.yaml` | Arena box, 16 cameras on 8 nodes, camera model `pi_gs_imx296`, noise, bees, UDP ports, seed |

## `tests/`

| Module | Checks |
| --- | --- |
| `tests/test_config.py` | The default config loads; each validation rule fails with a clear message |
| `tests/test_protocol.py` | Round trip of every message type, rounding, splitting at 1400 bytes, decode errors |
| `tests/test_geometry.py` | Pixel-to-ray round trip below 1e-6 m at 1 m, image orientation, visibility |
| `tests/test_bees.py` | Bees stay in the arena for 60 s, speed limit, surface states, same seed gives same run |
| `tests/test_cameras.py` | Projection without noise, blob size, dropout, noise level |
| `tests/test_nodes.py` | Shared node delay, message content, empty cameras still report |
| `tests/test_simulator.py` | Identical output for the same seed, 16 detections and 1 truth per frame, UDP end to end |

## `simulator/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `simulator/bees.py` | Bee motion: sitting, walking, flying, one step per frame | `create_bees`, `step_bees`, `Bee`, `SITTING`, `WALKING`, `FLYING` | `simulator/main.py`, `simulator/cameras.py` |
| `simulator/cameras.py` | What one virtual camera reports for a frame: visible bees with noise, dropout and blob size | `observe_bees` | `simulator/main.py` |
| `simulator/nodes.py` | Camera node: one shared delay for its two cameras, builds the `detections` datagrams | `node_messages`, `Outgoing` | `simulator/main.py` |
| `simulator/main.py` | Fixed-step loop, `truth` output, pacing, command line | `simulate`, `run`, `main` | command line |

The window (`simulator/web/`) is not built yet (Stage 2).

## `robot_emulator/`

Not built yet (Stage 5).

## `tracker/`

Not built yet (Stage 3 and 4).

## `experiments/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `experiments/listen_detections.py` | Listener that prints one summary line of the `detections` per simulation second | `summarise`, `main` | command line |

Metrics and batch runs are not built yet (Stage 3, 5 and 7).
