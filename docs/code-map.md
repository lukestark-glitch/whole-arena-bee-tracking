# Code map

Every module, what it is for, and its public functions. This is the hand-written overview.
The full function reference is generated from the docstrings (see `docs/setup.md`).

Update this file in every stage. A module that is not listed here should not exist.

Status: Stage 0 is built.

## How to read this file

Each package gets a section. Each module gets one row.

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| _example:_ `shared/protocol.py` | _Build and parse the UDP messages_ | _`encode_detections`, `decode`_ | _simulator, tracker_ |

## `shared/`

| Module | Purpose | Public functions and classes | Used by |
| --- | --- | --- | --- |
| `shared/config.py` | Load and validate `config/*.yaml` into dataclasses | `load_config`, `ConfigError`, `Config`, `Arena`, `CameraModel`, `Camera`, `Node`, `Noise`, `Ports` | simulator, tracker (from Stage 1) |
| `shared/protocol.py` | Build and parse the UDP messages of `docs/protocol.md` | `encode_detections`, `encode_target`, `encode_robot_pose`, `encode_truth`, `decode`, `ProtocolError`, `Detections`, `Target`, `OtherBee`, `RobotPose`, `Truth`, `TruthBee`, `RobotAxes` | simulator, tracker (from Stage 1) |

`shared/geometry.py` is not built yet (Stage 1).

## `config/`

| File | Content |
| --- | --- |
| `config/default.yaml` | Arena box, 16 cameras on 8 nodes, camera model `pi_gs_imx296`, noise, UDP ports, seed |

## `tests/`

| Module | Checks |
| --- | --- |
| `tests/test_config.py` | The default config loads; each validation rule fails with a clear message |
| `tests/test_protocol.py` | Round trip of every message type, rounding, splitting at 1400 bytes, decode errors |

## `simulator/`

Not built yet (Stage 1 and 2).

## `robot_emulator/`

Not built yet (Stage 5).

## `tracker/`

Not built yet (Stage 3 and 4).

## `experiments/`

Not built yet (Stage 3, 5 and 7).
