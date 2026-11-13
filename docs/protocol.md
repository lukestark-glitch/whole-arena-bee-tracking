# Protocol

This file is binding. Code in `shared/protocol.py` must match it. To change a message,
change this file first.

## Transport

- UDP on localhost, one JSON object per datagram, UTF-8.
- Every message has `type` (string) and `v` (protocol version, currently `1`).
- A datagram must stay below 1400 bytes so it fits in one Ethernet frame. If a message
  would exceed that, the sender splits the list it carries over several messages.
- UDP gives no delivery guarantee. Receivers must tolerate lost, late and reordered
  messages. Order is decided by the `t` field, never by arrival order.
- JSON was chosen for readability (see `docs/decisions.md`, D2). Numbers are rounded by the
  sender: pixels to 0.01, metres to 0.0001, seconds to 0.000001.

## Ports

Defaults, set in `config/default.yaml` under `ports`.

| Port | Direction | Message |
| --- | --- | --- |
| 5005 | camera nodes (simulator) → tracker | `detections` |
| 5006 | tracker → robot (robot emulator) | `target` |
| 5007 | robot (robot emulator) → tracker | `robot_pose` |
| 5008 | simulator → metrics | `truth` (simulation only) |

The two browser windows use WebSockets. Those messages are internal to each program and are
described in `docs/architecture.md`, not here.

## Units and frames

Metres, seconds, radians. Positions are in the arena frame defined in `CLAUDE.md`.
Image coordinates are pixels with the origin at the top-left corner, `u` right, `v` down,
in the raw (distorted) image as the sensor delivers it. Time `t` is simulation time.

## `detections`

Sent by a camera node once per camera and frame, also when the camera sees nothing.

```json
{
  "type": "detections",
  "v": 1,
  "node_id": "node-03",
  "camera_id": "cam-06",
  "frame": 1042,
  "t": 17.366667,
  "bees": [[512.31, 300.87, 14.2], [901.05, 644.40, 9.8]],
  "markers": [[12, 88.10, 40.55]]
}
```

| Field | Type | Meaning |
| --- | --- | --- |
| `node_id` | string | The computer that sent the message. Two cameras share one node. |
| `camera_id` | string | The camera the points belong to. Unique in the system. |
| `frame` | int | Frame counter since start. Identical for all cameras for the same trigger pulse. |
| `t` | float, s | Time of the exposure (middle of the exposure). |
| `bees` | list of `[u, v, size]` | One entry per detected blob. `u`, `v` in px, `size` is the blob diameter in px. |
| `markers` | list of `[marker_id, u, v]` | Optional. Wall markers seen by this camera. Sent at a low rate (default once per second), omitted otherwise. |

A node has no knowledge of bee identity. The order of `bees` carries no meaning.

## `target`

Sent by the tracker to the robot while a bee is selected, at the tracker's output rate.

```json
{
  "type": "target",
  "v": 1,
  "t": 17.383333,
  "track_id": 4,
  "pos": [0.4120, 0.1503, 0.1711],
  "vel": [0.80, -0.12, 0.05],
  "acc": [0.0, 0.0, 0.0],
  "distance_m": 0.2140,
  "state": "flying",
  "confidence": 0.93,
  "others": [{"track_id": 7, "pos": [0.4301, 0.1100, 0.1650]}]
}
```

| Field | Type | Meaning |
| --- | --- | --- |
| `t` | float, s | The time for which `pos` is valid. The robot extrapolates with `vel` and `acc` to any later time it needs. |
| `track_id` | int | ID of the selected bee. |
| `pos`, `vel`, `acc` | `[x, y, z]` | State of the bee at time `t`, in m, m/s, m/s². |
| `distance_m` | float | Distance from the robot's lens reference point to `pos`, from the last known `robot_pose`. |
| `state` | `"sitting"` or `"flying"` | Derived from speed. Lets the robot switch dead zone and focus rate. |
| `confidence` | float 0..1 | Share of expected cameras that currently confirm the track. |
| `others` | list | Other tracked bees expected inside the robot's field of view at time `t`. May be empty. |

When no bee is selected the tracker sends nothing. The robot treats a silence longer than
0.5 s as "no target".

## `robot_pose`

Sent by the robot at its control rate.

```json
{"type": "robot_pose", "v": 1, "t": 17.380000, "rail_m": 0.3920, "tilt_rad": 0.1100, "focus_m": 0.2150}
```

| Field | Type | Meaning |
| --- | --- | --- |
| `rail_m` | float | Carriage position along the rail from its home position. |
| `tilt_rad` | float | Tilt of the camera, 0 = horizontal, positive = up. |
| `focus_m` | float | Distance the lens is currently focused at. |

## `truth` (simulation only)

Sent by the simulator once per frame. Read only by `experiments/metrics.py` and by the
simulator window. The tracker must never read it.

```json
{
  "type": "truth",
  "v": 1,
  "frame": 1042,
  "t": 17.366667,
  "bees": [{"id": 0, "pos": [0.41, 0.15, 0.17], "vel": [0.8, -0.1, 0.0], "state": "flying"}],
  "robot": {"rail_m": 0.392, "tilt_rad": 0.11, "focus_m": 0.215}
}
```

## Change log of the protocol

| Version | Change |
| --- | --- |
| 1 | First version. |
