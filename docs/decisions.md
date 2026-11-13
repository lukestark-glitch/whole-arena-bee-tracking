# Decisions

Choices that are not obvious from the code, with the reason. Add an entry whenever a stage
makes such a choice. Do not rewrite old entries. If a decision changes, add a new entry
that replaces the old one and say so.

| ID | Date | Decision | Reason |
| --- | --- | --- | --- |
| D1 | 2026-11-13 | Simulator and tracker are separate programs that talk only over UDP. | The tracker must be testable against real hardware later without changes, and must not be able to read the ground truth. |
| D2 | 2026-11-13 | Messages are JSON, one object per UDP datagram. | Readable in a terminal and easy to debug. The data volume (about 1,000 small messages per second) does not need a binary format. |
| D3 | 2026-11-13 | Python with NumPy and SciPy for both programs. | The load is small (16 cameras × 20 bees × 60 Hz is about 19,000 points per second). Readability matters more than speed here. Slow parts can move to C++ after measuring. |
| D4 | 2026-11-13 | Both windows are browser pages with three.js, without a build step. | No installation, runs on a 2019 Intel MacBook, 3D and blur are built in. |
| D5 | 2026-11-13 | The tracking cameras are assumed to work in near infrared (850 nm). Colour detection is the documented alternative. | Independent of the film lighting. Detection is an exchangeable step: triangulation and tracking only receive points. |
| D6 | 2026-11-13 | Virtual cameras deliver points, not images. | The open question of the concept is triangulation, identity and hand-off, not image processing. Rendering 16 full-resolution streams would be slow and say little about reality. See Stage 8. |
| D7 | 2026-11-13 | Two cameras per camera node. | Halves the number of computers. The simulation models node delay and node failure accordingly. |
| D8 | 2026-11-13 | A 3D point needs at least three agreeing cameras, plus four further safeguards against ghost points. | Two cameras and two bees already produce two false intersections. Details in the concept paper, section 6.2. |
| D9 | 2026-11-13 | All timestamps are simulation time supplied by the simulator. | Pause, slow motion and repeatable runs. |
| D10 | 2026-11-13 | Nodes send raw pixel coordinates. The server applies the lens model and camera pose. | Calibration lives in one place and nodes stay interchangeable. |
| D11 | 2026-11-13 | Function reference is generated from docstrings with pdoc. `docs/code-map.md` is the hand-written overview. | A generated reference cannot fall behind the code. |
| D12 | 2026-11-13 | The robot is represented by `robot_emulator/`, separate from `simulator/` and `tracker/`. | Keeps clear which behaviour would belong to the existing robot's software. |
| D13 | 2026-11-13 | Equidistant fisheye model for the tracking cameras. | Lenses of 110° and more are not described well by the standard pinhole model with radial terms. |
| D14 | 2026-11-13 | `detections` and `truth` are split into several datagrams when they would reach 1400 bytes. Parts carry no part counter: every part repeats `frame`, `t` and the ids, and the receiver joins them. `target` and `robot_pose` are never split. | A part counter would add a field that is almost never needed (one datagram holds about 60 bees of one camera). A receiver cannot tell that a part is missing, which is the same as any other lost detection under UDP. |
| D15 | 2026-11-13 | Rounding for quantities the first protocol text did not cover: blob size 0.01 px; velocity, acceleration and `tilt_rad` 0.0001; `confidence` 0.01. | Matches the resolution of the neighbouring fields and keeps messages short. |
| D16 | 2026-11-13 | The field of view in the config is in radians (`fov_horizontal_rad: 1.919862` for 110°). | One unit for angles everywhere, as `CLAUDE.md` requires. The degree value is a comment next to it. |
| D17 | 2026-11-13 | Cameras sit exactly on the corners and long edges of the arena box, two per long edge at x = 0.30 m and 0.60 m. A node holds a corner camera and its neighbour on the same edge. | Simplest layout that matches Figure 3 of the concept paper. Offsets outside the acrylic are a question for Stage 7. |
| D18 | 2026-11-13 | The config holds only the four UDP ports. WebSocket ports are added in Stages 2 and 4. | Nothing is configured before something uses it. |
| D19 | 2026-11-13 | Flying bees follow a random velocity with damping (Ornstein-Uhlenbeck process) with a typical speed of 1 m/s and a direction memory of 0.5 s, capped at the maximum speed. They bounce off walls. | Gives smooth random turns with two readable parameters. A plain random walk of the velocity would drift to the 5 m/s cap and stay there. |
| D20 | 2026-11-13 | A flying bee lands only when it touches a wall (default chance 20 % per contact). Take-off, start and stop of walking happen at random with rates from the config. | A landing at a random moment in mid-air would teleport the bee to a wall, which no tracker should have to follow. |
| D21 | 2026-11-13 | Blob size = focal length × body length / distance, with a body length of 15 mm. | Small-angle size on an equidistant lens. Orientation of the bee is ignored. |
| D22 | 2026-11-13 | One node delay is drawn per node and frame (normal distribution, 8 ms mean, 2 ms standard deviation, never negative) and used for both cameras of the node. | Both images are processed by the same computer in the same frame time. |
| D23 | 2026-11-13 | All cameras must have the same frame rate; the config is rejected otherwise. The simulation step is one frame period. | One trigger source pulses every camera (concept paper, section 5.3). |
| D24 | 2026-11-13 | The UDP host `127.0.0.1` is a constant in `shared/protocol.py`, not a config value. | `docs/protocol.md` fixes the transport to localhost for the proof of concept. |
| D25 | 2026-11-13 | Stage 1 does not merge overlapping bees into one detection, although `docs/camera-node.md` lists that behaviour. | The Stage 1 plan does not ask for it. To be decided before Stage 3 baseline numbers are recorded. |
| D26 | 2026-11-13 | A camera does not report a bee that is closer than one body length, and a walking bee turns back at the edge of its wall. | Cameras sit on the arena corners and edges, where a bee at zero distance would get an infinite blob size. Turning back keeps the true velocity equal to the real motion. |
