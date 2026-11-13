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
