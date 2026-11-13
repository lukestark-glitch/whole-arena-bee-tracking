# Whole-arena bee tracking

Multi-camera 3D tracking of bumblebees for a camera robot: a simulation and the tracking
and control software, as a proof of concept.

## The idea

Jeremy Fielding's "Engineer vs Bee" video series shows a camera robot that follows a flying bumblebee with a macro lens. Its tracker rides
on the robot and follows the largest yellow blob it sees, so with several bees in the arena
the camera jumps from one bee to the next.

This project adds a second, fixed system. Several small cameras around the flight arena see
every bee at once. A tracking server computes each bee's 3D position and gives each bee a
stable ID. An operator selects one bee, and the robot is told where that bee is, where it
will be a moment later, and how far away it is.

The full concept is in the [concept paper](docs/concept-paper.md), also available as
[PDF](docs/concept-paper.pdf).

## Status

Idea stage. No hardware exists and no code has been written yet. The plan is to build the
tracking software and test it against a simulator that behaves like the hardware would.
Progress is tracked in [`docs/implementation-plan.md`](docs/implementation-plan.md).

## How it is organised

| Part | Role |
| --- | --- |
| `simulator/` | Stand-in for the real world: bees, virtual cameras, camera nodes |
| `robot_emulator/` | Stand-in for the camera robot and its software |
| `tracker/` | The software under test: triangulation, IDs, prediction, control window |
| `shared/` | Configuration, message formats and the camera model |
| `experiments/` | Metrics and repeatable test runs |

The simulator and the tracker are separate programs. They exchange the same UDP messages
that real camera nodes and a real robot would use.

## Documentation

| File | Content |
| --- | --- |
| [`docs/concept-paper.md`](docs/concept-paper.md) | The idea, the problems it targets, the system design ([PDF](docs/concept-paper.pdf)) |
| [`docs/implementation-plan.md`](docs/implementation-plan.md) | Stages with tasks and acceptance criteria |
| [`docs/architecture.md`](docs/architecture.md) | Components and how they communicate |
| [`docs/protocol.md`](docs/protocol.md) | Every message and its fields |
| [`docs/code-map.md`](docs/code-map.md) | Every module and its public functions |
| [`docs/setup.md`](docs/setup.md) | Installation and how to run |
| [`docs/camera-node.md`](docs/camera-node.md) | What the code on a real camera node must do |
| [`docs/robot-interface.md`](docs/robot-interface.md) | What the robot's software would have to do |
| [`docs/decisions.md`](docs/decisions.md) | Design decisions and their reasons |
| [`CHANGELOG.md`](CHANGELOG.md) | What changed and measured results |
| [`CLAUDE.md`](CLAUDE.md) | Working rules for Claude Code |

## Credits

Idea and design decisions: Luke Stark. Written with Claude (Anthropic), which gave
technical feedback, collected the material and wrote the documents. Inspired by Jeremy
Fielding's camera robot. This project is independent and not affiliated with him.

## License

MIT, see [`LICENSE`](LICENSE).
