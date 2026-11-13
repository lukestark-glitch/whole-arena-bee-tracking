# Setup

How to install and run what exists. Keep this file correct: every stage adds its commands
here.

Status: Stages 0 to 4 are built (configuration, messages, simulator and tracker with
their windows, metrics). The sections marked "after Stage n"
are filled in by that stage.

## Requirements

- Python 3.11 or newer
- Git
- A current browser (Chrome, Firefox or Safari) for the two windows

## Get the code

```bash
git clone https://github.com/lukestark-glitch/whole-arena-bee-tracking.git
cd whole-arena-bee-tracking
```

## Install

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

## Run the tests

```bash
pytest
```

All tests must pass. They cover config loading and validation, every message type, the camera model, the bee
model, the simulator output, triangulation, the filter and the metrics. The whole suite
takes about one minute.

## Build the function reference

```bash
pdoc shared simulator robot_emulator tracker experiments -o site/api
```

Each stage that creates a package adds it to this command.

Open `site/api/index.html` in a browser. The `site/` folder is not committed.

## Run the simulator

```bash
python -m simulator.main
```

It sends `detections` (UDP 5005), `truth` (UDP 5008) and the robot's `robot_pose`
(UDP 5007) in real time until Ctrl+C, listens for `target` (UDP 5006) and serves the
simulator window on port 8765.

| Option | Meaning | Default |
| --- | --- | --- |
| `--config PATH` | Configuration file | `config/default.yaml` |
| `--robot PATH` | Robot file | `config/robot.yaml` |
| `--seed N` | Random seed | `seed` in the config |
| `--duration S` | Stop after `S` seconds of simulation time | run until Ctrl+C |
| `--time-scale X` | Speed at start against real time, `0.5` is half speed | `1.0` |
| `--fast` | As fast as possible, no pacing | off |

To see what it sends, start the listener in a second terminal first:

```bash
python -m experiments.listen_detections
```

It prints one line per second of simulation time. Expected with the default config:
16 cameras, 60 frames per camera.

## Run the tracker

```bash
python -m tracker.main
```

It listens for `detections` (UDP 5005) and `robot_pose` (UDP 5007), writes one line per
frame to `logs/tracks.jsonl`, sends `target` (UDP 5006) while a bee is selected and serves
the control window on port 8766. Start it before the simulator.

| Option | Meaning | Default |
| --- | --- | --- |
| `--config PATH` | Configuration file | `config/default.yaml` |
| `--robot PATH` | Robot file | `config/robot.yaml` |
| `--auto-select` | Select the oldest confirmed track whenever none is selected | off |
| `--log PATH` | Track log file | `logs/tracks.jsonl` |
| `--duration S` | Stop after `S` seconds of simulation time | run until Ctrl+C |

## Measure a run (baseline)

Three terminals, in this order. All three stop by themselves after 60 s of simulation time.

```bash
python -m experiments.metrics --tracks logs/tracks.jsonl --duration 60   # terminal 1
python -m tracker.main --duration 60                                     # terminal 2
python -m simulator.main --duration 60                                   # terminal 3
```

Terminal 1 prints the result: position error, ID swaps, ghost tracks and the share of
time each bee was tracked. The numbers of the default configuration are recorded in
`CHANGELOG.md`. The same seed gives the same numbers.

## Measure the robot

The same three terminals, with a bee selected automatically:

```bash
python -m experiments.metrics --tracks logs/tracks.jsonl --duration 60   # terminal 1
python -m tracker.main --duration 60 --auto-select                       # terminal 2
python -m simulator.main --duration 60                                   # terminal 3
```

Terminal 1 also prints `time in frame` and `time in focus`: the share of the frames with a
selected bee in which that bee was inside the cinema picture, and inside it and sharp.
Do not use `--fast` here: the robot only follows in real time.

## Try another robot

Robot values live in `config/robot.yaml`. Copy the file, change values, and pass the copy
to all three programs:

```bash
cp config/robot.yaml config/robot-slow.yaml      # then edit, for example rail max_speed_m_s
python -m experiments.metrics --tracks logs/tracks.jsonl --duration 60 --robot config/robot-slow.yaml
python -m tracker.main --duration 60 --auto-select --robot config/robot-slow.yaml
python -m simulator.main --duration 60 --robot config/robot-slow.yaml
```

## Open the windows

Simulator window: start the simulator, then open <http://127.0.0.1:8765/> in a browser.
The page loads three.js from a CDN, so the first load needs an internet connection.

- Drag to rotate, scroll to zoom.
- The panel changes the running simulation: number of bees, maximum speed, pixel noise,
  dropout, pause, time scale. "Restart" starts again at frame 0 with the given seed.

- "Camera nodes" stops and restarts single nodes.
- "Robot" shows rail, tilt and focus, and has sliders for the speed and acceleration limits
  of rail and tilt. They apply at once and last until the simulator stops.
- The robot model has a rail, a carriage and a camera head with lens, and a line to the focus
  plane. "Cinema view" is what its cinema camera sees.

Control window: start the tracker, then open <http://127.0.0.1:8766/>. Put both windows
side by side.

- It shows what the tracker believes: one coloured sphere per track with its ID, a trail
  of the last two seconds and a short line to the predicted position.
- Click a bee, or its row in the "Tracks" list, to select it. "Selected bee" then shows its
  speed, smoothed acceleration, state and number of cameras, live.
- The strip at the bottom shows every camera's image points with the ID each was used for.
- "Health" shows each node and turns red when a node has been silent for half a second.
- Selecting a bee sends it to the robot: in the simulator window the robot swings to it and
  follows it. The panel below the 3D view shows the robot's cinema picture on the left
  (while a simulator window is open) and the 16 camera panels on the right.
- The purple robot is drawn from the robot's own `robot_pose` messages. "Show robot" hides it.

If the simulator is restarted (seed and "Restart"), the tracker notices and starts with
fresh tracks.

## Run the experiments (after Stage 7)

To be written by Stage 7.

## Troubleshooting

- `OSError: [Errno 48] Address already in use`: another simulator or listener is still
  running. Stop it, or change the port under `ports` in the config.
- The metrics show far fewer frames than expected: the tracker was started after the
  simulator. Start metrics and tracker first.
