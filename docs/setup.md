# Setup

How to install and run what exists. Keep this file correct: every stage adds its commands
here.

Status: Stages 0 to 3 are built (configuration, messages, simulator with its window,
tracker without its window, metrics). The sections marked "after Stage n"
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
pdoc shared simulator tracker experiments -o site/api
```

Each stage that creates a package adds it to this command.

Open `site/api/index.html` in a browser. The `site/` folder is not committed.

## Run the simulator

```bash
python -m simulator.main
```

It sends `detections` (UDP 5005) and `truth` (UDP 5008) in real time until Ctrl+C, and
serves the simulator window on port 8765.

| Option | Meaning | Default |
| --- | --- | --- |
| `--config PATH` | Configuration file | `config/default.yaml` |
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

It listens for `detections` (UDP 5005) and writes one line per frame to `logs/tracks.jsonl`.
Start it before the simulator.

| Option | Meaning | Default |
| --- | --- | --- |
| `--config PATH` | Configuration file | `config/default.yaml` |
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

## Open the windows

Simulator window: start the simulator, then open <http://127.0.0.1:8765/> in a browser.
The page loads three.js from a CDN, so the first load needs an internet connection.

- Drag to rotate, scroll to zoom.
- The panel changes the running simulation: number of bees, maximum speed, pixel noise,
  dropout, pause, time scale. "Restart" starts again at frame 0 with the given seed.

Control window: to be written by Stage 4.

## Run the experiments (after Stage 7)

To be written by Stage 7.

## Troubleshooting

- `OSError: [Errno 48] Address already in use`: another simulator or listener is still
  running. Stop it, or change the port under `ports` in the config.
- The metrics show far fewer frames than expected: the tracker was started after the
  simulator. Start metrics and tracker first.
