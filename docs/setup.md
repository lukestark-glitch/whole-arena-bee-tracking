# Setup

How to install and run what exists. Keep this file correct: every stage adds its commands
here.

Status: Stages 0 and 1 are built (configuration, messages, simulator without a window). The sections marked "after Stage n"
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
model and the simulator output.

## Build the function reference

```bash
pdoc shared simulator experiments -o site/api
```

Each stage that creates a package adds it to this command.

Open `site/api/index.html` in a browser. The `site/` folder is not committed.

## Run the simulator

```bash
python -m simulator.main
```

It sends `detections` (UDP 5005) and `truth` (UDP 5008) in real time until Ctrl+C.

| Option | Meaning | Default |
| --- | --- | --- |
| `--config PATH` | Configuration file | `config/default.yaml` |
| `--seed N` | Random seed | `seed` in the config |
| `--duration S` | Stop after `S` seconds of simulation time | run until Ctrl+C |
| `--time-scale X` | Speed against real time, `0.5` is half speed | `1.0` |
| `--fast` | As fast as possible, no pacing | off |

To see what it sends, start the listener in a second terminal first:

```bash
python -m experiments.listen_detections
```

It prints one line per second of simulation time. Expected with the default config:
16 cameras, 60 frames per camera.

## Run the tracker (after Stage 3)

To be written by Stage 3.

## Open the windows (after Stages 2 and 4)

To be written by Stages 2 and 4.

## Run the experiments (after Stage 7)

To be written by Stage 7.

## Troubleshooting

Add problems and their fixes here as they come up.
