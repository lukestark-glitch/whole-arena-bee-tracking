# Setup

How to install and run what exists. Keep this file correct: every stage adds its commands
here.

Status: no code yet. The sections marked "after Stage n" are filled in by that stage.

## Requirements

- Python 3.11 or newer
- Git
- A current browser (Chrome, Firefox or Safari) for the two windows

## Get the code

```bash
git clone https://github.com/lukestark-glitch/whole-arena-bee-tracking.git
cd whole-arena-bee-tracking
```

## Install (after Stage 0)

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

## Run the tests (after Stage 0)

```bash
pytest
```

## Build the function reference (after Stage 0)

```bash
pdoc shared simulator robot_emulator tracker experiments -o site/api
```

Open `site/api/index.html` in a browser. The `site/` folder is not committed.

## Run the simulator (after Stage 1)

To be written by Stage 1.

## Run the tracker (after Stage 3)

To be written by Stage 3.

## Open the windows (after Stages 2 and 4)

To be written by Stages 2 and 4.

## Run the experiments (after Stage 7)

To be written by Stage 7.

## Troubleshooting

Add problems and their fixes here as they come up.
