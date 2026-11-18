# Paper figures

Screenshots of both windows for the paper, taken on 2026-11-18 with headless Chrome at
1280 × 800 px and a device pixel ratio of 1 (no retina copies). Each run used 5 bees, random
flight, seed 1, the robot values of `config/robot.yaml` and the tracker with
`--auto-select`. They show what the windows look like. No result in
`experiments/REPORT_DATA.md` comes from them.

| File | Shows |
| --- | --- |
| `simulator-window.png` | Simulator window, 16 cameras: the arena with 5 bees, view cones of the tracking cameras, the robot on its rail and the control panel |
| `control-window-selected-bee.png` | Control window: a selected flying bee (track 2, about 0.9 m/s) with trails and prediction lines, the selected-bee card, the cinema view and the first camera panels |
| `control-window-camera-panels.png` | Control window scrolled to the camera panels: the detections of every tracking camera, labelled with the track that used them |
| `cinema-view.png` | The cinema view relayed from the simulator window. The bee is centred but outside the 2 mm depth of field, so it is drawn as a soft disc; with the cautious robot limits this is the common case (see the robot tables in `experiments/REPORT_DATA.md`) |
| `simulator-camera-ids.png` | Simulator window with "Show camera IDs" on |
| `control-camera-ids.png` | Control window with "Show camera IDs" on |
| `cameras-8-simulator-window.png` | Simulator window with the 8-camera layout (`config/cameras-8.yaml`, one camera per corner) and camera IDs |
| `cameras-8-control-window.png` | Control window of the same 8-camera run, with camera IDs |

To take them again, run tracker and simulator as in `docs/setup.md` with a copy of the
config that has `bees.count: 5`, open both windows at 1280 × 800 and capture them.
