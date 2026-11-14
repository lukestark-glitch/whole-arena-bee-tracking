# Robot interface specification

What the camera robot's software would have to do to work with the tracking server.

Status: proposal. The real robot belongs to Jeremy Fielding's project and its software is
not part of this repository. Nothing here has been agreed with him. In the simulation,
`robot_emulator/` plays this role.

## What the robot already does (from the video series)

- Moves a cinema camera with a macro lens on a linear rail, tilts it, and turns the focus
  ring with a motor.
- Carries a calibrated stereo camera pair that measures the distance to the bee.
- Finds the bee as the largest yellow blob, searching only near the last position once a
  bee is found.

## What changes

The robot stops deciding which bee to follow. It receives that from the tracking server
and uses its own vision only to refine centring and focus.

## Messages

Defined in `docs/protocol.md`.

| Message | Direction | Purpose |
| --- | --- | --- |
| `target` | server → robot | Where the selected bee is, how it moves, how far it is, and which other bees are about to cross the view |
| `robot_pose` | robot → server | Rail position, tilt angle and focus distance, so the server can compute distances and project into the robot's image |

## Required behaviour

1. **Extrapolate.** `target` is valid for its time `t`. Use `vel` and `acc` to compute the
   position for the moment the stereo camera exposes and for the moment the motors will act.
2. **Search window.** Project the predicted position into the stereo image and search only
   a small window around it.
3. **Selection rule.** Inside the window, accept a blob only if all three hold:
   - it is the blob nearest to the predicted point,
   - its measured distance matches `distance_m` within a tolerance,
   - it is not explained by one of the bees in `others`.
4. **Fallback.** If no blob qualifies, follow the prediction and keep the last focus
   distance trend. Never switch to a blob that failed the rule.
5. **Feed-forward.** Use `vel` to move ahead of the bee instead of chasing the last image.
6. **Mode from state.** Use `state` to switch behaviour: a dead zone and a slow focus rate
   while the bee is `sitting`, no dead zone and a fast focus rate while it is `flying`.
7. **Focus preset.** On a change of `track_id`, drive focus to `distance_m` while the
   camera swings over, then hand focus back to the stereo measurement.
8. **Report pose.** Send `robot_pose` at the control rate with true axis positions. Flex
   that the encoders do not see makes every projection wrong.
9. **No target.** If no `target` arrives for 0.5 s, hold position.

## Calibration data the robot must provide

- Rail and tilt readings and the focus motor position while the operator centres and
  focuses on the calibration LED at 10 to 20 places.
- From these, the server fits the robot's pose in the arena frame and a mapping from arena
  distance to focus position. The mapping absorbs the lens reference point, so nothing has
  to be measured by hand.

## Timing

- A shared trigger for the tracking cameras and the stereo camera is preferred. Both then
  expose at the same instants and no interpolation is needed.
- Without a shared trigger, both sides need synchronised clocks, and the robot extrapolates
  `target` to its own exposure time.
- The cinema camera's frame rate is independent of all of this.

## Robot parameters: known and missing

The emulator reads all robot values from `config/robot.yaml`. Values marked `assumed` there
should be replaced as soon as real ones are known. Another file can be selected with
`--robot PATH` (simulator, tracker and metrics must get the same one).

Known from the videos:

| Parameter | Value |
| --- | --- |
| First rail base | Extruded aluminium, about 4 ft (1.2 m) |
| Rail drive | Timing belt with glass-fibre core, 50:1 gearbox, motor with encoder |
| Later rail | Commercial linear rail, smaller and lighter |
| Axes | Rail, tilt, focus motor |
| Rail speed | 3 m/s (stated by Luke, rated speed of the rail actuator; not from the videos) |
| Tilt and focus drives | NEMA 23 stepper motors with gearboxes (stated by Luke) |
| Cinema lens | 180 mm macro at 1.5× magnification |
| Stereo camera | 2.2 mm lenses, 105° rectified field of view, 50 mm baseline, bee at about 150 mm |
| Tracking rate | about 32 Hz full image, about 60 Hz in the search window |
| Focus update rate | 20 Hz |
| Dead zone | 2 to 3 mm |
| Tilt flex | almost 2°, about 7 mm at 200 mm |

Missing and to be asked for:

- Length of travel of the current rail
- Maximum acceleration of the rail (the speed of 3 m/s is given)
- Gear ratio, maximum speed and acceleration of the tilt axis, and its range
- Maximum speed of the focus motor and the focus range
- Moving mass of the carriage with camera
- Delay from stereo exposure to motor command
- Position of the rail and the lens reference point relative to the arena

For scale: a bee reversing from +5 m/s to −5 m/s in 50 ms needs about 200 m/s². The rail
will not reach that, which is why prediction matters and why the simulation should test a
range of limits.

## What the emulator simplifies

| Real robot | Emulator |
| --- | --- |
| Stereo image processing | Blobs computed from true positions with noise in image position, size and distance |
| Motor drives and mechanics | Axes with speed and acceleration limits, no vibration or flex |
| Lens focus curve | Focus set directly as a distance |
| Cinema image | Rendered view with blur by distance from the focus plane |
| Delay from stereo exposure to motor command | None: a blob is used in the control step of its exposure |
| Mode from `state` (dead zone, focus rate) | Not emulated: the same control for sitting and flying bees |
| Home position | Every axis starts in the middle of its travel |

How the emulator reads two of the rules:

- *Distance must match.* The expected distance is recomputed from the extrapolated position
  and the robot's present pose. It is the same quantity as `distance_m`, made current.
- *Not explained by another bee.* A blob is rejected when a bee in `others` is expected
  nearer to it in the image than the target.
- *Fallback.* The last accepted stereo measurement is kept as an offset to the prediction
  (centring and focus), so the robot follows the prediction and keeps the last trend.
