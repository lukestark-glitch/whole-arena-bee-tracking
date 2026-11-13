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

## What the emulator simplifies

| Real robot | Emulator |
| --- | --- |
| Stereo image processing | Blobs computed from true positions with noise in image position, size and distance |
| Motor drives and mechanics | Axes with speed and acceleration limits, no vibration or flex |
| Lens focus curve | Focus set directly as a distance |
| Cinema image | Rendered view with blur by distance from the focus plane |
