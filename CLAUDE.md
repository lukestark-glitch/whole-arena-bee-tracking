# CLAUDE.md

Rules for Claude Code in this repository. Read this file fully before any work.

## What this project is

A proof of concept for whole-arena 3D tracking of bumblebees. Fixed cameras around a flight
arena see every bee, a tracking server triangulates them and keeps one ID per bee, and a
camera robot is told where the selected bee is. No hardware exists. A simulator stands in for
bees, cameras, camera nodes and the robot. Background: `docs/concept-paper.md`.

Two programs talk over UDP with the messages a real system would use:

- `simulator/` plus `robot_emulator/`: the stand-in for the real world.
- `tracker/`: the software under test (tracking server and control UI).

## Where the truth is written down

| Question | File |
| --- | --- |
| What to build next, and when a stage is done | `docs/implementation-plan.md` |
| Components and how they communicate | `docs/architecture.md` |
| Exact message fields (binding) | `docs/protocol.md` |
| Which module does what | `docs/code-map.md` |
| Why a choice was made | `docs/decisions.md` |
| How to install and run | `docs/setup.md` |
| What real camera-node code must do | `docs/camera-node.md` |
| What the real robot software must do | `docs/robot-interface.md` |

If code and one of these files disagree, stop and fix the disagreement first.

## How to work

1. Work on exactly one stage of `docs/implementation-plan.md` at a time. Do not start the
   next stage unless the user asks for it.
2. Before coding, restate the stage's acceptance criteria and list the files you will touch.
3. Create a branch `stage-<n>-<short-name>`. Make small commits with clear messages.
4. When the acceptance criteria are met, complete the documentation checklist below, set the
   stage status in the plan to `done`, and open a pull request to `main`.
5. When the pull request of a stage is open, send the user a push notification to their
   phone (PushNotification tool): stage number, that it is done, and the pull request link.
6. If the plan is unclear or seems wrong, ask the user. Do not guess and do not silently
   change the plan.

## Write only the code that is needed

- Implement what the current stage's acceptance criteria require. Nothing speculative, no
  options "for later", no unused parameters.
- Prefer plain functions and `@dataclass` types. Add a class only when it holds state.
  No inheritance hierarchies, no plugin systems, no frameworks.
- One obvious way per task. Do not add a second code path that does the same thing.
- Allowed runtime dependencies: `numpy`, `scipy`, `pyyaml`, `websockets`. Development:
  `pytest`, `pdoc`. Browser: `three.js` from a pinned CDN URL, no build step, no npm.
  Any other dependency needs an entry in `docs/decisions.md` and the user's approval first.
- Delete code that is no longer used. Do not comment it out.
- A new idea from the user goes into the plan or `docs/decisions.md` first, then into code.

## Make the code easy to follow

- Everything is in English: code, comments, docs, commit messages.
- Python 3.11 or newer, type hints on every function signature.
- Every module starts with a docstring that says what the module is for and how it fits in.
- Every function and class has a docstring in Google style: one summary line, then `Args`,
  `Returns`, and `Raises` when relevant. State units and coordinate frame for every quantity.
- Put the unit in the name where a mix-up is possible: `distance_m`, `time_s`, `tilt_rad`,
  `u_px`, `rate_hz`.
- Comments explain why, not what. Explain every formula in one sentence or name its source.
- Functions stay short. If a function needs section comments, split it.
- No magic numbers. Constants come from the config file or a named constant with a comment.

## Conventions that must never be broken

- **Ground truth stays in the simulator.** `tracker/` must not import from `simulator/` or
  `robot_emulator/`, and must never receive `truth` messages. Only `experiments/metrics.py`
  reads the truth.
- **Simulation time.** All timestamps are simulation time in seconds, supplied by the
  simulator. `tracker/` never calls the wall clock for anything that affects results.
- **Units.** Metres, seconds, radians. Pixels for image coordinates, origin top-left,
  `u` to the right, `v` down.
- **Arena frame.** Right-handed. Origin at the bottom corner of the arena nearest the robot's
  rail start. `x` along the long side (parallel to the rail), `y` away from the robot,
  `z` up.
- **Configuration.** Arena, cameras, nodes, camera models, noise and ports live in
  `config/*.yaml`. Code reads them through `shared/config.py`. Nothing configurable is
  hard-coded.
- **Protocol.** Messages are built and parsed only in `shared/protocol.py`. To change a
  message, change `docs/protocol.md` first, then the code, in the same pull request.
- **Determinism.** All randomness comes from one seeded `numpy.random.Generator` passed in
  explicitly. The same seed and config give the same run.

## Tests

- Use `pytest`. Test the maths and the protocol: projection, ray construction, triangulation,
  assignment, the filter, message round trips, config loading.
- Each acceptance criterion that can be checked by code gets a test.
- Do not write tests for the browser UI. Describe a manual check in the pull request instead.
- All tests pass before a stage is called done.

## Documentation checklist (part of "done" for every stage)

A stage is not done until all of these are true:

- [ ] Every new or changed function, class and module has an up-to-date docstring.
- [ ] `docs/code-map.md` lists every module with its purpose and public functions.
- [ ] `docs/architecture.md` matches the components and the communication between them.
- [ ] `docs/protocol.md` matches the messages actually sent.
- [ ] `docs/setup.md` contains every command needed to install and run what exists now.
- [ ] `docs/decisions.md` has an entry for every non-obvious choice made in this stage.
- [ ] `CHANGELOG.md` has an entry for the stage.
- [ ] The stage status in `docs/implementation-plan.md` is updated.
- [ ] `pdoc` builds the API reference without warnings (command in `docs/setup.md`).

Documentation may be brought up to date at the end of a long session in one go, but never
later than the pull request for the stage.

## What not to build

- No real camera-node code and no real robot code. `docs/camera-node.md` and
  `docs/robot-interface.md` are specifications. Update them when the design changes.
- No image rendering for the tracking cameras before Stage 8. Virtual cameras deliver points.
- No user accounts, databases, cloud services or packaging.

## Commands

The current commands are in `docs/setup.md`. Keep that file correct.
