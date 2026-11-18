# Stage 7 report data

Tables and findings of the Stage 7 experiments, written so that a paper can be built from
them. Every number comes from `experiments/metrics.py` through `experiments/batch.py` and is
in `experiments/results/summary.csv` (mean over the seeds) and `runs.csv` (one row per run).
Screenshots in `docs/paper-figures/` only show the windows; no number here comes from them.

**How the numbers were made.** `python -m experiments.batch` (code of commit c562e5b plus the
fixes in this pull request), 37 cases, seeds 1 to 5, 20 simulated seconds per run, the first
2 s left out (track start-up), so 18 s are evaluated. Each run puts simulator, tracker and
robot emulator in one process, so it repeats exactly. Datagrams are delivered in send-time
order (D95). The tracker selects a bee by itself (`auto_select`: the oldest confirmed track),
and the robot follows it. Base case unless a table says otherwise: 16 cameras on 8 nodes,
10 bees, random flight, pixel noise 0.5 px, dropout 2 %, node delay 8 ± 2 ms, true camera
poses, the cautious robot limits of `config/robot.yaml`. The full batch took 23 minutes on an
8-core laptop.

**Columns.** Mean and p95 error: distance of each matched track from its bee. ID swaps: times
a bee was matched to another track id than before, over 18 s. Ghost tracks: confirmed tracks
that matched no bee in more than half their frames (worst seed). Ghost share: share of all
confirmed track-frames that matched no bee. Tracked: share of each bee's frames in which a
track matched it (mean over bees, and the worst bee). Prediction: constant-velocity
prediction from the logged state, 1 frame (17 ms) and 100 ms ahead, against where the bee
really was (D96). In frame / in focus: share of the frames with a selected bee in which that
bee was inside the cinema picture, and inside it and within the 2 mm depth of field; sd is
the standard deviation over the seeds. Offset and focus error: mean distance of the followed
bee from the cinema camera's optical axis and from its focus plane.

## 1. Can 8 cameras do the job? (the main result)

**Criterion, written down before any run (D93).** At 5 bees, for both flight models,
averaged over the seeds: at least 95 % of the volume seen by 3 or more cameras; mean error at
most 1 mm and p95 at most 2 mm; no ghost track in any seed; tracked share at least 95 %; ID
swaps at most 1.5 times those of 16 cameras; robot time in frame and in focus at most 5
points below 16 cameras.

**Layout.** `config/cameras-8.yaml`: one camera on each corner of the box (8 of the 16
default cameras), all aimed at the centre, 4 nodes with the two cameras of one vertical edge
each (D94). Same arena, same robot.

**Coverage** (`experiments/coverage.py`, grid of 1.5 cm cells, 24 000 points; expected error
from 0.5 px noise and the ray geometry):

| Cameras | Nodes | ≥2 | ≥3 | ≥4 | ≥6 | Expected error median (mm) | p95 (mm) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 16 | 8 | 100.0 % | 100.0 % | 100.0 % | 100.0 % | 0.15 | 0.25 |
| 12 | 6 | 100.0 % | 100.0 % | 100.0 % | 97.1 % | 0.17 | 0.34 |
| 10 | 5 | 100.0 % | 100.0 % | 100.0 % | 94.9 % | 0.18 | 0.36 |
| 8 | 4 | 100.0 % | 100.0 % | 100.0 % | 92.4 % | 0.25 | 0.39 |

Every layout sees the whole box with at least 4 cameras, so the ghost-point rule (3 cameras)
always has a margin. Fewer cameras mainly raise the expected error and shrink the part seen
by 6 or more.

**Verdict against the criterion** (`experiments/results/criterion.csv`):

| Cameras | Flight | ≥3 cams 95 % | mean ≤1 mm | p95 ≤2 mm | no ghost | tracked ≥95 % | swaps ≤1.5× | in frame −5 | in focus −5 | Verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 16 | random | yes | yes | yes | yes | yes | yes | yes | yes | good enough |
| 16 | trips | yes | yes | yes | yes | yes | yes | yes | yes | good enough |
| 12 | random | yes | yes | yes | **no** | yes | **no** | yes | yes | not good enough: no_ghost_track, id_swaps_le_1_5x |
| 12 | trips | yes | yes | **no** | yes | yes | **no** | yes | **no** | not good enough: p95_error_le_2mm, id_swaps_le_1_5x, in_focus_within_5 |
| 10 | random | yes | yes | yes | yes | yes | **no** | yes | yes | not good enough: id_swaps_le_1_5x |
| 10 | trips | yes | yes | yes | yes | yes | yes | yes | **no** | not good enough: in_focus_within_5 |
| 8 | random | yes | yes | yes | yes | yes | **no** | yes | yes | not good enough: id_swaps_le_1_5x |
| 8 | trips | yes | yes | **no** | yes | yes | yes | yes | **no** | not good enough: p95_error_le_2mm, in_focus_within_5 |

**In plain words.**

- **8 cameras locate bees well enough but are not "good enough" by the criterion.** With
  random flight they fail one point: ID swaps (2.4 against 0.4 for 16 cameras in 18 s, 1.6
  against 0.27 swaps per bee and minute). With trips they fail two: the p95 error is
  2.33 mm (limit 2 mm) and the robot is in focus 68 % of the time against 79 % with 16
  cameras.
- **Where 8 cameras are weak.** Errors and swaps grow with trips, where bees gather at the
  end walls by the feeder and the entrance, right next to the corner cameras. A camera sees
  a bee that close at a steep angle and from few directions, and bees close together near a
  wall are confused more often. The data do not locate the errors in space. "Near the end
  walls" follows from the trips layout and the coverage maps, not from a measurement.
- **A node failure hurts 8 cameras on 4 nodes much more than 16 on 8.** With one of the 4
  nodes stopped, a whole vertical edge goes dark. The worst bee is then tracked only 84 %
  (random) or 63 % (trips) of the time, and the p95 error with trips is 3.4 mm. With 16
  cameras, two of 8 nodes stopped still leave the worst bee at 98 %.
- **10 and 12 cameras do not pass either.** 10 cameras fail only ID swaps (random) or only
  focus (trips). 12 cameras had one ghost track in one random seed and a p95 of 2.01 mm with
  trips. 12 is not clearly better than 10 or 8. With 5 seeds of 18 s, a difference of 1 to 2
  ID swaps or 0.1 to 0.3 mm of p95 is within the spread between seeds (sd of ID swaps 0.7 to
  3.2, sd of p95 0.12 to 0.26 mm; see `summary.csv`).
- **The smallest number that passes is 16.** No layout below 16 passed all points in these
  runs. The criterion is strict where 16 cameras are nearly perfect: 0.4 swaps in 18 s leave
  little room for 1.5 times.
- **Robot numbers do not separate the layouts.** With random flight the robot was in frame
  75 % of the time with 8 cameras and 62 % with 16. The spread between seeds is 13 to 20
  points, because each seed follows a different bee.

**Hardware for 8 cameras.** Counts from the layouts. Prices are rough list prices, assumed
and to be checked before buying: camera with a wide lens about 80 USD, Raspberry Pi 5 with
PoE board, card and case about 100 USD, a PoE switch 60 to 150 USD. The trigger generator is
the same in all three.

| Layout | Cameras | Nodes | Switch ports | Rough cost | One node fails |
| --- | --- | --- | --- | --- | --- |
| 16 cameras, 8 nodes (default) | 16 | 8 Pi 5 | 8 | about 2 200 USD | 2 of 16 cameras lost; little effect (table in section 3) |
| 8 cameras, 4 nodes (two per Pi 5) | 8 | 4 Pi 5 | 4 | about 1 100 USD | 2 of 8 cameras lost, a whole corner edge; worst bee 63 to 84 % tracked |
| 8 cameras, 8 nodes (one per Pi) | 8 | 8 Pi | 8 | about 1 600 USD | 1 of 8 cameras lost; not simulated (Open questions) |

An 8-camera arena halves the cost of the first build. Its p95 error is 0.3 mm higher with
random flight and 1.3 mm higher with trips. It swaps IDs more often and is fragile when a
node fails.
Its cameras are 8 of the 16 default positions, so it can be extended to 16 later.

**Comparison at 5 bees, all metrics:**

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 16 cameras, random | 0.41 | 1.05 | 0.40 | 0.00 | 0.0 | 100.0 | 100.0 |
| 16 cameras, trips | 0.41 | 1.00 | 2.40 | 0.00 | 0.1 | 99.4 | 98.7 |
| 12 cameras, random | 0.49 | 1.27 | 1.60 | 1.00 | 0.3 | 99.9 | 99.9 |
| 12 cameras, trips | 0.68 | 2.01 | 4.60 | 0.00 | 0.2 | 99.0 | 96.9 |
| 10 cameras, random | 0.51 | 1.31 | 2.20 | 0.00 | 0.4 | 99.9 | 99.7 |
| 10 cameras, trips | 0.67 | 1.93 | 1.60 | 0.00 | 0.1 | 99.0 | 97.5 |
| 8 cameras, random | 0.52 | 1.32 | 2.40 | 0.00 | 0.2 | 99.9 | 99.7 |
| 8 cameras, trips | 0.79 | 2.33 | 2.80 | 0.00 | 0.2 | 98.8 | 96.6 |
| 8 cameras, 1 of 4 nodes stopped, random | 0.62 | 1.56 | 3.00 | 0.00 | 0.8 | 96.6 | 84.4 |
| 8 cameras, 1 of 4 nodes stopped, trips | 1.03 | 3.40 | 3.60 | 0.00 | 0.9 | 88.4 | 63.4 |

| Case | Prediction 17 ms mean (mm) | p95 (mm) | Prediction 100 ms mean (mm) | p95 (mm) |
| --- | --- | --- | --- | --- |
| 16 cameras, random | 3.48 | 10.50 | 32.81 | 112.42 |
| 16 cameras, trips | 1.05 | 2.40 | 5.84 | 12.52 |
| 12 cameras, random | 3.73 | 10.48 | 34.43 | 115.71 |
| 12 cameras, trips | 1.61 | 4.77 | 8.22 | 22.12 |
| 10 cameras, random | 3.92 | 11.06 | 36.42 | 118.33 |
| 10 cameras, trips | 1.60 | 4.57 | 8.16 | 21.47 |
| 8 cameras, random | 3.48 | 10.23 | 31.32 | 109.06 |
| 8 cameras, trips | 1.83 | 5.21 | 8.81 | 23.14 |
| 8 cameras, 1 of 4 nodes stopped, random | 3.69 | 10.70 | 32.41 | 110.32 |
| 8 cameras, 1 of 4 nodes stopped, trips | 2.34 | 7.41 | 10.88 | 32.90 |

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| 16 cameras, random | 61.8 | 13.0 | 33.4 | 10.4 | 22.42 | 19.27 |
| 16 cameras, trips | 98.7 | 1.6 | 79.2 | 8.2 | 3.26 | 1.82 |
| 12 cameras, random | 64.4 | 16.9 | 43.3 | 25.1 | 23.31 | 16.29 |
| 12 cameras, trips | 97.2 | 1.7 | 67.2 | 6.2 | 2.48 | 1.37 |
| 10 cameras, random | 57.4 | 10.0 | 35.6 | 8.0 | 21.07 | 19.81 |
| 10 cameras, trips | 97.0 | 2.2 | 73.9 | 7.5 | 3.88 | 0.90 |
| 8 cameras, random | 75.4 | 20.1 | 54.0 | 18.9 | 14.42 | 11.02 |
| 8 cameras, trips | 97.8 | 2.3 | 68.0 | 7.0 | 4.62 | 3.10 |
| 8 cameras, 1 of 4 nodes stopped, random | 66.1 | 25.9 | 45.0 | 24.2 | 25.50 | 21.43 |
| 8 cameras, 1 of 4 nodes stopped, trips | 97.6 | 1.3 | 60.5 | 6.2 | 2.44 | 1.11 |

## 2. Number of bees and flight model

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 bees, random | 0.44 | 1.06 | 0.20 | 0.00 | 0.3 | 100.0 | 100.0 |
| 3 bees, random | 0.43 | 1.11 | 0.20 | 0.00 | 0.1 | 100.0 | 99.9 |
| 5 bees, random | 0.41 | 1.05 | 0.40 | 0.00 | 0.0 | 100.0 | 100.0 |
| 10 bees, random | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| 20 bees, random | 0.48 | 1.18 | 17.80 | 0.00 | 0.2 | 99.8 | 98.8 |
| 1 bees, trips | 0.41 | 0.95 | 0.00 | 0.00 | 0.0 | 100.0 | 100.0 |
| 3 bees, trips | 0.41 | 1.00 | 0.40 | 0.00 | 0.1 | 99.7 | 99.1 |
| 5 bees, trips | 0.41 | 1.00 | 2.40 | 0.00 | 0.1 | 99.4 | 98.7 |
| 10 bees, trips | 0.54 | 1.27 | 16.40 | 1.00 | 0.6 | 97.5 | 92.6 |
| 20 bees, trips | 0.67 | 1.81 | 63.60 | 1.00 | 1.0 | 95.8 | 87.0 |

| Case | Prediction 17 ms mean (mm) | p95 (mm) | Prediction 100 ms mean (mm) | p95 (mm) |
| --- | --- | --- | --- | --- |
| 1 bees, random | 3.85 | 11.48 | 36.72 | 120.70 |
| 3 bees, random | 3.94 | 11.28 | 37.42 | 123.28 |
| 5 bees, random | 3.48 | 10.50 | 32.81 | 112.42 |
| 10 bees, random | 3.46 | 10.27 | 32.42 | 113.32 |
| 20 bees, random | 3.70 | 10.93 | 34.49 | 118.72 |
| 1 bees, trips | 1.03 | 2.28 | 5.24 | 11.35 |
| 3 bees, trips | 1.04 | 2.41 | 5.85 | 12.74 |
| 5 bees, trips | 1.05 | 2.40 | 5.84 | 12.52 |
| 10 bees, trips | 1.20 | 2.97 | 6.16 | 13.22 |
| 20 bees, trips | 1.39 | 3.79 | 6.66 | 14.91 |

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| 1 bees, random | 53.8 | 9.6 | 33.6 | 15.3 | 27.02 | 23.14 |
| 3 bees, random | 61.4 | 14.3 | 44.5 | 16.7 | 18.57 | 18.26 |
| 5 bees, random | 61.8 | 13.0 | 33.4 | 10.4 | 22.42 | 19.27 |
| 10 bees, random | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| 20 bees, random | 58.8 | 13.3 | 37.4 | 14.9 | 21.68 | 17.10 |
| 1 bees, trips | 99.5 | 0.3 | 81.9 | 10.0 | 0.80 | 0.63 |
| 3 bees, trips | 98.7 | 1.1 | 76.8 | 9.2 | 1.10 | 0.77 |
| 5 bees, trips | 98.7 | 1.6 | 79.2 | 8.2 | 3.26 | 1.82 |
| 10 bees, trips | 97.9 | 2.2 | 81.7 | 4.6 | 1.75 | 1.72 |
| 20 bees, trips | 96.5 | 2.1 | 74.6 | 1.9 | 3.80 | 2.47 |

- Position error barely depends on the number of bees with random flight (p95 1.05 to
  1.18 mm from 1 to 20 bees). With trips it grows from 0.95 to 1.81 mm, because more bees
  gather at feeder and entrance.
- ID swaps grow faster than the number of bees. Random flight: 0.27 swaps per bee and minute
  at 5 bees, 1.8 at 10 and 3.0 at 20. Trips: 1.6, 5.5 and 10.6. Trips at 10 and 20 bees also
  gave one ghost track in one seed, and the worst bee was tracked only 87 % of the time at
  20 bees. Crowding at the two ends is the weak point of the tracker, not the cameras.
- Trips are far easier to predict: 100 ms ahead the error is about 6 mm against about 33 mm
  with random flight. The random model changes direction at random every 0.5 s, so a
  constant-velocity prediction cannot do better.
- The robot looks far better with trips (in frame 97 to 99 %, in focus 75 to 82 %). This is
  mostly because the followed bee sits at the entrance or the feeder for several seconds
  between flights. It does not mean the robot follows flight better.

## 3. Measurement noise, dropouts and failed nodes

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| noise low | 0.36 | 0.98 | 4.20 | 0.00 | 0.2 | 99.9 | 99.0 |
| noise nominal | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| noise high | 0.60 | 1.41 | 4.60 | 0.00 | 0.1 | 99.9 | 99.2 |

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 of 8 nodes stopped | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| 1 of 8 nodes stopped | 0.44 | 1.11 | 5.20 | 0.00 | 0.1 | 99.9 | 99.1 |
| 2 of 8 nodes stopped | 0.47 | 1.17 | 4.80 | 0.00 | 0.1 | 99.7 | 97.8 |

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 of 8 nodes stopped | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| 1 of 8 nodes stopped | 43.2 | 13.7 | 27.0 | 13.5 | 39.07 | 29.83 |
| 2 of 8 nodes stopped | 32.4 | 10.9 | 15.5 | 11.5 | 48.71 | 37.65 |

- Noise of 0.25, 0.5 and 1.0 px (with 0, 2 and 10 % dropout) moves the mean error from 0.36
  to 0.60 mm and the p95 from 0.98 to 1.41 mm. Tracking stays complete, with no ghost tracks
  and at least 99 % tracked.
- With 16 cameras, one or two of 8 nodes stopped (2 or 4 cameras lost) changes tracking very
  little. The robot loses more: in frame 50, 43 and 32 %. This may be chance, because each
  seed follows a different bee. A cause in the tracking is not visible.

## 4. Network delay and jitter

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| delay 5 ± 1 ms | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| delay 8 ± 2 ms | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| delay 15 ± 5 ms | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| delay 30 ± 10 ms | 0.46 | 1.12 | 4.60 | 0.00 | 0.2 | 99.9 | 99.1 |

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| delay 5 ± 1 ms | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| delay 8 ± 2 ms | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| delay 15 ± 5 ms | 44.5 | 12.2 | 24.0 | 13.7 | 34.25 | 26.43 |
| delay 30 ± 10 ms | 42.7 | 14.0 | 25.3 | 13.9 | 40.33 | 28.76 |

- Delays of 5, 8 and 15 ms give practically the same tracking, because a frame is complete
  before the next one starts either way. At 30 ± 10 ms the numbers change slightly, probably
  because some messages come later than the frame timeout of two frame periods (33 ms).
- The robot loses time in frame as the delay grows (50 % at 8 ms, 45 % at 15 ms, 43 % at
  30 ms), because the target it receives is older. The 30 ms case is still usable.

## 5. Calibration error (Stage 6)

All cases except "true" use `--miscalibrated`: every true camera is 1 cm and 2° away from
the config, and the robot's rail start about 1 cm.

| Case | Mean error (mm) | p95 error (mm) | ID swaps (18 s) | Ghost tracks (worst seed) | Ghost share (%) | Tracked (%) | Lowest bee (%) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| calibration: true | 0.43 | 1.06 | 5.40 | 0.00 | 0.2 | 99.9 | 99.1 |
| calibration: none | 11.47 | 17.86 | 0.00 | 2.00 | 67.4 | 0.0 | 0.0 |
| calibration: markers | 0.47 | 1.20 | 6.60 | 0.00 | 0.2 | 99.8 | 99.1 |
| calibration: wand | 0.46 | 1.15 | 6.60 | 0.00 | 0.2 | 99.8 | 99.1 |

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| calibration: true | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| calibration: none | 0.0 | 0.0 | 0.0 | 0.0 | 383.76 | 155.31 |
| calibration: markers | 50.8 | 18.5 | 25.9 | 21.2 | 23.88 | 21.87 |
| calibration: wand | 50.7 | 18.1 | 25.8 | 20.8 | 23.93 | 21.85 |

- **Without any correction the tracker fails completely.** 2° of camera error put the rays
  centimetres apart, so almost no point is confirmed (0 % tracked) and two thirds of the few
  tracks are ghosts.
- **The wall-marker check alone repairs the cameras.** Each camera's pose is re-estimated
  at its first marker report (once per second), and tracking returns to 0.47 mm mean error. The wand calibration
  gives practically the same result (0.46 mm). Both mean errors are within 0.05 mm of the
  run with true cameras.
- The robot reaches the same numbers in all three corrected cases, even without the wand's
  robot calibration. Probably its own stereo camera makes up for the shifted rail start.

## 6. Robot

| Case | In frame (%) | sd | In focus (%) | sd | Offset from axis (mm) | Focus error (mm) |
| --- | --- | --- | --- | --- | --- | --- |
| robot limits cautious (file) | 49.7 | 10.0 | 25.1 | 13.9 | 27.67 | 22.87 |
| robot limits moderate (D74) | 64.1 | 8.6 | 27.5 | 13.1 | 15.82 | 14.90 |
| robot limits fast (hypothetical) | 83.6 | 4.7 | 31.9 | 12.5 | 6.70 | 7.90 |

- With the cautious limits of `config/robot.yaml` (rail 3 m/s and 10 m/s², tilt 8 rad/s and
  50 rad/s², focus 0.5 m/s and 10 m/s²) the robot keeps the followed bee in frame about half
  the time and in focus about a quarter of the time, with random flight and 10 bees.
- Faster limits help the framing a lot (in frame 64 % with the moderate limits of D74, 84 %
  with a hypothetical fast drive) but the focus much less (28 % and 32 %). The 2 mm depth of
  field, not the axes, limits the focus. The mean focus error stays 8 to 23 mm.
- Tracking is the same in all three cases, as it should be: the robot does not feed back into
  the tracker.

## 7. What did not work or is weak

- No layout with fewer than 16 cameras passed the criterion.
- 8 cameras on 4 nodes are fragile: one failed node removes a corner edge and leaves the worst
  bee tracked 63 % of the time with trips.
- Crowding (trips, 10 to 20 bees) causes most ID swaps and most ghost tracks of the
  correctly calibrated runs.
- Focus is the robot's weak point in every case: at most about a third of the time with
  random flight, even with fast limits.
- The robot numbers vary by 10 to 25 points between seeds. Five seeds of 18 s are too few to
  rank layouts by them.
- Prediction 100 ms ahead is poor with random flight (33 mm mean). This is a property of the
  random flight model as much as of the tracker.
- Not run: the arena from an STL mesh with occlusion (needs `trimesh`, not approved, D97),
  and the 8-camera layout on 8 nodes with a failed node.

## 8. Assumptions the numbers depend on

- **Robot values are assumed** (`config/robot.yaml`, D92): only the rail speed of 3 m/s is
  stated. Acceleration, tilt and focus limits are the cautious end of a NEMA 23 drive. The
  cinema picture of 24 mm and the depth of field of 2 mm are derived, not measured. Every
  robot number changes when Jeremy gives real values.
- **Both flight models are guesses.** Random flight (Ornstein-Uhlenbeck velocity, 1 m/s RMS,
  0.5 s direction time) and trips (feeder at the lit end, entrance at the other, timings
  assumed, D87 to D91). Neither is fitted to real bumblebee flight.
- **Camera noise and timing** are the defaults of `docs/camera-node.md` (0.5 px, 2 %
  dropout, 8 ± 2 ms), not measured on a Raspberry Pi.
- **The robot emulator** has no delay from stereo exposure to motor command beyond one frame,
  no vibration, no flex and a perfect focus curve (D57).
- **The selected bee** is always the oldest confirmed track, not a bee a person would pick.

## 9. Limits of the simulation

- Virtual cameras deliver points, not images: no reflections, no motion blur, no lighting,
  no background clutter and no blob detector (Stage 8).
- Bees are points with a size; they do not hide each other, and nothing hides them (no
  occlusion, no robot in the view of a tracking camera).
- The arena is an empty box; walls are glass that cameras see through.
- The network is ideal apart from the configured delay and jitter: no lost datagrams other
  than dropout, no clock drift, a perfect trigger.
- Each case was run for 18 evaluated seconds with 5 seeds: enough for tracking numbers, too
  few for robot numbers.

## 10. Files

- `experiments/results/runs.csv`, `summary.csv`, `criterion.csv`, `coverage.json`.
- Plots (SVG, plain Python, no plotting library, D97): `plot-cameras-*.svg`,
  `plot-bees-*.svg`, `plot-coverage.svg`, `plot-noise-*.svg`, `plot-delay-*.svg`,
  `plot-calibration-*.svg`, `plot-robot-*.svg`, `plot-nodes-*.svg`, and coverage maps
  `coverage-16.svg`, `coverage-12.svg`, `coverage-10.svg`, `coverage-8.svg` (three
  horizontal slices each, colour = number of cameras).
- Paper figures: `docs/paper-figures/` with its own README.
