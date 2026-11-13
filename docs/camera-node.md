# Camera node specification

What the software on a real camera node has to do, how it could be built, and what it
outputs. A camera node is one small computer with two tracking cameras.

Status: specification only. This code does not exist in the repository and is not part of
the simulation. In the simulation, `simulator/cameras.py` and `simulator/nodes.py` produce
the same output directly from the known bee positions. Nothing in this file has been tested
on hardware.

## Job in one sentence

For every trigger pulse, capture one image from each of its two cameras, find the bees in
both images, and send their pixel coordinates to the tracking server before the next pulse.

## Assumed hardware

| Part | Assumption |
| --- | --- |
| Computer | Raspberry Pi 5 class, two CSI camera ports |
| Cameras | Global shutter, monochrome, Sony IMX296 class, 1456 × 1088 px, about 60 fps, external trigger input |
| Lens | Wide angle, about 110° horizontal, fixed focus, 850 nm infrared-pass filter |
| Trigger | One pulse per frame from a shared source, wired to both cameras of every node |
| Network | Wired Ethernet to the tracking server |

## Inputs

- Two image streams, one frame per trigger pulse each.
- A per-node configuration: node id, the two camera ids, server address and port,
  thresholds, marker rate.

A node does not need the camera calibration. It reports raw pixel coordinates, and the
server applies the lens model and the camera pose. This keeps nodes interchangeable and
keeps all calibration in one place.

## Output

One `detections` message per camera and frame, exactly as defined in `docs/protocol.md`,
also when nothing was found. An empty message tells the server that the camera is alive and
saw no bee.

## Processing per frame

For each of the two images:

1. **Capture.** Take the frame that belongs to the trigger pulse. Record the frame counter
   and the exposure time.
2. **Background model.** Keep a slowly updated image of the empty arena, for example a
   running average with a long time constant. Do not update it where a bee was detected,
   otherwise a sitting bee fades into the background.
3. **Difference.** Subtract the background and take the absolute value. This removes the
   static wall pattern and fixed bright spots such as opposite infrared LEDs.
4. **Threshold.** Mark pixels whose difference exceeds a threshold. A fixed mask removes
   regions that are never valid.
5. **Blobs.** Group marked pixels into connected regions. Discard regions that are too
   small (noise) or too large (a hand, a door).
6. **Centre.** Compute the intensity-weighted centre of each region. This gives sub-pixel
   precision, which the 3D accuracy depends on.
7. **Size.** Report the blob's diameter in pixels.
8. **Markers.** Once per second, also locate the wall markers and report their ids and
   pixel positions.
9. **Send.** Build the message and send it over UDP.

Two bees that overlap in the image become one blob. The node reports it as one detection.
The server handles this case, because other cameras usually still see two.

## Timing

- At 60 Hz, a node has 16.7 ms for both images.
- The delay from exposure to message matters more than the frame rate, because the robot
  must predict across it. Aim for a short and constant delay.
- Use short exposures to avoid motion blur. A bee at 5 m/s moves 5 mm in 1 ms.
- Send the two messages as soon as each is ready. Do not wait to combine them.

## Time stamps

The server needs to know which messages belong to the same instant.

- The frame counter is the main key. All cameras count the same trigger pulses, so equal
  counters mean equal instants. All nodes reset the counter on a common start signal.
- The time `t` is the frame counter times the trigger period plus the start time, shifted
  to the middle of the exposure. Derived this way, it does not depend on each node's clock.
- Node clocks should still be synchronised over the network for logging.

## How it could be implemented

- **First version:** Python with OpenCV and the Raspberry Pi camera library. Background
  subtraction, threshold and connected components are single library calls. Measure whether
  two full-resolution images fit into the frame time.
- **If it is too slow:** process at half resolution and refine each centre at full
  resolution in a small window; or search only near the last detections and scan the full
  image every few frames; or move the pixel loop to C++.
- **Robustness:** start on boot, reconnect by itself, and keep sending empty messages when
  nothing is seen so the server can tell a silent arena from a dead node.

## Failure behaviour

- If one camera fails, the node keeps sending for the other.
- If the node fails, both cameras disappear. The camera layout must tolerate the loss of
  any single node, which is one of the questions for the simulation.

## What the simulation assumes about a node

The simulator models a node by its outside behaviour only. If hardware tests show different
values, change them in the configuration.

| Property | Simulated as |
| --- | --- |
| Detection position | True projection plus Gaussian noise (default 0.5 px) |
| Missed detections | Random dropout per detection (default 2 %) |
| Processing delay | Fixed delay per node plus jitter (default 8 ms ± 2 ms) |
| Overlapping bees | Merged into one detection when closer than the blob size |
| Node failure | Both cameras stop reporting |

## Open questions for hardware

- How bright is a bumblebee at 850 nm against the chosen background material?
- Does the faint wall pattern survive background subtraction without leaving residue?
- Can two full-resolution images be processed in 16.7 ms on the chosen computer?
- Do cameras inside the arena need covers, and do covers cause reflections?
