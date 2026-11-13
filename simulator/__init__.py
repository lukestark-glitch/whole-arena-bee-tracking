"""Stand-in for the real world: bees, virtual cameras and camera nodes.

The simulator owns simulation time and the ground truth. It sends the `detections`
messages real camera nodes would send, and `truth` for the metrics. The tracker never
imports from this package.
"""
