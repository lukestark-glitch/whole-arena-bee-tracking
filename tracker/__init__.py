"""The software under test: turns `detections` into tracked 3D positions with stable IDs.

The tracker knows nothing about the simulator. It reads the configuration and the
`detections` messages, and never sees the ground truth. All times are simulation time
taken from the messages.
"""
