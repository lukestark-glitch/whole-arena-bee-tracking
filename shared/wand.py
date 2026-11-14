"""Schedule of the calibration wand run, shared by the simulator and the tracker.

During calibration a single LED (the wand) is the only bright point in the arena. The run
has three phases in a fixed order, with durations from the ``calibration`` section of the
config:

1. reference: the wand rests on each known reference point in turn,
2. sweep: the wand moves through the arena,
3. stop: the wand rests on stop points while the robot is aimed at it.

The simulator uses the schedule to place the wand, the tracker to tell which frames belong
to which phase. With real hardware the operator would follow the same order.
"""

from shared.config import CalibrationSettings

REFERENCE = "reference"
SWEEP = "sweep"
STOP = "stop"
DONE = "done"


def wand_duration_s(settings: CalibrationSettings) -> float:
    """Length of the whole wand run.

    Args:
        settings: Calibration settings (durations in s).

    Returns:
        The duration in simulation seconds.
    """
    reference_s = len(settings.reference_points_m) * settings.reference_dwell_s
    return reference_s + settings.sweep_duration_s + settings.stop_count * settings.stop_dwell_s


def wand_phase(settings: CalibrationSettings, t_s: float) -> tuple[str, int, float]:
    """Tell which phase of the wand run a time belongs to.

    Args:
        settings: Calibration settings (durations in s).
        t_s: Simulation time since the start of the run (s), 0 or more.

    Returns:
        The phase (REFERENCE, SWEEP, STOP or DONE), the index of the reference or stop point
        (0 for SWEEP and DONE) and the time since that point or phase began (s).
    """
    reference_s = len(settings.reference_points_m) * settings.reference_dwell_s
    if t_s < reference_s:
        return REFERENCE, int(t_s // settings.reference_dwell_s), t_s % settings.reference_dwell_s
    t_s -= reference_s
    if t_s < settings.sweep_duration_s:
        return SWEEP, 0, t_s
    t_s -= settings.sweep_duration_s
    if t_s < settings.stop_count * settings.stop_dwell_s:
        return STOP, int(t_s // settings.stop_dwell_s), t_s % settings.stop_dwell_s
    return DONE, 0, t_s - settings.stop_count * settings.stop_dwell_s
