"""SVG bar charts of the Stage 7 results, written with plain Python (no plotting library).

`experiments.batch` calls `write_plots` with its summary rows; each chart is one SVG file in
``experiments/results/``. The charts are for reading the results and for the paper; every
number in them is also in ``summary.csv``.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

# Chart geometry in SVG pixels.
BAR_PX = 22
GROUP_GAP_PX = 18
PLOT_HEIGHT_PX = 180
LEFT_PX = 56
TOP_PX = 34
BOTTOM_PX = 70
# One colour per series; colour-blind safe (Okabe and Ito).
SERIES_COLOURS = ("#0072B2", "#E69F00", "#009E73", "#CC79A7")
# Grid lines on the value axis.
TICKS = 4


def bar_chart(title: str, unit: str, groups: list[str], series: dict[str, list[float]]) -> str:
    """Draw a grouped bar chart.

    Args:
        title: Chart title.
        unit: Unit of the values, shown on the axis.
        groups: Label of each group of bars, left to right.
        series: Series name -> one value per group (nan draws no bar).

    Returns:
        SVG text.
    """
    values = [v for vs in series.values() for v in vs if not math.isnan(v)]
    top = _nice_top(max(values, default=1.0))
    # Groups are wide enough for their label (about 6.5 px per character at 11 px).
    group_px = max(len(series) * BAR_PX, 6.5 * max(len(g) for g in groups)) + GROUP_GAP_PX
    width_px = LEFT_PX + len(groups) * group_px + 20
    legend_px = 18 * len(series)
    height_px = TOP_PX + PLOT_HEIGHT_PX + BOTTOM_PX + legend_px
    base_px = TOP_PX + PLOT_HEIGHT_PX
    parts = [f'<text x="{LEFT_PX}" y="18" font-size="14" font-weight="bold">{_escape(title)}</text>']
    for tick in range(TICKS + 1):
        value = top * tick / TICKS
        y_px = base_px - PLOT_HEIGHT_PX * tick / TICKS
        parts.append(f'<line x1="{LEFT_PX}" x2="{width_px - 20}" y1="{y_px:.1f}" y2="{y_px:.1f}" '
                     f'stroke="#ddd"/><text x="{LEFT_PX - 6}" y="{y_px + 4:.1f}" font-size="11" '
                     f'text-anchor="end">{value:g}</text>')
    parts.append(f'<text x="12" y="{TOP_PX + PLOT_HEIGHT_PX / 2}" font-size="11" '
                 f'transform="rotate(-90 12 {TOP_PX + PLOT_HEIGHT_PX / 2})" '
                 f'text-anchor="middle">{_escape(unit)}</text>')
    for g, group in enumerate(groups):
        x0_px = LEFT_PX + GROUP_GAP_PX / 2 + g * group_px
        for s, (name, vs) in enumerate(series.items()):
            value = vs[g]
            if math.isnan(value):
                continue
            h_px = PLOT_HEIGHT_PX * value / top
            x_px = x0_px + s * BAR_PX
            parts.append(f'<rect x="{x_px:.1f}" y="{base_px - h_px:.1f}" width="{BAR_PX - 3}" '
                         f'height="{h_px:.1f}" fill="{SERIES_COLOURS[s % len(SERIES_COLOURS)]}">'
                         f'<title>{_escape(name)}, {_escape(group)}: {value:.3g}</title></rect>')
        centre_px = x0_px + len(series) * BAR_PX / 2
        parts.append(f'<text x="{centre_px:.1f}" y="{base_px + 16}" font-size="11" '
                     f'text-anchor="middle">{_escape(group)}</text>')
    for s, name in enumerate(series):
        y_px = base_px + 40 + 18 * s
        parts.append(f'<rect x="{LEFT_PX}" y="{y_px - 10}" width="12" height="12" '
                     f'fill="{SERIES_COLOURS[s % len(SERIES_COLOURS)]}"/>'
                     f'<text x="{LEFT_PX + 18}" y="{y_px}" font-size="11">{_escape(name)}</text>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" '
            f'font-family="sans-serif"><rect width="100%" height="100%" fill="white"/>'
            + "".join(parts) + "</svg>")


def _nice_top(value: float) -> float:
    """Round the largest value up to 1, 2 or 5 times a power of ten for the axis."""
    if value <= 0:
        return 1.0
    power = 10 ** math.floor(math.log10(value))
    return next(step * power for step in (1, 2, 5, 10) if step * power >= value)


def _escape(text: str) -> str:
    """Escape the characters SVG text cannot hold."""
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _select(summary: list[dict[str, Any]], experiment: str) -> list[dict[str, Any]]:
    """Return the summary rows of one experiment, in run order."""
    return [row for row in summary if row["experiment"] == experiment]


def _by_model(rows: list[dict[str, Any]], key: str, metric: str) -> tuple[list[str], dict[str, list[float]]]:
    """Arrange rows as groups by ``key`` and one series per flight model.

    Args:
        rows: Summary rows.
        key: Column that names the groups (for example ``bees`` or ``cameras``).
        metric: Column to plot.

    Returns:
        Group labels and series.
    """
    groups = list(dict.fromkeys(row[key] for row in rows))
    series = {model: [next((r[metric] for r in rows if r[key] == g and r["flight_model"] == model),
                           math.nan) for g in groups]
              for model in dict.fromkeys(row["flight_model"] for row in rows)}
    return [str(g) for g in groups], series


def write_plots(summary: list[dict[str, Any]], coverage: dict[int, dict], out: Path) -> None:
    """Write the SVG charts of the Stage 7 results.

    Args:
        summary: Rows of `experiments.batch.summarise`.
        coverage: Number of cameras -> coverage summary.
        out: Output directory.
    """
    charts = {}
    bees = _select(summary, "bees")
    cameras = [r for r in _select(summary, "cameras") if r["stopped_nodes"] == 0]
    for metric, unit, title in (
        ("id_swaps_per_bee_min", "swaps per bee and minute", "ID swaps by number of bees"),
        ("p95_error_mm", "mm", "95th percentile position error by number of bees"),
        ("in_frame_pct", "%", "Robot: time in frame by number of bees"),
    ):
        charts[f"bees-{metric}"] = bar_chart(title, unit, *_by_model(bees, "bees", metric))
    for metric, unit, title in (
        ("p95_error_mm", "mm", "95th percentile position error by number of cameras (5 bees)"),
        ("id_swaps", "swaps in 18 s", "ID swaps by number of cameras (5 bees)"),
        ("in_focus_pct", "%", "Robot: time in focus by number of cameras (5 bees)"),
    ):
        charts[f"cameras-{metric}"] = bar_chart(title, unit, *_by_model(cameras, "cameras", metric))
    layouts = sorted(coverage, reverse=True)
    charts["coverage"] = bar_chart(
        "Share of the arena seen by at least n cameras", "%", [f"{n} cameras" for n in layouts],
        {f"≥ {level}": [coverage[n]["share_by_level"][level] * 100 for n in layouts]
         for level in (3, 4, 6)})
    for experiment, metric, unit, title in (
        ("noise", "p95_error_mm", "mm", "95th percentile error by noise and dropout"),
        ("delay", "in_frame_pct", "%", "Robot: time in frame by node delay"),
        ("calibration", "p95_error_mm", "mm", "95th percentile error by calibration"),
        ("robot", "in_frame_pct", "%", "Robot: time in frame by robot limits"),
        ("nodes", "tracked_pct", "%", "Tracked share by stopped nodes"),
    ):
        rows = _select(summary, experiment)
        charts[f"{experiment}-{metric}"] = bar_chart(
            title, unit, [r["label"] for r in rows], {"random": [r[metric] for r in rows]})
    for name, svg in charts.items():
        (out / f"plot-{name}.svg").write_text(svg, encoding="utf-8")
