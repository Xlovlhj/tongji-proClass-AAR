"""Export full-resolution logs and concise, self-contained scientific SVG plots.

Plots contain a title, coordinate axes and necessary legends. Statistics remain
in JSON/CSV. XY axes use an equal metric scale; pen lifts never join strokes.
Comparisons pair samples at the same time rather than aligning nearest points.
"""

from __future__ import annotations

import csv
from html import escape
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


CSV_FIELDS = (
    "time", "target_x", "target_y", "target_z", "actual_x", "actual_y", "actual_z",
    "pen_down", "stroke_index",
    *(f"q{i}" for i in range(1, 8)),
    *(f"dq{i}" for i in range(1, 8)),
    *(f"q_ref{i}" for i in range(1, 8)),
    *(f"tau{i}" for i in range(1, 8)),
)
TARGET_COLOR = "#64748b"
ACTUAL_COLOR = "#008577"
DISTURBED_COLOR = "#d55e00"
AXIS_COLORS = ("#0072b2", "#d55e00", "#8b3bb8")


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} requires a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} requires a finite number")
    return number


def _text(x, y, content, *, size=16, anchor="start", weight=400, rotation=None):
    transform = f' transform="rotate({rotation} {x:.2f} {y:.2f})"' if rotation is not None else ""
    return (f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" fill="#111111" '
            f'text-anchor="{anchor}" font-weight="{weight}"{transform}>{escape(str(content))}</text>')


def _start(title: str, height: int) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="800" height="{height}" viewBox="0 0 800 {height}" role="img" aria-labelledby="title">',
        f'<title id="title">{escape(title)}</title>',
        f'<rect width="800" height="{height}" fill="white"/>',
        '<g font-family="Arial, Helvetica, sans-serif">',
        _text(400, 38, title, size=23, anchor="middle", weight=600),
    ]


def _line(x1, y1, x2, y2, color="#dedede", width=1, dash=""):
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{color}" stroke-width="{width}"{extra}/>')


def _thin_xy(points, preserve=(), maximum=2500):
    if len(points) <= maximum:
        return points
    stride = math.ceil(len(points) / maximum)
    indices = {0, len(points) - 1, *range(0, len(points), stride), *preserve}
    return [points[i] for i in sorted(indices) if 0 <= i < len(points)]


def _thin_time(points, maximum=2000):
    """Retain min/max in each bucket, so narrow error peaks remain visible."""
    if len(points) <= maximum:
        return points
    bucket = math.ceil(len(points) / (maximum // 2))
    indices = {0, len(points) - 1}
    for start in range(0, len(points), bucket):
        stop = min(start + bucket, len(points))
        indices.add(min(range(start, stop), key=lambda i: points[i][1]))
        indices.add(max(range(start, stop), key=lambda i: points[i][1]))
    return [points[i] for i in sorted(indices)]


def _polyline(points, color, width=2, *, dash="", temporal=False, preserve=()):
    points = _thin_time(points) if temporal else _thin_xy(points, preserve)
    if not points:
        return ""
    if len(points) == 1:
        x, y = points[0]
        return f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{max(width / 2, 1):.2f}" fill="{color}"/>'
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    coordinates = " ".join(f"{x:.3f},{y:.3f}" for x, y in points)
    return (f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linejoin="round" stroke-linecap="round"{extra}/>')


def _tick(value, decimals=1):
    if abs(value) < .5 * 10 ** -decimals:
        value = 0.0
    return f"{value:.{decimals}f}"


def _legend(parts, entries):
    # Shared locations keep the plotting region independent of labels.
    spacing = 660 / max(len(entries), 1)
    for index, (label, color, width, dash) in enumerate(entries):
        x = 108 + index * spacing
        parts.extend([_line(x, 73, x + 32, 73, color, width, dash),
                      _text(x + 42, 78, label, size=15)])


def _drawing_segments(rows):
    segments = []
    current = []
    previous_stroke = None
    for row in rows:
        stroke = row.get("stroke_index")
        if not row.get("pen_down", False):
            if current:
                segments.append(current)
                current = []
            previous_stroke = None
            continue
        if current and stroke != previous_stroke:
            segments.append(current)
            current = []
        current.append(row)
        previous_stroke = stroke
    if current:
        segments.append(current)
    return segments


def _xy_svg(title, series):
    """Series: (label, prefix, color, width, dash, segmented row lists)."""
    parts = _start(title, 800)
    _legend(parts, [(label, color, width, dash) for label, _, color, width, dash, _ in series])
    points = [(float(row[f"{prefix}_x"]), float(row[f"{prefix}_y"]))
              for _, prefix, _, _, _, segments in series for segment in segments for row in segment]
    if points:
        low_x, high_x = min(x for x, y in points), max(x for x, y in points)
        low_y, high_y = min(y for x, y in points), max(y for x, y in points)
        center_x, center_y = (low_x + high_x) / 2, (low_y + high_y) / 2
        span = max(high_x - low_x, high_y - low_y, .01) * 1.14
    else:
        center_x = center_y = 0
        span = .2
    x0, y0, size = 120.0, 115.0, 600.0
    low_x, low_y = center_x - span / 2, center_y - span / 2

    def project(x, y):
        return x0 + (x - low_x) / span * size, y0 + size - (y - low_y) / span * size

    for i in range(5):
        fraction = i / 4
        x, y = x0 + fraction * size, y0 + size - fraction * size
        parts.extend([_line(x, y0, x, y0 + size), _line(x0, y, x0 + size, y),
                      _text(x, y0 + size + 26, _tick((low_x + fraction * span) * 1000), size=13, anchor="middle"),
                      _text(x0 - 12, y + 5, _tick((low_y + fraction * span) * 1000), size=13, anchor="end")])
    for label, prefix, color, width, dash, segments in series:
        for segment in segments:
            coords = [project(float(row[f"{prefix}_x"]), float(row[f"{prefix}_y"])) for row in segment]
            peaks = []
            for key in ("normal_tip_error_m", "disturbed_tip_error_m", "disturbance_deviation_m"):
                if segment and key in segment[0]:
                    peaks.append(max(range(len(segment)), key=lambda j: float(segment[j][key])))
            if prefix == "actual" and segment:
                peaks.append(max(range(len(segment)), key=lambda j: sum(
                    (float(segment[j][f"actual_{a}"]) - float(segment[j][f"target_{a}"])) ** 2 for a in "xyz")))
            parts.append(_polyline(coords, color, width, dash=dash, preserve=peaks))
    parts.extend([_line(x0, y0 + size, x0 + size, y0 + size, "#111", 1.3),
                  _line(x0, y0, x0, y0 + size, "#111", 1.3),
                  _text(x0 + size / 2, 779, "X (mm)", size=18, anchor="middle"),
                  _text(29, y0 + size / 2, "Y (mm)", size=18, anchor="middle", rotation=-90),
                  "</g></svg>"])
    return "\n".join(parts)


def _time_svg(title, times, series, ylabel, *, nonnegative=False):
    """Series: (label, values in displayed units, color, dash)."""
    parts = _start(title, 580)
    _legend(parts, [(label, color, 2.2, dash) for label, _, color, dash in series])
    low_t = times[0] if times else 0
    high_t = times[-1] if len(times) > 1 and times[-1] > low_t else low_t + 1
    values = [v for _, data, _, _ in series for v in data]
    low_y = min([0.0, *values])
    high_y = max([0.0, *values])
    margin = max((high_y - low_y) * .08, .05)
    low_y = 0.0 if nonnegative else low_y - margin
    high_y += margin
    x0, y0, width, height = 113.0, 115.0, 630.0, 375.0

    def project(t, value):
        return (x0 + (t - low_t) / (high_t - low_t) * width,
                y0 + (high_y - value) / (high_y - low_y) * height)

    for i in range(6):
        f = i / 5
        x, y = x0 + f * width, y0 + height - f * height
        parts.extend([_line(x, y0, x, y0 + height), _line(x0, y, x0 + width, y),
                      _text(x, y0 + height + 26, _tick(low_t + f * (high_t - low_t)), size=13, anchor="middle"),
                      _text(x0 - 11, y + 5, _tick(low_y + f * (high_y - low_y), 2), size=13, anchor="end")])
    if low_y <= 0 <= high_y:
        zero = project(low_t, 0)[1]
        parts.append(_line(x0, zero, x0 + width, zero, "#999", .9, "4 4"))
    for label, data, color, dash in series:
        parts.append(_polyline([project(t, value) for t, value in zip(times, data)], color, 2, dash=dash, temporal=True))
    parts.extend([_line(x0, y0 + height, x0 + width, y0 + height, "#111", 1.3),
                  _line(x0, y0, x0, y0 + height, "#111", 1.3),
                  _text(x0 + width / 2, 553, "Time (s)", size=18, anchor="middle"),
                  _text(27, y0 + height / 2, ylabel, size=17, anchor="middle", rotation=-90),
                  "</g></svg>"])
    return "\n".join(parts)


def _validate_rows(rows):
    for index, row in enumerate(rows):
        for key in ("time", "target_x", "target_y", "target_z", "actual_x", "actual_y", "actual_z"):
            _finite(row.get(key), f"row {index} {key}")
        if row.get("pen_down", False) not in (True, False, 0, 1):
            raise ValueError(f"row {index} pen_down must be boolean")
        if index and float(row["time"]) < float(rows[index - 1]["time"]):
            raise ValueError("row times must be nondecreasing")


def _boolean(value, name):
    if value in (True, 1, "True", "true", "1"):
        return True
    if value in (False, 0, "False", "false", "0", ""):
        return False
    raise ValueError(f"{name} must be boolean")


def load_run(output_dir: str | Path):
    """Read full CSV rows and metadata for comparison or plot regeneration."""
    output = Path(output_dir)
    with (output / "metrics.json").open(encoding="utf-8") as stream:
        metadata = json.load(stream)
    if not isinstance(metadata, dict):
        raise ValueError("metrics.json must contain an object")
    with (output / "trajectory.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for i, row in enumerate(rows):
        row["pen_down"] = _boolean(row.get("pen_down", ""), f"row {i} pen_down")
        row["stroke_index"] = int(row.get("stroke_index") or -1)
        for key in ("time", "target_x", "target_y", "target_z", "actual_x", "actual_y", "actual_z"):
            row[key] = _finite(row.get(key), f"row {i} {key}")
        if "disturbance_active" in row:
            row["disturbance_active"] = _boolean(row["disturbance_active"], f"row {i} disturbance_active")
        if "disturbance_pulse_index" in row:
            row["disturbance_pulse_index"] = int(row["disturbance_pulse_index"])
        for axis in "xyz":
            key = f"external_force_{axis}_n"
            if key in row:
                row[key] = _finite(row[key], f"row {i} {key}")
    _validate_rows(rows)
    return rows, metadata


def _reference_rows(output: Path, fallback):
    source = output / "reference_cartesian.csv"
    if not source.is_file():
        return fallback
    with source.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    reference = []
    for i, row in enumerate(rows):
        reference.append({"time": _finite(row.get("time_s"), f"reference row {i} time"),
                          **{f"target_{axis}": _finite(row.get(f"{axis}_m"), f"reference row {i} {axis}") for axis in "xyz"},
                          "pen_down": _boolean(row.get("pen_down", ""), f"reference row {i} pen_down"),
                          "stroke_index": int(row.get("stroke_index") or -1)})
    return reference


def _write_csv(path, rows, standard=()):
    fields = list(standard)
    known = set(fields)
    for row in rows:
        for key in row:
            if key not in known:
                fields.append(key)
                known.add(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_results(output_dir: str | Path, rows: Sequence[Mapping[str, Any]],
                 metadata: Mapping[str, Any], *, reference_rows=None) -> dict[str, Any]:
    """Preserve all supplied statistics/log columns; write four concise plots.

    The complete target is read from reference_cartesian.csv when present.
    ``reference_rows`` can explicitly supply target_x/y/z, pen_down and stroke
    fields instead. No plot adds summary statistics or explanatory footnotes.
    Empty/interrupted recordings remain exportable with empty plot axes.
    """
    output = Path(output_dir)
    rows = list(rows)
    metadata = dict(metadata)
    metrics = metadata.get("metrics", {})
    if not isinstance(metrics, Mapping):
        raise ValueError("metadata['metrics'] must be a mapping")
    metadata["metrics"] = dict(metrics)
    _validate_rows(rows)
    metadata_json = json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    complete_target = _reference_rows(output, rows) if reference_rows is None else list(reference_rows)
    segments = _drawing_segments(rows)
    target_segments = _drawing_segments(complete_target)
    drawing = _xy_svg("Trajectory tracking", [("Target", "target", TARGET_COLOR, 3, "", segments),
                                                ("Actual", "actual", ACTUAL_COLOR, 1.8, "", segments)])
    target = _xy_svg("Target trajectory", [("Target", "target", TARGET_COLOR, 2, "", target_segments)])
    actual = _xy_svg("Actual trajectory", [("Actual", "actual", ACTUAL_COLOR, 2, "", segments)])
    times = [float(row["time"]) for row in rows]
    errors = [[(float(row[f"actual_{axis}"]) - float(row[f"target_{axis}"])) * 1000 for row in rows] for axis in "xyz"]
    error_plot = _time_svg("Position tracking error", times,
                           [(axis.upper(), values, color, "") for axis, values, color in zip("xyz", errors, AXIS_COLORS)],
                           "Position error (mm)")
    output.mkdir(parents=True, exist_ok=True)
    _write_csv(output / "trajectory.csv", rows, CSV_FIELDS)
    (output / "metrics.json").write_text(metadata_json, encoding="utf-8")
    for filename, plot in (("drawing_result.svg", drawing), ("error_history.svg", error_plot),
                           ("target_trajectory.svg", target), ("actual_trajectory.svg", actual)):
        (output / filename).write_text(plot + "\n", encoding="utf-8")
    return dict(metrics)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def _summary(values, mask):
    magnitudes = [_norm(v) for v in values]
    xy = [math.hypot(v[0], v[1]) for v in values]

    def rms(numbers):
        return math.sqrt(sum(x * x for x in numbers) / len(numbers)) if numbers else None

    drawing = [v for v, down in zip(magnitudes, mask) if down]
    drawing_xy = [v for v, down in zip(xy, mask) if down]
    return {"sample_count": len(values), "drawing_sample_count": sum(mask),
            "tip_rmse_m": rms(magnitudes), "tip_max_error_m": max(magnitudes, default=None),
            "xy_rmse_m": rms(xy), "xy_max_error_m": max(xy, default=None),
            "drawing_tip_rmse_m": rms(drawing), "drawing_tip_max_error_m": max(drawing, default=None),
            "drawing_xy_rmse_m": rms(drawing_xy), "drawing_xy_max_error_m": max(drawing_xy, default=None)}


def _recovery_metrics(times, deviation, metadata):
    windows = metadata.get("disturbance_windows", [])
    if not isinstance(windows, list):
        raise ValueError("disturbance_windows must be a list")
    threshold = _finite(metadata.get("recovery_threshold_m", .002), "recovery_threshold_m")
    hold = _finite(metadata.get("recovery_hold_s", .5), "recovery_hold_s")
    if threshold <= 0 or hold < 0:
        raise ValueError("recovery threshold must be positive and hold time nonnegative")
    result = []
    last_end = -math.inf
    checked = []
    for window in windows:
        if not isinstance(window, Mapping):
            raise ValueError("disturbance windows must be objects")
        start = _finite(window.get("start_s"), "disturbance start_s")
        end = _finite(window.get("end_s"), "disturbance end_s")
        if start < times[0] - 1e-9 or end > times[-1] + 1e-9 or end < start or start < last_end:
            raise ValueError("disturbance windows must be ordered, non-overlapping and within the recorded times")
        checked.append((start, end, dict(window)))
        last_end = end
    magnitudes = [_norm(values) for values in deviation]
    for index, (start, end, original) in enumerate(checked):
        next_start = checked[index + 1][0] if index + 1 < len(checked) else math.inf
        recovery_start = None
        recovered_at = None
        confirmed_at = None
        for t, error in zip(times, magnitudes):
            if t < end - 1e-9:
                continue
            if t >= next_start - 1e-9:
                break
            if error <= threshold:
                if recovery_start is None:
                    recovery_start = t
                if t - recovery_start >= hold - 1e-9:
                    recovered_at = recovery_start
                    confirmed_at = t
                    break
            else:
                recovery_start = None
        affected = [error for t, error in zip(times, magnitudes) if start - 1e-9 <= t < next_start - 1e-9]
        during = [error for t, error in zip(times, magnitudes) if start - 1e-9 <= t <= end + 1e-9]
        result.append({**original,
                       "recovery_time_s": max(0.0, recovered_at - end) if recovered_at is not None else None,
                       "recovery_started_at_s": recovered_at,
                       "recovery_confirmed_at_s": confirmed_at,
                       "peak_deviation_during_force_m": max(during, default=None),
                       "peak_deviation_until_next_force_m": max(affected, default=None),
                       "recovery_observation_end_s": min(next_start, times[-1])})
    return {"metric": "3D distance between disturbed and normal positions at the same time",
            "threshold_m": threshold, "required_hold_s": hold, "windows": result}


def compare_runs(normal_output: str | Path, disturbed_output: str | Path,
                 destination: str | Path) -> dict[str, Any]:
    """Compare completed experiments with identical times, targets and pen flags.

    Writes comparison.csv (every paired sample), comparison_metrics.json and
    three concise SVG figures. Recovery uses the disturbed-minus-normal 3D
    deviation, with thresholds and force windows from the disturbed metadata.
    A failed alignment check creates no comparison output directory.
    """
    normal, normal_meta = load_run(normal_output)
    disturbed, disturbed_meta = load_run(disturbed_output)
    if normal_meta.get("completed") is not True or disturbed_meta.get("completed") is not True:
        raise ValueError("comparison requires two completed runs")
    if not normal or len(normal) != len(disturbed):
        raise ValueError("comparison requires equal nonempty sample counts")
    for index, (a, b) in enumerate(zip(normal, disturbed)):
        if index and a["time"] <= normal[index - 1]["time"]:
            raise ValueError("comparison times must be strictly increasing")
        if not math.isclose(a["time"], b["time"], rel_tol=0, abs_tol=1e-9):
            raise ValueError(f"comparison time mismatch at sample {index}")
        for axis in "xyz":
            if not math.isclose(a[f"target_{axis}"], b[f"target_{axis}"], rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"comparison target mismatch at sample {index}")
        if a["pen_down"] != b["pen_down"] or a["stroke_index"] != b["stroke_index"]:
            raise ValueError(f"comparison pen state mismatch at sample {index}")
    for label, meta in (("normal", normal_meta), ("disturbed", disturbed_meta)):
        if "reference_duration_s" in meta:
            duration = _finite(meta["reference_duration_s"], f"{label} reference duration")
            if normal[0]["time"] > 1e-9 or normal[-1]["time"] < duration - 1e-9:
                raise ValueError(f"{label} data do not cover the complete reference")
    rows = []
    normal_errors, disturbed_errors, deviations = [], [], []
    for a, b in zip(normal, disturbed):
        target = [a[f"target_{axis}"] for axis in "xyz"]
        n = [a[f"actual_{axis}"] for axis in "xyz"]
        d = [b[f"actual_{axis}"] for axis in "xyz"]
        ne, de, delta = ([n[i] - target[i] for i in range(3)],
                         [d[i] - target[i] for i in range(3)], [d[i] - n[i] for i in range(3)])
        normal_errors.append(ne)
        disturbed_errors.append(de)
        deviations.append(delta)
        row = {"time": a["time"], "pen_down": a["pen_down"], "stroke_index": a["stroke_index"]}
        for prefix, values in (("target", target), ("normal", n), ("disturbed", d),
                               ("normal_error", ne), ("disturbed_error", de), ("deviation", delta)):
            row.update({f"{prefix}_{axis}": value for axis, value in zip("xyz", values)})
        row.update(normal_tip_error_m=_norm(ne), disturbed_tip_error_m=_norm(de), disturbance_deviation_m=_norm(delta))
        rows.append(row)
    times = [row["time"] for row in rows]
    mask = [row["pen_down"] for row in rows]
    metadata = {"normal_output": str(Path(normal_output).resolve()),
                "disturbed_output": str(Path(disturbed_output).resolve()),
                "sample_count": len(rows), "duration_s": times[-1] - times[0],
                "units": {"time": "s", "positions_and_errors": "m"},
                "metrics": {"normal_tracking": _summary(normal_errors, mask),
                            "disturbed_tracking": _summary(disturbed_errors, mask),
                            "disturbance_deviation": _summary(deviations, mask)},
                "recovery": _recovery_metrics(times, deviations, disturbed_meta)}
    metadata_text = json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    segments = _drawing_segments(rows)
    trajectory_plot = _xy_svg("Trajectory comparison", [("Target", "target", TARGET_COLOR, 3, "", segments),
                                                        ("Normal", "normal", ACTUAL_COLOR, 1.7, "", segments),
                                                        ("Disturbed", "disturbed", DISTURBED_COLOR, 1.7, "4 3", segments)])
    tracking_plot = _time_svg("Tracking error comparison", times,
                              [("Normal", [row["normal_tip_error_m"] * 1000 for row in rows], ACTUAL_COLOR, ""),
                               ("Disturbed", [row["disturbed_tip_error_m"] * 1000 for row in rows], DISTURBED_COLOR, "4 3")],
                              "3D tracking error (mm)", nonnegative=True)
    deviation_plot = _time_svg("Disturbance deviation", times,
                               [(axis.upper(), [row[f"deviation_{axis}"] * 1000 for row in rows], color, "")
                                for axis, color in zip("xyz", AXIS_COLORS)], "Position difference (mm)")
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    _write_csv(destination / "comparison.csv", rows)
    (destination / "comparison_metrics.json").write_text(metadata_text, encoding="utf-8")
    for filename, plot in (("trajectory_comparison.svg", trajectory_plot),
                           ("tracking_error_comparison.svg", tracking_plot),
                           ("disturbance_deviation.svg", deviation_plot)):
        (destination / filename).write_text(plot + "\n", encoding="utf-8")
    return metadata
