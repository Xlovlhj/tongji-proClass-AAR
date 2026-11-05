"""Analyze one L4 path-tracking or terrain Isaac run.

The input is an experiment directory containing ``run.json`` and
``states.csv``.  Path runs additionally contain ``reference_path.csv`` and
normally ``path_metrics.json``.  Results default to the run's ``analysis/``
subdirectory so L4 experiments remain isolated from one another.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover - only a broken project environment
    raise SystemExit(
        "缺少项目已有依赖 Pillow，无法生成 L4 PNG 分析图。"
    ) from exc


CORE_FIELDS = (
    "time_s", "base_x_m", "base_y_m", "base_z_m", "roll_rad",
    "pitch_rad", "yaw_rad", "vx_world_m_s", "vy_world_m_s",
    "cmd_vx_body_m_s", "cmd_vy_body_m_s", "walking_command",
)
PATH_FIELDS = (
    "path:cross_track_error_m", "path:path_distance_m",
    "path:heading_error_rad", "path:progress_m",
    "path:progress_fraction", "path:goal_distance_m", "path:reached_goal",
)

BLUE = (23, 105, 170)
ORANGE = (209, 122, 0)
GRAY = (91, 100, 114)
PROJECT_ROOT = Path(__file__).resolve().parents[2]
STEP_FRONT_CLEARANCE_THRESHOLD_M = 0.30
STEP_FRONT_PROXY_NOTE = (
    "base-only proxy：仅依据台阶 y 范围内的基座最大 x；states.csv 未记录足端 "
    "contact prim，因此不能等同于双脚已接触上台阶的证明。"
)


def _load_json(path: Path, *, required: bool = True):
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"缺少文件：{path}")
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} 的顶层必须是 JSON 对象")
    return value


def load_columns(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if not fields:
            raise ValueError(f"CSV 没有表头：{path}")
        if len(fields) != len(set(fields)):
            raise ValueError(f"CSV 含重复列名：{path}")
        result = {field: [] for field in fields}
        for line_number, row in enumerate(reader, 2):
            for field in fields:
                try:
                    value = float(row[field])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"{path}:{line_number} 的 {field} 不是数值"
                    ) from exc
                if not math.isfinite(value):
                    raise ValueError(
                        f"{path}:{line_number} 的 {field} 含 NaN/Inf"
                    )
                result[field].append(value)
    if not result[fields[0]]:
        raise ValueError(f"CSV 没有数据行：{path}")
    return result


def load_reference_path(path: Path) -> tuple[list[float], list[float]]:
    columns = load_columns(path)
    missing = [field for field in ("x_m", "y_m") if field not in columns]
    if missing:
        raise KeyError("reference_path.csv 缺少：" + ", ".join(missing))
    if len(columns["x_m"]) < 2:
        raise ValueError("reference_path.csv 至少需要两个路径点")
    return columns["x_m"], columns["y_m"]


def _require(columns: dict[str, list[float]], names: Iterable[str]) -> None:
    missing = [name for name in names if name not in columns]
    if missing:
        raise KeyError("states.csv 缺少：" + ", ".join(missing))


def _mean(values: list[float]) -> float:
    if not values:
        raise ValueError("均值输入不能为空")
    return sum(values) / len(values)


def _rmse(values: list[float]) -> float:
    return math.sqrt(_mean([value * value for value in values]))


def _path_length(xs: list[float], ys: list[float], indices=None) -> float:
    selected = list(range(len(xs))) if indices is None else list(indices)
    return sum(
        math.hypot(xs[b] - xs[a], ys[b] - ys[a])
        for a, b in zip(selected, selected[1:])
    )


def _peak_joint_torque(columns: dict[str, list[float]]):
    series = [values for name, values in columns.items()
              if name.startswith("tau_nm:")]
    return max((max(map(abs, values)) for values in series), default=None)


def _body_velocity_errors(columns, indices):
    forward, lateral = [], []
    for index in indices:
        yaw = columns["yaw_rad"][index]
        c, s = math.cos(yaw), math.sin(yaw)
        vx = columns["vx_world_m_s"][index]
        vy = columns["vy_world_m_s"][index]
        forward.append(c * vx + s * vy - columns["cmd_vx_body_m_s"][index])
        lateral.append(-s * vx + c * vy - columns["cmd_vy_body_m_s"][index])
    return forward, lateral


def _common_metrics(columns, run):
    _require(columns, CORE_FIELDS)
    samples = len(columns["time_s"])
    walking = [index for index, value in enumerate(columns["walking_command"])
               if value >= 0.5]
    forward_error, lateral_error = _body_velocity_errors(columns, walking)
    xs, ys = columns["base_x_m"], columns["base_y_m"]
    return {
        "run_status": run.get("status"),
        "run_error": run.get("error"),
        "logged_samples": samples,
        "logged_duration_s": columns["time_s"][-1] - columns["time_s"][0],
        "walking_samples": len(walking),
        "initial_base_position_m": [xs[0], ys[0], columns["base_z_m"][0]],
        "final_logged_base_position_m": [xs[-1], ys[-1], columns["base_z_m"][-1]],
        "net_forward_displacement_m": xs[-1] - xs[0],
        "net_lateral_displacement_m": ys[-1] - ys[0],
        "planar_distance_travelled_m": _path_length(xs, ys),
        "walking_planar_distance_travelled_m": _path_length(xs, ys, walking),
        "minimum_logged_base_height_m": min(columns["base_z_m"]),
        "maximum_logged_base_height_m": max(columns["base_z_m"]),
        "maximum_logged_tilt_rad": max(
            max(map(abs, columns["roll_rad"])),
            max(map(abs, columns["pitch_rad"])),
        ),
        "maximum_logged_abs_roll_rad": max(map(abs, columns["roll_rad"])),
        "maximum_logged_abs_pitch_rad": max(map(abs, columns["pitch_rad"])),
        "forward_velocity_tracking_rmse_m_s": (
            _rmse(forward_error) if forward_error else None
        ),
        "lateral_velocity_tracking_rmse_m_s": (
            _rmse(lateral_error) if lateral_error else None
        ),
        "peak_abs_logged_joint_torque_Nm": _peak_joint_torque(columns),
        "run_manifest_summary": {
            "simulation_duration_s": run.get("simulation_duration_s"),
            "samples": run.get("samples"),
            "minimum_base_height_m": run.get("minimum_base_height_m"),
            "maximum_tilt_rad": run.get("maximum_tilt_rad"),
            "maximum_joint_torque_nm": run.get("maximum_joint_torque_nm"),
        },
    }


def _same_value(reported, computed) -> bool:
    if isinstance(reported, bool) or isinstance(computed, bool):
        return bool(reported) == bool(computed)
    if reported is None or computed is None:
        return reported is computed
    if isinstance(reported, (int, float)) and isinstance(computed, (int, float)):
        return math.isclose(float(reported), float(computed),
                            rel_tol=1e-6, abs_tol=1e-9)
    return reported == computed


def _reconcile(reported: dict | None, computed: dict,
               keys: Iterable[str] | None = None) -> dict:
    if reported is None:
        return {"available": False, "all_compared_values_match": None,
                "values": {}}
    values = {}
    for key in (computed if keys is None else keys):
        computed_value = computed[key]
        reported_value = reported.get(key)
        values[key] = {
            "reported": reported_value,
            "recomputed": computed_value,
            "matches": _same_value(reported_value, computed_value),
        }
    return {
        "available": True,
        "all_compared_values_match": all(item["matches"] for item in values.values()),
        "values": values,
    }


@dataclass(frozen=True)
class _Series:
    x: Sequence[float]
    y: Sequence[float]
    color: tuple[int, int, int]
    label: str | None = None
    dashed: bool = False


def _font(size: int, bold: bool = False):
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold
        else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            pass
    return ImageFont.load_default()


def _text_size(draw, text, used_font):
    box = draw.textbbox((0, 0), text, font=used_font)
    return box[2] - box[0], box[3] - box[1]


def _dashed_line(draw, points, color, width=4, dash=11):
    phase = 0.0
    pattern = 2.0 * dash
    for start, end in zip(points, points[1:]):
        x0, y0 = start
        x1, y1 = end
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 0:
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        cursor = 0.0
        while cursor < length:
            in_ink = phase < dash
            boundary = dash if in_ink else pattern
            advance = min(boundary - phase, length - cursor)
            stop = cursor + advance
            if in_ink:
                draw.line((x0 + ux * cursor, y0 + uy * cursor,
                           x0 + ux * stop, y0 + uy * stop),
                          fill=color, width=width)
            cursor = stop
            phase = (phase + advance) % pattern


def _padded_range(low, high):
    if math.isclose(low, high):
        padding = max(abs(low) * 0.08, 0.01)
    else:
        padding = (high - low) * 0.08
    return low - padding, high + padding


def _format_tick(value: float, step: float) -> str:
    """Format adjacent ticks distinctly without adding unnecessary zeros."""
    magnitude = abs(value)
    if magnitude >= 1e5 or (0 < magnitude < 1e-4):
        return f"{value:.2e}"
    if step <= 0 or not math.isfinite(step):
        return f"{value:g}"
    decimals = max(0, min(8, -math.floor(math.log10(abs(step))) + 1))
    text = f"{value:.{decimals}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return "0" if text in {"-0", "+0"} else text


def _save_plot(path: Path, title: str, x_label: str, y_label: str,
               series: Sequence[_Series], *, equal_scale: bool = False):
    if not series:
        raise ValueError(f"{title} 没有可画数据")
    width, height = 1500, 900
    left, top, right, bottom = 155, 150, 1445, 740
    xs = [float(value) for item in series for value in item.x]
    ys = [float(value) for item in series for value in item.y]
    if not xs or not ys:
        raise ValueError(f"{title} 没有可画数据")
    if x_label == "Time (s)" and not math.isclose(min(xs), max(xs)):
        x_low, x_high = min(xs), max(xs)
    else:
        x_low, x_high = _padded_range(min(xs), max(xs))
    y_low, y_high = _padded_range(min(ys), max(ys))
    if equal_scale:
        plot_ratio = (right - left) / (bottom - top)
        data_ratio = (x_high - x_low) / (y_high - y_low)
        if data_ratio < plot_ratio:
            target = (y_high - y_low) * plot_ratio
            center = (x_low + x_high) / 2
            x_low, x_high = center - target / 2, center + target / 2
        else:
            target = (x_high - x_low) / plot_ratio
            center = (y_low + y_high) / 2
            y_low, y_high = center - target / 2, center + target / 2

    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    title_font, label_font = _font(37, True), _font(25)
    tick_font, legend_font = _font(21), _font(22)
    x_step = (x_high - x_low) / 5
    y_step = (y_high - y_low) / 5
    for tick in range(6):
        fraction = tick / 5
        px = left + fraction * (right - left)
        py = bottom - fraction * (bottom - top)
        draw.line((px, top, px, bottom), fill=(229, 232, 236), width=1)
        draw.line((left, py, right, py), fill=(229, 232, 236), width=1)
        x_text = _format_tick(x_low + fraction * (x_high - x_low), x_step)
        y_text = _format_tick(y_low + fraction * (y_high - y_low), y_step)
        x_width, _ = _text_size(draw, x_text, tick_font)
        y_width, y_height = _text_size(draw, y_text, tick_font)
        draw.text((px - x_width / 2, bottom + 12), x_text,
                  fill=(54, 60, 69), font=tick_font)
        draw.text((left - y_width - 14, py - y_height / 2), y_text,
                  fill=(54, 60, 69), font=tick_font)
    draw.line((left, top, left, bottom, right, bottom), fill=(55, 61, 70), width=2)

    for item in series:
        points = [
            (left + (x - x_low) / (x_high - x_low) * (right - left),
             bottom - (y - y_low) / (y_high - y_low) * (bottom - top))
            for x, y in zip(item.x, item.y)
        ]
        if len(points) > 2500:
            stride = math.ceil(len(points) / 2500)
            points = points[::stride]
        if len(points) >= 2:
            if item.dashed:
                _dashed_line(draw, points, item.color)
            else:
                draw.line(points, fill=item.color, width=4, joint="curve")

    title_width, _ = _text_size(draw, title, title_font)
    draw.text(((width - title_width) / 2, 25), title,
              fill=(28, 33, 41), font=title_font)
    x_width, _ = _text_size(draw, x_label, label_font)
    draw.text(((left + right - x_width) / 2, bottom + 58), x_label,
              fill=(35, 40, 48), font=label_font)
    draw.text((20, top - 42), y_label, fill=(35, 40, 48), font=label_font)

    legend = [item for item in series if item.label]
    if legend:
        widths = [_text_size(draw, item.label, legend_font)[0] + 72
                  for item in legend]
        cursor = (width - sum(widths) - 18 * (len(widths) - 1)) / 2
        for item, item_width in zip(legend, widths):
            line_y = 108
            if item.dashed:
                _dashed_line(draw, [(cursor, line_y), (cursor + 45, line_y)],
                             item.color, width=4, dash=8)
            else:
                draw.line((cursor, line_y, cursor + 45, line_y),
                          fill=item.color, width=4)
            draw.text((cursor + 55, 94), item.label, fill=(35, 40, 48),
                      font=legend_font)
            cursor += item_width + 18
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True)


def _plot_planar_trajectory(columns, output: Path, reference=None):
    series = []
    if reference is not None:
        series.append(_Series(reference[0], reference[1], GRAY, "Reference", True))
    series.append(_Series(columns["base_x_m"], columns["base_y_m"], BLUE, "Robot"))
    _save_plot(output / "planar_trajectory.png", "Planar trajectory",
               "World x (m)", "World y (m)", series, equal_scale=True)


def _plot_path_errors(columns, output: Path):
    time = columns["time_s"]
    _save_plot(output / "cross_track_error.png", "Cross-track error",
               "Time (s)", "Cross-track error (m)",
               [_Series(time, columns["path:cross_track_error_m"], BLUE)])
    _save_plot(output / "heading_error.png", "Heading error",
               "Time (s)", "Heading error (rad)",
               [_Series(time, columns["path:heading_error_rad"], ORANGE)])


def _plot_terrain_state(columns, output: Path):
    time = columns["time_s"]
    _save_plot(output / "base_height.png", "Base height", "Time (s)",
               "Base height (m)", [_Series(time, columns["base_z_m"], BLUE)])
    _save_plot(output / "base_attitude.png", "Base attitude", "Time (s)",
               "Angle (rad)", [
                   _Series(time, columns["roll_rad"], BLUE, "Roll"),
                   _Series(time, columns["pitch_rad"], ORANGE, "Pitch", True),
               ])
    actual = [
        math.cos(yaw) * vx + math.sin(yaw) * vy
        for yaw, vx, vy in zip(columns["yaw_rad"], columns["vx_world_m_s"],
                               columns["vy_world_m_s"])
    ]
    _save_plot(output / "forward_velocity.png", "Forward velocity", "Time (s)",
               "Body-frame velocity (m/s)", [
                   _Series(time, actual, BLUE, "Robot"),
                   _Series(time, columns["cmd_vx_body_m_s"], GRAY,
                           "Reference", True),
               ])


def analyze_path(run_directory: Path, output: Path, columns, run):
    _require(columns, PATH_FIELDS)
    reference = load_reference_path(run_directory / "reference_path.csv")
    common = _common_metrics(columns, run)
    walking = [index for index, value in enumerate(columns["walking_command"])
               if value >= 0.5]
    distances = [columns["path:path_distance_m"][index] for index in walking]
    headings = [columns["path:heading_error_rad"][index] for index in walking]
    last = len(columns["time_s"]) - 1
    reference_length = _path_length(reference[0], reference[1])
    recomputed = {
        "sample_count": len(walking),
        "cross_track_rmse_m": _rmse(distances) if distances else None,
        "maximum_path_distance_m": max(distances) if distances else None,
        "heading_error_rmse_rad": _rmse(headings) if headings else None,
        "final_progress_m": columns["path:progress_m"][last],
        "path_length_m": reference_length,
        "completion_fraction": columns["path:progress_fraction"][last],
        "reached_goal": bool(columns["path:reached_goal"][last] >= 0.5),
        "final_goal_distance_m": columns["path:goal_distance_m"][last],
    }
    reported = _load_json(run_directory / "path_metrics.json", required=False)
    run_config = run.get("config")
    path_config = (run_config.get("path_tracking")
                   if isinstance(run_config, dict) else None)
    metrics = {
        "analysis_kind": "path_tracking",
        "source_files": {
            "run": str(run_directory / "run.json"),
            "states": str(run_directory / "states.csv"),
            "reference_path": str(run_directory / "reference_path.csv"),
            "path_metrics": (str(run_directory / "path_metrics.json")
                             if reported is not None else None),
        },
        "path_mode": path_config.get("mode") if isinstance(path_config, dict) else None,
        **common,
        "path_tracking": recomputed,
        "reported_path_metrics": reported,
        "path_metrics_reconciliation": _reconcile(
            reported,
            recomputed,
            ("sample_count", "cross_track_rmse_m",
             "maximum_path_distance_m", "heading_error_rmse_rad",
             "path_length_m"),
        ),
        "final_metric_sampling_note": (
            "path_metrics.json 的 final_*、completion_fraction 与 reached_goal 来自最后一个 "
            "1 kHz 物理步；states.csv 最后一行来自 100 Hz 日志，因此不把这些终值做严格相等对账。"
        ),
        "figures": ["planar_trajectory.png", "cross_track_error.png",
                    "heading_error.png"],
    }
    _plot_planar_trajectory(columns, output, reference)
    _plot_path_errors(columns, output)
    return metrics


def _finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def _positive_number(value, name: str) -> float:
    result = _finite_number(value, name)
    if result <= 0:
        raise ValueError(f"{name} 必须是正数")
    return result


def _bounds_from_plan(plan, terrain_type: str):
    """Return task-region bounds from a serialized terrain plan when present."""
    if isinstance(plan, dict):
        direct_keys = ("terrain_x_start_m", "terrain_x_end_m",
                       "terrain_y_min_m", "terrain_y_max_m")
        if all(key in plan for key in direct_keys):
            return tuple(_finite_number(plan[key], f"terrain.plan.{key}")
                         for key in direct_keys)
        for key in ("items", "primitives", "boxes"):
            if isinstance(plan.get(key), list):
                plan = plan[key]
                break
    if not isinstance(plan, list):
        return None
    prefixes = {
        "slope": ("Slope",),
        "slope_course": ("Slope",),
        "rough_blocks": ("Rough_",),
        "random_obstacles": ("Obstacle",),
        "blind_step": ("Step",),
    }.get(terrain_type, ())
    selected = [item for item in plan
                if isinstance(item, dict)
                and isinstance(item.get("name"), str)
                and item["name"].startswith(prefixes)]
    if not selected:
        return None
    x_min, x_max, y_min, y_max = [], [], [], []
    for index, item in enumerate(selected):
        center = item.get("center_m")
        size = item.get("size_m")
        if not (isinstance(center, (list, tuple)) and len(center) >= 2
                and isinstance(size, (list, tuple)) and len(size) >= 2):
            raise ValueError(f"terrain.plan[{index}] 缺少 center_m/size_m")
        cx = _finite_number(center[0], f"terrain.plan[{index}].center_m[0]")
        cy = _finite_number(center[1], f"terrain.plan[{index}].center_m[1]")
        sx = _positive_number(size[0], f"terrain.plan[{index}].size_m[0]")
        sy = _positive_number(size[1], f"terrain.plan[{index}].size_m[1]")
        # Slope boxes are rotated about y.  The task region follows their
        # horizontal run along the surface centreline; thickness is excluded.
        rotation = abs(_finite_number(item.get("rotate_y_deg", 0.0),
                                      f"terrain.plan[{index}].rotate_y_deg"))
        horizontal_size = sx * abs(math.cos(math.radians(rotation)))
        x_min.append(cx - horizontal_size / 2.0)
        x_max.append(cx + horizontal_size / 2.0)
        y_min.append(cy - sy / 2.0)
        y_max.append(cy + sy / 2.0)
    return min(x_min), max(x_max), min(y_min), max(y_max)


def terrain_bounds(terrain_config: dict | None, terrain_manifest: dict | None = None):
    """Resolve the finite x/y task region, preferring ``terrain.plan``."""
    if not isinstance(terrain_config, dict):
        raise ValueError("run.json 缺少 terrain.config")
    terrain_type = terrain_config.get("type")
    plan = terrain_manifest.get("plan") if isinstance(terrain_manifest, dict) else None
    planned = _bounds_from_plan(plan, terrain_type) if plan is not None else None
    if planned is not None:
        x_start, x_end, y_min, y_max = planned
        source = "terrain.plan"
    elif terrain_type == "slope":
        x_start = _finite_number(terrain_config.get("start_x_m", 0.8),
                                 "terrain.config.start_x_m")
        length = _positive_number(terrain_config.get("length_m", 2.5),
                                  "terrain.config.length_m")
        angle = _finite_number(terrain_config.get("angle_deg"),
                               "terrain.config.angle_deg")
        x_end = x_start + length * math.cos(math.radians(angle))
        width = _positive_number(terrain_config.get("width_m", 2.0),
                                 "terrain.config.width_m")
        y_min, y_max, source = -width / 2.0, width / 2.0, "terrain.config"
    elif terrain_type == "slope_course":
        x_start = _finite_number(terrain_config.get("start_x_m", 0.8),
                                 "terrain.config.start_x_m")
        segments = terrain_config.get("segments")
        if not isinstance(segments, list) or not segments:
            raise ValueError("terrain.config.segments 至少需要一段")
        horizontal = 0.0
        for index, segment in enumerate(segments):
            if not isinstance(segment, dict):
                raise ValueError(f"terrain.config.segments[{index}] 必须是对象")
            length = _positive_number(segment.get("length_m"),
                                      f"terrain.config.segments[{index}].length_m")
            angle = _finite_number(segment.get("angle_deg"),
                                   f"terrain.config.segments[{index}].angle_deg")
            horizontal += length * math.cos(math.radians(angle))
        x_end = x_start + horizontal
        width = _positive_number(terrain_config.get("width_m", 2.0),
                                 "terrain.config.width_m")
        y_min, y_max, source = -width / 2.0, width / 2.0, "terrain.config"
    elif terrain_type == "rough_blocks":
        x_start = _finite_number(terrain_config.get("start_x_m", 0.8),
                                 "terrain.config.start_x_m")
        rows = terrain_config.get("rows", 16)
        columns = terrain_config.get("columns", 10)
        if isinstance(rows, bool) or not isinstance(rows, int) or rows < 1:
            raise ValueError("terrain.config.rows 必须是正整数")
        if isinstance(columns, bool) or not isinstance(columns, int) or columns < 1:
            raise ValueError("terrain.config.columns 必须是正整数")
        tile = _positive_number(terrain_config.get("tile_size_m", 0.2),
                                "terrain.config.tile_size_m")
        x_end = x_start + rows * tile
        y_min, y_max = -columns * tile / 2.0, columns * tile / 2.0
        source = "terrain.config"
    elif terrain_type == "random_obstacles":
        region_x = terrain_config.get("region_x_m", [0.8, 3.5])
        region_y = terrain_config.get("region_y_m", [-0.8, 0.8])
        if not (isinstance(region_x, (list, tuple)) and len(region_x) == 2
                and isinstance(region_y, (list, tuple)) and len(region_y) == 2):
            raise ValueError("terrain.config 的 random obstacle 区域必须是 [min,max]")
        x_start, x_end = (_finite_number(value, f"terrain.config.region_x_m[{i}]")
                          for i, value in enumerate(region_x))
        y_min, y_max = (_finite_number(value, f"terrain.config.region_y_m[{i}]")
                        for i, value in enumerate(region_y))
        source = "terrain.config"
    elif terrain_type == "blind_step":
        x_start = _finite_number(terrain_config.get("start_x_m", 0.9),
                                 "terrain.config.start_x_m")
        length = _positive_number(terrain_config.get("length_m", 4.0),
                                  "terrain.config.length_m")
        x_end = x_start + length
        width = _positive_number(terrain_config.get("width_m", 2.0),
                                 "terrain.config.width_m")
        y_min, y_max, source = -width / 2.0, width / 2.0, "terrain.config"
    elif terrain_type == "flat":
        return {
            "terrain_bounds_source": "terrain.config",
            "terrain_x_start_m": None, "terrain_x_end_m": None,
            "terrain_y_min_m": None, "terrain_y_max_m": None,
        }
    else:
        raise ValueError(f"不支持的 terrain.config.type：{terrain_type!r}")
    if not x_end > x_start:
        raise ValueError("地形 x 终点必须大于起点")
    if not y_max > y_min:
        raise ValueError("地形 y 上界必须大于下界")
    return {
        "terrain_bounds_source": source,
        "terrain_x_start_m": x_start,
        "terrain_x_end_m": x_end,
        "terrain_y_min_m": y_min,
        "terrain_y_max_m": y_max,
    }


def terrain_traversal_metrics(columns, run_status: str, bounds: dict):
    """Measure entry and traversal using base position within terrain width."""
    x_start = bounds["terrain_x_start_m"]
    x_end = bounds["terrain_x_end_m"]
    y_min = bounds["terrain_y_min_m"]
    y_max = bounds["terrain_y_max_m"]
    if None in (x_start, x_end, y_min, y_max):
        return {
            "maximum_base_x_m": None, "entry_time_s": None,
            "exit_time_s": None, "samples_inside": 0,
            "forward_progress_on_terrain_m": 0.0, "progress_fraction": 0.0,
            "entered_terrain": False, "traversed_terrain": False,
            "completion_reason": "运行失败" if run_status != "completed" else "未进入",
        }
    time = columns["time_s"]
    xs, ys = columns["base_x_m"], columns["base_y_m"]
    within_width = [index for index, y in enumerate(ys) if y_min <= y <= y_max]
    maximum_x = max((xs[index] for index in within_width), default=None)
    inside = [index for index in within_width if x_start <= xs[index] <= x_end]
    completed = run_status == "completed"
    observed_entry = next((index for index in within_width if xs[index] >= x_start), None)
    observed_exit = next((index for index in within_width
                          if observed_entry is not None and index >= observed_entry
                          and xs[index] >= x_end), None)
    entered = completed and observed_entry is not None
    traversed = completed and observed_exit is not None
    length = x_end - x_start
    progress = (0.0 if maximum_x is None
                else min(length, max(0.0, maximum_x - x_start)))
    if not completed:
        reason = "运行失败"
    elif not entered:
        reason = "未进入"
    elif traversed:
        reason = "穿越"
    else:
        reason = "进入未穿越"
    return {
        "maximum_base_x_m": maximum_x,
        "entry_time_s": time[observed_entry] if entered else None,
        "exit_time_s": time[observed_exit] if traversed else None,
        "samples_inside": len(inside),
        "forward_progress_on_terrain_m": progress,
        "progress_fraction": progress / length,
        "entered_terrain": entered,
        "traversed_terrain": traversed,
        "completion_reason": reason,
    }


def blind_step_front_metrics(terrain_type: str, run_status: str,
                             bounds: dict, traversal: dict) -> dict:
    """Return a clearly labelled base-only proxy for clearing a step front."""
    if terrain_type != "blind_step":
        return {
            "step_front_clearance_m": None,
            "step_front_crossed_proxy": None,
            "step_front_proxy_note": None,
        }
    start = bounds["terrain_x_start_m"]
    maximum_x = traversal["maximum_base_x_m"]
    clearance = (None if start is None or maximum_x is None
                 else maximum_x - start)
    crossed = bool(
        run_status == "completed"
        and clearance is not None
        and clearance >= STEP_FRONT_CLEARANCE_THRESHOLD_M
    )
    return {
        "step_front_clearance_m": clearance,
        "step_front_crossed_proxy": crossed,
        "step_front_proxy_note": STEP_FRONT_PROXY_NOTE,
    }


def analyze_terrain(run_directory: Path, output: Path, columns, run):
    common = _common_metrics(columns, run)
    terrain_manifest = run.get("terrain")
    terrain_config = None
    if isinstance(terrain_manifest, dict):
        candidate = terrain_manifest.get("config")
        if isinstance(candidate, dict):
            terrain_config = candidate
    if terrain_config is None:
        run_config = run.get("config")
        candidate = run_config.get("terrain") if isinstance(run_config, dict) else None
        if isinstance(candidate, dict):
            terrain_config = candidate
    bounds = terrain_bounds(terrain_config, terrain_manifest)
    traversal = terrain_traversal_metrics(columns, run.get("status"), bounds)
    step_front = blind_step_front_metrics(
        terrain_config.get("type"), run.get("status"), bounds, traversal)
    metrics = {
        "analysis_kind": "terrain",
        "source_files": {
            "run": str(run_directory / "run.json"),
            "states": str(run_directory / "states.csv"),
        },
        "terrain_type": terrain_config.get("type") if terrain_config else None,
        "terrain_config": terrain_config,
        "terrain_manifest": terrain_manifest,
        **bounds,
        **traversal,
        **step_front,
        **common,
        "figures": ["planar_trajectory.png", "base_height.png",
                    "base_attitude.png", "forward_velocity.png"],
    }
    _plot_planar_trajectory(columns, output)
    _plot_terrain_state(columns, output)
    return metrics


TERRAIN_SUMMARY_FIELDS = (
    "experiment", "config_relative_path", "output_relative_path",
    "run_status", "terrain_type", "terrain_bounds_source",
    "terrain_x_start_m", "terrain_x_end_m", "terrain_y_min_m",
    "terrain_y_max_m", "maximum_base_x_m", "entry_time_s", "exit_time_s",
    "samples_inside", "forward_progress_on_terrain_m", "progress_fraction",
    "entered_terrain", "traversed_terrain", "step_front_clearance_m",
    "step_front_crossed_proxy", "step_front_proxy_note", "completion_reason",
)


def _project_relative(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(PROJECT_ROOT.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError(f"路径不在项目根目录内：{path}") from exc


def terrain_summary_row(run_directory: Path, metrics: dict) -> dict:
    run = _load_json(run_directory / "run.json")
    run_config = run.get("config")
    config_path = run_config.get("config") if isinstance(run_config, dict) else None
    if not isinstance(config_path, str) or not config_path.startswith("Pro2/"):
        raise ValueError(f"{run_directory} 的配置路径不是项目根目录相对路径")
    row = {
        "experiment": run_directory.name,
        "config_relative_path": config_path,
        "output_relative_path": _project_relative(run_directory),
        "run_status": metrics["run_status"],
        "terrain_type": metrics["terrain_type"],
    }
    for field in TERRAIN_SUMMARY_FIELDS[5:]:
        row[field] = metrics[field]
    return row


def update_terrain_summary(run_directory: Path, metrics: dict,
                           summary_directory: Path | None = None):
    """Insert or replace one experiment in the appendable terrain summary."""
    run_directory = Path(run_directory)
    summary_directory = (Path(summary_directory) if summary_directory is not None
                         else run_directory.parent)
    summary_directory.mkdir(parents=True, exist_ok=True)
    json_path = summary_directory / "terrain_summary.json"
    csv_path = summary_directory / "terrain_summary.csv"
    existing = _load_json(json_path, required=False)
    rows = []
    if existing is not None:
        candidate = existing.get("experiments")
        if not isinstance(candidate, list):
            raise ValueError(f"{json_path} 缺少 experiments 数组")
        rows = candidate
    for item in rows:
        if not isinstance(item, dict):
            raise ValueError(f"{json_path} 的 experiments 必须只含对象")
        item.setdefault("step_front_clearance_m", None)
        item.setdefault("step_front_crossed_proxy", None)
        item.setdefault("step_front_proxy_note", None)
    row = terrain_summary_row(run_directory, metrics)
    by_name = {item.get("experiment"): item for item in rows
               if isinstance(item, dict) and isinstance(item.get("experiment"), str)}
    order = [item["experiment"] for item in rows
             if isinstance(item, dict) and item.get("experiment") in by_name]
    if row["experiment"] not in by_name:
        order.append(row["experiment"])
    by_name[row["experiment"]] = row
    rows = [by_name[name] for name in order]
    summary = {
        "schema_version": 2,
        "metric_source": "各实验 analysis/metrics.json",
        "traversal_rule": (
            "run.status 必须为 completed；基座 y 必须位于地形宽度内；"
            "x 到达起点记为进入，到达终点记为穿越。"
        ),
        "step_front_proxy_rule": STEP_FRONT_PROXY_NOTE,
        "experiment_count": len(rows),
        "traversed_count": sum(bool(item["traversed_terrain"]) for item in rows),
        "experiments": rows,
    }
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=TERRAIN_SUMMARY_FIELDS)
        writer.writeheader()
        for item in rows:
            writer.writerow({
                key: (str(item[key]).lower() if isinstance(item[key], bool)
                      else item[key] if item[key] is not None else "")
                for key in TERRAIN_SUMMARY_FIELDS
            })
    return json_path, csv_path


def analyze_run(run_directory: Path, output: Path | None = None,
                kind: str = "auto") -> dict:
    run_directory = Path(run_directory)
    output = Path(output) if output is not None else run_directory / "analysis"
    run = _load_json(run_directory / "run.json")
    columns = load_columns(run_directory / "states.csv")
    if kind == "auto":
        kind = ("path" if (run_directory / "reference_path.csv").is_file()
                or any(name.startswith("path:") for name in columns)
                else "terrain")
    if kind not in {"path", "terrain"}:
        raise ValueError("kind 必须是 auto、path 或 terrain")
    output.mkdir(parents=True, exist_ok=True)
    metrics = (analyze_path(run_directory, output, columns, run)
               if kind == "path"
               else analyze_terrain(run_directory, output, columns, run))
    (output / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path,
                        help="包含 run.json 与 states.csv 的单次实验目录")
    parser.add_argument("--kind", choices=("auto", "path", "terrain"),
                        default="auto")
    parser.add_argument("--output", type=Path,
                        help="默认写入 <run_directory>/analysis/")
    parser.add_argument("--update-terrain-summary", action="store_true",
                        help="将本次地形结果追加或更新到同级 terrain_summary.json/csv")
    args = parser.parse_args(argv)
    metrics = analyze_run(args.run_directory, args.output, args.kind)
    summary_paths = None
    if args.update_terrain_summary:
        if metrics["analysis_kind"] != "terrain":
            parser.error("--update-terrain-summary 只能用于地形实验")
        summary_paths = update_terrain_summary(args.run_directory, metrics)
    print(json.dumps({
        "analysis_kind": metrics["analysis_kind"],
        "run_status": metrics["run_status"],
        "logged_samples": metrics["logged_samples"],
        "output": str(args.output or args.run_directory / "analysis"),
        "terrain_summary": ([str(path) for path in summary_paths]
                            if summary_paths is not None else None),
    }, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
