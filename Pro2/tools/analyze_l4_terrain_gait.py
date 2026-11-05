"""Recompute L4 terrain gait/stability evidence from the four original logs.

Geometry is used only to align a transition window. Region traversal is not a
stability criterion. All plots contain only titles, axes and necessary legends.
Run from the project root with ``python3 Pro2/tools/analyze_l4_terrain_gait.py``.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

from PIL import Image

from analyze_l4_path_terrain import (
    BLUE, ORANGE, GRAY, _Series, _save_plot, load_columns, terrain_bounds,
)


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "Pro2/output/L4/03_terrain_and_jump"
CASES = (
    ("slope_05deg", "5°坡道"),
    ("rough_medium_run02", "粗糙地面"),
    ("random_obstacles_seed01_full", "随机障碍"),
    ("blind_step_up_run02", "6 cm 台阶"),
)
COLORS = (BLUE, ORANGE, (0, 130, 108), (149, 78, 163))
KNEE_LEFT = "q_rad:J_knee_l_pitch"
KNEE_RIGHT = "q_rad:J_knee_r_pitch"


def rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def mean(values):
    return statistics.fmean(values)


def rms(values):
    return math.sqrt(mean([v * v for v in values]))


def corr(left, right):
    a, b = mean(left), mean(right)
    numerator = sum((x - a) * (y - b) for x, y in zip(left, right))
    denominator = math.sqrt(sum((x - a) ** 2 for x in left) * sum((y - b) ** 2 for y in right))
    return numerator / denominator if denominator else None


def measure(c, indices, events):
    if not indices:
        raise ValueError("Empty analysis window")
    def vals(key):
        return [c[key][i] for i in indices]
    t = vals("time_s")
    roll, pitch = vals("roll_deg"), vals("pitch_deg")
    tilt_axis = [max(abs(r), abs(p)) for r, p in zip(roll, pitch)]
    tilt_peak_index = max(range(len(t)), key=lambda i: tilt_axis[i])
    left, right = vals("knee_left_deg"), vals("knee_right_deg")
    index_set = set(indices)
    selected_events = [e for e in events if e["sample_index"] in index_set]
    intervals = [b["time_s"] - a["time_s"] for a, b in zip(selected_events, selected_events[1:])]
    # A cycle here is a return to the same controller support state, NOT a
    # measured heel-strike/foot-contact cycle.
    cycles = []
    for a, b in zip(selected_events, selected_events[2:]):
        if a["to_leg_state"] == b["to_leg_state"]:
            cycles.append(b["time_s"] - a["time_s"])
    schedule_states = sorted(set(int(c["controller:leg_state"][i]) for i in indices))
    return {
        "first_sample_s": t[0], "last_sample_s": t[-1], "samples": len(indices),
        "sample_span_s": t[-1] - t[0],
        "roll_rms_deg": rms(roll), "pitch_rms_deg": rms(pitch),
        "tilt_combined_rms_deg": math.sqrt(mean([r * r + p * p for r, p in zip(roll, pitch)])),
        "max_abs_roll_deg": max(map(abs, roll)), "max_abs_pitch_deg": max(map(abs, pitch)),
        "max_abs_tilt_axis_deg": tilt_axis[tilt_peak_index],
        "max_abs_tilt_axis_time_s": t[tilt_peak_index],
        "roll_std_deg": statistics.pstdev(roll), "pitch_std_deg": statistics.pstdev(pitch),
        "forward_velocity_rmse_m_s": rms(vals("vx_error_m_s")),
        "lateral_velocity_rmse_m_s": rms(vals("vy_error_m_s")),
        "forward_velocity_mean_m_s": mean(vals("vx_body_m_s")),
        "forward_velocity_std_m_s": statistics.pstdev(vals("vx_body_m_s")),
        "command_vx_mean_m_s": mean(vals("cmd_vx_body_m_s")),
        "base_height_min_m": min(vals("base_z_m")), "base_height_max_m": max(vals("base_z_m")),
        "knee_left_min_deg": min(left), "knee_left_max_deg": max(left),
        "knee_right_min_deg": min(right), "knee_right_max_deg": max(right),
        "knee_left_range_deg": max(left) - min(left),
        "knee_right_range_deg": max(right) - min(right),
        "knee_zero_lag_correlation": corr(left, right),
        "controller_support_states_observed": schedule_states,
        "controller_support_switch_count": len(selected_events),
        "controller_support_switch_interval_count": len(intervals),
        "controller_support_switch_interval_mean_s": mean(intervals) if intervals else None,
        "controller_support_switch_interval_std_s": statistics.pstdev(intervals) if intervals else None,
        "support_switch_interval_cv_percent": statistics.pstdev(intervals) / mean(intervals) * 100 if intervals else None,
        "controller_support_switch_interval_min_s": min(intervals) if intervals else None,
        "controller_support_switch_interval_max_s": max(intervals) if intervals else None,
        "controller_support_cycle_mean_s": mean(cycles) if cycles else None,
        "controller_support_switch_events": selected_events,
        "mpc_nonzero_qp_status_samples": sum(c["controller:mpc_qp_status"][i] != 0 for i in indices),
        "wbc_nonzero_qp_status_samples": sum(c["controller:wbc_qp_status"][i] != 0 for i in indices),
    }


def make_window(c, start, end, events):
    # Numerical tolerance makes decimal boundaries stable; the interval remains
    # half open [start, end), yielding 1000 samples for a 10-s / 100-Hz window.
    indices = [i for i, t in enumerate(c["time_s"]) if start - 1e-8 <= t < end - 1e-8]
    metrics = measure(c, indices, events)
    metrics.update({"requested_start_s": start, "requested_end_s": end,
                    "requested_duration_s": end - start, "interval": "[start, end)"})
    return indices, metrics


def graph(c, indices, entry, path, title, fields, unit, labels):
    x = [c["time_s"][i] - entry for i in indices]
    series = [_Series(x, [c[field][i] for i in indices], color, label)
              for field, label, color in zip(fields, labels, COLORS)]
    low = min(min(item.y) for item in series)
    high = max(max(item.y) for item in series)
    if min(x) <= 0 <= max(x):
        series.append(_Series([0, 0], [low, high], GRAY, "基座入区参考", True))
    _save_plot(path, title, "Time from base entry (s)", unit, series)


def analyze_case(case, label, half_window):
    directory = BASE / case
    run_path, config_path, state_path = (directory / f for f in ("run.json", "config.json", "states.csv"))
    run = json.loads(run_path.read_text(encoding="utf-8"))
    config = json.loads(config_path.read_text(encoding="utf-8"))
    c = load_columns(state_path)
    needed = ["time_s", "roll_rad", "pitch_rad", "yaw_rad", "vx_world_m_s", "vy_world_m_s",
              "cmd_vx_body_m_s", "cmd_vy_body_m_s", "walking_command", KNEE_LEFT, KNEE_RIGHT,
              "controller:leg_state", "controller:mpc_qp_status", "controller:wbc_qp_status"]
    missing = set(needed) - set(c)
    if missing:
        raise ValueError(f"{case}: missing {sorted(missing)}")
    t = c["time_s"]
    intervals = [b - a for a, b in zip(t, t[1:])]
    dt_log = statistics.median(intervals)
    if min(intervals) <= 0 or max(abs(v - dt_log) for v in intervals) > 1e-7:
        raise ValueError(f"{case}: timestamps are not uniform/increasing")
    bounds = terrain_bounds(config["terrain"], run.get("terrain"))
    x0, x1 = bounds["terrain_x_start_m"], bounds["terrain_x_end_m"]
    y0, y1 = bounds["terrain_y_min_m"], bounds["terrain_y_max_m"]
    entry_index = next(i for i, (x, y) in enumerate(zip(c["base_x_m"], c["base_y_m"])) if x >= x0 and y0 <= y <= y1)
    entry = t[entry_index]
    c["roll_deg"] = list(map(math.degrees, c["roll_rad"]))
    c["pitch_deg"] = list(map(math.degrees, c["pitch_rad"]))
    c["knee_left_deg"] = list(map(math.degrees, c[KNEE_LEFT]))
    c["knee_right_deg"] = list(map(math.degrees, c[KNEE_RIGHT]))
    c["tilt_combined_deg"] = [math.hypot(r, p) for r, p in zip(c["roll_deg"], c["pitch_deg"])]
    c["vx_body_m_s"] = [math.cos(yaw) * vx + math.sin(yaw) * vy for yaw, vx, vy in zip(c["yaw_rad"], c["vx_world_m_s"], c["vy_world_m_s"])]
    c["vy_body_m_s"] = [-math.sin(yaw) * vx + math.cos(yaw) * vy for yaw, vx, vy in zip(c["yaw_rad"], c["vx_world_m_s"], c["vy_world_m_s"])]
    c["vx_error_m_s"] = [v - ref for v, ref in zip(c["vx_body_m_s"], c["cmd_vx_body_m_s"])]
    c["vy_error_m_s"] = [v - ref for v, ref in zip(c["vy_body_m_s"], c["cmd_vy_body_m_s"])]
    events = [{"time_s": t[i], "sample_index": i,
               "from_leg_state": int(c["controller:leg_state"][i - 1]),
               "to_leg_state": int(c["controller:leg_state"][i])}
              for i in range(1, len(t)) if c["controller:leg_state"][i - 1] != c["controller:leg_state"][i]]
    transition_i, transition = make_window(c, entry - half_window, entry + half_window, events)
    if len(transition_i) != round(half_window * 2 / dt_log):
        raise ValueError(f"{case}: transition window incomplete")
    primary_i, primary = make_window(c, 7.0, config["duration"], events)
    walking_i, walking = make_window(c, config["walk_start"], config["duration"], events)
    steady_i, steady = make_window(c, 7.0, 24.0, events)
    inside_i = [i for i, (x, y) in enumerate(zip(c["base_x_m"], c["base_y_m"])) if x0 <= x <= x1 and y0 <= y <= y1]
    inside = measure(c, inside_i, events)
    inside["definition"] = "Samples whose base origin lies inside the terrain x/y bounds; not measured foot contact."
    failure_path = directory / "failure.json"
    failure = json.loads(failure_path.read_text(encoding="utf-8")) if failure_path.exists() else None
    duration = run.get("simulation_duration_s", run["steps"] * config["dt"])
    normal_end = run.get("status") == "completed" and math.isclose(duration, config["duration"], abs_tol=config["dt"]) and run["steps"] == round(config["duration"] / config["dt"]) and not failure_path.exists()
    output = directory / "analysis_gait"
    output.mkdir(parents=True, exist_ok=True)
    figures = []
    for suffix, ii in (("full", walking_i), ("transition", transition_i)):
        for name, title, fields, unit, labels in (
            ("base_attitude", "机身姿态", ["roll_deg", "pitch_deg"], "Angle (deg)", ["Roll", "Pitch"]),
            ("knee_angles", "左右膝关节实际角度", ["knee_left_deg", "knee_right_deg"], "Angle (deg)", ["左膝", "右膝"]),
            ("forward_velocity", "前向速度", ["vx_body_m_s", "cmd_vx_body_m_s"], "Velocity (m/s)", ["机体系实际速度", "指令速度"]),
        ):
            path = output / f"{name}_{suffix}.png"
            graph(c, ii, entry, path, label + "｜" + title, fields, unit, labels)
            figures.append(rel(path))
    metrics = {
        "schema": "l4_terrain_gait_stability_v1", "experiment": case, "label": label,
        "metrics_path": rel(output / "metrics.json"), "primary_window_key": "post_ramp_walk",
        "source_files": {"run": rel(run_path), "config": rel(config_path), "states": rel(state_path)},
        "source_sha256": {rel(p): sha256(p) for p in (run_path, config_path, state_path)},
        "conditions": {k: config[k] for k in ("duration", "dt", "walk_start", "vx", "step_height", "swing_time")},
        "entry_alignment": {"base_entry_time_s": entry, "sample_index": entry_index,
                            "definition": "First logged base origin x >= terrain start while y lies in the terrain bounds; a geometric alignment proxy, not first foot contact.", **bounds},
        "sampling": {"logged_samples": len(t), "logging_interval_s": dt_log, "all_numeric_fields_finite": True},
        "run_end": {"run_status": run.get("status"), "requested_duration_s": config["duration"],
                    "simulation_duration_s": duration, "physics_steps": run["steps"],
                    "failure_file_present": failure_path.exists(), "run_error": run.get("error"),
                    "normal_duration_end": normal_end,
                    "instability_stop_triggered": (False if normal_end else
                        True if failure and failure.get("failure_kind") == "physical_instability" else None),
                    "scope": "Finite observed run only; duration completion alone is not a stability proof."},
        "windows": {"post_ramp_walk": primary, "full_walking": walking,
                    "transition_around_entry": transition,
                    "common_steady_7_to_24": steady, "terrain_inside": inside},
        "figures": figures,
    }
    write_json(output / "metrics.json", metrics)
    return metrics, c, transition_i


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--half-window", type=float, default=5.0)
    args = parser.parse_args()
    if args.half_window <= 0:
        parser.error("--half-window must be positive")
    summary_dir = BASE / "gait_stability_analysis"
    summary_dir.mkdir(parents=True, exist_ok=True)
    all_results = [analyze_case(case, label, args.half_window) for case, label in CASES]
    figures = []
    for field, filename, title, unit in (
        ("tilt_combined_deg", "transition_tilt.png", "四地形过渡段机身倾斜", "Tilt magnitude (deg)"),
        ("vx_error_m_s", "transition_velocity_error.png", "四地形过渡段前向速度误差", "Velocity error (m/s)"),
    ):
        series = []
        for (metrics, c, indices), color in zip(all_results, COLORS):
            entry = metrics["entry_alignment"]["base_entry_time_s"]
            series.append(_Series([c["time_s"][i] - entry for i in indices], [c[field][i] for i in indices], color, metrics["label"]))
        path = summary_dir / filename
        _save_plot(path, title, "Time from base entry (s)", unit, series)
        figures.append(rel(path))
    # Preserve individual axes/titles; no metrics or commentary are placed in
    # the image. These montage panels are useful alongside the report table.
    for suffix in ("full", "transition"):
        for name in ("base_attitude", "knee_angles", "forward_velocity"):
            panel = Image.new("RGB", (3000, 1800), "white")
            for i, (case, _) in enumerate(CASES):
                with Image.open(BASE / case / "analysis_gait" / f"{name}_{suffix}.png") as source:
                    panel.paste(source, ((i % 2) * 1500, (i // 2) * 900))
            path = summary_dir / f"{name}_{suffix}_four_panels.png"
            panel.save(path, optimize=True)
            figures.append(rel(path))
    methods = {
        "primary_window": "post_ramp_walk: [7 s, requested run end), all available walking after the velocity command ramp. Observations last 17/21/38/17 s; duration differs by original run.",
        "primary_window_reason": "Retain terrain transitions and later obstacles instead of selecting a local window around a favourable event. The four original runs are descriptive scenario observations, not a strict terrain-difficulty ranking.",
        "transition_window": f"[base-entry - {args.half_window:g} s, base-entry + {args.half_window:g} s), supplementary common-duration transition analysis; no contact-time claim.",
        "tilt_combined_rms_deg": "sqrt(mean(roll_deg^2 + pitch_deg^2)); actual base orientation relative to world horizontal, including mean lean.",
        "max_abs_tilt_axis_deg": "max over samples of max(abs(roll_deg), abs(pitch_deg)).",
        "velocity": "vx_body = cos(yaw)*vx_world + sin(yaw)*vy_world; RMSE relative to logged cmd_vx_body_m_s. Horizontal heading frame, not full 3D body-frame rotation.",
        "knee_angles": "PhysX actual joint positions from q_rad:J_knee_l_pitch and q_rad:J_knee_r_pitch, converted to degrees. Range/correlation describe actual joint movement, not foot clearance or contact.",
        "controller_support_switch": "controller:leg_state: 0 left support, 1 right support, 2 double support. Switching uses estimated force and phase. This is controller scheduling, not measured foot contact, measured stance time, or actual step length. CV = 100 * population standard deviation / mean of intervals between consecutive switches wholly observed within the window.",
        "sampling": "Metrics use 100-Hz states.csv; support-switch timings have one sample (0.01 s) resolution. Raw physics peaks can exceed sampled peaks.",
        "run_end": "status completed, physics step count and elapsed simulation duration must match requested duration, with no failure.json. This distinguishes duration end from a recorded instability stop.",
        "comparison_scope": "Four scenario-specific configurations differ in speed and step height. Descriptive gait/stability diagnosis only; not a controlled causal ranking of terrain difficulty. No repeated-trial reliability estimate.",
        "geometric_scope": "No region-traversal pass/fail criterion. Base-entry time merely aligns the curves. Sparse obstacles are not verified foot-by-foot because these original logs lack contact prim identifiers and measured foot-contact flags.",
    }
    metrics_list = [item[0] for item in all_results]
    summary = {"schema": "l4_terrain_gait_stability_summary_v1", "primary_window_key": "post_ramp_walk", "analysis_script": rel(Path(__file__)),
               "analysis_script_sha256": sha256(Path(__file__)), "methods": methods,
               "experiments": metrics_list, "figures": figures,
               "source_sha256": {p: value for m in metrics_list for p, value in m["source_sha256"].items()}}
    write_json(summary_dir / "summary.json", summary)
    rows = []
    for m in metrics_list:
        p = m["windows"]["post_ramp_walk"]
        rows.append({"experiment": m["experiment"], "label": m["label"], "entry_s": m["entry_alignment"]["base_entry_time_s"],
                     "window_start_s": p["requested_start_s"], "window_end_s": p["requested_end_s"],
                     "samples": p["samples"], "command_vx_m_s": m["conditions"]["vx"],
                     "step_height_m": m["conditions"]["step_height"],
                     **{k: p[k] for k in ("tilt_combined_rms_deg", "max_abs_tilt_axis_deg", "forward_velocity_rmse_m_s", "knee_left_range_deg", "knee_right_range_deg", "controller_support_switch_interval_mean_s", "controller_support_switch_interval_min_s", "controller_support_switch_interval_max_s", "support_switch_interval_cv_percent")},
                     "normal_duration_end": m["run_end"]["normal_duration_end"]})
    with (summary_dir / "summary.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({"summary": rel(summary_dir / "summary.json"), "cases": rows}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
