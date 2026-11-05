"""Generate traceable plots and metrics for one formal H1 FixStand run.

The analyzer only reads ``run.json``, ``metrics.json`` and ``states.csv`` from
the selected run directory.  Derived files are written to a separate
``analysis/`` directory so the simulator's original evidence remains intact.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

try:
    from .analyze_l4_path_terrain import BLUE, GRAY, ORANGE, _Series, _save_plot
except ImportError:  # direct ``python3 Pro2/tools/...`` execution
    from analyze_l4_path_terrain import BLUE, GRAY, ORANGE, _Series, _save_plot


PREFIXES = {
    "q_nominal": "q_nominal_rad:",
    "q_desired": "q_des_rad:",
    "q_actual": "q_rad:",
    "dq": "dq_rad_s:",
    "balance": "q_balance_correction_rad:",
    "tau_pd": "tau_pd_est_nm:",
    "tau_projected": "tau_projected_nm:",
}
PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _project_relative(path: Path) -> str:
    """Return a stable project-root-relative path when possible."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(PROJECT_ROOT).as_posix()
    except ValueError:
        pass
    return resolved.as_posix()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON 顶层必须是对象：{path}")
    return value


def _load_csv(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f"CSV 没有表头：{path}")
        rows: list[dict[str, float]] = []
        for line_number, raw in enumerate(reader, 2):
            if None in raw:
                raise ValueError(f"{path}:{line_number} 的列数超过表头")
            parsed: dict[str, float] = {}
            for name in reader.fieldnames:
                try:
                    number = float(raw[name])
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"{path}:{line_number} 的 {name} 不是数值") from exc
                if not math.isfinite(number):
                    raise ValueError(
                        f"{path}:{line_number} 的 {name} 不是有限数值")
                parsed[name] = number
            rows.append(parsed)
    if not rows:
        raise ValueError(f"CSV 没有数据行：{path}")
    return list(reader.fieldnames), rows


def _columns_by_joint(fieldnames: list[str], prefix: str) -> dict[str, str]:
    return {
        name.removeprefix(prefix): name
        for name in fieldnames
        if name.startswith(prefix)
    }


def _rms(values: list[float]) -> float:
    if not values:
        raise ValueError("不能计算空序列的 RMS")
    return math.sqrt(sum(value * value for value in values) / len(values))


def analyze(run_directory: Path, output: Path | None = None) -> dict:
    run_directory = run_directory.resolve()
    output = (output or run_directory / "analysis").resolve()
    source_paths = {
        "run_json": run_directory / "run.json",
        "runtime_metrics_json": run_directory / "metrics.json",
        "states_csv": run_directory / "states.csv",
    }
    missing = [str(path) for path in source_paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError("缺少 H1 正式运行文件：" + ", ".join(missing))

    run = _load_json(source_paths["run_json"])
    runtime_metrics = _load_json(source_paths["runtime_metrics_json"])
    fieldnames, rows = _load_csv(source_paths["states_csv"])
    required_scalars = {
        "time_s", "base_x_m", "base_y_m", "base_z_m",
        "roll_rad", "pitch_rad",
    }
    missing_columns = sorted(required_scalars.difference(fieldnames))
    if missing_columns:
        raise KeyError("states.csv 缺少列：" + ", ".join(missing_columns))

    groups = {
        key: _columns_by_joint(fieldnames, prefix)
        for key, prefix in PREFIXES.items()
    }
    joint_names = list(run.get("joint_names", []))
    if not joint_names:
        raise ValueError("run.json 缺少 joint_names")
    for group_name, mapping in groups.items():
        absent = [joint for joint in joint_names if joint not in mapping]
        if absent:
            raise KeyError(
                f"states.csv 的 {group_name} 列缺少关节：" + ", ".join(absent))

    time = [row["time_s"] for row in rows]
    if any(b <= a for a, b in zip(time, time[1:])):
        raise ValueError("states.csv 的 time_s 必须严格递增")
    transition_time = float(runtime_metrics["transition_time_s"])
    hold_rows = [row for row in rows if row["time_s"] >= transition_time]
    if not hold_rows:
        raise ValueError("states.csv 没有 FixStand 保持阶段样本")

    origin_x = rows[0]["base_x_m"]
    origin_y = rows[0]["base_y_m"]

    def phase_metrics(selected: list[dict[str, float]]) -> dict:
        heights = [row["base_z_m"] for row in selected]
        tracking_errors = [
            row[groups["q_actual"][joint]] - row[groups["q_desired"][joint]]
            for row in selected for joint in joint_names
        ]
        nominal_errors = [
            row[groups["q_actual"][joint]] - row[groups["q_nominal"][joint]]
            for row in selected for joint in joint_names
        ]
        return {
            "sample_count": len(selected),
            "time_start_s": selected[0]["time_s"],
            "time_end_s": selected[-1]["time_s"],
            "minimum_base_height_m": min(heights),
            "maximum_base_height_m": max(heights),
            "mean_base_height_m": statistics.fmean(heights),
            "base_height_population_std_m": statistics.pstdev(heights),
            "maximum_abs_roll_or_pitch_rad": max(
                max(abs(row["roll_rad"]), abs(row["pitch_rad"]))
                for row in selected),
            "maximum_horizontal_drift_from_initial_m": max(
                math.hypot(row["base_x_m"] - origin_x,
                           row["base_y_m"] - origin_y)
                for row in selected),
            "joint_tracking_rmse_to_commanded_target_rad": _rms(tracking_errors),
            "joint_tracking_rmse_to_nominal_target_rad": _rms(nominal_errors),
            "maximum_abs_balance_correction_rad": max(
                abs(row[groups["balance"][joint]])
                for row in selected for joint in joint_names),
            "maximum_abs_sampled_pd_effort_estimate_nm": max(
                abs(row[groups["tau_pd"][joint]])
                for row in selected for joint in joint_names),
            "maximum_abs_sampled_projected_joint_force_nm": max(
                abs(row[groups["tau_projected"][joint]])
                for row in selected for joint in joint_names),
            "maximum_abs_sampled_joint_velocity_rad_s": max(
                abs(row[groups["dq"][joint]])
                for row in selected for joint in joint_names),
        }

    final = rows[-1]
    targets = run.get("position_targets_rad")
    if not isinstance(targets, list) or len(targets) != len(joint_names):
        raise ValueError("run.json 的 position_targets_rad 与 joint_names 维数不一致")
    final_nominal_errors = [
        final[groups["q_actual"][joint]] - float(target)
        for joint, target in zip(joint_names, targets)
    ]
    final_commanded_errors = [
        final[groups["q_actual"][joint]]
        - final[groups["q_desired"][joint]]
        for joint in joint_names
    ]
    sampled_metrics = {
        "all_logged_samples": phase_metrics(rows),
        "fixstand_hold_samples": phase_metrics(hold_rows),
        "last_logged_sample": {
            "time_s": final["time_s"],
            "base_position_m": [
                final["base_x_m"], final["base_y_m"], final["base_z_m"]],
            "roll_rad": final["roll_rad"],
            "pitch_rad": final["pitch_rad"],
            "joint_rmse_to_final_nominal_target_rad": _rms(final_nominal_errors),
            "joint_rmse_to_commanded_target_rad": _rms(final_commanded_errors),
        },
    }

    output.mkdir(parents=True, exist_ok=True)
    heights = [row["base_z_m"] for row in rows]
    rolls = [row["roll_rad"] for row in rows]
    pitches = [row["pitch_rad"] for row in rows]
    tracking_rmse = []
    max_pd = []
    max_projected = []
    balance_pitch = []
    balance_roll = []
    left_ankle_pitch = "left_ankle_pitch_joint"
    left_ankle_roll = "left_ankle_roll_joint"
    for row in rows:
        tracking_rmse.append(_rms([
            row[groups["q_actual"][joint]]
            - row[groups["q_desired"][joint]]
            for joint in joint_names
        ]))
        max_pd.append(max(
            abs(row[groups["tau_pd"][joint]]) for joint in joint_names))
        max_projected.append(max(
            abs(row[groups["tau_projected"][joint]])
            for joint in joint_names))
        balance_pitch.append(
            row[groups["balance"][left_ankle_pitch]])
        balance_roll.append(row[groups["balance"][left_ankle_roll]])

    _save_plot(output / "base_height.png", "H1 base height", "Time (s)",
               "Height (m)", [
        _Series(time, heights, BLUE, ""),
    ])
    _save_plot(output / "base_attitude.png", "H1 base attitude", "Time (s)",
               "Angle (rad)", [
        _Series(time, rolls, BLUE, "Roll"),
        _Series(time, pitches, ORANGE, "Pitch"),
    ])
    _save_plot(output / "joint_tracking.png", "H1 joint tracking",
               "Time (s)", "RMSE to commanded target (rad)", [
        _Series(time, tracking_rmse, BLUE, ""),
    ])
    _save_plot(output / "balance_feedback.png", "H1 ankle balance feedback",
               "Time (s)", "Target correction (rad)", [
        _Series(time, balance_pitch, BLUE, "Left ankle pitch"),
        _Series(time, balance_roll, ORANGE, "Left ankle roll"),
    ])
    _save_plot(output / "joint_effort.png", "H1 joint effort", "Time (s)",
               "Torque (N·m)", [
        _Series(time, max_pd, BLUE, "Max |PD estimate|"),
        _Series(time, max_projected, GRAY, "Max |projected force|", True),
    ])

    result = {
        "schema_version": 1,
        "analysis_type": "unitree_h1_2_fixstand_feedback",
        "generated_by": "Pro2/tools/analyze_l4_h1_stand.py",
        "source": {
            "run_directory": _project_relative(run_directory),
            **{
                key: _project_relative(path)
                for key, path in source_paths.items()
            },
            "sha256": {
                path.name: _sha256(path) for path in source_paths.values()
            },
        },
        "sampling": {
            "rows": len(rows),
            "columns": len(fieldnames),
            "time_start_s": time[0],
            "time_end_s": time[-1],
            "logged_time_span_s": time[-1] - time[0],
            "median_log_interval_s": statistics.median(
                b - a for a, b in zip(time, time[1:])),
            "simulator_duration_s_from_runtime_metrics": runtime_metrics[
                "simulated_duration_s"],
            "note": (
                "The 100 Hz CSV ends before the final 500 Hz simulator state. "
                "Runtime acceptance values therefore remain authoritative; "
                "sampled extrema and final logged values are reported separately."
            ),
        },
        "metric_definitions": {
            "maximum_abs_roll_or_pitch_rad": "max(max(abs(roll), abs(pitch)))",
            "maximum_horizontal_drift_from_initial_m": (
                "max(hypot(base_x-base_x_first, base_y-base_y_first))"),
            "joint_tracking_rmse_to_commanded_target_rad": (
                "sqrt(mean((q_actual-q_desired)^2)) over selected samples and all joints"),
            "joint_tracking_rmse_to_nominal_target_rad": (
                "sqrt(mean((q_actual-q_nominal)^2)) over selected samples and all joints"),
            "base_height_population_std_m": (
                "population standard deviation of base_z over selected samples"),
            "sampled_effort_extrema": (
                "maximum absolute logged value over selected samples and all joints"),
        },
        "runtime_reported_metrics": runtime_metrics,
        "derived_from_states_csv": sampled_metrics,
        "figures": [
            "base_height.png", "base_attitude.png", "joint_tracking.png",
            "balance_feedback.png", "joint_effort.png",
        ],
    }
    (output / "metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = analyze(args.run_directory, args.output)
    print(json.dumps({
        "output": _project_relative(
            args.output or args.run_directory / "analysis"),
        "samples": result["sampling"]["rows"],
        "runtime_acceptance_passed": result[
            "runtime_reported_metrics"]["acceptance_passed"],
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
