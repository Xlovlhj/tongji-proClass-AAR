"""Analyze L4 MPC/WBC/PD/Isaac effort decomposition from one states.csv."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

try:
    from .analyze_l4_path_terrain import BLUE, GRAY, ORANGE, _Series, _save_plot
except ImportError:  # direct ``python3 Pro2/tools/...`` execution
    from analyze_l4_path_terrain import BLUE, GRAY, ORANGE, _Series, _save_plot


PURPLE = (132, 85, 190)


DEFAULT_JOINTS = (
    "J_hip_l_pitch", "J_knee_l_pitch", "J_ankle_l_pitch",
    "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch",
)


def load_columns(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f"CSV 没有表头：{path}")
        columns = {name: [] for name in reader.fieldnames}
        for row_number, row in enumerate(reader, 2):
            for name in columns:
                try:
                    value = float(row[name])
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"{path}:{row_number} 的 {name} 不是数值") from exc
                if not math.isfinite(value):
                    raise ValueError(f"{path}:{row_number} 的 {name} 不是有限数值")
                columns[name].append(value)
    if not columns["time_s"]:
        raise ValueError(f"CSV 没有数据行：{path}")
    return columns


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def analyze(input_csv, output, joints):
    columns = load_columns(input_csv)
    output.mkdir(parents=True, exist_ok=True)
    figures = output / "figures"
    figures.mkdir(exist_ok=True)
    time = columns["time_s"]
    rows = []
    for joint in joints:
        keys = {
            "wbc": f"control:wbc_feedforward_Nm:{joint}",
            "pd_raw": f"control:pd_raw_Nm:{joint}",
            "pd_filtered": f"control:pd_filtered_Nm:{joint}",
            "prelimit": f"control:prelimit_Nm:{joint}",
            "command": f"control:command_Nm:{joint}",
        }
        missing = [key for key in keys.values() if key not in columns]
        if missing:
            raise KeyError("states.csv 缺少控制量分解列：" + ", ".join(missing))
        applied_key = f"isaac_applied_Nm:{joint}"
        applied = columns.get(applied_key)
        command = columns[keys["command"]]
        residual = ([a - b for a, b in zip(applied, command)]
                    if applied is not None else None)
        rows.append({
            "joint": joint,
            "wbc_rms_Nm": rms(columns[keys["wbc"]]),
            "pd_filtered_rms_Nm": rms(columns[keys["pd_filtered"]]),
            "command_peak_abs_Nm": max(map(abs, command)),
            "saturation_samples": sum(
                abs(before - after) > 1e-8
                for before, after in zip(columns[keys["prelimit"]], command)),
            "isaac_applied_available": applied is not None,
            "command_vs_applied_rmse_Nm": rms(residual) if residual is not None else None,
        })

        plot_series = [
            _Series(time, columns[keys["wbc"]], BLUE, "WBC feedforward"),
            _Series(time, columns[keys["pd_filtered"]], ORANGE,
                    "PD correction", True),
            _Series(time, command, GRAY, "Command"),
        ]
        if applied is not None:
            plot_series.append(_Series(time, applied, PURPLE, "Isaac applied", True))
        _save_plot(figures / f"{joint}.png", joint, "Time (s)",
                   "Joint torque (N·m)", plot_series)

    left = "control:mpc_wrench:left_fz_N"
    right = "control:mpc_wrench:right_fz_N"
    if left in columns and right in columns:
        _save_plot(figures / "mpc_vertical_contact_force.png",
                   "MPC vertical contact force", "Time (s)", "Force (N)", [
                       _Series(time, columns[left], BLUE, "Left foot"),
                       _Series(time, columns[right], ORANGE, "Right foot", True),
                   ])

    with (output / "joint_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "input": str(input_csv),
        "samples": len(time),
        "duration_s": time[-1] - time[0],
        "joints": rows,
        "actual_effort_semantics": (
            "isaac_applied_Nm is get_dof_efforts actuation when present; "
            "projected joint force must be logged separately."
        ),
    }
    (output / "metrics.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--joints", nargs="+", default=DEFAULT_JOINTS)
    args = parser.parse_args(argv)
    summary = analyze(args.input_csv, args.output, args.joints)
    print(json.dumps({"samples": summary["samples"],
                      "duration_s": summary["duration_s"],
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
