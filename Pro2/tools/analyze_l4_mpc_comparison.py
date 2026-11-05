"""Compare one native Convex-MPC run and one external-NMPC Isaac run."""
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


# These fields describe the shared experiment conditions.  The intentionally
# different algorithm selector (``mpc_backend``), source config path, and output
# directory are recorded in provenance but excluded from the equality check.
KEY_EXPERIMENT_CONFIG_FIELDS = (
    "level",
    "mode",
    "log_control_decomposition",
    "mpc_horizon_steps",
    "duration",
    "dt",
    "mpc_dt",
    "swing_time",
    "step_height",
    "stand_height",
    "walk_start",
    "vx",
    "vy",
    "wz",
    "render_hz",
    "log_hz",
    "push_at",
    "push_duration",
    "push_force",
    "u_weight",
    "state_weights",
    "input_weights",
    "path_tracking",
    "terrain",
    "jump",
)


def load_run_manifest(csv_path):
    """Load the optional ``run.json`` next to a states CSV."""
    manifest_path = csv_path.parent / "run.json"
    if not manifest_path.is_file():
        return None, manifest_path
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取 {manifest_path}：{exc}") from exc
    if not isinstance(manifest, dict):
        raise ValueError(f"{manifest_path} 的顶层必须是 JSON 对象")
    return manifest, manifest_path


def _key_experiment_config(manifest):
    config = manifest.get("config") if manifest else None
    config = config if isinstance(config, dict) else {}
    return {key: config.get(key) for key in KEY_EXPERIMENT_CONFIG_FIELDS}


def _missing_key_experiment_config(manifest):
    config = manifest.get("config") if manifest else None
    config = config if isinstance(config, dict) else {}
    return [key for key in KEY_EXPERIMENT_CONFIG_FIELDS if key not in config]


def manifest_provenance(csv_path, manifest, manifest_path):
    """Return the compact evidence needed to reproduce and audit one run."""
    if manifest is None:
        return {
            "states_csv": str(csv_path),
            "run_json": None,
            "run_json_available": False,
        }
    config = manifest.get("config")
    config = config if isinstance(config, dict) else {}
    return {
        "states_csv": str(csv_path),
        "run_json": str(manifest_path),
        "run_json_available": True,
        "status": manifest.get("status"),
        "controller_library": manifest.get("controller_library"),
        "controller_library_sha256": manifest.get("controller_library_sha256"),
        "upstream_commit": manifest.get("upstream_commit"),
        "isaac_version": manifest.get("isaac_version"),
        "robot_usd": manifest.get("robot_usd"),
        "feedback": manifest.get("feedback"),
        "physics_backend": manifest.get("physics_backend"),
        "config_source": config.get("config"),
        "mpc_backend": config.get("mpc_backend", manifest.get("mpc_backend")),
        "key_experiment_config": _key_experiment_config(manifest),
        "missing_key_experiment_config_fields":
            _missing_key_experiment_config(manifest),
        "external_nmpc": manifest.get("external_nmpc"),
    }


def manifest_comparison_checks(convex_manifest, nmpc_manifest):
    """Check whether both manifests support a fair, same-binary comparison."""
    both_available = convex_manifest is not None and nmpc_manifest is not None
    if not both_available:
        return {
            "run_json_available_for_both": False,
            "controller_library_sha256_match": None,
            "both_runs_completed": None,
            "key_experiment_config_match": None,
            "key_experiment_config_mismatches": None,
            "key_experiment_config_missing_in_convex": None,
            "key_experiment_config_missing_in_nmpc": None,
        }

    convex_hash = convex_manifest.get("controller_library_sha256")
    nmpc_hash = nmpc_manifest.get("controller_library_sha256")
    convex_key = _key_experiment_config(convex_manifest)
    nmpc_key = _key_experiment_config(nmpc_manifest)
    convex_missing = _missing_key_experiment_config(convex_manifest)
    nmpc_missing = _missing_key_experiment_config(nmpc_manifest)
    mismatches = [
        key for key in KEY_EXPERIMENT_CONFIG_FIELDS
        if convex_key[key] != nmpc_key[key]
    ]
    return {
        "run_json_available_for_both": True,
        "controller_library_sha256_match": bool(
            convex_hash and nmpc_hash and convex_hash == nmpc_hash),
        "both_runs_completed": (
            convex_manifest.get("status") == "completed"
            and nmpc_manifest.get("status") == "completed"
        ),
        "key_experiment_config_match": not (
            mismatches or convex_missing or nmpc_missing),
        "key_experiment_config_mismatches": mismatches,
        "key_experiment_config_missing_in_convex": convex_missing,
        "key_experiment_config_missing_in_nmpc": nmpc_missing,
    }


def load(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = reader.fieldnames or []
    if not rows:
        raise ValueError(f"CSV 没有数据：{path}")
    result = {}
    for field in fields:
        try:
            values = [float(row[field]) for row in rows]
        except ValueError as exc:
            raise ValueError(f"{path} 的 {field} 包含非数值") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path} 的 {field} 包含 NaN/Inf")
        result[field] = values
    return result


def mean(values):
    return sum(values) / len(values)


def rmse(values):
    return math.sqrt(mean([value * value for value in values]))


def percentile(values, probability):
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def external_update_indices(columns):
    """Return one logged row for every external NMPC update.

    ``external_mpc_solve_s`` and ``external_mpc_status`` are sample-and-hold
    diagnostics: a 100 Hz logger can therefore record the same 50 Hz solve
    twice.  Timing and failure statistics must use the monotonically
    increasing update counter rather than all CSV rows.
    """
    key = "controller:external_mpc_update_count"
    if key not in columns:
        raise KeyError(
            "NMPC states.csv 缺少 controller:external_mpc_update_count，"
            "无法排除 sample-and-hold 重复样本"
        )
    counters = columns[key]
    selected = []
    seen = set()
    previous = 0
    for index, value in enumerate(counters):
        rounded = round(value)
        if value < 0 or not math.isclose(value, rounded, abs_tol=1e-9):
            raise ValueError(f"{key}[{index}]={value!r} 不是非负整数")
        counter = int(rounded)
        if counter < previous:
            raise ValueError(f"{key} 在第 {index} 行发生回退：{previous} -> {counter}")
        previous = counter
        if counter > 0 and counter not in seen:
            selected.append(index)
            seen.add(counter)
    return selected


def metrics(columns, algorithm):
    required = ("time_s", "base_z_m", "roll_rad", "pitch_rad", "yaw_rad",
                "vx_world_m_s", "vy_world_m_s", "cmd_vx_body_m_s",
                "cmd_vy_body_m_s")
    missing = [key for key in required if key not in columns]
    if missing:
        raise KeyError("states.csv 缺少：" + ", ".join(missing))
    vx_error, vy_error = [], []
    for yaw, vx, vy, cmd_vx, cmd_vy in zip(
            columns["yaw_rad"], columns["vx_world_m_s"], columns["vy_world_m_s"],
            columns["cmd_vx_body_m_s"], columns["cmd_vy_body_m_s"]):
        c, s = math.cos(yaw), math.sin(yaw)
        vx_body = c * vx + s * vy
        vy_body = -s * vx + c * vy
        vx_error.append(vx_body - cmd_vx)
        vy_error.append(vy_body - cmd_vy)
    solve_key = ("controller:external_mpc_solve_s" if algorithm == "nmpc"
                 else "controller:mpc_solve_s")
    raw_solve = columns.get(solve_key, [])
    solve = raw_solve
    status_key = ("controller:external_mpc_status" if algorithm == "nmpc"
                  else "controller:mpc_qp_status")
    raw_status = columns.get(status_key, [])
    status = raw_status
    # The external result is held between NMPC updates.  Deduplicate by the
    # native monotonically increasing update counter before timing statistics.
    selected = None
    held_samples_removed = None
    active_solver_samples = len(raw_solve)
    solver_timing_scope = "all_logged_samples"
    if algorithm == "nmpc":
        selected = external_update_indices(columns)
        solve = [solve[index] for index in selected] if solve else []
        status = [status[index] for index in selected] if status else []
        positive_counter_samples = sum(
            value > 0 for value in columns["controller:external_mpc_update_count"])
        active_solver_samples = positive_counter_samples
        held_samples_removed = positive_counter_samples - len(selected)
        solver_timing_scope = "all_external_updates_deduplicated_by_counter"
    else:
        # The native MPC diagnostics are zero during the initial stand phase.
        # Exclude those rows from timing statistics when an explicit walking
        # flag exists.  The remaining 100 Hz rows are still samples of the
        # controller's 200 Hz internal MPC updates, not a log of every solve.
        walking = columns.get("walking_command")
        if walking is not None:
            selected = [index for index, value in enumerate(walking)
                        if value > 0.5]
            if selected:
                solve = [raw_solve[index] for index in selected] if raw_solve else []
                status = [raw_status[index] for index in selected] if raw_status else []
                active_solver_samples = len(selected)
                solver_timing_scope = (
                    "walking_phase_100hz_samples_of_200hz_internal_updates")
    torque_columns = [values for key, values in columns.items() if key.startswith("tau_nm:")]
    peak_torque = max((max(map(abs, values)) for values in torque_columns), default=None)
    result = {
        "algorithm": algorithm,
        "samples": len(columns["time_s"]),
        "duration_s": columns["time_s"][-1] - columns["time_s"][0],
        "vx_body_rmse_m_s": rmse(vx_error),
        "vy_body_rmse_m_s": rmse(vy_error),
        "max_abs_roll_rad": max(map(abs, columns["roll_rad"])),
        "max_abs_pitch_rad": max(map(abs, columns["pitch_rad"])),
        "minimum_base_height_m": min(columns["base_z_m"]),
        "peak_abs_joint_torque_Nm": peak_torque,
        "solver_failure_samples": sum(value != 0 for value in status),
        "solver_evaluations_logged": len(status),
        "solver_timing_raw_csv_samples": len(raw_solve),
        "solver_timing_active_csv_samples": active_solver_samples,
        "solver_held_samples_removed": held_samples_removed,
        "solver_timing_scope": solver_timing_scope,
        "solve_time_mean_s": mean(solve) if solve else None,
        "solve_time_p95_s": percentile(solve, .95) if solve else None,
        "solve_time_max_s": max(solve) if solve else None,
    }
    if algorithm != "nmpc" and raw_solve:
        result.update({
            "solve_time_all_logged_samples_mean_s": mean(raw_solve),
            "solve_time_all_logged_samples_p95_s": percentile(raw_solve, .95),
            "solve_time_all_logged_samples_max_s": max(raw_solve),
        })
    if algorithm == "nmpc":
        source = columns.get("controller:wrench_source_external", [])
        result["external_wrench_sample_fraction"] = mean(source) if source else None
        result["external_updates_observed"] = len(selected)
        result["external_update_counter_last"] = int(
            round(columns["controller:external_mpc_update_count"][-1]))
    return result


def _fraction(numerator, denominator):
    return numerator / denominator if denominator else None


def _float_matches(left, right):
    if left is None or right is None:
        return None
    try:
        return math.isclose(float(left), float(right), rel_tol=1e-12,
                            abs_tol=1e-12)
    except (TypeError, ValueError):
        return False


def add_external_nmpc_reconciliation(summary, columns, manifest):
    """Add manifest counters and their independently reconstructed CSV values."""
    external = manifest.get("external_nmpc") if manifest else None
    if not isinstance(external, dict):
        summary["external_nmpc_manifest_available"] = False
        return

    selected = external_update_indices(columns)
    counters = columns["controller:external_mpc_update_count"]
    status_values = columns.get("controller:external_mpc_status")
    solve_values = columns.get("controller:external_mpc_solve_s")
    statuses = ([status_values[index] for index in selected]
                if status_values is not None else None)
    solves = ([solve_values[index] for index in selected]
              if solve_values is not None else None)
    csv_updates = len(selected)
    csv_counter_last = int(round(counters[-1]))
    csv_failed = (sum(value != 0 for value in statuses)
                  if statuses is not None else None)
    csv_successful = (csv_updates - csv_failed
                      if csv_failed is not None else None)
    config = manifest.get("config")
    config = config if isinstance(config, dict) else {}
    nmpc_config = config.get("nmpc")
    nmpc_config = nmpc_config if isinstance(nmpc_config, dict) else {}
    update_period = nmpc_config.get("update_period_s")
    if solves is not None and isinstance(update_period, (int, float)):
        csv_deadline_misses = sum(value > update_period for value in solves)
    else:
        csv_deadline_misses = None
    csv_success_fraction = (_fraction(csv_successful, csv_updates)
                            if csv_successful is not None else None)
    csv_deadline_fraction = (_fraction(csv_deadline_misses, csv_updates)
                             if csv_deadline_misses is not None else None)

    # Keep these fields flat so comparison.csv remains a simple tabular file.
    summary.update({
        "external_nmpc_manifest_available": True,
        "external_manifest_updates": external.get("updates"),
        "external_manifest_successful_updates":
            external.get("successful_updates"),
        "external_manifest_failed_updates": external.get("failed_updates"),
        "external_manifest_deadline_misses": external.get("deadline_misses"),
        "external_manifest_success_fraction": external.get("success_fraction"),
        "external_manifest_deadline_miss_fraction":
            external.get("deadline_miss_fraction"),
        "external_csv_unique_updates": csv_updates,
        "external_csv_update_counter_last": csv_counter_last,
        "external_csv_successful_updates": csv_successful,
        "external_csv_failed_updates": csv_failed,
        "external_csv_deadline_misses": csv_deadline_misses,
        "external_csv_success_fraction": csv_success_fraction,
        "external_csv_deadline_miss_fraction": csv_deadline_fraction,
    })
    checks = {
        "external_manifest_updates_match_csv_unique_updates":
            external.get("updates") == csv_updates,
        "external_manifest_updates_match_csv_counter_last":
            external.get("updates") == csv_counter_last,
        "external_manifest_successful_updates_match_csv":
            external.get("successful_updates") == csv_successful,
        "external_manifest_failed_updates_match_csv":
            external.get("failed_updates") == csv_failed,
        "external_manifest_deadline_misses_match_csv":
            (external.get("deadline_misses") == csv_deadline_misses
             if csv_deadline_misses is not None else None),
        "external_manifest_success_fraction_match_csv":
            _float_matches(external.get("success_fraction"),
                           csv_success_fraction),
        "external_manifest_deadline_miss_fraction_match_csv":
            _float_matches(external.get("deadline_miss_fraction"),
                           csv_deadline_fraction),
    }
    summary.update(checks)
    summary["external_manifest_statistics_match_csv"] = all(
        value is True for value in checks.values())


def analyze(convex_csv, nmpc_csv, output):
    convex_manifest, convex_manifest_path = load_run_manifest(convex_csv)
    nmpc_manifest, nmpc_manifest_path = load_run_manifest(nmpc_csv)
    datasets = {"Convex MPC": load(convex_csv), "NMPC": load(nmpc_csv)}
    summaries = [metrics(datasets["Convex MPC"], "convex_mpc"),
                 metrics(datasets["NMPC"], "nmpc")]
    add_external_nmpc_reconciliation(
        summaries[1], datasets["NMPC"], nmpc_manifest)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = list(dict.fromkeys(key for row in summaries for key in row))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summaries)
    time_vectors_match = (datasets["Convex MPC"]["time_s"] ==
                          datasets["NMPC"]["time_s"])
    reference_commands_match = time_vectors_match and all(
        datasets["Convex MPC"].get(key) == datasets["NMPC"].get(key)
        for key in ("cmd_vx_body_m_s", "cmd_vy_body_m_s", "cmd_wz_rad_s")
    )
    comparison_checks = {
        "time_vectors_match_exactly": time_vectors_match,
        "reference_commands_match_exactly": reference_commands_match,
    }
    comparison_checks.update(manifest_comparison_checks(
        convex_manifest, nmpc_manifest))
    provenance = {
        "convex_mpc": manifest_provenance(
            convex_csv, convex_manifest, convex_manifest_path),
        "nmpc": manifest_provenance(
            nmpc_csv, nmpc_manifest, nmpc_manifest_path),
    }
    (output / "metrics.json").write_text(
        json.dumps({"convex_input": str(convex_csv), "nmpc_input": str(nmpc_csv),
                    "runs": summaries,
                    "provenance": provenance,
                    "comparison_checks": comparison_checks,
                    "scope": "One run per algorithm; no statistical significance implied."},
                   ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")

    for velocity, command, ylabel in (
            ("vx_world_m_s", "cmd_vx_body_m_s", "Forward velocity (m/s)"),
            ("vy_world_m_s", "cmd_vy_body_m_s", "Lateral velocity (m/s)")):
        plot_series = []
        for (label, data), color in zip(datasets.items(), (BLUE, ORANGE)):
            # Convert the selected world component to body frame.
            body = []
            for yaw, vx, vy in zip(data["yaw_rad"], data["vx_world_m_s"],
                                   data["vy_world_m_s"]):
                c, s = math.cos(yaw), math.sin(yaw)
                body.append(c * vx + s * vy if velocity.startswith("vx") else -s * vx + c * vy)
            plot_series.append(_Series(data["time_s"], body, color, label))
        convex_command = datasets["Convex MPC"]
        nmpc_command = datasets["NMPC"]
        if (convex_command["time_s"] == nmpc_command["time_s"]
                and convex_command[command] == nmpc_command[command]):
            plot_series.append(_Series(convex_command["time_s"],
                                       convex_command[command], GRAY,
                                       "Reference", True))
        else:
            plot_series.extend((
                _Series(convex_command["time_s"], convex_command[command], GRAY,
                        "Convex reference", True),
                _Series(nmpc_command["time_s"], nmpc_command[command],
                        (132, 85, 190), "NMPC reference", True),
            ))
        _save_plot(output / f"{velocity}_comparison.png",
                   "MPC velocity tracking comparison", "Time (s)", ylabel,
                   plot_series)
    return summaries


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("convex_csv", type=Path)
    parser.add_argument("nmpc_csv", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    summaries = analyze(args.convex_csv, args.nmpc_csv, args.output)
    print(json.dumps(summaries, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
