"""Validate the original L0 clip and supplementary L2/L4 demo recordings.

python3 Pro2/output/final/verify_demo_recordings.py --available
python3 Pro2/output/final/verify_demo_recordings.py
python3 Pro2/output/final/verify_demo_recordings.py --only L4/random_obstacles_seed01_full --only L4/blind_step_up

This script runs no simulation and leaves the original experiment reports intact.
QP status counts describe sampled controller results; they are not pass criteria
for recording integrity and must not be presented as proof of zero QP failures.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
CHECKS = OUT / "checks"
sys.path.insert(0, str(ROOT / "Pro2"))
from run_isaac import arguments as simulation_arguments
from isaac.jump_controller import PHASE_CODES
from record_missing_demos import jobs
from build_demo_video import is_recorded_instability

METADATA_FIELDS = {"isaac_root", "repo_root", "level", "config", "check_env",
                   "build_only", "no_build", "rebuild_asset", "show_config",
                   "headless", "record_video", "video_fps", "render_hz",
                   "log_hz", "output", "jobs", "log_control_decomposition"}
GROUPS = {
    "mpc_vx_weight": ("state_weights[9]", [.2, 1, 5]),
    "prediction_horizon": ("mpc_horizon_steps", [5, 10, 15]),
    "step_height": ("step_height", [.08, .12, .16]),
    "lateral_push": ("push_force[1]", [0, 50, 100]),
}


def rel(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def sha(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def ffmpeg_path():
    found = shutil.which("ffmpeg")
    if found:
        return found
    for env in ("rllab", "amp-env"):
        candidates = list((Path.home() / "miniconda3/envs" / env).glob(
            "lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux*"))
        if candidates:
            return str(candidates[0])
    raise RuntimeError("FFmpeg not found")


def all_jobs():
    original = {"name": "L0/walk_10s_video", "level": "L0", "config": None,
                "output": "Pro2/output/L0/walk_10s_video", "extra": []}
    weights = [{"name": f"L2/mpc_vx_weight/{case}", "level": "L2",
                "config": f"Pro2/experiments/L2/mpc_vx_weight/{case}.json",
                "output": f"Pro2/output/L2/ppt_demo_20261104/{case}", "extra": []}
               for case in ("low", "baseline", "high")]
    selected = [original, *weights, *jobs()]
    # The revised disturbance videos and formal metrics share the same runs.
    # Fourteen unchanged source clips plus these three = seventeen clips.
    for job in selected:
        if job["name"].startswith("L2/lateral_push/"):
            case = job["name"].rsplit("/", 1)[1]
            job["output"] = f"Pro2/output/L2/lateral_push/long_push_20261105/{case}"
    selected.extend({
        "name": f"L4/{case}", "level": "L4",
        "config": f"Pro2/experiments/L4/terrain/{case}.json",
        "output": f"Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/{case}",
        "extra": [],
    } for case in ("random_obstacles_seed01_full", "blind_step_up"))
    return selected


def physical(config):
    return {key: value for key, value in config.items() if key not in METADATA_FIELDS}


def flatten(value, prefix=""):
    result = {}
    if isinstance(value, dict):
        for key, item in value.items():
            result.update(flatten(item, f"{prefix}.{key}" if prefix else key))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            result.update(flatten(item, f"{prefix}[{index}]"))
    else:
        result[prefix] = value
    return result


def differences(left, right):
    a, b = flatten(left), flatten(right)
    return {key: {"expected": a.get(key), "actual": b.get(key)}
            for key in sorted(a.keys() | b.keys()) if a.get(key) != b.get(key)}


def inspect_video(ffmpeg, video, name):
    proc = subprocess.run([ffmpeg, "-hide_banner", "-nostdin", "-nostats",
                           "-xerror", "-i", str(video), "-map", "0:v:0",
                           "-an", "-f", "null", "-", "-progress", "pipe:1"],
                          capture_output=True, text=True)
    log = CHECKS / "recordings" / "decode" / (name.replace("/", "_") + ".log")
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(proc.stderr + "\n" + proc.stdout)
    duration = re.search(r"Duration:\s*(\d+):(\d+):(\d+(?:\.\d+)?)", proc.stderr)
    info = next((line.strip() for line in proc.stderr.splitlines() if "Video:" in line), "")
    dimensions = re.search(r"(?<!\d)(\d{2,5})x(\d{2,5})(?!\d)", info)
    fps = re.search(r"(\d+(?:\.\d+)?) fps", info)
    frames = re.findall(r"^frame=(\d+)$", proc.stdout, re.M)
    return {"full_decode_succeeded": proc.returncode == 0,
            "duration_s": sum(float(v) * scale for v, scale in zip(duration.groups(), [3600, 60, 1])) if duration else None,
            "width": int(dimensions[1]) if dimensions else None,
            "height": int(dimensions[2]) if dimensions else None,
            "fps": float(fps[1]) if fps else None,
            "decoded_frames": int(frames[-1]) if frames else None,
            "stream": info, "decode_log": rel(log)}


def inspect_csv(path):
    statuses = {key: Counter() for key in ("controller:mpc_qp_status", "controller:wbc_qp_status")}
    phase_counts, phase_values_valid, phase_codes_match = Counter(), True, True
    count, finite, first, last, previous, monotonic = 0, True, None, None, None, True
    bounds = {key: [math.inf, -math.inf] for key in
              ("base_x_m", "base_y_m", "base_z_m", "roll_rad", "pitch_rad", "force_y_n")}
    nonfinite = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        for row in reader:
            count += 1
            for key, raw in row.items():
                if key == "jump:phase":
                    phase_counts[raw] += 1
                    phase_values_valid &= raw in PHASE_CODES
                    try:
                        phase_codes_match &= float(row["jump:phase_code"]) == PHASE_CODES.get(raw)
                    except (KeyError, ValueError, TypeError):
                        phase_codes_match = False
                    continue
                try:
                    value = float(raw)
                except (TypeError, ValueError):
                    finite = False
                    if len(nonfinite) < 8:
                        nonfinite.append({"row": count, "field": key, "value": raw})
                    continue
                if not math.isfinite(value):
                    finite = False
                    if len(nonfinite) < 8:
                        nonfinite.append({"row": count, "field": key, "value": raw})
                if key in bounds:
                    bounds[key] = [min(bounds[key][0], value), max(bounds[key][1], value)]
            current = float(row["time_s"])
            first = current if first is None else first
            last = current
            monotonic &= previous is None or current > previous
            previous = current
            for key, counter in statuses.items():
                if key in row:
                    counter[str(float(row[key]))] += 1
    return {"samples": count, "all_values_finite": finite, "first_nonfinite": nonfinite,
            "jump_phase_counts": dict(phase_counts), "jump_phase_values_valid": phase_values_valid,
            "jump_phase_codes_match": phase_codes_match,
            "time_bounds_s": [first, last], "time_strictly_increases": monotonic,
            "bounds": bounds,
            "qp_statuses": {key: {"present": key in fields, "counts": dict(counter),
                "nonzero_samples": sum(n for value, n in counter.items() if float(value) != 0)}
                for key, counter in statuses.items()}}


def inspect_run(job, ffmpeg):
    directory = ROOT / job["output"]
    manifest_path = directory / "run.json"
    if not manifest_path.exists():
        return {**job, "state": "pending", "reason": "run.json not yet written"}
    manifest = json.loads(manifest_path.read_text())
    physical_stop = job["name"] == "L2/lateral_push/push_100N" and is_recorded_instability(manifest)
    if manifest.get("status") != "completed" and not physical_stop:
        return {**job, "state": "pending", "reason": "run status: " + str(manifest.get("status"))}
    config = manifest["config"]
    video = inspect_video(ffmpeg, directory / "demo.mp4", job["name"])
    states = inspect_csv(directory / "states.csv")
    planned_duration = config["duration"]
    expected_steps = manifest["steps"] if physical_stop else round(planned_duration / config["dt"])
    duration = expected_steps * config["dt"]
    expected_samples = math.ceil(expected_steps / max(1, round(1 / (config["log_hz"] * config["dt"]))))
    expected_frames = math.ceil(duration * config["video_fps"] - 1e-9) if physical_stop else round(duration * config["video_fps"])
    checks = {"run_completed_or_documented_physical_stop": True, "all_state_values_finite": states["all_values_finite"],
              "categorical_jump_phases_valid": states["jump_phase_values_valid"],
              "jump_phase_codes_match_names": states["jump_phase_codes_match"],
              "state_time_monotonic": states["time_strictly_increases"],
              "steps_match_completed_or_physically_stopped_run": manifest.get("steps") == expected_steps,
              "simulation_duration_matches": abs(manifest.get("simulation_duration_s", -1) - duration) < 1e-8,
              "sample_count_matches": states["samples"] == manifest.get("samples") == expected_samples,
              "samples_cover_full_run": states["time_bounds_s"][0] == 0 and
                    abs(states["time_bounds_s"][1] - ((expected_samples - 1) / config["log_hz"])) < config["dt"] * 1.01,
              "video_saved": manifest.get("video_saved") is True,
              "full_video_decode_succeeded": video["full_decode_succeeded"],
              "video_frame_count_matches": video["decoded_frames"] == manifest.get("video_frames") == expected_frames,
              "video_duration_matches": video["duration_s"] is not None and abs(video["duration_s"] - expected_frames / config["video_fps"]) < .02,
              "video_fps_matches": video["fps"] == config["video_fps"],
              "saved_config_matches_manifest": json.loads((directory / "config.json").read_text()) == config}
    if physical_stop:
        checks.update({
            "partial_video_explicitly_not_complete": manifest.get("video_complete") is False,
            "physical_stop_time_matches_executed_steps": abs(manifest["termination_state"]["time_s"] - duration) < 1e-8,
            "physical_stop_precedes_planned_end": 0 < duration < planned_duration,
            "partial_video_time_metadata_matches": abs(manifest["video_duration_s"] - expected_frames / config["video_fps"]) < 1e-8,
            "raw_video_not_filled_to_planned_end": video["duration_s"] < planned_duration,
        })
    config_diff = {}
    if job["config"]:
        expected = vars(simulation_arguments(["--config", str(ROOT / job["config"]), "--level", job["level"]]))
        config_diff = differences(physical(expected), physical(config))
        checks["source_config_and_defaults_match_effective_physics"] = not config_diff
        library = Path(manifest["controller_library"])
        checks["controller_library_matches_recorded_hash"] = library.is_file() and sha(library) == manifest.get("controller_library_sha256")
        checks["controller_library_horizon_matches"] = library.parent.name == f"mpc_n{config['mpc_horizon_steps']}"
    analyses = {}
    for path in [directory / "path_metrics.json", directory / "jump_metrics.json", directory / "analysis" / "metrics.json"]:
        if path.is_file():
            analyses[rel(path)] = {"sha256": sha(path), "data": json.loads(path.read_text())}
    files = [directory / name for name in ("run.json", "config.json", "states.csv", "demo.mp4")]
    if job["config"]:
        files.append(ROOT / job["config"])
    return {**job, "state": "validated", "pass": all(checks.values()), "checks": checks,
            "video": video, "states": states, "effective_physics": physical(config),
            "config_differences": config_diff, "analysis_evidence": analyses,
            "controller_library_sha256": manifest.get("controller_library_sha256"),
            "controller_library": manifest.get("controller_library"),
            "source_sha256": {rel(path): sha(path) for path in files},
            "maximum_tilt_rad": manifest.get("maximum_tilt_rad"),
            "minimum_base_height_m": manifest.get("minimum_base_height_m"),
            "path_visualization": manifest.get("path_visualization"),
            "force_visualization": manifest.get("force_visualization"),
            "behavior_status": manifest["status"],
            "physical_instability_stop": physical_stop,
            "termination_state": manifest.get("termination_state"),
            "jump_control": manifest.get("jump_control")}


def inspect_builds():
    data = {}
    for horizon in (5, 10, 15):
        directory = ROOT / f"Pro2/isaac/build/mpc_n{horizon}"
        commands = json.loads((directory / "compile_commands.json").read_text())
        normalized = []
        sources, objects = {}, []
        for row in commands:
            source = Path(row["file"])
            source_name = rel(source).replace(f"mpc_n{horizon}/", "mpc_nN/")
            sources[source_name] = sha(source)
            normalized.append({key: re.sub(r"OPENLOONG_MPC_N=\d+", "OPENLOONG_MPC_N=N",
                               value.replace(str(directory), "<build_directory>"))
                               for key, value in row.items()})
            tokens = shlex.split(row["command"])
            obj = directory / tokens[tokens.index("-o") + 1]
            objects.append({"source": rel(source), "object": rel(obj),
                            "object_exists": obj.exists(),
                            "object_not_older_than_source": obj.exists() and obj.stat().st_mtime >= source.stat().st_mtime})
        cache = (directory / "CMakeCache.txt").read_text()
        flags = (directory / "CMakeFiles/openloong_controller.dir/flags.make").read_text()
        checks = {"horizon_cache_correct": f"OPENLOONG_MPC_HORIZON_STEPS:STRING={horizon}\n" in cache,
                  "horizon_compiler_definition_correct": f"-DOPENLOONG_MPC_N={horizon} " in flags,
                  "all_objects_exist_and_are_current_for_cpp_sources": all(row["object_not_older_than_source"] for row in objects)}
        data[str(horizon)] = {"checks": checks, "source_sha256": sources,
                              "normalized_compile_commands": normalized,
                              "library_sha256": sha(directory / "libopenloong_controller.so"),
                              "objects": objects}
    checks = {"same_cpp_source_hashes_across_horizons": all(row["source_sha256"] == data["10"]["source_sha256"] for row in data.values()),
              "same_compiler_options_except_horizon": all(row["normalized_compile_commands"] == data["10"]["normalized_compile_commands"] for row in data.values()),
              "all_build_checks_pass": all(all(row["checks"].values()) for row in data.values())}
    source_files = [ROOT / "Pro2/run_isaac.py", *sorted((ROOT / "Pro2/isaac").glob("*.py")),
                    *sorted((ROOT / "Pro2/isaac/native").glob("*"))]
    for subdir in ("algorithm", "common", "math"):
        source_files += sorted((ROOT / "Pro2/OpenLoong-Dyn-Control" / subdir).glob("*.h"))
    return {"checks": checks, "builds": data, "pass": all(checks.values()),
            "shared_source_sha256": {rel(path): sha(path) for path in source_files if path.is_file()},
            "scope": "Compile manifests and generated C++ source hashes agree except for OPENLOONG_MPC_N; run.json pins the loaded library hash. Historical build-time snapshots are not reconstructed."}


def comparisons(recordings):
    result = {}
    for group, (variable, expected_values) in GROUPS.items():
        rows = [row for row in recordings if row["name"].startswith(f"L2/{group}/") and row["state"] == "validated"]
        if len(rows) != 3:
            result[group] = {"state": "pending", "available_cases": len(rows)}
            continue
        normalized = [flatten(row["effective_physics"]) for row in rows]
        values = [row.pop(variable) for row in normalized]
        identical_library = len({row["controller_library_sha256"] for row in rows}) == 1
        checks = {"exact_controlled_values": values == expected_values,
                  "only_named_physics_parameter_differs": all(row == normalized[0] for row in normalized),
                  "same_library_except_horizon_experiment": group == "prediction_horizon" or identical_library,
                  "same_video_fps_and_start": len({(row["video"]["fps"], row["states"]["time_bounds_s"][0]) for row in rows}) == 1,
                  "equal_planned_duration_or_disclosed_physical_stop": len({row["effective_physics"]["duration"] for row in rows}) == 1 and
                    all(abs(row["video"]["duration_s"] - row["effective_physics"]["duration"]) < .02 or row["physical_instability_stop"] for row in rows)}
        result[group] = {"state": "validated", "variable": variable, "values": values,
                         "checks": checks, "pass": all(checks.values()),
                         "behavior_statuses": {row["name"]: row["behavior_status"] for row in rows},
                         "observed_video_durations_s": {row["name"]: row["video"]["duration_s"] for row in rows}}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--available", action="store_true", help="Validate completed runs while other recordings remain pending")
    parser.add_argument("--only", action="append", choices=[job["name"] for job in all_jobs()],
                        help="Validate only this recording; repeat for several clips. Skip unrelated build/group checks and preserve the full verification report.")
    parser.add_argument("--report", type=Path,
                        help="Separate JSON output path; defaults to terrain_recordings_verification.json for targeted terrain clips or selected_recordings_verification.json for other selections")
    args = parser.parse_args()
    selected_jobs = [job for job in all_jobs() if not args.only or job["name"] in args.only]
    report = args.report
    if report is None:
        if args.only:
            terrain_names = {"L4/random_obstacles_seed01_full", "L4/blind_step_up"}
            report = CHECKS / ("terrain_recordings_verification.json" if set(args.only) <= terrain_names
                               else "selected_recordings_verification.json")
        else:
            report = CHECKS / "recordings_verification.json"
    if args.only and report.resolve() == (CHECKS / "recordings_verification.json").resolve():
        parser.error("Targeted validation must use a separate report to preserve the full recording verification.")
    CHECKS.mkdir(parents=True, exist_ok=True)
    ffmpeg = ffmpeg_path()
    recordings = []
    for job in selected_jobs:
        try:
            result = inspect_run(job, ffmpeg)
        except Exception as error:
            result = {**job, "state": "error", "pass": False, "error": f"{type(error).__name__}: {error}"}
        recordings.append(result)
        print(result["name"], result["state"], result.get("pass", ""), flush=True)
    build = inspect_builds() if not args.only else {
        "state": "not_requested", "pass": True,
        "scope": "Targeted recording integrity and per-run controller-library hashes only; existing full-build provenance is preserved in recordings_verification.json."}
    cross = comparisons(recordings) if not args.only else {}
    complete = all(row["state"] == "validated" for row in recordings)
    available_pass = all(row.get("pass", True) for row in recordings) and build["pass"] and all(row.get("pass", True) for row in cross.values())
    result = {"verified_at": datetime.now().astimezone().isoformat(),
              "purpose": "PPT demonstration evidence. Revised lateral-push recordings and formal L2 metrics share the 2026-11-05 runs; other recordings retain their separate formal experiment sources.",
              "ffmpeg_binary": ffmpeg, "requested_clips": len(recordings),
              "validated_clips": sum(row["state"] == "validated" for row in recordings),
              "all_requested_recording_evidence_ready": complete, "available_recordings_pass": available_pass,
              "all_selected_simulations_completed": complete and all(row.get("behavior_status") == "completed" for row in recordings),
              "physically_failed_cases": [row["name"] for row in recordings if row.get("physical_instability_stop")],
              "pass": complete and available_pass, "available_only": args.available,
              "selection": args.only, "report_path": rel(report),
              "recordings": recordings, "comparisons": cross, "controller_build_provenance": build,
              "sampling_boundary": "QP status counts are states.csv samples (100 Hz), not unique QP solve counts. Nonzero status counts are disclosed and do not change the recording-integrity verdict; inspect stage metrics for behavioral acceptance."}
    write_json(report, result)
    print(json.dumps({key: result[key] for key in ("requested_clips", "validated_clips", "all_requested_recording_evidence_ready", "all_selected_simulations_completed", "physically_failed_cases", "available_recordings_pass", "pass")}, ensure_ascii=False))
    if not available_pass or (not args.available and not complete):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
