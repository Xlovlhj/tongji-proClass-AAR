"""Record the missing presentation demos through the existing Isaac play loop.

python3 Pro2/output/final/record_missing_demos.py
Each run keeps its own config, states, logs, frames and MP4 in its stage folder.
"""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import time

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
TAG = "ppt_video_20261104"


def jobs():
    result = []
    for group, cases in [
        ("prediction_horizon", ["short_n5", "baseline_n10", "long_n15"]),
        ("step_height", ["low_008", "baseline_012", "high_016"]),
        ("lateral_push", ["no_push", "push_50N", "push_100N"]),
    ]:
        for case in cases:
            result.append(dict(name=f"L2/{group}/{case}", level="L2",
                config=f"Pro2/experiments/L2/{group}/{case}.json",
                output=f"Pro2/output/L2/{group}/{TAG}/{case}", extra=[]))
    for group, case, section in [
        ("path_tracking", "waypoints", "02_path_tracking"),
        ("terrain", "rough_medium", "03_terrain_and_jump"),
        ("terrain", "slope_05deg", "03_terrain_and_jump"),
        ("jump", "vertical_jump", "03_terrain_and_jump"),
    ]:
        result.append(dict(name=f"L4/{case}", level="L4",
            config=f"Pro2/experiments/L4/{group}/{case}.json",
            output=f"Pro2/output/L4/{section}/{TAG}/{case}",
            extra=["--log-control-decomposition"] if group == "jump" else []))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--isaac-root", default="/home/xqw/isaacsim6.0.1")
    args = parser.parse_args()
    logdir = OUT / "checks" / "recordings"
    logdir.mkdir(parents=True, exist_ok=True)
    records = []
    for horizon in [5, 15]:
        command = [sys.executable, "Pro2/run_isaac.py", "--build-only",
                   "--mpc-horizon-steps", str(horizon), "--jobs", "2"]
        print("BUILD", subprocess.list2cmdline(command), flush=True)
        if not args.dry_run:
            with (logdir / f"build_n{horizon}.log").open("w") as stream:
                subprocess.run(command, cwd=ROOT, stdout=stream,
                               stderr=subprocess.STDOUT, check=True)
    for index, job in enumerate(jobs(), 1):
        command = [sys.executable, "Pro2/run_isaac.py", "--isaac-root", args.isaac_root,
                   "--no-build", "--headless", "--level", job["level"],
                   "--config", job["config"], "--record-video", "--video-fps", "30",
                   "--output", job["output"], *job["extra"]]
        output = ROOT / job["output"]
        record = dict(**job, command=command)
        if args.dry_run:
            print(subprocess.list2cmdline(command), flush=True)
            continue
        manifest = output / "run.json"
        if manifest.exists():
            data = json.loads(manifest.read_text())
            if data.get("status") == "completed" and data.get("video_saved") and (output / "demo.mp4").is_file():
                print("REUSE", job["name"], flush=True)
                record["status"] = "reused_completed"
                records.append(record)
                continue
            raise RuntimeError(f"Existing incomplete run: {output}; preserve it and choose another destination")
        print(f"RECORD {index}/13 {job['name']}", flush=True)
        output.mkdir(parents=True, exist_ok=True)
        (output / "recording_command.json").write_text(json.dumps(command, ensure_ascii=False, indent=2)+"\n")
        start = time.monotonic()
        with (output / "console.log").open("w") as stream:
            process = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        record.update(returncode=process.returncode, elapsed_wall_s=round(time.monotonic()-start, 2))
        data = json.loads(manifest.read_text()) if manifest.is_file() else {}
        record.update(status=data.get("status", "failed_to_start"),
                      video_saved=data.get("video_saved", False),
                      video_frames=data.get("video_frames"),
                      simulation_duration_s=data.get("simulation_duration_s"))
        records.append(record)
        (logdir / "recording_status.json").write_text(json.dumps(records, ensure_ascii=False, indent=2)+"\n")
        if process.returncode or record["status"] != "completed" or not record["video_saved"]:
            raise RuntimeError(f"Recording failed; inspect {output}/console.log and run.json")
        print("DONE", job["name"], record["simulation_duration_s"], "s", flush=True)
    print("ALL MISSING RECORDINGS COMPLETED", flush=True)


if __name__ == "__main__":
    main()
