"""Record the two missing terrain videos using the existing Isaac play loop.

python3 Pro2/output/final/record_terrain_quad_demos.py
"""
from pathlib import Path
import json
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
DEST = ROOT / "Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105"


def main():
    records = []
    for case in ("random_obstacles_seed01_full", "blind_step_up"):
        output = DEST / case
        command = [sys.executable, "Pro2/run_isaac.py", "--isaac-root",
                   "/home/xqw/isaacsim6.0.1", "--no-build", "--headless",
                   "--level", "L4", "--config", f"Pro2/experiments/L4/terrain/{case}.json",
                   "--record-video", "--video-fps", "30", "--output", str(output)]
        manifest = output / "run.json"
        if output.exists():
            data = json.loads(manifest.read_text()) if manifest.exists() else {}
            if data.get("status") == "completed" and data.get("video_saved") and (output / "demo.mp4").is_file():
                print("REUSE", case, flush=True)
                continue
            raise RuntimeError(f"Preserve existing unfinished run: {output}")
        output.mkdir(parents=True)
        (output / "recording_command.json").write_text(json.dumps(command, ensure_ascii=False, indent=2) + "\n")
        print("RECORD", case, flush=True)
        started = time.monotonic()
        with (output / "console.log").open("w") as stream:
            result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        data = json.loads(manifest.read_text()) if manifest.exists() else {}
        records.append({"case": case, "returncode": result.returncode,
                        "wall_seconds": round(time.monotonic() - started, 2),
                        "status": data.get("status"), "video_saved": data.get("video_saved"),
                        "simulation_duration_s": data.get("simulation_duration_s")})
        (DEST / "recording_status.json").write_text(json.dumps(records, ensure_ascii=False, indent=2) + "\n")
        if result.returncode or data.get("status") != "completed" or not data.get("video_saved"):
            raise RuntimeError(f"Inspect {output}/console.log and run.json")
        print("DONE", case, data.get("simulation_duration_s"), flush=True)


if __name__ == "__main__":
    main()
