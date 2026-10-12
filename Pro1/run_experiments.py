#!/usr/bin/env python3
"""Run four controllers with and without force, serially, from Python.

This supervisor imports no Isaac/GPU modules. Each simulation runs in a child
process; a failed run stops the batch and remains visible in its log/manifest.
Resume only reuses this runner's completed, unchanged, validated artifacts.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
from typing import Any
import zipfile

from main import DEFAULTS
from raa.config import CONTROLLERS, load_config, merge_config
from raa.figures import export_directory, locate_chrome
from raa.outputs import compare_runs
from raa.video_validation import inspect_mp4


ROOT = Path(__file__).resolve().parent
CONDITIONS = ("normal", "disturbed")
COMMON_EXCLUDED = {"controller", "adaptive", "output", "record_video", "disturbance"}
ERROR_FIELDS = ("tip_rmse_m", "tip_max_error_m", "xy_rmse_m", "xy_max_error_m",
                "drawing_tip_rmse_m", "drawing_tip_max_error_m",
                "drawing_xy_rmse_m", "drawing_xy_max_error_m")


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_value(value):
    return json.loads(json.dumps(value, sort_keys=True, default=str, allow_nan=False))


def _digest(value) -> str:
    return hashlib.sha256(json.dumps(_json_value(value), sort_keys=True,
                                     separators=(",", ":")).encode()).hexdigest()


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON 顶层必须是对象：{path}")
    return value


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                    allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def source_fingerprint(isaac_root: str | Path | None = None) -> dict[str, Any]:
    """Hash executable project sources, vector input, presets and robot inputs."""
    sources = [ROOT / "main.py", ROOT / "run_experiments.py",
               *sorted((ROOT / "raa").glob("*.py")),
               *sorted((ROOT / "configs").glob("*.json")), ROOT / "assets/chatgpt_logo.svg"]
    if isaac_root is not None:
        isaac = Path(isaac_root)
        sources.extend(isaac / relative for relative in (
            "VERSION",
            "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf",
            "extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf",
            "extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/rmpflow/robot_descriptor.yaml"))
    files = {str(path.resolve()): file_sha256(path) for path in sources}
    return {"sha256": _digest(files), "files": files}


def build_plan(args: argparse.Namespace) -> list[dict[str, Any]]:
    plan = []
    baseline = None
    for condition in args.conditions:
        for controller in args.controllers:
            output = (args.output_root / condition / controller).resolve()
            video = (args.video_root / f"{controller}_{condition}.mp4").resolve()
            overrides = {"headless": True, "output": output, "record_video": video,
                         "hold_seconds": 0.0, "max_steps": 0,
                         "disturbance": {"enabled": condition == "disturbed"}}
            if args.isaac_root is not None:
                overrides["isaac_root"] = args.isaac_root
            config = vars(merge_config(DEFAULTS, load_config(controller), overrides))
            resolved = _json_value({key: value for key, value in config.items()
                                    if key not in {"preview", "check_env", "config", "config_path"}})
            common = {key: value for key, value in resolved.items() if key not in COMMON_EXCLUDED}
            common["disturbance_schedule"] = {key: value for key, value in resolved["disturbance"].items() if key != "enabled"}
            if baseline is None:
                baseline = common
            elif common != baseline:
                raise ValueError("四个预设的轨迹、控制增益、物理步长或视频参数不一致，无法公平批量比较。")
            command = [sys.executable, str(ROOT / "main.py"), "--config", controller,
                       "--headless", "--record-video", str(video), "--output", str(output),
                       "--hold-seconds", "0", "--max-steps", "0",
                       "--disturbance" if condition == "disturbed" else "--no-disturbance"]
            if args.isaac_root is not None:
                command.extend(["--isaac-root", str(config["isaac_root"])])
            plan.append({"id": f"{controller}_{condition}", "controller": controller,
                         "condition": condition, "output": str(output), "video": str(video),
                         "pictures": str((args.pictures_root / condition / controller).resolve()),
                         "log": str((args.output_root / "logs" / f"{controller}_{condition}.log").resolve()),
                         "command": command, "resolved_config": resolved,
                         "resolved_config_sha256": _digest(resolved),
                         "common_config_sha256": _digest(common)})
    return plan


def _finite(value, name: str, *, positive=False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} 需要有限数值")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} 需要有限数值") from exc
    if not math.isfinite(number) or (positive and number <= 0):
        raise ValueError(f"{name} 需要有限" + ("正数" if positive else "数值"))
    return number


def _positive_int(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} 需要正整数")
    return value


def _bool(value: str, name: str) -> bool:
    if value in {"1", "True", "true"}:
        return True
    if value in {"0", "False", "false"}:
        return False
    raise ValueError(f"{name} 需要布尔值")


def validate_mp4(path: Path) -> dict[str, Any]:
    """Check the native encoder's ISO-BMFF container without another codec install."""
    total = path.stat().st_size
    boxes = []
    media_bytes = 0
    with path.open("rb") as stream:
        offset = 0
        while offset < total:
            header = stream.read(8)
            if len(header) != 8:
                raise ValueError(f"MP4 包含截断的 box：{path}")
            size, kind = struct.unpack(">I4s", header)
            header_size = 8
            if size == 1:
                extended = stream.read(8)
                if len(extended) != 8:
                    raise ValueError(f"MP4 包含截断的扩展 box：{path}")
                size = struct.unpack(">Q", extended)[0]
                header_size = 16
            elif size == 0:
                size = total - offset
            if size < header_size or offset + size > total:
                raise ValueError(f"MP4 box 大小超出文件：{path}")
            boxes.append(kind.decode("ascii", errors="replace"))
            if kind == b"mdat":
                media_bytes += size - header_size
            stream.seek(offset + size)
            offset += size
    if not {"ftyp", "moov", "mdat"}.issubset(boxes) or media_bytes == 0:
        raise ValueError(f"MP4 缺少完整的 ftyp/moov/mdat 或视频数据：{path}")
    return {"file_size_bytes": total, "top_level_boxes": boxes, "media_data_bytes": media_bytes}


def validate_run(item: dict[str, Any]) -> dict[str, Any]:
    """Independently check completion, 1 kHz clock, finite logs and video metadata."""
    output, video_path = Path(item["output"]), Path(item["video"])
    metadata = _load_json(output / "metrics.json")
    if metadata.get("completed") is not True or metadata.get("timeline_play_enabled") is not True:
        raise ValueError("实验必须完整完成，并在 Isaac Play 状态下运行。")
    if metadata.get("controller") != item["controller"]:
        raise ValueError("结果的 controller 与计划不一致。")
    if metadata.get("external_disturbance_enabled") is not (item["condition"] == "disturbed"):
        raise ValueError("结果的外力条件与计划不一致。")
    resolved = _load_json(output / "resolved_config.json")
    if resolved != item["resolved_config"]:
        raise ValueError("实际 resolved_config.json 与计划配置不一致。")
    metrics = metadata.get("metrics")
    if not isinstance(metrics, dict):
        raise ValueError("实验缺少 metrics 对象。")
    count = _positive_int(metrics.get("sample_count"), "sample_count")
    dt = _finite(metadata.get("physics_dt_s"), "physics_dt_s", positive=True)
    duration = _finite(metadata.get("reference_duration_s"), "reference_duration_s", positive=True)
    elapsed = _finite(metadata.get("physics_elapsed_s"), "physics_elapsed_s", positive=True)
    if dt != item["resolved_config"]["dt"] or count != math.ceil(duration / dt) + 1:
        raise ValueError("物理步长或完整参考轨迹的样本数不匹配。")
    # PhysX stores dt in float32; allow its accumulation error, not an extra step.
    if not math.isclose(count * dt, elapsed, rel_tol=1e-6, abs_tol=1e-5):
        raise ValueError(f"物理时钟 {elapsed:.9f}s 与 {count}×{dt}s 不一致。")
    sums = {key: 0.0 for key in ("tip", "xy", "drawing_tip", "drawing_xy")}
    maxima = dict(sums)
    drawing_count = 0
    read_count = 0
    last_time = None
    strokes = set()
    numeric_fields = None
    with (output / "trajectory.csv").open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"time", "pen_down", "stroke_index", "disturbance_active",
                    *[f"{prefix}_{axis}" for prefix in ("target", "actual") for axis in "xyz"],
                    *[f"external_force_{axis}_n" for axis in "xyz"]}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("trajectory.csv 缺少时间、位置、笔状态或外力列。")
        numeric_fields = [key for key in reader.fieldnames if key not in {"pen_down", "disturbance_active"}]
        for index, row in enumerate(reader):
            values = {key: _finite(row.get(key), f"CSV row {index} {key}") for key in numeric_fields}
            t = values["time"]
            if not math.isclose(t, index * dt, rel_tol=0, abs_tol=1e-9):
                raise ValueError(f"CSV 时间与物理步不匹配：row {index}")
            down = _bool(row["pen_down"], "pen_down")
            active = _bool(row["disturbance_active"], "disturbance_active")
            force = [values[f"external_force_{axis}_n"] for axis in "xyz"]
            if item["condition"] == "normal" and (active or any(force)):
                raise ValueError("正常实验记录了外力。")
            if active and not down:
                raise ValueError("外力脉冲发生在抬笔时段。")
            if down:
                drawing_count += 1
                strokes.add(int(values["stroke_index"]))
            error = [values[f"actual_{a}"] - values[f"target_{a}"] for a in "xyz"]
            tip_squared = sum(x * x for x in error)
            xy_squared = sum(x * x for x in error[:2])
            for key, squared in (("tip", tip_squared), ("xy", xy_squared)):
                sums[key] += squared
                maxima[key] = max(maxima[key], math.sqrt(squared))
                if down:
                    sums[f"drawing_{key}"] += squared
                    maxima[f"drawing_{key}"] = max(maxima[f"drawing_{key}"], math.sqrt(squared))
            last_time = t
            read_count += 1
    if read_count != count or drawing_count != metrics.get("drawing_sample_count"):
        raise ValueError("CSV 实际样本数/落笔样本数与 metrics 不一致。")
    if not strokes or -1 in strokes or last_time < duration - 1e-9:
        raise ValueError("CSV 未覆盖完整的落笔轨迹。")
    for key in sums:
        divisor = drawing_count if key.startswith("drawing_") else count
        for suffix, expected in (("rmse_m", math.sqrt(sums[key] / divisor)),
                                 ("max_error_m", maxima[key])):
            measured = _finite(metrics.get(f"{key}_{suffix}"), f"{key}_{suffix}")
            if not math.isclose(measured, expected, rel_tol=1e-9, abs_tol=1e-12):
                raise ValueError(f"CSV 重算 {key}_{suffix} 与 metrics 不一致。")
    with zipfile.ZipFile(output / "reference_joints.npz") as archive:
        if not {"time.npy", "q.npy", "dq.npy", "tip.npy"}.issubset(archive.namelist()) or archive.testzip():
            raise ValueError("reference_joints.npz 不完整或损坏。")
    if not (output / "reference_cartesian.csv").is_file():
        raise ValueError("缺少笛卡尔参考轨迹。")
    video = metadata.get("video")
    if not isinstance(video, dict) or video.get("completed") is not True:
        raise ValueError("视频录制未完整完成。")
    sidecar = video_path.with_suffix(".recording.json")
    if _load_json(sidecar) != video:
        raise ValueError("视频独立元数据与实验元数据不一致。")
    if Path(video.get("path", "")).resolve() != video_path.resolve():
        raise ValueError("视频路径与计划不一致。")
    frames = _positive_int(video.get("frame_count"), "frame_count")
    motion_frames = _positive_int(video.get("motion_frame_count"), "motion_frame_count")
    hold_frames = video.get("hold_frame_count")
    if isinstance(hold_frames, bool) or not isinstance(hold_frames, int) or hold_frames < 0:
        raise ValueError("hold_frame_count 需要非负整数。")
    if frames != motion_frames + hold_frames or video.get("dropped_frames") != 0 or video.get("no_dropped_frames") is not True:
        raise ValueError("视频帧计数不一致或存在丢帧。")
    fps = _finite(video.get("fps"), "video fps", positive=True)
    if fps != resolved["video_fps"] or video.get("resolution") != [resolved["video_width"], resolved["video_height"]]:
        raise ValueError("视频帧率/分辨率与计划不一致。")
    if not math.isclose(_finite(video.get("duration_s"), "video duration"), frames / fps, rel_tol=0, abs_tol=1e-9):
        raise ValueError("视频时长与帧数不一致。")
    if not math.isclose(_finite(video.get("last_sim_time_s"), "last video time"), last_time, rel_tol=0, abs_tol=1e-9):
        raise ValueError("视频没有覆盖最后一个物理样本。")
    container = validate_mp4(video_path)
    if container["file_size_bytes"] != video.get("file_size_bytes"):
        raise ValueError("MP4 文件大小与录制元数据不一致。")
    inspection = inspect_mp4(video_path)
    if inspection.get("validated") is not True or inspection.get("recording_consistent") is not True:
        raise ValueError(f"MP4 的实际帧数/编解码/时间表验证失败：{inspection.get('errors')}")
    if inspection.get("media_duration_s") is None or inspection.get("movie_duration_s") is None:
        raise ValueError("MP4 仍有未知时长的 mdhd/mvhd，不能标记为播放验证成功。")
    return {"metadata": metadata, "independent_validation": {
        "csv_sample_count": read_count, "drawing_sample_count": drawing_count,
        "drawing_stroke_count": len(strokes), "nominal_physics_elapsed_s": count * dt,
        "physics_elapsed_s": elapsed, "clock_absolute_error_s": abs(elapsed - count * dt),
        "time_aligned_metrics_recomputed": True, "video_container": container},
        "video_validation": inspection}


def artifact_hashes(item: dict[str, Any], pngs=()) -> dict[str, str]:
    output, video = Path(item["output"]), Path(item["video"])
    paths = [output / name for name in ("metrics.json", "trajectory.csv", "reference_cartesian.csv",
             "reference_joints.npz", "resolved_config.json", "controller_state.json", "scene.usda")]
    if (output / "controller_state.npz").is_file():
        paths.append(output / "controller_state.npz")
    paths.extend([video, video.with_suffix(".recording.json"), *sorted(output.glob("*.svg")),
                  *[Path(path) for path in pngs]])
    if item.get("pictures"):
        paths.extend(path for path in sorted(Path(item["pictures"]).glob("*")) if path.suffix in {".svg", ".png"})
    return {str(path.resolve()): file_sha256(path) for path in paths}


def copy_figures(source: str | Path, destination: str | Path) -> list[str]:
    """Mirror editable SVG and PNG figures into the requested pictures folder."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    paths = []
    for svg in sorted(Path(source).glob("*.svg")):
        for original in (svg, svg.with_suffix(".png")):
            if not original.is_file():
                raise FileNotFoundError(f"图形尚未导出：{original}")
            target = destination / original.name
            if original.resolve() != target.resolve():
                shutil.copyfile(original, target)
            paths.append(str(target.resolve()))
    return paths


def can_resume(record: dict[str, Any] | None, item: dict[str, Any], signature: str) -> bool:
    if not isinstance(record, dict) or record.get("status") != "completed" or record.get("exit_code") != 0:
        return False
    if record.get("signature_sha256") != signature or not record.get("artifact_sha256"):
        return False
    if record.get("pictures") != item.get("pictures"):
        return False
    output, video = Path(item["output"]), Path(item["video"])
    mandatory = {str((output / name).resolve()) for name in (
        "metrics.json", "trajectory.csv", "reference_cartesian.csv", "reference_joints.npz", "resolved_config.json")}
    mandatory.update((str(video.resolve()), str(video.with_suffix(".recording.json").resolve())))
    if not mandatory.issubset(record["artifact_sha256"]):
        return False
    try:
        if any(file_sha256(path) != digest for path, digest in record["artifact_sha256"].items()):
            return False
        validate_run(item)
    except (OSError, ValueError, KeyError, zipfile.BadZipFile):
        return False
    return True


def write_summary(path: Path, manifest: dict[str, Any]) -> None:
    fields = ["controller", "condition", "status", "exit_code", "sample_count", "drawing_sample_count",
              "drawing_stroke_count", "physics_dt_s", "physics_elapsed_s", *ERROR_FIELDS,
              "torque_saturation_fraction", "normal_tracking_drawing_tip_rmse_m",
              "disturbed_tracking_drawing_tip_rmse_m", "disturbance_deviation_drawing_tip_rmse_m",
              "normal_tracking_tip_rmse_m", "disturbed_tracking_tip_rmse_m", "disturbance_deviation_tip_rmse_m",
              "output", "metrics_path", "trajectory_path", "video_path", "log_path", "comparison_metrics_path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in manifest["runs"].values():
            row = {key: record.get(key) for key in ("controller", "condition", "status", "exit_code", "output")}
            output = Path(record["output"])
            row.update(metrics_path=str(output / "metrics.json"), trajectory_path=str(output / "trajectory.csv"),
                       video_path=record["video"], log_path=record["log"])
            metadata = record.get("metadata", {})
            metrics = metadata.get("metrics", {})
            row.update({key: metrics.get(key) for key in ("sample_count", "drawing_sample_count", *ERROR_FIELDS, "torque_saturation_fraction")})
            row.update(physics_dt_s=metadata.get("physics_dt_s"), physics_elapsed_s=metadata.get("physics_elapsed_s"),
                       drawing_stroke_count=record.get("independent_validation", {}).get("drawing_stroke_count"))
            comparison = manifest.get("comparisons", {}).get(record["controller"], {})
            if comparison.get("status") == "completed":
                row["comparison_metrics_path"] = str(Path(comparison["output"]) / "comparison_metrics.json")
                for name, values in comparison["metadata"]["metrics"].items():
                    for metric in ("drawing_tip_rmse_m", "tip_rmse_m"):
                        row[f"{name}_{metric}"] = values[metric]
            writer.writerow(row)
    temporary.replace(path)


def run_batch(args: argparse.Namespace) -> int:
    plan = build_plan(args)
    if args.plan:
        print(json.dumps({"serial_run_count": len(plan), "plan": plan}, ensure_ascii=False, indent=2))
        return 0
    from raa.runtime import locate_isaac
    isaac = locate_isaac(args.isaac_root)
    browser = locate_chrome(args.chrome)
    sources = source_fingerprint(isaac)
    destination = args.output_root.resolve()
    manifest_path = destination / "experiment_manifest.json"
    if args.resume and manifest_path.is_file():
        manifest = _load_json(manifest_path)
        if manifest.get("format_version") != 1 or not isinstance(manifest.get("runs"), dict):
            raise ValueError("已有实验 manifest 的格式不支持恢复。")
    else:
        manifest = {"format_version": 1, "project": "robot arm artist", "version": "raa-v0",
                    "created_at": _now(), "runs": {}, "comparisons": {}}
    manifest.update(status="running", started_at=_now(), source=sources,
                    isaac_root=str(isaac), requested_runs=[item["id"] for item in plan],
                    common_config_sha256=plan[0]["common_config_sha256"],
                    summary_path=str(destination / "summary.csv"))
    manifest.pop("failure", None)
    reference_hash = None
    _write_json(manifest_path, manifest)
    active_record = None
    active_comparison = None
    try:
        for index, item in enumerate(plan, 1):
            signature = _digest({"source": sources["sha256"], "run": item["resolved_config_sha256"],
                                 "common": item["common_config_sha256"], "isaac_root": str(isaac)})
            previous = manifest["runs"].get(item["id"])
            if args.resume and can_resume(previous, item, signature):
                record = previous
                print(f"[raa-v0] {index}/{len(plan)} 已验证并复用：{item['id']}", flush=True)
            else:
                record = {**item, "status": "running", "started_at": _now(), "signature_sha256": signature}
                manifest["runs"][item["id"]] = record
                active_record = record
                log = Path(item["log"])
                log.parent.mkdir(parents=True, exist_ok=True)
                _write_json(manifest_path, manifest)
                print(f"[raa-v0] {index}/{len(plan)} 开始 {item['id']}；日志：{log}", flush=True)
                with log.open("w", encoding="utf-8") as stream:
                    stream.write(json.dumps({"started_at": record["started_at"], "command": item["command"]}, ensure_ascii=False) + "\n")
                    stream.flush()
                    result = subprocess.run(item["command"], stdout=stream, stderr=subprocess.STDOUT,
                                            cwd=ROOT.parent, check=False)
                record["exit_code"] = result.returncode
                if result.returncode != 0:
                    raise RuntimeError(f"{item['id']} 子进程退出码 {result.returncode}；请查看 {log}")
                validation = validate_run(item)
                record.update(validation)
                # A change while a simulation is running invalidates the experiment signature.
                if source_fingerprint(isaac)["sha256"] != sources["sha256"]:
                    raise RuntimeError("实验运行期间源码/预设/机器人输入发生修改，本组不能作为同版本实验继续。")
                pngs = export_directory(item["output"], chrome=browser)
                record["picture_paths"] = copy_figures(item["output"], item["pictures"])
                record.update(png_paths=[str(path) for path in pngs], artifact_sha256=artifact_hashes(item, pngs),
                              status="completed", finished_at=_now())
                print(f"[raa-v0] 已完成并核验：{item['id']}，{record['metadata']['metrics']['sample_count']} 个样本", flush=True)
            current_reference = file_sha256(Path(item["output"]) / "reference_cartesian.csv")
            if reference_hash is None:
                reference_hash = current_reference
            elif current_reference != reference_hash:
                raise ValueError("各实验的笛卡尔参考轨迹哈希不一致。")
            manifest["reference_cartesian_sha256"] = reference_hash
            _write_json(manifest_path, manifest)
            write_summary(destination / "summary.csv", manifest)
            active_record = None
        for controller in args.controllers:
            normal = manifest["runs"].get(f"{controller}_normal")
            disturbed = manifest["runs"].get(f"{controller}_disturbed")
            if not all(isinstance(record, dict) and record.get("status") == "completed" for record in (normal, disturbed)):
                continue
            if any(record["common_config_sha256"] != plan[0]["common_config_sha256"] for record in (normal, disturbed)):
                continue
            paired_valid = all(can_resume(record, record, _digest({"source": sources["sha256"],
                    "run": record["resolved_config_sha256"], "common": record["common_config_sha256"],
                    "isaac_root": str(isaac)})) for record in (normal, disturbed))
            if not paired_valid:
                if set(args.conditions) == set(CONDITIONS):
                    raise ValueError(f"{controller} 已完成实验的产物发生变化，不能生成可信的正常/受扰比较。")
                continue
            comparison_dir = destination / "comparisons" / controller
            manifest["comparisons"][controller] = {"status": "running", "output": str(comparison_dir)}
            active_comparison = manifest["comparisons"][controller]
            _write_json(manifest_path, manifest)
            print(f"[raa-v0] 按相同时刻比较：{controller}", flush=True)
            comparison = compare_runs(normal["output"], disturbed["output"], comparison_dir)
            pngs = export_directory(comparison_dir, chrome=browser)
            comparison_pictures = copy_figures(comparison_dir, args.pictures_root / "comparisons" / controller)
            manifest["comparisons"][controller] = {"status": "completed", "output": str(comparison_dir),
                "metadata": comparison, "png_paths": [str(path) for path in pngs], "picture_paths": comparison_pictures,
                "artifact_sha256": {str(path): file_sha256(path) for path in sorted(comparison_dir.iterdir()) if path.is_file()}}
            _write_json(manifest_path, manifest)
            write_summary(destination / "summary.csv", manifest)
            active_comparison = None
        manifest.update(status="completed", finished_at=_now())
        _write_json(manifest_path, manifest)
        write_summary(destination / "summary.csv", manifest)
        print(f"[raa-v0] 实验记录：{manifest_path}\n[raa-v0] 误差汇总：{destination / 'summary.csv'}", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if active_record is not None:
            active_record.update(status="failed", finished_at=_now(), failure=str(exc))
        if active_comparison is not None:
            active_comparison.update(status="failed", finished_at=_now(), failure=str(exc))
        manifest.update(status="failed", finished_at=_now(), failure=str(exc))
        _write_json(manifest_path, manifest)
        write_summary(destination / "summary.csv", manifest)
        print(f"[raa-v0] 批量实验停止：{exc}", file=sys.stderr, flush=True)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="raa-v0：串行运行四控制器正常/受扰实验，录制八个 MP4 并导出 SVG/PNG 对比图。")
    result.add_argument("--controllers", choices=CONTROLLERS, nargs="+", default=list(CONTROLLERS))
    result.add_argument("--conditions", choices=CONDITIONS, nargs="+", default=list(CONDITIONS))
    result.add_argument("--output-root", type=Path, default=ROOT / "output")
    result.add_argument("--video-root", type=Path, default=ROOT / "video")
    result.add_argument("--pictures-root", type=Path, default=ROOT / "pictures")
    result.add_argument("--isaac-root", type=Path)
    result.add_argument("--chrome", help="已有 Chrome 可执行文件；默认自动查找")
    result.add_argument("--resume", action="store_true", help="只复用 manifest 中源码、配置、参考轨迹、数据与视频哈希一致的已完成实验")
    result.add_argument("--plan", action="store_true", help="打印八组命令和最终配置后退出，不启动 Isaac 或 Chrome")
    return result


def main() -> int:
    args = parser().parse_args()
    if len(set(args.controllers)) != len(args.controllers) or len(set(args.conditions)) != len(args.conditions):
        raise ValueError("controllers / conditions 不能包含重复项目。")
    return run_batch(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, FileNotFoundError) as exc:
        print(f"[raa-v0] {exc}", file=sys.stderr)
        raise SystemExit(2)
