"""Compose the 132 s L0/L2/L4 presentation movie from real Isaac recordings.

    python3 Pro2/output/final/build_demo_video.py --prepare
    python3 Pro2/output/final/build_demo_video.py

--prepare renders complete available sections while the remaining recordings
are being produced. This program never starts or changes a simulation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import shutil
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
ASSETS = OUT / "assets"
WORK = ASSETS / "video_build_L0_L2_L4"
TARGET = ASSETS / "项目验收演示_L0_L2_L4.mp4"
SIZE = (1920, 1080)
FPS = 30
TOTAL_SECONDS = 132
PUSH_CONFIG = ROOT / "Pro2/experiments/L2/lateral_push/push_100N.json"
FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
BG = "#FAFBFC"
INK = "#172B36"
TEAL = "#006D68"
# Complete 4:3 images, with labels above each row and a separate subtitle area.
QUAD_PANES = ((348, 144), (1028, 144), (348, 604), (1028, 604))
QUAD_SIZE = (544, 408)
TERRAIN_CONFIGS = {
    key: ROOT / f"Pro2/experiments/L4/terrain/{key}.json"
    for key in ("slope_05deg", "rough_medium", "random_obstacles_seed01_full", "blind_step_up")
}

INPUTS = {
    "l0": "Pro2/output/L0/walk_10s_video/demo.mp4",
    **{f"weight_{key}": f"Pro2/output/L2/ppt_demo_20261104/{key}/demo.mp4"
       for key in ("low", "baseline", "high")},
    **{f"horizon_{key}": f"Pro2/output/L2/prediction_horizon/ppt_video_20261104/{key}/demo.mp4"
       for key in ("short_n5", "baseline_n10", "long_n15")},
    **{f"step_{key}": f"Pro2/output/L2/step_height/ppt_video_20261104/{key}/demo.mp4"
       for key in ("low_008", "baseline_012", "high_016")},
    **{f"push_{key}": f"Pro2/output/L2/lateral_push/long_push_20261105/{key}/demo.mp4"
       for key in ("no_push", "push_50N", "push_100N")},
    "path": "Pro2/output/L4/02_path_tracking/ppt_video_20261104/waypoints/demo.mp4",
    **{key: f"Pro2/output/L4/03_terrain_and_jump/ppt_video_20261104/{key}/demo.mp4"
       for key in ("rough_medium", "slope_05deg", "vertical_jump")},
    **{key: f"Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/{key}/demo.mp4"
       for key in ("random_obstacles_seed01_full", "blind_step_up")},
}


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def ffmpeg_path() -> str:
    found = shutil.which("ffmpeg")
    if found:
        return found
    for name in ("rllab", "amp-env"):
        candidates = list((Path.home() / "miniconda3/envs" / name).glob(
            "lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux*"))
        if candidates:
            return str(candidates[0])
    raise RuntimeError("未发现 FFmpeg。")


def run_ffmpeg(ffmpeg: str, args: list[str], log: Path) -> str:
    result = subprocess.run([ffmpeg, "-hide_banner", "-nostdin", "-y", *args],
                            capture_output=True, text=True)
    log.write_text(result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"FFmpeg failed ({result.returncode}); see {log}\n{result.stderr[-2500:]}")
    return result.stdout


def probe(ffmpeg: str, path: Path) -> dict:
    result = subprocess.run([ffmpeg, "-hide_banner", "-i", str(path)],
                            capture_output=True, text=True)
    body = result.stderr
    duration = re.search(r"Duration:\s*(\d+):(\d+):(\d+(?:\.\d+)?)", body)
    line = next((x for x in body.splitlines() if "Video:" in x), "")
    dimensions = re.search(r"(?<!\d)(\d{2,5})x(\d{2,5})(?!\d)", line)
    fps = re.search(r"(\d+(?:\.\d+)?) fps", line)
    if not (duration and dimensions and fps):
        raise RuntimeError(f"Unable to read video metadata: {path}\n{body}")
    return {
        "duration_s": 3600 * int(duration[1]) + 60 * int(duration[2]) + float(duration[3]),
        "width": int(dimensions[1]), "height": int(dimensions[2]), "fps": float(fps[1]),
        "codec": line.split("Video:", 1)[1].split("(", 1)[0].split(",", 1)[0].strip(),
    }


def plan() -> list[dict]:
    """The user-specified sections; time maps are explicit and reproducible."""
    push = json.loads(PUSH_CONFIG.read_text())
    push_start, push_duration = push["push_at"], push["push_duration"]
    push_end = push_start + push_duration
    specs = [
        ("l0", 10, "L0｜跑通开源人形机器人控制示例", ["l0"], [],
         "保留 OpenLoong MPC / WBC 控制栈，在 Isaac Sim 中完成行走"),
        ("l2_weight", 12, "L2｜实验一：前向速度代价权重", 
         ["weight_low", "weight_baseline", "weight_high"],
         ["低权重 0.2", "基线 1.0", "高权重 5.0"],
         "只改变速度误差权重；三组从同一仿真时刻同步播放"),
        ("l2_horizon", 12, "L2｜实验二：预测时域", 
         ["horizon_short_n5", "horizon_baseline_n10", "horizon_long_n15"],
         ["短时域 N = 5", "基线 N = 10", "长时域 N = 15"],
         "只改变预测步数；比较行走过程中的姿态与动作"),
        ("l2_step", 12, "L2｜实验三：抬腿高度", 
         ["step_low_008", "step_baseline_012", "step_high_016"],
         ["低抬腿 0.08 m", "基线 0.12 m", "高抬腿 0.16 m"],
         "只改变抬腿高度；比较足端抬起和落下的动作"),
        ("l2_push", 12, "L2｜实验四：侧向扰动", 
         ["push_no_push", "push_push_50N", "push_push_100N"],
         ["无侧推 0 N", "侧推 50 N", "侧推 100 N"],
         f"{push_start:g}–{push_end:g} 秒持续施加侧向力；橙红箭头显示受力方向和大小"),
        ("l4_path", 30, "L4｜给定路径跟踪", ["path"], [],
         "青色线是给定路径；机器人根据位置和航向误差闭环跟踪"),
        ("l4_terrain_quad", 24, "L4｜四种地形行走", 
         ["slope_05deg", "rough_medium", "random_obstacles_seed01_full", "blind_step_up"],
         ["5° 坡道", "粗糙地面", "随机障碍", "6 cm 台阶"],
         "同一仿真时刻同步播放；展示各组前 24 秒行走过程"),
        ("l4_jump", 13, "L4｜竖直跳跃与落地恢复", ["vertical_jump"], [],
         "下蹲准备 → 蹬伸起跳 → 腾空落地 → 恢复站立"),
        ("l4_jump_replay", 7, "L4｜跳跃回放 · 0.5× 慢放", ["vertical_jump"], [],
         "原视频 8.0–11.5 秒，以 0.5× 速度回放；观察离地、落地和恢复"),
    ]
    elapsed = 0
    result = []
    for number, (name, duration, title, keys, labels, caption) in enumerate(specs, 1):
        replay = name == "l4_jump_replay"
        item = {
            "number": number, "name": name, "title": title,
            "start_s": elapsed, "duration_s": duration, "end_s": elapsed + duration,
            "kind": {1: "single", 3: "triple", 4: "quad"}[len(keys)], "video_keys": keys,
            "condition_labels": labels, "caption": caption,
            "speed": 0.5 if replay else 1.0, "source_start_s": 8.0 if replay else 0.0,
            "source_end_s": 11.5 if replay else float(duration),
            "sources": [INPUTS[key] for key in keys],
        }
        result.append(item)
        if name == "l2_push":
            item["push_at_s"] = push_start
            item["push_end_s"] = push_end
            item["push_duration_s"] = push_duration
            run_path = (ROOT / INPUTS["push_push_100N"]).parent / "run.json"
            if run_path.is_file():
                run = json.loads(run_path.read_text())
                if is_recorded_instability(run):
                    item["stopped_panel"] = {
                        "video_key": "push_push_100N", "panel_index": 2,
                        "freeze_at_s": run["video_duration_s"],
                        "simulation_stop_s": run["termination_state"]["time_s"],
                        "reason": "physical_instability",
                    }
        elapsed += duration
    assert elapsed == TOTAL_SECONDS
    return result


def centered_text(draw, center_x, y, value, size, *, color=INK, bold=False):
    selected = BOLD if bold else FONT
    selected_font = ImageFont.truetype(str(selected), size)
    draw.text((center_x, y), value, font=selected_font, fill=color, anchor="mt")


def prepare_background(seg: dict) -> Path:
    background = Image.new("RGB", SIZE, BG)
    draw = ImageDraw.Draw(background)
    draw.text((56, 36), seg["title"], font=ImageFont.truetype(str(BOLD), 50), fill=INK)
    if seg["kind"] == "triple":
        for x, label in zip((16, 648, 1280), seg["condition_labels"]):
            centered_text(draw, x + 312, 190, label, 44, color=TEAL, bold=True)
    elif seg["kind"] == "quad":
        for (x, y), label in zip(QUAD_PANES, seg["condition_labels"]):
            centered_text(draw, x + QUAD_SIZE[0] / 2, y - 44, label, 36,
                          color=TEAL, bold=True)
    path = WORK / f"{seg['name']}_background.png"
    background.save(path)
    return path


def ass_time(seconds: float) -> str:
    total = int(round(seconds * 100))
    return f"{total // 360000}:{(total // 6000) % 60:02}:{(total // 100) % 60:02}.{total % 100:02}"


def caption_events(seg: dict) -> list[tuple[float, float, str]]:
    if seg["name"] == "l0":
        return [(0, 3, "L0 跑通：初始化并建立站姿"),
                (3, 10, "L0 跑通：保留 OpenLoong MPC / WBC，在 Isaac Sim 中完成行走")]
    if seg["name"] == "l2_push":
        start, end = seg["push_at_s"], seg["push_end_s"]
        events = [(0, start, f"三组同步行走；第 {start:g} 秒施加侧向力，持续 {seg['push_duration_s']:g} 秒"),
                (start, end, "橙红箭头：世界 +Y 外力；右侧两组分别受到 50 N / 100 N 侧推"),
                (end, 12, "外力已结束；比较无侧推、50 N 与 100 N 三组的横向偏移和稳定性")]
        if "stopped_panel" in seg:
            stop = seg["stopped_panel"]["freeze_at_s"]
            events[-1] = (end, stop, events[-1][2])
            events.append((stop, 12, "100 N 已失稳停仿并冻结末帧；无侧推与 50 N 两组继续运行"))
        return events
    if seg["name"] == "l4_jump":
        return [(0, 3, "准备：建立稳定站姿"),
                (3, 8.5, "下蹲：降低身体高度，为蹬伸起跳做准备"),
                (8.5, 9.1, "蹬伸起跳 → 双脚腾空 → 接触落地"),
                (9.1, 13, "落地后恢复姿态与站立高度")]
    return [(0, seg["duration_s"], seg["caption"])]


def prepare_subtitles(seg: dict) -> Path:
    # ASS is burned into the picture. No subtitle player or font support is
    # required when the resulting MP4 is embedded in a presentation.
    lines = [
        "[Script Info]", "ScriptType: v4.00+", "PlayResX: 1920", "PlayResY: 1080",
        "WrapStyle: 2", "ScaledBorderAndShadow: yes", "", "[V4+ Styles]",
        "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding",
        "Style: Caption,Noto Sans CJK SC,42,&H00362B17,&H00362B17,&H00FCFBFA,&H00FCFBFA,0,0,0,0,100,100,0,0,1,0,0,2,48,48,38,1",
        "Style: Clock,Noto Sans CJK SC,36,&H00686D00,&H00686D00,&H00FCFBFA,&H00FCFBFA,0,0,0,0,100,100,0,0,1,0,0,9,56,56,40,1",
        "Style: Cue,Noto Sans CJK SC,42,&H00686D00,&H00686D00,&H00FCFBFA,&H00FCFBFA,-1,0,0,0,100,100,0,0,1,0,0,8,48,48,800,1",
        "", "[Events]", "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text",
    ]
    if seg["kind"] == "quad":
        # The second image row ends at y = 1012. Keep the caption below it;
        # changing this style only for quad preserves every other existing encode.
        lines = [line.replace(
            "Style: Caption,Noto Sans CJK SC,42,", "Style: Caption,Noto Sans CJK SC,40,")
            .replace(",2,48,48,38,1", ",2,48,48,16,1")
            if line.startswith("Style: Caption,") else line for line in lines]
    if seg["name"] == "l2_push":
        lines.insert(lines.index("[Events]") - 1,
            "Style: Stop,Noto Sans CJK SC,38,&H00FFFFFF,&H00FFFFFF,&H0020209B,&H0020209B,-1,0,0,0,100,100,0,0,3,10,0,5,0,0,0,1")
    for start, end, caption in caption_events(seg):
        lines.append(f"Dialogue: 0,{ass_time(start)},{ass_time(end)},Caption,,0,0,0,,{caption}")
    for tick in range(int(seg["duration_s"] * 10)):
        source_time = seg["source_start_s"] + tick / 10 * seg["speed"]
        text = f"仿真时间 {source_time:04.1f} s"
        lines.append(f"Dialogue: 1,{ass_time(tick / 10)},{ass_time((tick + 1) / 10)},Clock,,0,0,0,,{text}")
    if seg["name"] == "l2_push":
        start, end = seg["push_at_s"], seg["push_end_s"]
        lines.extend([
            f"Dialogue: 2,{ass_time(start)},{ass_time(end)},Cue,,0,0,0,,侧向力施加中：0 N / 50 N / 100 N",
            f"Dialogue: 2,{ass_time(end)},0:00:09.00,Cue,,0,0,0,,{start:g}–{end:g} s 侧推已结束，继续观察响应",
        ])
        if "stopped_panel" in seg:
            stopped = seg["stopped_panel"]
            label = (r"{\an5\pos(1592,395)}100 N：失稳保护\N仿真已停止于 "
                     + f"{stopped['simulation_stop_s']:.3f} s" + r"\N当前画面为停止帧")
            lines.append(f"Dialogue: 3,{ass_time(stopped['freeze_at_s'])},{ass_time(seg['duration_s'])},Stop,,0,0,0,,{label}")
    path = WORK / f"{seg['name']}.ass"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def is_recorded_instability(run: dict) -> bool:
    return (run.get("status") == "failed" and run.get("termination_reason") == "physical_instability"
            and run.get("video_saved") is True and run.get("video_complete") is False
            and isinstance(run.get("termination_state", {}).get("time_s"), (int, float))
            and run.get("video_duration_s", 0) > 0)


def complete_input(path: Path) -> bool:
    if not path.is_file():
        return False
    manifest = path.parent / "run.json"
    if not manifest.is_file():
        return False
    try:
        data = json.loads(manifest.read_text())
    except (OSError, ValueError):
        return False
    return ((data.get("status") == "completed" and data.get("video_saved") is True)
            or (path == ROOT / INPUTS["push_push_100N"] and is_recorded_instability(data)))


def validate_source(ffmpeg: str, key: str, end_time: float):
    path = ROOT / INPUTS[key]
    metadata = probe(ffmpeg, path)
    data = json.loads((path.parent / "run.json").read_text())
    partial = key == "push_push_100N" and is_recorded_instability(data)
    if metadata["duration_s"] + 0.005 < end_time and not partial:
        raise RuntimeError(f"录像时间不足：{path}: {metadata['duration_s']} < {end_time}")
    if partial:
        assert abs(metadata["duration_s"] - data["video_duration_s"]) < .02, path
        assert data["termination_state"]["time_s"] <= data["video_duration_s"] + 1 / FPS, path
    config = data["config"]
    if key.startswith("weight_"):
        expected = {"weight_low": .2, "weight_baseline": 1., "weight_high": 5.}[key]
        assert math.isclose(config["state_weights"][9], expected), path
    if key.startswith("horizon_"):
        expected = {"horizon_short_n5": 5, "horizon_baseline_n10": 10, "horizon_long_n15": 15}[key]
        assert config["mpc_horizon_steps"] == expected, path
    if key.startswith("step_"):
        expected = {"step_low_008": .08, "step_baseline_012": .12, "step_high_016": .16}[key]
        assert math.isclose(config["step_height"], expected), path
    if key.startswith("push_"):
        expected_push = json.loads(PUSH_CONFIG.read_text())
        expected = {"push_no_push": 0., "push_push_50N": 50., "push_push_100N": 100.}[key]
        if expected:
            assert math.isclose(config["push_at"], 7.), path
            assert math.isclose(config["push_duration"], expected_push["push_duration"]), path
            assert config["push_force"] == [0., expected, 0.], path
        else:
            assert config["push_at"] is None or config["push_force"] == [0., 0., 0.], path
    if key == "path":
        assert data.get("path_visualization", {}).get("enabled") is True, path
    if key in TERRAIN_CONFIGS:
        expected = json.loads(TERRAIN_CONFIGS[key].read_text())
        assert config["mode"] == "walk" and data.get("status") == "completed", path
        for field in ("duration", "walk_start", "vx", "step_height"):
            assert math.isclose(config[field], expected[field]), (path, field)
        for field, value in expected["terrain"].items():
            assert config["terrain"].get(field) == value, (path, "terrain", field)
    if key == "vertical_jump":
        jump = json.loads((path.parent / "jump_metrics.json").read_text())
        assert jump.get("pass") is True and jump.get("external_force_assist") is False, path


def filter_path(path: Path) -> str:
    return str(path).replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def local_render_fingerprint(fingerprint: dict) -> dict:
    """Ignore placement in the complete movie, while retaining local content.

    Inserting or removing a chapter changes its following chapter numbers and
    global times, but not their pixels. Support the existing version-1 cache so
    those chapters can be copied byte-for-byte instead of encoded again.
    """
    result = dict(fingerprint)
    if result.get("script_sha256") == "530f375f83352f77eac001f6270b48357827e31c011f3df7b47f10c584b48d43":
        result.pop("script_sha256")
        result["renderer_version"] = 1
    result["plan"] = {key: value for key, value in result["plan"].items()
                      if key not in ("number", "start_s", "end_s")}
    return result


def render_segment(ffmpeg: str, seg: dict, force=False) -> Path:
    background = prepare_background(seg)
    subtitles = prepare_subtitles(seg)
    target = WORK / f"segment_{seg['number']:02d}_{seg['name']}.mp4"
    fingerprint_path = target.with_suffix(".json")
    paths = [background, subtitles] + [ROOT / value for value in seg["sources"]]
    # The existing single/triple renderer is unchanged. Global timeline offsets
    # do not enter the fingerprint: all eight unaffected movies stay identical.
    fingerprint = local_render_fingerprint({
        "renderer_version": 1, "plan": seg,
        "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in paths}})
    if not force:
        candidates = [target] + sorted(path for path in WORK.glob(f"segment_*_{seg['name']}.mp4")
                                       if path != target)
        for cached in candidates:
            cached_metadata = cached.with_suffix(".json")
            if not cached.is_file() or not cached_metadata.is_file():
                continue
            previous = json.loads(cached_metadata.read_text())
            if local_render_fingerprint(previous) == fingerprint:
                if cached != target:
                    shutil.copyfile(cached, target)
                fingerprint_path.write_text(json.dumps(fingerprint, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Reuse {target.name}" + (f" (byte copy of {cached.name})" if cached != target else ""), flush=True)
                return target
    for key in seg["video_keys"]:
        validate_source(ffmpeg, key, seg["source_end_s"])
    args = ["-loop", "1", "-framerate", str(FPS), "-i", str(background)]
    for key in seg["video_keys"]:
        args.extend(["-i", str(ROOT / INPUTS[key])])
    filters = []
    for number, key in enumerate(seg["video_keys"], 1):
        if seg["kind"] == "triple":
            width, height, x, y = 624, 468, (16, 648, 1280)[number - 1], 278
        elif seg["kind"] == "quad":
            width, height = QUAD_SIZE
            x, y = QUAD_PANES[number - 1]
        else:
            width, height, x, y = 1120, 840, 400, 132
        freeze = ""
        if seg.get("stopped_panel", {}).get("video_key") == key:
            # Freeze only this explicitly labelled failed pane. Raw source
            # footage remains its actual short duration and is never padded.
            freeze = f"tpad=stop_mode=clone:stop_duration={seg['duration_s'] - seg['stopped_panel']['freeze_at_s']},"
        filters.append(
            f"[{number}:v]trim=start={seg['source_start_s']}:end={seg['source_end_s']},"
            f"setpts=(PTS-STARTPTS)/{seg['speed']},fps={FPS},{freeze}"
            f"scale={width}:{height}:force_original_aspect_ratio=decrease:flags=lanczos,"
            f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2:color={BG},setsar=1[v{number}]")
        prior = "0:v" if number == 1 else f"s{number - 1}"
        filters.append(f"[{prior}][v{number}]overlay={x}:{y}:shortest=1[s{number}]")
    filters.append(f"[s{len(seg['video_keys'])}]ass=filename='{filter_path(subtitles)}'[out]")
    args.extend(["-filter_complex_threads", "1", "-filter_complex", ";".join(filters),
                 "-map", "[out]", "-an", "-t", str(seg["duration_s"]), "-r", str(FPS),
                 "-frames:v", str(seg["duration_s"] * FPS), "-c:v", "libx264",
                 "-preset", "fast", "-crf", "20", "-threads", "2",
                 "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target)])
    print(f"Render {target.name} ({seg['duration_s']} s)", flush=True)
    run_ffmpeg(ffmpeg, args, target.with_suffix(".log"))
    metadata = probe(ffmpeg, target)
    if abs(metadata["duration_s"] - seg["duration_s"]) > .005:
        raise RuntimeError(f"Unexpected segment duration: {target}: {metadata}")
    fingerprint_path.write_text(json.dumps(fingerprint, ensure_ascii=False, indent=2), encoding="utf-8")
    return target


def srt_time(seconds: float) -> str:
    total = int(round(seconds * 1000))
    return f"{total // 3600000:02}:{(total // 60000) % 60:02}:{(total // 1000) % 60:02},{total % 1000:03}"


def write_srt(segments: list[dict]) -> Path:
    values = []
    for seg in segments:
        for start, end, text in caption_events(seg):
            values.append(f"{len(values) + 1}\n{srt_time(seg['start_s'] + start)} --> "
                          f"{srt_time(seg['start_s'] + end)}\n{seg['title']}\n{text}\n")
    path = TARGET.with_suffix(".srt")
    path.write_text("\n".join(values), encoding="utf-8")
    return path


def make_poster(ffmpeg: str, first_segment: Path):
    # A frame of actual robot motion, with the same subtitles as the movie.
    poster = ASSETS / "演示视频封面.png"
    run_ffmpeg(ffmpeg, ["-ss", "6", "-i", str(first_segment), "-frames:v", "1", str(poster)],
               WORK / "poster.log")
    shutil.copyfile(poster, ASSETS / "poster.png")


def source_row(ffmpeg: str, key: str) -> dict:
    path = ROOT / INPUTS[key]
    row = {"key": key, "path": INPUTS[key], "sha256": digest(path), **probe(ffmpeg, path)}
    run_path = path.parent / "run.json"
    run = json.loads(run_path.read_text())
    row["provenance"] = ("L0 原验收录像" if key == "l0" else
                         "2026-11-05 延长侧推并可视化外力的正式重测录像" if key.startswith("push_") else
                         "2026-11-05 沿用原地形实验配置补录的真实 Isaac 行走录像" if key in ("random_obstacles_seed01_full", "blind_step_up") else
                         "沿用原实验配置补录的真实 Isaac 仿真录像")
    row["run_manifest"] = {
        "path": str(run_path.relative_to(ROOT)), "sha256": digest(run_path),
        "status": run.get("status"), "video_saved": run.get("video_saved"),
        "controller_library_sha256": run.get("controller_library_sha256"),
    }
    if key.startswith("push_"):
        row["run_manifest"].update({"video_complete": run.get("video_complete"),
                                    "termination_reason": run.get("termination_reason"),
                                    "simulation_duration_s": run.get("simulation_duration_s"),
                                    "termination_state": run.get("termination_state")})
    config_path = path.parent / "config.json"
    if config_path.is_file():
        row["config"] = {"path": str(config_path.relative_to(ROOT)), "sha256": digest(config_path)}
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare", action="store_true", help="Render available completed sections only")
    parser.add_argument("--force", action="store_true", help="Rebuild unchanged sections too")
    args = parser.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    ffmpeg = ffmpeg_path()
    segments = plan()
    missing = [rel for rel in INPUTS.values() if not complete_input(ROOT / rel)]
    (WORK / "timeline.json").write_text(json.dumps(segments, ensure_ascii=False, indent=2), encoding="utf-8")
    subtitles = write_srt(segments)
    if missing and not args.prepare:
        raise SystemExit("等待真实录像完成：\n" + "\n".join(missing))
    rendered = []
    for seg in segments:
        if any(not complete_input(ROOT / INPUTS[key]) for key in seg["video_keys"]):
            print(f"Pending {seg['name']}: source recording still running", flush=True)
            continue
        rendered.append(render_segment(ffmpeg, seg, args.force))
        if seg["number"] == 1:
            make_poster(ffmpeg, rendered[-1])
    if args.prepare:
        print(f"Prepared {len(rendered)}/{len(segments)} sections; {len(missing)} source recordings pending.", flush=True)
        return
    concat = WORK / "concat.txt"
    concat.write_text("".join("file '" + str(p).replace("'", "'\\''") + "'\n" for p in rendered), encoding="utf-8")
    run_ffmpeg(ffmpeg, ["-f", "concat", "-safe", "0", "-i", str(concat), "-c", "copy",
                       "-movflags", "+faststart", str(TARGET)], WORK / "concat.log")
    metadata = probe(ffmpeg, TARGET)
    progress = run_ffmpeg(ffmpeg, ["-v", "error", "-i", str(TARGET), "-an", "-progress", "pipe:1",
                                 "-nostats", "-f", "null", "-"], WORK / "full_decode.log")
    decoded = int(re.findall(r"^frame=(\d+)$", progress, flags=re.M)[-1])
    assert metadata == {"duration_s": float(TOTAL_SECONDS), "width": 1920, "height": 1080, "fps": 30.0, "codec": "h264"}, metadata
    assert decoded == TOTAL_SECONDS * FPS, decoded
    manifest = {
        "output": str(TARGET.relative_to(ROOT)), "sha256": digest(TARGET),
        "poster": str((ASSETS / "演示视频封面.png").relative_to(ROOT)),
        "subtitle_file": str(subtitles.relative_to(ROOT)), "subtitles_burned_in": True,
        "metadata": metadata, "decoded_frames": decoded, "full_decode_success": True,
        "audio": "none", "timeline": segments,
        "sources": [source_row(ffmpeg, key) for key in INPUTS],
        "boundaries": [
            "运动画面均来自原始 Isaac 录像；合成只缩放、排版、加字幕或按明确标记改变播放速度。100 N 失稳停止后仅该画格冻结末帧，并显著标明真实停仿时间；其他两组继续运行。",
            "L2 四个实验分别在同一画面并排播放三种条件，仿真时间从 0 秒对齐。",
            "L4 跳跃最后 7 秒为原始 8.0–11.5 秒片段的 0.5× 慢放；其他段落均为原速。",
            "L1、L3 为原理和分析阶段，不虚构独立机器人演示；其内容保留在 PPT 正文。",
            "L2 侧推为 2026-11-05 延长外力的正式重测，报告和 PPT 第 9 页使用同批新数据；其余补录只用于演示，原正式定量曲线继续保留各自来源。",
            "四地形段按左上 5°坡道、右上粗糙地面、左下随机障碍、右下 6 cm 台阶排列；四组均截取仿真 0–24 秒，原速同步播放，不冻结、不裁切。",
            "四地形同屏展示共同观察窗口内的行走过程；该片段不构成穿越完整地形区域的通过判据，完整录像和阶段报告保留各自实验结果。随机障碍原录像为 45 秒，粗糙地面原录像为 28 秒。",
            "四种地形沿用各自原实验配置，行走速度和抬腿高度不完全相同；同屏视频用于展示不同场景，不作为仅改变地形的单变量对照。",
        ],
    }
    (ASSETS / "video_sources.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(TARGET), "metadata": metadata, "decoded_frames": decoded}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
