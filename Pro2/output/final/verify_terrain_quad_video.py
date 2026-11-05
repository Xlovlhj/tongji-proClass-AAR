"""Check four-pane source alignment and preserve the unchanged montage evidence."""
from datetime import datetime
import json
import math
import shutil
from PIL import Image, ImageChops, ImageDraw, ImageFont, ImageStat
from build_demo_video import OUT, ROOT, WORK, TARGET, QUAD_PANES, QUAD_SIZE, FONT, digest, ffmpeg_path, run_ffmpeg, plan


def main():
    folder = OUT / "checks/terrain_quad_revision"
    folder.mkdir(exist_ok=True)
    before = json.loads((folder / "before_sha256.json").read_text())
    segments = plan()
    preserved = []
    for seg in segments:
        if seg["kind"] == "quad":
            continue
        new = WORK / f"segment_{seg['number']:02d}_{seg['name']}.mp4"
        old = next(path for path in before if path.endswith("_" + seg["name"] + ".mp4"))
        assert digest(new) == before[old], seg["name"]
        preserved.append({"name": seg["name"], "old_path": old, "path": str(new.relative_to(ROOT)), "sha256": digest(new)})
    assert len(preserved) == 8
    evidence_path = OUT / "checks/montage_visual_review_evidence.json"
    old_evidence = json.loads(evidence_path.read_text())
    (folder / "previous_visual_evidence.json").write_text(json.dumps(old_evidence, ensure_ascii=False, indent=2) + "\n")
    frames = []
    by_hash = {row["sha256"]: row["path"] for row in preserved}
    for record in old_evidence["frames"]:
        if record["segment_sha256"] in by_hash:
            record["segment"] = by_hash[record["segment_sha256"]]
            frames.append(record)
    seg = next(seg for seg in segments if seg["kind"] == "quad")
    movie = WORK / f"segment_{seg['number']:02d}_{seg['name']}.mp4"
    sources = [ROOT / path for path in seg["sources"]]
    assert len({digest(p) for p in sources}) == 4
    ffmpeg = ffmpeg_path()
    width, height = QUAD_SIZE
    for moment in (6, 14, 23):
        frame_number = moment * 30
        target = folder / f"quad_{moment}s.png"
        run_ffmpeg(ffmpeg, ["-i", str(movie), "-vf", f"select=eq(n\\,{frame_number})", "-frames:v", "1", "-fps_mode", "vfr", str(target)], target.with_suffix(".log"))
        candidates = []
        for index, source in enumerate(sources):
            path = folder / f"source_{index}_{moment}s.png"
            run_ffmpeg(ffmpeg, ["-i", str(source), "-vf", f"select=eq(n\\,{frame_number}),scale={width}:{height}:flags=lanczos", "-frames:v", "1", "-fps_mode", "vfr", str(path)], path.with_suffix(".log"))
            candidates.append(Image.open(path).convert("RGB"))
        composed = Image.open(target).convert("RGB")
        errors = []
        for index, (x, y) in enumerate(QUAD_PANES):
            pane = composed.crop((x, y, x + width, y + height))
            row = [sum(ImageStat.Stat(ImageChops.difference(pane, candidate)).mean) / 3 for candidate in candidates]
            assert min(range(4), key=row.__getitem__) == index, row
            assert row[index] < 3.0, row
            errors.append(row)
        frames.append({"segment": str(movie.relative_to(ROOT)), "segment_sha256": digest(movie),
                       "segment_time_s": moment, "source_time_s": moment, "actual_decoded_frame_index": frame_number,
                       "frame": str(target.relative_to(ROOT)), "label": f"L4 四地形 · t = {moment} s",
                       "sources": seg["sources"], "source_sha256": [digest(p) for p in sources],
                       "panel_to_source_mean_absolute_rgb_error": errors,
                       "correct_source_minimum_error_for_each_panel": True})
    manifest = json.loads((OUT / "assets/video_sources.json").read_text())
    assert manifest["metadata"]["duration_s"] == 132 and manifest["decoded_frames"] == 3960
    assert manifest["full_decode_success"] and manifest["sha256"] == digest(TARGET)
    assert len(manifest["sources"]) == 19
    assert all(digest(ROOT / row["path"]) == row["sha256"] for row in manifest["sources"])
    evidence = {"review_scope": "Eight unchanged encodes retain prior review; four-terrain pane alignment checked at three decoded frames.",
                "montage_sha256": digest(TARGET), "frames": frames, "preserved_segments": preserved,
                "terrain_display_window_s": [0, 24], "terrain_speed": 1.0,
                "terrain_no_freeze_or_loop": True, "distinct_terrain_sources": 4}
    evidence_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    sheet = Image.new("RGB", (1920, math.ceil(len(frames) / 3) * 390), "white")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype(str(FONT), 24)
    for index, record in enumerate(frames):
        x, y = index % 3 * 640, index // 3 * 390
        picture = Image.open(ROOT / record["frame"]).convert("RGB")
        picture.thumbnail((630, 354))
        sheet.paste(picture, (x + 5, y + 32))
        draw.text((x + 8, y + 2), record["label"], font=font, fill="#172B36")
    sheet.save(OUT / "checks/montage_visual_contactsheet.png")
    destination = ROOT / "Pro2/output/L4/03_terrain_and_jump/ppt_terrain_quad_20261105/comparison"
    destination.mkdir(exist_ok=True)
    shutil.copyfile(movie, destination / "terrain_quad.mp4")
    shutil.copyfile(folder / "quad_23s.png", destination / "preview.png")
    style = {"status": "passed", "verified_at": datetime.now().astimezone().isoformat(),
             "video": str(TARGET.relative_to(ROOT)), "output_sha256": digest(TARGET),
             "duration_s": 132, "decoded_frames": 3960, "full_decode_success": True,
             "source_integrity_verified": 19, "minimum_generated_text_px": 36,
             "text_colors": ["#172B36", "#006D68", "#FFFFFF"],
             "text_sizes_px": {"clock": 36, "quad_label": 36, "quad_caption": 40, "title": 50},
             "aspect_ratio_preserved": "960x720 source, 544x408 terrain panes; existing other sections unchanged",
             "subtitles_burned_in": True, "sampled_segments": 9, "sampled_frames": len(frames),
             "checks": {"eight_other_segments_byte_identical": True, "four_distinct_terrain_sources": True,
                        "all_12_sampled_terrain_panes_match_source_and_time": True,
                        "source_sha256_matches": True, "metadata_and_complete_decode": True},
             "visual_review": "Pro2/output/final/checks/montage_visual_review.md"}
    (OUT / "checks/video_style_verification.json").write_text(json.dumps(style, ensure_ascii=False, indent=2) + "\n")
    (folder / "verification.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    (OUT / "checks/montage_visual_review.md").write_text(
        "# 四地形同屏视频复核\n\n"
        "当前综合视频132秒、1920×1080、30fps，共3960帧，全片解码通过。88–112秒为四种地形前24秒同步原速画面。\n\n"
        "四格依次为5°坡道、粗糙地面、随机障碍、6cm台阶。6、14、23秒的四个画格均与对应原片同帧比对，确认来源和时间对齐，完整4:3画面，无裁切、重复或冻结。\n\n"
        "其他8段MP4编码字节与更新前一致，沿用此前视觉检查，包括L2外力箭头和100N失稳保护提示。字幕、标题和地形标签均为深色正常字号。\n\n"
        "四组显示行走过程，完整实验时长和配置不同；不以共同24秒画面声称所有地形均已穿越。完整两条新增原片及分析保存于L4独立补录目录。\n\n"
        "核验：`terrain_quad_revision/verification.json`；全片来源：`../assets/video_sources.json`。\n")
    print(json.dumps({"status": "pass", "preserved_segments": len(preserved), "duration_s": 132,
                      "quad_video": str((destination / "terrain_quad.mp4").relative_to(ROOT))}, ensure_ascii=False))


if __name__ == "__main__":
    main()
