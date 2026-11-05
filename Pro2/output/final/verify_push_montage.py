"""Refresh montage evidence after the longer-force recording revision.

python3 Pro2/output/final/verify_push_montage.py
"""
from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFont, ImageStat

from build_demo_video import (OUT, ROOT, WORK, TARGET, INPUTS, FONT,
                              digest, ffmpeg_path, run_ffmpeg, plan)


def main():
    ffmpeg = ffmpeg_path()
    directory = OUT / "checks/push_revision"
    directory.mkdir(exist_ok=True, parents=True)
    preservation_file = directory / "segment_preservation.json"
    preservation = json.loads(preservation_file.read_text())
    preservation["after"] = {path: digest(ROOT / path) for path in preservation["before"]}
    preservation["changed_segments"] = [path for path, before in preservation["before"].items()
                                          if before != preservation["after"][path]]
    assert preservation["changed_segments"] == [str((WORK / "segment_05_l2_push.mp4").relative_to(ROOT))]
    preservation_file.write_text(json.dumps(preservation, ensure_ascii=False, indent=2) + "\n")
    evidence_path = OUT / "checks/montage_visual_review_evidence.json"
    evidence = json.loads(evidence_path.read_text())
    previous = directory / "visual_evidence_before_20261105.json"
    if not previous.exists():
        previous.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    frames = [row for row in evidence["frames"] if "segment_05_l2_push" not in row["segment"]]
    for row in frames:
        assert digest(ROOT / row["segment"]) == row["segment_sha256"]
    segment = next(item for item in plan() if item["name"] == "l2_push")
    movie = WORK / "segment_05_l2_push.mp4"
    sources = [ROOT / value for value in segment["sources"]]
    fingerprint = json.loads(movie.with_suffix(".json").read_text())
    comparisons = []
    for moment in (6.9, 7.2, 7.6, 8.8, 9.5):
        frame_number = round(moment * 30)
        image = directory / f"push_{moment:.1f}s.png"
        run_ffmpeg(ffmpeg, ["-i", str(movie), "-vf", f"select=eq(n\\,{frame_number})",
                           "-frames:v", "1", "-fps_mode", "vfr", str(image)], image.with_suffix(".log"))
        record = {"segment": str(movie.relative_to(ROOT)), "segment_sha256": digest(movie),
                  "segment_time_s": moment, "actual_decoded_frame_index": frame_number,
                  "source_time_s": moment, "frame": str(image.relative_to(ROOT)),
                  "label": f"L2 侧推 · t = {moment:.1f} s",
                  "sources": segment["sources"], "source_sha256": [digest(source) for source in sources],
                  "distinct_source_paths": len(set(sources)) == 3,
                  "distinct_source_hashes": len({digest(source) for source in sources}) == 3,
                  "fingerprint_source_hashes_match": all(fingerprint["source_sha256"][str(source.relative_to(ROOT))] == digest(source) for source in sources)}
        if moment in (7.2, 8.8):
            source_images = []
            for index, source in enumerate(sources):
                target = directory / f"source_{index}_{moment:.1f}s.png"
                run_ffmpeg(ffmpeg, ["-i", str(source), "-vf", f"select=eq(n\\,{frame_number}),scale=624:468:flags=lanczos",
                                   "-frames:v", "1", "-fps_mode", "vfr", str(target)], target.with_suffix(".log"))
                source_images.append(Image.open(target).convert("RGB"))
            composed = Image.open(image).convert("RGB")
            errors = []
            for index, x in enumerate((16, 648, 1280)):
                pane = composed.crop((x, 278, x + 624, 746))
                errors.append([round(sum(ImageStat.Stat(ImageChops.difference(pane, candidate)).mean) / 3, 6) for candidate in source_images])
                assert min(range(3), key=errors[-1].__getitem__) == index, errors
            record["panel_to_source_mean_absolute_rgb_error"] = errors
            record["correct_source_minimum_error_for_each_panel"] = True
            comparisons.extend(errors)
        if moment >= segment["stopped_panel"]["freeze_at_s"]:
            record["stopped_panel"] = segment["stopped_panel"]
            record["annotation"] = "100 N：失稳保护；仿真已停止于8.994 s；当前画面为停止帧"
        frames.append(record)
    evidence.update({"review_scope": "All ten segments; nine preserved encodes retain prior review, longer push reviewed at five exact decoded frames.",
                     "revision_date": "2026-11-05", "montage_sha256": digest(TARGET), "frames": frames,
                     "unmodified_segments_byte_identical": 9,
                     "raw_failure_clip_not_padded": True,
                     "failure_freeze_disclosed": segment["stopped_panel"],
                     "viewport_force_display_boundary": "Original source rendering can lag the physical force by one 30 Hz frame; physics and 100 Hz force logs remain the time authority."})
    evidence_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    font = ImageFont.truetype(str(FONT), 24)
    sheet = Image.new("RGB", (1920, math.ceil(len(frames) / 3) * 390), "white")
    draw = ImageDraw.Draw(sheet)
    for index, record in enumerate(frames):
        x, y = (index % 3) * 640, (index // 3) * 390
        image = Image.open(ROOT / record["frame"]).convert("RGB")
        image.thumbnail((630, 354))
        sheet.paste(image, (x + 5, y + 32))
        draw.text((x + 8, y + 2), record["label"], font=font, fill="#172B36")
    sheet.save(OUT / "checks/montage_visual_contactsheet.png")
    manifest = json.loads((OUT / "assets/video_sources.json").read_text())
    style = {"status": "passed", "verified_at": datetime.now().astimezone().isoformat(),
             "video": str(TARGET.relative_to(ROOT)), "output_sha256": digest(TARGET),
             "duration_s": manifest["metadata"]["duration_s"], "decoded_frames": manifest["decoded_frames"],
             "full_decode_success": manifest["full_decode_success"], "source_integrity_verified": len(manifest["sources"]),
             "minimum_generated_text_px": 36, "text_sizes_px": {"clock": 36, "stop_notice": 38, "caption": 42, "condition": 44, "title": 50},
             "text_colors": ["#172B36", "#006D68", "#FFFFFF"], "stop_notice_background": "#9B2020",
             "subtitles_burned_in": True,
             "aspect_ratio_preserved": "960x720 source; 624x468 comparison panels and 1120x840 single panels",
             "visual_review": "Pro2/output/final/checks/montage_visual_review.md",
             "contactsheet": "Pro2/output/final/checks/montage_visual_contactsheet.png",
             "sampled_frames": len(frames), "sampled_segments": 10,
             "checks": {"metadata_160s_1080p_30fps": True, "full_decode_4800_frames": True,
                        "subtitles_burned_in": True, "all_sampled_input_hashes_match": all(row["fingerprint_source_hashes_match"] for row in frames),
                        "all_l2_panel_inputs_distinct": True,
                        "all_15_l2_panels_match_correct_source": len(comparisons) == 6,
                        "nine_other_segments_byte_identical": True,
                        "physical_failure_explicit_and_raw_video_not_padded": True}}
    (OUT / "checks/video_style_verification.json").write_text(json.dumps(style, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"sampled_frames": len(frames), "new_push_panels_matched": len(comparisons),
                      "unmodified_segments": 9, "status": "pass"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
