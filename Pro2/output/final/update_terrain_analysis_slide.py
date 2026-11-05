"""Update only page 13 with observed terrain gait and stability metrics.

Run from the project root:
    python3 Pro2/output/final/update_terrain_analysis_slide.py

Always start from the current saved presentation. Preserve the other fourteen
slides, all original media, and the right-hand jump plot and analysis.
"""
from __future__ import annotations

import copy
import io
import json
import re
import tempfile
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from pptx import Presentation
from pptx.util import Inches

from build_presentation import table, text
from update_cover_video import NS, OUT, ROOT, STEM, resolve_relationship, sha, xml_bytes
from update_push_presentation import replace_text_frame

BASE = "Pro2/output/L4/03_terrain_and_jump/"
CASES = [
    ("5° 坡道", "slope_05deg"),
    ("粗糙地面", "rough_medium_run02"),
    ("随机障碍", "random_obstacles_seed01_full"),
    ("6 cm 台阶", "blind_step_up_run02"),
]
JUMP = BASE + "vertical_jump_acceptance_run01/analysis_final/height.png"
REPORT = "Pro2/output/L4/L4验收报告.md"
SUMMARY = BASE + "gait_stability_analysis/summary.json"
EXPLANATION = (
    "四组均保持行走，未触发失稳保护。\n"
    "台阶速度误差与换腿节奏波动更大。\n"
    "统计 7 s 后行走，含接近与地形过渡。\n"
    "各组观察时长与步态参数不同。\n"
    "换腿节奏依据控制器状态统计，\n"
    "尚未用实际足底接触数据核验。"
)


def formal_data() -> tuple[list[list[str]], list[dict], list[str]]:
    summary = json.loads((ROOT / SUMMARY).read_text())
    experiments = {row["experiment"]: row for row in summary["experiments"]}
    assert summary["primary_window_key"] == "post_ramp_walk"
    rows = [["地形", "姿态峰值\n(°)", "速度RMSE\n(m/s)", "调度间隔\nCV (%)"]]
    evidence, sources = [], [REPORT, JUMP, SUMMARY]
    for label, name in CASES:
        run_source = BASE + name + "/run.json"
        metric_source = BASE + name + "/analysis_gait/metrics.json"
        run = json.loads((ROOT / run_source).read_text())
        metric = json.loads((ROOT / metric_source).read_text())
        official = experiments[name]
        if run["status"] != "completed":
            raise ValueError(f"Formal run did not complete: {name}")
        window = metric["windows"]["post_ramp_walk"]
        if window != official["windows"]["post_ramp_walk"]:
            raise ValueError(f"Metric differs from gait summary: {name}")
        rows.append([label, f"{window['max_abs_tilt_axis_deg']:.2f}",
                     f"{window['forward_velocity_rmse_m_s']:.3f}",
                     f"{window['support_switch_interval_cv_percent']:.1f}"])
        evidence.append({"experiment": name, "window": window,
                         "run_source": run_source, "metric_source": metric_source})
        sources.extend([run_source, metric_source, BASE + name + "/states.csv"])
    return rows, evidence, sources


def main() -> None:
    target = OUT / (STEM + ".pptx")
    original = target.read_bytes()
    original_hash = sha(original)
    snapshot = Path(tempfile.gettempdir()) / f"pro2_terrain_analysis_before_{original_hash[:12]}.pptx"
    if snapshot.exists() and sha(snapshot.read_bytes()) != original_hash:
        raise RuntimeError("Existing terrain analysis snapshot has an unexpected checksum")
    snapshot.write_bytes(original)
    rows, evidence, sources = formal_data()
    prs = Presentation(io.BytesIO(original))
    if len(prs.slides) != 15:
        raise ValueError("Expected the current fifteen-page formal presentation")
    slide = prs.slides[12]
    jump_hash = sha((ROOT / JUMP).read_bytes())
    jump = next(s for s in slide.shapes if hasattr(s, "image") and sha(s.image.blob) == jump_hash)
    title = next(s for s in slide.shapes if s.name == "TextBox 1")
    retained = [s for s in slide.shapes if s == title or s == jump or s.left >= Inches(7.0)]
    if len(retained) != 4:
        raise ValueError("Expected title, jump image, jump heading and jump analysis")
    original_retained = {s.shape_id: etree.tostring(s._element) for s in retained}
    for shape in list(slide.shapes):
        if shape in retained:
            continue
        image_rids = shape._element.xpath(".//a:blip/@r:embed")
        shape._element.getparent().remove(shape._element)
        for rid in image_rids:
            slide.part.drop_rel(rid)
    text(slide, .55, 1.28, 6.2, .4, "四种地形：步态与行走稳定性", 22, True, True)
    metrics_table = table(slide, .55, 1.84, 6.2, 2.88, rows,
                          [1.55/6.2, 1.4/6.2, 1.8/6.2, 1.45/6.2], 20)
    # LibreOffice uses taller CJK line boxes than PowerPoint; reserve enough
    # height for two header lines instead of clipping the units at the row edge.
    metrics_table.table.rows[0].height = Inches(.88)
    for row in list(metrics_table.table.rows)[1:]:
        row.height = Inches(.50)
    text(slide, .55, 4.90, 6.2, 2.45, EXPLANATION, 20)
    # Do not move or regenerate the existing jump plot or its explanation.
    for shape in retained:
        assert etree.tostring(shape._element) == original_retained[shape.shape_id]
    scratch = io.BytesIO()
    prs.save(scratch)
    manifest_path = OUT / "presentation_sources.json"
    manifest = json.loads(manifest_path.read_text())
    original_records = copy.deepcopy(manifest["decks"][STEM])
    record = manifest["decks"][STEM][12]
    record["sources"] = sources
    record["images"] = [{"source": JUMP, "sha256": jump_hash}]
    record["tables"] = [rows]
    record["body_text"] = "\n".join(s.text for s in slide.shapes if s.has_text_frame)
    record["notes"] = "使用四组正式运行的姿态、速度误差与支撑调度间隔分析行走稳定性；不以区域覆盖评价步态能力，右侧跳跃内容保留。"
    note_lines = [record["body_text"], "正式数据表："]
    note_lines.extend("；".join(value.replace("\n", " ") for value in row) for row in rows)
    note_lines += [
        "步态评价关注真实关节运动是否持续、机体是否出现失稳、前进速度是否平顺。仿真到预设时长正常结束，不把未走完区域判作步态失败。",
        "主表统一从7 s（速度命令加速结束）统计至各次运行结束：坡道7–24 s，粗糙7–28 s，随机障碍7–45 s，台阶7–24 s。包含接近和过渡过程，以免只看基座入区后漏掉提前发生的足端扰动，或只看短窗漏掉后段波动。",
        "姿态峰值取主窗口100 Hz状态日志中max(|roll|,|pitch|)，由弧度转换成度；不是偏航角，也不是合成倾角。",
        "实际前向速度由世界速度经yaw转换到机体系，速度RMSE为该速度与前进命令差的均方根。",
        "调度间隔CV为窗口内完整相邻换腿间隔的总体标准差除以均值再乘100%，反映控制器换腿节奏的波动；CV较大表示节奏更不均匀，不是实际脚触地周期的测量。",
        "四组正式运行均按预设时长正常完成，日志保留实际左右膝关节的周期运动，未触发高度或姿态失稳保护。报告同时分析全行走曲线、入区前后各5 s的过渡窗和共同7–24 s窗口，避免仅凭一个峰值或窗口评价稳定性。",
        "四种地形的运行时长、速度与抬脚高度不同，此表描述各自场景表现，不是同一条件下的地形难度排名。",
        "leg_state是结合估计力与相位切换的控制器支撑状态。正式日志没有实测足底接触序列，不能据此得出无打滑、实际支撑占空比或严格稳定裕度的结论。",
        "表格数据只取slope_05deg、rough_medium_run02、random_obstacles_seed01_full和blind_step_up_run02四次正式运行，未使用PPT补录替换正式结果。",
        "证据来源", *sources,
    ]
    notes_text = "\n".join(note_lines)
    with ZipFile(io.BytesIO(original)) as before, ZipFile(scratch) as generated:
        relname = "ppt/slides/_rels/slide13.xml.rels"
        replacements = {name: generated.read(name) for name in ["ppt/slides/slide13.xml", relname]}
        rels = etree.fromstring(replacements[relname])
        notesrel = next(r for r in rels if r.get("Type", "").endswith("/notesSlide"))
        notesname = resolve_relationship(relname, notesrel.get("Target"))
        notes = etree.fromstring(before.read(notesname))
        bodies = notes.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]/p:txBody", namespaces=NS)
        if len(bodies) != 1:
            raise ValueError("Expected one speaker-notes body for page 13")
        replace_text_frame(bodies[0], notes_text)
        replacements[notesname] = xml_bytes(notes)
        output = io.BytesIO()
        with ZipFile(output, "w") as after:
            after.comment = before.comment
            for info in before.infolist():
                after.writestr(copy.copy(info), replacements.get(info.filename, before.read(info.filename)))
        updated = output.getvalue()
        with ZipFile(io.BytesIO(updated)) as after:
            assert after.testzip() is None and after.namelist() == before.namelist()
            changed = [n for n in before.namelist() if before.read(n) != after.read(n)]
            assert set(changed) <= set(replacements)
            assert all(before.read(n) == after.read(n) for n in before.namelist() if n.startswith("ppt/media/"))
            assert all(before.read(f"ppt/slides/slide{i}.xml") == after.read(f"ppt/slides/slide{i}.xml")
                       for i in range(1, 16) if i != 13)
    companion = OUT / "逐页讲稿与证据来源.md"
    original_prose = companion.read_text()
    section = f"## 第 13 页｜{record['stage']}｜{record['title']}\n\n{notes_text}\n\n"
    prose, count = re.subn(r"^## 第 13 页[^\n]*\n.*?(?=^## 第 14 页)", lambda _: section,
                          original_prose, count=1, flags=re.M | re.S)
    if count != 1:
        raise ValueError("Cannot identify page 13's external speaker notes")
    for i, previous in enumerate(original_records):
        if i != 12:
            assert manifest["decks"][STEM][i] == previous
    source_paths = {path for page in manifest["decks"][STEM] for path in page["sources"]}
    manifest["source_sha256"] = {path: sha((ROOT / path).read_bytes()) for path in sorted(source_paths)}
    manifest["revision"] = "page 13: assess gait continuity, body motion, speed error and support scheduling; preserve all other pages and jump content"
    audit = {"status": "pass", "snapshot_before": str(snapshot), "input_pptx_sha256": original_hash,
             "output_pptx_sha256": sha(updated), "changed_parts": changed,
             "all_original_media_byte_identical": True, "jump_shapes_byte_identical": True,
             "preserved_slides": [i for i in range(1, 16) if i != 13],
             "preserved_manifest_pages": [i for i in range(1, 16) if i != 13],
             "formal_data": evidence, "display_table": rows, "original_image_placements": 14,
             "metrics_note": "post_ramp_walk [7,run_end): observed max absolute roll/pitch, body-forward velocity RMSE and controller support-switch interval CV; coverage is not a stability criterion"}
    if sha(target.read_bytes()) != original_hash:
        raise RuntimeError("PPT saved externally; rerun against the current file")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as handle:
        handle.write(updated)
        temporary = Path(handle.name)
    temporary.replace(target)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    companion.write_text(prose)
    (OUT / "checks/terrain_analysis_update.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: audit[key] for key in ["status", "changed_parts", "all_original_media_byte_identical",
                                                "jump_shapes_byte_identical", "preserved_slides", "display_table"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
