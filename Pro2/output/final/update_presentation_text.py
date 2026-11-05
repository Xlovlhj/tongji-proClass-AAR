"""Clarify the L1 objective and tidy presentation text in the current user deck.

Only text XML is patched. Existing video/image bytes, relationships, and all
unaffected slide shapes are preserved; experiment logs are never rewritten.
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

from update_cover_video import (NS, OUT, STEM, replace_paragraph_text,
                               resolve_relationship, sha, update_metadata,
                               xml_bytes)
from update_push_presentation import replace_text_frame


L1_TEXT = {
    "TextBox 3": "MPC 决定未来双脚怎样发力。",
    "TextBox 4": (
        "优化变量：未来左右脚的接触力与力矩序列。\n"
        "预测状态：姿态、位置、角速度、线速度（12 维）。"
    ),
    "TextBox 5": (
        "目标：最小化两项误差的加权平方和：\n"
        "① 预测状态与参考运动的误差；\n"
        "② 接触输入与参考支撑载荷的偏差。\n"
        "约束：摩擦、单边接触、足底力矩与支撑切换。"
    ),
    "TextBox 6": (
        "模型：线性化单刚体；qpOASES 求解。\n"
        "只把第一步接触输入交给 WBC，再求关节力矩。\n"
        "预测 10 × 5 ms = 50 ms；独立控制 3 步。\n"
        "MPC 200 Hz；WBC、PVT、物理 1 kHz。"
    ),
}


def presentation_text(value: str) -> str:
    """Use enumeration punctuation, retaining actual paths and SI units."""
    value = value.replace("圆开 / 闭环", "圆开环、闭环")
    value = value.replace("失败 / 接触不匹配", "失败或接触不匹配")
    value = value.replace("分层伪逆 / 零空间", "分层伪逆与零空间")
    value = value.replace("36 / 350 更新", "36 次，共 350 次")
    value = re.sub(r"\s+[/\\]\s+", "、", value)
    value = re.sub(r"(?<=[）)])\s*[/\\]\s*(?=\d)", "、", value)
    value = re.sub(r"\s*、\s*", "、", value)
    # These are display-only approximations of the original sample/end times.
    value = re.sub(r"11\.99(?=\s*s\b)", "12", value)
    value = re.sub(r"8\.994(?=\s*s\b)", "约 9", value)
    value = re.sub(r"8\.99(?=\s*s\b)", "9", value)
    value = re.sub(r"0\.269(?=\s*s\b)", "0.27", value)
    value = re.sub(r"(?<![\d.])(\d+)\.0+(?=\s*(?:s\b|[–—-]))", r"\1", value)
    return value


def main() -> None:
    pptx = OUT / (STEM + ".pptx")
    current = pptx.read_bytes()
    original = current
    audit_path = OUT / "checks/presentation_text_update.json"
    if audit_path.is_file():
        earlier = json.loads(audit_path.read_text())
        earlier_snapshot = Path(earlier.get("snapshot_before", ""))
        if (earlier.get("output_pptx_sha256") == sha(current)
                and earlier_snapshot.is_file()
                and sha(earlier_snapshot.read_bytes()) == earlier.get("input_pptx_sha256")):
            original = earlier_snapshot.read_bytes()
    snapshot = Path(tempfile.gettempdir()) / ("pro2_text_before_" + sha(original)[:12] + ".pptx")
    if not snapshot.exists():
        snapshot.write_bytes(original)
    manifest_path = OUT / "presentation_sources.json"
    previous = json.loads(manifest_path.read_text())
    replacements, changes = {}, []
    with ZipFile(io.BytesIO(original)) as source:
        for page in range(1, 16):
            name = f"ppt/slides/slide{page}.xml"
            tree = etree.fromstring(source.read(name))
            changed = False
            if page == 4:
                for shape_name, value in L1_TEXT.items():
                    found = tree.xpath("./p:cSld/p:spTree/p:sp[p:nvSpPr/p:cNvPr/@name=$name]",
                                       namespaces=NS, name=shape_name)
                    if len(found) != 1:
                        raise ValueError(f"Cannot identify L1 {shape_name}")
                    shape = found[0]
                    body = shape.find("p:txBody", NS)
                    old = "\n".join("".join(p.xpath(".//a:t/text()", namespaces=NS))
                                    for p in body.findall("a:p", NS))
                    if old != value:
                        replace_text_frame(body, value)
                        changes.append({"page": page, "shape": shape_name, "before": old, "after": value})
                        changed = True
                    # Four readable objective/constraint lines use the free gap
                    # above the existing solver paragraph. Other layouts stay put.
                    if shape_name == "TextBox 5":
                        shape.find("p:spPr/a:xfrm/a:off", NS).set("y", str(round(3.35 * 914400)))
                        shape.find("p:spPr/a:xfrm/a:ext", NS).set("cy", str(round(1.65 * 914400)))
                        changed = True
            for paragraph in tree.xpath(".//a:p", namespaces=NS):
                old = "".join(paragraph.xpath(".//a:t/text()", namespaces=NS))
                new = presentation_text(old)
                if new != old:
                    replace_paragraph_text(paragraph, new)
                    changes.append({"page": page, "before": old, "after": new})
                    changed = True
            if changed:
                replacements[name] = xml_bytes(tree)

            # Keep speaker notes aligned with the now-current slide body/table.
            relname = f"ppt/slides/_rels/slide{page}.xml.rels"
            relations = etree.fromstring(source.read(relname))
            notesrel = next(r for r in relations if r.get("Type", "").endswith("/notesSlide"))
            notesname = resolve_relationship(relname, notesrel.get("Target"))
            notestree = etree.fromstring(source.read(notesname))
            body = notestree.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]/p:txBody", namespaces=NS)
            if len(body) != 1:
                raise ValueError(f"Missing notes on page {page}")
            lines = ["".join(p.xpath(".//a:t/text()", namespaces=NS))
                     for p in tree.xpath(".//a:p", namespaces=NS)]
            if page == 4:
                lines += ["补充讲解：权重 Q、R 决定状态跟踪与接触输入代价的取舍。",
                          "代码的13维输入包含12个接触力/力矩分量和1个固定重力增广项；重力项不能自由优化。",
                          "MPC输出第一步双脚接触输入；WBC结合全身动力学求关节前馈力矩。当前基座状态来自Isaac仿真真值。"]
            lines += ["证据来源", *previous["decks"][STEM][page - 1]["sources"]]
            replace_text_frame(body[0], "\n".join(line for line in lines if line))
            if xml_bytes(notestree) != source.read(notesname):
                replacements[notesname] = xml_bytes(notestree)

        output = io.BytesIO()
        with ZipFile(output, "w") as destination:
            destination.comment = source.comment
            for info in source.infolist():
                destination.writestr(copy.copy(info), replacements.get(info.filename, source.read(info.filename)))
        updated = output.getvalue()
        with ZipFile(io.BytesIO(updated)) as after:
            assert after.testzip() is None and after.namelist() == source.namelist()
            changed_parts = [n for n in source.namelist() if source.read(n) != after.read(n)]
            assert set(changed_parts) <= set(replacements)
            media = [n for n in source.namelist() if n.startswith("ppt/media/")]
            assert all(source.read(n) == after.read(n) for n in media)
            preserved_slides = [i for i in range(1, 16)
                                if source.read(f"ppt/slides/slide{i}.xml") == after.read(f"ppt/slides/slide{i}.xml")]

    prs = Presentation(io.BytesIO(updated))
    manifest, notes, _ = update_metadata(prs, previous)
    for slide, record in zip(prs.slides, manifest["decks"][STEM]):
        record["title"] = presentation_text(record["title"])
        record["tables"] = [[[cell.text for cell in row.cells] for row in shape.table.rows]
                            for shape in slide.shapes if shape.has_table]
    manifest["revision"] = "clarify L1 MPC optimization; use enumeration commas and readable display times; preserve current user edits and all media"
    # Generate this companion from the final text, including tables.
    notes = ["# 逐页讲稿与证据来源\n", "正文与当前PPT同步；实验数据保留原始精度，汇报时间按易读精度展示。\n"]
    for slide, record in zip(prs.slides, manifest["decks"][STEM]):
        notes += [f"## 第 {record['page']} 页｜{record['stage']}｜{record['title']}\n",
                  slide.notes_slide.notes_text_frame.text + "\n"]
    audit = {"status": "pass", "snapshot_before": str(snapshot),
             "input_pptx_sha256": sha(original), "output_pptx_sha256": sha(updated),
             "changed_parts": changed_parts, "text_changes": changes,
             "preserved_slide_xml_pages": preserved_slides,
             "all_media_byte_identical": True, "preserved_media_count": len(media),
             "experimental_data_modified": False, "slides": 15}
    if sha(pptx.read_bytes()) != sha(current):
        raise RuntimeError("PPT was saved by another application; rerun against that latest version")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as handle:
        handle.write(updated)
        temporary = Path(handle.name)
    temporary.replace(pptx)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (OUT / "逐页讲稿与证据来源.md").write_text("\n".join(notes))
    (OUT / "checks/presentation_text_update.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: audit[k] for k in ("status", "preserved_slide_xml_pages", "all_media_byte_identical", "output_pptx_sha256")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
