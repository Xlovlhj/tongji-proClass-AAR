"""Add existing terrain evidence to page 13 and clarify page 14's timing text."""
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

from build_presentation import text
from update_cover_video import NS, OUT, ROOT, STEM, replace_paragraph_text, resolve_relationship, sha, xml_bytes
from update_push_presentation import replace_text_frame

BASE = "Pro2/output/L4/03_terrain_and_jump/"
TERRAIN = [
    ("5° 坡道：部分进入", BASE + "slope_05deg/final_frame.png", .55, 1.28),
    ("粗糙地面：部分进入", BASE + "rough_medium_run02/final_frame.png", 3.75, 1.28),
    ("随机障碍：区域穿越", BASE + "random_obstacles_seed01_full/final_frame.png", .55, 4.03),
    ("盲踩台阶：部分进入", BASE + "blind_step_up_run02/final_frame.png", 3.75, 4.03),
]
JUMP = BASE + "vertical_jump_acceptance_run01/analysis_final/height.png"
OLD_TIMING = "前向误差接近，当前 NMPC 实现计算更重。"
NEW_TIMING = "前向误差接近，但当前 NMPC 的平均求解时间更长。"


def main():
    target = OUT / (STEM + ".pptx")
    current = target.read_bytes()
    original = current
    audit_path = OUT / "checks/terrain_slide_update.json"
    if audit_path.is_file():
        previous = json.loads(audit_path.read_text())
        prior = Path(previous.get("snapshot_before", ""))
        if (previous.get("output_pptx_sha256") == sha(current) and prior.is_file()
                and sha(prior.read_bytes()) == previous.get("input_pptx_sha256")):
            original = prior.read_bytes()
    snapshot = Path(tempfile.gettempdir()) / ("pro2_terrain_before_" + sha(original)[:12] + ".pptx")
    if not snapshot.exists():
        snapshot.write_bytes(original)

    manifest_path = OUT / "presentation_sources.json"
    manifest = json.loads(manifest_path.read_text())
    prs = Presentation(io.BytesIO(original))
    slide = prs.slides[12]
    jump_hash = sha((ROOT / JUMP).read_bytes())
    jump = next(s for s in slide.shapes if hasattr(s, "image") and sha(s.image.blob) == jump_hash)
    title = next(s for s in slide.shapes if s.name == "TextBox 1")
    for shape in list(slide.shapes):
        if shape not in (jump, title):
            shape._element.getparent().remove(shape._element)
    # Retain the original report plot at its original aspect ratio.
    ratio = jump.image.size[1] / jump.image.size[0]
    jump.left, jump.top = Inches(7.05), Inches(1.72)
    jump.width, jump.height = Inches(5.7), Inches(5.7 * ratio)
    for caption, source, x, y in TERRAIN:
        text(slide, x, y, 3.0, .36, caption, 20, True, True)
        picture = slide.shapes.add_picture(str(ROOT / source), Inches(x), Inches(y + .42), width=Inches(3.0))
        picture.name = "原图 " + source
    text(slide, 7.05, 1.28, 5.7, .4, "竖直跳跃", 22, True, True)
    text(slide, 7.05, 5.25, 5.7, 1.85,
         "两次独立运行，各 21 项验收通过。\n"
         "增高 0.0608 m；腾空 0.27 s。\n"
         "腾空关闭接触 MPC，着陆恢复支撑。\n"
         "未达 0.12 m 目标，落地冲击待优化。", 20)
    text(slide, .55, 6.99, 6.2, .36, "穿越判据：基座越过地形区域末端。", 20)
    scratch = io.BytesIO()
    prs.save(scratch)
    record13 = manifest["decks"][STEM][12]
    new_sources = [path for _, path, _, _ in TERRAIN]
    record13["sources"] = list(dict.fromkeys(record13["sources"] + new_sources))
    record13["images"] = [{"source": path, "sha256": sha((ROOT / path).read_bytes())}
                          for path in [JUMP] + new_sources]

    with ZipFile(io.BytesIO(original)) as before, ZipFile(scratch) as generated:
        new_parts = set(generated.namelist()) - set(before.namelist())
        if not all(n.startswith("ppt/media/") and n.endswith(".png") for n in new_parts):
            raise ValueError(f"Unexpected generated package parts: {new_parts}")
        # PNG is already registered; existing package content types stay intact.
        types = etree.fromstring(before.read("[Content_Types].xml"))
        assert any(n.get("Extension") == "png" and n.get("ContentType") == "image/png" for n in types)
        replacements = {name: generated.read(name) for name in
                        ["ppt/slides/slide13.xml", "ppt/slides/_rels/slide13.xml.rels", *sorted(new_parts)]}
        tree14 = etree.fromstring(before.read("ppt/slides/slide14.xml"))
        changed14 = False
        for paragraph in tree14.xpath(".//a:p", namespaces=NS):
            old = "".join(paragraph.xpath(".//a:t/text()", namespaces=NS))
            if OLD_TIMING in old:
                replace_paragraph_text(paragraph, old.replace(OLD_TIMING, NEW_TIMING))
                changed14 = True
        if not changed14 and NEW_TIMING not in "".join(tree14.xpath(".//a:t/text()", namespaces=NS)):
            raise ValueError("Cannot identify the requested NMPC sentence")
        replacements["ppt/slides/slide14.xml"] = xml_bytes(tree14)

        for page in (13, 14):
            relname = f"ppt/slides/_rels/slide{page}.xml.rels"
            relationships = etree.fromstring(replacements.get(relname, before.read(relname)))
            notesrel = next(r for r in relationships if r.get("Type", "").endswith("/notesSlide"))
            notesname = resolve_relationship(relname, notesrel.get("Target"))
            notes = etree.fromstring(before.read(notesname))
            body = notes.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]/p:txBody", namespaces=NS)[0]
            if page == 13:
                lines = [s.text for s in slide.shapes if s.has_text_frame]
                lines += ["四张场景图直接复用正式运行的最终截图。坡道、粗糙地面、盲踩台阶均已进入，但未走完整段。",
                          "随机障碍结果是基座穿过整个测试区域；日志没有逐足接触对象，不能据此宣称每块障碍均被踩越。",
                          "证据来源", *record13["sources"]]
                replace_text_frame(body, "\n".join(lines))
            else:
                for paragraph in body.findall("a:p", NS):
                    old = "".join(paragraph.xpath(".//a:t/text()", namespaces=NS))
                    if OLD_TIMING in old:
                        replace_paragraph_text(paragraph, old.replace(OLD_TIMING, NEW_TIMING))
            replacements[notesname] = xml_bytes(notes)

        output = io.BytesIO()
        with ZipFile(output, "w") as after:
            after.comment = before.comment
            for info in before.infolist():
                after.writestr(copy.copy(info), replacements.get(info.filename, before.read(info.filename)))
            for name in sorted(new_parts):
                after.writestr(copy.copy(generated.getinfo(name)), replacements[name])
        updated = output.getvalue()
        with ZipFile(io.BytesIO(updated)) as after:
            assert after.testzip() is None
            changed = [n for n in before.namelist() if before.read(n) != after.read(n)]
            assert set(changed) <= set(replacements)
            assert all(before.read(n) == after.read(n) for n in before.namelist() if n.startswith("ppt/media/"))
            assert all(before.read(f"ppt/slides/slide{i}.xml") == after.read(f"ppt/slides/slide{i}.xml")
                       for i in range(1, 16) if i not in (13, 14))

    final = Presentation(io.BytesIO(updated))
    companion = OUT / "逐页讲稿与证据来源.md"
    prose = companion.read_text()
    for page in (13, 14):
        record = manifest["decks"][STEM][page - 1]
        final_slide = final.slides[page - 1]
        record["body_text"] = "\n".join(s.text for s in final_slide.shapes if s.has_text_frame)
        record["notes"] = "复用正式地形截图，图外标注条件与实际结果。" if page == 13 else record.get("notes", "")
        replacement = f"## 第 {page} 页｜{record['stage']}｜{record['title']}\n\n" + final_slide.notes_slide.notes_text_frame.text + "\n\n"
        prose, count = re.subn(rf"^## 第 {page} 页[^\n]*\n.*?(?=^## 第 {page+1} 页)", lambda _: replacement,
                              prose, count=1, flags=re.M | re.S)
        assert count == 1
    manifest["revision"] = "add four formal terrain screenshots to page 13; clarify average NMPC solve time on page 14"
    for path in new_sources:
        manifest["source_sha256"][path] = sha((ROOT / path).read_bytes())
    audit = {"status": "pass", "snapshot_before": str(snapshot), "input_pptx_sha256": sha(original),
             "output_pptx_sha256": sha(updated), "changed_parts": changed, "added_parts": sorted(new_parts),
             "all_original_media_byte_identical": True,
             "preserved_slides": [i for i in range(1, 16) if i not in (13, 14)],
             "terrain_images": new_sources, "nm_pc_wording": NEW_TIMING}
    if sha(target.read_bytes()) != sha(current):
        raise RuntimeError("PPT saved externally; rerun against the latest file")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as handle:
        handle.write(updated)
        temporary = Path(handle.name)
    temporary.replace(target)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    companion.write_text(prose)
    audit_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
