"""Add a short reading guide to slide 12, preserving every other slide/media."""
import copy
import io
import json
import re
import tempfile
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from pptx import Presentation

from update_cover_video import NS, OUT, STEM, resolve_relationship, sha, xml_bytes
from update_push_presentation import replace_text_frame

EXPLANATION = "左图：灰虚线为参考路径，\n蓝实线为机器人实际轨迹。\n两线越接近，路径跟踪越准确。"


def main():
    target = OUT / (STEM + ".pptx")
    current = target.read_bytes()
    original = current
    audit_path = OUT / "checks/page12_explanation_update.json"
    if audit_path.is_file():
        earlier = json.loads(audit_path.read_text())
        earlier_snapshot = Path(earlier.get("snapshot_before", ""))
        if (earlier.get("output_pptx_sha256") == sha(current)
                and earlier_snapshot.is_file()
                and sha(earlier_snapshot.read_bytes()) == earlier.get("input_pptx_sha256")):
            original = earlier_snapshot.read_bytes()
    snapshot = Path(tempfile.gettempdir()) / ("pro2_page12_before_" + sha(original)[:12] + ".pptx")
    if not snapshot.exists():
        snapshot.write_bytes(original)
    with ZipFile(io.BytesIO(original)) as source:
        slide_name = "ppt/slides/slide12.xml"
        tree = etree.fromstring(source.read(slide_name))
        shapes = tree.find("p:cSld/p:spTree", NS)

        def textbox(name):
            result = shapes.xpath("./p:sp[p:nvSpPr/p:cNvPr/@name=$name]", namespaces=NS, name=name)
            if len(result) != 1:
                raise ValueError(f"Cannot identify page 12 textbox: {name}")
            return result[0]

        table = shapes.xpath("./p:graphicFrame[.//a:tbl]", namespaces=NS)
        if len(table) != 1:
            raise ValueError("Expected one table on page 12")
        height = round(3.0 * 914400)
        table[0].find("p:xfrm/a:ext", NS).set("cy", str(height))
        rows = table[0].xpath(".//a:tbl/a:tr", namespaces=NS)
        for row in rows:
            row.set("h", str(height // len(rows)))

        caption = copy.deepcopy(textbox("TextBox 2"))
        props = caption.find("p:nvSpPr/p:cNvPr", NS)
        props.set("id", str(max(int(n.get("id")) for n in tree.xpath(".//p:cNvPr", namespaces=NS)) + 1))
        props.set("name", "图示说明")
        transform = caption.find("p:spPr/a:xfrm", NS)
        for key, value in (("x", 7.3), ("y", 5.12)):
            transform.find("a:off", NS).set(key, str(round(value * 914400)))
        for key, value in (("cx", 5.4), ("cy", 1.1)):
            transform.find("a:ext", NS).set(key, str(round(value * 914400)))
        body = caption.find("p:txBody", NS)
        replace_text_frame(body, EXPLANATION)
        for style in body.xpath(".//a:rPr|.//a:endParaRPr", namespaces=NS):
            style.set("sz", "2100")
        for paragraph in body.findall("a:p", NS):
            spacing = paragraph.find("a:pPr/a:lnSpc", NS)
            spacing.clear()
            etree.SubElement(spacing, "{" + NS["a"] + "}spcPts", val="2400")
            paragraph.find("a:pPr/a:spcAft/a:spcPts", NS).set("val", "0")
        for margin in ("lIns", "rIns", "tIns", "bIns"):
            body.find("a:bodyPr", NS).set(margin, "0")
        shapes.append(caption)
        textbox("TextBox 5").find("p:spPr/a:xfrm/a:off", NS).set("y", str(round(6.32 * 914400)))

        relname = "ppt/slides/_rels/slide12.xml.rels"
        relationships = etree.fromstring(source.read(relname))
        notesrel = next(r for r in relationships if r.get("Type", "").endswith("/notesSlide"))
        notesname = resolve_relationship(relname, notesrel.get("Target"))
        notes = etree.fromstring(source.read(notesname))
        notesbody = notes.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]/p:txBody", namespaces=NS)[0]
        lines = ["".join(p.xpath(".//a:t/text()", namespaces=NS)) for p in notesbody.findall("a:p", NS)]
        index = lines.index("证据来源")
        lines[index:index] = [EXPLANATION, "图中为圆形闭环实验。横纵轴为世界坐标 x、y，单位为米。",
                              "横向 RMSE 约 0.134 m，来自路径法向误差的均方根；它不是终点距离。进入终点范围不代表全程与参考路径重合。"]
        replace_text_frame(notesbody, "\n".join(lines))
        replacements = {slide_name: xml_bytes(tree), notesname: xml_bytes(notes)}
        buffer = io.BytesIO()
        with ZipFile(buffer, "w") as output:
            output.comment = source.comment
            for info in source.infolist():
                output.writestr(copy.copy(info), replacements.get(info.filename, source.read(info.filename)))
        updated = buffer.getvalue()
        with ZipFile(io.BytesIO(updated)) as after:
            assert after.testzip() is None and after.namelist() == source.namelist()
            changed = [n for n in source.namelist() if source.read(n) != after.read(n)]
            assert set(changed) <= set(replacements)
            assert all(source.read(n) == after.read(n) for n in source.namelist() if n.startswith("ppt/media/"))
            assert all(source.read(f"ppt/slides/slide{i}.xml") == after.read(f"ppt/slides/slide{i}.xml")
                       for i in range(1, 16) if i != 12)

    prs = Presentation(io.BytesIO(updated))
    manifest_file = OUT / "presentation_sources.json"
    manifest = json.loads(manifest_file.read_text())
    record = manifest["decks"][STEM][11]
    slide = prs.slides[11]
    record["body_text"] = "\n".join(s.text for s in slide.shapes if s.has_text_frame)
    record["notes"] = "图外补充参考路径、实际轨迹及如何判断跟踪效果的说明。"
    manifest["revision"] = "add a reading guide to the existing slide 12 plot; preserve other slides and all media"
    companion = OUT / "逐页讲稿与证据来源.md"
    prose = companion.read_text()
    replacement = f"## 第 12 页｜{record['stage']}｜{record['title']}\n\n" + slide.notes_slide.notes_text_frame.text + "\n\n"
    prose, count = re.subn(r"^## 第 12 页[^\n]*\n.*?(?=^## 第 13 页)", lambda _: replacement,
                          prose, count=1, flags=re.M | re.S)
    if count != 1:
        raise ValueError("Cannot identify page 12 companion section")
    audit = {"status": "pass", "snapshot_before": str(snapshot),
             "input_pptx_sha256": sha(original), "output_pptx_sha256": sha(updated),
             "changed_parts": changed, "preserved_slides": [i for i in range(1, 16) if i != 12],
             "other_14_slides_byte_identical": True, "all_media_byte_identical": True,
             "explanation": EXPLANATION}
    if sha(target.read_bytes()) != sha(current):
        raise RuntimeError("PPT was saved externally; rerun against the latest copy")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as temporary:
        temporary.write(updated)
        temporary_path = Path(temporary.name)
    temporary_path.replace(target)
    manifest_file.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    companion.write_text(prose)
    (OUT / "checks/page12_explanation_update.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
