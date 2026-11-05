"""Update only the cover movie and lateral-push slide of the current deck.

python3 Pro2/output/final/update_push_presentation.py

The current PPT may contain manual LibreOffice edits. Package parts outside
the cover and slide 9 are copied byte-for-byte, without rebuilding the deck.
"""
from __future__ import annotations

import argparse
import copy
import io
import json
import tempfile
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from PIL import Image
from pptx import Presentation

from update_cover_video import (NS, OUT, ROOT, STEM, VIDEO, POSTER,
                               patch_pptx_bytes, replace_paragraph_text,
                               resolve_relationship, sha, update_metadata,
                               xml_bytes)

SUMMARY = ROOT / "Pro2/output/L2/lateral_push/long_push_20261105/comparison/presentation_summary.json"


def replace_text_frame(frame, text: str) -> None:
    """Replace content while keeping the existing paragraph and run styles."""
    paragraphs = frame.findall("a:p", NS)
    if not paragraphs:
        raise ValueError("Text frame has no existing paragraph style")
    lines = text.split("\n")
    template = copy.deepcopy(next((p for p in paragraphs if p.find("a:r/a:t", NS) is not None), paragraphs[0]))
    while len(paragraphs) < len(lines):
        paragraph = copy.deepcopy(template)
        frame.append(paragraph)
        paragraphs.append(paragraph)
    for paragraph, line in zip(paragraphs, lines):
        if not paragraph.xpath(".//a:t", namespaces=NS):
            run = etree.Element("{" + NS["a"] + "}r")
            style = template.find("a:r/a:rPr", NS)
            if style is not None:
                run.append(copy.deepcopy(style))
            etree.SubElement(run, "{" + NS["a"] + "}t")
            ending = paragraph.find("a:endParaRPr", NS)
            paragraph.insert(paragraph.index(ending) if ending is not None else len(paragraph), run)
        replace_paragraph_text(paragraph, line)
    for paragraph in paragraphs[len(lines):]:
        frame.remove(paragraph)


def presentation_content(summary: dict) -> dict:
    content = summary["ppt"]
    required = {"condition", "table_rows", "conclusion", "reason", "config_path"}
    if not required <= set(content):
        raise ValueError(f"Missing verified PPT content: {required - set(content)}")
    if len(content["table_rows"]) != 4 or any(len(row) != 2 for row in content["table_rows"]):
        raise ValueError("Slide 9 requires the current four-row, two-column table")
    return content


def patch_push_slide(original: bytes, summary: dict) -> tuple[bytes, dict]:
    content = presentation_content(summary)
    figure = ROOT / summary["figure"]
    if not figure.is_file():
        raise FileNotFoundError(figure)
    slide_name = "ppt/slides/slide9.xml"
    rel_name = "ppt/slides/_rels/slide9.xml.rels"
    with ZipFile(io.BytesIO(original)) as source:
        tree = etree.fromstring(source.read(slide_name))
        relationships = etree.fromstring(source.read(rel_name))
        rels = {item.get("Id"): item for item in relationships}
        replacements = {}
        updates = {
            "TextBox 2": content["condition"],
            "TextBox 5": content["conclusion"],
            "TextBox 6": content["reason"],
            "TextBox 7": "配置：" + content["config_path"],
        }
        for shape_name, value in updates.items():
            found = tree.xpath("./p:cSld/p:spTree/p:sp[p:nvSpPr/p:cNvPr/@name=$name]",
                               namespaces=NS, name=shape_name)
            if len(found) != 1:
                raise ValueError(f"Cannot unambiguously identify slide 9 {shape_name}")
            replace_text_frame(found[0].find("p:txBody", NS), value)
        tables = tree.xpath(".//a:tbl", namespaces=NS)
        if len(tables) != 1:
            raise ValueError("Slide 9 must contain exactly one comparison table")
        rows = tables[0].findall("a:tr", NS)
        if len(rows) != 4 or any(len(row.findall("a:tc", NS)) != 2 for row in rows):
            raise ValueError("Current slide 9 table dimensions changed")
        for row, values in zip(rows, content["table_rows"]):
            for cell, value in zip(row.findall("a:tc", NS), values):
                replace_text_frame(cell.find("a:txBody", NS), str(value))

        pictures = tree.xpath("./p:cSld/p:spTree/p:pic", namespaces=NS)
        if len(pictures) != 1:
            raise ValueError("Slide 9 must contain exactly one source report picture")
        picture = pictures[0]
        rid = picture.xpath("./p:blipFill/a:blip/@r:embed", namespaces=NS)[0]
        image_part = resolve_relationship(rel_name, rels[rid].get("Target"))
        for name in source.namelist():
            if name == rel_name or not name.endswith(".rels"):
                continue
            for relation in etree.fromstring(source.read(name)):
                if relation.get("TargetMode") != "External" and resolve_relationship(name, relation.get("Target", "")) == image_part:
                    raise ValueError(f"Slide 9 image is shared with {name}")
        # Fit the new report image into the user's current image rectangle,
        # preserving its center and avoiding distortion or cropping.
        xfrm = picture.find("p:spPr/a:xfrm", NS)
        off, ext = xfrm.find("a:off", NS), xfrm.find("a:ext", NS)
        old_x, old_y, old_w, old_h = (int(off.get("x")), int(off.get("y")),
                                     int(ext.get("cx")), int(ext.get("cy")))
        with Image.open(figure) as image:
            width, height = image.size
        scale = min(old_w / width, old_h / height)
        new_w, new_h = round(width * scale), round(height * scale)
        off.set("x", str(old_x + (old_w - new_w) // 2))
        off.set("y", str(old_y + (old_h - new_h) // 2))
        ext.set("cx", str(new_w)); ext.set("cy", str(new_h))
        picture.find("p:nvPicPr/p:cNvPr", NS).set("name", "原图 " + summary["figure"])
        replacements[slide_name] = xml_bytes(tree)
        replacements[image_part] = figure.read_bytes()

        notes_rel = [r for r in relationships if r.get("Type", "").endswith("/notesSlide")]
        if len(notes_rel) != 1:
            raise ValueError("Missing notes for slide 9")
        notes_name = resolve_relationship(rel_name, notes_rel[0].get("Target"))
        notes_tree = etree.fromstring(source.read(notes_name))
        bodies = notes_tree.xpath(".//p:sp[p:nvSpPr/p:nvPr/p:ph[@type='body']]/p:txBody", namespaces=NS)
        if len(bodies) != 1:
            raise ValueError("Cannot identify slide 9 notes body")
        text = ["L2  对照实验四：侧向扰动", content["condition"],
                *[" | ".join(map(str, row)) for row in content["table_rows"]],
                content["conclusion"], content["reason"], "配置：" + content["config_path"],
                "证据来源", "Pro2/output/L2/L2验收报告.md", summary["figure"],
                str(SUMMARY.relative_to(ROOT))]
        replace_text_frame(bodies[0], "\n".join(text))
        replacements[notes_name] = xml_bytes(notes_tree)
        output = io.BytesIO()
        with ZipFile(output, "w") as destination:
            destination.comment = source.comment
            for info in source.infolist():
                destination.writestr(copy.copy(info), replacements.get(info.filename, source.read(info.filename)))
    return output.getvalue(), {"allowed_changed_parts": sorted(replacements),
                               "figure_source": summary["figure"], "figure_sha256": sha(figure.read_bytes())}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, default=SUMMARY)
    args = parser.parse_args()
    summary = json.loads(args.summary.read_text())
    pptx = OUT / (STEM + ".pptx")
    original = pptx.read_bytes()
    # Keep a recovery copy outside the delivery folder; there is still one
    # formal deck in output/final.
    snapshot = Path(tempfile.gettempdir()) / ("pro2_push_before_" + sha(original)[:12] + ".pptx")
    if not snapshot.exists():
        snapshot.write_bytes(original)
    audit_baseline = original
    previous_audit = {}
    previous_audit_path = OUT / "checks/push_presentation_update.json"
    if previous_audit_path.is_file():
        previous_audit = json.loads(previous_audit_path.read_text())
        prior_snapshot = Path(previous_audit.get("snapshot_before", ""))
        if (previous_audit.get("output_pptx_sha256") == sha(original) and prior_snapshot.is_file()
                and sha(prior_snapshot.read_bytes()) == previous_audit.get("input_pptx_sha256")):
            snapshot = prior_snapshot
            audit_baseline = snapshot.read_bytes()
    cover, cover_audit = patch_pptx_bytes(original, (ROOT / VIDEO).read_bytes(), (ROOT / POSTER).read_bytes())
    updated, push_audit = patch_push_slide(cover, summary)
    allowed = set(cover_audit["allowed_changed_parts"]) | set(push_audit["allowed_changed_parts"])
    with ZipFile(io.BytesIO(audit_baseline)) as before, ZipFile(io.BytesIO(updated)) as after:
        if before.namelist() != after.namelist() or after.testzip():
            raise ValueError("Package structure or ZIP CRC changed unexpectedly")
        changed = [name for name in before.namelist() if before.read(name) != after.read(name)]
        if not set(changed) <= allowed:
            raise ValueError("Changes exceed the cover and side-push slide")
        preserved = {name: sha(before.read(name)) for name in before.namelist() if name not in allowed}
        preserved_slides = [i for i in range(1, 16) if i not in (1, 9)]
        assert all(before.read(f"ppt/slides/slide{i}.xml") == after.read(f"ppt/slides/slide{i}.xml") for i in preserved_slides)
    old_manifest = json.loads((OUT / "presentation_sources.json").read_text())
    page9 = old_manifest["decks"][STEM][8]
    old_figures = {image["source"] for image in page9["images"]}
    page9["images"] = [{"source": summary["figure"], "sha256": push_audit["figure_sha256"]}]
    page9["sources"] = [source for source in page9["sources"] if source not in old_figures]
    page9["sources"].extend([summary["figure"], str(args.summary.resolve().relative_to(ROOT))])
    page9["tables"] = [summary["ppt"]["table_rows"]]
    manifest, notes, user_changes = update_metadata(Presentation(io.BytesIO(updated)), old_manifest)
    manifest["revision"] = "longer visible lateral push: replace cover media and slide 9; preserve all other current user-edited slides"
    notes += "\n## 第 9 页更新后的数据表\n\n" + "\n".join(
        " | ".join(map(str, row)) for row in summary["ppt"]["table_rows"]) + "\n"
    audit = {"status": "pass", "snapshot_before": str(snapshot),
             "input_pptx_sha256": sha(audit_baseline), "output_pptx_sha256": sha(updated),
             "changed_parts": changed, "allowed_changed_parts": sorted(allowed),
             "preserved_slides": preserved_slides, "preserved_slide_xml_byte_identical": True,
             "preserved_part_count": len(preserved), "preserved_part_sha256": preserved,
             "preserved_user_text_changes": previous_audit.get("preserved_user_text_changes", []) + [change for change in user_changes if change["page"] != 9],
             "cover_media": cover_audit["media"], "push": push_audit,
             "analysis_summary": str(args.summary.resolve().relative_to(ROOT)),
             "analysis_summary_sha256": sha(args.summary.read_bytes())}
    if sha(pptx.read_bytes()) != sha(original):
        raise RuntimeError("PPT changed during preparation; rerun against the latest saved file")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as handle:
        handle.write(updated)
        temp = Path(handle.name)
    temp.replace(pptx)
    (OUT / "presentation_sources.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (OUT / "逐页讲稿与证据来源.md").write_text(notes)
    (OUT / "checks/push_presentation_update.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: audit[key] for key in ("status", "input_pptx_sha256", "output_pptx_sha256", "changed_parts", "preserved_slides")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
