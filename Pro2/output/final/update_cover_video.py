"""Replace the current PPT's cover video while preserving all other slides.

Run after the montage and poster have been rendered:
    python3 Pro2/output/final/update_cover_video.py

The existing PPT may contain user edits made in LibreOffice. This command
patches only its cover XML, cover notes, and the two cover media parts. It
does not reconstruct the presentation from the original slide generator.
"""
from __future__ import annotations

import copy
import hashlib
import io
import json
import posixpath
import re
import tempfile
from pathlib import Path
from zipfile import ZipFile

from lxml import etree
from pptx import Presentation

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
STEM = "青龙MPC_项目汇报_15页"
VIDEO = "Pro2/output/final/assets/项目验收演示_L0_L2_L4.mp4"
POSTER = "Pro2/output/final/assets/poster.png"
PREVIOUS_CAPTION = "2 分 40 秒：跑通、四组对照与综合演示"
OLD_CAPTION = "点击画面：2 分 30 秒默认运行与参数对照"
OLD_VIDEO = "Pro2/output/final/assets/项目验收演示_150s.mp4"
OLD_POSTER = "Pro2/output/final/assets/演示视频封面.png"
NS = {
    "a": "http://schemas.openxmlformats.org/drawingml/2006/main",
    "p": "http://schemas.openxmlformats.org/presentationml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def xml_bytes(tree) -> bytes:
    return etree.tostring(tree, xml_declaration=True, encoding="UTF-8", standalone=True)


def replace_paragraph_text(paragraph, value: str) -> None:
    nodes = paragraph.xpath(".//a:t", namespaces=NS)
    if not nodes:
        raise ValueError("The expected text paragraph has no text runs")
    nodes[0].text = value
    for node in nodes[1:]:
        node.text = ""


def resolve_relationship(rel_name: str, target: str) -> str:
    owner_dir = posixpath.dirname(rel_name).replace("/_rels", "")
    return posixpath.normpath(posixpath.join(owner_dir, target))


def video_manifest() -> dict:
    """Use the verified montage's duration, never an obsolete fixed duration."""
    data = json.loads((OUT / "assets/video_sources.json").read_text())
    duration = float(data["metadata"]["duration_s"])
    if duration <= 0 or not data.get("full_decode_success"):
        raise ValueError("The montage must have a positive duration and a successful full decode")
    if abs(float(data["timeline"][-1]["end_s"]) - duration) > 0.05:
        raise ValueError("Montage timeline and media duration disagree")
    return data


def cover_caption(data: dict) -> str:
    minutes, seconds = divmod(round(float(data["metadata"]["duration_s"])), 60)
    return f"{minutes} 分 {seconds} 秒：跑通、四组对照与综合演示"


def cover_explanation(data: dict) -> str:
    duration = round(float(data["metadata"]["duration_s"]))
    minutes, seconds = divmod(duration, 60)
    lines = [f"演示安排：共 {duration} 秒（{minutes} 分 {seconds} 秒），依次展示 L0 跑通、L2 四组对照与 L4 综合演示。"]
    def timestamp(value: float) -> str:
        minute, second = divmod(round(value), 60)
        return f"{minute:02}:{second:02}"
    quad = [segment for segment in data["timeline"] if segment.get("kind") == "quad"]
    if quad:
        segment = quad[0]
        start, end = timestamp(segment["start_s"]), timestamp(segment["end_s"])
        window = round(segment["duration_s"])
        lines.append(f"{start}–{end}：5° 坡道、粗糙地面、随机障碍与 6 cm 台阶按 2×2 同屏播放，四组均展示前 {window} 秒并按仿真时间同步。")
        lines.append("这段画面用于观察不同地形下的步态；完整录像保留各自运行时长，姿态、速度和换腿节奏的分析见 L4 报告第5节。")
    for segment in data["timeline"]:
        if "jump" in segment["name"]:
            label = "跳跃关键动作 0.5 倍速回放" if segment.get("speed") == 0.5 else "竖直跳跃"
            lines.append(f"{timestamp(segment['start_s'])}–{timestamp(segment['end_s'])}：{label}。")
    lines.append(f"综合视频引用 {len(data['sources'])} 条原片，原始录像与配置按 L0、L2、L4 分别保存。")
    return "\n".join(lines)


def patch_pptx_bytes(original: bytes, video: bytes, poster: bytes, montage: dict | None = None) -> tuple[bytes, dict]:
    """Return a patched package and a byte-level preservation audit."""
    montage = montage or video_manifest()
    caption = cover_caption(montage)
    explanation = cover_explanation(montage)
    slide_name = "ppt/slides/slide1.xml"
    rel_name = "ppt/slides/_rels/slide1.xml.rels"
    with ZipFile(io.BytesIO(original)) as source:
        names = source.namelist()
        if len(names) != len(set(names)):
            raise ValueError("PPT package contains duplicate part names")
        if source.testzip():
            raise ValueError("PPT package failed ZIP CRC verification")
        tree = etree.fromstring(source.read(slide_name))
        rel_tree = etree.fromstring(source.read(rel_name))
        rels = {rel.get("Id"): rel for rel in rel_tree}
        movies = tree.xpath(".//p:pic[p:nvPicPr/p:nvPr/a:videoFile]", namespaces=NS)
        if len(movies) != 1:
            raise ValueError(f"Expected one cover movie, found {len(movies)}")
        movie = movies[0]
        video_rid = movie.xpath("./p:nvPicPr/p:nvPr/a:videoFile/@r:link", namespaces=NS)[0]
        poster_rid = movie.xpath("./p:blipFill/a:blip/@r:embed", namespaces=NS)[0]
        for rid in [video_rid, poster_rid]:
            if rels[rid].get("TargetMode") == "External":
                raise ValueError("Cover media must already be embedded")
        video_part = resolve_relationship(rel_name, rels[video_rid].get("Target"))
        poster_part = resolve_relationship(rel_name, rels[poster_rid].get("Target"))
        if not video_part.endswith(".mp4") or not poster_part.endswith(".png"):
            raise ValueError("Expected an existing MP4 movie and PNG cover poster")

        # Replacing a shared media part could change another slide. Refuse that
        # case rather than silently altering the rest of the user's deck.
        for name in names:
            if name == rel_name or not name.endswith(".rels"):
                continue
            for rel in etree.fromstring(source.read(name)):
                if rel.get("TargetMode") == "External":
                    continue
                target = resolve_relationship(name, rel.get("Target", ""))
                if target in {video_part, poster_part}:
                    raise ValueError(f"Cover media is shared by another package part: {name}")

        caption_paragraphs = []
        for paragraph in tree.xpath(".//p:sp/p:txBody/a:p", namespaces=NS):
            value = "".join(paragraph.xpath(".//a:t/text()", namespaces=NS))
            if value in {OLD_CAPTION, PREVIOUS_CAPTION, caption}:
                caption_paragraphs.append(paragraph)
        if len(caption_paragraphs) > 1:
            raise ValueError("Cannot identify the cover movie caption unambiguously")
        # The user may intentionally remove the caption in LibreOffice. Do not
        # restore deleted content when the only requested change is the movie.
        replacements = {video_part: video, poster_part: poster}
        if caption_paragraphs:
            current_caption = "".join(caption_paragraphs[0].xpath(".//a:t/text()", namespaces=NS))
            if current_caption != caption:
                replace_paragraph_text(caption_paragraphs[0], caption)
                replacements[slide_name] = xml_bytes(tree)

        notes_rels = [rel for rel in rel_tree if rel.get("Type", "").endswith("/notesSlide")]
        if len(notes_rels) != 1:
            raise ValueError("Expected exactly one notes part for the cover")
        notes_name = resolve_relationship(rel_name, notes_rels[0].get("Target"))
        notes = etree.fromstring(source.read(notes_name))
        notes_changed = False
        for paragraph in notes.xpath(".//a:p", namespaces=NS):
            value = "".join(paragraph.xpath(".//a:t/text()", namespaces=NS))
            updated = value.replace(OLD_CAPTION, caption).replace(PREVIOUS_CAPTION, caption).replace(OLD_VIDEO, VIDEO).replace(OLD_POSTER, POSTER)
            if value.startswith("演示安排："):
                updated = explanation
            if updated != value:
                replace_paragraph_text(paragraph, updated)
                notes_changed = True
        body = notes.xpath('.//p:sp[p:nvSpPr/p:nvPr/p:ph[@type="body"]]/p:txBody', namespaces=NS)
        if len(body) != 1:
            raise ValueError("Expected one notes body on the cover")
        paragraphs = body[0].findall("a:p", NS)
        if not any("".join(p.xpath(".//a:t/text()", namespaces=NS)).startswith("演示安排：") for p in paragraphs):
            source_heading = [p for p in paragraphs if "".join(p.xpath(".//a:t/text()", namespaces=NS)) == "证据来源"]
            if len(source_heading) != 1:
                raise ValueError("Cannot identify the cover notes source heading")
            new_paragraph = copy.deepcopy(source_heading[0])
            replace_paragraph_text(new_paragraph, explanation)
            body[0].insert(body[0].index(source_heading[0]), new_paragraph)
            notes_changed = True
        if notes_changed:
            replacements[notes_name] = xml_bytes(notes)

        result = io.BytesIO()
        with ZipFile(result, "w") as output:
            output.comment = source.comment
            for info in source.infolist():
                output.writestr(copy.copy(info), replacements.get(info.filename, source.read(info.filename)))
        patched = result.getvalue()
        with ZipFile(io.BytesIO(patched)) as output:
            if output.testzip() or output.namelist() != names:
                raise ValueError("Updated package failed integrity verification")
            changed = [name for name in names if source.read(name) != output.read(name)]
            if not set(changed) <= set(replacements):
                raise ValueError("A package part outside the cover was changed")
            preserved = {name: sha(source.read(name)) for name in names if name not in replacements}
            unchanged_slides = [name for name in names if name.startswith("ppt/slides/slide")
                                and name.endswith(".xml") and name != slide_name]
            if len(unchanged_slides) != 14:
                raise ValueError("Expected exactly fourteen preserved non-cover slides")
    audit = {
        "status": "pass", "input_pptx_sha256": sha(original), "output_pptx_sha256": sha(patched),
        "changed_parts": changed, "allowed_changed_parts": sorted(replacements),
        "preserved_part_count": len(preserved), "preserved_part_sha256": preserved,
        "slides_2_through_15_byte_identical": True,
        "slide1_xml_byte_identical": slide_name not in changed,
        "cover_video_duration_s": montage["metadata"]["duration_s"],
        "media": {"video_part": video_part, "poster_part": poster_part,
                  "video_source": VIDEO, "poster_source": POSTER,
                  "video_sha256": sha(video), "poster_sha256": sha(poster)},
    }
    return patched, audit


def update_metadata(prs: Presentation, previous: dict, montage: dict | None = None) -> tuple[dict, str, list]:
    montage = montage or video_manifest()
    manifest = copy.deepcopy(previous)
    if list(manifest["decks"]) != [STEM] or len(prs.slides) != 15:
        raise ValueError("Expected the existing single fifteen-page presentation")
    pages = manifest["decks"][STEM]
    cover = pages[0]
    cover["sources"] = [VIDEO if value == OLD_VIDEO else POSTER if value == OLD_POSTER else value
                        for value in cover["sources"]]
    cover["videos"] = [{"source": VIDEO, "sha256": sha((ROOT / VIDEO).read_bytes()),
                         "poster": POSTER, "embedded": True, "autoplay": False}]
    cover["body_text"] = "\n".join(shape.text for shape in prs.slides[0].shapes if shape.has_text_frame)
    cover["notes"] = cover_explanation(montage)
    # Detailed explanations on later pages are hand-edited evidence. Regenerating
    # them from slide text would delete the explanations for pages 4, 12 and 13.
    original_notes = (OUT / "逐页讲稿与证据来源.md").read_text()
    heading = re.search(r"^## 第 1 页｜.*$", original_notes, re.MULTILINE)
    next_heading = re.search(r"^## 第 2 页｜", original_notes, re.MULTILINE)
    if not heading or not next_heading or heading.start() >= next_heading.start():
        raise ValueError("Expected distinct first- and second-page sections in the speaker notes")
    cover_section = (heading.group(0) + "\n\n" + cover["body_text"] + "\n\n"
                     + cover["notes"] + "\n\n证据来源\n" + "\n".join(cover["sources"]) + "\n\n")
    notes = original_notes[:heading.start()] + cover_section + original_notes[next_heading.start():]
    sources = {value for record in pages for value in record["sources"]}
    manifest["source_sha256"] = {value: sha((ROOT / value).read_bytes()) for value in sorted(sources)}
    manifest["revision"] = "cover-only video update; preserve current user-edited fifteen-page deck"
    manifest["cover_video_duration_s"] = montage["metadata"]["duration_s"]
    return manifest, notes, []


def main() -> None:
    pptx = OUT / (STEM + ".pptx")
    original = pptx.read_bytes()
    video, poster = (ROOT / VIDEO).read_bytes(), (ROOT / POSTER).read_bytes()
    montage = video_manifest()
    if sha(video) != montage["sha256"]:
        raise ValueError("The cover video does not match its verified source manifest")
    updated, audit = patch_pptx_bytes(original, video, poster, montage)
    manifest_path = OUT / "presentation_sources.json"
    manifest, notes, user_changes = update_metadata(Presentation(io.BytesIO(updated)), json.loads(manifest_path.read_text()), montage)
    audit["preserved_user_text_changes"] = user_changes
    snapshot = Path(tempfile.gettempdir()) / f"pro2_cover_before_{sha(original)[:12]}.pptx"
    if snapshot.exists() and sha(snapshot.read_bytes()) != sha(original):
        raise RuntimeError("Existing cover snapshot has an unexpected checksum")
    snapshot.write_bytes(original)
    audit["snapshot_before"] = str(snapshot)
    old_notes = (OUT / "逐页讲稿与证据来源.md").read_text()
    marker = "## 第 2 页｜"
    if old_notes[old_notes.index(marker):] != notes[notes.index(marker):]:
        raise ValueError("Speaker notes after page one changed during the cover update")
    audit["speaker_notes_pages_2_through_15_byte_identical"] = True
    # Abort if an external editor saved again while this update was prepared.
    if sha(pptx.read_bytes()) != sha(original):
        raise RuntimeError("PPT changed during preparation; rerun against its latest saved state")
    with tempfile.NamedTemporaryFile(dir=OUT, suffix=".tmp", delete=False) as handle:
        temp_path = Path(handle.name)
        handle.write(updated)
    try:
        temp_path.replace(pptx)
    finally:
        temp_path.unlink(missing_ok=True)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    (OUT / "逐页讲稿与证据来源.md").write_text(notes)
    (OUT / "checks").mkdir(exist_ok=True)
    (OUT / "checks/cover_video_update.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: audit[key] for key in ["status", "changed_parts", "preserved_part_count",
                                                "slides_2_through_15_byte_identical"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
