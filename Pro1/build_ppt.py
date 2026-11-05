"""Build an editable, ten-slide raa-v0 presentation from completed runs.

Run from the project root: python3 Pro1/build_ppt.py
No simulation is launched and no experiment outputs are overwritten.
Text, tables, flowchart, outline plots and time-series chart are native PPTX
objects. Actual Isaac screenshots are raster pictures; eight original MP4
videos are embedded in the PPTX, each with an editable explanation below it.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from zipfile import ZipFile

from lxml import etree

from pptx import Presentation
from pptx.chart.data import XyChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import XL_CHART_TYPE, XL_LEGEND_POSITION, XL_TICK_MARK
from pptx.enum.dml import MSO_LINE_DASH_STYLE
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.xmlchemy import OxmlElement
from pptx.util import Inches, Pt


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "ppt"
W, H = 13.333333, 7.5
TOTAL_SLIDES = 10
FONT = "Noto Sans CJK SC"
BG = "FAFBFC"
INK = "20343D"
MUTED = "65747B"
TEAL = "008C82"
PALE = "EAF5F3"
GRID = "DCE4E7"
TARGET = "8D9AA2"
ORANGE = "C77936"
BLUE = "5075AA"
WHITE = "FFFFFF"
CONTROL = ("pd", "gc-pd", "rbf", "fuzzy")
LABEL = {"pd": "PD", "gc-pd": "重力补偿 PD", "rbf": "RBF 自适应", "fuzzy": "模糊自适应"}
SHORT = {"pd": "PD", "gc-pd": "重力补偿 PD", "rbf": "RBF", "fuzzy": "模糊"}
SERIES_COLOR = {"pd": INK, "gc-pd": BLUE, "rbf": ORANGE, "fuzzy": TEAL}
SERIES_DASH = {"pd": MSO_LINE_DASH_STYLE.SOLID, "gc-pd": MSO_LINE_DASH_STYLE.DASH,
               "rbf": MSO_LINE_DASH_STYLE.LONG_DASH, "fuzzy": MSO_LINE_DASH_STYLE.SOLID}
PPT_NS = {"p": "http://schemas.openxmlformats.org/presentationml/2006/main",
          "a": "http://schemas.openxmlformats.org/drawingml/2006/main"}
VIDEO_SUBTITLE_OLD = "点击视频画面播放；每个视频对应一种控制器。"
VIDEO_SUBTITLE = "进入本页后，四个视频自动同时播放。"


def rgb(value):
    return RGBColor.from_string(value)


def set_font(run, size, color=INK, bold=False):
    run.font.name = FONT
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = rgb(color)
    props = run._r.get_or_add_rPr()
    props.set("lang", "zh-CN")
    for name in ("a:ea", "a:cs"):
        node = props.find(name, props.nsmap)
        if node is None:
            node = OxmlElement(name)
            props.append(node)
        node.set("typeface", FONT)


def text(slide, x, y, w, h, content, size=22, color=INK, bold=False,
         align=PP_ALIGN.LEFT, vertical=MSO_ANCHOR.TOP, name=None):
    shape = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    shape.name = name or content.split("\n")[0][:48]
    tf = shape.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.015)
    tf.margin_top = tf.margin_bottom = Inches(0.02)
    tf.vertical_anchor = vertical
    for i, line in enumerate(content.split("\n")):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        p.space_before = Pt(0)
        p.space_after = Pt(5)
        p.line_spacing = 1.12
        run = p.add_run()
        run.text = line
        set_font(run, size, color, bold)
    return shape


def rect(slide, x, y, w, h, fill=WHITE, line_color=None, radius=False, name=""):
    s = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE if radius else MSO_SHAPE.RECTANGLE,
                               Inches(x), Inches(y), Inches(w), Inches(h))
    if radius:
        s.adjustments[0] = 0.05
    s.fill.solid()
    s.fill.fore_color.rgb = rgb(fill)
    if line_color:
        s.line.color.rgb = rgb(line_color)
        s.line.width = Pt(0.7)
    else:
        s.line.fill.background()
    if name:
        s.name = name
    return s


def line(slide, x1, y1, x2, y2, color=GRID, width=1, dash=None):
    s = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1),
                                   Inches(x2), Inches(y2))
    s.line.color.rgb = rgb(color)
    s.line.width = Pt(width)
    if dash:
        s.line.dash_style = dash
    return s


def picture(slide, source, x, y, w, h):
    # These project viewport captures have a 4:3 aspect ratio; preserve it.
    return slide.shapes.add_picture(str(source), Inches(x), Inches(y),
                                    width=Inches(w), height=Inches(h))


def base(prs, number, title=None, subtitle=None):
    s = prs.slides.add_slide(prs.slide_layouts[6])
    s.background.fill.solid()
    s.background.fill.fore_color.rgb = rgb(BG)
    rect(s, 0.6, 0.53, 0.075, 0.43, TEAL)
    if title:
        text(s, 0.82, 0.48, 11.82, 0.64, title, 30, bold=True)
    if subtitle:
        text(s, 0.82, 1.13, 11.7, 0.45, subtitle, 18, MUTED)
    line(s, 0.6, 7.02, 12.73, 7.02, GRID, 0.6)
    text(s, 0.6, 7.12, 8.2, 0.2, "robot arm artist  ·  raa-v0", 9, MUTED)
    text(s, 12.0, 7.10, 0.73, 0.25, f"{number:02d} / {TOTAL_SLIDES:02d}", 10, MUTED, align=PP_ALIGN.RIGHT)
    return s


def table(slide, x, y, w, h, rows, widths=None, size=20, highlight_row=None):
    shape = slide.shapes.add_table(len(rows), len(rows[0]), Inches(x), Inches(y),
                                  Inches(w), Inches(h))
    tab = shape.table
    if widths:
        for col, fraction in zip(tab.columns, widths):
            col.width = Inches(w * fraction)
    for r, values in enumerate(rows):
        tab.rows[r].height = Inches(h / len(rows))
        for c, value in enumerate(values):
            cell = tab.cell(r, c)
            cell.text = str(value)
            cell.margin_left = Inches(0.13)
            cell.margin_right = Inches(0.10)
            cell.margin_top = cell.margin_bottom = Inches(0.035)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.fill.solid()
            cell.fill.fore_color.rgb = rgb(PALE if r == 0 or r == highlight_row else WHITE)
            # Explicit cell borders avoid template-dependent decoration.
            tcpr = cell._tc.get_or_add_tcPr()
            for tag in ("lnL", "lnR", "lnT", "lnB"):
                node = OxmlElement("a:" + tag)
                node.set("w", "6350")
                solid = OxmlElement("a:solidFill")
                srgb = OxmlElement("a:srgbClr")
                srgb.set("val", GRID)
                solid.append(srgb)
                node.append(solid)
                tcpr.append(node)
            for p in cell.text_frame.paragraphs:
                p.space_before = p.space_after = Pt(0)
                p.alignment = PP_ALIGN.LEFT if c == 0 else PP_ALIGN.CENTER
                for run in p.runs:
                    set_font(run, size, TEAL if r == highlight_row else INK, r == 0)
    return shape


def rdp(points, epsilon=0.05):
    """Simplify only the displayed mm polyline, never the numeric metrics."""
    if len(points) < 3:
        return points
    a, b = points[0], points[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    largest, index = -1.0, 0
    for i in range(1, len(points) - 1):
        p = points[i]
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom)) if denom else 0
        distance = math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)
        if distance > largest:
            largest, index = distance, i
    if largest <= epsilon:
        return [a, b]
    return rdp(points[:index + 1], epsilon)[:-1] + rdp(points[index:], epsilon)


def read_strokes(path):
    target, actual = {}, {}
    current, previous = None, None
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        fields = next(reader)
        at = {name: fields.index(name) for name in
              ("target_x", "target_y", "actual_x", "actual_y", "pen_down", "stroke_index")}
        for index, row in enumerate(reader):
            down = row[at["pen_down"]] in ("True", "true", "1")
            stroke = int(row[at["stroke_index"]]) if down else None
            if stroke != current:
                if current is not None and previous:
                    target[current].append(previous[0])
                    actual[current].append(previous[1])
                current = stroke
                previous = None
            if not down:
                continue
            point = ((float(row[at["target_x"]]) * 1000, float(row[at["target_y"]]) * 1000),
                     (float(row[at["actual_x"]]) * 1000, float(row[at["actual_y"]]) * 1000))
            if stroke not in target:
                target[stroke], actual[stroke] = [], []
            if not target[stroke] or index % 10 == 0:
                target[stroke].append(point[0])
                actual[stroke].append(point[1])
            previous = point
    if current is not None and previous:
        target[current].append(previous[0])
        actual[current].append(previous[1])
    assert len(target) == len(actual) == 8, path
    return {"target": [rdp(target[k]) for k in sorted(target)],
            "actual": [rdp(actual[k]) for k in sorted(actual)]}


def polyline(slide, points, color=TEAL, width=1.4, dash=None, name="editable trajectory"):
    if len(points) < 2:
        return None
    # Local integer coordinates avoid rounding paths in the inch coordinate system.
    sx, sy = points[0]
    builder = slide.shapes.build_freeform(round(sx * 10000), round(sy * 10000),
                                          scale=Inches(1) / 10000)
    builder.add_line_segments([(round(x * 10000), round(y * 10000)) for x, y in points[1:]], close=False)
    shape = builder.convert_to_shape()
    shape.fill.background()
    shape.line.color.rgb = rgb(color)
    shape.line.width = Pt(width)
    if dash:
        shape.line.dash_style = dash
    shape.name = name
    return shape


def legend(slide, x, y, entries, size=16, spacing=1.26):
    for i, (label, color, dash) in enumerate(entries):
        offset = x + i * spacing
        line(slide, offset, y + 0.13, offset + 0.34, y + 0.13, color, 1.8, dash)
        text(slide, offset + 0.42, y - 0.035, spacing - 0.42, 0.35, label, size, MUTED)


def outline_plot(slide, x, y, w, h, title, entries, *, small=False, axes=True):
    # Every trajectory chart has the same mm domain and equal spatial aspect.
    title_h = 0.40
    text(slide, x, y, w, title_h, title, 20 if small else 22, bold=True,
         align=PP_ALIGN.CENTER)
    if axes:
        reserve_left, reserve_bottom = (0.53, 0.61) if small else (0.64, 0.63)
        side = min(w - reserve_left - 0.16, h - title_h - reserve_bottom)
        px = x + reserve_left + (w - reserve_left - 0.16 - side) / 2
        py = y + title_h + 0.08
        xmin, xmax, ymin, ymax = 320.0, 580.0, -130.0, 130.0
        ticks_x, ticks_y = (350, 450, 550), (-100, 0, 100)
        label_size = 13 if small else 16
        for value in ticks_x:
            xx = px + (value - xmin) / (xmax - xmin) * side
            line(slide, xx, py, xx, py + side, GRID, 0.45)
            text(slide, xx - 0.25, py + side + 0.03, 0.5, 0.27, str(value), label_size,
                 MUTED, align=PP_ALIGN.CENTER)
        for value in ticks_y:
            yy = py + (ymax - value) / (ymax - ymin) * side
            line(slide, px, yy, px + side, yy, GRID, 0.45)
            text(slide, px - 0.70, yy - 0.135, 0.60, 0.28, str(value), label_size,
                 MUTED, align=PP_ALIGN.RIGHT)
        line(slide, px, py, px, py + side, TARGET, 0.8)
        line(slide, px, py + side, px + side, py + side, TARGET, 0.8)
        text(slide, px, py + side + 0.32, side, 0.30, "X / mm", label_size + 1,
             MUTED, align=PP_ALIGN.CENTER)
        text(slide, px - 0.57, py - 0.13, 0.90, 0.30, "Y / mm", label_size, MUTED)
    else:
        side = min(w, h - title_h - 0.12)
        px, py = x + (w - side) / 2, y + title_h + 0.12
        xmin, xmax, ymin, ymax = 360.0, 540.0, -90.0, 90.0
    for label, strokes, color, dash in entries:
        for index, stroke in enumerate(strokes):
            for point in stroke:
                assert xmin <= point[0] <= xmax and ymin <= point[1] <= ymax, (label, point)
            points = [(px + (a - xmin) / (xmax - xmin) * side,
                       py + (ymax - b) / (ymax - ymin) * side) for a, b in stroke]
            polyline(slide, points, color, 1.35 if small else 1.7, dash,
                     f"{title} / {label} / stroke {index + 1}")


def load_notes():
    source = OUT / "speaker_notes.md"
    if not source.exists():
        return {}
    notes = {}
    parts = re.split(r"(?m)^##\s+", source.read_text())
    for part in parts[1:]:
        first, _, body = part.partition("\n")
        match = re.match(r"(?:第\s*)?(\d+)", first)
        if match:
            notes[int(match.group(1))] = body.strip()
    return notes


def configure_parallel_video_autoplay(slide_root):
    """Start all four media nodes together at slide-root time zero.

    Native p:video nodes are siblings in one parallel time container. Changing
    their start conditions from indefinite (manual) to zero starts them with
    the slide, rather than sequencing them through individual clicks. Keep
    media durations implicit so each MP4 uses its actual playback duration.
    """
    children = slide_root.find(
        'p:timing/p:tnLst/p:par/p:cTn[@nodeType="tmRoot"]/p:childTnLst', PPT_NS)
    if children is None:
        raise ValueError("Video slide has no shared parallel timing root")
    videos = children.findall("p:video", PPT_NS)
    if len(videos) != 4:
        raise ValueError("Each video slide must have four parallel video nodes")
    shape_ids = []
    for video in videos:
        ctn = video.find("p:cMediaNode/p:cTn", PPT_NS)
        conditions = ctn.find("p:stCondLst", PPT_NS) if ctn is not None else None
        target = video.find("p:cMediaNode/p:tgtEl/p:spTgt", PPT_NS)
        if conditions is None or target is None:
            raise ValueError("Video timing requires a start condition and shape target")
        starts = conditions.findall("p:cond", PPT_NS)
        if len(starts) != 1 or starts[0].get("evt") is not None or len(starts[0]):
            raise ValueError("Unexpected interactive start condition; cannot silently replace it")
        starts[0].set("delay", "0")
        shape_ids.append(target.get("spid"))
    if len(set(shape_ids)) != 4:
        raise ValueError("The four video nodes must target distinct movie shapes")
    return shape_ids


def synchronize_existing_presentation(destination):
    """Patch playback in the existing PPTX, preserving the user's other edits."""
    if not destination.is_file():
        raise FileNotFoundError(destination)
    temporary = destination.with_name(f".{destination.stem}.syncing.pptx")
    changed, targets, original_media_hashes = [], {}, {}
    text_replacements = {
        VIDEO_SUBTITLE_OLD: VIDEO_SUBTITLE,
        "放映时直接点击画面播放": "进入本页后，四个视频自动同时播放",
        "放映时点击各自的画面即可播放": "进入本页后四个视频自动同时播放",
        "点击画面即可播放": "进入本页后四个视频自动同时播放",
    }
    xml_names = {f"ppt/slides/slide{i}.xml" for i in (8, 9)}
    xml_names.update(f"ppt/notesSlides/notesSlide{i}.xml" for i in (8, 9))
    try:
        with ZipFile(destination) as source, ZipFile(temporary, "w") as result:
            for item in source.infolist():
                if item.filename in xml_names:
                    original = source.read(item.filename)
                    root = etree.fromstring(original)
                    if item.filename.startswith("ppt/slides/"):
                        targets[item.filename] = configure_parallel_video_autoplay(root)
                    for element in root.findall(".//a:t", PPT_NS):
                        if element.text:
                            for old, new in text_replacements.items():
                                element.text = element.text.replace(old, new)
                    updated = etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)
                    result.writestr(item, updated)
                    if original != updated:
                        changed.append(item.filename)
                else:
                    # Copy the complete existing media and other package parts.
                    # Video bytes are not decoded or re-encoded.
                    digest = hashlib.sha256() if item.filename.endswith(".mp4") else None
                    with source.open(item) as incoming, result.open(item, "w") as outgoing:
                        while chunk := incoming.read(1024 * 1024):
                            outgoing.write(chunk)
                            if digest is not None:
                                digest.update(chunk)
                    if digest is not None:
                        original_media_hashes[item.filename] = digest.hexdigest()
        if len(targets) != 2 or len(original_media_hashes) != 8:
            raise ValueError("Expected two video slides and eight embedded MP4 files")
        with ZipFile(temporary) as result:
            for name, expected in original_media_hashes.items():
                with result.open(name) as stream:
                    actual = hashlib.file_digest(stream, "sha256").hexdigest()
                if actual != expected:
                    raise ValueError(f"Embedded video changed unexpectedly: {name}")
            for name in targets:
                root = etree.fromstring(result.read(name))
                nodes = root.findall(".//p:video/p:cMediaNode/p:cTn/p:stCondLst/p:cond", PPT_NS)
                if len(nodes) != 4 or any(n.get("delay") != "0" or n.get("evt") for n in nodes):
                    raise ValueError(f"Parallel zero-delay playback validation failed: {name}")
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    report = {"presentation": str(destination), "video_slides": [8, 9],
              "mode": "four videos automatically start together on slide entry",
              "start_delay_ms": 0, "parallel_targets": targets,
              "modified_package_entries": changed,
              "embedded_media_sha256": original_media_hashes,
              "embedded_videos_unchanged": True,
              "pptx_sha256": hashlib.file_digest(destination.open("rb"), "sha256").hexdigest()}
    (OUT / "video_playback_verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    sources_file = OUT / "presentation_sources.json"
    if sources_file.exists():
        sources = json.loads(sources_file.read_text())
        sources["video_delivery"] = "8 complete internally embedded MP4 files; four videos start automatically in parallel on each video slide; no external video links"
        sources["video_playback"] = {"trigger": "slide entry", "start_delay_ms": 0,
                                      "mode": "parallel", "videos_per_slide": 4, "slides": [8, 9]}
        sources_file.write_text(json.dumps(sources, ensure_ascii=False, indent=2) + "\n")
    print("Updated both video slides: four parallel videos, zero start delay, all eight MP4s unchanged.")
    return report


def video_slide(prs, number, condition, source_paths):
    normal = condition == "normal"
    title = "正常绘画：四种控制器的视频" if normal else "外力干扰：四种控制器的视频"
    subtitle = VIDEO_SUBTITLE
    slide = base(prs, number, title, subtitle)
    explanations = {
        "normal": {
            "pd": "按位置偏差纠正。\n自重使轮廓偏离明显。",
            "gc-pd": "先抵消机械臂自重。\n轮廓更贴近目标线。",
            "rbf": "边画边学习补偿。\n轮廓接近目标线。",
            "fuzzy": "根据误差规则补偿。\n本次整体误差最小。",
        },
        "disturbed": {
            "pd": "推力增加轨迹偏移。\n撤力后逐步恢复。",
            "gc-pd": "受推时短暂偏离。\n本次撤力恢复最快。",
            "rbf": "受推时在线学补偿。\n偏移后仍能继续画。",
            "fuzzy": "按误差持续更新补偿。\n本次受推误差最小。",
        },
    }
    embedded = []
    for i, controller in enumerate(CONTROL):
        x, width = 0.82 + i * 2.97, 2.78
        video = ROOT / f"video/{controller}_{condition}.mp4"
        stamp = "113_9s" if normal else "55_5s"
        poster = ROOT / f"output/video_checks/{controller}_{condition}/frame_{stamp}.png"
        if not video.is_file() or not poster.is_file():
            raise FileNotFoundError(f"Missing original video or poster: {video}, {poster}")
        print(f"Embedding complete video: {video.name}", flush=True)
        movie = slide.shapes.add_movie(str(video), Inches(x), Inches(2.13),
                                       Inches(width), Inches(width * 0.75),
                                       poster_frame_image=str(poster), mime_type="video/mp4")
        movie.name = f"Embedded video / {controller} / {condition}"
        text(slide, x, 4.48, width, 0.48, LABEL[controller], 22, bold=True,
             align=PP_ALIGN.CENTER, name=f"Video title / {controller} / {condition}")
        text(slide, x, 5.09, width, 1.05, explanations[condition][controller], 18, MUTED,
             align=PP_ALIGN.CENTER, name=f"Video explanation / {controller} / {condition}")
        source_paths.extend((video, poster))
        embedded.append({"controller": controller, "condition": condition, "slide": number,
                         "source": str(video.relative_to(ROOT)),
                         "poster": str(poster.relative_to(ROOT)),
                         "explanation": explanations[condition][controller]})
    bottom = ("纸面线条显示实际 XY 投影；误差按真实三维笔尖位置计算。" if normal else
              "三次推力各持续 2 秒：20–22 秒、55–57 秒、90–92 秒。")
    text(slide, 0.82, 6.56, 11.7, 0.35, bottom, 17, MUTED)
    configure_parallel_video_autoplay(slide._element)
    return embedded


def time_series_chart(slide, x, y, w, h, title, traces, ylabel):
    """Native XY chart; dashed boundaries mark the actual 20–22 s force."""
    text(slide, x, y - 0.42, w, 0.35, title, 19, bold=True, align=PP_ALIGN.CENTER)
    data = XyChartData()
    values = [v for points in traces.values() for _, v in points]
    lo, hi = min(0, min(values)), max(values)
    step = 10 if hi - lo > 30 else 2 if hi - lo > 8 else 1
    lower, upper = math.floor(lo / step) * step, math.ceil(hi / step) * step
    if lower == upper:
        upper += step
    for controller in CONTROL:
        series = data.add_series(SHORT[controller])
        for t, value in traces[controller]:
            series.add_data_point(t, value)
    for boundary in (20, 22):
        series = data.add_series("施力起止")
        for value in (lower, upper):
            series.add_data_point(boundary, value)
    chart = slide.shapes.add_chart(XL_CHART_TYPE.XY_SCATTER_LINES_NO_MARKERS,
                                  Inches(x), Inches(y), Inches(w), Inches(h), data).chart
    chart.has_title = chart.has_legend = False
    chart.chart_style = 2
    for i, series in enumerate(chart.series):
        color = SERIES_COLOR[CONTROL[i]] if i < 4 else TARGET
        series.format.line.color.rgb = rgb(color)
        series.format.line.width = Pt(1.6 if i < 4 else 1.0)
        series.format.line.dash_style = SERIES_DASH[CONTROL[i]] if i < 4 else MSO_LINE_DASH_STYLE.DASH
        series.smooth = False
    xaxis, yaxis = chart.category_axis, chart.value_axis
    for axis in (xaxis, yaxis):
        axis.tick_labels.font.name = FONT
        axis.tick_labels.font.size = Pt(12)
        axis.tick_labels.font.color.rgb = rgb(MUTED)
        axis.major_tick_mark = axis.minor_tick_mark = XL_TICK_MARK.NONE
        axis.format.line.color.rgb = rgb(TARGET)
        axis.format.line.width = Pt(0.7)
        axis.has_title = True
    xaxis.minimum_scale, xaxis.maximum_scale, xaxis.major_unit = 19, 27, 2
    yaxis.minimum_scale, yaxis.maximum_scale, yaxis.major_unit = lower, upper, step
    yaxis.has_major_gridlines = True
    yaxis.major_gridlines.format.line.color.rgb = rgb(GRID)
    yaxis.major_gridlines.format.line.width = Pt(0.45)
    for axis, content in ((xaxis, "时间 / s"), (yaxis, ylabel)):
        axis.axis_title.text_frame.text = content
        for p in axis.axis_title.text_frame.paragraphs:
            for run in p.runs:
                set_font(run, 12, MUTED)
    return chart


def build(destination):
    OUT.mkdir(parents=True, exist_ok=True)
    summary_path = ROOT / "output/summary.csv"
    manifest_path = ROOT / "output/experiment_manifest.json"
    diagnostic_path = OUT / "assignment_diagnostics.json"
    manifest = json.loads(manifest_path.read_text())
    diagnostics = json.loads(diagnostic_path.read_text())
    assert manifest["status"] == "completed", "PPT requires the completed eight-run batch"
    with summary_path.open(newline="") as stream:
        summary = {(r["controller"], r["condition"]): r for r in csv.DictReader(stream)}
    assert len(summary) == 8
    rmse = lambda c, condition: float(summary[c, condition]["drawing_tip_rmse_m"]) * 1000
    normal, forced, recovery = {}, {}, {}
    source_paths = [summary_path, manifest_path, diagnostic_path, ROOT / "6546682.pdf",
                    OUT / "speaker_notes.md", Path(__file__).resolve()]
    for controller in CONTROL:
        for condition, storage in (("normal", normal), ("disturbed", forced)):
            path = ROOT / f"output/{condition}/{controller}/trajectory.csv"
            print(f"Reading {condition} trajectory: {controller}", flush=True)
            storage[controller] = read_strokes(path)
            source_paths.append(path)
        comparison = diagnostics["comparisons"][controller]
        recovery[controller] = [w["recovery_time_s"] for w in comparison["recovery"]["windows"]]
        assert comparison["recovery"]["threshold_m"] == 0.002
        assert comparison["recovery"]["required_hold_s"] == 0.5
    improvement = (1 - rmse("fuzzy", "disturbed") / rmse("gc-pd", "disturbed")) * 100
    reference = diagnostics["reference"]
    ik_residual_mm = reference["recomputed_max_ik_tip_error_m"] * 1000
    joint_jump_deg = reference["max_adjacent_planning_joint_jump_deg"]

    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(W), Inches(H)
    prs.core_properties.title = "机械臂画家 · robot arm artist · raa-v0"
    prs.core_properties.subject = "《机器人智能控制》个人作业1：Isaac Sim 七轴机械臂绘图与抗扰"
    prs.core_properties.author = "肖淇文（2453703）"
    prs.core_properties.keywords = "raa-v0, Isaac Sim, Panda, PD, RBF, 模糊自适应, 可编辑"
    prs.core_properties.comments = "Measured results from the completed eight runs; computed-torque control and model-mismatch experiments are explicitly untested."
    scene = ROOT / "output/video_checks/fuzzy_normal/frame_113_9s.png"
    push_scene = ROOT / "output/video_checks/gc-pd_disturbed/frame_55_5s.png"

    # 1 — Editable identity and an actual simulator image.
    s = base(prs, 1)
    text(s, 0.82, 0.82, 5.7, 0.5, "《机器人智能控制》个人作业 1", 21, MUTED)
    text(s, 0.82, 1.73, 5.6, 0.95, "机械臂画家", 46, bold=True)
    text(s, 0.82, 2.82, 5.5, 0.62, "让七轴机械臂画出 ChatGPT 标志", 24)
    text(s, 0.82, 3.66, 5.4, 0.68, "画得准，也要经得住推力", 27, TEAL)
    text(s, 0.82, 4.75, 5.4, 0.9, "姓名：肖淇文\n学号：2453703", 23)
    text(s, 0.82, 6.05, 5.4, 0.35, "2026 年 10 月 13 日  ·  Isaac Sim", 18, MUTED)
    picture(s, scene, 6.65, 1.72, 6.0, 4.5)

    # 2 — System and environment are merged to leave room for videos.
    s = base(prs, 2, "系统总览与机器人环境", "从目标路线到真实关节力矩，再读回机械臂的运动状态。")
    steps = ["Panda URDF 模型", "笛卡尔轨迹规划", "连续逆运动学（IK）",
             "关节力矩控制器", "PhysX 物理仿真", "关节与笔尖状态"]
    for i, label in enumerate(steps):
        y = 1.82 + i * 0.78
        rect(s, 0.82, y, 3.33, 0.60, PALE, GRID, radius=True)
        text(s, 0.94, y + 0.075, 3.09, 0.43, label, 20, align=PP_ALIGN.CENTER)
        if i < len(steps) - 1:
            text(s, 2.20, y + 0.60, 0.6, 0.19, "↓", 13, TEAL, align=PP_ALIGN.CENTER)
    text(s, 0.82, 6.61, 3.4, 0.30, "关节反馈回到力矩控制器", 15, MUTED)
    table(s, 4.52, 1.82, 4.75, 4.79,
          [["项目", "本次设置"], ["模型 / 基座", "Panda URDF / 固定"],
           ["手臂关节", "panda_joint1–7"], ["末端连杆", "panda_hand"],
           ["虚拟笔尖", "手部局部 Z + 0.13 m"], ["物理 / 控制频率", "1000 Hz"],
           ["重力", "9.81 m/s²"], ["关节力矩限制", "±87 / ±12 N·m"]],
          [0.40, 0.60], 17)
    picture(s, scene, 9.58, 1.86, 2.88, 2.16)
    text(s, 9.58, 4.35, 2.9, 1.82, "Isaac Sim / PhysX\n读取：关节角、速度\n发送：关节力矩 τ", 19)
    text(s, 9.58, 6.19, 2.9, 0.60, "禁用内置位置 / 速度驱动", 17, TEAL)

    # 3 — State exactly which planner and interpolation were actually used.
    s = base(prs, 3, "轨迹规划与连续逆运动学", "先让路线走得平稳，再让七个关节沿同一构型连续运动。")
    outline_plot(s, 0.82, 1.83, 3.53, 4.57, "目标轮廓",
                 [("目标", normal["fuzzy"]["target"], TEAL, None)])
    text(s, 4.84, 1.84, 7.66, 0.43, "① SVG 轮廓 → 平稳的笔尖路线", 23, bold=True)
    text(s, 4.84, 2.39, 7.66, 0.96,
         "8 条轮廓，最大尺寸 16 cm；速度上限 0.04 m/s。\n按累计弧长做五次时间规划；换笔画时抬笔 2.5 cm。", 20)
    text(s, 4.84, 3.60, 7.66, 0.43, "② 上一点关节解 → 下一点 IK 初值", 23, bold=True)
    text(s, 4.84, 4.13, 7.66, 0.90,
         "首点用 Lula IK，后续用热启动阻尼迭代。\n统一手部与虚拟笔尖坐标，减少肘部翻转和固定偏差。", 20)
    text(s, 4.84, 5.21, 7.66, 0.43, "③ 60 Hz 规划 → 1000 Hz 力矩控制", 23, bold=True)
    text(s, 4.84, 5.73, 7.66, 0.79,
         "三次 Hermite 插值得到连续关节角与速度。\n当前未使用加速度前馈；接入前需进一步平滑。", 19)
    text(s, 0.82, 6.64, 11.7, 0.31,
         f"IK 最大位置残差 {ik_residual_mm:.3f} mm   ·   相邻规划点最大关节变化 {joint_jump_deg:.3f}°", 17, TEAL)

    # 4 — Four real controllers, plus the assignment's untested principle.
    s = base(prs, 4, "控制律：从纠偏到模型与学习补偿", "e = q_d − q；de/dt 是速度误差。PD 纠偏，模型和学习提供补偿。")
    table(s, 0.82, 1.83, 11.7, 2.65,
          [["已运行控制器", "关节力矩", "用到的模型 / 学习"],
           ["PD", "τ_PD = K_p e + K_d (de/dt)", "位置与速度误差"],
           ["重力补偿 PD", "τ = τ_PD + G(q)", "物理引擎的重力模型"],
           ["RBF 自适应", "τ = τ_PD + G(q) + Wᵀφ(q, dq/dt)", "每轴 9 个高斯基函数，在线学补偿"],
           ["模糊自适应", "τ = τ_PD + G(q) + Wᵀφ(e, de/dt)", "每轴 25 条规则，在线学补偿"]],
          [0.20, 0.43, 0.37], 18)
    text(s, 0.82, 4.73, 11.7, 0.43,
         "自适应权重从零开始，逐步更新；限制权重与补偿力矩。", 20, TEAL)
    rect(s, 0.82, 5.42, 11.7, 1.26, WHITE, GRID, radius=True)
    text(s, 1.02, 5.54, 11.25, 0.40, "计算力矩：附件中的原理对照（尚未实测）", 21, bold=True)
    text(s, 1.02, 6.03, 11.25, 0.42,
         "τ = M(q) (d²q_d/dt²) + C(q, dq_d/dt) (dq_d/dt) + G(q) + τ_PD", 21, TEAL)

    # 5 — Fair real-force experiment; no made-up 50 N result.
    s = base(prs, 5, "公平实验：在真实笔尖位置施加外力", "同一目标、初始状态、PD 增益与外力；四种控制器各运行正常和受扰两组。")
    text(s, 0.82, 1.84, 6.23, 0.45, "三个推力窗口（世界坐标系）", 23, bold=True)
    table(s, 0.82, 2.45, 6.13, 2.08,
          [["时段 / s", "外力 (Fx, Fy, Fz) / N"],
           ["20–22", "(3, 0, 0)"], ["55–57", "(0, −3, 0)"], ["90–92", "(2, 2, 0)"]],
          [0.33, 0.67], 20)
    text(s, 0.82, 4.86, 6.1, 1.00,
         "关闭内置驱动 → 读状态 → 算并限幅 τ\n发送关节力矩 → 前进 1 ms → 记录状态", 20)
    text(s, 0.82, 6.19, 6.1, 0.60, "记录目标 / 实际轨迹、关节力矩与外力。", 18, MUTED)
    picture(s, push_scene, 7.54, 1.87, 4.83, 3.6225)
    text(s, 7.54, 5.77, 4.83, 0.98,
         "共 8 次运行；Isaac Sim ▶ Play 视口录制。\n本次约 3 N；附件 50 N 场景尚未测试。", 18, MUTED)

    # 6 — Four paired outlines; all metrics remain outside chart areas.
    s = base(prs, 6, "画作与跟踪精度", "相同坐标与尺度；图中显示实际 XY 投影，表中统计落笔阶段的真实三维误差。")
    for i, c in enumerate(CONTROL):
        outline_plot(s, 0.82 + i * 2.97, 1.78, 2.78, 2.72, LABEL[c],
                     [("目标", normal[c]["target"], TARGET, MSO_LINE_DASH_STYLE.DASH),
                      ("正常", normal[c]["actual"], TEAL, None),
                      ("受扰", forced[c]["actual"], ORANGE, None)], small=True)
    legend(s, 4.45, 4.78, [("目标", TARGET, MSO_LINE_DASH_STYLE.DASH),
                           ("正常", TEAL, None), ("受扰", ORANGE, None)], 15, 1.50)
    table(s, 0.82, 5.15, 11.7, 1.59,
          [["控制器", "正常 RMSE / mm", "受扰 RMSE / mm", "受扰峰值误差 / mm"]] +
          [[SHORT[c], f"{rmse(c, 'normal'):.3f}", f"{rmse(c, 'disturbed'):.3f}",
            f"{diagnostics['runs']['disturbed'][c]['peak_tracking_errors']['pen_down']['error_m'] * 1000:.3f}"]
           for c in CONTROL], [0.25, 0.25, 0.25, 0.25], 16, highlight_row=4)

    # 7 — Target tracking, additional deviation and actual torque with windows.
    s = base(prs, 7, "受扰偏移、关节力矩与恢复时间", "三张曲线展示第一次推力；下表比较三次撤力后的恢复，并给出全程受扰偏移峰值。")
    legend(s, 1.0, 1.66,
           [(SHORT[c], SERIES_COLOR[c], SERIES_DASH[c]) for c in CONTROL] +
           [("施力起止", TARGET, MSO_LINE_DASH_STYLE.DASH)], 14, 2.32)
    tracked, deviations, torques = {}, {}, {}
    for c in CONTROL:
        trace = diagnostics["runs"]["disturbed"][c]["chart_traces"]
        paired = diagnostics["comparisons"][c]["chart_traces"]
        tracked[c] = list(zip(trace["time_s"], trace["tracking_error_mm"]))
        deviations[c] = list(zip(paired["time_s"], paired["disturbance_deviation_mm"]))
        torques[c] = list(zip(trace["time_s"], trace["tau4_nm"]))
    for x, title, traces, ylabel in (
        (0.82, "笔尖跟踪误差", tracked, "误差 / mm"),
        (4.80, "额外受扰偏移", deviations, "偏移 / mm"),
        (8.78, "关节 4 力矩", torques, "力矩 / N·m")):
        time_series_chart(s, x, 2.39, 3.74, 2.35, title, traces, ylabel)
    table(s, 0.82, 5.02, 11.7, 1.60,
          [["控制器", "全程偏移峰值 / mm", "第 1 次恢复 / s", "第 2 次恢复 / s", "第 3 次恢复 / s"]] +
          [[SHORT[c], f"{diagnostics['comparisons'][c]['full_peak_paired_deviation']['error_m'] * 1000:.3f}"] +
           [f"{v:.3f}" for v in recovery[c]] for c in CONTROL],
          [0.20, 0.26, 0.18, 0.18, 0.18], 15, highlight_row=2)
    text(s, 0.82, 6.67, 11.7, 0.30,
         "额外偏移 = 受扰位置 − 正常位置；恢复：三维距离 ≤ 2 mm 且连续保持 0.5 s。", 16, MUTED)

    # 8–9 — Preserve complete embedded originals and four parallel starts.
    embedded_videos = video_slide(prs, 8, "normal", source_paths)
    embedded_videos += video_slide(prs, 9, "disturbed", source_paths)

    # 10 — Answer the assignment's questions without claiming unrun results.
    s = base(prs, 10, "三个问题：差别、抗扰与实际选型")
    takeaways = [
        ("01", "为什么 PD 会把图案画歪？",
         "没有重力补偿，需要持续偏差来托住自重；正常 RMSE 约 33 mm。\n加上 G(q) 后降至约 0.56 mm，整体偏移显著减少。"),
        ("02", "受推时谁偏得多，撤力后谁恢复快？",
         "额外受扰偏移：施力时重力补偿 PD 最大 12.53 mm；全程 PD 最大 19.68 mm。\n重力补偿 PD 恢复最快；模糊的受扰整体 RMSE 最小（1.77 mm）。"),
        ("03", "从二轴到七轴，实际使用怎么选？",
         "新增冗余构型、关节限位、笔尖坐标与固定控制频率问题。\n本次慢速绘图优先重力补偿 PD；高速可对照计算力矩，模型不准可验证自适应补偿。"),
    ]
    for i, (number, title, explanation) in enumerate(takeaways):
        y = 1.69 + i * 1.49
        text(s, 0.82, y + 0.01, 0.80, 0.55, number, 30, TEAL, bold=True)
        text(s, 1.88, y, 10.64, 0.46, title, 24, bold=True)
        text(s, 1.88, y + 0.56, 10.64, 0.84, explanation, 18, MUTED)
        if i < 2:
            line(s, 1.88, y + 1.36, 12.48, y + 1.36, GRID, 0.7)
    text(s, 0.82, 6.45, 11.7, 0.41,
         "计算力矩与模型失配尚未实测；慢速时惯性项较小，预期与重力补偿接近，待后续验证。", 17, TEAL)

    notes = load_notes()
    assert set(notes) == set(range(1, 11)), "All ten slides require assignment-aligned speaker notes"
    for i, slide in enumerate(prs.slides, 1):
        slide.notes_slide.notes_text_frame.text = notes[i] + "\n\n编辑说明：文字、表格、流程图、轨迹线及时间曲线均为原生 PowerPoint 对象。时间曲线可通过“编辑数据”修改；轨迹线可通过“编辑顶点”修改。真实仿真截图为图片。第8、9页的八个视频完整内嵌，视频下方说明为可编辑文本框。"
    assert len(prs.slides) == TOTAL_SLIDES
    for slide in prs.slides:
        for shape in slide.shapes:
            for effect_ref in shape._element.xpath(".//a:effectRef"):
                effect_ref.set("idx", "0")
            for props in shape._element.xpath("./p:spPr"):
                if props.find("a:effectLst", props.nsmap) is None:
                    props.append(OxmlElement("a:effectLst"))
    for i, slide in enumerate(prs.slides, 1):
        for shape in slide.shapes:
            assert shape.left >= -1000 and shape.top >= -1000, (i, shape.name)
            assert shape.left + shape.width <= prs.slide_width + 1000, (i, shape.name, "right edge")
            assert shape.top + shape.height <= prs.slide_height + 1000, (i, shape.name, "bottom edge")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.stem}.building.pptx")
    try:
        prs.save(temporary)
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    source_hashes = {}
    for p in source_paths + [scene, push_scene]:
        with p.open("rb") as stream:
            source_hashes[str(p.relative_to(ROOT))] = hashlib.file_digest(stream, "sha256").hexdigest()
    audit = {
        "presentation": str(destination.relative_to(ROOT)) if destination.is_relative_to(ROOT) else str(destination),
        "assignment": "6546682.pdf", "student": {"name": "肖淇文", "id": "2453703"},
        "date": "2026-10-13", "slide_count": TOTAL_SLIDES, "slide_size": "16:9", "font": FONT,
        "metric": "pen-down 3D position RMSE, mm",
        "normal_rmse_mm": {c: rmse(c, "normal") for c in CONTROL},
        "disturbed_rmse_mm": {c: rmse(c, "disturbed") for c in CONTROL},
        "disturbed_drawing_peak_tracking_error_mm": {
            c: diagnostics["runs"]["disturbed"][c]["peak_tracking_errors"]["pen_down"]["error_m"] * 1000 for c in CONTROL},
        "full_peak_paired_deviation_mm": {
            c: diagnostics["comparisons"][c]["full_peak_paired_deviation"]["error_m"] * 1000 for c in CONTROL},
        "recovery_s": recovery,
        "fuzzy_vs_gc_pd_disturbed_rmse_reduction_percent": improvement,
        "max_ik_tip_error_mm": ik_residual_mm, "max_adjacent_planning_joint_jump_deg": joint_jump_deg,
        "recovery_definition": diagnostics["definitions"]["recovery"],
        "outline_display": "10 ms displayed samples, stroke endpoints preserved, RDP tolerance 0.05 mm; all reported metrics use complete 1 ms data",
        "time_series_display": "Three native charts; 19 <= t < 27 s; 10 ms display; native dashed force boundaries at 20 and 22 s",
        "native_editable": ["text", "tables", "vertical flowchart", "trajectory freeform paths", "three XY charts with embedded Excel workbooks"],
        "raster_assets": [str(p.relative_to(ROOT)) for p in (scene, push_scene)],
        "embedded_videos": embedded_videos,
        "video_delivery": "8 complete internally embedded MP4 files; four videos start automatically in parallel on each video slide; no external video links",
        "video_playback": {"trigger": "slide entry", "start_delay_ms": 0, "mode": "parallel", "videos_per_slide": 4, "slides": [8, 9]},
        "pdf_preview": "Static poster frames only; embedded videos play in the PPTX slideshow",
        "scope": {"measured_controllers": list(CONTROL), "platform": "Isaac Sim / PhysX",
                  "force_n": [[3, 0, 0], [0, -3, 0], [2, 2, 0]],
                  "computed_torque": "principle only, not implemented or measured",
                  "model_mismatch": "future experiment, not measured", "50N_example": "not tested",
                  "qdd_feedforward": "not used; current Hermite interpolation is C1, not C2"},
        "source_sha256": source_hashes, "notes_from_markdown": sorted(notes),
        "shape_counts": [len(s.shapes) for s in prs.slides],
    }
    (OUT / "presentation_sources.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(f"Saved editable presentation: {destination}", flush=True)
    print("Ten slides; eight complete embedded videos; three editable charts; ten assignment-aligned notes.", flush=True)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUT / "raa-v0.pptx")
    parser.add_argument("--sync-videos", action="store_true",
                        help="Update only the existing PPTX: four videos automatically start together on each video slide")
    args = parser.parse_args()
    destination = args.output.expanduser().resolve()
    if args.sync_videos:
        synchronize_existing_presentation(destination)
    else:
        build(destination)


if __name__ == "__main__":
    main()
