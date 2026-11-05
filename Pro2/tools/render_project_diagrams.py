#!/usr/bin/env python3
"""Render the L1, L2, and L3 project diagrams as PNG files."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
REGULAR_FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
BOLD_FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")

COLORS = {
    "ink": "#172033",
    "muted": "#536173",
    "line": "#344054",
    "feedback": "#137A78",
    "blue_fill": "#E7F2FF",
    "blue_edge": "#2F6B9A",
    "orange_fill": "#FFF2DC",
    "orange_edge": "#B96316",
    "purple_fill": "#EEE8FA",
    "purple_edge": "#6F4AA8",
    "green_fill": "#E4F4E7",
    "green_edge": "#2E7D4F",
    "gray_fill": "#F0F3F6",
    "gray_edge": "#65717E",
    "red_fill": "#FFF0F0",
    "red_edge": "#B42318",
    "white": "#FFFFFF",
}


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = BOLD_FONT if bold else REGULAR_FONT
    if not path.is_file():
        raise FileNotFoundError(f"缺少中文字体：{path}")
    return ImageFont.truetype(str(path), size=size)


def text_width(draw: ImageDraw.ImageDraw, value: str, text_font: ImageFont.FreeTypeFont) -> float:
    return draw.textlength(value, font=text_font)


def wrap_text(
    draw: ImageDraw.ImageDraw,
    value: str,
    text_font: ImageFont.FreeTypeFont,
    max_width: int,
) -> list[str]:
    """Wrap Chinese/Latin text by measured glyph width while preserving newlines."""
    result: list[str] = []
    for paragraph in value.splitlines() or [""]:
        if not paragraph:
            result.append("")
            continue
        current = ""
        for token in paragraph.split(" "):
            candidate = token if not current else f"{current} {token}"
            if text_width(draw, candidate, text_font) <= max_width:
                current = candidate
                continue
            if current:
                result.append(current)
                current = ""
            if text_width(draw, token, text_font) <= max_width:
                current = token
                continue
            piece = ""
            for char in token:
                candidate_piece = piece + char
                if piece and text_width(draw, candidate_piece, text_font) > max_width:
                    result.append(piece)
                    piece = char
                else:
                    piece = candidate_piece
            current = piece
        if current:
            result.append(current)
    return result


def draw_centered_lines(
    draw: ImageDraw.ImageDraw,
    lines: list[str],
    box: tuple[int, int, int, int],
    text_font: ImageFont.FreeTypeFont,
    fill: str,
    spacing: int = 8,
) -> None:
    left, top, right, bottom = box
    bbox = text_font.getbbox("国Ag")
    line_height = bbox[3] - bbox[1]
    total_height = len(lines) * line_height + max(0, len(lines) - 1) * spacing
    y = top + (bottom - top - total_height) / 2
    for line in lines:
        width = text_width(draw, line, text_font)
        draw.text(((left + right - width) / 2, y), line, font=text_font, fill=fill)
        y += line_height + spacing


def box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    title: str,
    body: str,
    *,
    fill: str,
    edge: str,
    title_size: int = 43,
    body_size: int = 29,
    radius: int = 28,
    width: int = 5,
) -> None:
    left, top, right, bottom = xy
    draw.rounded_rectangle(xy, radius=radius, fill=fill, outline=edge, width=width)
    title_font = font(title_size, bold=True)
    body_font = font(body_size)
    title_y = top + 24
    title_w = text_width(draw, title, title_font)
    draw.text(((left + right - title_w) / 2, title_y), title, font=title_font, fill=COLORS["ink"])
    rule_y = title_y + title_size + 15
    draw.line((left + 32, rule_y, right - 32, rule_y), fill=edge, width=2)
    lines = wrap_text(draw, body, body_font, right - left - 70)
    draw_centered_lines(draw, lines, (left + 30, rule_y + 12, right - 30, bottom - 16), body_font, COLORS["muted"], 7)


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    *,
    color: str | None = None,
    width: int = 7,
    head: int = 20,
    dashed: bool = False,
) -> None:
    color = color or COLORS["line"]
    x1, y1 = start
    x2, y2 = end
    if dashed:
        length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        if length == 0:
            return
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        position = 0.0
        while position < length - head:
            segment_end = min(position + 20, length - head)
            draw.line(
                (x1 + ux * position, y1 + uy * position, x1 + ux * segment_end, y1 + uy * segment_end),
                fill=color,
                width=width,
            )
            position += 34
    else:
        draw.line((x1, y1, x2, y2), fill=color, width=width)
    import math

    angle = math.atan2(y2 - y1, x2 - x1)
    points = [
        (x2, y2),
        (x2 - head * math.cos(angle - 0.55), y2 - head * math.sin(angle - 0.55)),
        (x2 - head * math.cos(angle + 0.55), y2 - head * math.sin(angle + 0.55)),
    ]
    draw.polygon(points, fill=color)


def poly_arrow(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[int, int]],
    *,
    color: str,
    width: int = 7,
    head: int = 20,
    dashed: bool = False,
) -> None:
    for first, second in zip(points[:-2], points[1:-1]):
        if dashed:
            arrow(draw, first, second, color=color, width=width, head=0, dashed=True)
        else:
            draw.line((*first, *second), fill=color, width=width)
    arrow(draw, points[-2], points[-1], color=color, width=width, head=head, dashed=dashed)


def label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    *,
    size: int = 25,
    fill: str | None = None,
    anchor: str = "mm",
    background: bool = True,
) -> None:
    text_font = font(size)
    fill = fill or COLORS["muted"]
    bounds = draw.textbbox(xy, value, font=text_font, anchor=anchor)
    if background:
        draw.rounded_rectangle(
            (bounds[0] - 10, bounds[1] - 4, bounds[2] + 10, bounds[3] + 5),
            radius=8,
            fill=COLORS["white"],
        )
    draw.text(xy, value, font=text_font, fill=fill, anchor=anchor)


def page_title(draw: ImageDraw.ImageDraw, title: str, subtitle: str, width: int) -> None:
    title_font = font(65, bold=True)
    subtitle_font = font(30)
    title_w = text_width(draw, title, title_font)
    subtitle_w = text_width(draw, subtitle, subtitle_font)
    draw.text(((width - title_w) / 2, 55), title, font=title_font, fill=COLORS["ink"])
    draw.text(((width - subtitle_w) / 2, 145), subtitle, font=subtitle_font, fill=COLORS["muted"])


def render_l1(path: Path) -> None:
    width, height = 2100, 2700
    image = Image.new("RGB", (width, height), COLORS["white"])
    draw = ImageDraw.Draw(image)

    title_font = font(72, bold=True)
    draw.text(
        (width // 2, 90),
        "人形机器人闭环控制框架",
        font=title_font,
        fill=COLORS["ink"],
        anchor="mm",
    )

    def module(
        xy: tuple[int, int, int, int],
        title: str,
        *,
        fill: str,
        edge: str,
        title_size: int = 56,
    ) -> None:
        draw.rounded_rectangle(xy, radius=30, fill=fill, outline=edge, width=6)
        center_x = (xy[0] + xy[2]) // 2
        center_y = (xy[1] + xy[3]) // 2
        draw.text(
            (center_x, center_y),
            title,
            font=font(title_size, bold=True),
            fill=COLORS["ink"],
            anchor="mm",
        )

    command = (180, 250, 930, 475)
    state = (1170, 250, 1920, 475)
    planning = (420, 650, 1680, 900)
    mpc = (420, 1060, 1680, 1310)
    wbc = (420, 1470, 1680, 1720)
    joint = (420, 1880, 1680, 2130)
    isaac = (420, 2290, 1680, 2540)
    logging = (1740, 2310, 2050, 2520)

    module(
        command,
        "运动命令",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
    )
    module(
        state,
        "状态与动力学",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
        title_size=52,
    )
    module(
        planning,
        "参考与步态规划",
        fill=COLORS["orange_fill"],
        edge=COLORS["orange_edge"],
    )
    module(
        mpc,
        "MPC",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
    )
    module(
        wbc,
        "WBC",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
    )
    module(
        joint,
        "关节控制",
        fill=COLORS["green_fill"],
        edge=COLORS["green_edge"],
    )
    module(
        isaac,
        "Isaac Sim / PhysX",
        fill=COLORS["green_fill"],
        edge=COLORS["green_edge"],
    )
    module(
        logging,
        "数据记录",
        fill=COLORS["gray_fill"],
        edge=COLORS["gray_edge"],
        title_size=42,
    )

    poly_arrow(draw, [(555, 475), (555, 560), (830, 560), (830, 650)], color=COLORS["line"])
    poly_arrow(draw, [(1545, 475), (1545, 560), (1270, 560), (1270, 650)], color=COLORS["line"])
    arrow(draw, (1050, 900), (1050, 1060))
    arrow(draw, (1050, 1310), (1050, 1470))
    arrow(draw, (1050, 1720), (1050, 1880))
    arrow(draw, (1050, 2130), (1050, 2290))

    # The single outer loop makes the simulated-state feedback explicit.
    poly_arrow(
        draw,
        [(1050, 2540), (1050, 2620), (2070, 2620), (2070, 190), (1545, 190), (1545, 250)],
        color=COLORS["feedback"],
        width=9,
        head=24,
    )
    label(draw, (1810, 190), "状态反馈", size=29, fill=COLORS["feedback"])

    poly_arrow(
        draw,
        [(1680, 2415), (1710, 2415), (1740, 2415)],
        color=COLORS["gray_edge"],
        width=5,
        head=18,
        dashed=True,
    )

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True, dpi=(300, 300))


def render_l2(path: Path) -> None:
    width, height = 2100, 2850
    image = Image.new("RGB", (width, height), COLORS["white"])
    draw = ImageDraw.Draw(image)

    title_font = font(72, bold=True)
    draw.text(
        (width // 2, 90),
        "L2 单变量对照实验框架",
        font=title_font,
        fill=COLORS["ink"],
        anchor="mm",
    )

    def module(
        xy: tuple[int, int, int, int],
        title: str,
        *,
        fill: str,
        edge: str,
        title_size: int = 54,
    ) -> None:
        draw.rounded_rectangle(xy, radius=30, fill=fill, outline=edge, width=6)
        center_x = (xy[0] + xy[2]) // 2
        center_y = (xy[1] + xy[3]) // 2
        draw.text(
            (center_x, center_y),
            title,
            font=font(title_size, bold=True),
            fill=COLORS["ink"],
            anchor="mm",
        )

    fixed = (300, 260, 1800, 500)
    group = (100, 680, 2000, 1510)
    weight = (160, 830, 990, 1120)
    horizon = (1110, 830, 1940, 1120)
    step_height = (160, 1180, 990, 1470)
    push = (1110, 1180, 1940, 1470)
    saved = (300, 1690, 1800, 1930)
    metric = (300, 2110, 1800, 2350)
    compare = (300, 2530, 1800, 2770)

    module(
        fixed,
        "统一实验条件",
        fill=COLORS["gray_fill"],
        edge=COLORS["gray_edge"],
    )

    draw.rounded_rectangle(group, radius=34, fill="#FFFDF8", outline=COLORS["orange_edge"], width=6)
    draw.text(
        (1050, 750),
        "四类独立对照实验",
        font=font(50, bold=True),
        fill=COLORS["ink"],
        anchor="mm",
    )
    module(
        weight,
        "MPC 代价权重",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
        title_size=44,
    )
    module(
        horizon,
        "预测时域",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
        title_size=44,
    )
    module(
        step_height,
        "抬脚高度",
        fill=COLORS["orange_fill"],
        edge=COLORS["orange_edge"],
        title_size=44,
    )
    module(
        push,
        "横向扰动",
        fill=COLORS["green_fill"],
        edge=COLORS["green_edge"],
        title_size=44,
    )
    module(
        saved,
        "独立保存实验数据",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
    )
    module(
        metric,
        "统一指标分析",
        fill=COLORS["gray_fill"],
        edge=COLORS["gray_edge"],
    )
    module(
        compare,
        "曲线对比与结论",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
    )

    arrow(draw, (1050, 500), (1050, 680))
    arrow(draw, (1050, 1510), (1050, 1690))
    arrow(draw, (1050, 1930), (1050, 2110))
    arrow(draw, (1050, 2350), (1050, 2530))

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True, dpi=(300, 300))


def render_l3(path: Path) -> None:
    """Render a compact optimal-control method map for the L3 report."""
    width, height = 2300, 2800
    image = Image.new("RGB", (width, height), COLORS["white"])
    draw = ImageDraw.Draw(image)
    page_title(
        draw,
        "L3 最优控制方法关系与 OpenLoong 定位",
        "",
        width,
    )

    def method_node(
        xy: tuple[int, int, int, int],
        title: str,
        *,
        fill: str,
        edge: str,
        title_size: int = 51,
        edge_width: int = 6,
    ) -> None:
        left, top, right, bottom = xy
        draw.rounded_rectangle(
            xy,
            radius=34,
            fill=fill,
            outline=edge,
            width=edge_width,
        )
        title_font = font(title_size, bold=True)
        draw.text(
            ((left + right) / 2, (top + bottom) / 2),
            title,
            font=title_font,
            fill=COLORS["ink"],
            anchor="mm",
        )

    optimal = (650, 285, 1650, 555)
    lqr = (100, 910, 720, 1190)
    ilqr = (840, 910, 1460, 1190)
    mpc = (1580, 910, 2200, 1190)
    convex_mpc = (1190, 1630, 1770, 1910)
    nmpc = (1840, 1630, 2220, 1910)
    current = (1120, 2300, 1840, 2600)

    method_node(
        optimal,
        "最优控制",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
        title_size=60,
    )
    method_node(
        lqr,
        "LQR",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
    )
    method_node(
        ilqr,
        "iLQR",
        fill=COLORS["orange_fill"],
        edge=COLORS["orange_edge"],
    )
    method_node(
        mpc,
        "MPC",
        fill=COLORS["blue_fill"],
        edge=COLORS["blue_edge"],
    )
    method_node(
        convex_mpc,
        "Convex MPC",
        fill=COLORS["purple_fill"],
        edge=COLORS["purple_edge"],
        title_size=44,
    )
    method_node(
        nmpc,
        "NMPC",
        fill=COLORS["green_fill"],
        edge=COLORS["green_edge"],
        title_size=46,
    )
    method_node(
        current,
        "OpenLoong",
        fill=COLORS["red_fill"],
        edge=COLORS["red_edge"],
        title_size=54,
        edge_width=8,
    )

    # One top-level family, one MPC split, and one highlighted project location.
    arrow(draw, (1150, 555), (1150, 710), width=8, head=24)
    draw.line((410, 710, 1890, 710), fill=COLORS["line"], width=8)
    arrow(draw, (410, 710), (410, 910), width=8, head=24)
    arrow(draw, (1150, 710), (1150, 910), width=8, head=24)
    arrow(draw, (1890, 710), (1890, 910), width=8, head=24)

    arrow(draw, (1890, 1190), (1890, 1390), color=COLORS["blue_edge"], width=8, head=24)
    draw.line((1480, 1390, 2030, 1390), fill=COLORS["blue_edge"], width=8)
    arrow(draw, (1480, 1390), (1480, 1630), color=COLORS["blue_edge"], width=8, head=24)
    arrow(draw, (2030, 1390), (2030, 1630), color=COLORS["blue_edge"], width=8, head=24)

    poly_arrow(
        draw,
        [(1150, 1190), (1150, 1480), (1950, 1480), (1950, 1630)],
        color=COLORS["orange_edge"],
        width=6,
        head=22,
        dashed=True,
    )

    arrow(
        draw,
        (1480, 1910),
        (1480, 2300),
        color=COLORS["red_edge"],
        width=9,
        head=27,
    )

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True, dpi=(300, 300))


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--diagram",
        choices=("all", "L1", "L2", "L3"),
        default="all",
        help="选择要重新生成的图片",
    )
    return parser.parse_args()


def main() -> None:
    args = arguments()
    outputs: list[Path] = []
    if args.diagram in ("all", "L1"):
        path = ROOT / "output/L1/figures/L1闭环控制框架.png"
        render_l1(path)
        outputs.append(path)
    if args.diagram in ("all", "L2"):
        path = ROOT / "output/L2/figures/L2实验框架图.png"
        render_l2(path)
        outputs.append(path)
    if args.diagram in ("all", "L3"):
        path = ROOT / "output/L3/figures/L3理论关系图.png"
        render_l3(path)
        outputs.append(path)
    for path in outputs:
        print(path.relative_to(ROOT.parent))


if __name__ == "__main__":
    main()
