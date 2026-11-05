#!/usr/bin/env python3
"""Render the concise, report-ready framework diagrams used by the L4 report."""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw

from render_project_diagrams import (
    COLORS,
    arrow,
    font,
    page_title,
    poly_arrow,
)


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output/L4/figures"
WIDTH = 1900


def canvas(height: int, title: str):
    image = Image.new("RGB", (WIDTH, height), COLORS["white"])
    draw = ImageDraw.Draw(image)
    page_title(draw, title, "", WIDTH)
    return image, draw


def save(image: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True, dpi=(300, 300))


def module_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    title: str,
    fill: str,
    edge: str,
    *,
    title_size: int = 58,
) -> None:
    """Draw one first-level module using only its short name."""
    draw.rounded_rectangle(
        xy,
        radius=30,
        fill=COLORS[fill],
        outline=COLORS[edge],
        width=5,
    )
    left, top, right, bottom = xy
    draw.text(
        ((left + right) / 2, (top + bottom) / 2),
        title,
        font=font(title_size, bold=True),
        fill=COLORS["ink"],
        anchor="mm",
    )


def render_architecture(path: Path) -> None:
    image, draw = canvas(
        2440,
        "L4 Isaac Sim 适配总体架构",
    )
    nodes = [
        ((300, 300, 1600, 570), "实验任务",
         "blue_fill", "blue_edge"),
        ((300, 720, 1600, 990), "Python 适配层",
         "green_fill", "green_edge"),
        ((300, 1140, 1600, 1410), "OpenLoong 控制器",
         "purple_fill", "purple_edge"),
        ((300, 1560, 1600, 1830), "Isaac Sim / PhysX",
         "gray_fill", "gray_edge"),
        ((300, 1980, 1600, 2250), "实验输出",
         "red_fill", "red_edge"),
    ]
    for xy, title, fill, edge in nodes:
        module_box(draw, xy, title, fill, edge)
    for first, second in zip(nodes, nodes[1:]):
        arrow(draw, ((first[0][0] + first[0][2]) // 2, first[0][3]),
              ((second[0][0] + second[0][2]) // 2, second[0][1]))
    poly_arrow(
        draw,
        [(300, 1695), (130, 1695), (130, 855), (300, 855)],
        color=COLORS["feedback"],
        width=7,
    )
    save(image, path)


def render_path_loop(path: Path) -> None:
    image, draw = canvas(
        2440,
        "L4 路径跟踪闭环",
    )
    nodes = [
        ((300, 300, 1600, 570), "参考路径",
         "blue_fill", "blue_edge"),
        ((300, 720, 1600, 990), "路径跟踪器",
         "green_fill", "green_edge"),
        ((300, 1140, 1600, 1410), "速度指令",
         "orange_fill", "orange_edge"),
        ((300, 1560, 1600, 1830), "全身控制器",
         "purple_fill", "purple_edge"),
        ((300, 1980, 1600, 2250), "Isaac Sim / PhysX",
         "gray_fill", "gray_edge"),
    ]
    for xy, title, fill, edge in nodes:
        module_box(draw, xy, title, fill, edge)
    for first, second in zip(nodes, nodes[1:]):
        arrow(draw, (950, first[0][3]), (950, second[0][1]))
    poly_arrow(
        draw,
        [(300, 2115), (130, 2115), (130, 855), (300, 855)],
        color=COLORS["feedback"],
        width=7,
    )
    save(image, path)


def render_mpc_comparison(path: Path) -> None:
    image, draw = canvas(
        2740,
        "L4 Convex MPC 与 NMPC 对照链路",
    )
    shared = (300, 300, 1600, 570)
    convex = (120, 800, 880, 1100)
    nmpc = (1020, 800, 1780, 1100)
    merge = (300, 1330, 1600, 1600)
    wbc = (300, 1780, 1600, 2050)
    physx = (300, 2230, 1600, 2500)
    module_box(draw, shared, "共同输入", "blue_fill", "blue_edge")
    module_box(draw, convex, "Convex MPC", "purple_fill", "purple_edge", title_size=52)
    module_box(draw, nmpc, "NMPC", "green_fill", "green_edge", title_size=52)
    module_box(draw, merge, "足底力输出", "orange_fill", "orange_edge")
    module_box(draw, wbc, "全身与关节控制", "gray_fill", "gray_edge")
    module_box(draw, physx, "闭环评价", "red_fill", "red_edge")
    draw.line((950, shared[3], 950, 745), fill=COLORS["line"], width=7)
    draw.line((505, 745, 1395, 745), fill=COLORS["line"], width=7)
    arrow(draw, (505, 745), (505, convex[1]))
    arrow(draw, (1395, 745), (1395, nmpc[1]))
    arrow(draw, (505, convex[3]), (505, 1210), color=COLORS["purple_edge"])
    arrow(draw, (1395, nmpc[3]), (1395, 1210), color=COLORS["green_edge"])
    draw.line((505, 1210, 1395, 1210), fill=COLORS["line"], width=7)
    arrow(draw, (950, 1210), (950, merge[1]))
    arrow(draw, (950, merge[3]), (950, wbc[1]))
    arrow(draw, (950, wbc[3]), (950, physx[1]))
    save(image, path)


def render_torque_decomposition(path: Path) -> None:
    image, draw = canvas(
        2740,
        "L4 足底力与关节力矩分解",
    )
    shared = (300, 300, 1600, 570)
    feedforward = (120, 800, 880, 1100)
    feedback = (1020, 800, 1780, 1100)
    merge = (300, 1330, 1600, 1600)
    actuator = (300, 1780, 1600, 2050)
    observation = (300, 2230, 1600, 2500)
    module_box(draw, shared, "状态与目标", "blue_fill", "blue_edge")
    module_box(draw, feedforward, "MPC + WBC", "purple_fill", "purple_edge", title_size=52)
    module_box(draw, feedback, "关节 PD", "green_fill", "green_edge", title_size=52)
    module_box(draw, merge, "力矩合成", "orange_fill", "orange_edge")
    module_box(draw, actuator, "限幅与执行", "red_fill", "red_edge")
    module_box(draw, observation, "Isaac 观测", "gray_fill", "gray_edge")
    draw.line((950, shared[3], 950, 745), fill=COLORS["line"], width=7)
    draw.line((505, 745, 1395, 745), fill=COLORS["line"], width=7)
    arrow(draw, (505, 745), (505, feedforward[1]))
    arrow(draw, (1395, 745), (1395, feedback[1]))
    arrow(draw, (505, feedforward[3]), (505, 1210), color=COLORS["purple_edge"])
    arrow(draw, (1395, feedback[3]), (1395, 1210), color=COLORS["green_edge"])
    draw.line((505, 1210, 1395, 1210), fill=COLORS["line"], width=7)
    arrow(draw, (950, 1210), (950, merge[1]))
    arrow(draw, (950, merge[3]), (950, actuator[1]))
    arrow(draw, (950, actuator[3]), (950, observation[1]))
    save(image, path)


def main() -> None:
    outputs = [
        (render_architecture, OUTPUT / "L4总体适配架构.png"),
        (render_path_loop, OUTPUT / "L4路径跟踪闭环.png"),
        (render_mpc_comparison, OUTPUT / "L4凸MPC与NMPC对比.png"),
        (render_torque_decomposition, OUTPUT / "L4力矩分解链路.png"),
    ]
    for renderer, path in outputs:
        renderer(path)
        print(path.relative_to(ROOT.parent))


if __name__ == "__main__":
    main()
