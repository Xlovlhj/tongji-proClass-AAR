#!/usr/bin/env python3
"""统一启动 Pro2 各层任务的可视化演示。

所有命令都从项目根目录执行，仿真结果按 L0--L4 的既有目录规则保存。
L1、L3 是分析任务，因此生成并打开相应框架图；其余任务启动 Isaac Sim GUI。
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import webbrowser
from datetime import datetime
from html import escape
from pathlib import Path
from typing import Sequence


PRO2_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PRO2_ROOT.parent

TASKS = {
    "l0": "L0 基线行走；约第 5 秒可按 A/D 观察转向",
    "l1": "L1 控制闭环框架图（静态分析任务）",
    "l2-weight": "L2 代价权重低/基线/高三组 GUI 对照",
    "l2-horizon": "L2 预测时域 N=5/10/15 三组 GUI 对照",
    "l2-step": "L2 抬腿高度 0.08/0.12/0.16 m 三组 GUI 对照",
    "l2-push": "L2 无侧推/50 N/100 N 三组 GUI 对照",
    "l3": "L3 最优控制方法关系图（静态理论任务）",
    "l4-robot": "L4-1 H1-2 模型更换与 FixStand 站立",
    "l4-path": "L4-2 AzureLoong 多路径点闭环跟踪（地面显示参考线与路径点）",
    "l4-terrain": "L4-3 AzureLoong 随机障碍区域穿越",
    "l4-jump": "L4-3 AzureLoong 原地竖直跳跃与落地恢复",
    "l4-mpc": "L4-4 Convex MPC 与 NMPC 两次 GUI 运行及曲线对照",
    "l4-decomposition": "L4-5 MPC/WBC/PD/Isaac 力矩分解",
    "l4-all": "依次执行 L4 五项（地形项同时演示跳跃）",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="用一个 Python 入口查看 Pro2 各项任务的动态或静态效果。"
    )
    parser.add_argument("--list", action="store_true", help="列出可视化任务后退出")
    parser.add_argument(
        "task_name",
        nargs="?",
        choices=TASKS,
        help="要查看的任务；也可使用 --task",
    )
    parser.add_argument("--task", choices=TASKS, help="要查看的任务")
    parser.add_argument(
        "--isaac-root",
        default="/home/xqw/isaacsim6.0.1",
        help="Isaac Sim 安装目录",
    )
    parser.add_argument("--render-hz", type=float, default=30.0, help="GUI 刷新率")
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="对 run_isaac.py 动态任务同时生成 demo.mp4；H1 入口暂不支持录像",
    )
    parser.add_argument(
        "--tag",
        help="在自动时间戳后追加的输出目录标记；只能包含字母、数字、横线或下划线",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="只打印将执行的命令，不启动程序"
    )
    parser.add_argument(
        "--no-open", action="store_true", help="生成静态图片后不自动用浏览器打开"
    )
    args = parser.parse_args()
    if args.task_name and args.task and args.task_name != args.task:
        parser.error("位置任务与 --task 不能指定两个不同的任务")
    args.task = args.task or args.task_name
    if not args.list and not args.task:
        parser.error("请指定任务，或先使用 --list 查看任务")
    if args.tag and re.fullmatch(r"[A-Za-z0-9_-]+", args.tag) is None:
        parser.error("--tag 只能包含字母、数字、横线和下划线")
    return args


def relative(path: Path) -> str:
    return str(path.relative_to(PROJECT_ROOT))


def run(command: Sequence[str], *, dry_run: bool) -> None:
    printable = subprocess.list2cmdline([str(item) for item in command])
    print(f"\n$ {printable}", flush=True)
    if not dry_run:
        subprocess.run(
            [str(item) for item in command], cwd=PROJECT_ROOT, check=True
        )


def output_path(*parts: str) -> Path:
    return PROJECT_ROOT.joinpath("Pro2", "output", *parts)


def isaac_command(
    args: argparse.Namespace,
    *,
    level: str,
    output: Path,
    config: Path | None = None,
    extra: Sequence[str] = (),
) -> list[str]:
    command = [
        sys.executable,
        relative(PRO2_ROOT / "run_isaac.py"),
        "--isaac-root",
        args.isaac_root,
        "--level",
        level,
        "--no-build",
        "--render-hz",
        str(args.render_hz),
    ]
    if config is not None:
        command.extend(["--config", relative(config)])
    command.extend([str(item) for item in extra])
    if args.record_video:
        command.extend(["--record-video", "--video-fps", "30"])
    command.extend(["--output", relative(output)])
    return command


def show_static_diagram(args: argparse.Namespace, level: str) -> None:
    diagram = level.upper()
    image_name = {
        "L1": "L1闭环控制框架.png",
        "L3": "L3理论关系图.png",
    }[diagram]
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "tools" / "render_project_diagrams.py"),
            "--diagram",
            diagram,
        ],
        dry_run=args.dry_run,
    )
    image_path = output_path(diagram, "figures", image_name)
    print(f"图片：{relative(image_path)}")
    if not args.dry_run and not args.no_open:
        webbrowser.open(image_path.as_uri())


def open_figure_gallery(
    args: argparse.Namespace, directory: Path, title: str
) -> None:
    """Create a small local gallery for analysis figures and open it."""
    print(f"分析图片：{relative(directory)}/")
    if args.dry_run or args.no_open:
        return
    images = sorted(directory.glob("*.png"))
    if not images:
        print("该目录没有生成 PNG，跳过自动打开。")
        return
    cards = []
    for image in images:
        label = escape(image.stem.replace("_", " "))
        source = escape(image.name)
        cards.append(
            f'<section><h2>{label}</h2><img src="{source}" alt="{label}"></section>'
        )
    gallery = directory / "visual_gallery.html"
    gallery.write_text(
        "<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
        f"<title>{escape(title)}</title><style>"
        "body{font-family:sans-serif;max-width:1200px;margin:2rem auto;padding:0 1rem;"
        "background:#f5f6f8;color:#202124}section{background:white;padding:1rem;"
        "margin:1rem 0;border-radius:10px}img{display:block;max-width:100%;height:auto;"
        "margin:auto}h1,h2{font-weight:600}</style></head><body>"
        f"<h1>{escape(title)}</h1>{''.join(cards)}</body></html>",
        encoding="utf-8",
    )
    webbrowser.open(gallery.as_uri())


def run_l0(args: argparse.Namespace, tag: str) -> None:
    output = output_path("L0", f"visual_walk_turn_{tag}")
    command = isaac_command(
        args,
        level="L0",
        output=output,
        extra=(
            "--mode",
            "walk",
            "--duration",
            "10",
            "--walk-start",
            "3",
            "--vx",
            "0.2",
        ),
    )
    print("L0：0--3 s 建立站姿，3--10 s 前进；点一下窗口，约第 5 s 按 A 或 D 转向。")
    run(command, dry_run=args.dry_run)


def run_l2_group(
    args: argparse.Namespace, tag: str, group: str, configs: Sequence[str]
) -> None:
    print("L2 对照会依次启动三次 GUI。观察期间不要按键，以免破坏单变量条件。")
    for config_name in configs:
        stem = Path(config_name).stem
        config = PRO2_ROOT / "experiments" / "L2" / group / config_name
        output = output_path("L2", group, f"{stem}_visual_{tag}")
        run(
            isaac_command(
                args, level="L2", output=output, config=config
            ),
            dry_run=args.dry_run,
        )
    titles = {
        "mpc_vx_weight": "L2 代价权重正式对照结果",
        "prediction_horizon": "L2 预测时域正式对照结果",
        "step_height": "L2 抬腿高度正式对照结果",
        "lateral_push": "L2 侧向扰动正式对照结果",
    }
    print("GUI 结束后打开既有正式定量实验的对照图，用于辨认肉眼不明显的差异。")
    open_figure_gallery(
        args, output_path("L2", group, "comparison"), titles[group]
    )


def run_h1(args: argparse.Namespace, tag: str) -> Path:
    output = output_path("L4", "01_robot_swap", f"visual_h1_{tag}")
    if args.record_video:
        print("提示：H1 独立入口目前不支持录像；H1 仍打开 GUI，其余动态任务照常录像。")
    command = [
        sys.executable,
        relative(PRO2_ROOT / "run_h1_isaac.py"),
        "--isaac-root",
        args.isaac_root,
        "--no-headless",
        "--render-hz",
        str(args.render_hz),
        "--output",
        relative(output),
    ]
    run(command, dry_run=args.dry_run)
    return output


def run_path(args: argparse.Namespace, tag: str) -> Path:
    output = output_path("L4", "02_path_tracking", f"visual_waypoints_{tag}")
    config = PRO2_ROOT / "experiments" / "L4" / "path_tracking" / "waypoints.json"
    run(
        isaac_command(args, level="L4", output=output, config=config),
        dry_run=args.dry_run,
    )
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "tools" / "analyze_l4_path_terrain.py"),
            relative(output),
        ],
        dry_run=args.dry_run,
    )
    open_figure_gallery(args, output / "analysis", "L4 多路径点跟踪分析")
    return output


def run_terrain(args: argparse.Namespace, tag: str) -> Path:
    output = output_path(
        "L4", "03_terrain_and_jump", f"visual_random_obstacles_{tag}"
    )
    config = (
        PRO2_ROOT
        / "experiments"
        / "L4"
        / "terrain"
        / "random_obstacles_seed01_full.json"
    )
    run(
        isaac_command(args, level="L4", output=output, config=config),
        dry_run=args.dry_run,
    )
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "tools" / "analyze_l4_path_terrain.py"),
            relative(output),
        ],
        dry_run=args.dry_run,
    )
    open_figure_gallery(args, output / "analysis", "L4 随机障碍穿越分析")
    return output


def run_jump(args: argparse.Namespace, tag: str) -> Path:
    """Show one vertical jump and write its evidence into one isolated run."""
    output = output_path(
        "L4", "03_terrain_and_jump", f"visual_vertical_jump_{tag}"
    )
    config = PRO2_ROOT / "experiments" / "L4" / "jump" / "vertical_jump.json"
    print(
        "L4 跳跃：机器人先完成站稳和下蹲，约第 8.5 s 起跳，"
        "随后落地并恢复站立。"
    )
    run(
        isaac_command(
            args,
            level="L4",
            output=output,
            config=config,
            extra=("--log-control-decomposition",),
        ),
        dry_run=args.dry_run,
    )
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "tools" / "analyze_l4_jump.py"),
            "--run",
            relative(output),
        ],
        dry_run=args.dry_run,
    )
    open_figure_gallery(args, output / "analysis", "L4 竖直跳跃验收分析")
    return output


def run_mpc(args: argparse.Namespace, tag: str) -> tuple[Path, Path, Path]:
    base = PRO2_ROOT / "experiments" / "L4" / "mpc_comparison"
    convex = output_path("L4", "04_mpc_comparison", f"visual_cpp_convex_{tag}")
    nmpc = output_path("L4", "04_mpc_comparison", f"visual_nmpc_rti_{tag}")
    analysis = output_path("L4", "04_mpc_comparison", f"visual_comparison_{tag}")
    for config_name, output in (
        ("cpp_convex_walk_run.json", convex),
        ("nmpc_rti_walk_run.json", nmpc),
    ):
        run(
            isaac_command(
                args,
                level="L4",
                output=output,
                config=base / config_name,
            ),
            dry_run=args.dry_run,
        )
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "run_l4_control.py"),
            "analyze-comparison",
            relative(convex / "states.csv"),
            relative(nmpc / "states.csv"),
            relative(analysis),
        ],
        dry_run=args.dry_run,
    )
    open_figure_gallery(args, analysis, "L4 Convex MPC 与 NMPC 对照")
    return convex, nmpc, analysis


def analyze_decomposition(
    args: argparse.Namespace, tag: str, states_csv: Path
) -> Path:
    analysis = output_path(
        "L4", "05_control_decomposition", f"visual_decomposition_{tag}", "analysis"
    )
    run(
        [
            sys.executable,
            relative(PRO2_ROOT / "run_l4_control.py"),
            "analyze-decomposition",
            relative(states_csv),
            relative(analysis),
        ],
        dry_run=args.dry_run,
    )
    open_figure_gallery(args, analysis / "figures", "L4 控制量与力矩分解")
    return analysis


def run_decomposition(args: argparse.Namespace, tag: str) -> Path:
    run_output = output_path(
        "L4", "05_control_decomposition", f"visual_decomposition_{tag}", "raw_run"
    )
    config = (
        PRO2_ROOT
        / "experiments"
        / "L4"
        / "mpc_comparison"
        / "cpp_convex_walk_run.json"
    )
    run(
        isaac_command(args, level="L4", output=run_output, config=config),
        dry_run=args.dry_run,
    )
    return analyze_decomposition(args, tag, run_output / "states.csv")


def main() -> int:
    args = arguments()
    if args.list:
        print("可用任务：")
        for name, description in TASKS.items():
            print(f"  {name:20s} {description}")
        return 0

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    tag = timestamp if args.tag is None else f"{timestamp}_{args.tag}"
    task = args.task
    if task == "l0":
        run_l0(args, tag)
    elif task == "l1":
        show_static_diagram(args, "L1")
    elif task == "l2-weight":
        run_l2_group(
            args, tag, "mpc_vx_weight", ("low.json", "baseline.json", "high.json")
        )
    elif task == "l2-horizon":
        run_l2_group(
            args,
            tag,
            "prediction_horizon",
            ("short_n5.json", "baseline_n10.json", "long_n15.json"),
        )
    elif task == "l2-step":
        run_l2_group(
            args,
            tag,
            "step_height",
            ("low_008.json", "baseline_012.json", "high_016.json"),
        )
    elif task == "l2-push":
        run_l2_group(
            args,
            tag,
            "lateral_push",
            ("no_push.json", "push_50N.json", "push_100N.json"),
        )
    elif task == "l3":
        show_static_diagram(args, "L3")
    elif task == "l4-robot":
        run_h1(args, tag)
    elif task == "l4-path":
        run_path(args, tag)
    elif task == "l4-terrain":
        run_terrain(args, tag)
    elif task == "l4-jump":
        run_jump(args, tag)
    elif task == "l4-mpc":
        run_mpc(args, tag)
    elif task == "l4-decomposition":
        run_decomposition(args, tag)
    elif task == "l4-all":
        run_h1(args, tag)
        run_path(args, tag)
        run_terrain(args, tag)
        run_jump(args, tag)
        convex, _, _ = run_mpc(args, tag)
        analyze_decomposition(args, tag, convex / "states.csv")
    else:  # argparse choices make this unreachable.
        raise ValueError(f"unknown task: {task}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
