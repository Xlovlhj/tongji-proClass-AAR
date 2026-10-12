#!/usr/bin/env python3
"""robot arm artist / raa-v0. Run with python3 Pro1/main.py."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
DEFAULTS = {
    "preview": False, "check_env": False, "controller": "gc-pd", "headless": False,
    "isaac_root": None, "output": ROOT / "output", "logo_width": 0.16,
    "center": (0.45, 0.0), "draw_z": 0.305, "speed": 0.04, "dt": 0.001,
    "planning_dt": 1 / 60, "max_steps": 0, "hold_seconds": 15.0,
    "control": {"kp": [160, 160, 120, 120, 70, 55, 35],
                "kd": [22, 22, 18, 18, 2, 1.4, 0.8],
                "torque_limits": [87, 87, 87, 87, 12, 12, 12]}, "adaptive": {},
    "disturbance": {"enabled": False}, "record_video": None, "video_fps": 30,
    "video_width": 960, "video_height": 720, "video_tail_seconds": 2.0,
}


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="robot arm artist（raa-v0）：Isaac Panda 绘制 ChatGPT 六瓣结标志。",
                                argument_default=argparse.SUPPRESS)
    p.add_argument("--config", help="配置预设 pd / gc-pd / rbf / fuzzy，或 JSON 文件路径")
    p.add_argument("--show-config", action="store_true", help="打印最终配置后退出，不启动仿真")
    p.add_argument("--preview", action="store_true", help="只生成 SVG 图案和轨迹预览，不启动 Isaac，也不需要第三方库")
    p.add_argument("--check-env", action="store_true", help="检查已有 Isaac、GPU 和本地 Panda 模型，不启动仿真")
    display = p.add_mutually_exclusive_group()
    display.add_argument("--headless", action="store_true", help="无窗口运行，完成后退出并保存实验数据")
    display.add_argument("--gui", action="store_false", dest="headless", help="打开可视化窗口，覆盖配置中的 headless")
    p.add_argument("--controller", choices=("gc-pd", "pd", "rbf", "fuzzy"), help="选择关节力矩控制器；默认 gc-pd")
    p.add_argument("--isaac-root", help="已有 Isaac Sim 安装目录；默认自动查找")
    p.add_argument("--output", type=Path, help="结果目录；默认 Pro1/output，预设使用各控制器子目录")
    p.add_argument("--logo-width", type=float, help="标志最大边长，单位米；默认 0.16")
    p.add_argument("--center", type=float, nargs=2, metavar=("X", "Y"))
    p.add_argument("--draw-z", type=float, help="世界坐标中绘图平面高度，单位米；默认 0.305")
    p.add_argument("--speed", type=float, help="绘图目标速度，单位米/秒；默认 0.04")
    p.add_argument("--dt", type=float, help="物理与力矩控制步长，单位秒；默认 0.001")
    p.add_argument("--planning-dt", type=float, help="笛卡尔轨迹及 IK 采样间隔，单位秒；默认 1/60")
    p.add_argument("--max-steps", type=int, help="限制物理步数，用于短时间验证；0 表示绘制完整图案")
    p.add_argument("--hold-seconds", type=float, help="完成后窗口继续展示的秒数；默认 15；无窗口模式忽略")
    force = p.add_mutually_exclusive_group()
    force.add_argument("--disturbance", dest="disturbance_enabled", action="store_true",
                       help="在笔尖施加统一的外力脉冲，记录受扰绘图")
    force.add_argument("--no-disturbance", dest="disturbance_enabled", action="store_false",
                       help="关闭配置中的外力")
    p.add_argument("--record-video", type=Path, help="录制 Isaac Play 运行时的三维视口，保存为 MP4")
    p.add_argument("--video-fps", type=int, help="录制帧率；默认 30")
    p.add_argument("--video-width", type=int, help="视频宽度；默认 960，必须是偶数")
    p.add_argument("--video-height", type=int, help="视频高度；默认 720，必须是偶数")
    p.add_argument("--video-tail-seconds", type=float, help="视频结尾静止展示时长；默认 2 秒")
    return p


def main() -> int:
    explicit = vars(parser().parse_args())
    show_config = explicit.pop("show_config", False)
    force_override = explicit.pop("disturbance_enabled", None)
    if force_override is not None:
        explicit["disturbance"] = {"enabled": force_override}
    from raa.config import load_config, merge_config
    args = merge_config(DEFAULTS, load_config(explicit.get("config")), explicit)
    resolved = {name: value for name, value in vars(args).items()
                if name not in {"preview", "check_env", "config", "config_path"}}
    config_text = json.dumps(resolved, ensure_ascii=False, indent=2, default=str, allow_nan=False)
    if show_config:
        print(config_text)
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    if args.check_env:
        import subprocess
        from raa.runtime import locate_isaac
        root = locate_isaac(args.isaac_root)
        urdf = root / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
        print(json.dumps({"project": "robot arm artist", "version": "raa-v0", "isaac_root": str(root),
            "isaac_version": (root / "VERSION").read_text().strip(), "local_panda_urdf": str(urdf),
            "panda_model_exists": urdf.is_file(), "entry": "python3 Pro1/main.py"}, ensure_ascii=False, indent=2))
        result = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"], check=False)
        return result.returncode
    if args.preview:
        from raa.paths import build_reference, save_preview_svg
        reference = build_reference(width=args.logo_width, center_xy=tuple(args.center), draw_z=args.draw_z,
                                    speed=args.speed, dt=args.planning_dt)
        save_preview_svg(args.output / "logo_preview.svg")
        print(f"[raa-v0] 图案预览：{args.output / 'logo_preview.svg'}")
        print(f"[raa-v0] 共 {len(reference.strokes)} 笔，参考时长 {reference.duration:.2f} s，{len(reference.samples)} 个规划点")
        return 0
    from raa.runtime import bootstrap, locate_isaac
    root = locate_isaac(args.isaac_root)
    bootstrap(root, Path(__file__).resolve())
    (args.output / "resolved_config.json").write_text(config_text + "\n", encoding="utf-8")
    from raa.simulation import run
    return run(args, root)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, FileNotFoundError) as exc:
        print(f"[raa-v0] {exc}", file=sys.stderr)
        raise SystemExit(2)
