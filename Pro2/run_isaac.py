#!/usr/bin/env python3
"""Build and run the OpenLoong C++ MPC/WBC in Isaac Sim with a Python entry point."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-root", help="已有 Isaac Sim 解压安装目录（当前适配6.0）")
    parser.add_argument("--repo-root", type=Path, default=ROOT / "OpenLoong-Dyn-Control")
    parser.add_argument("--level", choices=("L0", "L1", "L2", "L3", "L4"), default="L0",
                        help="成果分层目录；默认L0")
    parser.add_argument("--config", type=Path, help="实验JSON设置；命令行覆盖JSON")
    parser.add_argument("--check-env", action="store_true", help="只读检查环境")
    parser.add_argument("--build-only", action="store_true", help="只编译C++库，不启动Isaac")
    parser.add_argument("--no-build", action="store_true", help="使用已编译控制库")
    parser.add_argument("--rebuild-asset", action="store_true")
    parser.add_argument("--show-config", action="store_true")
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--record-video", action="store_true", default=None,
                        help="把视口按仿真时间录制为demo.mp4")
    parser.add_argument("--video-fps", type=int)
    parser.add_argument("--mode", choices=("stand", "walk"))
    parser.add_argument("--mpc-backend", choices=("cpp_convex", "nmpc_rti"),
                        help="足底wrench优化器；默认原C++凸MPC")
    parser.add_argument("--log-control-decomposition", action=argparse.BooleanOptionalAction,
                        default=None, help="记录MPC/WBC/PD/Isaac关节力矩分解")
    parser.add_argument("--mpc-horizon-steps", type=int,
                        help="MPC预测步数；改变后使用对应的独立C++构建目录")
    for name in ("duration", "dt", "mpc-dt", "swing-time", "step-height", "stand-height",
                 "walk-start", "vx", "vy", "wz", "render-hz", "log-hz", "push-at", "push-duration"):
        parser.add_argument("--" + name, type=float)
    parser.add_argument("--push-force", type=float, nargs=3, metavar=("FX", "FY", "FZ"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args(argv)
    nmpc_defaults = {
        "prediction_dt_s": .02,
        "horizon_steps": 3,
        "update_period_s": .02,
        "input_weight": 2e-6,
        "wrench_rate_weight": 1e-7,
        "state_weights": [4.0, 4.0, 2.0, 1.0, 30.0, 30.0,
                          .1, .1, .1, 5.0, 20.0, 5.0],
        "state_difference": 1e-5,
        "wrench_difference": .001,
    }
    defaults = dict(mode="walk", duration=20.0, dt=.001, mpc_dt=.005,
                    mpc_horizon_steps=10, swing_time=.4,
                    step_height=.12, stand_height=1.08, walk_start=5.0, vx=.2, vy=0.0, wz=0.0,
                    render_hz=30.0, log_hz=100.0, headless=False, push_at=None,
                    record_video=False, video_fps=30,
                    push_duration=.1, push_force=[0.0, 50.0, 0.0],
                    u_weight=1e-6, state_weights=[1,1,1,.001,100,1,.001,.001,.001,1,100,1],
                    input_weights=[1.0]*13,
                    path_tracking=None, terrain={"type": "flat"}, jump=None,
                    mpc_backend="cpp_convex", log_control_decomposition=False,
                    nmpc=dict(nmpc_defaults))
    if args.config:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        if not isinstance(config, dict) or set(config) - set(defaults):
            raise ValueError(f"配置字段无效；允许：{', '.join(defaults)}")
        defaults.update(config)
    for key, value in defaults.items():
        override = getattr(args, key, None)
        setattr(args, key, value if override is None else override)
    if args.mode not in ("stand", "walk") or not isinstance(args.headless, bool):
        raise ValueError("mode必须是stand/walk，headless必须是布尔值")
    if not isinstance(args.record_video, bool):
        raise ValueError("record_video必须是布尔值")
    if args.mpc_backend not in ("cpp_convex", "nmpc_rti"):
        raise ValueError("mpc_backend必须是cpp_convex或nmpc_rti")
    if not isinstance(args.log_control_decomposition, bool):
        raise ValueError("log_control_decomposition必须是布尔值")
    for key in ("duration", "dt", "mpc_dt", "swing_time", "stand_height",
                "render_hz", "log_hz", "push_duration", "u_weight"):
        value = getattr(args, key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key}必须是有限正数")
    for key in ("walk_start", "vx", "vy", "wz"):
        value = getattr(args, key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key}必须是有限数值")
    if args.walk_start < 3.0 or args.stand_height < .8 or args.stand_height > 1.2:
        raise ValueError("walk_start须至少3秒；stand_height须在0.8至1.2米之间")
    if args.mpc_dt < args.dt or abs(args.mpc_dt / args.dt - round(args.mpc_dt / args.dt)) > 1e-8:
        raise ValueError("mpc_dt必须是dt的正整数倍")
    if isinstance(args.mpc_horizon_steps, bool) or not isinstance(args.mpc_horizon_steps, int) or not 3 <= args.mpc_horizon_steps <= 30:
        raise ValueError("mpc_horizon_steps必须是3至30之间的整数")
    if args.dt > .01 or args.mpc_dt > .1 or args.swing_time <= args.mpc_dt:
        raise ValueError("dt须不超过0.01秒，mpc_dt不超过0.1秒，swing_time须大于mpc_dt")
    if isinstance(args.step_height, bool) or not isinstance(args.step_height, (int, float)) or not math.isfinite(args.step_height) or not 0 <= args.step_height < .5:
        raise ValueError("step_height须为[0, 0.5)米范围内的有限数值")
    if args.push_at is not None and (not isinstance(args.push_at, (int,float)) or not math.isfinite(args.push_at) or args.push_at < 0):
        raise ValueError("push_at必须是非负有限数值")
    for key, length in (("push_force", 3), ("state_weights", 12), ("input_weights", 13)):
        values = getattr(args, key)
        if not isinstance(values, (list, tuple)) or len(values) != length or any(
            isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values):
            raise ValueError(f"{key}需要{length}个有限数值")
        if key != "push_force" and any(v < 0 for v in values):
            raise ValueError(f"{key}不得为负")
        if key == "input_weights" and any(v == 0 for v in values):
            raise ValueError("input_weights必须严格大于零")
    if args.jobs < 1:
        raise ValueError("jobs必须至少为1")
    if isinstance(args.video_fps, bool) or not isinstance(args.video_fps, int) or not 1 <= args.video_fps <= 60:
        raise ValueError("video_fps必须是1至60之间的整数")
    # Validate nested L4 settings before compiling or starting Isaac.  Keep the
    # original path mapping because PathTracker constructs its own typed state;
    # terrain/jump normalizers return JSON-compatible canonical mappings.
    from isaac.path_tracking import normalize_path_tracking
    from isaac.terrain import normalize_terrain
    from isaac.jump_controller import normalize_jump
    normalize_path_tracking(args.path_tracking)
    args.terrain = normalize_terrain(args.terrain)
    args.jump = normalize_jump(args.jump)
    if not isinstance(args.nmpc, dict):
        raise ValueError("nmpc必须是对象")
    unknown_nmpc = set(args.nmpc) - set(nmpc_defaults)
    if unknown_nmpc:
        raise ValueError("nmpc含未知字段：" + ", ".join(sorted(unknown_nmpc)))
    nmpc = {**nmpc_defaults, **args.nmpc}
    for key in ("prediction_dt_s", "update_period_s", "input_weight",
                "state_difference", "wrench_difference"):
        value = nmpc[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not math.isfinite(value) or value <= 0:
            raise ValueError(f"nmpc.{key}必须是有限正数")
        nmpc[key] = float(value)
    rate_weight = nmpc["wrench_rate_weight"]
    if isinstance(rate_weight, bool) or not isinstance(rate_weight, (int, float)) or \
            not math.isfinite(rate_weight) or rate_weight < 0:
        raise ValueError("nmpc.wrench_rate_weight必须是有限非负数")
    nmpc["wrench_rate_weight"] = float(rate_weight)
    horizon = nmpc["horizon_steps"]
    if isinstance(horizon, bool) or not isinstance(horizon, int) or not 1 <= horizon <= 10:
        raise ValueError("nmpc.horizon_steps必须是1至10之间的整数")
    weights = nmpc["state_weights"]
    if not isinstance(weights, (list, tuple)) or len(weights) != 12 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or
            not math.isfinite(v) or v < 0 for v in weights):
        raise ValueError("nmpc.state_weights必须包含12个有限非负数")
    nmpc["state_weights"] = [float(v) for v in weights]
    if nmpc["update_period_s"] < args.dt:
        raise ValueError("nmpc.update_period_s不得小于物理步长dt")
    args.nmpc = nmpc
    if args.jump is not None:
        if args.mode != "stand":
            raise ValueError("跳跃实验必须使用 mode=stand")
        if args.terrain["type"] != "flat":
            raise ValueError("当前跳跃控制验收必须使用 flat 平地")
        if args.path_tracking is not None:
            raise ValueError("跳跃不能与路径跟踪同时启用")
        if args.mpc_backend != "cpp_convex":
            raise ValueError("跳跃必须使用原版 cpp_convex MPC")
        if args.push_at is not None:
            raise ValueError("跳跃实验禁止同时施加外力，避免把外力误当成控制器起跳")
        if abs(args.stand_height - args.jump["stand_height_m"]) > 1e-9:
            raise ValueError("stand_height 必须与 jump.stand_height_m 一致")
        crouch_start = args.jump["start_time_s"] - args.jump["crouch_duration_s"]
        if crouch_start < 3.0:
            raise ValueError(
                "jump.start_time_s - jump.crouch_duration_s 必须至少为3秒，"
                "先完成初始姿态保持")
        required_duration = (
            args.jump["start_time_s"] + args.jump["thrust_duration_s"] +
            args.jump["landing_timeout_s"] +
            args.jump["landing_settle_timeout_s"] +
            args.jump["recovery_duration_s"]
        )
        if args.duration + 1e-9 < required_duration:
            raise ValueError(
                f"跳跃duration至少需要{required_duration:.2f}秒，才能覆盖起跳、落地超时和恢复")
    if args.level != "L4" and (args.mpc_backend != "cpp_convex" or
                               args.log_control_decomposition or args.jump is not None or
                               args.path_tracking is not None or args.terrain["type"] != "flat"):
        raise ValueError("路径、地形、跳跃、NMPC和控制量分解实验必须使用--level L4")
    args.repo_root = args.repo_root.expanduser().resolve()
    if not (args.repo_root / "algorithm/mpc.cpp").is_file():
        raise FileNotFoundError(f"青龙工程不存在：{args.repo_root}")
    level_output = (ROOT / "output" / args.level).resolve()
    if args.output is None:
        args.output = level_output / datetime.now().strftime("%Y%m%d-%H%M%S")
    args.output = args.output.expanduser().resolve()
    try:
        args.output.relative_to(level_output)
    except ValueError as exc:
        raise ValueError(f"{args.level}成果必须保存在：{level_output}") from exc
    return args


def main(argv=None):
    args = arguments(argv)
    from isaac.runtime import locate_isaac, bootstrap
    if args.show_config:
        print(json.dumps(vars(args), default=str, ensure_ascii=False, indent=2))
        return 0
    if args.check_env:
        import shutil
        root = locate_isaac(args.isaac_root)
        gpu = subprocess.run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
                             capture_output=True, text=True) if shutil.which("nvidia-smi") else None
        print(json.dumps({"isaac_root": str(root), "isaac_version": (root/"VERSION").read_text().strip(),
                         "repo_root": str(args.repo_root), "cmake": shutil.which("cmake"),
                         "compiler": shutil.which("g++-11") or shutil.which("g++"),
                         "gpu_available": gpu is not None and gpu.returncode == 0,
                         "gpu": (gpu.stdout or gpu.stderr).strip() if gpu else "nvidia-smi未安装",
                         "feedback": "simulation ground truth, base-link origin/world frame",
                         "torque": "joint-side Nm; upstream PD+WBC; simulator drives disabled"},
                         ensure_ascii=False, indent=2))
        return 0 if gpu is not None and gpu.returncode == 0 else 1
    from isaac.native import build_controller, find_library
    if not args.no_build and not os.environ.get("OPENLOONG_RUNTIME_READY"):
        library = build_controller(args.repo_root, args.jobs, args.mpc_horizon_steps)
        print(f"[OpenLoong] 控制库：{library}", flush=True)
    else:
        find_library(args.mpc_horizon_steps)
    if args.build_only:
        return 0
    root = locate_isaac(args.isaac_root)
    bootstrap(root, Path(__file__).resolve(), ROOT / "cache/runtime")
    from isaac.simulation import run
    return run(args, root)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"[OpenLoong] {exc}", file=sys.stderr)
        raise SystemExit(1)
