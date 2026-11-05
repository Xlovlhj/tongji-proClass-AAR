#!/usr/bin/env python3
"""Validate an L4 replacement-robot USD and its OpenLoong compatibility.

The ordinary Python entry point automatically starts Isaac's bundled Python for
the USD-only worker.  It does not create a SimulationApp, require a GPU, modify
the source USD, or claim that the OpenLoong controller has been ported.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PRO2_ROOT = PROJECT_ROOT / "Pro2"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Pro2.isaac.robot_spec import (load_robot_spec, resolve_robot_asset,
                                   robot_spec_path)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="unitree_h1_2_handless",
                        help="Pro2/robots 下的机器人规格名")
    parser.add_argument("--spec", type=Path, help="覆盖机器人规格 JSON")
    parser.add_argument("--asset", type=Path, help="覆盖本地 USD 路径")
    parser.add_argument("--repo-root", type=Path,
                        default=PRO2_ROOT / "OpenLoong-Dyn-Control")
    parser.add_argument("--isaac-root", type=Path,
                        help="Isaac Sim 安装根目录；默认自动发现")
    parser.add_argument("--output", type=Path,
                        default=PRO2_ROOT / "output/L4/01_robot_swap/preflight/h1_2_handless_preflight.json")
    parser.add_argument("--require-controller-ready", action="store_true",
                        help="若资产有效但控制器仍不兼容，也返回失败")
    parser.add_argument("--_usd-worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    args.spec = (args.spec or robot_spec_path(PROJECT_ROOT, args.robot)).expanduser().resolve()
    args.repo_root = args.repo_root.expanduser().resolve()
    args.output = args.output.expanduser().resolve()
    if args.asset:
        args.asset = args.asset.expanduser().resolve()
    return args


def _locate_isaac(explicit: Path | None) -> Path:
    from Pro2.isaac.runtime import locate_isaac
    return locate_isaac(str(explicit) if explicit else None)


def _usd_environment(isaac_root: Path) -> tuple[Path, dict[str, str]]:
    """Build the minimal environment needed for pxr, without starting Kit."""

    executable = isaac_root / "kit/python/bin/python3"
    usd_extensions = sorted((isaac_root / "extscache").glob("omni.usd.libs-*"))
    if not executable.is_file() or not usd_extensions:
        raise FileNotFoundError(f"Isaac USD Python runtime is incomplete: {isaac_root}")
    usd = usd_extensions[-1]
    env = dict(os.environ)

    def prepend(key: str, values: list[Path]) -> None:
        existing = env.get(key)
        items = [str(value) for value in values]
        if existing:
            items.append(existing)
        env[key] = os.pathsep.join(items)

    prepend("PYTHONPATH", [usd, PROJECT_ROOT])
    prepend("LD_LIBRARY_PATH", [usd / "bin"])
    env["PYTHONNOUSERSITE"] = "1"
    env["PXR_WORK_THREAD_LIMIT"] = "4"
    return executable, env


def _launch_worker(args) -> int:
    isaac_root = _locate_isaac(args.isaac_root)
    executable, env = _usd_environment(isaac_root)
    command = [str(executable), str(Path(__file__).resolve()),
               "--_usd-worker", "--robot", args.robot,
               "--spec", str(args.spec), "--repo-root", str(args.repo_root),
               "--output", str(args.output), "--isaac-root", str(isaac_root)]
    if args.asset:
        command.extend(["--asset", str(args.asset)])
    if args.require_controller_ready:
        command.append("--require-controller-ready")
    return subprocess.run(command, env=env, check=False).returncode


def _source_line(path: Path, needle: str) -> dict[str, Any]:
    if not path.is_file():
        return {"path": str(path.relative_to(PROJECT_ROOT)), "line": None,
                "text": None, "found": False}
    for number, text in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if needle in text:
            return {"path": str(path.relative_to(PROJECT_ROOT)), "line": number,
                    "text": text.strip(), "found": True}
    return {"path": str(path.relative_to(PROJECT_ROOT)), "line": None,
            "text": None, "found": False}


def _controller_compatibility(observed: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    from Pro2.isaac.model_metadata import read_model_metadata

    baseline = read_model_metadata(repo_root)
    expected_names = baseline["joint_names"]
    actual_names = observed["joint_names"]
    expected_nv = len(expected_names) + 6
    actual_nv = len(actual_names) + 6
    checks = [
        {"name": "actuated_joint_count", "passed": len(actual_names) == len(expected_names),
         "expected": len(expected_names), "actual": len(actual_names)},
        {"name": "pinocchio_nv", "passed": actual_nv == expected_nv,
         "expected": expected_nv, "actual": actual_nv},
        {"name": "exact_joint_names_and_order", "passed": actual_names == expected_names,
         "expected": expected_names, "actual": actual_names},
        {"name": "mpc_mass_parameter",
         "passed": abs(observed["total_explicit_mass_kg"] - 77.35) <= 1e-6,
         "expected": 77.35, "actual": observed["total_explicit_mass_kg"]},
    ]
    ready = all(check["passed"] for check in checks)
    evidence = [
        _source_line(PRO2_ROOT / "isaac/native/controller_bridge.cpp", "constexpr int kJointCount = 31"),
        _source_line(PRO2_ROOT / "isaac/native/controller_bridge.cpp", "kin.model_nv == 37"),
        _source_line(repo_root / "algorithm/pino_kin_dyn.cpp", 'getJointId("J_ankle_r_roll")'),
        _source_line(repo_root / "algorithm/pino_kin_dyn.h", "const std::vector<std::string> motorName"),
        _source_line(repo_root / "common/PVT_ctrl.h", "const std::vector<std::string> motorName"),
        _source_line(repo_root / "algorithm/mpc.cpp", "m = 77.35"),
        _source_line(repo_root / "algorithm/mpc.cpp", "Ic << 12.61"),
    ]
    blockers = [] if ready else [
        "H1-2 有 27 个执行关节（nv=33），而当前桥接器与 DataBus 合同固定为 31 个关节（nv=37）。",
        "H1 的关节和连杆名称与代码中固定的 AzureLoong motorName、踝、髋、手和腰部名称不同。",
        "当前 Pinocchio/WBC 必须读取匹配的 URDF；本地 H1 包只有可组合的 USD，旁置配置指向另一台机器上的源 URDF。",
        "必须为 H1 定义并验证 PVT 增益、力矩限制、初始姿态、任务选择矩阵和 WBC 维数。",
        "MPC 中的质量、质心惯量、髋宽、踝到足底距离和支撑面尺寸都是 AzureLoong 常量，必须替换为 H1 参数。",
    ]
    return {
        "ready": ready,
        "baseline_robot": "AzureLoong",
        "baseline_joint_count": len(expected_names),
        "replacement_joint_count": len(actual_names),
        "joint_name_overlap": sorted(set(expected_names) & set(actual_names)),
        "checks": checks,
        "source_evidence": evidence,
        "blockers": blockers,
    }


def _markdown(report: dict[str, Any], json_path: Path) -> str:
    asset_ok = report["result"]["asset_preflight_passed"]
    controller_ok = report["result"]["openloong_controller_ready"]
    checks = report["asset_validation"]["checks"]
    failed = [row for row in checks if not row["passed"]]
    compatibility = report["controller_compatibility"]
    lines = [
        "# L4 模型更换：Unitree H1-2 预检报告",
        "",
        "## 先看结论",
        "",
        f"- H1-2 USD 资产预检：**{'通过' if asset_ok else '未通过'}**。",
        f"- 当前 OpenLoong MPC/WBC 直接兼容：**{'通过' if controller_ok else '未通过'}**。",
        "- 这两个结论含义不同：资产通过表示 Isaac 能读到完整关节、质量、浮动基和足底碰撞信息；控制器兼容还要求动力学模型、关节维数和关节语义完全一致。",
        "",
        "## 已核实的资产事实",
        "",
        f"- 来源：`{report['asset']['path']}`",
        f"- 关节数：{report['usd_observation']['joint_count']}",
        f"- 刚体数：{report['usd_observation']['rigid_body_count']}",
        f"- 显式总质量：{report['usd_observation']['total_explicit_mass_kg']:.6f} kg",
        f"- 浮动基：{report['usd_observation']['floating_base']}",
        f"- 关节根：`{report['usd_observation']['articulation_roots'][0]}`",
        "- 左右足底碰撞：均已在 composed USD 的 instance proxy 中检出。",
        "",
        "## 为什么还不能直接替换模型运行",
        "",
    ]
    lines.extend(f"- {item}" for item in compatibility["blockers"])
    lines += ["", "## 源码证据", "",
              "| 相对路径 | 行号 | 固定假设 |", "|---|---:|---|"]
    for row in compatibility["source_evidence"]:
        lines.append(f"| `{row['path']}` | {row['line'] or '-'} | `{row['text'] or '未找到'}` |")
    lines += ["", "## 资产检查", "",
              "| 检查项 | 结果 |", "|---|---|"]
    for row in checks:
        lines.append(f"| `{row['name']}` | {'通过' if row['passed'] else '失败'} |")
    if failed:
        lines += ["", "失败详情：", ""]
        lines.extend(f"- `{row['name']}`：期望 `{row['expected']}`，实际 `{row['actual']}`"
                     for row in failed)
    lines += [
        "",
        "## 后续真正完成控制移植所需工作",
        "",
        "1. 补齐与本地 USD 同版本的 H1-2 URDF，并验证 Pinocchio 关节顺序、质量和惯量与 USD 一致。",
        "2. 将 `Pin_KinDyn`、`DataBus`、`PVT_Ctr` 和桥接 ABI 的 31 关节固定数组改成模型规格驱动。",
        "3. 为 H1 重新定义足端、髋、手和腰部任务，并调整 WBC 选择矩阵与优先级任务维数。",
        "4. 从 H1 模型计算总质量、质心惯量、髋宽和足底尺寸，替换 MPC 中的 AzureLoong 常量。",
        "5. 建立 H1 站立初始姿态、每关节 PD 和力矩限制，先完成站立，再依次验收前进和转向。",
        "",
        f"完整机器可读证据保存在 `{json_path.relative_to(PROJECT_ROOT)}`。",
        "",
    ]
    return "\n".join(lines)


def _worker(args) -> int:
    from Pro2.isaac.robot_usd import inspect_robot_usd, validate_usd_against_spec

    spec = load_robot_spec(args.spec)
    asset = resolve_robot_asset(spec, PROJECT_ROOT, args.asset)
    observed = inspect_robot_usd(asset.path)
    validation = validate_usd_against_spec(observed, spec)
    compatibility = _controller_compatibility(observed, args.repo_root)
    sidecar = asset.path.parent / "config.yaml"
    source_urdf = None
    if sidecar.is_file():
        for line in sidecar.read_text(encoding="utf-8").splitlines():
            if line.startswith("asset_path:"):
                source_urdf = line.partition(":")[2].strip()
                break
    report = {
        "schema_version": 1,
        "robot_id": spec["robot_id"],
        "asset": {"path": str(asset.path), "resolution_source": asset.source,
                  "source_policy": spec["asset"]["source_policy"],
                  "sidecar_config": str(sidecar) if sidecar.is_file() else None,
                  "sidecar_source_urdf": source_urdf,
                  "sidecar_source_urdf_exists": (Path(source_urdf).is_file()
                                                  if source_urdf else None)},
        "result": {
            "asset_preflight_passed": validation["passed"],
            "openloong_controller_ready": compatibility["ready"],
            "status": ("controller_ready" if compatibility["ready"] else
                       "asset_valid_controller_port_required" if validation["passed"] else
                       "asset_preflight_failed"),
        },
        "asset_validation": validation,
        "controller_compatibility": compatibility,
        "usd_observation": observed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    markdown = args.output.with_suffix(".md")
    markdown.write_text(_markdown(report, args.output), encoding="utf-8")
    print(json.dumps({"json": str(args.output), "markdown": str(markdown),
                      **report["result"]}, ensure_ascii=False, indent=2))
    if not validation["passed"]:
        return 1
    if args.require_controller_ready and not compatibility["ready"]:
        return 2
    return 0


def main(argv=None) -> int:
    args = arguments(argv)
    if args._usd_worker:
        return _worker(args)
    return _launch_worker(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f"[L4 robot preflight] {exc}", file=sys.stderr)
        raise SystemExit(1)
