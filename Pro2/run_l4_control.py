#!/usr/bin/env python3
"""Python-only entry point for L4 MPC checks, tests and result analysis."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parent


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-root", help="已有 Isaac Sim 解压安装目录")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check-dependencies")
    self_check = commands.add_parser("self-check")
    self_check.add_argument("--controller", required=True,
                            choices=("convex", "nmpc-slsqp", "nmpc-rti"))
    self_check.add_argument("--output", type=Path)
    commands.add_parser("test")
    decomposition = commands.add_parser("analyze-decomposition")
    decomposition.add_argument("input_csv", type=Path)
    decomposition.add_argument("output", type=Path)
    decomposition.add_argument("--joints", nargs="+")
    comparison = commands.add_parser("analyze-comparison")
    comparison.add_argument("convex_csv", type=Path)
    comparison.add_argument("nmpc_csv", type=Path)
    comparison.add_argument("output", type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    from isaac.runtime import bootstrap, locate_isaac
    isaac_root = locate_isaac(args.isaac_root)
    bootstrap(isaac_root, Path(__file__).resolve(), ROOT / "cache/control-runtime")

    if args.command == "check-dependencies":
        from control.check_dependencies import main as check
        return check()
    if args.command == "self-check":
        modules = {
            "convex": "control.convex_mpc_reference",
            "nmpc-slsqp": "control.nmpc_srbd",
            "nmpc-rti": "control.nmpc_rti",
        }
        module = __import__(modules[args.controller], fromlist=["main"])
        command = ["--self-check"]
        if args.output:
            command += ["--output", str(args.output)]
        return module.main(command)
    if args.command == "test":
        suite = unittest.defaultTestLoader.discover(str(ROOT / "control/tests"))
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        return 0 if result.wasSuccessful() else 1
    if args.command == "analyze-decomposition":
        from tools.analyze_l4_control_decomposition import main as analyze
        command = [str(args.input_csv), str(args.output)]
        if args.joints:
            command += ["--joints", *args.joints]
        analyze(command)
        return 0
    if args.command == "analyze-comparison":
        from tools.analyze_l4_mpc_comparison import main as analyze
        analyze([str(args.convex_csv), str(args.nmpc_csv), str(args.output)])
        return 0
    raise AssertionError(args.command)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError, KeyError) as exc:
        print(f"[OpenLoong L4] {exc}", file=sys.stderr)
        raise SystemExit(1)
