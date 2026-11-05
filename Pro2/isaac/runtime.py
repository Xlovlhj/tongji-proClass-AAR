"""Discover and launch a local Isaac Sim bundle directly from Python."""
from __future__ import annotations

import os
from pathlib import Path
import sys


def locate_isaac(explicit: str | None = None) -> Path:
    def valid(path):
        return (path / "kit/python/bin/python3").is_file() and (path / "apps").is_dir()
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if valid(candidate):
            return candidate
        raise FileNotFoundError(f"不是可用的 Isaac Sim 安装目录：{candidate}")
    candidates = [Path(os.environ[k]).expanduser() for k in ("OPENLOONG_ISAAC_ROOT", "ISAAC_PATH") if os.environ.get(k)]
    candidates.extend(sorted(Path.home().glob("isaacsim*"), reverse=True))
    candidates.extend([Path.home() / "IsaacLab/_isaac_sim", Path.home() / "isaaclab3-beta2/_isaac_sim"])
    for candidate in candidates:
        if valid(candidate):
            return candidate.resolve()
    raise FileNotFoundError("未找到 Isaac Sim，使用 --isaac-root 指定已有安装目录。")


def environment(root: Path, cache: Path) -> dict[str, str]:
    env = dict(os.environ)
    env.update(CARB_APP_PATH=str(root / "kit"), ISAAC_PATH=str(root), EXP_PATH=str(root / "apps"))
    versions = sorted((root / "kit/python/lib").glob("python3.*"))
    if not versions:
        raise FileNotFoundError(f"Isaac 自带 Python 标准库不存在：{root}")
    python_lib = versions[-1]
    python_paths = [python_lib, python_lib / "site-packages", root / "python_packages",
                    root / "exts/isaacsim.simulation_app", root / "kit/kernel/py",
                    root / "kit/plugins/bindings-python"]
    python_paths.extend(root / p for p in (
        "extsDeprecated/omni.isaac.ml_archive/pip_prebundle",
        "exts/isaacsim.asset.exporter.urdf/pip_prebundle",
        "exts/omni.isaac.core_archive/pip_prebundle",
        "exts/omni.pip.compute/pip_prebundle", "exts/omni.pip.cloud/pip_prebundle"))
    for pattern in ("omni.kit.pip_archive-*/pip_prebundle", "omni.cip.pip-*/pip_prebundle"):
        python_paths.extend((root / "extscache").glob(pattern))
    library_paths = [root, root / "exts/isaacsim.robot.schema/plugins/lib", root / "kit"]
    library_paths.extend(root / "kit" / p for p in (
        "kernel/plugins", "libs/iray", "plugins", "plugins/bindings-python",
        "plugins/carb_gfx", "plugins/rtx", "plugins/gpu.foundation"))
    library_paths.append(root / "exts/isaacsim.asset.exporter.urdf/pip_prebundle")
    env["PYTHONPATH"] = os.pathsep.join(str(p) for p in python_paths if p.exists())
    env["LD_LIBRARY_PATH"] = os.pathsep.join(str(p) for p in library_paths if p.exists())
    env["LD_PRELOAD"] = str(root / "kit/libcarb.so")
    env["PXR_WORK_THREAD_LIMIT"] = "8"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "4"
    env["RESOURCE_NAME"] = "openloong-mpc-wbc"
    env["OPENLOONG_RUNTIME_READY"] = str(root)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    for key, folder in (("XDG_CACHE_HOME", "xdg-cache"), ("XDG_CONFIG_HOME", "xdg-config"),
                        ("XDG_DATA_HOME", "xdg-data"), ("CUDA_CACHE_PATH", "cuda"),
                        ("WARP_CACHE_PATH", "warp")):
        path = cache / folder
        path.mkdir(parents=True, exist_ok=True)
        env[key] = str(path)
    env.pop("PYTHONHOME", None)
    return env


def bootstrap(root: Path, entry: Path, cache: Path) -> None:
    if os.environ.get("OPENLOONG_RUNTIME_READY") == str(root):
        return
    print(f"[OpenLoong] 使用 Isaac Sim：{root}", flush=True)
    executable = root / "kit/python/bin/python3"
    os.execve(str(executable), [str(executable), str(entry), *sys.argv[1:]], environment(root, cache))
