"""Launch an existing Isaac Sim installation without a shell wrapper."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def locate_isaac(explicit: str | None = None) -> Path:
    candidates = []
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if (candidate / "kit/python/bin/python3").is_file() and (candidate / "apps").is_dir():
            return candidate
        raise FileNotFoundError(f"指定目录不是可用的 Isaac Sim 安装：{candidate}")
    if os.environ.get("RAA_ISAAC_ROOT"):
        candidates.append(Path(os.environ["RAA_ISAAC_ROOT"]).expanduser())
    if os.environ.get("ISAAC_PATH"):
        candidates.append(Path(os.environ["ISAAC_PATH"]).expanduser())
    candidates.extend(sorted(Path.home().glob("isaacsim*"), reverse=True))
    candidates.extend((Path.home() / "isaaclab3-beta2/_isaac_sim", Path.home() / "IsaacLab/_isaac_sim"))
    for candidate in candidates:
        candidate = candidate.resolve()
        if (candidate / "kit/python/bin/python3").is_file() and (candidate / "apps").is_dir():
            return candidate
    raise FileNotFoundError("找不到已有 Isaac Sim 安装。请使用 --isaac-root /你的/isaacsim目录。")


def environment(root: Path) -> dict[str, str]:
    """Equivalent Linux library setup, implemented entirely in Python."""
    env = dict(os.environ)
    env.update(CARB_APP_PATH=str(root / "kit"), ISAAC_PATH=str(root), EXP_PATH=str(root / "apps"))
    python_paths = [
        root / "kit/python/lib/python3.12",
        root / "kit/python/lib/python3.12/site-packages",
        root / "python_packages",
        root / "exts/isaacsim.simulation_app",
        root / "kit/kernel/py",
        root / "kit/plugins/bindings-python",
    ]
    python_paths.extend(root / p for p in (
        "extsDeprecated/isaacsim.robot_motion.lula/pip_prebundle",
        "extsDeprecated/omni.isaac.ml_archive/pip_prebundle",
        "exts/isaacsim.asset.exporter.urdf/pip_prebundle",
        "exts/omni.isaac.core_archive/pip_prebundle",
        "exts/omni.pip.compute/pip_prebundle",
        "exts/omni.pip.cloud/pip_prebundle",
    ))
    # Core/compute wheels are bundled with Kit; no pip installation is needed.
    for pattern in ("omni.kit.pip_archive-*/pip_prebundle", "omni.cip.pip-*/pip_prebundle"):
        python_paths.extend((root / "extscache").glob(pattern))
    library_paths = [root, root / "exts/isaacsim.robot.schema/plugins/lib", root / "kit"]
    library_paths.extend(root / "kit" / p for p in (
        "kernel/plugins", "libs/iray", "plugins", "plugins/bindings-python",
        "plugins/carb_gfx", "plugins/rtx", "plugins/gpu.foundation",
    ))
    library_paths.extend(root / p for p in (
        "extsDeprecated/isaacsim.robot_motion.lula/pip_prebundle",
        "exts/isaacsim.asset.exporter.urdf/pip_prebundle",
    ))
    env["PYTHONPATH"] = os.pathsep.join(str(p) for p in python_paths if p.exists())
    env["LD_LIBRARY_PATH"] = os.pathsep.join([*(str(p) for p in library_paths if p.exists()), env.get("LD_LIBRARY_PATH", "")])
    env["LD_PRELOAD"] = os.pathsep.join([str(root / "kit/libcarb.so"), *filter(None, [env.get("LD_PRELOAD", "")])])
    env["RESOURCE_NAME"] = "raa-v0"
    env["PXR_WORK_THREAD_LIMIT"] = "8"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "8"
    env["RAA_RUNTIME_READY"] = str(root)
    # Do not let the caller's Conda interpreter or package search override Kit.
    env.pop("PYTHONHOME", None)
    return env


def bootstrap(root: Path, entry: Path) -> None:
    if os.environ.get("RAA_RUNTIME_READY") == str(root):
        return
    print(f"[raa-v0] 使用已有 Isaac Sim：{root}", flush=True)
    os.execve(str(root / "kit/python/bin/python3"), [str(root / "kit/python/bin/python3"), str(entry), *sys.argv[1:]], environment(root))
