"""Python/ctypes interface to the original C++ controller; all torques are joint-side Nm."""
from __future__ import annotations

import ctypes as ct
import math
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent


def _build_directory(mpc_horizon_steps: int) -> Path:
    if isinstance(mpc_horizon_steps, bool) or not isinstance(mpc_horizon_steps, int):
        raise ValueError("mpc_horizon_steps 必须是整数")
    if not 3 <= mpc_horizon_steps <= 30:
        raise ValueError("mpc_horizon_steps 必须在 3 至 30 之间")
    return HERE / "build" / f"mpc_n{mpc_horizon_steps}"


def build_controller(repo_root: Path, jobs: int = 2, mpc_horizon_steps: int = 10) -> Path:
    if not shutil.which("cmake"):
        raise RuntimeError("未找到 cmake。请安装 CMake 和 C++17 编译器。")
    build = _build_directory(mpc_horizon_steps)
    command = ["cmake", "-S", str(HERE / "native"), "-B", str(build),
               f"-DOPENLOONG_ROOT={repo_root}", "-DCMAKE_BUILD_TYPE=Release",
               f"-DOPENLOONG_MPC_HORIZON_STEPS={mpc_horizon_steps}"]
    if not (build / "CMakeCache.txt").exists() and shutil.which("g++-11"):
        command.extend(["-DCMAKE_CXX_COMPILER=g++-11", "-DCMAKE_C_COMPILER=gcc-11"])
    subprocess.run(command, check=True)
    subprocess.run(["cmake", "--build", str(build), "--parallel", str(jobs)], check=True)
    return find_library(mpc_horizon_steps)


def find_library(mpc_horizon_steps: int = 10) -> Path:
    candidates = list(_build_directory(mpc_horizon_steps).glob("**/libopenloong_controller.so"))
    if len(candidates) != 1:
        raise FileNotFoundError(
            f"N={mpc_horizon_steps} 的控制库尚未构建；"
            "运行 python3 Pro2/run_isaac.py --build-only "
            f"--mpc-horizon-steps {mpc_horizon_steps}"
        )
    return candidates[0].resolve()


def _vector(values, size, name):
    values = list(values)
    if len(values) != size or any(not math.isfinite(float(v)) for v in values):
        raise ValueError(f"{name} 必须包含 {size} 个有限数值")
    return (ct.c_double * size)(*map(float, values))


class Controller:
    def __init__(self, repo_root: Path, *, dt=.001, mpc_dt=.005, mpc_horizon_steps=10,
                 swing_time=.4, step_height=.12, stand_height=1.08,
                 library: Path | None = None):
        self._handle = None
        self.library_path = Path(library or find_library(mpc_horizon_steps)).resolve()
        self._lib = ct.CDLL(str(self.library_path), mode=ct.RTLD_LOCAL)
        lib = self._lib
        ptr = ct.POINTER(ct.c_double)
        lib.ol_create.argtypes = [ct.c_char_p] + [ct.c_double] * 5
        lib.ol_create.restype = ct.c_void_p
        lib.ol_destroy.argtypes = [ct.c_void_p]
        lib.ol_destroy.restype = None
        lib.ol_last_error.argtypes = []
        lib.ol_last_error.restype = ct.c_char_p
        lib.ol_joint_count.argtypes = [ct.c_void_p]
        lib.ol_joint_count.restype = ct.c_int
        lib.ol_joint_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_joint_name.restype = ct.c_char_p
        lib.ol_initial_joints.argtypes = [ct.c_void_p, ptr, ct.c_int]
        lib.ol_initial_joints.restype = ct.c_int
        lib.ol_step.argtypes = [ct.c_void_p, ct.c_double] + [ptr] * 7 + [ct.c_int, ptr]
        lib.ol_step.restype = ct.c_int
        lib.ol_step_jump.argtypes = (
            [ct.c_void_p, ct.c_double] + [ptr] * 6 + [ct.c_int] +
            [ct.c_double] * 4 + [ptr]
        )
        lib.ol_step_jump.restype = ct.c_int
        lib.ol_set_mpc_weights.argtypes = [ct.c_void_p, ct.c_double, ptr, ptr]
        lib.ol_set_mpc_weights.restype = ct.c_int
        lib.ol_set_external_mpc_wrench.argtypes = [ct.c_void_p, ct.c_int, ptr,
                                                   ct.c_int, ct.c_double,
                                                   ct.c_int, ct.c_int]
        lib.ol_set_external_mpc_wrench.restype = ct.c_int
        lib.ol_diagnostic_count.argtypes = [ct.c_void_p]
        lib.ol_diagnostic_count.restype = ct.c_int
        lib.ol_diagnostic_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_diagnostic_name.restype = ct.c_char_p
        lib.ol_diagnostics.argtypes = [ct.c_void_p, ptr, ct.c_int]
        lib.ol_diagnostics.restype = ct.c_int
        lib.ol_control_signal_count.argtypes = [ct.c_void_p]
        lib.ol_control_signal_count.restype = ct.c_int
        lib.ol_control_signal_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_control_signal_name.restype = ct.c_char_p
        lib.ol_control_signals.argtypes = [ct.c_void_p, ptr, ct.c_int]
        lib.ol_control_signals.restype = ct.c_int
        self._handle = lib.ol_create(str(repo_root).encode(), dt, mpc_dt, swing_time, step_height, stand_height)
        if not self._handle:
            raise RuntimeError(self.error())
        self.joint_names = [lib.ol_joint_name(self._handle, i).decode()
                            for i in range(lib.ol_joint_count(self._handle))]
        self._tau = (ct.c_double * len(self.joint_names))()
        self.diagnostic_names = [lib.ol_diagnostic_name(self._handle, i).decode()
                                 for i in range(lib.ol_diagnostic_count(self._handle))]
        self._diagnostics = (ct.c_double * len(self.diagnostic_names))()
        signal_count = lib.ol_control_signal_count(self._handle)
        if signal_count < 0:
            raise RuntimeError(self.error())
        self.control_signal_names = [lib.ol_control_signal_name(self._handle, i).decode()
                                     for i in range(signal_count)]
        self._control_signals = (ct.c_double * signal_count)()

    def error(self):
        return (self._lib.ol_last_error() or b"Unknown C++ controller error").decode(errors="replace")

    def _check(self, status):
        if status < 0:
            raise RuntimeError(self.error())

    def initial_joints(self):
        out = (ct.c_double * len(self.joint_names))()
        self._check(self._lib.ol_initial_joints(self._handle, out, len(out)))
        return list(out)

    def set_weights(self, u_weight, state_weights, input_weights):
        if not math.isfinite(u_weight) or u_weight <= 0:
            raise ValueError("u_weight 必须为有限正数")
        self._check(self._lib.ol_set_mpc_weights(self._handle, u_weight,
                    _vector(state_weights, 12, "state_weights"), _vector(input_weights, 13, "input_weights")))

    def set_external_mpc_wrench(self, wrench=None, *, solve_time=0.0,
                                solver_status=0, iterations=0):
        """Select a caller-solved contact wrench or restore internal Convex MPC.

        ``wrench`` is ordered left ``Fx,Fy,Fz,Mx,My,Mz`` followed by the right
        foot, in the world frame and SI units.  Passing ``None`` disables the
        override.  A nonzero solver status records the failed solve and makes
        the C++ bridge fall back to its internal Convex MPC for that update.
        """
        enabled = wrench is not None
        values = _vector(wrench, 12, "external_mpc_wrench") if enabled else None
        self._check(self._lib.ol_set_external_mpc_wrench(
            self._handle, int(enabled), values, 12 if enabled else 0,
            float(solve_time), int(solver_status), int(iterations)))

    def step(self, t, joints, joint_vel, base_pos, quat_wxyz, linvel_world, angvel_world, command, walking):
        if not self._handle:
            raise RuntimeError("控制器已关闭")
        n = len(self.joint_names)
        self._check(self._lib.ol_step(self._handle, float(t), _vector(joints, n, "joints"),
                    _vector(joint_vel, n, "joint_vel"), _vector(base_pos, 3, "base_pos"),
                    _vector(quat_wxyz, 4, "quat_wxyz"), _vector(linvel_world, 3, "linvel_world"),
                    _vector(angvel_world, 3, "angvel_world"), _vector(command, 3, "command"),
                    int(walking), self._tau))
        return list(self._tau)

    def step_jump(self, t, joints, joint_vel, base_pos, quat_wxyz,
                  linvel_world, angvel_world, reference):
        """Execute one native jump phase using a ``JumpReference``."""
        if not self._handle:
            raise RuntimeError("控制器已关闭")
        n = len(self.joint_names)
        self._check(self._lib.ol_step_jump(
            self._handle, float(t), _vector(joints, n, "joints"),
            _vector(joint_vel, n, "joint_vel"),
            _vector(base_pos, 3, "base_pos"),
            _vector(quat_wxyz, 4, "quat_wxyz"),
            _vector(linvel_world, 3, "linvel_world"),
            _vector(angvel_world, 3, "angvel_world"),
            int(reference.phase_code), float(reference.base_height_m),
            float(reference.vertical_velocity_m_s),
            float(reference.phase_progress),
            float(reference.landing_foot_advance_m), self._tau))
        return list(self._tau)

    def diagnostics(self):
        self._check(self._lib.ol_diagnostics(self._handle, self._diagnostics, len(self._diagnostics)))
        return dict(zip(self.diagnostic_names, map(float, self._diagnostics)))

    def control_signals(self):
        """Return the MPC/WBC/PD/final-torque decomposition for logging."""
        self._check(self._lib.ol_control_signals(
            self._handle, self._control_signals, len(self._control_signals)))
        return dict(zip(self.control_signal_names, map(float, self._control_signals)))

    def close(self):
        if self._handle:
            self._lib.ol_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
