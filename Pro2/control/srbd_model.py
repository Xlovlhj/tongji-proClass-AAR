"""Shared single-rigid-body dynamics utilities for the L4 MPC implementations.

State order deliberately matches ``OpenLoong-Dyn-Control/algorithm/mpc.cpp``::

    [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]

Angular and linear velocities, foot positions and contact wrenches are expressed
in the world frame.  A wrench is ``[Fx,Fy,Fz,Mx,My,Mz]`` and the 12-vector is
left foot followed by right foot.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - exercised by dependency probe
    raise RuntimeError(
        "L4 MPC 数学模块需要 NumPy；请通过 "
        "python3 Pro2/run_l4_control.py 启动"
    ) from exc


STATE_SIZE = 12
WRENCH_SIZE = 12


@dataclass(frozen=True)
class SrbdParameters:
    """Physical/contact parameters shared by both comparison controllers."""

    mass: float = 77.35
    gravity: float = 9.8
    inertia_body: tuple[tuple[float, float, float], ...] = (
        (12.61, 0.0, 0.37),
        (0.0, 11.15, 0.01),
        (0.37, 0.01, 2.15),
    )
    friction: float = 0.5
    foot_front: float = 0.073
    foot_back: float = 0.125
    foot_half_width: float = 0.025
    max_normal_force_factor: float = 3.0
    max_roll_moment: float = 20.0
    max_pitch_moment: float = 80.0
    max_yaw_moment: float = 100.0

    def validate(self) -> None:
        values = (self.mass, self.gravity, self.friction, self.foot_front,
                  self.foot_back, self.foot_half_width,
                  self.max_normal_force_factor)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("SRBD 质量、重力、摩擦和足底参数必须为有限正数")
        inertia = np.asarray(self.inertia_body, dtype=float)
        if inertia.shape != (3, 3) or not np.allclose(inertia, inertia.T):
            raise ValueError("inertia_body 必须是 3x3 对称矩阵")
        if np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError("inertia_body 必须正定")

    @property
    def inertia(self):
        self.validate()
        return np.asarray(self.inertia_body, dtype=float)

    @property
    def max_normal_force(self) -> float:
        return self.max_normal_force_factor * self.mass * self.gravity


def as_state(value, name="state"):
    result = np.asarray(value, dtype=float)
    if result.shape != (STATE_SIZE,) or not np.isfinite(result).all():
        raise ValueError(f"{name} 必须是包含 12 个有限数值的一维数组")
    return result


def as_foot_positions(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (2, 3) or not np.isfinite(result).all():
        raise ValueError("foot_positions_world 必须是形状为 (2,3) 的有限数组")
    return result


def as_reference(value, horizon):
    result = np.asarray(value, dtype=float)
    if result.shape == (STATE_SIZE,):
        result = np.repeat(result[None, :], horizon, axis=0)
    if result.shape != (horizon, STATE_SIZE) or not np.isfinite(result).all():
        raise ValueError(f"reference 必须是 (12,) 或 ({horizon},12) 的有限数组")
    return result


def as_contact_schedule(value, horizon):
    result = np.asarray(value, dtype=bool)
    if result.shape == (2,):
        result = np.repeat(result[None, :], horizon, axis=0)
    if result.shape != (horizon, 2):
        raise ValueError(f"contacts 必须是 (2,) 或 ({horizon},2) 的布尔数组")
    if np.any(np.sum(result, axis=1) == 0):
        raise ValueError("当前 SRBD 比较不允许预测步内双脚同时无接触")
    return result


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_rpy(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def euler_rates_from_world_omega(rpy, omega_world):
    """Return XYZ Euler rates for a world-frame angular velocity."""

    roll, pitch, _ = np.asarray(rpy, dtype=float)
    cp = math.cos(pitch)
    if abs(cp) < 1e-3:
        cp = math.copysign(1e-3, cp if cp else 1.0)
    body = rotation_rpy(rpy).T @ np.asarray(omega_world, dtype=float)
    sr, cr = math.sin(roll), math.cos(roll)
    tp = math.sin(pitch) / cp
    transform = np.array([
        [1.0, sr * tp, cr * tp],
        [0.0, cr, -sr],
        [0.0, sr / cp, cr / cp],
    ])
    return transform @ body


def nominal_wrench(params: SrbdParameters, contacts):
    """Static gravity-support wrench for one contact mode."""

    active = np.asarray(contacts, dtype=bool)
    if active.shape != (2,) or not active.any():
        raise ValueError("contacts 必须指定至少一只支撑脚")
    result = np.zeros(WRENCH_SIZE)
    force = params.mass * params.gravity / int(active.sum())
    for foot in range(2):
        if active[foot]:
            result[foot * 6 + 2] = force
    return result


def velocity_reference(state, command_vx_vy_wz, *, dt, horizon,
                       stand_height=None):
    """Build a smooth local reference from a body-frame velocity command.

    The reference starts at the measured pose and integrates the requested
    planar body velocity and yaw rate.  It is suitable for feeding both Python
    comparison controllers with exactly the same target trajectory.
    """

    current = as_state(state).copy()
    command = np.asarray(command_vx_vy_wz, dtype=float)
    if command.shape != (3,) or not np.isfinite(command).all():
        raise ValueError("command_vx_vy_wz 必须包含 3 个有限数值")
    if not math.isfinite(dt) or dt <= 0 or int(horizon) < 1:
        raise ValueError("dt 与 horizon 必须为正")
    height = current[5] if stand_height is None else float(stand_height)
    result = np.empty((int(horizon), STATE_SIZE))
    for step in range(int(horizon)):
        yaw = current[2] + command[2] * dt
        c, s = math.cos(yaw), math.sin(yaw)
        velocity_world = np.array([
            c * command[0] - s * command[1],
            s * command[0] + c * command[1],
            0.0,
        ])
        current[0:2] = 0.0
        current[2] = yaw
        current[3:6] += velocity_world * dt
        current[5] = height
        current[6:9] = [0.0, 0.0, command[2]]
        current[9:12] = velocity_world
        result[step] = current
    return result


def predict_contact_schedule(current_leg_state, next_leg_state, gait_phase,
                             *, dt, horizon, swing_time):
    """Replicate the current C++ MPC's one-transition contact prediction.

    Leg-state values follow ``DataBus::LegState``: ``0=LSt``, ``1=RSt``,
    ``2=DSt``.  The returned columns are ``[left_contact,right_contact]``.
    """

    names = {0: (True, False), 1: (False, True), 2: (True, True)}
    current_leg_state, next_leg_state = int(current_leg_state), int(next_leg_state)
    if current_leg_state not in names or next_leg_state not in names:
        raise ValueError("leg_state 必须是 0(LSt)、1(RSt) 或 2(DSt)")
    if not all(math.isfinite(v) for v in (gait_phase, dt, swing_time)):
        raise ValueError("步态相位和时间必须为有限数值")
    if dt <= 0 or swing_time <= dt or int(horizon) < 1:
        raise ValueError("要求 dt>0、swing_time>dt、horizon>=1")
    schedule = []
    for step in range(int(horizon)):
        predicted_phase = gait_phase + step * dt / swing_time
        schedule.append(names[next_leg_state if predicted_phase > 1.0 else current_leg_state])
    return np.asarray(schedule, dtype=bool)


def wrench_constraint_values(wrench, contacts, params: SrbdParameters,
                             *, circular_friction=True):
    """Inequality margins; every returned value must be nonnegative."""

    wrench = np.asarray(wrench, dtype=float).reshape(2, 6)
    contacts = np.asarray(contacts, dtype=bool)
    margins = []
    for foot, active in enumerate(contacts):
        f = wrench[foot, :3]
        moment = wrench[foot, 3:]
        if not active:
            margins.extend(1e-9 - np.abs(wrench[foot]))
            continue
        fx, fy, fz = f
        mx, my, mz = moment
        tangential = math.hypot(fx, fy) if circular_friction else max(abs(fx), abs(fy))
        margins.extend((fz, params.max_normal_force - fz,
                        params.friction * fz - tangential,
                        params.foot_half_width * fz - abs(mx),
                        params.foot_front * fz - my,
                        params.foot_back * fz + my,
                        params.max_yaw_moment - abs(mz)))
    return np.asarray(margins, dtype=float)
