"""Continuous local Panda IK, using Lula only for forward kinematics.

Independent global IK queries can switch redundant seven-joint branches even
with a warm start. We solve only the first pose globally; each subsequent pose
starts at its predecessor and follows a damped local Jacobian update.
"""

from __future__ import annotations

from typing import Callable

import numpy as np


PANDA_LOWER_LIMITS = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
PANDA_UPPER_LIMITS = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])


def _rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Small-angle spatial rotation error, expressed in the world frame."""
    return 0.5 * np.sum(np.cross(current.T, target.T), axis=0)


def _rotation_angle(current: np.ndarray, target: np.ndarray) -> float:
    cosine = (np.trace(current.T @ target) - 1.0) * 0.5
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def solve_continuous(
    solver,
    tip_positions: np.ndarray,
    tip_offset: np.ndarray,
    target_rotation: np.ndarray,
    target_quaternion: np.ndarray,
    seed: np.ndarray,
    progress: Callable[[int, int], None] | None = None,
) -> tuple[np.ndarray, float]:
    """Return continuous ``(samples, 7)`` joint references and max tip error.

    Units are metres/radians. ``tip_offset`` is expressed in ``panda_hand`` and
    ``target_rotation`` maps that frame into the world. Quaternion order follows
    Lula/Isaac: ``w, x, y, z``. Progress receives completed and total point counts.
    Hard acceptance tolerances are 50 micrometres and 0.5 milliradians; the local
    iterations use tighter stopping tolerances to avoid noisy reference slopes.
    """

    positions = np.asarray(tip_positions, dtype=np.float64)
    offset = np.asarray(tip_offset, dtype=np.float64)
    rotation = np.asarray(target_rotation, dtype=np.float64)
    quaternion = np.asarray(target_quaternion, dtype=np.float64)
    q = np.asarray(seed, dtype=np.float64).copy()
    if positions.ndim != 2 or positions.shape[1:] != (3,) or len(positions) == 0:
        raise ValueError("tip_positions must have nonempty shape (samples, 3)")
    if offset.shape != (3,) or rotation.shape != (3, 3) or quaternion.shape != (4,) or q.shape != (7,):
        raise ValueError("tip offset / rotation / quaternion / seed dimensions are invalid")
    if not all(np.all(np.isfinite(v)) for v in (positions, offset, rotation, quaternion, q)):
        raise ValueError("IK inputs must be finite")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError("target_rotation must be a proper rotation matrix")
    if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-6):
        raise ValueError("target_quaternion must be normalized")
    w, x, y, z = quaternion
    quaternion_rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])
    if not np.allclose(quaternion_rotation, rotation, atol=1e-6):
        raise ValueError("target_rotation and target_quaternion describe different orientations")

    lower, upper = PANDA_LOWER_LIMITS + 1e-7, PANDA_UPPER_LIMITS - 1e-7
    if np.any(q < lower) or np.any(q > upper):
        raise ValueError("IK seed is outside Panda joint limits")
    hand_targets = positions - rotation @ offset
    q, ok = solver.compute_inverse_kinematics(
        "panda_hand", hand_targets[0], quaternion, warm_start=q,
        position_tolerance=5e-5, orientation_tolerance=5e-4,
    )
    q = np.asarray(q, dtype=np.float64).copy()
    if not ok or q.shape != (7,) or not np.all(np.isfinite(q)):
        raise RuntimeError("第一个绘图点 IK 不可达，请检查绘图位置和尺寸。")
    if np.any(q < lower - 1e-6) or np.any(q > upper + 1e-6):
        raise RuntimeError("第一个绘图点 IK 超出 Panda 关节限位。")
    q = np.clip(q, lower, upper)

    joints = np.empty((len(positions), 7), dtype=np.float64)
    max_tip_error = 0.0
    eps, damping = 1e-5, 0.002

    def forward(values):
        p, r = solver.compute_forward_kinematics("panda_hand", values)
        return np.asarray(p, dtype=np.float64), np.asarray(r, dtype=np.float64)

    for index, hand_target in enumerate(hand_targets):
        previous = q.copy()
        hand_position, hand_rotation = forward(q)
        for _ in range(60):
            position_error = hand_target - hand_position
            orientation_error = _rotation_error(hand_rotation, rotation)
            angle = _rotation_angle(hand_rotation, rotation)
            if np.linalg.norm(position_error) <= 2e-6 and angle <= 2e-5:
                break
            error = np.concatenate((position_error, orientation_error))
            jacobian = np.empty((6, 7), dtype=np.float64)
            for joint in range(7):
                shifted = q.copy()
                # Backward difference at an upper bound keeps FK in range.
                delta = eps if q[joint] + eps <= upper[joint] else -eps
                shifted[joint] += delta
                shifted_position, shifted_rotation = forward(shifted)
                jacobian[:3, joint] = (shifted_position - hand_position) / delta
                jacobian[3:, joint] = _rotation_error(hand_rotation, shifted_rotation) / delta
            change = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping*damping*np.eye(6), error)
            largest_change = float(np.max(np.abs(change)))
            if largest_change > 0.05:
                change *= 0.05 / largest_change
            # Avoid taking a full Newton step when clamping or singularity makes
            # its linear approximation misleading. No global branch fallback.
            current_cost = float(np.dot(error, error))
            accepted = False
            for fraction in (1.0, 0.5, 0.25, 0.125):
                candidate = np.clip(q + fraction * change, lower, upper)
                candidate_position, candidate_rotation = forward(candidate)
                candidate_error = np.concatenate((hand_target - candidate_position,
                    _rotation_error(candidate_rotation, rotation)))
                if float(np.dot(candidate_error, candidate_error)) < current_cost:
                    q, hand_position, hand_rotation = candidate, candidate_position, candidate_rotation
                    accepted = True
                    break
            if not accepted:
                break

        tip_error = float(np.linalg.norm(hand_position + hand_rotation @ offset - positions[index]))
        angle_error = _rotation_angle(hand_rotation, rotation)
        if tip_error > 5e-5 or angle_error > 5e-4:
            raise RuntimeError(
                f"第 {index} 个局部 IK 点未收敛：笔尖误差 {tip_error:.6f} m，"
                f"朝向误差 {angle_error:.6f} rad。可缩小图案或调整绘图中心。"
            )
        if index > 0 and np.max(np.abs(q - previous)) > 0.12:
            raise RuntimeError(f"第 {index} 个局部 IK 点出现关节跳变，已停止执行。")
        joints[index] = q
        max_tip_error = max(max_tip_error, tip_error)
        if progress is not None and (index == 0 or (index + 1) % 1000 == 0 or index + 1 == len(positions)):
            progress(index + 1, len(positions))
    return joints, max_tip_error
