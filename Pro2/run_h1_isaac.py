#!/usr/bin/env python3
"""Run a real Unitree H1-2 standing experiment in local Isaac Sim.

This is an independent L4 model-swap experiment.  It validates the replacement
asset and the local H1-2 FixStand joint-position controller.  It does not claim
that the AzureLoong-specific OpenLoong MPC/WBC has been ported to H1.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="unitree_h1_2_handless")
    parser.add_argument("--spec", type=Path)
    parser.add_argument("--asset", type=Path)
    parser.add_argument("--isaac-root")
    parser.add_argument("--output", type=Path,
                        help="独立实验目录；省略时按时间生成，避免覆盖既有成果")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--transition-time", type=float, default=3.0,
                        help="从零位插值到目标姿态的时间；0 表示直接初始化在目标姿态")
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--base-height", type=float, default=1.031)
    parser.add_argument("--log-hz", type=float, default=100.0)
    parser.add_argument("--render-hz", type=float, default=30.0)
    parser.add_argument("--kp-scale", type=float, default=1.0)
    parser.add_argument("--kd-scale", type=float, default=1.0)
    parser.add_argument("--balance-pitch-kp", type=float, default=1.5,
                        help="骨盆 Pitch 到双踝 Pitch 目标修正的比例")
    parser.add_argument("--balance-pitch-kd", type=float, default=0.1,
                        help="骨盆 Pitch 角速度到双踝 Pitch 目标修正的比例")
    parser.add_argument("--balance-roll-kp", type=float, default=1.2,
                        help="骨盆 Roll 到双踝 Roll 目标修正的比例")
    parser.add_argument("--balance-roll-kd", type=float, default=0.15,
                        help="骨盆 Roll 角速度到双踝 Roll 目标修正的比例")
    parser.add_argument("--balance-x-kp", type=float, default=2.0,
                        help="骨盆 X 位移到双踝 Pitch 目标修正的比例")
    parser.add_argument("--balance-x-kd", type=float, default=1.0,
                        help="骨盆 X 速度到双踝 Pitch 目标修正的比例")
    parser.add_argument("--balance-y-kp", type=float, default=0.0,
                        help="骨盆 Y 位移到双踝 Roll 目标修正的比例")
    parser.add_argument("--balance-y-kd", type=float, default=0.0,
                        help="骨盆 Y 速度到双踝 Roll 目标修正的比例")
    parser.add_argument("--max-balance-correction", type=float, default=0.75,
                        help="每个踝关节平衡修正的绝对上限，单位 rad")
    parser.add_argument("--self-collisions", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args(argv)
    from Pro2.isaac.robot_spec import robot_spec_path
    args.spec = (args.spec or robot_spec_path(PROJECT_ROOT, args.robot)).expanduser().resolve()
    if args.asset:
        args.asset = args.asset.expanduser().resolve()
    if args.output is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output = ROOT / f"output/L4/01_robot_swap/h1_fixstand_feedback_{stamp}"
    args.output = args.output.expanduser().resolve()
    allowed = (ROOT / "output/L4/01_robot_swap").resolve()
    try:
        args.output.relative_to(allowed)
    except ValueError as exc:
        raise ValueError(f"H1 模型更换成果必须保存在：{allowed}") from exc
    for name in ("duration", "dt", "base_height", "log_hz", "render_hz",
                 "kp_scale", "kd_scale"):
        value = getattr(args, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} 必须是有限正数")
    if args.duration < 3.0:
        raise ValueError("模型站立验收至少需要 3 秒")
    if not math.isfinite(args.transition_time) or args.transition_time < 0:
        raise ValueError("transition_time 必须是有限非负数")
    for name in ("balance_pitch_kp", "balance_pitch_kd", "balance_roll_kp",
                 "balance_roll_kd", "balance_x_kp", "balance_x_kd",
                 "balance_y_kp", "balance_y_kd", "max_balance_correction"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} 必须是有限非负数")
    if args.transition_time >= args.duration:
        raise ValueError("transition_time 必须小于 duration，留出目标姿态保持时间")
    if args.dt > 0.01:
        raise ValueError("dt 不得大于 0.01 秒")
    if not 0.7 <= args.base_height <= 1.3:
        raise ValueError("base_height 必须在 0.7 至 1.3 米之间")
    return args


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                               allow_nan=False, default=str) + "\n", encoding="utf-8")


def _markdown(args, report, metrics) -> str:
    passed = metrics["acceptance_passed"]
    return f"""# L4 模型更换：H1-2 站立实验

## 先看结论

- Isaac 实验是否完成：**{'是' if report['status'] == 'completed' else '否'}**。
- {args.duration:g} 秒 FixStand 验收：**{'通过' if passed else '未通过'}**。
- 实际控制方式：**Unitree H1-2 FixStand 关节位置 PD + 骨盆姿态/位置踝关节反馈**。
- OpenLoong MPC/WBC 是否控制本实验：**否**。

这个实验真实加载并运行了另一台人形机器人，用于验收模型资产、关节、接触和 Isaac 物理初始化。它没有把 AzureLoong 的 31 关节控制力矩直接套到 H1 的 27 关节上，因此不能作为“OpenLoong 已完成 H1 移植”的证据。

## 实验设置

- 机器人：Unitree H1-2 Handless
- 仿真时长：{args.duration:.3f} s
- 实际姿态插值时间：{args.transition_time:.3f} s
- 初始化方式：{'直接使用 FixStand 最终姿态' if args.transition_time == 0 else '从零位插值到 FixStand 最终姿态'}
- 最终姿态保持时间：{metrics['target_hold_duration_s']:.3f} s
- 物理步长：{args.dt:.6f} s
- 初始骨盆高度：{args.base_height:.4f} m
- 关节数：{report['joint_count']}
- 物理后端：PhysX CPU
- 控制目标：本机 H1-2 `FixStand` 的 27 关节目标姿态
- 参数来源：`{report['stand_control_source']}` 的 `{report['stand_control_source_section']}`
- 源配置姿态插值区间：{report['stand_control_source_transition_interval_s']} s
- PD 增益倍率：Kp × {args.kp_scale:g}，Kd × {args.kd_scale:g}
- 平衡反馈：Pitch ({args.balance_pitch_kp:g}, {args.balance_pitch_kd:g})，Roll ({args.balance_roll_kp:g}, {args.balance_roll_kd:g})，修正限幅 ±{args.max_balance_correction:g} rad
- 基座位置反馈：X ({args.balance_x_kp:g}, {args.balance_x_kd:g})，Y ({args.balance_y_kp:g}, {args.balance_y_kd:g})

## 验收指标

| 指标 | 数值 | 判据 |
|---|---:|---:|
| 完成仿真时间 | {metrics['simulated_duration_s']:.3f} s | ≥ {args.duration:.3f} s |
| 最低骨盆高度 | {metrics['minimum_base_height_m']:.6f} m | ≥ 0.85 m |
| 最大 Roll/Pitch | {metrics['maximum_tilt_rad']:.6f} rad | ≤ 0.20 rad |
| 最大水平漂移 | {metrics['maximum_horizontal_drift_m']:.6f} m | ≤ 0.10 m |
| 最终关节目标 RMSE | {metrics['final_joint_target_rmse_rad']:.6f} rad | ≤ 0.10 rad |
| 最大关节速度 | {metrics['maximum_abs_joint_velocity_rad_s']:.6f} rad/s | 记录项 |
| 最大 PD 力矩估计 | {metrics['maximum_abs_pd_effort_estimate_nm']:.6f} N·m | 记录项 |
| 最大显式 actuation 缓冲 | {metrics['maximum_abs_explicit_actuation_nm']:.6f} N·m | 记录项 |
| 最大投影关节力矩 | {metrics['maximum_abs_projected_joint_force_nm']:.6f} N·m | 记录项 |
| 最大踝关节平衡修正 | {metrics['maximum_abs_balance_correction_rad']:.6f} rad | ≤ {args.max_balance_correction:.3f} rad |

`explicit actuation` 是通过 `set_dof_efforts()` 写入的显式力矩缓冲；本实验使用隐式 PD drive，因此它应为零。`projected joint force` 是 PhysX 将连杆传入关节的作用力投影到转动自由度后的量，更适合观察本实验中的关节受力。平衡反馈只修正左右踝的姿态目标；表中的 27 关节 FixStand 名义姿态和每关节 Kp/Kd 仍直接来自上述配置文件。

## 文件

- `config.json`：实际运行参数；
- `states.csv`：基座、关节位置、速度、PD 力矩估计和 PhysX 投影关节力矩；
- `metrics.json`：验收指标；
- `run.json`：Isaac、资产和控制来源；
- `scene.usda`：本次组合后的场景。

## 与 OpenLoong 的边界

当前 OpenLoong 桥接器仍固定 31 关节和 `nv=37`，H1-2 是 27 关节和 `nv=33`。因此本实验只证明 H1-2 可以在 Isaac 中完成独立的物理站立验收。MPC/WBC 的 H1 移植仍需 H1 URDF、动力学参数、WBC 任务重建和控制桥维数参数化，详见同层目录的 `preflight/h1_2_handless_preflight.md`。
"""


def _run_experiment(args, isaac_root: Path, app) -> int:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage

    from Pro2.isaac.adapter import rpy_wxyz
    from Pro2.isaac.robot_spec import load_robot_spec, resolve_robot_asset

    spec = load_robot_spec(args.spec)
    asset = resolve_robot_asset(spec, PROJECT_ROOT, args.asset)
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    add_reference_to_stage(str(asset.path), "/World/H1")

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(2.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.05))
    ground.AddScaleOp().Set(Gf.Vec3f(10.0, 10.0, 0.05))
    ground.CreateDisplayColorAttr([(0.22, 0.24, 0.27)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    material = UsdShade.Material.Define(stage, "/World/GroundMaterial")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(1.0)
    physics_material.CreateDynamicFrictionAttr(1.0)
    physics_material.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(material)

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            articulation.CreateSolverPositionIterationCountAttr(16)
            articulation.CreateSolverVelocityIterationCountAttr(4)
            articulation.CreateEnabledSelfCollisionsAttr(args.self_collisions)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateDisableGravityAttr(False)
            body.CreateLinearDampingAttr(0.0)
            body.CreateAngularDampingAttr(0.0)

    robot = Articulation("/World/H1")
    world = World(physics_dt=args.dt, rendering_dt=args.dt,
                  stage_units_in_meters=1.0, device="cpu")
    world.reset()
    expected_names = spec["joints"]["isaac_dof_order"]
    actual_names = list(robot.dof_names)
    if actual_names != expected_names:
        raise RuntimeError(f"Isaac DOF 顺序与 H1 规格不一致：{actual_names}")
    count = len(actual_names)
    zeros = np.zeros(count, dtype=float)
    logical_names = spec["joints"]["names"]
    stand = spec["stand_control"]

    def reorder(field):
        by_name = dict(zip(logical_names, stand[field]))
        return np.asarray([by_name[name] for name in actual_names], dtype=float)

    targets = reorder("position_targets_rad")
    stiffness_np = reorder("kp_nm_per_rad").reshape(1, -1) * args.kp_scale
    damping_np = reorder("kd_nm_s_per_rad").reshape(1, -1) * args.kd_scale
    # FixStand's source configuration uses ``ts: [0, 3]``.  A zero-to-target
    # transition follows that behavior when transition_time is positive.  A
    # zero transition initializes directly at the same final target so model
    # and PD holding ability can be checked without transition dynamics.
    initial_targets = targets if args.transition_time == 0 else zeros
    robot.set_dof_positions(initial_targets)
    robot.set_dof_velocities(zeros)
    robot.set_world_poses(positions=np.array([[0.0, 0.0, args.base_height]]),
                          orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
    robot.set_velocities(linear_velocities=np.zeros((1, 3)),
                         angular_velocities=np.zeros((1, 3)))
    robot.set_dof_gains(stiffnesses=stiffness_np, dampings=damping_np)
    robot.set_dof_position_targets(initial_targets)
    maximum_efforts = robot.get_dof_max_efforts().numpy()[0].astype(float)

    report = {
        "status": "running",
        "robot": spec["display_name"],
        "asset": str(asset.path),
        "asset_resolution_source": asset.source,
        "isaac_root": str(isaac_root),
        "isaac_version": (isaac_root / "VERSION").read_text().strip(),
        "physics_backend": "PhysX CPU",
        "controller": "Unitree H1-2 FixStand joint position PD with pelvis-state ankle feedback",
        "openloong_mpc_wbc_used": False,
        "joint_count": count,
        "joint_names": actual_names,
        "stand_control_source": str(Path(stand["source"]).expanduser()),
        "stand_control_source_section": stand["source_section"],
        "stand_control_source_transition_interval_s": stand["source_transition_interval_s"],
        "position_targets_rad": targets.tolist(),
        "stiffness_nm_per_rad": stiffness_np[0].tolist(),
        "damping_nm_s_per_rad": damping_np[0].tolist(),
        "maximum_efforts_nm": maximum_efforts.tolist(),
        "config": vars(args),
    }
    _write_json(args.output / "run.json", report)

    log_every = max(1, round(1.0 / (args.log_hz * args.dt)))
    render_every = max(1, round(1.0 / (args.render_hz * args.dt)))
    total_steps = math.ceil(args.duration / args.dt)
    min_height = math.inf
    max_tilt = 0.0
    max_drift = 0.0
    max_joint_velocity = 0.0
    max_pd_effort = 0.0
    max_explicit_actuation = 0.0
    max_projected_joint_force = 0.0
    max_balance_correction = 0.0
    steps_done = 0
    fields = ["time_s", "target_alpha", "base_x_m", "base_y_m", "base_z_m",
              "roll_rad", "pitch_rad", "yaw_rad",
              "base_vx_m_s", "base_vy_m_s", "base_vz_m_s",
              "base_wx_rad_s", "base_wy_rad_s", "base_wz_rad_s"]
    fields += [prefix + name for prefix in ("q_nominal_rad:", "q_des_rad:",
                                             "q_balance_correction_rad:",
                                             "q_rad:", "dq_rad_s:",
                                             "tau_pd_est_nm:",
                                             "tau_explicit_actuation_nm:",
                                             "tau_projected_nm:")
               for name in actual_names]
    with (args.output / "states.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        name_to_index = {name: index for index, name in enumerate(actual_names)}
        ankle_pitch_indices = [name_to_index[name] for name in
                               ("left_ankle_pitch_joint", "right_ankle_pitch_joint")]
        ankle_roll_indices = [name_to_index[name] for name in
                              ("left_ankle_roll_joint", "right_ankle_roll_joint")]
        for step in range(total_steps):
            if not app.is_running():
                raise RuntimeError("Isaac 在实验完成前停止")
            command_time = step * args.dt
            target_alpha = (1.0 if args.transition_time == 0 else
                            min(1.0, command_time / args.transition_time))
            nominal_targets = target_alpha * targets
            feedback_positions, feedback_orientations = robot.get_world_poses()
            feedback_linear, feedback_angular = robot.get_velocities()
            feedback_positions = feedback_positions.numpy()[0].astype(float)
            feedback_linear = feedback_linear.numpy()[0].astype(float)
            feedback_roll, feedback_pitch, _ = rpy_wxyz(
                feedback_orientations.numpy()[0].astype(float))
            feedback_angular = feedback_angular.numpy()[0].astype(float)
            balance_correction = np.zeros(count, dtype=float)
            pitch_correction = np.clip(
                args.balance_pitch_kp * feedback_pitch +
                args.balance_pitch_kd * feedback_angular[1] +
                args.balance_x_kp * feedback_positions[0] +
                args.balance_x_kd * feedback_linear[0],
                -args.max_balance_correction, args.max_balance_correction)
            roll_correction = np.clip(
                args.balance_roll_kp * feedback_roll +
                args.balance_roll_kd * feedback_angular[0] +
                args.balance_y_kp * feedback_positions[1] +
                args.balance_y_kd * feedback_linear[1],
                -args.max_balance_correction, args.max_balance_correction)
            balance_correction[ankle_pitch_indices] = pitch_correction
            balance_correction[ankle_roll_indices] = roll_correction
            commanded_targets = nominal_targets + balance_correction
            robot.set_dof_position_targets(commanded_targets)
            world.step(render=(not args.headless and step % render_every == 0))
            elapsed = (step + 1) * args.dt
            q = robot.get_dof_positions().numpy()[0].astype(float)
            dq = robot.get_dof_velocities().numpy()[0].astype(float)
            pd_effort = np.clip(
                stiffness_np[0] * (commanded_targets - q) - damping_np[0] * dq,
                                -maximum_efforts, maximum_efforts)
            explicit_actuation = robot.get_dof_efforts().numpy()[0].astype(float)
            projected_force = robot.get_dof_projected_joint_forces().numpy()[0].astype(float)
            positions, orientations = robot.get_world_poses()
            position = positions.numpy()[0].astype(float)
            quaternion = orientations.numpy()[0].astype(float)
            linear, angular = robot.get_velocities()
            linear = linear.numpy()[0].astype(float)
            angular = angular.numpy()[0].astype(float)
            roll, pitch, yaw = rpy_wxyz(quaternion)
            state = np.concatenate([q, dq, pd_effort, explicit_actuation, projected_force,
                                    position, quaternion, linear, angular])
            if not np.all(np.isfinite(state)):
                raise RuntimeError(f"t={step * args.dt:.3f}s：状态出现 NaN/Inf")
            height = float(position[2])
            tilt = max(abs(roll), abs(pitch))
            drift = float(np.linalg.norm(position[:2]))
            min_height = min(min_height, height)
            max_tilt = max(max_tilt, tilt)
            max_drift = max(max_drift, drift)
            max_joint_velocity = max(max_joint_velocity, float(np.max(np.abs(dq))))
            max_pd_effort = max(max_pd_effort, float(np.max(np.abs(pd_effort))))
            max_explicit_actuation = max(
                max_explicit_actuation, float(np.max(np.abs(explicit_actuation))))
            max_projected_joint_force = max(
                max_projected_joint_force, float(np.max(np.abs(projected_force))))
            max_balance_correction = max(
                max_balance_correction, float(np.max(np.abs(balance_correction))))
            if step % log_every == 0:
                writer.writerow([elapsed, target_alpha, *position, roll, pitch, yaw,
                                 *linear, *angular, *nominal_targets, *commanded_targets,
                                 *balance_correction, *q, *dq, *pd_effort,
                                 *explicit_actuation, *projected_force])
            if height < 0.6 or tilt > 1.0:
                raise RuntimeError(
                    f"t={elapsed:.3f}s：H1 失稳，z={height:.3f}m，tilt={tilt:.3f}rad")
            steps_done = step + 1
            if step % max(1, round(1.0 / args.dt)) == 0:
                print(f"[H1] t={elapsed:.1f}s alpha={target_alpha:.3f} "
                      f"base=({position[0]:.3f},"
                      f"{position[1]:.3f},{position[2]:.3f}) "
                      f"rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})", flush=True)
                stream.flush()

    simulated = steps_done * args.dt
    final_q = robot.get_dof_positions().numpy()[0].astype(float)
    final_joint_target_rmse = float(np.sqrt(np.mean(np.square(final_q - targets))))
    metrics = {
        "simulated_duration_s": simulated,
        "transition_time_s": args.transition_time,
        "target_hold_duration_s": max(0.0, simulated - args.transition_time),
        "minimum_base_height_m": min_height,
        "maximum_tilt_rad": max_tilt,
        "maximum_horizontal_drift_m": max_drift,
        "maximum_abs_joint_velocity_rad_s": max_joint_velocity,
        "maximum_abs_pd_effort_estimate_nm": max_pd_effort,
        "maximum_abs_explicit_actuation_nm": max_explicit_actuation,
        "maximum_abs_projected_joint_force_nm": max_projected_joint_force,
        "maximum_abs_balance_correction_rad": max_balance_correction,
        "final_joint_target_rmse_rad": final_joint_target_rmse,
        "acceptance_thresholds": {
            "minimum_base_height_m": 0.85,
            "maximum_tilt_rad": 0.20,
            "maximum_horizontal_drift_m": 0.10,
            "final_joint_target_rmse_rad": 0.10,
        },
    }
    metrics["acceptance_passed"] = (
        simulated + 1e-12 >= args.duration and min_height >= 0.85 and
        max_tilt <= 0.20 and max_drift <= 0.10 and
        final_joint_target_rmse <= 0.10)
    _write_json(args.output / "metrics.json", metrics)
    stage.GetRootLayer().Export(str(args.output / "scene.usda"))
    report["status"] = "completed"
    report["steps"] = steps_done
    report["acceptance_passed"] = metrics["acceptance_passed"]
    _write_json(args.output / "run.json", report)
    (args.output / "H1站立验收.md").write_text(
        _markdown(args, report, metrics), encoding="utf-8")
    print(json.dumps({"output": str(args.output), **metrics},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if metrics["acceptance_passed"] else 2


def run(args, isaac_root: Path) -> int:
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "run.json").exists():
        raise FileExistsError(f"输出目录已有实验：{args.output}")
    _write_json(args.output / "config.json", vars(args))
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    from isaacsim import SimulationApp
    cache = ROOT / "cache/runtime"
    extra = ["--portable-root", str(cache / "kit"),
             f"--/app/tokens/cache={cache / 'kit-cache'}",
             f"--/app/tokens/data={cache / 'kit-data'}",
             f"--/app/tokens/logs={cache / 'kit-logs'}",
             f"--/app/tokens/omni_global_cache={cache / 'omni'}",
             f"--/app/userConfigPath={cache / 'user.config.json'}",
             f"--/log/file={args.output / 'isaac.log'}"]
    app = SimulationApp({"headless": args.headless, "width": 960, "height": 720,
                         "renderer": "MinimalRendering", "minimal_shading_mode": 3,
                         "anti_aliasing": 0, "multi_gpu": False,
                         "limit_cpu_threads": 8, "extra_args": extra},
                        experience=str(ROOT / "isaac/openloong.kit"))
    status = 1
    try:
        status = _run_experiment(args, isaac_root, app)
    except Exception as exc:
        traceback.print_exc()
        _write_json(args.output / "failure.json",
                    {"error": str(exc), "type": type(exc).__name__})
        run_path = args.output / "run.json"
        if run_path.is_file():
            report = json.loads(run_path.read_text(encoding="utf-8"))
            report.update(status="failed", error=str(exc), error_type=type(exc).__name__)
            _write_json(run_path, report)
    finally:
        app.close(exit_code=status)
    return status


def main(argv=None) -> int:
    args = arguments(argv)
    from Pro2.isaac.runtime import locate_isaac, bootstrap
    isaac_root = locate_isaac(args.isaac_root)
    bootstrap(isaac_root, Path(__file__).resolve(), ROOT / "cache/runtime")
    return run(args, isaac_root)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"[H1] {exc}", file=sys.stderr)
        raise SystemExit(1)
