"""Isaac Sim / PhysX adapter for the first robot arm artist experiment."""

from __future__ import annotations

import math
from pathlib import Path
import time


def run(args, isaac_root: Path) -> int:
    # Legacy World/Lula wrappers require the bundled torch. Load its native
    # libraries before Kit creates renderer/physics worker threads.
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    # Simulator imports deliberately follow SimulationApp construction.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "width": 960, "height": 720,
                         "renderer": "MinimalRendering", "minimal_shading_mode": 3,
                         "anti_aliasing": 0, "multi_gpu": False, "limit_cpu_threads": 8,
                         "samples_per_pixel_per_frame": 1},
                        experience=str(Path(__file__).resolve().parents[1] / "raa.experience.kit"))
    status = 0
    try:
        status = _run(args, isaac_root, app)
    except Exception:
        import traceback
        traceback.print_exc()
        status = 1
    finally:
        recorder = getattr(app, "_raa_recorder", None)
        if recorder is not None:
            recorder.abort("Simulation ended before successful video finalization")
        # Kit's fast shutdown otherwise masks exceptions with exit status 0.
        app.close(exit_code=status)
    return status


def _run(args, isaac_root, app) -> int:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

    from .control import PDConfig, TorquePD, tracking_metrics
    from .adaptive_control import make_adaptive_controller
    from .paths import build_reference, save_preview_svg, LOGO_SOURCE
    from .outputs import save_results
    from .disturbance import build_default_schedule, DisturbanceSchedule, ForcePulse, apply_tip_force
    import carb
    carb.settings.get_settings().set("/app/window/title", f"robot arm artist · raa-v0 · {args.controller}")

    pd_config = PDConfig(**args.control, gravity_compensation=args.controller != "pd")
    controller = (make_adaptive_controller(args.controller, pd_config=pd_config,
                  dt=args.dt, parameters=args.adaptive) if args.controller in {"rbf", "fuzzy"}
                  else TorquePD(pd_config))

    reference = build_reference(width=args.logo_width, center_xy=tuple(args.center), draw_z=args.draw_z,
                                speed=args.speed, dt=args.planning_dt)
    save_preview_svg(args.output / "logo_preview.svg")
    reference.save_csv(args.output / "reference_cartesian.csv")
    if "pulses" in args.disturbance:
        schedule = DisturbanceSchedule([ForcePulse(p["start_s"], p["end_s"], tuple(p["force_world_n"]))
                                       for p in args.disturbance["pulses"]],
                                      enabled=args.disturbance.get("enabled", False))
        if any(p.end_s > reference.duration for p in schedule.pulses):
            raise ValueError("外力脉冲结束时间不得超过完整参考轨迹")
    else:
        schedule = build_default_schedule(reference, enabled=args.disturbance.get("enabled", False))
    print(f"[raa-v0] 已生成 {len(reference.strokes)} 笔，参考时长 {reference.duration:.2f} s", flush=True)

    package = isaac_root / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description"
    urdf = package / "robots/panda_arm_hand.urdf"
    cache = Path(__file__).resolve().parents[1] / "cache"
    cache.mkdir(exist_ok=True)
    cached_assets = sorted(cache.glob("**/panda_arm_hand.usda"))
    if cached_assets:
        usd = str(cached_assets[0])
    else:
        if not urdf.is_file():
            raise FileNotFoundError(f"此 Isaac 安装未包含本地 Panda 模型：{urdf}")
        print("[raa-v0] 将本地 Panda URDF 导入项目缓存……", flush=True)
        config = URDFImporterConfig(urdf_path=str(urdf), usd_path=str(cache), fix_base=True,
            joint_drive_type="force", joint_target_type="none", override_joint_stiffness=0.0,
            override_joint_damping=0.0, ros_package_paths=[{"name": "franka_description", "path": str(package)}])
        usd = URDFImporter(config).import_urdf()
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    add_reference_to_stage(usd_path=str(usd), prim_path="/World/Panda")

    def box(path, xyz, scale, color, collision=False):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        cube.AddScaleOp().Set(Gf.Vec3d(*scale))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    box("/World/Ground", (0, 0, -0.035), (3, 3, 0.06), (0.11, 0.14, 0.18), True)
    box("/World/Table", (*args.center, args.draw_z - 0.02), (0.34, 0.34, 0.03), (0.63, 0.45, 0.30), True)
    box("/World/Paper", (*args.center, args.draw_z - 0.004), (0.28, 0.28, 0.001), (0.96, 0.97, 0.96))
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1000)
    distant = UsdLux.DistantLight.Define(stage, "/World/Sun")
    distant.CreateIntensityAttr(2500)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -30))
    # Keep the small drawing readable under the robot's hand.
    UsdLux.ShadowAPI.Apply(distant.GetPrim()).CreateShadowEnableAttr(False)
    fill = UsdLux.DistantLight.Define(stage, "/World/FillLight")
    fill.CreateIntensityAttr(1800)
    fill.AddRotateXYZOp().Set(Gf.Vec3f(-35, 35, 145))
    UsdLux.ShadowAPI.Apply(fill.GetPrim()).CreateShadowEnableAttr(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(False)

    def curve(path, points, color, width=0.0008):
        geom = UsdGeom.BasisCurves.Define(stage, path)
        geom.CreateTypeAttr("linear")
        geom.CreateWrapAttr("nonperiodic")
        geom.CreateCurveVertexCountsAttr([len(points)])
        geom.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        geom.CreateWidthsAttr([width])
        geom.SetWidthsInterpolation("constant")
        geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        return geom

    for i, stroke in enumerate(reference.strokes):
        curve(f"/World/Target/Stroke_{i}", [(p[0], p[1], args.draw_z - 0.002) for p in stroke], (0.5, 0.57, 0.65))
    # A visual force direction marker; it has no collision or rigid body.
    force_arrow = UsdGeom.Xform.Define(stage, "/World/ForceArrow")
    force_arrow.MakeInvisible()
    force_shaft = curve("/World/ForceArrow/Shaft", [(0, 0, 0), (0, 0, 0.01)], (1.0, 0.25, 0.02), 0.003)
    force_head = UsdGeom.Cone.Define(stage, "/World/ForceArrow/Head")
    force_head.CreateRadiusAttr(0.006)
    force_head.CreateHeightAttr(0.020)
    force_head.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.25, 0.02)])
    force_head_position = force_head.AddTranslateOp()
    force_head_orientation = force_head.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr(32.0)
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.15, -1.2, 1.2), Gf.Vec3d(0.30, 0, 0.42), Gf.Vec3d(0, 0, 1)).GetInverse()
    camera.AddTransformOp().Set(matrix)
    from omni.kit.viewport.utility import get_active_viewport

    robot = Articulation("/World/Panda")
    world = World(physics_dt=args.dt, rendering_dt=args.dt, stage_units_in_meters=1.0, device="cpu")
    world.reset()
    viewport = get_active_viewport()
    if viewport:
        viewport.set_active_camera("/World/Camera")
    if args.record_video and not viewport:
        raise RuntimeError("没有可录制的 Isaac 三维视口")
    arm_names = [f"panda_joint{i}" for i in range(1, 8)]
    arm = robot.get_dof_indices(arm_names).list()
    robot.set_dof_gains(stiffnesses=0.0, dampings=0.0, dof_indices=arm)
    hand_id = robot.get_link_indices("panda_hand").list()[0]
    hand_path = robot.link_paths[0][hand_id]
    hand = RigidPrim(hand_path)
    # A visual pen only: no table contact or added unmodelled physical load.
    pen = UsdGeom.Cylinder.Define(stage, hand_path + "/ArtistPen")
    pen.CreateRadiusAttr(0.003)
    pen.CreateHeightAttr(0.13)
    pen.CreateAxisAttr("Z")
    pen.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.065))
    pen.CreateDisplayColorAttr([Gf.Vec3f(0.07, 0.65, 0.48)])
    tip_offset = np.array([0, 0, 0.13])
    tip_mesh = UsdGeom.Sphere.Define(stage, hand_path + "/ArtistTip")
    tip_mesh.CreateRadiusAttr(0.004)
    tip_mesh.AddTranslateOp().Set(Gf.Vec3d(*tip_offset))
    tip_mesh.CreateDisplayColorAttr([Gf.Vec3f(0.03, 0.20, 0.15)])
    finger_names = [n for n in robot.dof_names if "finger" in n]
    if finger_names:
        fingers = robot.get_dof_indices(finger_names).list()
        robot.set_dof_positions(0.012, dof_indices=fingers)
        robot.set_dof_gains(stiffnesses=600.0, dampings=40.0, dof_indices=fingers)
        robot.set_dof_position_targets(0.012, dof_indices=fingers)

    motion_dir = isaac_root / "extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka"
    solver = LulaKinematicsSolver(robot_description_path=str(motion_dir / "rmpflow/robot_descriptor.yaml"),
                                  urdf_path=str(motion_dir / "lula_franka_gen.urdf"))
    if solver.get_joint_names() != arm_names:
        raise RuntimeError(f"IK 与关节顺序不一致：{solver.get_joint_names()}")
    times = np.asarray(reference.times)
    positions = np.asarray(reference.positions)
    # Tool +Z points down; the pen tip lies 13 cm below the hand frame.
    quaternion = np.array([0.0, 0.0, 1.0, 0.0])
    rotation = np.diag([-1.0, 1.0, -1.0])
    seed = np.array([0, -0.6, 0, -2.4, 0, 1.8, 0.75])
    print(f"[raa-v0] 连续求解 {len(positions)} 个 IK 点……", flush=True)
    from .kinematics import solve_continuous
    def planning_progress(index, total):
        print(f"[raa-v0] IK {index}/{total}", flush=True)
        world.render()
    joints, max_ik_error = solve_continuous(solver, positions, tip_offset, rotation, quaternion,
                                          seed, progress=planning_progress)
    slopes = np.gradient(joints, times, axis=0)
    slopes[[0, -1]] = 0
    np.savez_compressed(args.output / "reference_joints.npz", time=times, q=joints, dq=slopes, tip=positions)
    robot.set_dof_positions(joints[0], dof_indices=arm)
    robot.set_dof_velocities(np.zeros(7), dof_indices=arm)

    def tip_pose():
        p, quat = hand.get_world_poses()
        p, quat = p.numpy()[0], quat.numpy()[0]
        w, x, y, z = quat
        # Third column of the quaternion's rotation matrix suffices for +Z pen.
        local_z = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        return p + local_z * tip_offset[2]

    def sample(t):
        i = min(max(int(np.searchsorted(times, t, side="right")) - 1, 0), len(times) - 2)
        h = times[i + 1] - times[i]
        u = float(np.clip((t - times[i]) / h, 0, 1))
        q0, q1, m0, m1 = joints[i], joints[i + 1], slopes[i], slopes[i + 1]
        q_ref = (2*u**3-3*u**2+1)*q0 + (u**3-2*u**2+u)*h*m0 + (-2*u**3+3*u**2)*q1 + (u**3-u**2)*h*m1
        dq_ref = ((6*u**2-6*u)*q0 + (3*u**2-4*u+1)*h*m0 + (-6*u**2+6*u)*q1 + (3*u**2-2*u)*h*m1) / h
        target = positions[i] * (1-u) + positions[i + 1] * u
        down = reference.samples[i].pen_down and (u <= 1e-9 or reference.samples[i + 1].pen_down)
        return q_ref, dq_ref, target, down, reference.samples[i].stroke_index

    # Settle using our own torque controller; no position-controller assistance.
    initialiser = TorquePD(PDConfig(**args.control, gravity_compensation=True))
    for _ in range(round(1.0 / args.dt)):
        q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
        dq = robot.get_dof_velocities(dof_indices=arm).numpy()[0]
        gravity = robot.get_dof_gravity_compensation_forces(dof_indices=arm).numpy()[0]
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)) or np.max(np.abs(dq)) > 12:
            raise RuntimeError("初始化过程中关节状态异常；请减小 --dt。")
        robot.set_dof_efforts(initialiser.compute(q, dq, joints[0], np.zeros(7), gravity), dof_indices=arm)
        world.step(render=False)
    initial_tip = tip_pose()
    if np.linalg.norm(initial_tip - positions[0]) > 0.025:
        raise RuntimeError(f"初始化笔尖与 IK 不一致：误差 {np.linalg.norm(initial_tip-positions[0]):.4f} m")
    rows = []
    traces = {}
    trace_curves = {}
    total_steps = math.ceil(reference.duration / args.dt) + 1
    steps = min(total_steps, args.max_steps) if args.max_steps else total_steps
    print(f"[raa-v0] 开始绘图，控制器 {args.controller}，{steps} 个物理步", flush=True)
    wall_start = time.monotonic()
    render_interval = max(1, round((0.1 if args.headless else 1 / 30) / args.dt))
    stopped = False
    recorder = None
    if args.record_video:
        from .recording import VideoRecorder
        grid_frames = math.floor((steps - 1) * args.dt * args.video_fps + 1e-9) + 1
        last_grid_step = math.ceil((grid_frames - 1) / (args.dt * args.video_fps) - 1e-9)
        motion_frames = grid_frames + int(last_grid_step < steps - 1)
        expected_frames = motion_frames + round(args.video_tail_seconds * args.video_fps)
        recorder = VideoRecorder(viewport, args.record_video, fps=args.video_fps,
                                 width=args.video_width, height=args.video_height,
                                 expected_frames=expected_frames)
        app._raa_recorder = recorder
    physics_start_time = world.current_time

    def refresh_ink():
        for sid, points in traces.items():
            if len(points) < 2:
                continue
            # Show the measured XY drawing on the paper. Preserve the raw
            # 3D tip in traces/CSV and use it for force application and errors.
            ink_points = [(p[0], p[1], args.draw_z + 0.001) for p in points]
            if sid not in trace_curves:
                trace_curves[sid] = curve(f"/World/Ink/Stroke_{sid}", ink_points, (0.03, 0.65, 0.49), 0.001)
            else:
                trace_curves[sid].GetPointsAttr().Set([Gf.Vec3f(*p) for p in ink_points])
                trace_curves[sid].GetCurveVertexCountsAttr().Set([len(points)])

    for step in range(steps):
        if not app.is_running():
            stopped = True
            break
        t = step * args.dt
        q_ref, dq_ref, target, down, stroke = sample(min(t, reference.duration))
        q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
        dq = robot.get_dof_velocities(dof_indices=arm).numpy()[0]
        actual = tip_pose()
        gravity = robot.get_dof_gravity_compensation_forces(dof_indices=arm).numpy()[0]
        external = schedule.sample_step(step, args.dt)
        tau = controller.compute(q, dq, q_ref, dq_ref, gravity, dt=args.dt)
        if not np.all(np.isfinite(actual)) or np.max(np.abs(dq)) > 12:
            raise RuntimeError("状态异常或关节速度过大，已停止仿真。")
        row = {"time": t, **{f"target_{axis}": float(v) for axis, v in zip("xyz", target)},
               **{f"actual_{axis}": float(v) for axis, v in zip("xyz", actual)},
               "pen_down": bool(down), "stroke_index": int(stroke),
               "disturbance_active": bool(external.active), "disturbance_pulse_index": external.pulse_index,
               **{f"external_force_{axis}_n": float(v) for axis, v in zip("xyz", external.force_world)}}
        for prefix, values in (("q", q), ("dq", dq), ("q_ref", q_ref), ("tau", tau)):
            row.update({f"{prefix}{i+1}": float(v) for i, v in enumerate(values)})
        if args.controller in {"rbf", "fuzzy"}:
            row.update({f"adaptive_tau{i+1}": float(v) for i, v in enumerate(controller.last_compensation)})
            row.update({f"sliding_error{i+1}": float(v) for i, v in enumerate(controller.last_sliding_error)})
            row.update({f"weight_norm{i+1}": float(v) for i, v in enumerate(controller.weight_norms)})
        rows.append(row)
        if down and step % max(1, round(0.01 / args.dt)) == 0:
            traces.setdefault(stroke, []).append(tuple(float(v) for v in actual))
        if external.active:
            force_arrow.MakeVisible()
            direction = external.force_world / np.linalg.norm(external.force_world)
            arrow_start = actual + np.array([0.0, 0.0, 0.012])
            arrow_end = arrow_start + 0.035 * external.force_world
            force_shaft.GetPointsAttr().Set([Gf.Vec3f(*arrow_start), Gf.Vec3f(*arrow_end)])
            force_head_position.Set(Gf.Vec3d(*(arrow_end - direction * 0.010)))
            force_head_orientation.Set(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*direction)).GetQuat())
        else:
            force_arrow.MakeInvisible()
        if recorder and (t + 1e-10 >= recorder.motion_frame_count / args.video_fps or step == steps - 1):
            refresh_ink()
            before_capture = world.current_time
            recorder.capture(world.render, t)
            if abs(world.current_time - before_capture) > args.dt * 1e-6:
                recorder.abort("render capture advanced the physics clock")
                raise RuntimeError("录制过程额外推进了物理时间，已停止以避免影响控制比较")
        robot.set_dof_efforts(tau, dof_indices=arm)
        # Apply after capture and directly before the one physical step.
        # A force at the world-space pen tip includes its moment automatically.
        if schedule.enabled:
            apply_tip_force(hand, external, actual)
        world.step(render=False)
        if not recorder and step % render_interval == 0:
            refresh_ink()
            world.render()
        if step % round(10 / args.dt) == 0:
            print(f"[raa-v0] {t:.1f}/{reference.duration:.1f} s，笔尖误差 {np.linalg.norm(actual-target)*1000:.2f} mm", flush=True)
    if not rows:
        raise RuntimeError("未获得任何仿真样本。")
    metrics = tracking_metrics([[r[f"target_{a}"] for a in "xyz"] for r in rows],
        [[r[f"actual_{a}"] for a in "xyz"] for r in rows], [r["pen_down"] for r in rows],
        [[r[f"tau{i}"] for i in range(1,8)] for r in rows],
        torque_limits=controller.torque_limits, dt=args.dt)
    metadata = {"project": "robot arm artist", "version": "raa-v0", "simulator": "Isaac Sim / PhysX",
        "isaac_version": (isaac_root / "VERSION").read_text().strip(), "controller": args.controller,
        "completed": len(rows) == total_steps and not stopped, "reference_duration_s": reference.duration,
        "wall_time_s": time.monotonic() - wall_start, "physics_dt_s": args.dt,
        "max_ik_tip_error_m": max_ik_error, "logo_source": LOGO_SOURCE,
        "virtual_pen_offset_m": tip_offset.tolist(), "gravity_enabled": True,
        "arm_position_drives_enabled": False, "external_disturbance_enabled": schedule.enabled,
        "ink_visualization": "actual tip XY projected onto drawing plane",
        "ink_plane_z_m": args.draw_z + 0.001,
        "disturbance": schedule.metadata(), "disturbance_windows": schedule.metadata()["pulses"],
        "recovery_threshold_m": 0.002, "recovery_hold_s": 0.5,
        "timeline_play_enabled": world.is_playing(),
        "physics_elapsed_s": world.current_time - physics_start_time,
        "config_path": args.config_path, "controller_state": controller.state_summary(),
        "metrics": metrics}
    refresh_ink()
    if recorder:
        if recorder.last_sim_time_s is None or rows[-1]["time"] > recorder.last_sim_time_s + 1e-10:
            recorder.capture(world.render, rows[-1]["time"])
        recorder.hold_final(args.video_tail_seconds)
        metadata["video"] = recorder.finish()
    import json
    (args.output / "controller_state.json").write_text(
        json.dumps(metadata["controller_state"], ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    if args.controller in {"rbf", "fuzzy"}:
        np.savez_compressed(args.output / "controller_state.npz", **controller.state_arrays())
    # Snapshot nested links from their actual physical world poses. Author the
    # copy from parent to child, without modifying the running simulation.
    from .scene import export_snapshot
    bodies = RigidPrim(robot.link_paths[0])
    link_positions, link_quaternions = bodies.get_world_poses()
    metadata["snapshot_max_link_position_error_m"] = export_snapshot(
        stage, bodies.paths, link_positions.numpy(), link_quaternions.numpy(), args.output / "scene.usda")
    save_results(args.output, rows, metadata)
    if viewport:
        from omni.kit.viewport.utility import capture_viewport_to_file
        snapshot = args.output / "scene.png"
        snapshot.unlink(missing_ok=True)
        for _ in range(5):
            world.render()
        capture = capture_viewport_to_file(viewport, str(snapshot))
        deadline = time.monotonic() + 8
        while not snapshot.exists() and time.monotonic() < deadline:
            world.render()
    print(f"[raa-v0] 已保存：{args.output}", flush=True)
    drawing_rmse = metrics["drawing_tip_rmse_m"]
    error_label = f"{drawing_rmse*1000:.3f} mm" if drawing_rmse is not None else "尚未进入绘图段"
    print(f"[raa-v0] 绘图段三维 RMSE：{error_label}；完整绘制：{metadata['completed']}", flush=True)
    if not args.headless:
        end = time.monotonic() + args.hold_seconds
        while app.is_running() and time.monotonic() < end:
            q = robot.get_dof_positions(dof_indices=arm).numpy()[0]
            dq = robot.get_dof_velocities(dof_indices=arm).numpy()[0]
            gravity = robot.get_dof_gravity_compensation_forces(dof_indices=arm).numpy()[0]
            robot.set_dof_efforts(controller.compute(q, dq, q_ref, np.zeros(7), gravity,
                                                     dt=args.dt, adapt=False), dof_indices=arm)
            world.step(render=False)
            world.render()
    return 0
