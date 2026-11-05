"""Floating-base Isaac/PhysX simulation driven by the upstream C++ MPC/WBC."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]

# These capabilities are exercised by ol_step_jump: runtime z/vz references,
# a flight IK/PV branch that bypasses contact WBC, and bilateral estimated-force
# events consumed by JumpStateMachine.
JUMP_BRIDGE_CAPABILITIES = {
    "dynamic_base_height_reference": True,
    "flight_phase_control": True,
    "bilateral_contact_feedback": True,
}


class PhysicalInstabilityError(RuntimeError):
    """A finite physical state crossed the existing height/tilt stop limits."""

    def __init__(self, time_s, position, roll, pitch):
        self.state = {"time_s": float(time_s),
                      "base_position_m": [float(value) for value in position],
                      "roll_rad": float(roll), "pitch_rad": float(pitch)}
        super().__init__(f"t={time_s:.3f}s：机器人失稳，z={position[2]:.3f}m，"
                         f"roll={roll:.3f}，pitch={pitch:.3f}")


def _json(path, value):
    path.write_text(json.dumps(value, default=str, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def run(args, isaac_root):
    cache = ROOT / "cache/runtime"
    cache.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "run.json").exists():
        raise FileExistsError(f"输出目录已包含实验，请换一个 --output：{args.output}")
    _json(args.output / "config.json", vars(args))
    if args.jump is not None:
        from .jump_controller import jump_preflight
        report = jump_preflight(args.jump, JUMP_BRIDGE_CAPABILITIES, raise_on_missing=False)
        _json(args.output / "jump_preflight.json", report)
        if not report["ready"]:
            message = "跳跃预检未通过，当前 Isaac 控制桥缺少：" + "、".join(report["missing"])
            _json(args.output / "failure.json", {"error": message, "type": "UnsupportedJumpRuntime"})
            _json(args.output / "run.json", {"status": "preflight_failed", "experiment": "vertical_jump",
                                               "config": vars(args), "jump_preflight": report})
            print(f"[OpenLoong] {message}；未启动 Isaac，未生成伪跳跃结果。", flush=True)
            return 1
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    from isaacsim import SimulationApp
    extra = ["--portable-root", str(cache / "kit"),
             f"--/app/tokens/cache={cache / 'kit-cache'}", f"--/app/tokens/data={cache / 'kit-data'}",
             f"--/app/tokens/logs={cache / 'kit-logs'}", f"--/app/tokens/omni_global_cache={cache / 'omni'}",
             f"--/app/userConfigPath={cache / 'user.config.json'}", f"--/log/file={args.output / 'isaac.log'}"]
    app = SimulationApp({"headless": args.headless, "width": 960, "height": 720,
                         "renderer": "MinimalRendering", "minimal_shading_mode": 3,
                         "anti_aliasing": 0, "multi_gpu": False, "limit_cpu_threads": 8,
                         "extra_args": extra}, experience=str(Path(__file__).with_name("openloong.kit")))
    status = 0
    try:
        _run(args, isaac_root, app)
    except Exception as exc:
        traceback.print_exc()
        _json(args.output / "failure.json", {"error": str(exc), "type": type(exc).__name__})
        status = 1
    finally:
        app.close(exit_code=status)
    return status


class Keyboard:
    """Optional GUI controls. All commands are consumed on the physics clock."""
    def __init__(self, args):
        self.command = [args.vx, args.vy, args.wz]
        self.override = None
        self._subscription = None
        self._interface = None
        if args.headless:
            return
        import carb.input
        import omni.appwindow
        window = omni.appwindow.get_default_app_window()
        if window is None or window.get_keyboard() is None:
            return
        self._interface = carb.input.acquire_input_interface()
        self._keyboard = window.get_keyboard()
        def on_event(event, *_):
            if event.type != carb.input.KeyboardEventType.KEY_PRESS:
                return True
            key = event.input
            K = carb.input.KeyboardInput
            if key == K.SPACE:
                self.override = not bool(self.override if self.override is not None else args.mode == "walk")
                if not self.override:
                    self.command = [0.0, 0.0, 0.0]
            elif key == K.W:
                self.command[0] = max(.1, args.vx)
                self.override = True
            elif key == K.S:
                self.command[0] = 0.0
            elif key == K.A:
                self.command[2] = .15
            elif key == K.D:
                self.command[2] = -.15
            elif key == K.H:
                self.command[2] = 0.0
            return True
        self._callback = on_event
        self._subscription = self._interface.subscribe_to_keyboard_events(self._keyboard, on_event)

    def walking(self, t, args):
        return t >= args.walk_start and (self.override if self.override is not None else args.mode == "walk")

    def close(self):
        if self._subscription is not None:
            self._interface.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)


class ViewportVideoRecorder:
    """Capture simulation-time frames and stream them into Isaac's H.264 encoder."""
    def __init__(self, args, viewport):
        self.enabled = args.record_video
        self.viewport = viewport
        self.fps = args.video_fps
        self.target_frames = int(round(args.duration * self.fps))
        self.frame_count = 0
        self.path = args.output / "demo.mp4"
        self.frames_dir = args.output / "video_frames"
        self.saved = False
        self.termination_reason = None
        if not self.enabled:
            return
        if viewport is None:
            raise RuntimeError("没有可用视口，无法录制视频")
        if self.target_frames < 1:
            raise ValueError("视频至少需要一帧")
        self.frames_dir.mkdir(parents=True, exist_ok=False)

    def due(self, step, dt):
        if not self.enabled or self.frame_count >= self.target_frames:
            return False
        target_step = round(self.frame_count / (self.fps * dt))
        return step >= target_step

    def capture(self, world):
        import asyncio
        from omni.kit.viewport.utility import capture_viewport_to_file

        frame_path = self.frames_dir / f"frame.{self.frame_count:06d}.png"
        capture = capture_viewport_to_file(self.viewport, str(frame_path), is_hdr=False)
        future = asyncio.ensure_future(capture.wait_for_result(completion_frames=2))
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            world.render()
            if future.done() and frame_path.is_file() and frame_path.stat().st_size > 0:
                break
            time.sleep(.001)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"第{self.frame_count}帧视口捕获超时")
        if not future.result():
            raise RuntimeError(f"第{self.frame_count}帧视口捕获失败")
        if not frame_path.is_file() or frame_path.stat().st_size == 0:
            raise RuntimeError(f"第{self.frame_count}帧图片没有写入")
        self.frame_count += 1

    def finish(self, allow_partial=False, *, termination_reason=None):
        if not self.enabled or self.saved:
            return
        if self.frame_count < 1:
            raise RuntimeError("没有已完成的视频帧，不能编码")
        if allow_partial and not termination_reason:
            raise ValueError("部分录像必须注明提前终止原因")
        if self.frame_count != self.target_frames and not allow_partial:
            raise RuntimeError(f"视频帧数不完整：{self.frame_count}/{self.target_frames}")
        # Encode only the frames captured before the physical stop. No repeated
        # frames, padding, resumed dynamics, or synthetic recovery are added.
        self.termination_reason = termination_reason or "completed"
        from video_encoding import encode_image_file_sequence
        pattern = str(self.frames_dir / "frame.%06d.png")
        if not encode_image_file_sequence(pattern, 0, self.fps, str(self.path), True):
            raise RuntimeError("Isaac H.264编码器执行失败")
        self.saved = self.path.is_file() and self.path.stat().st_size > 0
        if not self.saved:
            raise RuntimeError("视频编码结束但demo.mp4未生成")

    def close(self):
        pass

    def manifest(self):
        if not self.enabled:
            return {}
        return {"video_requested": True, "video_saved": self.saved,
                "video_complete": self.saved and self.frame_count == self.target_frames,
                "video_planned_frames": self.target_frames,
                "video_planned_duration_s": self.target_frames / self.fps,
                "video_termination_reason": self.termination_reason,
                "video_path": str(self.path), "video_fps": self.fps,
                "video_frames": self.frame_count,
                "video_duration_s": self.frame_count / self.fps,
                "video_frame_directory": str(self.frames_dir)}


def _run(args, isaac_root, app):
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.utils.stage import create_new_stage, add_reference_to_stage
    from omni.kit.viewport.utility import get_active_viewport
    from .asset_builder import ensure_robot_usd
    from .model_metadata import read_model_metadata, initial_base_height
    from .adapter import joint_indices, base_origin_velocity, rpy_wxyz
    from .native import Controller
    from .jump_controller import JumpStateMachine, PHASE_CODES
    from .path_tracking import PathTracker, build_path_visualization
    from .terrain import build_terrain
    from .force_visualization import AppliedForceArrow

    external_runtime = None
    external_solver = None
    if args.mpc_backend == "nmpc_rti":
        from control.nmpc_rti import RtiNmpc
        from control.runtime_adapter import ExternalMpcRuntime
        from control.srbd_model import velocity_reference, predict_contact_schedule
        external_solver = RtiNmpc(
            dt=args.nmpc["prediction_dt_s"],
            horizon=args.nmpc["horizon_steps"],
            state_weights=args.nmpc["state_weights"],
            input_weight=args.nmpc["input_weight"],
            wrench_rate_weight=args.nmpc["wrench_rate_weight"],
            state_difference=args.nmpc["state_difference"],
            wrench_difference=args.nmpc["wrench_difference"],
        )
        external_runtime = ExternalMpcRuntime(
            external_solver, update_period_s=args.nmpc["update_period_s"])

    tracker = PathTracker(args.path_tracking) if args.path_tracking is not None else None
    jump_machine = JumpStateMachine(args.jump) if args.jump is not None else None

    asset_dir = ROOT / "cache/robot"
    print("[OpenLoong] 准备青龙浮动基 USD 模型……", flush=True)
    usd = ensure_robot_usd(args.repo_root, asset_dir, force=args.rebuild_asset)
    metadata = read_model_metadata(args.repo_root)
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    add_reference_to_stage(str(usd), "/World/OpenLoong")
    terrain_manifest = build_terrain(stage, args.terrain)
    path_visualization_manifest = (
        build_path_visualization(stage, tracker.reference_points)
        if tracker is not None else None
    )
    force_arrow = AppliedForceArrow(stage) if args.push_at is not None else None
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1000)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -30))
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr(28.0)
    camera_transform = camera.AddTransformOp()
    def point_camera(x, y):
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(x+3.0, y-4.0, 2.25),
                                        Gf.Vec3d(x, y, 1.0), Gf.Vec3d(0, 0, 1)).GetInverse()
        camera_transform.Set(matrix)
    point_camera(0, 0)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body.CreateDisableGravityAttr(False)
            body.CreateLinearDampingAttr(0.0)
            body.CreateAngularDampingAttr(0.0)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            articulation.CreateSolverPositionIterationCountAttr(16)
            articulation.CreateSolverVelocityIterationCountAttr(4)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(.002)
            collision.CreateRestOffsetAttr(0.0)

    robot = Articulation("/World/OpenLoong")
    world = World(physics_dt=args.dt, rendering_dt=args.dt, stage_units_in_meters=1.0, device="cpu")
    world.reset()
    viewport = get_active_viewport()
    if viewport:
        viewport.set_active_camera("/World/Camera")
    base_indices = robot.get_link_indices(metadata["base_link"]).list()
    if len(base_indices) != 1:
        raise RuntimeError("无法唯一确定浮动基连杆")
    base = RigidPrim(robot.link_paths[0][base_indices[0]])
    foot_prims = None
    jump_contact_feet = None
    if external_runtime is not None or jump_machine is not None:
        foot_names = ("Link_ankle_l_roll", "Link_ankle_r_roll")
        foot_indices = robot.get_link_indices(list(foot_names)).list()
        if len(foot_indices) != 2:
            raise RuntimeError("无法确定左右踝足端连杆")
        foot_paths = [str(robot.link_paths[0][index]) for index in foot_indices]
        if external_runtime is not None:
            foot_prims = tuple(RigidPrim(path) for path in foot_paths)
        if jump_machine is not None:
            # The jump configuration is restricted to flat terrain.  Query
            # per-foot PhysX forces against the actual ground rigid body for
            # takeoff/landing events; retain inverse-dynamics estimates as an
            # independent diagnostic channel.
            for path in foot_paths:
                contact_api = PhysxSchema.PhysxContactReportAPI.Apply(
                    stage.GetPrimAtPath(path))
                contact_api.CreateThresholdAttr().Set(0.0)
            jump_contact_feet = RigidPrim(
                foot_paths, contact_filter_paths="/World/Terrain/Ground",
                max_contact_count=32)
    com_positions, _ = robot.get_link_coms(link_indices=base_indices)
    com_offset = com_positions.numpy()[0, 0].astype(float)
    if tracker is not None:
        with (args.output / "reference_path.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["point_index", "x_m", "y_m"])
            writer.writerows((i, *point) for i, point in enumerate(tracker.reference_points))
    keyboard = Keyboard(args)
    manifest = {"status": "running", "isaac_version": (isaac_root/"VERSION").read_text().strip(),
                "robot_usd": str(usd), "feedback": "simulation_ground_truth",
                "base_velocity_reference": "base_link origin, world frame",
                "torque_units": "joint-side Nm", "physics_backend": "PhysX CPU",
                "com_offset_body_m": com_offset.tolist(), "config": vars(args),
                "terrain": terrain_manifest,
                "path_visualization": path_visualization_manifest,
                "force_visualization": force_arrow.manifest() if force_arrow is not None else None,
                "command_source": "path_tracking" if tracker is not None else "keyboard_or_constant",
                "mpc_backend": args.mpc_backend,
                "nmpc": args.nmpc if external_runtime is not None else None,
                "control_decomposition_logged": args.log_control_decomposition,
                "jump_control": ({
                    "algorithm": "upstream jump_mpc phase logic adapted to Isaac",
                    "phase_codes": PHASE_CODES,
                    "contact_source": "PhysX per-foot ground contact force",
                    "contact_cross_check": "OpenLoong inverse-dynamics foot-force estimate",
                    "takeoff_force_threshold_n": args.jump["takeoff_force_threshold_n"],
                    "landing_force_threshold_n": args.jump["landing_force_threshold_n"],
                    "contact_confirm_duration_s": args.jump["contact_confirm_duration_s"],
                    "landing_foot_advance_m": args.jump["landing_foot_advance_m"],
                    "landing_foot_swing_duration_s":
                        args.jump["landing_foot_swing_duration_s"],
                    "landing_vertical_velocity_threshold_m_s":
                        args.jump["landing_vertical_velocity_threshold_m_s"],
                    "landing_settle_timeout_s":
                        args.jump["landing_settle_timeout_s"],
                    "landing_transition":
                        "source landing MPC absorbs the bilateral impact; after vertical "
                        "velocity settles, double-support WBC keeps both feet fixed and "
                        "raises the body, then the recovery WBC continues the steady "
                        "double-support hold in the complete phase",
                    "external_force_assist": False,
                } if jump_machine is not None else None)}
    revision = subprocess.run(["git", "-C", str(args.repo_root), "rev-parse", "HEAD"], capture_output=True, text=True)
    manifest["upstream_commit"] = revision.stdout.strip()
    _json(args.output / "run.json", manifest)
    start = time.monotonic()
    samples = 0
    steps_done = 0
    max_tilt = 0.0
    min_height = math.inf
    max_torque = 0.0
    timing = dict(read_state=0.0, external_mpc=0.0, controller=0.0,
                  physics=0.0, render=0.0)
    external_metrics = {
        "updates": 0,
        "successful_updates": 0,
        "failed_updates": 0,
        "deadline_misses": 0,
        "solve_time_sum_s": 0.0,
        "solve_time_max_s": 0.0,
    }
    path_metrics = {"samples": 0, "squared_cross_track_error_m2": 0.0,
                    "maximum_path_distance_m": 0.0, "squared_heading_error_rad2": 0.0}
    jump_metrics = None
    last_jump_phase = None
    if jump_machine is not None:
        jump_metrics = {
            "phase_sequence": [],
            "takeoff_time_s": None,
            "touchdown_time_s": None,
            "takeoff_base_height_m": None,
            "touchdown_base_height_m": None,
            "minimum_crouch_height_m": math.inf,
            "apex_height_m": -math.inf,
            "maximum_upward_velocity_m_s": -math.inf,
            "maximum_left_contact_force_n": 0.0,
            "maximum_right_contact_force_n": 0.0,
            "flight_mpc_disabled": True,
            "minimum_required_rise_m": max(0.03, 0.5 * args.jump["target_height_m"]),
            "external_force_assist": False,
        }
    last_position = None
    error = None
    video = None
    try:
        video = ViewportVideoRecorder(args, viewport)
        with Controller(args.repo_root, dt=args.dt, mpc_dt=args.mpc_dt,
                        mpc_horizon_steps=args.mpc_horizon_steps, swing_time=args.swing_time,
                        step_height=args.step_height, stand_height=args.stand_height) as controller:
            library_bytes = controller.library_path.read_bytes()
            manifest.update(
                controller_library=str(controller.library_path),
                controller_library_sha256=hashlib.sha256(library_bytes).hexdigest(),
                mpc_horizon_steps=args.mpc_horizon_steps,
                mpc_prediction_horizon_s=args.mpc_horizon_steps * args.mpc_dt,
                mpc_control_horizon_steps=3,
                mpc_control_horizon_s=3 * args.mpc_dt,
            )
            if jump_machine is not None:
                manifest["jump_mpc_prediction_horizon_s"] = (
                    args.mpc_horizon_steps * args.dt)
            controller.set_weights(args.u_weight, args.state_weights, args.input_weights)
            indices = joint_indices(controller.joint_names, robot.dof_names)
            initial = controller.initial_joints()
            height = initial_base_height(args.repo_root, dict(zip(controller.joint_names, initial)))
            robot.set_dof_gains(stiffnesses=0.0, dampings=0.0)
            robot.set_dof_positions(initial, dof_indices=indices)
            robot.set_dof_velocities(np.zeros(len(indices)), dof_indices=indices)
            robot.set_world_poses(positions=np.array([[0.0, 0.0, height]]), orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            robot.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
            robot.set_dof_efforts(np.zeros(len(indices)), dof_indices=indices)
            manifest.update(joint_names=controller.joint_names, joint_indices=indices,
                            initial_joint_positions_rad=initial, initial_base_height_m=height)
            last_diagnostics = controller.diagnostics()
            external_enabled = False
            _json(args.output / "run.json", manifest)
            if video.enabled:
                # Flush the loading frame after the initial pose and active camera are set,
                # without advancing PhysX simulation time.
                point_camera(0.0, 0.0)
                for _ in range(3):
                    world.render()
            activity = "跳跃" if jump_machine is not None else args.mode
            print(f"[OpenLoong] {len(indices)}关节，初始基座高度{height:.4f}m；开始{activity}，{args.duration:g}秒仿真", flush=True)
            if jump_machine is not None:
                print("[OpenLoong] 跳跃控制：WBC稳定下蹲、蹬伸MPC、腾空收腿/前摆IK、PhysX双脚接触判定。", flush=True)
            elif tracker is None:
                print("[OpenLoong] GUI键盘：W前进，S减速到零，A/D转向，H停止转向，空格切换站立/行走", flush=True)
            else:
                print("[OpenLoong] L4路径跟踪已接管速度指令；地面青色参考线、绿色起点、黄色途经点和红色终点已显示；空格仍可暂停/恢复行走。", flush=True)
            fields = ["time_s", "base_x_m", "base_y_m", "base_z_m", "roll_rad", "pitch_rad", "yaw_rad",
                      "vx_world_m_s", "vy_world_m_s", "vz_world_m_s", "cmd_vx_body_m_s", "cmd_vy_body_m_s", "cmd_wz_rad_s",
                      "walking_command", "force_x_n", "force_y_n", "force_z_n"]
            fields += [prefix+name for prefix in ("q_rad:", "dq_rad_s:", "tau_nm:") for name in controller.joint_names]
            fields += ["controller:"+key for key in controller.diagnostic_names]
            if jump_machine is not None:
                fields += ["jump:phase", "jump:phase_code", "jump:height_reference_m",
                           "jump:vertical_velocity_reference_m_s", "jump:phase_progress",
                           "jump:landing_foot_advance_m",
                           "jump:decision_left_contact_force_z_n",
                           "jump:decision_right_contact_force_z_n",
                           "jump:measured_left_contact_force_x_n",
                           "jump:measured_left_contact_force_y_n",
                           "jump:measured_right_contact_force_x_n",
                           "jump:measured_right_contact_force_y_n"]
            if tracker is not None:
                fields += ["path:" + key for key in tracker.diagnostic_names]
            if args.log_control_decomposition:
                fields += ["control:" + key for key in controller.control_signal_names]
                fields += ["isaac_applied_Nm:" + name for name in controller.joint_names]
                fields += ["isaac_projected_Nm:" + name for name in controller.joint_names]
            render_every = max(1, round(1.0 / (args.render_hz * args.dt)))
            log_every = max(1, round(1.0 / (args.log_hz * args.dt)))
            count = math.ceil(args.duration / args.dt)
            with (args.output / "states.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                for step in range(count):
                    if not app.is_running():
                        raise RuntimeError("用户在实验完成前关闭了仿真窗口")
                    while not world.is_playing():
                        if not app.is_running():
                            raise RuntimeError("用户在实验完成前关闭了仿真窗口")
                        if world.is_stopped():
                            raise RuntimeError("仿真时间线已停止；请重新启动程序以重置控制器")
                        if force_arrow is not None:
                            force_arrow.hide()
                        world.render()
                    t = step * args.dt
                    mark = time.perf_counter()
                    q = robot.get_dof_positions(dof_indices=indices).numpy()[0].astype(float)
                    dq = robot.get_dof_velocities(dof_indices=indices).numpy()[0].astype(float)
                    positions, quats = robot.get_world_poses()
                    position, quaternion = positions.numpy()[0].astype(float), quats.numpy()[0].astype(float)
                    linear, angular = robot.get_velocities()
                    angular = angular.numpy()[0].astype(float)
                    linear = base_origin_velocity(linear.numpy()[0], angular, quaternion, com_offset)
                    roll, pitch, yaw = rpy_wxyz(quaternion)
                    if not np.all(np.isfinite(np.concatenate([q,dq,position,quaternion,linear,angular]))):
                        raise RuntimeError(f"t={t:.3f}s：物理状态出现非有限值")
                    if position[2] < .60 or max(abs(roll), abs(pitch)) > 1.0:
                        raise PhysicalInstabilityError(t, position, roll, pitch)
                    timing["read_state"] += time.perf_counter() - mark
                    mark = time.perf_counter()
                    jump_reference = None
                    jump_decision_forces = None
                    jump_tangent_forces = None
                    if jump_machine is not None:
                        contact_forces = jump_contact_feet.get_net_contact_forces(
                            dt=args.dt).numpy()
                        if contact_forces.shape != (2, 3) or not np.all(np.isfinite(contact_forces)):
                            raise RuntimeError("PhysX左右脚接触力返回无效数据")
                        jump_decision_forces = tuple(
                            max(0.0, float(value)) for value in contact_forces[:, 2])
                        jump_tangent_forces = (
                            float(contact_forces[0, 0]),
                            float(contact_forces[0, 1]),
                            float(contact_forces[1, 0]),
                            float(contact_forces[1, 1]),
                        )
                        jump_metrics["maximum_left_contact_force_n"] = max(
                            jump_metrics["maximum_left_contact_force_n"],
                            jump_decision_forces[0])
                        jump_metrics["maximum_right_contact_force_n"] = max(
                            jump_metrics["maximum_right_contact_force_n"],
                            jump_decision_forces[1])
                        jump_reference = jump_machine.update(
                            t, float(position[2]), float(linear[2]),
                            *jump_decision_forces)
                        if jump_reference.phase != last_jump_phase:
                            jump_metrics["phase_sequence"].append({
                                "phase": jump_reference.phase,
                                "time_s": t,
                            })
                            last_jump_phase = jump_reference.phase
                            print(f"[OpenLoong] 跳跃阶段：{jump_reference.phase}，t={t:.3f}s", flush=True)
                        if jump_machine.takeoff_time_s is not None and \
                                jump_metrics["takeoff_time_s"] is None:
                            jump_metrics["takeoff_time_s"] = jump_machine.takeoff_time_s
                            jump_metrics["takeoff_base_height_m"] = float(position[2])
                        if jump_machine.touchdown_time_s is not None and \
                                jump_metrics["touchdown_time_s"] is None:
                            jump_metrics["touchdown_time_s"] = jump_machine.touchdown_time_s
                            jump_metrics["touchdown_base_height_m"] = float(position[2])
                        if jump_reference.phase in {"crouch", "thrust"}:
                            jump_metrics["minimum_crouch_height_m"] = min(
                                jump_metrics["minimum_crouch_height_m"], float(position[2]))
                        if jump_machine.left_ground and jump_machine.touchdown_time_s is None:
                            jump_metrics["apex_height_m"] = max(
                                jump_metrics["apex_height_m"], float(position[2]))
                        jump_metrics["maximum_upward_velocity_m_s"] = max(
                            jump_metrics["maximum_upward_velocity_m_s"], float(linear[2]))
                    walking = False if jump_machine is not None else keyboard.walking(t, args)
                    path_diagnostics = None
                    if jump_machine is not None:
                        command = [0.0, 0.0, 0.0]
                    elif tracker is not None:
                        if walking:
                            command, path_diagnostics = tracker.update(position[:2], yaw, args.dt)
                            # ``stop_at_goal`` means a zero velocity reference.
                            # Keep the gait scheduler active after the command
                            # ramps down: OpenLoong's Walk2Stand transition can
                            # be unstable when a multi-turn path ends with a
                            # different stance/yaw, whereas zero-command walking
                            # safely steps in place and preserves the reached
                            # terminal state.
                        else:
                            command = tracker.stop(args.dt)
                            path_diagnostics = tracker.last_diagnostics
                    else:
                        command = keyboard.command if walking else [0.0, 0.0, 0.0]
                    if external_runtime is not None:
                        if walking and bool(last_diagnostics["ready"]):
                            external_mark = time.perf_counter()
                            feet_world = np.vstack([
                                foot.get_world_poses()[0].numpy()[0].astype(float)
                                for foot in foot_prims
                            ])
                            state12 = np.asarray([
                                roll, pitch, yaw, *position, *angular, *linear
                            ], dtype=float)
                            ramped_command = np.asarray([
                                last_diagnostics["command_vx_mps"],
                                last_diagnostics["command_vy_mps"],
                                last_diagnostics["command_wz_radps"],
                            ], dtype=float)
                            reference = velocity_reference(
                                state12, ramped_command,
                                dt=external_solver.dt,
                                horizon=external_solver.horizon,
                                stand_height=args.stand_height,
                            )
                            contacts = predict_contact_schedule(
                                last_diagnostics["leg_state"],
                                last_diagnostics["next_leg_state"],
                                last_diagnostics["gait_phase"],
                                dt=external_solver.dt,
                                horizon=external_solver.horizon,
                                swing_time=args.swing_time,
                            )
                            update = external_runtime.update(
                                controller, t, state12, reference,
                                feet_world, contacts)
                            external_enabled = True
                            if update.updated:
                                external_metrics["updates"] += 1
                                key = ("successful_updates" if update.success
                                       else "failed_updates")
                                external_metrics[key] += 1
                                external_metrics["solve_time_sum_s"] += update.solve_time_s
                                external_metrics["solve_time_max_s"] = max(
                                    external_metrics["solve_time_max_s"], update.solve_time_s)
                                if update.solve_time_s > args.nmpc["update_period_s"]:
                                    external_metrics["deadline_misses"] += 1
                            timing["external_mpc"] += time.perf_counter() - external_mark
                        elif external_enabled:
                            external_runtime.disable(controller)
                            external_enabled = False
                    mark = time.perf_counter()
                    if jump_reference is not None:
                        torque = controller.step_jump(
                            t, q, dq, position, quaternion, linear, angular,
                            jump_reference)
                    else:
                        torque = controller.step(
                            t, q, dq, position, quaternion, linear, angular,
                            command, walking)
                    timing["controller"] += time.perf_counter() - mark
                    force = np.zeros(3)
                    if args.push_at is not None and args.push_at <= t < args.push_at + args.push_duration:
                        force = np.asarray(args.push_force)
                    if force_arrow is not None and not np.any(force):
                        force_arrow.hide()
                    robot.set_dof_efforts(torque, dof_indices=indices)
                    diagnostics = controller.diagnostics()
                    last_diagnostics = diagnostics
                    if step % log_every == 0:
                        row = [t,*position,roll,pitch,yaw,*linear,*command,int(walking),*force,
                               *q,*dq,*torque,*[diagnostics[k] for k in controller.diagnostic_names]]
                        if jump_reference is not None:
                            row.extend([
                                jump_reference.phase, jump_reference.phase_code,
                                jump_reference.base_height_m,
                                jump_reference.vertical_velocity_m_s,
                                jump_reference.phase_progress,
                                jump_reference.landing_foot_advance_m,
                                *jump_decision_forces,
                                *jump_tangent_forces,
                            ])
                        if tracker is not None:
                            row.extend(path_diagnostics[k] for k in tracker.diagnostic_names)
                            if walking:
                                distance = path_diagnostics["path_distance_m"]
                                heading_error = path_diagnostics["heading_error_rad"]
                                path_metrics["samples"] += 1
                                path_metrics["squared_cross_track_error_m2"] += distance * distance
                                path_metrics["maximum_path_distance_m"] = max(
                                    path_metrics["maximum_path_distance_m"], distance)
                                path_metrics["squared_heading_error_rad2"] += heading_error * heading_error
                        if jump_reference is not None and \
                                jump_reference.phase in {"flight_up", "flight_down"} and \
                                diagnostics["mpc_enabled"] != 0.0:
                            jump_metrics["flight_mpc_disabled"] = False
                        if args.log_control_decomposition:
                            signals = controller.control_signals()
                            applied = robot.get_dof_efforts(
                                dof_indices=indices).numpy()[0].astype(float)
                            projected = robot.get_dof_projected_joint_forces(
                                dof_indices=indices).numpy()[0].astype(float)
                            row.extend(signals[key] for key in controller.control_signal_names)
                            row.extend(applied)
                            row.extend(projected)
                        writer.writerow(row)
                        samples += 1
                    mark = time.perf_counter()
                    if np.any(force):
                        base.apply_forces(force.reshape(1,3), local_frame=False)
                    world.step(render=False)
                    timing["physics"] += time.perf_counter() - mark
                    steps_done = step + 1
                    last_position = position.tolist()
                    max_tilt = max(max_tilt, abs(roll), abs(pitch))
                    min_height = min(min_height, float(position[2]))
                    max_torque = max(max_torque, max(abs(v) for v in torque))
                    if video.due(step, args.dt):
                        mark = time.perf_counter()
                        point_camera(float(position[0]), float(position[1]))
                        if force_arrow is not None and np.any(force):
                            force_arrow.update(base.get_world_poses()[0].numpy()[0], force)
                        video.capture(world)
                        timing["render"] += time.perf_counter() - mark
                    elif step % render_every == 0:
                        mark = time.perf_counter()
                        point_camera(float(position[0]), float(position[1]))
                        if force_arrow is not None and np.any(force):
                            force_arrow.update(base.get_world_poses()[0].numpy()[0], force)
                        world.render()
                        timing["render"] += time.perf_counter() - mark
                    if step % max(1, round(1.0 / args.dt)) == 0:
                        print(f"[OpenLoong] t={t:.1f}s base=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
                              f"rpy=({roll:.3f},{pitch:.3f},{yaw:.3f}) tau_max={max(abs(v) for v in torque):.1f}Nm", flush=True)
                        stream.flush()
            if jump_machine is not None:
                if not jump_machine.complete:
                    raise RuntimeError(
                        f"跳跃仿真结束时仍处于 {jump_machine.phase} 阶段，未完成恢复")
                rise = (jump_metrics["apex_height_m"] -
                        jump_metrics["takeoff_base_height_m"])
                if not math.isfinite(rise) or rise < jump_metrics["minimum_required_rise_m"]:
                    raise RuntimeError(
                        f"跳跃高度不足：实际{rise:.3f}m，要求至少"
                        f"{jump_metrics['minimum_required_rise_m']:.3f}m")
                if not jump_metrics["flight_mpc_disabled"]:
                    raise RuntimeError("腾空阶段检测到接触力MPC仍开启")
            robot.set_dof_efforts(np.zeros(len(indices)), dof_indices=indices)
            if force_arrow is not None:
                force_arrow.hide()
            video.finish()
            final_position = robot.get_world_poses()[0].numpy()[0].astype(float)
            last_position = final_position.tolist()
            point_camera(float(final_position[0]), float(final_position[1]))
            if viewport:
                import asyncio
                from omni.kit.viewport.utility import capture_viewport_to_file
                capture = capture_viewport_to_file(viewport, str(args.output / "final_frame.png"))
                future = asyncio.ensure_future(capture.wait_for_result(completion_frames=30))
                capture_deadline = time.monotonic() + 5.0
                while time.monotonic() < capture_deadline:
                    world.render()
                    if future.done() and (args.output / "final_frame.png").is_file():
                        break
                    time.sleep(.01)
                if future.done():
                    future.result()
                else:
                    future.cancel()
                manifest["screenshot_saved"] = (args.output / "final_frame.png").is_file()
            stage.GetRootLayer().Export(str(args.output / "scene.usda"))
            manifest["status"] = "completed"
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["error"] = str(exc)
        error = exc
        try:
            robot.set_dof_efforts(0.0)
        except Exception:
            pass
        if isinstance(exc, PhysicalInstabilityError):
            manifest["termination_reason"] = "physical_instability"
            manifest["termination_state"] = exc.state
            if video is not None and video.enabled and video.frame_count > 0:
                try:
                    video.finish(allow_partial=True, termination_reason=str(exc))
                    print(f"[OpenLoong] 保留失稳前真实录像：{video.frame_count}帧，"
                          f"{video.frame_count / video.fps:.3f}秒；实验状态仍为 failed。", flush=True)
                except Exception as video_error:
                    # Preserve the original physics failure even if encoding
                    # itself fails; never label a failed encoding as saved.
                    manifest["video_finalization_error"] = str(video_error)
    finally:
        if force_arrow is not None:
            force_arrow.hide()
        keyboard.close()
        if video is not None:
            video.close()
            manifest.update(video.manifest())
        if tracker is not None:
            sample_count = path_metrics["samples"]
            summary = {
                "sample_count": sample_count,
                "cross_track_rmse_m": math.sqrt(path_metrics["squared_cross_track_error_m2"] / sample_count)
                    if sample_count else None,
                "maximum_path_distance_m": path_metrics["maximum_path_distance_m"] if sample_count else None,
                "heading_error_rmse_rad": math.sqrt(path_metrics["squared_heading_error_rad2"] / sample_count)
                    if sample_count else None,
                "final_progress_m": tracker.progress_m,
                "path_length_m": tracker.path.total_length,
                "completion_fraction": tracker.progress_m / tracker.path.total_length,
                "reached_goal": tracker.reached_goal,
                "final_goal_distance_m": tracker.last_diagnostics["goal_distance_m"],
            }
            _json(args.output / "path_metrics.json", summary)
            manifest["path_tracking"] = summary
        if external_runtime is not None:
            updates = external_metrics["updates"]
            external_metrics["solve_time_mean_s"] = (
                external_metrics["solve_time_sum_s"] / updates if updates else None)
            external_metrics["success_fraction"] = (
                external_metrics["successful_updates"] / updates if updates else None)
            external_metrics["deadline_miss_fraction"] = (
                external_metrics["deadline_misses"] / updates if updates else None)
            _json(args.output / "nmpc_metrics.json", external_metrics)
            manifest["external_nmpc"] = external_metrics
        if jump_machine is not None:
            takeoff_height = jump_metrics["takeoff_base_height_m"]
            apex_height = jump_metrics["apex_height_m"]
            actual_rise = (apex_height - takeoff_height
                           if takeoff_height is not None and math.isfinite(apex_height)
                           else None)
            final_height_error = (
                abs(last_position[2] - args.jump["stand_height_m"])
                if last_position is not None else None)
            jump_metrics.update({
                "completed_phase_sequence": jump_machine.complete,
                "final_phase": jump_machine.phase,
                "failed_reason": jump_machine.failed_reason,
                "actual_rise_m": actual_rise,
                "flight_duration_s": (
                    jump_metrics["touchdown_time_s"] - jump_metrics["takeoff_time_s"]
                    if jump_metrics["takeoff_time_s"] is not None and
                    jump_metrics["touchdown_time_s"] is not None else None),
                "final_base_height_m": last_position[2] if last_position is not None else None,
                "final_height_error_m": final_height_error,
                "maximum_tilt_rad": max_tilt,
                "maximum_joint_torque_nm": max_torque,
                "pass": (
                    manifest.get("status") == "completed" and jump_machine.complete and
                    jump_metrics["takeoff_time_s"] is not None and
                    jump_metrics["touchdown_time_s"] is not None and
                    actual_rise is not None and
                    actual_rise >= jump_metrics["minimum_required_rise_m"] and
                    jump_metrics["flight_mpc_disabled"] and
                    final_height_error is not None and final_height_error <= 0.12 and
                    max_tilt <= 0.35
                ),
            })
            for key in ("minimum_crouch_height_m", "apex_height_m",
                        "maximum_upward_velocity_m_s"):
                if not math.isfinite(jump_metrics[key]):
                    jump_metrics[key] = None
            _json(args.output / "jump_metrics.json", jump_metrics)
            manifest["jump"] = jump_metrics
        manifest.update(steps=steps_done, samples=samples, simulation_duration_s=steps_done*args.dt,
                        wall_time_s=time.monotonic()-start, final_base_position_m=last_position,
                        maximum_tilt_rad=max_tilt, minimum_base_height_m=min_height if math.isfinite(min_height) else None,
                        maximum_joint_torque_nm=max_torque, loop_wall_time_by_phase_s=timing)
        _json(args.output / "run.json", manifest)
    if error:
        raise error
    print(f"[OpenLoong] 完成，数据：{args.output}", flush=True)
