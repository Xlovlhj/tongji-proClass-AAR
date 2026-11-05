"""Offline, read-only diagnostics for the assignment presentation.

Run with an existing NumPy environment; this does not launch Isaac, change
controller code, regenerate experiments, or change any recorded experiment.
The sole output is assignment_diagnostics.json beside this script.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
ROOT = PROJECT.parent
ISAAC = Path("/home/xqw/isaacsim6.0.1")
CONTROLLERS = ("pd", "gc-pd", "rbf", "fuzzy")
CONDITIONS = ("normal", "disturbed")
PROVENANCE = {}


def source(path):
    path = Path(path)
    key = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    PROVENANCE[key] = {"path": key, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                       "bytes": path.stat().st_size}
    return path


def read_json(path):
    return json.loads(source(path).read_text(encoding="utf-8"))


def rpy_rotation(rpy):
    r, p, y = rpy
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def urdf_chain(urdf):
    tree = ET.parse(source(urdf))
    joints = {joint.find("child").get("link"): joint for joint in tree.getroot().findall("joint")}
    chain = []
    child = "panda_hand"
    while child in joints:
        joint = joints[child]
        chain.append(joint)
        child = joint.find("parent").get("link")
    return chain[::-1]


def forward_batch(urdf, q):
    transforms = np.tile(np.eye(4), (len(q), 1, 1))
    names = [f"panda_joint{i}" for i in range(1, 8)]
    for joint in urdf_chain(urdf):
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        transform = np.eye(4)
        transform[:3, :3] = rpy_rotation(rpy)
        transform[:3, 3] = xyz
        transforms = transforms @ transform
        if joint.get("type") == "revolute":
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            angle = q[:, names.index(joint.get("name"))]
            rotated = np.tile(np.eye(4), (len(q), 1, 1))
            rotated[:, :3, :3] = (np.eye(3) + np.sin(angle)[:, None, None]*skew
                                      + (1-np.cos(angle))[:, None, None]*(skew @ skew))
            transforms = transforms @ rotated
    return transforms


def reference_diagnostics(metrics):
    path = source(PROJECT / "output/normal/gc-pd/reference_joints.npz")
    with np.load(path) as saved:
        time, q, dq, tip = (saved[k] for k in ("time", "q", "dq", "tip"))
    planner_urdf = ISAAC / "extsDeprecated/isaacsim.robot_motion.motion_generation/motion_policy_configs/franka/lula_franka_gen.urdf"
    physical_urdf = ISAAC / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
    transforms = forward_batch(planner_urdf, q)
    rotations = transforms[:, :3, :3]
    fk_tip = transforms[:, :3, 3] + rotations @ np.array([0., 0., .13])
    position_error = np.linalg.norm(fk_tip-tip, axis=1)
    orientation_angle = np.arccos(np.clip((np.einsum("nii->n", np.swapaxes(rotations, 1, 2) @ np.diag([-1., 1., -1.]))-1)/2, -1., 1.))
    physical_fk = forward_batch(physical_urdf, q)
    physical_tip = physical_fk[:, :3, 3] + physical_fk[:, :3, :3] @ np.array([0., 0., .13])
    joint_defs = {j.get("name"): j for j in ET.parse(physical_urdf).getroot().findall("joint")}
    limits = [{"joint": f"panda_joint{i}", **{k: float(v) for k, v in joint_defs[f"panda_joint{i}"].find("limit").attrib.items()}}
              for i in range(1, 8)]
    h = np.diff(time)[:, None]
    acceleration_start = (-6*q[:-1]+6*q[1:]-4*h*dq[:-1]-2*h*dq[1:])/(h*h)
    acceleration_end = (6*q[:-1]-6*q[1:]+2*h*dq[:-1]+4*h*dq[1:])/(h*h)
    jumps = np.abs(np.diff(q, axis=0))
    dt = metrics["physics_dt_s"]
    sample_times = np.arange(metrics["metrics"]["sample_count"])*dt
    index = np.clip(np.searchsorted(time, sample_times, side="right")-1, 0, len(time)-2)
    interval = time[index+1]-time[index]
    u = np.clip((sample_times-time[index])/interval, 0, 1)[:, None]
    interval = interval[:, None]
    q0, q1, m0, m1 = q[index], q[index+1], dq[index], dq[index+1]
    qq = ((2*u**3-3*u**2+1)*q0+(u**3-2*u**2+u)*interval*m0
          +(-2*u**3+3*u**2)*q1+(u**3-u**2)*interval*m1)
    velocity = ((6*u**2-6*u)*q0+(3*u**2-4*u+1)*interval*m0
                +(-6*u**2+6*u)*q1+(3*u**2-2*u)*interval*m1)/interval
    acceleration = ((12*u-6)*q0+(6*u-4)*interval*m0+(-12*u+6)*q1+(6*u-2)*interval*m1)/(interval*interval)
    reconstructed = np.gradient(q, time, axis=0)
    reconstructed[[0, -1]] = 0
    return {
        "planning_sample_count": len(time), "planning_dt_s": float(time[1]-time[0]),
        "source": str(path.relative_to(ROOT)), "position_residual_definition": "Euclidean norm of URDF FK virtual-tip position minus saved Cartesian IK target at each planning knot",
        "orientation_residual_definition": "acos((trace(R_hand.T @ diag(-1,1,-1))-1)/2), rad; recomputed from the exact saved planner URDF and q knots",
        "reported_max_ik_tip_error_m": metrics["max_ik_tip_error_m"],
        "recomputed_max_ik_tip_error_m": float(position_error.max()),
        "recomputed_max_orientation_error_rad": float(orientation_angle.max()),
        "recomputed_max_orientation_error_deg": float(np.degrees(orientation_angle).max()),
        "planner_vs_physical_fk_max_tip_difference_m": float(np.linalg.norm(physical_tip-fk_tip, axis=1).max()),
        "ik_acceptance_position_m": 5e-5, "ik_acceptance_orientation_rad": 5e-4,
        "local_iteration_stop_position_m": 2e-6, "local_iteration_stop_orientation_rad": 2e-5,
        "max_adjacent_planning_joint_jump_rad": float(jumps.max()),
        "max_adjacent_planning_joint_jump_deg": float(np.degrees(jumps.max())),
        "max_adjacent_planning_joint_jump_per_joint_rad": jumps.max(axis=0).tolist(),
        "reference_knot_q_min_rad": q.min(axis=0).tolist(), "reference_knot_q_max_rad": q.max(axis=0).tolist(),
        "interpolated_q_min_rad": qq.min(axis=0).tolist(), "interpolated_q_max_rad": qq.max(axis=0).tolist(),
        "saved_dq_matches_gradient_with_zero_endpoints": bool(np.array_equal(dq, reconstructed)),
        "reference_peak_abs_velocity_rad_s_per_joint": np.abs(velocity).max(axis=0).tolist(),
        "reference_peak_abs_acceleration_rad_s2_per_joint": np.abs(acceleration).max(axis=0).tolist(),
        "max_acceleration_jump_at_internal_hermite_knots_rad_s2": float(np.abs(acceleration_end[:-1]-acceleration_start[1:]).max()),
        "joint_limits": limits,
        "ik_algorithm": "First pose Lula IK; subsequent poses warm-started at preceding q and solved using finite-difference 6x7 spatial Jacobian with damped least squares, bounded steps, backtracking, joint-limit clipping; no independent global branch reselection",
        "trajectory_algorithm": "SVG outline polylines; eight pen-down contours with 0.025 m lifts; quintic arc-length timing per motion section; 60 Hz Cartesian positions; continuous local IK; knot dq=np.gradient(q,time) with endpoints zero; cubic Hermite interpolation to 1 kHz q_ref and its analytic dq_ref",
        "hermite_continuity": "q_ref and dq_ref continuous at knots (C1). qdd_ref is generally discontinuous at knots (not C2); diagnostic qdd is analytic only and is not used by current controllers.",
        "q_ref_formula": "(2u^3-3u^2+1)q0+(u^3-2u^2+u)h*m0+(-2u^3+3u^2)q1+(u^3-u^2)h*m1",
        "dq_ref_formula": "[(6u^2-6u)q0+(3u^2-4u+1)h*m0+(-6u^2+6u)q1+(3u^2-2u)h*m1]/h",
        "qdd_ref_formula_diagnostic_only": "[(12u-6)q0+(6u-4)h*m0+(-12u+6)q1+(6u-2)h*m1]/h^2",
    }


def run_diagnostics(controller, condition):
    directory = PROJECT / "output" / condition / controller
    metric = read_json(directory / "metrics.json")
    config = read_json(directory / "resolved_config.json")
    path = source(directory / "trajectory.csv")
    tau_min = [math.inf]*7
    tau_max = [-math.inf]*7
    tau_sum_sq = [0.]*7
    peak = {"full": {"error_m": -1}, "pen_down": {"error_m": -1}, "first_force": {"error_m": -1}, "first_force_and_recovery": {"error_m": -1}}
    traces = {"time_s": [], "tau4_nm": [], "tracking_error_mm": [], "pen_down": []}
    all_sq, pen_sq, all_count, pen_count = 0., 0., 0, 0
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.reader(stream)
        header = next(rows)
        indices = {name: header.index(name) for name in ["time", "pen_down", "actual_x", "actual_y", "actual_z", "target_x", "target_y", "target_z", *[f"tau{i}" for i in range(1, 8)]]}
        for sample_index, row in enumerate(rows):
            t = float(row[indices["time"]])
            down = row[indices["pen_down"]].lower() in ("true", "1")
            err = math.sqrt(sum((float(row[indices[f"actual_{axis}"]])-float(row[indices[f"target_{axis}"]]))**2 for axis in "xyz"))
            all_sq += err*err
            all_count += 1
            if down:
                pen_sq += err*err
                pen_count += 1
            active = ["full"] + (["pen_down"] if down else [])
            if 20 <= t < 22:
                active.append("first_force")
            if 19 <= t < 27:
                active.append("first_force_and_recovery")
            for key in active:
                if err > peak[key]["error_m"]:
                    peak[key] = {"error_m": err, "time_s": t, "pen_down": down}
            torques = [float(row[indices[f"tau{i}"]]) for i in range(1, 8)]
            for j, tau in enumerate(torques):
                tau_min[j] = min(tau_min[j], tau)
                tau_max[j] = max(tau_max[j], tau)
                tau_sum_sq[j] += tau*tau
            if 19 <= t < 27 and sample_index % 10 == 0:
                traces["time_s"].append(t)
                traces["tau4_nm"].append(torques[3])
                traces["tracking_error_mm"].append(err*1000)
                traces["pen_down"].append(down)
    assert all_count == metric["metrics"]["sample_count"]
    assert pen_count == metric["metrics"]["drawing_sample_count"]
    assert math.isclose(math.sqrt(all_sq/all_count), metric["metrics"]["tip_rmse_m"], abs_tol=1e-12)
    assert math.isclose(math.sqrt(pen_sq/pen_count), metric["metrics"]["drawing_tip_rmse_m"], abs_tol=1e-12)
    assert len(traces["time_s"]) == 800
    return {"controller": controller, "condition": condition, "raw_csv": str(path.relative_to(ROOT)),
            "sample_count": all_count, "drawing_sample_count": pen_count, "metrics": metric["metrics"],
            "peak_tracking_errors": peak,
            "joint_torque_min_nm": tau_min, "joint_torque_max_nm": tau_max,
            "joint_torque_rms_nm": [math.sqrt(x/all_count) for x in tau_sum_sq],
            "torque_saturation_fraction": metric["metrics"]["torque_saturation_fraction"],
            "chart_traces": traces, "config": config,
            "gravity_enabled": metric["gravity_enabled"], "position_drives_enabled": metric["arm_position_drives_enabled"]}


def comparison_diagnostics(controller):
    directory = PROJECT / "output/comparisons" / controller
    recorded = read_json(directory / "comparison_metrics.json")
    path = source(directory / "comparison.csv")
    peak = {"error_m": -1}
    first_peak = {"error_m": -1}
    trace = {"time_s": [], "disturbance_deviation_mm": []}
    before_force_zero = True
    with path.open(newline="", encoding="utf-8") as stream:
        rows = csv.reader(stream)
        header = next(rows)
        ti, di = header.index("time"), header.index("disturbance_deviation_m")
        for sample_index, row in enumerate(rows):
            t, d = float(row[ti]), float(row[di])
            if d > peak["error_m"]:
                peak = {"error_m": d, "time_s": t}
            if 20 <= t < 22 and d > first_peak["error_m"]:
                first_peak = {"error_m": d, "time_s": t}
            if t < 20 and d != 0:
                before_force_zero = False
            if 19 <= t < 27 and sample_index % 10 == 0:
                trace["time_s"].append(t)
                trace["disturbance_deviation_mm"].append(d*1000)
    assert math.isclose(peak["error_m"], recorded["metrics"]["disturbance_deviation"]["tip_max_error_m"], abs_tol=1e-12)
    assert before_force_zero
    return {"metrics": recorded["metrics"], "recovery": recorded["recovery"],
            "full_peak_paired_deviation": peak, "first_force_peak_paired_deviation": first_peak,
            "before_first_force_positions_exactly_equal": before_force_zero, "chart_traces": trace}


def main():
    for filename in ("simulation.py", "kinematics.py", "paths.py", "control.py", "adaptive_control.py", "disturbance.py"):
        source(PROJECT / "raa" / filename)
    first_metric = read_json(PROJECT / "output/normal/gc-pd/metrics.json")
    source(PROJECT / "output/normal/gc-pd/scene.usda")
    result = {
        "project": "robot arm artist / raa-v0",
        "purpose": "Read-only supplemental diagnostics for assignment-aligned editable presentation; every quantitative value is from the preserved full runs or their saved reference",
        "definitions": {"tracking": "actual 3D virtual tip minus target at the same time", "paired_deviation": "disturbed actual 3D virtual tip minus normal actual at the same time",
                        "published_rmse": "3D tracking RMSE using only pen-down samples; lift/travel excluded",
                        "full_peaks": "3D Euclidean error at native 1 kHz sample rate; no plot decimation when determining peaks",
                        "traces": "19 <= t < 27 seconds, every tenth recorded 1 ms sample; 800 points per curve; units tau=N*m and error=mm",
                        "force_window": "20 <= t < 22 seconds, world-frame force (3,0,0) N",
                        "recovery": "After force end, paired 3D deviation <=2 mm continuously for at least 0.5 s; report stable-window start minus force-end time"},
        "robot": {"model": "Franka Emika Panda", "arm_joint_names": [f"panda_joint{i}" for i in range(1, 8)],
                  "end_link": "panda_hand", "virtual_tip_offset_hand_m": [0., 0., .13],
                  "desired_hand_rotation_world": [[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]],
                  "gravity_world_m_s2": [0., 0., -9.81], "base_fixed": True, "arm_position_and_velocity_drives": "stiffness=0,damping=0",
                  "control_mode": "real joint efforts passed through set_dof_efforts at 1 kHz",
                  "common_initialization": "All controllers begin after 1 s of GC-PD settling at initial IK q, with adaptive weights zero and not trained",
                  "pen_contact": "visual geometry only; no physical pen-table contact or added pen load",
                  "ink_projection": "actual tip XY is displayed on draw_z+0.001 m; stored actual XYZ and 3D errors are unprojected"},
        "reference": reference_diagnostics(first_metric),
        "runs": {condition: {controller: run_diagnostics(controller, condition) for controller in CONTROLLERS} for condition in CONDITIONS},
        "comparisons": {controller: comparison_diagnostics(controller) for controller in CONTROLLERS},
    }
    result["provenance"] = PROVENANCE
    destination = Path(__file__).with_suffix(".json")
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(destination), "ik_tip_um": result["reference"]["recomputed_max_ik_tip_error_m"]*1e6,
                      "ik_orientation_deg": result["reference"]["recomputed_max_orientation_error_deg"],
                      "max_planning_joint_step_deg": result["reference"]["max_adjacent_planning_joint_jump_deg"],
                      "source_files_hashed": len(PROVENANCE)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
