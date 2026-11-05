#pragma once

/* Simulator-independent C ABI. All arrays use double, SI units and the joint
 * ordering returned by ol_joint_name. One handle must only be used by one thread.
 * Quaternion is [w,x,y,z], root pose is the URDF base_link frame, and both input
 * velocities are expressed in world coordinates. Output is JOINT-SIDE torque,
 * including WBC feedforward, joint PD, low-pass filtering and torque limits.
 * No Python, MuJoCo or Isaac headers/libraries are needed to load this ABI.
 */
#ifdef __cplusplus
extern "C" {
#endif
void* ol_create(const char* repo_root, double dt, double mpc_dt,
                double swing_time, double step_height, double stand_height);
void ol_destroy(void* handle);
const char* ol_last_error(void);
int ol_joint_count(void* handle);
const char* ol_joint_name(void* handle, int index);
int ol_initial_joints(void* handle, double* out, int count);
/* Calls must be separated by dt simulation seconds. First three seconds hold
 * the initialized pose. Set walking=1 to start stepping afterwards; vx,vy are
 * body/yaw-frame velocity commands and wz is a yaw-rate command. Stopping ramps
 * all velocities down and completes the current step before double support.
 * On failure, output is zeroed and a negative value is returned. Recreate the
 * controller after simulation reset or a solver/numerical failure. */
int ol_step(void* handle, double sim_time, const double* joints,
            const double* joint_vel, const double* base_pos,
            const double* quat_wxyz, const double* linvel_world,
            const double* angvel_world, const double* command_vx_vy_wz,
            int walking, double* tau_joint_out);
/* Execute one jump-control step. phase is the stable code exported by
 * isaac.jump_controller.PHASE_CODES: prepare=0, crouch=1, thrust=2,
 * flight_up=3, flight_down=4, landing=5, recovery=6, complete=7.
 * height_reference is the dynamic base-height reference during contact and the
 * desired leg extension height during flight. phase_progress is in [0,1].
 * landing_foot_advance is the final apex-anchored forward foot displacement
 * used by flight_down, in metres. */
int ol_step_jump(void* handle, double sim_time, const double* joints,
                 const double* joint_vel, const double* base_pos,
                 const double* quat_wxyz, const double* linvel_world,
                 const double* angvel_world, int phase,
                 double height_reference, double vertical_velocity_reference,
                 double phase_progress, double landing_foot_advance,
                 double* tau_joint_out);
/* L order: rpy(3), position(3), angular velocity(3), linear velocity(3).
 * K order: left foot force(3), moment(3), right foot force(3), moment(3),
 * the fixed gravity input(1). Applied at each subsequent MPC solve. */
int ol_set_mpc_weights(void* handle, double u_weight,
                       const double* L12, const double* K13);
/* Select an externally solved first-stage contact wrench for the next and
 * subsequent controller steps.  wrench12 uses the same world-frame ordering
 * as OpenLoong's Fr_ff: left [Fx,Fy,Fz,Mx,My,Mz], then right.  The internal
 * Convex MPC still updates the common motion reference; status==0 applies the
 * external wrench, while a nonzero solver status falls back to the internal
 * Convex MPC result.  Call with enabled=0 to restore the original controller.
 * solve_time and iterations are diagnostic metadata supplied by the caller. */
int ol_set_external_mpc_wrench(void* handle, int enabled,
                               const double* wrench12, int count,
                               double solve_time, int solver_status,
                               int iterations);
int ol_diagnostic_count(void* handle);
const char* ol_diagnostic_name(void* handle, int index);
int ol_diagnostics(void* handle, double* out, int count);
/* Detailed control decomposition.  Names are stable for the lifetime of the
 * handle.  Signals contain MPC foot wrench, applied WBC feedforward, raw and
 * filtered joint PD corrections, pre-limit torque and final commanded torque.
 * Measured Isaac effort is simulator feedback and is intentionally logged by
 * Python rather than represented as a C++ controller output. */
int ol_control_signal_count(void* handle);
const char* ol_control_signal_name(void* handle, int index);
int ol_control_signals(void* handle, double* out, int count);
#ifdef __cplusplus
}
#endif
