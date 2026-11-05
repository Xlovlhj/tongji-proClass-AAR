// Adapter around the upstream OpenLoong controller (see its LICENSE and source
// notices). Isaac supplies exact simulated state; no IMU/Kalman filter runs here.
#include "controller_bridge.h"
#include "pino_kin_dyn.h"
#include "PVT_ctrl.h"
#include "wbc_priority.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "StateEst.h"
#include "useful_math.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string last_error;
constexpr int kJointCount = 31;
constexpr double kPoseHoldSeconds = 3.0;
constexpr double kAnkleToSole = 0.07;
constexpr int kLegJointOffset = kJointCount - 12;
// After the source landing-MPC braking window, keep dissipating the measured
// horizontal base velocity instead of abruptly dropping the horizontal wrench
// feed-forward to zero.  The total damping wrench is split equally between the
// two feet; the per-foot norm limit leaves margin inside the WBC friction cone.
constexpr double kJumpRecoveryHorizontalDamping = 450.0;  // N/(m/s)
constexpr double kJumpRecoveryHorizontalForcePerFoot = 150.0;  // N
constexpr std::array<const char*, 55> kDiagnostics = {
    "sim_time_s", "ready", "motion_state", "leg_state", "gait_phase",
    "mpc_enabled", "mpc_qp_status", "mpc_solve_s", "mpc_nwsr",
    "wbc_qp_status", "wbc_solve_s", "wbc_nwsr", "controller_step_s",
    "left_force_z_est_N", "right_force_z_est_N", "max_abs_joint_torque_Nm",
    "command_vx_mps", "command_vy_mps", "command_wz_radps",
    "initial_ik_error", "initial_ik_iterations", "base_height_m",
    "roll_rad", "pitch_rad", "base_yaw_rad", "wrench_source_external",
    "external_mpc_status", "external_mpc_solve_s", "external_mpc_iterations",
    "external_mpc_update_count", "next_leg_state",
    "jump_active", "jump_phase", "jump_height_reference_m",
    "jump_vertical_velocity_reference_m_s", "jump_phase_progress",
    "jump_landing_foot_advance_m",
    "com_x_m", "com_y_m", "com_z_m",
    "left_foot_x_m", "left_foot_y_m", "left_foot_z_m",
    "right_foot_x_m", "right_foot_y_m", "right_foot_z_m",
    "com_target_x_m", "com_target_y_m", "com_target_z_m",
    "left_foot_target_x_m", "left_foot_target_y_m", "left_foot_target_z_m",
    "right_foot_target_x_m", "right_foot_target_y_m", "right_foot_target_z_m"};
constexpr std::array<const char*, 12> kWrenchComponents = {
    "left_fx_N", "left_fy_N", "left_fz_N", "left_mx_Nm", "left_my_Nm", "left_mz_Nm",
    "right_fx_N", "right_fy_N", "right_fz_N", "right_mx_Nm", "right_my_Nm", "right_mz_Nm"};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void finite_array(const double* p, int count, const char* name) {
    require(p != nullptr, std::string(name) + " is null");
    for (int i = 0; i < count; ++i)
        require(std::isfinite(p[i]), std::string(name) + " contains NaN/Inf");
}

struct Controller {
    Pin_KinDyn kin;
    DataBus state;
    WBC_priority wbc;
    MPC mpc;
    MPC jump_mpc;
    GaitScheduler gait;
    PVT_Ctr pvt;
    FootPlacement feet;
    JoyStickInterpreter joy;
    StateEst forces;
    const double dt, height;
    const int mpc_stride;
    Eigen::VectorXd initial_q;
    Eigen::Matrix<double, 1, nx> weights_l;
    Eigen::Matrix<double, 1, nu> weights_k;
    double input_weight = 1e-6;
    std::array<double, kDiagnostics.size()> diagnostics{};
    std::vector<std::string> control_signal_names;
    std::array<double, 3> requested{};
    std::array<double, 12> external_wrench{};
    bool external_wrench_enabled = false;
    int external_mpc_status = 0, external_mpc_iterations = 0;
    double external_mpc_solve_s = 0.0;
    unsigned long long external_mpc_updates = 0;
    long long steps = 0;
    double first_time = -1, last_time = -1;
    bool have_state = false, failed = false;
    int jump_phase = -1, previous_jump_phase = -1;
    double jump_height_reference = 0.0, jump_velocity_reference = 0.0;
    double jump_phase_progress = 0.0, jump_landing_foot_advance = 0.0;
    bool jump_geometry_initialized = false;
    Eigen::Vector3d jump_left_foot_body, jump_right_foot_body;
    Eigen::Vector3d jump_flight_left_world_des, jump_flight_right_world_des;
    Eigen::Vector3d jump_flight_left_body_des, jump_flight_right_body_des;
    Eigen::Vector3d jump_flight_down_left_start, jump_flight_down_right_start;
    Eigen::Vector3d jump_apex_left_world, jump_apex_right_world;
    Eigen::Matrix3d jump_left_foot_rotation, jump_right_foot_rotation;
    Eigen::Vector3d jump_origin_position;
    Eigen::Vector2d jump_nominal_com_support_offset = Eigen::Vector2d::Zero();
    double jump_origin_yaw = 0.0;
    std::array<double, kJointCount> original_kp{}, original_kd{};

    Controller(const std::string& root, double time_step, double mpc_dt,
               double swing_time, double step_height, double stand_height)
        : kin(root + "/models/AzureLoong.urdf"), state(kin.model_nv),
          wbc(kin.model_nv, 18, 22, 0.7, time_step), mpc(mpc_dt),
          jump_mpc(time_step),
          gait(swing_time, time_step),
          pvt(time_step, (root + "/common/joint_ctrl_config.json").c_str()),
          joy(time_step), forces(time_step), dt(time_step), height(stand_height),
          mpc_stride(static_cast<int>(std::llround(mpc_dt / time_step))) {
        require(kin.model_nv == 37, "Expected the 31-joint AzureLoong model (nv=37)");
        for (int i = 0; i < kJointCount; ++i) {
            const auto id = kin.model_biped.getJointId(kin.motorName[i]);
            require(id < kin.model_biped.joints.size(), "Missing URDF joint: " + kin.motorName[i]);
            require(kin.model_biped.joints[id].idx_q() == i + 7 &&
                    kin.model_biped.joints[id].idx_v() == i + 6,
                    "URDF joint ordering changed: " + kin.motorName[i]);
            require(std::isfinite(pvt.maxTor[i]) && pvt.maxTor[i] > 0 &&
                    std::isfinite(pvt.gear[i]) && pvt.gear[i] > 0 &&
                    std::isfinite(pvt.pvt_Kp[i]) && pvt.pvt_Kp[i] >= 0 &&
                    std::isfinite(pvt.pvt_Kd[i]) && pvt.pvt_Kd[i] >= 0,
                    "Missing/invalid joint controller JSON entry: " + kin.motorName[i]);
            original_kp[i] = pvt.pvt_Kp[i];
            original_kd[i] = pvt.pvt_Kd[i];
        }
        feet.kp_vx = feet.kp_vy = feet.kp_wz = 0.03;
        feet.stepHeight = step_height;
        feet.legLength = height - kAnkleToSole;
        state.width_hips = 0.229;
        state.tSwing = swing_time;
        state.phi = 0;
        state.theta0 = 0;
        state.desWz_W = 0;
        state.isdqIni = true;
        state.leg_contact[0] = state.leg_contact[1] = true;
        state.qp_nWSR = state.qp_nWSR_MPC = 0;
        state.qp_status = state.qpStatus_MPC = 0;
        state.qp_cpuTime = state.qp_cpuTime_MPC = 0;
        for (auto* array : {state.rpy, state.fL, state.fR, state.basePos,
                            state.baseLinVel, state.baseAcc, state.baseAngVel})
            std::fill_n(array, 3, 0.0);
        state.des_q = Eigen::VectorXd::Zero(kin.model_nv + 1);
        state.stance_fe_rot_cur_W.setIdentity();
        state.slop.setZero();
        joy.reset();

        const Eigen::Matrix3d foot_rotation = eul2Rot(0, -0.008, 0);
        const auto ik = kin.computeInK_Leg(
            foot_rotation, Eigen::Vector3d(-0.018, 0.113, -feet.legLength),
            foot_rotation, Eigen::Vector3d(-0.018, -0.116, -feet.legLength));
        require(ik.status == 0 && ik.jointPosRes.allFinite(),
                "Initial leg IK failed; error norm=" + std::to_string(ik.err.norm()) +
                ", iterations=" + std::to_string(ik.itr) +
                "; check stand_height (default base height is 1.08 m)");
        diagnostics[19] = ik.err.norm();
        diagnostics[20] = ik.itr;
        initial_q = Eigen::VectorXd::Zero(kin.model_nv + 1);
        initial_q[2] = height;
        initial_q[6] = 1.0; // Pinocchio stores quaternion xyzw.
        initial_q.tail(kJointCount) = ik.jointPosRes;
        initial_q.segment<7>(7) << 0.475, -1.12, 1.9, 0.86, -0.356, 0, 0;
        initial_q.segment<7>(14) << -0.475, -1.12, -1.9, 0.86, 0.356, 0, 0;
        for (int i = 0; i < kJointCount; ++i)
            require(initial_q[i + 7] >= pvt.minPos[i] - 1e-6 &&
                    initial_q[i + 7] <= pvt.maxPos[i] + 1e-6,
                    "Initial pose violates joint limits: " + kin.motorName[i]);
        pvt.motor_pos_des_old = eigen2std(initial_q.tail(kJointCount));
        weights_l << 1, 1, 1, 1e-3, 100, 1, 1e-3, 1e-3, 1e-3, 1, 100, 1;
        weights_k.setOnes();
        for (const char* component : kWrenchComponents)
            control_signal_names.emplace_back(std::string("mpc_wrench:") + component);
        for (const char* component : kWrenchComponents)
            control_signal_names.emplace_back(std::string("wbc_wrench:") + component);
        const std::array<const char*, 6> base_acceleration_components = {
            "vx", "vy", "vz", "wx", "wy", "wz"};
        for (const char* component : base_acceleration_components)
            control_signal_names.emplace_back(
                std::string("wbc_kin_ddq_base:") + component);
        for (const char* component : base_acceleration_components)
            control_signal_names.emplace_back(
                std::string("wbc_opt_ddq_base:") + component);
        for (const char* component : {"x", "y", "z"})
            control_signal_names.emplace_back(
                std::string("com_velocity_m_s:") + component);
        const std::array<const char*, 5> groups = {
            "wbc_feedforward_Nm:", "pd_raw_Nm:", "pd_filtered_Nm:",
            "prelimit_Nm:", "command_Nm:"};
        for (const char* group : groups)
            for (const auto& name : kin.motorName)
                control_signal_names.emplace_back(std::string(group) + name);
        // set_weight rotates weights using R_curz, initialized by dataBusRead.
        // Delay its invocation until a complete sensor state has been loaded.
    }

    void set_external_wrench(bool enabled, const double* wrench, int count,
                             double solve_time, int status, int iterations) {
        require(status >= 0, "External MPC solver status must be nonnegative");
        require(std::isfinite(solve_time) && solve_time >= 0,
                "External MPC solve time must be finite and nonnegative");
        require(iterations >= 0, "External MPC iteration count must be nonnegative");
        if (enabled) {
            require(count == static_cast<int>(external_wrench.size()),
                    "External MPC wrench must have length 12");
            finite_array(wrench, count, "external MPC wrench");
            std::copy_n(wrench, count, external_wrench.begin());
            ++external_mpc_updates;
        } else {
            require(count == 0 || count == static_cast<int>(external_wrench.size()),
                    "Disabled external MPC wrench must have length 0 or 12");
            external_wrench.fill(0.0);
        }
        external_wrench_enabled = enabled;
        external_mpc_solve_s = solve_time;
        external_mpc_status = status;
        external_mpc_iterations = iterations;
    }

    void write_control_signals(double* out, int count) const {
        require(out != nullptr, "Control signal output is null");
        require(count == static_cast<int>(control_signal_names.size()),
                "Control signal output has wrong length");
        int at = 0;
        for (int i = 0; i < 12; ++i) out[at++] = state.Fr_ff[i];
        for (int i = 0; i < 12; ++i) out[at++] = state.wbc_FrRes[i];
        for (int i = 0; i < 6; ++i) out[at++] = wbc.ddq_final_kin[i];
        for (int i = 0; i < 6; ++i) out[at++] = wbc.eigen_ddq_Opt[i];
        const Eigen::Vector3d com_velocity = state.Jcom_W * state.dq;
        for (int i = 0; i < 3; ++i) out[at++] = com_velocity[i];
        const std::array<const std::vector<double>*, 5> groups = {
            &pvt.motor_tor_des, &pvt.motor_tor_pd_raw, &pvt.motor_tor_pd_filtered,
            &pvt.motor_tor_pre_limit, &pvt.motor_tor_out_link};
        for (const auto* group : groups) {
            require(group->size() == kJointCount, "PVT torque signal has wrong length");
            std::copy(group->begin(), group->end(), out + at);
            at += kJointCount;
        }
    }

    bool external_wrench_matches_support() const {
        // GaitScheduler may change support inside command() after Python has
        // solved the external MPC from the previous step's diagnostics.  A
        // one-step-old single-support wrench must never be sent to WBC for the
        // opposite support foot.  Fall back to the internal convex MPC until
        // the next external update has caught up with the new contact mode.
        constexpr double kContactForceTolerance = 1e-6;
        const bool left_active = external_wrench[2] > kContactForceTolerance;
        const bool right_active = external_wrench[8] > kContactForceTolerance;
        if (state.legState == DataBus::LSt)
            return left_active && !right_active;
        if (state.legState == DataBus::RSt)
            return !left_active && right_active;
        if (state.legState == DataBus::DSt)
            return left_active && right_active;
        return false;
    }

    void load_state(const double* joints, const double* joint_vel,
                    const double* pos, const double* quat,
                    const double* linear, const double* angular) {
        Eigen::Quaterniond orientation(quat[0], quat[1], quat[2], quat[3]);
        require(orientation.norm() > 1e-8, "Quaternion has zero norm");
        orientation.normalize();
        state.base_rot = orientation.toRotationMatrix();
        const double previous_yaw = state.base_rpy[2];
        const double raw_yaw = std::atan2(state.base_rot(1, 0), state.base_rot(0, 0));
        // MPC compares Euler-yaw values directly. Preserve a continuous branch
        // so a turn across +pi/-pi does not create a spurious 2*pi tracking error.
        const double yaw = have_state ? previous_yaw +
            std::remainder(raw_yaw - previous_yaw, 2.0 * 3.14159265358979323846) : raw_yaw;
        state.base_rpy << std::atan2(state.base_rot(2, 1), state.base_rot(2, 2)),
            std::asin(std::clamp(-state.base_rot(2, 0), -1.0, 1.0)),
            yaw;
        state.base_pos = Eigen::Map<const Eigen::Vector3d>(pos);
        state.base_vel = Eigen::Map<const Eigen::Vector3d>(linear);
        state.base_omega_W = Eigen::Map<const Eigen::Vector3d>(angular);
        state.base_omega_L = state.base_rot.transpose() * state.base_omega_W;
        state.q.head<3>() = state.base_pos;
        state.q.segment<4>(3) = orientation.coeffs();
        state.q.tail(kJointCount) = Eigen::Map<const Eigen::VectorXd>(joints, kJointCount);
        state.dq.head<3>() = state.base_vel;
        state.dq.segment<3>(3) = state.base_omega_W;
        state.dq.tail(kJointCount) = Eigen::Map<const Eigen::VectorXd>(joint_vel, kJointCount);
        state.qOld = state.q;
        state.motors_pos_cur.assign(joints, joints + kJointCount);
        state.motors_vel_cur.assign(joint_vel, joint_vel + kJointCount);
        for (int i = 0; i < 3; ++i) {
            state.basePos[i] = pos[i];
            state.baseLinVel[i] = linear[i];
            state.baseAngVel[i] = state.base_omega_L[i];
            state.rpy[i] = state.base_rpy[i];
        }
        // Populate legacy estimator fields with simulator truth for diagnostics.
        // Do not call StateEst::set/update/get; that would filter/overwrite truth.
        state.base_pos_est = state.base_pos;
        state.base_vel_est = state.base_vel;
        state.eul_est = state.base_rpy;
        state.omegaW_est = state.base_omega_W;
        kin.dataBusRead(state); // converts world dq to Pinocchio local-root dq.
        kin.computeJ_dJ();
        kin.computeDyn();
        kin.dataBusWrite(state);
        require(state.dyn_M.allFinite() && state.dyn_Non.allFinite(),
                "Non-finite rigid-body dynamics");
        forces.setF(state);
        forces.updateF();
        forces.getF(state);
        require(state.FL_est.allFinite() && state.FR_est.allFinite(),
                "Non-finite foot-force estimate");
        state.fe_l_pos_W_est = state.fe_l_pos_W;
        state.fe_r_pos_W_est = state.fe_r_pos_W;
    }

    void capture_stand_reference() {
        wbc.setQini(initial_q, state.q);
        wbc.fe_l_pos_des_W = state.fe_l_pos_W;
        wbc.fe_r_pos_des_W = state.fe_r_pos_W;
        wbc.fe_l_rot_des_W = state.fe_l_rot_W;
        wbc.fe_r_rot_des_W = state.fe_r_rot_W;
        wbc.pCoMDes = state.pCoM_W;
    }

    void capture_jump_landing_reference() {
        capture_stand_reference();
        // Touchdown can happen while the torso still has a small horizontal
        // offset.  Freezing that instantaneous CoM position would turn the
        // impact error into the new balance point.  Preserve the robot's
        // nominal CoM-to-support-centre offset instead, using the two measured
        // foot poses that define the actual Isaac support polygon.
        const Eigen::Vector2d support_centre = 0.5 *
            (state.fe_l_pos_W.head<2>() + state.fe_r_pos_W.head<2>());
        wbc.pCoMDes.head<2>() =
            support_centre + jump_nominal_com_support_offset;
    }

    void command(const double* target, bool walking, bool ready) {
        if (ready && walking && state.motionState == DataBus::Stand) {
            gait.start();
            joy.setIniPos(state.q[0], state.q[1], height, state.base_rpy[2]);
            state.motionState = DataBus::Walk;
            // Force an MPC solve immediately when support mode changes.
            steps = 0;
        }
        const std::array<double, 3> desired = ready && walking
            ? std::array<double, 3>{target[0], target[1], target[2]}
            : std::array<double, 3>{0, 0, 0};
        if (desired[0] != requested[0]) joy.setVxDesLPara(desired[0], walking ? 2.0 : 0.5);
        if (desired[1] != requested[1]) joy.setVyDesLPara(desired[1], walking ? 2.0 : 0.5);
        if (desired[2] != requested[2]) joy.setWzDesLPara(desired[2], walking ? 1.0 : 0.5);
        requested = desired;
        if (!walking && state.motionState == DataBus::Walk &&
            std::abs(joy.vx_L) < 0.01 && std::abs(joy.vy_L) < 0.01 && std::abs(joy.wz_L) < 0.01)
            state.motionState = DataBus::Walk2Stand;
        if (!ready || state.motionState == DataBus::Walk2Stand)
            joy.setIniPos(state.q[0], state.q[1], height, state.base_rpy[2]);
        if (state.motionState != DataBus::Stand) {
            joy.step();
            joy.dataBusWrite(state);
            state.js_pos_des[2] = height;
            gait.dataBusRead(state);
            gait.step();
            gait.dataBusWrite(state);
            if (state.motionState != DataBus::Stand) {
                feet.dataBusRead(state);
                feet.getSwingPos();
                feet.dataBusWrite(state);
            } else {
                // Initialize double-support references at the completed step.
                joy.reset();
                joy.setIniPos(state.q[0], state.q[1], height, state.base_rpy[2]);
                joy.dataBusWrite(state);
                state.js_pos_des[2] = height;
                capture_stand_reference();
            }
        }
        if (!ready || state.motionState == DataBus::Walk2Stand) capture_stand_reference();
    }

    void set_leg_pd() {
        for (const char* side : {"l", "r"}) {
            const std::array<const char*, 6> names = {
                "hip_", "hip_", "hip_", "knee_", "ankle_", "ankle_"};
            const std::array<const char*, 6> axes = {
                "_roll", "_yaw", "_pitch", "_pitch", "_pitch", "_roll"};
            const std::array<double, 6> kp = {400, 200, 300, 300, 300, 300};
            const std::array<double, 6> kd = {15, 10, 10, 14, 18, 16};
            for (int i = 0; i < 6; ++i)
                pvt.setJointPD(kp[i], kd[i],
                    (std::string("J_") + names[i] + side + axes[i]).c_str());
        }
    }

    void restore_original_pd() {
        for (int i = 0; i < kJointCount; ++i) {
            pvt.pvt_Kp[i] = original_kp[i];
            pvt.pvt_Kd[i] = original_kd[i];
        }
    }

    void reset_jump_leg_pd_filters() {
        // A disabled PV term still passes through the first-order torque LPF.
        // Clear its stored output once at each jump mode boundary so an
        // airborne IK torque cannot be added to the landing wrench torque.
        for (int i = 0; i < 12; ++i)
            pvt.resetPDFilter(kLegJointOffset + i);
    }

    void clear_wbc_diagnostics() {
        state.qp_status = 0;
        state.qp_cpuTime = 0.0;
        state.qp_nWSR = 0;
        state.wbc_tauJointRes.setZero();
        state.wbc_delta_q_final.setZero();
        state.wbc_dq_final.setZero();
        state.wbc_ddq_final.setZero();
        state.wbc_FrRes.setZero();
    }

    void set_wbc_motor_targets() {
        const Eigen::VectorXd q_des_full = kin.integrateDIY(
            state.q, state.wbc_delta_q_final);
        Eigen::VectorXd q_des = q_des_full.tail(kJointCount);
        Eigen::VectorXd dq_des = state.wbc_dq_final.tail(kJointCount);
        for (int i = 0; i < kJointCount; ++i) {
            const int q_index = i + 7;
            const double lower = kin.model_biped.lowerPositionLimit[q_index];
            const double upper = kin.model_biped.upperPositionLimit[q_index];
            q_des[i] = std::clamp(q_des[i], lower, upper);
            if ((state.motors_pos_cur[i] >= upper - 1e-4 && dq_des[i] > 0.0) ||
                (state.motors_pos_cur[i] <= lower + 1e-4 && dq_des[i] < 0.0))
                dq_des[i] = 0.0;
        }
        state.motors_pos_des = eigen2std(q_des);
        state.motors_vel_des = eigen2std(dq_des);
        state.motors_tor_des = eigen2std(state.wbc_tauJointRes);
        set_leg_pd();
    }

    bool run_standard_control(const double* target, bool walking, bool ready,
                              double height_override = std::numeric_limits<double>::quiet_NaN()) {
        pvt.enablePV();
        command(target, walking, ready);
        const double control_height = std::isfinite(height_override)
            ? height_override : height;
        state.js_pos_des[2] = control_height;
        const bool moving = state.motionState != DataBus::Stand;
        if (moving) mpc.enable(); else mpc.disable();
        if (steps % mpc_stride == 0) {
            mpc.dataBusRead(state);
            mpc.set_weight(input_weight, weights_l, weights_k);
            mpc.cal();
        }
        if (moving) {
            mpc.dataBusWrite(state);
            const bool use_external = external_wrench_enabled &&
                external_mpc_status == 0 && external_wrench_matches_support();
            if (use_external)
                state.Fr_ff = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(
                    external_wrench.data());
            require(state.qpStatus_MPC == 0,
                    "MPC QP failed, status=" + std::to_string(state.qpStatus_MPC));
        } else {
            state.Fr_ff.setZero();
            state.Fr_ff[2] = state.Fr_ff[8] = std::max(0.0, 0.5 * state.dyn_G[2]);
            state.des_ddq.setZero();
            state.des_dq.setZero();
            state.des_delta_q.setZero();
            state.base_rpy_des << 0, 0, joy.thetaZ;
            state.base_pos_des = state.js_pos_des;
            state.base_pos_des[2] = control_height;
        }
        wbc.dataBusRead(state);
        wbc.computeDdq(kin);
        wbc.computeTau();
        wbc.dataBusWrite(state);
        require(state.qp_status == 0,
                "WBC QP failed, status=" + std::to_string(state.qp_status));
        require(state.wbc_tauJointRes.allFinite() &&
                state.wbc_delta_q_final.allFinite() &&
                state.wbc_dq_final.allFinite(), "Non-finite WBC result");

        if (!ready) {
            state.motors_pos_des = eigen2std(initial_q.tail(kJointCount));
            state.motors_vel_des.assign(kJointCount, 0);
            state.motors_tor_des.assign(kJointCount, 0);
        } else {
            set_wbc_motor_targets();
        }
        return moving;
    }

    void initialize_jump_geometry() {
        jump_left_foot_body = state.fe_l_pos_L;
        jump_right_foot_body = state.fe_r_pos_L;
        jump_flight_left_world_des = state.base_rot * state.fe_l_pos_L;
        jump_flight_right_world_des = state.base_rot * state.fe_r_pos_L;
        jump_flight_left_body_des = state.fe_l_pos_L;
        jump_flight_right_body_des = state.fe_r_pos_L;
        jump_flight_down_left_start = jump_flight_left_world_des;
        jump_flight_down_right_start = jump_flight_right_world_des;
        jump_apex_left_world = jump_flight_left_world_des;
        jump_apex_right_world = jump_flight_right_world_des;
        jump_left_foot_rotation = eul2Rot(0.0, -0.008, 0.0);
        jump_right_foot_rotation = eul2Rot(0.0, -0.008, 0.0);
        jump_origin_position = state.base_pos;
        jump_origin_yaw = state.base_rpy[2];
        const Eigen::Vector2d support_centre = 0.5 *
            (state.fe_l_pos_W.head<2>() + state.fe_r_pos_W.head<2>());
        jump_nominal_com_support_offset =
            state.pCoM_W.head<2>() - support_centre;
        jump_geometry_initialized = true;
    }

    void set_jump_joint_targets(const Eigen::VectorXd& leg_solution,
                                bool compensate_pitch) {
        require(leg_solution.size() == kJointCount && leg_solution.allFinite(),
                "Jump leg IK returned invalid joint vector");
        Eigen::VectorXd q_des = initial_q.tail(kJointCount);
        q_des.tail<12>() = leg_solution.tail<12>();
        if (compensate_pitch) {
            // Preserve the upstream pitch compensation during flight only.
            q_des[23] += state.base_rpy[1];
            q_des[29] += state.base_rpy[1];
        }
        // The PVT JSON contains conservative walking limits (notably ankle
        // pitch +0.61087 rad), while the URDF/Isaac joint itself allows
        // +0.87 rad.  The upstream jump IK needs about +0.857 rad at a 0.90 m
        // crouch.  Clamp to the actual model limits here, otherwise the ankle
        // saturation forces the torso to pitch during the crouch.
        for (int i = 0; i < kJointCount; ++i) {
            const int q_index = i + 7;
            q_des[i] = std::clamp(q_des[i],
                kin.model_biped.lowerPositionLimit[q_index],
                kin.model_biped.upperPositionLimit[q_index]);
        }
        state.motors_pos_des = eigen2std(q_des);
        state.motors_vel_des.assign(kJointCount, 0.0);
        state.motors_tor_des.assign(kJointCount, 0.0);
    }

    void run_jump_ik_control(double reference_height, bool compensate_pitch) {
        require(reference_height > kAnkleToSole + 0.60 && reference_height < 1.30,
                "Jump IK height reference is outside the supported range");
        jump_mpc.disable();
        state.Fr_ff.setZero();
        state.fe_react_tau_cmd.setZero();
        state.qpStatus_MPC = 0;
        state.qp_cpuTime_MPC = 0.0;
        state.qp_nWSR_MPC = 0;
        clear_wbc_diagnostics();
        pvt.enablePV();
        restore_original_pd();

        Eigen::Vector3d left = jump_left_foot_body;
        Eigen::Vector3d right = jump_right_foot_body;
        left[2] = right[2] = -(reference_height - kAnkleToSole);
        const auto ik = kin.computeInK_Leg(
            jump_left_foot_rotation, left, jump_right_foot_rotation, right);
        require(ik.status == 0 && ik.jointPosRes.allFinite(),
                "Jump leg IK failed; error norm=" + std::to_string(ik.err.norm()));
        set_jump_joint_targets(ik.jointPosRes, compensate_pitch);
    }

    void run_jump_flight_control(double reference_height, double phase_progress,
                                 double landing_foot_advance,
                                 bool forward_landing_swing) {
        require(reference_height > kAnkleToSole + 0.60 && reference_height < 1.30,
                "Jump flight IK height reference is outside the supported range");
        require(phase_progress >= 0.0 && phase_progress <= 1.0 + 1e-9,
                "Jump flight progress is outside [0,1]");
        require(landing_foot_advance >= 0.0 && landing_foot_advance <= 0.20,
                "Jump landing foot advance is outside [0,0.20] m");
        jump_mpc.disable();
        state.Fr_ff.setZero();
        state.fe_react_tau_cmd.setZero();
        state.qpStatus_MPC = 0;
        state.qp_cpuTime_MPC = 0.0;
        state.qp_nWSR_MPC = 0;
        clear_wbc_diagnostics();
        pvt.enablePV();
        restore_original_pd();

        if (!forward_landing_swing) {
            // flight_up: preserve the world-oriented x/y anchor saved before
            // crouching and retract z, as in jump_mpc.cpp.
            jump_flight_left_world_des[2] =
                -(reference_height - kAnkleToSole);
            jump_flight_right_world_des[2] =
                -(reference_height - kAnkleToSole);
            jump_flight_left_body_des =
                state.base_rot.transpose() * jump_flight_left_world_des;
            jump_flight_right_body_des =
                state.base_rot.transpose() * jump_flight_right_world_des;
        } else {
            // flight_down: re-sample the feet at the apex and advance the
            // world-oriented x target.  Match jump_mpc.cpp exactly here: after
            // transforming that target back to the body frame, update only x;
            // keep the flight-up y/z leg geometry through touchdown.
            const double p = std::clamp(phase_progress, 0.0, 1.0);
            jump_flight_left_world_des[0] =
                jump_flight_down_left_start[0] + p *
                (jump_apex_left_world[0] + landing_foot_advance -
                 jump_flight_down_left_start[0]);
            jump_flight_right_world_des[0] =
                jump_flight_down_right_start[0] + p *
                (jump_apex_right_world[0] + landing_foot_advance -
                 jump_flight_down_right_start[0]);
            const Eigen::Vector3d left_body_from_world =
                state.base_rot.transpose() * jump_flight_left_world_des;
            const Eigen::Vector3d right_body_from_world =
                state.base_rot.transpose() * jump_flight_right_world_des;
            jump_flight_left_body_des[0] = left_body_from_world[0];
            jump_flight_right_body_des[0] = right_body_from_world[0];
        }
        const auto ik = kin.computeInK_Leg(
            jump_left_foot_rotation, jump_flight_left_body_des,
            jump_right_foot_rotation, jump_flight_right_body_des);
        require(ik.status == 0 && ik.jointPosRes.allFinite(),
                "Jump flight leg IK failed; error norm=" +
                std::to_string(ik.err.norm()));
        // The source demo adds base pitch to both ankle-pitch references after
        // the world/body position transform; retain that compensation.
        set_jump_joint_targets(ik.jointPosRes, true);
    }

    void seed_jump_mpc(const Eigen::Matrix<double, 1, nx>& l,
                       const Eigen::Matrix<double, 1, nu>& k) {
        jump_mpc.disable();
        jump_mpc.dataBusRead(state);
        jump_mpc.set_weight(1e-6, l, k);
        jump_mpc.cal();
    }

    void run_jump_force_control(double reference_height,
                                double vertical_velocity, bool landing_weights,
                                bool entering_phase) {
        require(reference_height > 0.70 && reference_height < 1.30,
                "Jump MPC height reference is outside the supported range");
        state.motionState = DataBus::Stand;
        state.legState = state.legStateNext = DataBus::DSt;
        state.js_eul_des << 0.0, 0.0, jump_origin_yaw;
        state.js_pos_des = jump_origin_position;
        state.js_pos_des[2] = reference_height;
        state.js_omega_des.setZero();
        state.js_vel_des << 0.0, 0.0, vertical_velocity;

        Eigen::Matrix<double, 1, nx> l;
        Eigen::Matrix<double, 1, nu> k;
        if (landing_weights) {
            l << 2.0, 10.0, 1.0, 100.0, 100.0, 200.0,
                 1e-4, 1e-4, 1e-4, 0.5, 0.01, 0.5;
            k << 0.1, 0.1, 0.1, 0.01, 0.01, 1.0,
                 0.1, 0.1, 0.1, 0.01, 0.01, 1.0, 1.0;
        } else {
            l << 50.0, 50.0, 1.0, 50.0, 50.0, 200.0,
                 0.1, 0.1, 0.1, 0.01, 0.1, 20.0;
            k << 1.0, 1.0, 0.1, 10.0, 10.0, 10.0,
                 1.0, 1.0, 0.1, 10.0, 10.0, 10.0, 1.0;
        }
        if (entering_phase)
            seed_jump_mpc(l, k);
        jump_mpc.enable();
        jump_mpc.dataBusRead(state);
        jump_mpc.set_weight(1e-6, l, k);
        jump_mpc.cal();
        jump_mpc.dataBusWrite(state);
        require(state.qpStatus_MPC == 0,
                "Jump MPC QP failed, status=" + std::to_string(state.qpStatus_MPC));

        if (landing_weights) {
            // Keep the upstream landing MPC as the feed-forward wrench, but
            // use WBC for the PhysX contact geometry and full-body posture.
            // Direct -J^T mapping is retained for thrust below; applying it at
            // touchdown caused a rapid roll divergence in Isaac (run27).
            state.des_ddq.setZero();
            state.des_dq.setZero();
            state.des_delta_q.setZero();
            state.base_rpy_des = state.js_eul_des;
            state.base_pos_des = state.js_pos_des;
            wbc.dataBusRead(state);
            wbc.computeDdq(kin);
            wbc.computeTau();
            wbc.dataBusWrite(state);
            require(state.qp_status == 0,
                    "Landing WBC QP failed, status=" +
                    std::to_string(state.qp_status));
            require(state.wbc_tauJointRes.allFinite() &&
                    state.wbc_delta_q_final.allFinite() &&
                    state.wbc_dq_final.allFinite(),
                    "Non-finite landing WBC result");
            pvt.enablePV();
            set_wbc_motor_targets();
            return;
        }

        clear_wbc_diagnostics();

        Eigen::Matrix<double, 12, 12> jacobian;
        jacobian.setZero();
        jacobian.topRows<6>() = state.J_l.block(0, kin.model_nv - 12, 6, 12);
        jacobian.bottomRows<6>() = state.J_r.block(0, kin.model_nv - 12, 6, 12);
        const Eigen::Matrix<double, 12, 1> leg_torque =
            -jacobian.transpose() * state.Fr_ff;
        require(leg_torque.allFinite(), "Jump Jacobian torque contains NaN/Inf");

        pvt.enablePV();
        state.motors_pos_des = eigen2std(initial_q.tail(kJointCount));
        state.motors_vel_des.assign(kJointCount, 0.0);
        state.motors_tor_des.assign(kJointCount, 0.0);
        constexpr std::array<double, 6> limits = {400, 100, 400, 400, 80, 20};
        for (int side = 0; side < 2; ++side) {
            for (int joint = 0; joint < 6; ++joint) {
                const int leg_index = side * 6 + joint;
                const int motor_index = kLegJointOffset + leg_index;
                pvt.disablePV(motor_index);
                state.motors_tor_des[motor_index] = std::clamp(
                    leg_torque[leg_index], -limits[joint], limits[joint]);
            }
        }
    }

    void run_jump_recovery_control(double reference_height) {
        require(reference_height > 0.70 && reference_height < 1.30,
                "Jump recovery height reference is outside the supported range");
        jump_mpc.disable();
        state.qpStatus_MPC = 0;
        state.qp_cpuTime_MPC = 0.0;
        state.qp_nWSR_MPC = 0;

        state.motionState = DataBus::Stand;
        state.legState = state.legStateNext = DataBus::DSt;
        state.js_eul_des << 0.0, 0.0, jump_origin_yaw;
        state.js_pos_des = jump_origin_position;
        state.js_pos_des[2] = reference_height;
        state.js_omega_des.setZero();
        state.js_vel_des.setZero();
        state.des_ddq.setZero();
        state.des_dq.setZero();
        state.des_delta_q.setZero();
        state.base_rpy_des = state.js_eul_des;
        state.base_pos_des = state.js_pos_des;

        state.Fr_ff.setZero();
        state.Fr_ff[2] = state.Fr_ff[8] =
            std::max(0.0, 0.5 * state.dyn_G[2]);
        Eigen::Vector2d horizontal_force =
            -kJumpRecoveryHorizontalDamping * state.base_vel.head<2>();
        const double total_force_limit =
            2.0 * kJumpRecoveryHorizontalForcePerFoot;
        if (horizontal_force.norm() > total_force_limit)
            horizontal_force *= total_force_limit / horizontal_force.norm();
        for (const int offset : {0, 6}) {
            state.Fr_ff[offset] = 0.5 * horizontal_force[0];
            state.Fr_ff[offset + 1] = 0.5 * horizontal_force[1];
        }

        wbc.dataBusRead(state);
        wbc.computeDdq(kin);
        wbc.computeTau();
        wbc.dataBusWrite(state);
        require(state.qp_status == 0,
                "Jump recovery WBC QP failed, status=" +
                std::to_string(state.qp_status));
        require(state.wbc_tauJointRes.allFinite() &&
                state.wbc_delta_q_final.allFinite() &&
                state.wbc_dq_final.allFinite(),
                "Non-finite jump recovery WBC result");
        pvt.enablePV();
        set_wbc_motor_targets();
    }

    bool run_jump_control(int phase, double reference_height,
                          double vertical_velocity, bool ready) {
        require(phase >= 0 && phase <= 7, "Jump phase must be in [0,7]");
        if (!jump_geometry_initialized)
            initialize_jump_geometry();
        const bool entering_phase = phase != previous_jump_phase;
        const bool recovery_mpc_active =
            phase == 6 && jump_phase_progress < 1.0 - 1e-9;
        // Phase 5 keeps the source landing-MPC horizontal wrench exactly.  In
        // the early phase-6 MPC window, soft tracking lets WBC absorb impact.
        // The later bounded damping wrench is chosen to meet the last realized
        // MPC/WBC force continuously, so preserve it exactly; otherwise the
        // WBC correction can reverse the braking direction (run31/run32).
        const bool recovery_damping_active =
            (phase == 6 && !recovery_mpc_active) || phase == 7;
        wbc.setPreserveHorizontalContactWrench(
            phase == 5 || recovery_damping_active);
        wbc.setTrackHorizontalContactWrench(recovery_mpc_active);
        if (entering_phase && phase == 0)
            wbc.setStandRecoveryMode(false);
        if (entering_phase &&
            (phase == 2 || phase == 3 || phase == 5))
            reset_jump_leg_pd_filters();
        if (entering_phase && phase == 1) {
            // The source keeps the pre-crouch world-oriented x/y foot anchor
            // throughout flight_up instead of inheriting takeoff roll/pitch.
            jump_flight_left_world_des = state.base_rot * state.fe_l_pos_L;
            jump_flight_right_world_des = state.base_rot * state.fe_r_pos_L;
            jump_flight_left_body_des = state.fe_l_pos_L;
            jump_flight_right_body_des = state.fe_r_pos_L;
        }
        if (entering_phase && (phase == 2 || phase == 5)) {
            jump_origin_position = state.base_pos;
            jump_origin_yaw = state.base_rpy[2];
            if (phase == 5) {
                wbc.setStandRecoveryMode(true);
                capture_jump_landing_reference();
            }
        }
        if (entering_phase && phase == 4) {
            jump_apex_left_world = state.base_rot * state.fe_l_pos_L;
            jump_apex_right_world = state.base_rot * state.fe_r_pos_L;
            jump_flight_down_left_start = jump_flight_left_world_des;
            jump_flight_down_right_start = jump_flight_right_world_des;
        }

        static const double zero_command[3] = {0.0, 0.0, 0.0};
        if (!ready || phase == 0)
            return run_standard_control(zero_command, false, ready);
        if (phase == 1)
            return run_standard_control(
                zero_command, false, ready, reference_height);
        if (phase == 3 || phase == 4) {
            run_jump_flight_control(reference_height, jump_phase_progress,
                                    jump_landing_foot_advance,
                                    phase == 4);
            return false;
        }
        if (phase == 2) {
            run_jump_force_control(reference_height, vertical_velocity, false,
                                   entering_phase);
            return true;
        }
        if (phase == 6) {
            if (recovery_mpc_active) {
                // Short source-MPC braking window immediately after impact.
                run_jump_force_control(reference_height, vertical_velocity,
                                       true, entering_phase);
                return true;
            }
            // Keep a velocity-opposing horizontal wrench after the source MPC
            // window.  This avoids the measured Fx sign reversal at the run31
            // handoff while the fixed-foot WBC continues posture recovery.
            run_jump_recovery_control(reference_height);
            return false;
        }
        if (phase == 7) {
            run_jump_recovery_control(reference_height);
            return false;
        }
        run_jump_force_control(reference_height, vertical_velocity, true,
                               entering_phase);
        return true;
    }

    void step(double time, const double* joints, const double* joint_vel,
              const double* pos, const double* quat, const double* linear,
              const double* angular, const double* target, bool walking, double* out,
              int requested_jump_phase = -1, double requested_jump_height = 0.0,
              double requested_jump_velocity = 0.0,
              double requested_jump_progress = 0.0,
              double requested_landing_foot_advance = 0.0) {
        require(!failed, "Controller previously failed; recreate it before continuing");
        require(std::isfinite(time) && time >= 0, "sim_time must be finite and nonnegative");
        if (have_state)
            require(std::abs((time - last_time) - dt) <= std::max(1e-7, dt * 1e-3),
                    "ol_step must advance exactly one configured physics dt; recreate after reset");
        const auto begin = std::chrono::steady_clock::now();
        load_state(joints, joint_vel, pos, quat, linear, angular);
        if (!have_state) {
            first_time = time;
            have_state = true;
            joy.setIniPos(pos[0], pos[1], height, state.base_rpy[2]);
            joy.dataBusWrite(state);
            state.js_pos_des[2] = height;
            capture_stand_reference();
            // Seed MPC prediction horizon while disabled, before enabling gait.
            mpc.disable();
            mpc.dataBusRead(state);
            mpc.set_weight(input_weight, weights_l, weights_k);
            mpc.cal();
        }
        const bool ready = time - first_time >= kPoseHoldSeconds - 1e-9;
        jump_phase = requested_jump_phase;
        jump_height_reference = requested_jump_height;
        jump_velocity_reference = requested_jump_velocity;
        jump_phase_progress = requested_jump_progress;
        jump_landing_foot_advance = requested_landing_foot_advance;
        bool mpc_active = false;
        bool moving = false;
        if (jump_phase >= 0) {
            moving = run_jump_control(jump_phase, jump_height_reference,
                                      jump_velocity_reference, ready);
            mpc_active = jump_phase == 2 || jump_phase == 5 ||
                         (jump_phase == 6 && jump_phase_progress < 1.0 - 1e-9);
        } else {
            wbc.setStandRecoveryMode(false);
            wbc.setPreserveHorizontalContactWrench(false);
            wbc.setTrackHorizontalContactWrench(false);
            moving = run_standard_control(target, walking, ready);
            mpc_active = moving;
        }
        pvt.dataBusRead(state);
        if (!ready) pvt.calMotorsPVT(110.0 / 180.0 * 3.141592653589793 * dt);
        else pvt.calMotorsPVT();
        pvt.dataBusWrite(state);
        // motors_tor_out contains torque divided by gear for MuJoCo actuators.
        // Isaac's effort API takes the final link-side torque instead.
        finite_array(pvt.motor_tor_out_link.data(), kJointCount, "joint-side output torque");
        std::copy_n(pvt.motor_tor_out_link.begin(), kJointCount, out);
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        const auto peak = *std::max_element(out, out + kJointCount, [](double a, double b) {return std::abs(a) < std::abs(b);});
        diagnostics[0] = time;
        diagnostics[1] = ready;
        diagnostics[2] = state.motionState;
        diagnostics[3] = state.legState;
        diagnostics[4] = state.phi;
        diagnostics[5] = mpc_active;
        diagnostics[6] = state.qpStatus_MPC;
        diagnostics[7] = state.qp_cpuTime_MPC;
        diagnostics[8] = state.qp_nWSR_MPC;
        diagnostics[9] = state.qp_status;
        diagnostics[10] = state.qp_cpuTime;
        diagnostics[11] = state.qp_nWSR;
        diagnostics[12] = elapsed;
        diagnostics[13] = state.FL_est[2];
        diagnostics[14] = state.FR_est[2];
        diagnostics[15] = std::abs(peak);
        diagnostics[16] = joy.vx_L;
        diagnostics[17] = joy.vy_L;
        diagnostics[18] = joy.wz_L;
        diagnostics[21] = pos[2];
        diagnostics[22] = state.base_rpy[0];
        diagnostics[23] = state.base_rpy[1];
        diagnostics[24] = state.base_rpy[2];
        diagnostics[25] = jump_phase < 0 && moving && external_wrench_enabled &&
            external_mpc_status == 0 && external_wrench_matches_support();
        diagnostics[26] = external_mpc_status;
        diagnostics[27] = external_mpc_solve_s;
        diagnostics[28] = external_mpc_iterations;
        diagnostics[29] = static_cast<double>(external_mpc_updates);
        diagnostics[30] = state.legStateNext;
        diagnostics[31] = jump_phase >= 0;
        diagnostics[32] = jump_phase;
        diagnostics[33] = jump_height_reference;
        diagnostics[34] = jump_velocity_reference;
        diagnostics[35] = jump_phase_progress;
        diagnostics[36] = jump_landing_foot_advance;
        for (int axis = 0; axis < 3; ++axis) {
            diagnostics[37 + axis] = state.pCoM_W[axis];
            diagnostics[40 + axis] = state.fe_l_pos_W[axis];
            diagnostics[43 + axis] = state.fe_r_pos_W[axis];
            diagnostics[46 + axis] = wbc.pCoMDes[axis];
            diagnostics[49 + axis] = wbc.fe_l_pos_des_W[axis];
            diagnostics[52 + axis] = wbc.fe_r_pos_des_W[axis];
        }
        previous_jump_phase = jump_phase;
        last_time = time;
        ++steps;
    }
};

Controller& checked(void* handle) {
    require(handle != nullptr, "Controller handle is null");
    return *static_cast<Controller*>(handle);
}
template<class F> int protect(F&& f) {
    try { last_error.clear(); f(); return 0; }
    catch (const std::exception& e) { last_error = e.what(); return -1; }
    catch (...) { last_error = "Unknown C++ exception"; return -1; }
}
} // namespace

extern "C" {
void* ol_create(const char* repo_root, double dt, double mpc_dt, double swing_time,
                double step_height, double stand_height) {
    void* result = nullptr;
    protect([&] {
        require(repo_root && *repo_root, "repo_root is empty");
        const double parameters[] = {dt, mpc_dt, swing_time, step_height, stand_height};
        finite_array(parameters, 5, "controller parameters");
        require(dt > 0 && dt <= 0.01 && mpc_dt >= dt && mpc_dt <= 0.1,
                "Require 0 < dt <= .01 and dt <= mpc_dt <= .1 seconds");
        require(std::abs(mpc_dt / dt - std::round(mpc_dt / dt)) < 1e-6,
                "mpc_dt must be an integer multiple of dt");
        require(swing_time > mpc_dt && step_height >= 0 && step_height < 0.5,
                "Invalid swing_time or step_height");
        require(stand_height > 0.7 && stand_height < 1.3, "stand_height must be base height in metres (default 1.08)");
        const std::string root = std::filesystem::absolute(repo_root).lexically_normal().string();
        require(std::filesystem::is_regular_file(root + "/models/AzureLoong.urdf"), "AzureLoong.urdf is missing");
        require(std::filesystem::is_regular_file(root + "/common/joint_ctrl_config.json"), "Joint controller JSON is missing");
        result = new Controller(root, dt, mpc_dt, swing_time, step_height, stand_height);
    });
    return result;
}
void ol_destroy(void* handle) { delete static_cast<Controller*>(handle); }
const char* ol_last_error() { return last_error.c_str(); }
int ol_joint_count(void* handle) {
    return protect([&] { (void)checked(handle); }) == 0 ? kJointCount : -1;
}
const char* ol_joint_name(void* handle, int index) {
    const char* value = nullptr;
    protect([&] { auto& c = checked(handle); require(index >= 0 && index < kJointCount, "Joint index out of range"); value = c.kin.motorName[index].c_str(); });
    return value;
}
int ol_initial_joints(void* handle, double* out, int count) {
    return protect([&] { auto& c = checked(handle); require(out && count == kJointCount, "Initial joint array must have length 31"); std::copy_n(c.initial_q.data() + 7, kJointCount, out); });
}
int ol_step(void* handle, double time, const double* joints, const double* joint_vel,
            const double* base_pos, const double* quat, const double* linear,
            const double* angular, const double* target, int walking, double* out) {
    if (out) std::fill_n(out, kJointCount, 0.0);
    return protect([&] {
        auto& c = checked(handle);
        require(out, "Output array is null");
        finite_array(joints, kJointCount, "joint positions");
        finite_array(joint_vel, kJointCount, "joint velocities");
        finite_array(base_pos, 3, "base position");
        finite_array(quat, 4, "base quaternion wxyz");
        finite_array(linear, 3, "world linear velocity");
        finite_array(angular, 3, "world angular velocity");
        finite_array(target, 3, "velocity command");
        require(walking == 0 || walking == 1, "walking must be 0 or 1");
        try { c.step(time, joints, joint_vel, base_pos, quat, linear, angular, target, walking != 0, out); }
        catch (...) { c.failed = true; std::fill_n(out, kJointCount, 0.0); throw; }
    });
}
int ol_step_jump(void* handle, double time, const double* joints,
                 const double* joint_vel, const double* base_pos,
                 const double* quat, const double* linear,
                 const double* angular, int phase, double height_reference,
                 double vertical_velocity_reference, double phase_progress,
                 double landing_foot_advance,
                 double* out) {
    if (out) std::fill_n(out, kJointCount, 0.0);
    return protect([&] {
        auto& c = checked(handle);
        require(out, "Output array is null");
        finite_array(joints, kJointCount, "joint positions");
        finite_array(joint_vel, kJointCount, "joint velocities");
        finite_array(base_pos, 3, "base position");
        finite_array(quat, 4, "base quaternion wxyz");
        finite_array(linear, 3, "world linear velocity");
        finite_array(angular, 3, "world angular velocity");
        const double references[] = {
            height_reference, vertical_velocity_reference, phase_progress,
            landing_foot_advance};
        finite_array(references, 4, "jump reference");
        require(phase >= 0 && phase <= 7, "Jump phase must be in [0,7]");
        require(height_reference > 0.70 && height_reference < 1.30,
                "Jump height reference must be in (0.70,1.30) metres");
        require(phase_progress >= 0.0 && phase_progress <= 1.0,
                "Jump phase progress must be in [0,1]");
        require(landing_foot_advance >= 0.0 && landing_foot_advance <= 0.20,
                "Jump landing foot advance must be in [0,0.20] metres");
        static const double zero_target[3] = {0.0, 0.0, 0.0};
        try {
            c.step(time, joints, joint_vel, base_pos, quat, linear, angular,
                   zero_target, false, out, phase, height_reference,
                   vertical_velocity_reference, phase_progress,
                   landing_foot_advance);
        } catch (...) {
            c.failed = true;
            std::fill_n(out, kJointCount, 0.0);
            throw;
        }
    });
}
int ol_set_mpc_weights(void* handle, double u, const double* l, const double* k) {
    return protect([&] {
        auto& c = checked(handle);
        require(std::isfinite(u) && u > 0, "MPC input weight must be positive and finite");
        finite_array(l, nx, "MPC L"); finite_array(k, nu, "MPC K");
        for (int i = 0; i < nx; ++i) require(l[i] >= 0, "MPC L weights must be nonnegative");
        for (int i = 0; i < nu; ++i) require(k[i] > 0, "MPC K weights must be positive");
        c.input_weight = u;
        c.weights_l = Eigen::Map<const Eigen::Matrix<double, 1, nx>>(l);
        c.weights_k = Eigen::Map<const Eigen::Matrix<double, 1, nu>>(k);
    });
}
int ol_set_external_mpc_wrench(void* handle, int enabled, const double* wrench,
                               int count, double solve_time, int status,
                               int iterations) {
    return protect([&] {
        require(enabled == 0 || enabled == 1, "External MPC enabled must be 0 or 1");
        auto& c = checked(handle);
        c.set_external_wrench(enabled != 0, wrench, count, solve_time, status, iterations);
    });
}
int ol_diagnostic_count(void* handle) {
    return protect([&] { (void)checked(handle); }) == 0 ? static_cast<int>(kDiagnostics.size()) : -1;
}
const char* ol_diagnostic_name(void* handle, int index) {
    const char* value = nullptr;
    protect([&] { (void)checked(handle); require(index >= 0 && index < static_cast<int>(kDiagnostics.size()), "Diagnostic index out of range"); value = kDiagnostics[index]; });
    return value;
}
int ol_diagnostics(void* handle, double* out, int count) {
    return protect([&] { auto& c = checked(handle); require(out && count == static_cast<int>(kDiagnostics.size()), "Diagnostic output has wrong length"); std::copy(c.diagnostics.begin(), c.diagnostics.end(), out); });
}
int ol_control_signal_count(void* handle) {
    int count = -1;
    protect([&] { count = static_cast<int>(checked(handle).control_signal_names.size()); });
    return count;
}
const char* ol_control_signal_name(void* handle, int index) {
    const char* value = nullptr;
    protect([&] {
        auto& c = checked(handle);
        require(index >= 0 && index < static_cast<int>(c.control_signal_names.size()),
                "Control signal index out of range");
        value = c.control_signal_names[index].c_str();
    });
    return value;
}
int ol_control_signals(void* handle, double* out, int count) {
    return protect([&] { checked(handle).write_control_signals(out, count); });
}
} // extern C
