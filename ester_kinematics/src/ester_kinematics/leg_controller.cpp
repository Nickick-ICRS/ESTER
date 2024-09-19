#include "ester_kinematics/leg_controller.hpp"

using namespace ester_common;
using namespace ester_kinematics;

LegController::LegController(LegId id, bool silence_errors)
    :id_(id), silence_errors_(silence_errors)
{
    dyn_.reset(new LegDynamics(id));
}

LegController::JointControlParameters LegController::control_leg(
    const Eigen::Vector3d &tgt_xyz, const Eigen::Vector3d &tgt_grf,
    const LegJointMap<double> &current_pos, const Eigen::Isometry3d &tf,
    double dt, bool stance) const
{
    JointControlParameters result;
    result.valid_data = true;

    // Account for shoulder transform
    Eigen::Vector3d tgt_xyz_shoulder = tf * tgt_xyz;
    Eigen::Vector3d tgt_grf_shoulder = tf.linear() * tgt_grf;

    auto ik_res = dyn_->solve_ik(tgt_xyz_shoulder, current_pos);
    if (!ik_res.reachable) {
        if (!silence_errors_) {
            ROS_WARN_STREAM(
                "[LegController_" << str(id_)
                << "] Requested leg target unreachable: "
                << tgt_xyz.transpose());
        }
        result.valid_data = false;
        return result;
    }
    // Set calculated position from inverse kinematics
    result.pos = ik_res.pos;

    // Set calculated velocity from target and current positions
    auto set_vel = [&result, &current_pos, &dt](const auto &id) {
        result.vel[id] = (result.pos[id] - current_pos.at(id)) / dt;
    };
    set_vel(LegJointId::HIP_ROLL);
    set_vel(LegJointId::HIP_PITCH);
    set_vel(LegJointId::KNEE);

    if (!stance) {
        // We're in swing, we don't need to solve for joint torques
        result.use_torque_control = false;
        return result;
        // PD for torque control
        double kp = 1.;
        double kd = 0.1;
        for (const auto &jnt : ALL_LEG_JOINTS) {
            double err_p = result.pos[jnt] - current_pos.at(jnt) ;
            double err_v = err_p / dt;
            result.eff[jnt] = kp * err_p + kd * err_v;
        }
        tgt_grf_shoulder = Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d current_xyz = dyn_->solve_fk(current_pos).translation();
    Eigen::Vector3d foot_vel = (tgt_xyz_shoulder - current_xyz) / dt;

    // We're in stance, calculate torques and set torque control
    result.use_torque_control = true;

    LegJointMap<double> expected_vels;
    for (const auto &jnt : ALL_LEG_JOINTS) {
        // TODO: Due to large foot mass approximation, ID doesn't seem to work
        // with non zero joint velocities
        expected_vels[jnt] = 0;
    }

    // Set calculated joint torques from inverse dynamics
    auto id_res = dyn_->solve_id(
        current_pos/*result.pos*/, expected_vels, foot_vel, tgt_grf_shoulder);
    for (const auto &jnt : ALL_LEG_JOINTS) {
        if (stance) {
            // Initialise 0
            result.eff[jnt] = 0;
        }
        result.eff[jnt] += id_res.eff[jnt];
    }

    return result;
}
