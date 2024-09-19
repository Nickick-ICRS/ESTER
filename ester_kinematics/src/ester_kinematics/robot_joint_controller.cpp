#include "ester_kinematics/robot_joint_controller.hpp"

using namespace ester_common;
using namespace ester_kinematics;

RobotJointController::RobotJointController(bool silence_errors) :registered_joints_(0) {
    for (const auto &id : ALL_LEG_IDS) {
        leg_ctrl_[id].reset(new LegController(id, silence_errors));
    }
    spine_.reset(new SpineKinematics);
}

void RobotJointController::register_joint(
    const AllJointId &jnt,
    double *tgt_pos, double *tgt_vel, double *tgt_eff, bool *torque_ctrl)
{
    if (tgt_pos_.count(jnt) == 0) {
        registered_joints_++;
    }
    else {
        ROS_WARN_STREAM(
            "Re-registering " << jnt << " in RobotJointController...");
    }
    tgt_pos_[jnt] = tgt_pos;
    tgt_vel_[jnt] = tgt_vel;
    tgt_eff_[jnt] = tgt_eff;
    torque_ctrl_[jnt] = torque_ctrl;
}

void RobotJointController::control_robot(
    const AllJointMap<double> &pos,
    const LegMap<Eigen::Vector3d> &foot_positions,
    const LegMap<Eigen::Vector3d> &foot_grfs,
    const LegMap<bool> &foot_stances,
    double dt) const
{
    if (registered_joints_ != 16) {
        throw std::runtime_error("Needs 16 registered joints!");
    }
    SpineJointMap<double> sjm;
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        sjm[jnt] = pos.at(full_joint_id(jnt));
        // We don't control the spine in this method
    }
    spine_->set_spine_joint_positions(sjm);
    for (const auto &leg : ALL_LEG_IDS) {
        LegJointMap<double> ljm;
        for (const auto &jnt : ALL_LEG_JOINTS) {
            ljm[jnt] = pos.at(full_joint_id(leg, jnt));
        }
        auto tf = spine_->get_shoulder_transform(leg);
        auto ctrl = leg_ctrl_.at(leg)->control_leg(
            foot_positions.at(leg), foot_grfs.at(leg),
            ljm, tf, dt, foot_stances.at(leg));
        if (!ctrl.valid_data) {
            // Disable torque control on kinematics failure for safety
            for (const auto &jnt : ALL_LEG_JOINTS) {
                auto id = full_joint_id(leg, jnt);
                *tgt_vel_.at(id) = 0;
                *tgt_eff_.at(id) = 0;
                *torque_ctrl_.at(id) = false;
            }
            continue;
        }
        for (const auto &jnt : ALL_LEG_JOINTS) {
            auto id = full_joint_id(leg, jnt);
            *tgt_pos_.at(id) = ctrl.pos[jnt];
            *tgt_vel_.at(id) = ctrl.vel[jnt];
            *tgt_eff_.at(id) = ctrl.eff[jnt];
            *torque_ctrl_.at(id) = ctrl.use_torque_control;
        }
    }
    /*
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        auto id = full_joint_id(jnt);
        *tgt_pos_.at(id) = 0;
        *tgt_vel_.at(id) = 0;
        *tgt_eff_.at(id) = 0;
        *torque_ctrl_.at(id) = false;
    }
    */
}