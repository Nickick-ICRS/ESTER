#include "ester_kinematics/robot_model.hpp"

using namespace ester_kinematics;
using namespace ester_common;

Eigen::Vector3d RobotModel::calculate_grf(
    const LegId &leg, const LegJointMap<double> &pos,
    const LegJointMap<double> &/*vel*/, const LegJointMap<double> &eff,
    const SpineJointMap<double> &spine_pos) const
{
    // ** Dynamics doesn't work great with non-zero velocities **
    LegJointMap<double> zero_vel;
    for (const auto &jnt : ALL_LEG_JOINTS) {
        zero_vel[jnt] = 0;
    }
    Eigen::Vector3d f_hip = dynamics_.at(leg)->solve_fd(pos, zero_vel, eff);

    spine_kinematics_->set_spine_joint_positions(spine_pos);
    return spine_kinematics_->vector_to_base_frame(leg, f_hip);
}

Eigen::Vector3d RobotModel::calculate_fp(
    const LegId &leg, const LegJointMap<double> &pos,
    const SpineJointMap<double> &spine_pos) const
{
    Eigen::Vector3d xyz_hip = dynamics_.at(leg)->solve_fk(pos).translation();

    spine_kinematics_->set_spine_joint_positions(spine_pos);
    return spine_kinematics_->point_to_base_frame(leg, xyz_hip);
}