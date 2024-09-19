#include "ester_kinematics/spine_kinematics.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

using namespace ester_common;
using namespace ester_kinematics;

SpineKinematics::SpineKinematics() {
    std::string ester_description = ros::package::getPath(
        "ester_description");

    std::string urdf_path = ester_description + "/urdf/ester.urdf";
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
    skel_ = loader.parseSkeleton(
        dart::common::Uri(urdf_path), nullptr,
        dart::utils::DartLoader::Flags::FIXED_BASE_LINK);
    skel_->setGravity(Eigen::Vector3d(0, 0, 0));

    spine_joints_[SpineJointId::FRONT_PITCH] =
        skel_->getJoint("front_spine_pitch_joint");
    spine_joints_[SpineJointId::FRONT_YAW] =
        skel_->getJoint("front_spine_yaw_joint");
    spine_joints_[SpineJointId::REAR_PITCH] =
        skel_->getJoint("rear_spine_pitch_joint");
    spine_joints_[SpineJointId::REAR_YAW] =
        skel_->getJoint("rear_spine_yaw_joint");

    shoulders_[LegId::FL] = skel_->getBodyNode("front_left_hip_aa_link");
    shoulders_[LegId::FR] = skel_->getBodyNode("front_right_hip_aa_link");
    shoulders_[LegId::RL] = skel_->getBodyNode("rear_left_hip_aa_link");
    shoulders_[LegId::RR] = skel_->getBodyNode("rear_right_hip_aa_link");
}

void SpineKinematics::set_spine_joint_positions(const SpineJointMap<double> &pos) {
    auto set_joint = [](auto &dart_joint, double pos) {
        dart_joint->setPosition(0, pos);
        // Currently only doing kinematics so this is not required
        // dart_joint.setVelocity(0, 0);
        // dart_joint.setForce(0, 0);
    };
    for (const auto &pair : pos) {
        set_joint(spine_joints_[pair.first], pair.second);
    }
    // DART will automatically perform the necessary computations so this
    // may be unnecessary
    // skel_->computeForwardKinematics();
}

Eigen::Isometry3d SpineKinematics::get_shoulder_transform(LegId id) {
    return shoulders_[id]->getTransform().inverse();
}

Eigen::Vector3d SpineKinematics::point_to_shoulder_frame(
    LegId id, const Eigen::Vector3d &point_in_base_frame)
{
    auto tf = get_shoulder_transform(id);
    return tf * point_in_base_frame;
}

Eigen::Vector3d SpineKinematics::vector_to_shoulder_frame(
    LegId id, const Eigen::Vector3d &vector_in_base_frame)
{
    auto tf = get_shoulder_transform(id);
    return tf.linear() * vector_in_base_frame;
}

Eigen::Vector3d SpineKinematics::point_to_base_frame(
    LegId id, const Eigen::Vector3d &point_in_shoulder_frame)
{
    auto tf = get_shoulder_transform(id).inverse();
    return tf * point_in_shoulder_frame;
}

Eigen::Vector3d SpineKinematics::vector_to_base_frame(
    LegId id, const Eigen::Vector3d &vector_in_shoulder_frame)
{
    auto tf = get_shoulder_transform(id).inverse();
    return tf.linear() * vector_in_shoulder_frame;
}