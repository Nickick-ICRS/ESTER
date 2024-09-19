#ifndef __SPINE_KINEMATICS_HPP__
#define __SPINE_KINEMATICS_HPP__

#include <ester_common/ester_enums.hpp>

#include <Eigen/Geometry>
#include <dart/dart.hpp>

namespace ester_kinematics {

class SpineKinematics {
public:
    /**
     * @brief Constructor
     */
    SpineKinematics();

    /**
     * @brief Sets the spine joint positions
     *
     * @param pos The joint positions
     */
    void set_spine_joint_positions(const ester_common::SpineJointMap<double> &pos);

    /**
     * @brief Get the shoulder transform for a given leg id
     * @details Expects set_spine_joint_positions to have been called prior
     *
     * @param id Which leg we want the shoulder transform from
     *
     * @returns The transform from base_link to <leg>_hip_aa_link
     */
    Eigen::Isometry3d get_shoulder_transform(ester_common::LegId id);

    /**
     * @brief Transform a point (e.g. foot) from base_link to shoulder frame
     * @details Expects set_spine_joint_positions to have been called prior
     *
     * @param id Which leg's shoulder frame to transform into
     * @param point_in_base_frame The point to transform
     *
     * @returns The point in the leg's shoulder frame
     */
    Eigen::Vector3d point_to_shoulder_frame(
        ester_common::LegId id, const Eigen::Vector3d &point_in_base_frame);

    /**
     * @brief Transform a vector (e.g. GRF) from base_link to shoulder frame
     * @details Expects set_spine_joint_positions to have been called prior
     *
     * @param id Which leg's shoulder frame to transform into
     * @param vector_in_base_frame The vector to transform
     *
     * @returns The vector in the leg's shoulder frame
     */
    Eigen::Vector3d vector_to_shoulder_frame(
        ester_common::LegId id, const Eigen::Vector3d &vector_in_base_frame);

    /**
     * @brief Transform a point (e.g. foot) from shoulder frame to base_link
     * @details Expects set_spine_joint_positions to have been called prior
     *
     * @param id Which leg's shoulder frame to transform from
     * @param point_in_base_frame The point to transform
     *
     * @returns The point in the base_link frame
     */
    Eigen::Vector3d point_to_base_frame(
        ester_common::LegId id, const Eigen::Vector3d &point_in_shoulder_frame);

    /**
     * @brief Transform a vector (e.g. GRF) from shoulder frame to base_link
     * @details Expects set_spine_joint_positions to have been called prior
     *
     * @param id Which leg's shoulder frame to transform from
     * @param vector_in_base_frame The vector to transform
     *
     * @returns The vector in the base_link frame
     */
    Eigen::Vector3d vector_to_base_frame(
        ester_common::LegId id, const Eigen::Vector3d &vector_in_shoulder_frame);

private:
    dart::dynamics::SkeletonPtr skel_;
    ester_common::SpineJointMap<dart::dynamics::JointPtr> spine_joints_;
    ester_common::LegMap<dart::dynamics::BodyNodePtr> shoulders_;
};

} // namespace ester_kinematics

#endif // __SPINE_KINEMATICS_HPP__
