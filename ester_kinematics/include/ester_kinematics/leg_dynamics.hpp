#ifndef __LEG_DYNAMICS_HPP__
#define __LEG_DYNAMICS_HPP__

#include "ester_common/ester_enums.hpp"

#include <Eigen/Geometry>
#include <dart/dart.hpp>

namespace ester_kinematics {

/**
 * @brief Abstract base class
 */
class LegDynamicsBase {
public:
    LegDynamicsBase(const ester_common::LegId &) {};

    /**
     * @brief Helper data struct
     */
    struct IdResult {
        ester_common::LegJointMap<double> eff;
        ester_common::LegJointMap<double> vel;
    };

    /**
     * @brief Helper data struct
     */
    struct IkResult {
        ester_common::LegJointMap<double> pos;
        std::vector<ester_common::LegJointMap<double>> alternative_solutions;
        bool reachable;
    };

    /**
     * @brief Solve inverse dynamics for the leg
     *
     * @param ja The current joint angles of the leg
     * @param jv The current joint velocities of the leg
     * @param eef_vel Velocity of the end effector in the shoulder frame
     * @param grf The requested ground reaction force
     *
     * @returns IdResult containing joint torques, accelerations, velocities
     */
    virtual IdResult solve_id(
        const ester_common::LegJointMap<double> &ja,
        const ester_common::LegJointMap<double> &jv,
        const Eigen::Vector3d &eef_vel,
        const Eigen::Vector3d &grf) const = 0;

    /**
     * @brief Solve inverse kinematics for the leg
     *
     * @param tgt The target position of the foot in the shoulder frame
     * @param current_pos Analytical solutions closest to this value are prefered
     *
     * @returns IkResult containing joint positions and reachability
     */
    virtual IkResult solve_ik(
        const Eigen::Vector3d &tgt,
        const ester_common::LegJointMap<double> &current_pos={})
        const = 0;

    /**
     * @brief Solve forward kinematics for the leg
     *
     * @param ja The joint angles of the leg
     *
     * @returns Pose of the foot in the shoulder frame
     */
    virtual Eigen::Isometry3d solve_fk(
        const ester_common::LegJointMap<double> &ja) const = 0;

    /**
     * @brief Solve forward dynamics for the leg
     *
     * @param pos Joint angles
     * @param vel Joint velocities
     * @param eff Joint efforts
     * @param foot_vel If this is not none, the foot velocity will also
     *                 be calculated and stored here
     *
     * @returns Force at the end effector in the shoulder frame
     */
    virtual Eigen::Vector3d solve_fd(
        const ester_common::LegJointMap<double> &pos,
        const ester_common::LegJointMap<double> &vel,
        const ester_common::LegJointMap<double> &eff,
        Eigen::Vector3d *foot_vel=nullptr) const = 0;
};

/**
 * @brief Standard implementation solving forwards and backwards
 *        kinematics and dynamics
 */
class LegDynamics : public LegDynamicsBase {
public:
    /**
     * @brief Constructor
     *
     * @param id Which leg we solve dynamics for
     */
    LegDynamics(ester_common::LegId id);

    /**
     * @brief Solve inverse dynamics for the leg
     *
     * @param ja The current joint angles of the leg
     * @param jv The current joint velocities of the leg
     * @param eef_vel Velocity of the end effector in the shoulder frame
     * @param grf The requested ground reaction force
     *
     * @returns IdResult containing joint torques, accelerations, velocities
     */
    IdResult solve_id(
        const ester_common::LegJointMap<double> &ja,
        const ester_common::LegJointMap<double> &jv,
        const Eigen::Vector3d &eef_vel,
        const Eigen::Vector3d &grf) const override;

    /**
     * @brief Solve inverse kinematics for the leg
     *
     * @param tgt The target position of the foot in the shoulder frame
     * @param current_pos Analytical solutions closest to this value are prefered
     *
     * @returns IkResult containing joint positions and reachability
     */
    IkResult solve_ik(
        const Eigen::Vector3d &tgt,
        const ester_common::LegJointMap<double> &current_pos={})
        const override;

    /**
     * @brief Solve forward kinematics for the leg
     *
     * @param ja The joint angles of the leg
     *
     * @returns Pose of the foot in the shoulder frame
     */
    Eigen::Isometry3d solve_fk(
        const ester_common::LegJointMap<double> &ja) const override;

    /**
     * @brief Solve forward dynamics for the leg
     *
     * @param pos Joint angles
     * @param vel Joint velocities
     * @param eff Joint efforts
     * @param foot_vel If this is not none, the foot velocity will also
     *                 be calculated and stored here
     *
     * @returns Force at the end effector in the shoulder frame
     */
    Eigen::Vector3d solve_fd(
        const ester_common::LegJointMap<double> &pos,
        const ester_common::LegJointMap<double> &vel,
        const ester_common::LegJointMap<double> &eff,
        Eigen::Vector3d *foot_vel=nullptr) const override;

    const dart::dynamics::SkeletonPtr &get_leg() const { return leg_; };

private:
    dart::dynamics::SkeletonPtr leg_;
    dart::dynamics::JointPtr hip_roll_;
    dart::dynamics::JointPtr hip_pitch_;
    dart::dynamics::JointPtr knee_;
    dart::dynamics::BodyNodePtr foot_;
    dart::dynamics::InverseKinematicsPtr ik_;
    dart::dynamics::InverseKinematicsPtr ik_analytical_;

    bool pos_knee_;
};

} // namespace ester_kinematics

#endif // __LEG_DYNAMICS_HPP__
