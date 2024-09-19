#ifndef __ROBOT_JOINT_CONTROLLER_HPP__
#define __ROBOT_JOINT_CONTROLLER_HPP__

#include <ester_common/ester_enums.hpp>

#include "ester_kinematics/leg_controller.hpp"
#include "ester_kinematics/spine_kinematics.hpp"

#include <memory>

namespace ester_kinematics {

/**
 * @brief Class to determine joint torque, velocity and position targets
 */
class RobotJointController {
public:
    /**
     * @brief Constructor
     */
    RobotJointController(bool silence_errors = false);

    /**
     * @brief Register a joint with the controller
     */
    void register_joint(
        const ester_common::AllJointId &jnt,
        double *tgt_pos, double *tgt_vel, double *tgt_eff, bool *torque_ctrl);

    /**
     * @brief Update position, velocity and torque targets of the
     *        robot
     * 
     * @param pos The current joint positions
     * @param foot_positions The target foot positions
     * @param foot_grfs The target foot forces, ignored if in swing
     * @param foot_stances True in stance, false in swing
     * @param dt Expected update duration (s)
    */
    void control_robot(
        const ester_common::AllJointMap<double> &pos,
        const ester_common::LegMap<Eigen::Vector3d> &foot_positions,
        const ester_common::LegMap<Eigen::Vector3d> &foot_grfs,
        const ester_common::LegMap<bool> &foot_stances,
        double dt) const;

private:
    /**
     * @brief Counter of how many joints have been registered
     */
    unsigned int registered_joints_;

    /**
     * @brief Map of pointers to position targets
     */
    ester_common::AllJointMap<double*> tgt_pos_;

    /**
     * @brief Map of pointers to velocity targets
     */
    ester_common::AllJointMap<double*> tgt_vel_;

    /**
     * @brief Map of pointers to torque targets
     */
    ester_common::AllJointMap<double*> tgt_eff_;

    /**
     * @brief Map of pointers to control type flag
     */
    ester_common::AllJointMap<bool*> torque_ctrl_;

    /**
     * @brief Solver for leg control
     */
    ester_common::LegMap<std::unique_ptr<LegController>> leg_ctrl_;
    /**
     * @brief Solver for spine kinematics
     */
    std::unique_ptr<SpineKinematics> spine_;
};

}

#endif // __ROBOT_JOINT_CONTROLLER_HPP__