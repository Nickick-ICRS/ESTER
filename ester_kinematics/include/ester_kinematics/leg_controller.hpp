#ifndef __LEG_CONTROLLER_HPP__
#define __LEG_CONTROLLER_HPP__

#include "ester_common/ester_enums.hpp"
#include "ester_kinematics/leg_dynamics.hpp"

#include <Eigen/Geometry>
#include <ros/ros.h>

namespace ester_kinematics {

class LegController {
public:
    /**
     * @brief Constructor
     *
     * @param id Which leg we're controlling
     */
    LegController(ester_common::LegId id, bool silence_errors=false);

    /**
     * @brief Helper struct to show tgt position, velocity to travel at
     *        and expected (i.e. feedforward) torque at each joint
     */
    struct JointControlParameters {
        ester_common::LegJointMap<double> pos;
        ester_common::LegJointMap<double> vel;
        ester_common::LegJointMap<double> eff;
        bool use_torque_control;
        /**
         * @brief Set to false if we don't have enough data to solve yet
         */
        bool valid_data = true;
    };

    /**
     * @brief Calculates the expected joint states at a given time
     *
     * @param tgt_xyz Foot target in body frame
     * @param tgt_grf Required ground reaction force in body frame
     * @param current_pos Current position of each joint (radians)
     * @param tf Transform from body to this leg's shoulder frame
     * @param dt Time between current pos and target pos
     * @param stance Whether the leg is in stance or not
     *
     * @returns pos, vel, eff for each joint
     */
    JointControlParameters control_leg(
        const Eigen::Vector3d &target_xyz, const Eigen::Vector3d &grf,
        const ester_common::LegJointMap<double> &current_pos,
        const Eigen::Isometry3d &tf,
        double dt, bool stance) const;

    /**
     * @brief Get access to the dynamics solver
     */
    const std::shared_ptr<LegDynamics> get_solver() const {
        return dyn_;
    };

private:
    /**
     * @brief Solver for leg kinematics and dynamics
     */
    std::shared_ptr<LegDynamics> dyn_;

    ester_common::LegId id_;

    bool silence_errors_;
};

} // namespace ester_kinematics

#endif // __LEG_CONTROLLER_HPP__
