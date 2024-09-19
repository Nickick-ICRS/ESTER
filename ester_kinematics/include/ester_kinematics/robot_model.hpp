#ifndef __ROBOT_MODEL_HPP__
#define __ROBOT_MODEL_HPP__

#include "ester_kinematics/leg_dynamics.hpp"
#include "ester_kinematics/spine_kinematics.hpp"

namespace ester_kinematics {

class RobotModel {
public:
    /**
     * @brief Constructor
     */
    template<class dyn = LegDynamics> RobotModel(
        const ester_common::LegMap<std::shared_ptr<dyn>> &dyns)
    {
        for (const auto & id : ester_common::ALL_LEG_IDS) {
            if (dyns.find(id) != dyns.end() && dyns.at(id) != nullptr) {
                dynamics_[id] = dyns.at(id);
            }
            else {
                std::cerr << "[RobotModel] Creating new dynamics solver for leg "
                          << id << std::endl;
                dynamics_[id].reset(new dyn(id));
            }
        }

        spine_kinematics_.reset(new SpineKinematics());
    };

    /**
     * @brief Default no-argument constructor
     */
    RobotModel() : RobotModel(
        ester_common::LegMap<std::shared_ptr<LegDynamics>>()) {};

    /**
     * @brief Calculates the grf at a foot given the joint torques
     *
     * @param leg Which leg to solve
     * @param pos Joint positions of the leg
     * @param vel Joint velcoities of the leg
     * @param eff Joint efforts of the leg
     * @param spine_pos Position of the spine
     *
     * @returns The grf in the spine_center_link frame
     */
    Eigen::Vector3d calculate_grf(
        const ester_common::LegId &leg,
        const ester_common::LegJointMap<double> &pos,
        const ester_common::LegJointMap<double> &vel,
        const ester_common::LegJointMap<double> &eff,
        const ester_common::SpineJointMap<double> &spine_pos) const;

    /**
     * @brief Calculates the foot position given the joint positions
     *
     * @param leg Which leg to solve
     * @param pos Joint positions of the leg
     * @param spine_pos Position of the spine
     *
     * @returns The position of the foot in the spine_center_link frame
     */
    Eigen::Vector3d calculate_fp(
        const ester_common::LegId &leg,
        const ester_common::LegJointMap<double> &pos,
        const ester_common::SpineJointMap<double> &spine_pos) const;

private:
    /**
     * @brief Leg dynamics solver
     */
    std::map<ester_common::LegId, std::shared_ptr<LegDynamicsBase>> dynamics_;

    /**
     * @brief Spine kinematics solver
     */
    std::shared_ptr<SpineKinematics> spine_kinematics_;
};

} // ns ester_kinematics

#endif // __ROBOT_MODEL_HPP__
