#ifndef __GAIT_MANAGER_HPP__
#define __GAIT_MANAGER_HPP__

#include <unordered_map>
#include <Eigen/Geometry>

#include "ester_mpc/trajectories.hpp"
#include "ester_mpc/trajectory_planner.hpp"

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace Eigen
{
using Vector6d = Matrix<double, 6, 1>;
} // namespace Eigen

namespace ester_mpc
{

enum class GaitType {
    STATIONARY = 0,
    WALKING = 1,
    TROTTING = 2,
    BOUNDING = 3,
    PACING = 4,
    PRONKING = 5,
};

std::string str(const GaitType &gait);

std::ostream& operator<<(std::ostream &os, const GaitType &gait);


class GaitManager {
public:
    /**
     * @brief Constructor
     * 
     * @param feet_pos Current position of the robot feet w.r.t the body frame
     * @param config ROS config for gaits and trajectory parameters
     *               "/ester/gait_manager"
     * @param nh Optional pointer to ROS nodehandle - if not-null then debug details
     *           will be passed to ROS
     */
    GaitManager(
        const ester_common::LegMap<Eigen::Vector3d> &feet_pos,
        const XmlRpc::XmlRpcValue &config,
        const std::shared_ptr<ros::NodeHandle> &nh=nullptr);

    /**
     * @brief Default destructor
     */
    virtual ~GaitManager() = default;

    /**
     * @brief Update the current foot trajectories for MPC
     * 
     * @param dt Time since last update
     * @param current_vel Current velocity of the robot body (world frame, angular first)
     * @param cmd_vel Command velocity of the robot body (body frame, angular first)
     * @param cmd_height Command z height of the robot body (world frame)
     * @param foot_p Current foot positions (body frame)
     * @return Shared pointer containing the trajectories
     */
    const std::shared_ptr<Trajectories> &update_trajectories(
        double dt, const Eigen::Vector6d &current_vel,
        const Eigen::Vector6d &cmd_vel, const double cmd_height,
        const ester_common::LegMap<Eigen::Vector3d> &foot_p);

    /**
     * @brief Get the trajectories without recalculating
     * 
     * @return The trajectories
     */
    const std::shared_ptr<Trajectories> &get_trajectories() const {
        return trajs_;
    };

    /**
     * @brief Reset the gait manager and parameters back to their initial state
     * 
     * @param current_feet_pos The current positions of the feet
     * @param reset_gait If true the gait and phase counters will be reset to
     *                   STATIONARY. Default false
     */
    void reset(
        const ester_common::LegMap<Eigen::Vector3d> &current_feet_pos,
        bool reset_gait=false);

    /**
     * @brief Set the type of gait to generate trajectories for.
     * @details Will transition from the current gait to the requested gait
     * 
     * @param gait The gait type.
     */
    void set_gait_type(const GaitType &gait);

    /**
     * @brief Return to automatically determining the best gait
     * 
     * @param gait The gait to return to, MUST be one of:
     *      STATIONARY
     *      WALKING
     *      TROTTING
     *      BOUNDING
     */
    void return_to_automatic_gait_control(const GaitType &gait);

    double get_phase(const ester_common::LegId &id) { return t_params_.phase_counter[id]; };

    double get_phase_dur() { return t_params_.swing_dur + t_params_.stance_dur; };

protected:
    /**
     * @brief Calls the relevant function from below for gait "gait"
     *
     * @param gait The gait requested
     */
    void enter_gait(const GaitType &gait);

    /**
     * @brief Signal a change to standing still "gait"
     */
    void enter_stationary();

    /**
     * @brief Signal a change to walking gait
     */
    void enter_walking();

    /**
     * @brief Signal a change to trotting gait
     */
    void enter_trotting();

    /**
     * @brief Signal a change to pacing gait
     */
    void enter_pacing();

    /**
     * @brief Signal a change to pronking gait
     */
    void enter_pronking();

    /**
     * @brief Signal a change to bounding gait
     */
    void enter_bounding();

    /**
     * @brief Update gait phases in transition
     *
     * @param dt Time since last update
     */
    void update_gait_phase_transition(double dt);

    /**
     * @brief Update gait phases
     * 
     * @param dt Time since last update
     */
    void update_gait_phase(double dt);

    /**
     * @brief Generate gait for stationary gait type
     * 
     * @param dt Time since last update
     * @param cmd_height Current command height of the body
     * @param body_vel Current velocity of the body
     * @param foot_p Current foot positions
     */
    void prepare_stationary_gait(
        double dt, double cmd_height, const Eigen::Vector6d &body_vel,
        const ester_common::LegMap<Eigen::Vector3d> &foot_p);

    /**
     * @brief Generate gait for moving gait type
     *
     * @param dt Time since last update
     * @param cmd_height Current command height of the body
     * @param body_vel Current velocity of the body
     * @param cmd_vel Current velocity of the body
     * @param foot_p Current foot positions
     */
    void prepare_moving_gait(
        double dt, double cmd_height, const Eigen::Vector6d &body_vel,
        const Eigen::Vector6d &cmd_vel,
        const ester_common::LegMap<Eigen::Vector3d> &foot_p);

    /**
     * @brief Trajectory parameters and state
     */
    TrajectoryParameters t_params_;

    /**
     * @brief Most recently generated trajectories
     */
    std::shared_ptr<Trajectories> trajs_;

    /**
     * @brief Actually generates the trajectories
     */
    std::unique_ptr<TrajectoryPlanner> tp_;

    /**
     * @brief Currently executing gait
     */
    GaitType current_gait_;

    /**
     * @brief Target gait to switch to
     */
    GaitType target_gait_;

    /**
     * @brief Target phases for state transitions
     */
    ester_common::LegMap<double> target_phases_;

    /**
     * @brief If true, only change gait type when requested
     * @details If false, ignore gait type changes and determine best gait
     *          automatically based on speed
     */
    bool manual_gait_control_;

    /**
     * @brief If true, some gaits will consider spine positions
     * @details If false no gaits will consider spine positions
     */
    bool consider_spine_positions_;

    /**
     * @brief How far in the future to predict the trajectories
     */
    double traj_future_;

    /**
     * @brief Maps for gait parameters for each gait type
     */
    std::unordered_map<GaitType, double> stance_durs_map_;
    std::unordered_map<GaitType, double> ratios_map_;
    std::unordered_map<GaitType, double> Kpitch_map_;
    std::unordered_map<GaitType, double> Kyaw_map_;
    std::unordered_map<GaitType, ester_common::LegMap<double>> phase_map_;

    bool pub_to_ros_;
    ros::Publisher phase_pub_;
};
    
} // namespace ester_mpc


#endif // __GAIT_MANAGER_HPP__