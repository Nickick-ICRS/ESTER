#ifndef __MPC_MANAGER_HPP__
#define __MPC_MANAGER_HPP__

#include <thread>
#include <mutex>

#include <ester_common/ester_enums.hpp>
#include <ester_kinematics/robot_model.hpp>

#include "ester_mpc/mpc_base.hpp"
#include "ester_mpc/srbd_mpc.hpp"
#include "ester_mpc/trajectories.hpp"

namespace ester_mpc {

enum class ControllerType {
    SRBD,        // SingeRigidBodyMPC
    // \todo Others may arrive in the future
    UNSPECIFIED, // Load from parameter server
};

struct MPCControlOutput {
    bool contains_foot_forces = false;
    ester_common::LegMap<Eigen::Vector3d> foot_forces;

    bool contains_spine_pos = false;
    ester_common::SpineJointMap<double> spine_pos;

    bool contains_spine_vel = false;
    ester_common::SpineJointMap<double> spine_vel;

    bool contains_spine_eff = false;
    ester_common::SpineJointMap<double> spine_eff;
};

class MPCManager {
public:
    /**
     * @brief Constructor. Loads MPC parameters from parameter server
     *
     * @param param_ns The namespace within which to look for MPC parameters
     * @param rm Robot model pointer default none will initialise a default model
     * @param type Which type of MPC controller to run. Default to load from
     *             parameter server
     */
    MPCManager(
        const std::string &param_ns,
        const dart::dynamics::SkeletonPtr &robot,
        const std::shared_ptr<ester_kinematics::RobotModel> &rm = nullptr,
        ControllerType type=ControllerType::UNSPECIFIED);

    /**
     * @brief Destructor
     */
    ~MPCManager();

    /**
     * @brief Get the output of the MPC Controller
     * 
     * @details All cartesian values in an (identical) axis-aligned fixed
     *          global frame, except angular velocities which should be in the
     *          body (local) frame, and foot trajectories in the inertial
     *          (fixed local) frame
     * 
     * @param pos The current position of the main robot body
     * @param rot The current rotation of the main robot body
     * @param vel The current linear velocity of the main robot body
     * @param ang_vel The current angular velocity of the main robot body
     * @param cmd_pos The requested body position
     * @param cmd_rot The requested body rotation
     * @param cmd_lin_vel The requested linear velocity
     * @param cmd_ang_vel The requested angular velocity
     * @param jp The current joint positions
     * @param jp The current joint velocities
     * @param jt The current joint torques
     * @param foot_s The current foot stance state
     * @param trajs The trajectories for the next X timesteps (local frame)
     * 
     * @returns The MPC control output
     */
    MPCControlOutput control(
        const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
        const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
        const Eigen::Vector3d &cmd_pos, const Eigen::Matrix3d &cmd_rot,
        const Eigen::Vector3d &cmd_lin_vel, const Eigen::Vector3d &cmd_ang_vel,
        const ester_common::AllJointMap<double> &jp,
        const ester_common::AllJointMap<double> &jv,
        const ester_common::AllJointMap<double> &jt,
        const ester_common::LegMap<bool> &foot_s,
        const std::shared_ptr<Trajectories> &traj);

    /**
     * @brief Get the output of the MPC controller, given pre-calculated
     *        foot positions and grfs.
     * 
     * @details All cartesian values in an (identical) axis-aligned fixed
     *          global frame, except angular velocities which should be in the
     *          body (local) frame
     *
     * @param pos The current position of the main robot body
     * @param rot The current rotation of the main robot body
     * @param vel The current linear velocity of the main robot body
     * @param ang_vel The current angular velocity of the main robot body
     * @param cmd_pos The requested body position
     * @param cmd_rot The requested body rotation
     * @param cmd_lin_vel The requested linear velocity
     * @param cmd_ang_vel The requested angular velocity
     * @param foot_p The current foot positions (local frame)
     * @param foot_f The current foot GRFs (local frame)
     * @param foot_s The current foot stance state
     * @param trajs The trajectories for the next X timesteps (local frame)
     * @param spine_pos (Optional) Current position of the spine
     * @param spine_vel (Optional) Current velocity of the spine
     * 
     * @returns The MPC control output
     */
    MPCControlOutput control(
        const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
        const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
        const Eigen::Vector3d &cmd_pos, const Eigen::Matrix3d &cmd_rot,
        const Eigen::Vector3d &cmd_lin_vel, const Eigen::Vector3d &cmd_ang_vel,
        const ester_common::LegMap<Eigen::Vector3d> &foot_p,
        const ester_common::LegMap<Eigen::Vector3d> &foot_f,
        const ester_common::LegMap<bool> &foot_s,
        const std::shared_ptr<Trajectories> &trajs,
        const Eigen::Vector4d &spine_pos = Eigen::Vector4d::Zero(),
        const Eigen::Vector4d &spine_vel = Eigen::Vector4d::Zero());

    const std::shared_ptr<MPCBase::State> &get_state() const { return state_; };
    const std::shared_ptr<MPCBase::Solution> &get_solu() const { return solu_; };
    const Eigen::Vector3d &get_com_offset() const { return robot_com_offset_; };
    const ControllerType &get_type() const { return type_; };

private:
    /**
     * @brief Updates the current robot state
     *
     * @param pos The current position of the main robot body
     * @param rot The current rotation of the main robot body
     * @param vel The current velocity of the main robot body
     * @param ang_vel The current angular velocity of the main robot body
     * @param jp The current joint positions
     * @param jp The current joint velocities
     * @param jt The current joint torques
     * @param trajs The trajectories for the next X timesteps
     */
    void update_robot_state(
        const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
        const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
        const ester_common::AllJointMap<double> &jp,
        const ester_common::AllJointMap<double> &jv,
        const ester_common::AllJointMap<double> &jt,
        const std::shared_ptr<Trajectories> &traj);

    /**
     * @brief Fills the current state of the robot into a State message
     *
     * @details Calls the relevant implementation below
     *
     * @returns false if we can't perform control with this state
     */
    bool fill_state(const std::shared_ptr<MPCBase::State> &state);

    /**
     * @brief Fills the current state of the robot into a State message
     *        specifically for SingleRigidBodyMPC
     * 
     * @returns false if we can't perform control with this state
     */
    bool fill_state(
        const std::shared_ptr<SingleRigidBodyMPC::State> &state);

    /**
     * @brief Interpolate trajectories to fit the MPC 
     * 
     * @details The trajectories are expected to be in the base_link frame.
     *          The outputs will be in the COM (inertial) frame
     * 
     * @param stance The target foot stance for the next X timesteps
     * @param trajs The target foot (and spine) trajectories for the next X timesteps
     */
    void interpolate_trajectories(
        std::vector<ester_common::LegMap<bool>> &stance,
        const std::shared_ptr<Trajectories> &trajs);

    /**
     * @brief Interpolate trajectories to fit the MPC 
     * 
     * @details The trajectories are expected to be in the base_link frame.
     *          The outputs will be in the COM (inertial) frame
     * 
     * @param spine_pos The target spine positions for the next X timesteps
     * @param spine_vel The target spine velocities for the next X timesteps
     * @param trajs The target spine (and foot) trajectories for the next X timesteps
     */
    void interpolate_spine(
        std::vector<Eigen::Vector4d> &spine_pos,
        std::vector<Eigen::Vector4d> &spine_vel,
        const std::shared_ptr<Trajectories> &trajs);

    /**
     * @brief Extracts and updates the MPCControlOutput from a Solution
     *
     * @details Calls the relevant implementation below
     * 
     * @returns The control output
     */
    MPCControlOutput extract_solu(const std::shared_ptr<MPCBase::Solution> &solu);

    /**
     * @brief Extracts and updates the MPCControlOutput from a Solution
     *        specifically for SingleRigidBodyMPC
     * 
     * @returns The control output
     */
    MPCControlOutput extract_solu(
        const std::shared_ptr<SingleRigidBodyMPC::Solution> &solu);

    /**
     * @brief Load parameters from parameter server
     *
     * @brief The namespace to load parameters from
     */
    void load_ros_params(const std::string ns);

    /**
     * @brief The MPC controller
     */
    std::shared_ptr<MPCBase> mpc_;

    /**
     * @brief The state of the robot for the respective MPC controler
     */
    std::shared_ptr<MPCBase::State> state_;

    /**
     * @brief The most recent solution for the respective MPC controler
     */
    std::shared_ptr<MPCBase::Solution> solu_;

    /**
     * @brief Which controller we're running
     */
    ControllerType type_;

    /**
     * @brief Offset of robot COM w.r.t spine_center_link
     */
    Eigen::Vector3d robot_com_offset_;

    /**
     * @brief Current com position
     */
    Eigen::Vector3d robot_pos_;

    /**
     * @brief Current robot com orientation
     */
    Eigen::Matrix3d robot_rot_;

    /**
     * @brief Current robot com velocity
     */
    Eigen::Vector3d robot_vel_;

    /**
     * @brief Current robot angular velocity
     */
    Eigen::Vector3d robot_ang_vel_;

    /**
     * @brief Current robot foot positions
     */
    ester_common::LegMap<Eigen::Vector3d> robot_fp_;

    /**
     * @brief Current robot foot grfs
     */
    ester_common::LegMap<Eigen::Vector3d> robot_ff_;
    
    /**
     * @brief Current robot foot stance
     */
    ester_common::LegMap<bool> robot_stance_;

    /**
     * @brief Current spine joint positions
     */
    Eigen::Vector4d spine_pos_;

    /**
     * @brief Current spine joint velocities
     */
    Eigen::Vector4d spine_vel_;

    /**
     * @brief Cmd vel
     */
    Eigen::Vector3d cmd_v_;
    Eigen::Vector3d cmd_w_;

    /**
     * @brief What tgt vel to start from for horizon
     */
    Eigen::Vector3d strt_tgt_vel_;

    /**
     * @brief Used as targets if cmd_vel is small to prevent drifting
     */
    Eigen::Vector3d target_pos_;
    Eigen::Matrix3d target_rot_;

    /**
     * @brief Robot Model for calculating dynamics/kinematics
     */
    std::shared_ptr<ester_kinematics::RobotModel> robot_model_;

    /**
     * @brief Most recently commanded trajectory reference
     */
    std::shared_ptr<Trajectories> trajs_;
};

} // ns ester_mpc

#endif // __MPC_MANAGER_HPP__
