#ifndef __SPINE_TRAJECTORY_COT_WORLD_HPP__
#define __SPINE_TRAJECTORY_COT_WORLD_HPP__

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

#include <ester_common/ester_enums.hpp>
#include <ester_kinematics/leg_dynamics.hpp>
#include <ester_kinematics/leg_controller.hpp>
#include <ester_kinematics/robot_joint_controller.hpp>

#include "./replay_world_osg.hpp"
#include "ester_mpc/spine_trajectory_generator.hpp"
#include "ester_mpc/trajectory_planner.hpp"
#include "ester_mpc/mpc_manager.hpp"
#include "ester_mpc/gait_manager.hpp"

namespace ester_mpc
{

class SpineTrajectoryCotWorld {
public:
    SpineTrajectoryCotWorld(
        const dart::simulation::WorldPtr &world,
        const ester_common::SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> &stgs,
        const Eigen::VectorXd &spine_params,
        const std::vector<std::pair<double, double>> &velocities,
        GaitType gait,
        bool write_to_file,
        std::string gait_extra_name = "");

    void run();

    void pre_step();

    void post_step();

    const dart::simulation::WorldPtr &world() { return world_; };

    std::vector<double> get_cots() {return cots_; };
    std::vector<double> get_cots_legs_only() {return leg_only_cots_; };

    std::shared_ptr<History> record();

private:
    void setup_world();

    void step_mpc(double dt);

    void update_trajectories(double dt);

    void reset_world();

    void update_readings();

    void write_to_file();

    void record_cot();

    void record_cot_unstable();

    void update_contacts();
    
    void update_spine_joint_params();

    double interpolate(double from, double to, size_t step, size_t total_steps);

    ester_common::LegJointMap<double> toMap(const Eigen::Vector3d &v) const;

    Eigen::Vector3d toVector(const ester_common::LegJointMap<double> &lm) const;

    Eigen::VectorXd toVector(const ester_common::AllJointMap<double> &jm) const;

    void fillMap(
        const Eigen::VectorXd jnt_vector,
        ester_common::AllJointMap<double> &jnt_map) const;

    void get_foot_p_foot_f(
        ester_common::LegMap<Eigen::Vector3d> &foot_p,
        ester_common::LegMap<Eigen::Vector3d> &foot_f);

    void apply_spine_feedforward_torque(
        ester_common::AllJointMap<double> &cmd_torques,
        const ester_common::LegMap<Eigen::Vector3d> &foot_p,
        const ester_common::LegMap<Eigen::Vector3d> &foot_f);

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::SkeletonPtr plane_;

    dart::dynamics::BodyNodePtr chassis_rbd_;
    ester_common::LegMap<dart::dynamics::BodyNodePtr> feet_;

    ester_common::LegMap<bool> contacts_;
    ester_common::LegMap<bool> prev_contacts_;
    
    Eigen::VectorXd zeros_;
    Eigen::VectorXd dof_zeros_;

    ester_common::SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> spine_trajectory_generators_;
    ester_common::SpineJointMap<std::shared_ptr<SpineTrajectoryParams>> spine_trajectory_parameters_;
    std::unique_ptr<TrajectoryPlanner> trajectory_planner_;
    ester_kinematics::RobotJointController robot_ctrl_;
    ester_common::LegMap<std::shared_ptr<ester_kinematics::LegDynamics>> dyns_;
    ester_common::LegMap<std::shared_ptr<ester_kinematics::LegController>> ctrls_;
    std::shared_ptr<ester_kinematics::SpineKinematics> spine_;

    std::shared_ptr<MPCManager> mpc_;
    std::shared_ptr<Trajectories> trajs_;
    std::unique_ptr<GaitManager> gait_;

    // Position of the leg hip in the position / etc map
    ester_common::LegMap<size_t> leg_col_;
    // Position of the spine joint in the position / etc map
    ester_common::SpineJointMap<size_t> spine_idx_;

    Eigen::Vector3d cmd_pos_;
    Eigen::Matrix3d cmd_rot_;
    Eigen::Vector3d cmd_lin_vel_;
    Eigen::Vector3d cmd_ang_vel_;

    // For metrics
    ester_common::LegMap<std::vector<Eigen::Vector3d>> stance_start_pos_;
    ester_common::LegMap<std::vector<Eigen::Vector3d>> stance_end_pos_;
    ester_common::LegMap<std::vector<double>> all_foot_forces_;
    std::vector<Eigen::VectorXd> all_torques_;
    std::vector<double> total_spine_power_;
    std::vector<double> total_leg_power_;
    std::vector<double> total_power_;
    std::vector<Eigen::Vector3d> body_pos_err_;
    std::vector<Eigen::Vector3d> body_rot_err_;
    std::vector<Eigen::Vector3d> body_lin_vel_err_;
    std::vector<Eigen::Vector3d> body_ang_vel_err_;
    ester_common::SpineJointMap<std::vector<double>> spine_err_;
    ester_common::SpineJointMap<std::vector<double>> spine_pos_;
    ester_common::SpineJointMap<std::vector<double>> spine_fft_;
    ester_common::SpineJointMap<std::vector<double>> spine_eff_;
    std::vector<double> cots_;
    std::vector<double> leg_only_cots_;
    double fp_fft_ = 0;
    double fy_fft_ = 0;
    double rp_fft_ = 0;
    double ry_fft_ = 0;

    Eigen::VectorXd spine_params_;
    std::vector<std::pair<double, double>> test_velocities_;
    size_t test_idx_;
    GaitType test_gait_;
    bool finished_;
    bool record_cot_;
    double start_time_;

    double last_mpc_update_;
    Eigen::VectorXd prev_cmd_;

    ester_common::AllJointMap<double> cmd_jnt_pos_;
    ester_common::AllJointMap<double> cmd_jnt_vel_;
    ester_common::AllJointMap<double> cmd_jnt_eff_;
    ester_common::AllJointMap<bool>   torque_ctrl_;
    ester_common::AllJointMap<double> ctrl_kp_;
    ester_common::AllJointMap<double> ctrl_kd_;

    /**
     * TEST PARAMETERS
     */
    double test_dur_ = 10.0;
    double acc_ = 0.5;
    double ang_acc_ = 1.0;

    // Measured experimentally
    double motor_winding_resistance_ = 0.27;
    // 6s lipo
    double motor_v_ = 25.2;
    Eigen::VectorXd motor_kvs_;

    bool write_to_file_;
    std::string gait_extra_name_;
};

/**
 * @brief Visualise the above, useful for testing
 */
class SpineTrajectoryCotVisualiser {
public:
    SpineTrajectoryCotVisualiser(
        std::shared_ptr<SpineTrajectoryCotWorld> manager);

    virtual ~SpineTrajectoryCotVisualiser() noexcept;

    void run();

private:
    std::shared_ptr<SpineTrajectoryCotWorld> manager_;

    dart::gui::osg::ImGuiViewer viewer_;
    osg::ref_ptr<dart::gui::osg::WorldNode> ref_;

    std::thread worker_;
};

} // namespace ester_mpc


#endif // __SPINE_TRAJECTORY_COT_WORLD_HPP__