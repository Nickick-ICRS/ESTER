#include "ester_mpc/world_nodes/spine_trajectory_optimise_world.hpp"

#include <sophus/so3.hpp>

#include "ester_common/logging/file_logger.hpp"
#include "ester_kinematics/load_robot_model.hpp"

using namespace ester_common;
using namespace ester_kinematics;

namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;

namespace ester_mpc
{

SpineTrajectoryOptimiseWorld::SpineTrajectoryOptimiseWorld(
    const dart::simulation::WorldPtr &world,
    const ester_common::SpineJointMap<std::shared_ptr<SpineTrajectoryGenerator>> &stgs,
    const std::vector<Eigen::VectorXd> &parameters_to_test)

    :world_(world), spine_trajectory_generators_(stgs), parameters_to_test_(parameters_to_test),
     robot_ctrl_(true), write_to_file_(false)
{
    size_t num_params = 0;
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        num_params += spine_trajectory_generators_[jnt]->num_params();
    }
    assert(parameters_to_test.size() > 0);
    assert(parameters_to_test[0].rows() == num_params);
    search_iteration_ = 0;
    current_params_ = get_parameters();
    best_parameters_ = current_params_;
    best_reward_ = std::numeric_limits<double>::infinity();
    finished_solving_ = false;

    switch(test_gait_) {
    case GaitType::WALKING:
        tgt_vel_ = 0.35;
        break;
    case GaitType::TROTTING:
        tgt_vel_ = 0.65;
        break;
    case GaitType::BOUNDING:
        tgt_vel_ = 1.0;
        break;
    default:
        throw("Only WALKING, TROTTING and BOUNDING are valid gait types");
    }

    setup_world();
    reset_world();
    update_spine_joint_params();

    // Includes the free joint
    double kv_motor = 2 * M_PI * 270. / 60.;
    // kt_inv is kv, but the gear ratio also has an effect
    AllJointMap<double> kvs;
    kvs[AllJointId::F_SPINE_PITCH] = kv_motor / 16.;
    kvs[AllJointId::F_SPINE_YAW] = kv_motor / 9.;
    kvs[AllJointId::FL_HIP_ROLL] = kv_motor / 9.;
    kvs[AllJointId::FL_HIP_PITCH] = kv_motor / 16.;
    kvs[AllJointId::FL_KNEE] = kv_motor / 16.;
    kvs[AllJointId::FR_HIP_ROLL] = kv_motor / 9.;
    kvs[AllJointId::FR_HIP_PITCH] = kv_motor / 16.;
    kvs[AllJointId::FR_KNEE] = kv_motor / 16.;
    kvs[AllJointId::R_SPINE_PITCH] = kv_motor / 16.;
    kvs[AllJointId::R_SPINE_YAW] = kv_motor / 9.;
    kvs[AllJointId::RL_HIP_ROLL] = kv_motor / 9.;
    kvs[AllJointId::RL_HIP_PITCH] = kv_motor / 16.;
    kvs[AllJointId::RL_KNEE] = kv_motor / 16.;
    kvs[AllJointId::RR_HIP_ROLL] = kv_motor / 9.;
    kvs[AllJointId::RR_HIP_PITCH] = kv_motor / 16.;
    kvs[AllJointId::RR_KNEE] = kv_motor / 16.;
    motor_kvs_ = toVector(kvs);
}

void SpineTrajectoryOptimiseWorld::solve() {
    // Already been solved
    if (best_reward_ != std::numeric_limits<double>::infinity()) {
        return;
    }

    while (!finished_solving_ && ros::ok()) {
        pre_step();
        world_->step();
        post_step();
    }
}

std::shared_ptr<History> SpineTrajectoryOptimiseWorld::record() {
    write_to_file_ = true;
    std::shared_ptr<History> hist = std::make_shared<History>();
    while (search_iteration_ == 0 && ros::ok()) {
        State state;
        state.cfg = robot_->getConfiguration();
        state.aspect_state.reserve(robot_->getNumBodyNodes());
        for (size_t i = 0; i < robot_->getNumBodyNodes(); i++) {
            state.aspect_state.push_back(robot_->getBodyNode(i)->getCompositeState());
        }
        hist->push_back(state);
        pre_step();
        world_->step();
        post_step();
    }
    write_to_file_ = false;
    return hist;
}

void SpineTrajectoryOptimiseWorld::pre_step() {
    const double now = world_->getTime();
    // Update cmd velocity
    double speed = now * acc_;
    double ang_speed = now * ang_acc_;
    if (speed > tgt_vel_) {
        speed = tgt_vel_;
    }
    if (ang_speed > tgt_ang_vel_) {
        ang_speed = tgt_ang_vel_;
    }
    cmd_lin_vel_.x() = speed;
    cmd_ang_vel_.z() = ang_speed;
    cmd_rot_ = (Sophus::SO3d::exp(world_->getTimeStep() * cmd_ang_vel_) * Sophus::SO3d::fitToSO3(cmd_rot_)).matrix();
    cmd_pos_ += cmd_rot_ * cmd_lin_vel_ * world_->getTimeStep();

    // Limit MPC to ~200 Hz
    const double dt = now - last_mpc_update_;
    if (dt >= 4.9e-3) {
        last_mpc_update_ = now;
        update_trajectories(dt);
        step_mpc(dt);
    }
    else {
        // No update - maintain cmd forces
        robot_->setForces(prev_cmd_);
    }
}

void SpineTrajectoryOptimiseWorld::post_step() {
    update_contacts();
    if (finished_solving_) {
        return;
    }
    update_readings();
    bool done = false;
    // Check if done
    if (world_->getTime() >= test_dur_) {
        double cost = calculate_cost(current_params_);
        if (cost < best_reward_) {
            best_reward_ = cost;
            best_parameters_ = current_params_;
        }
        done = true;
    }
    // End early due to unstable simulation before we crash
    if (body_pos_err_.back().norm() > 0.2) {
        done = true;
    }
    for(const auto &jnt : ALL_SPINE_JOINTS) {
        // Respect physical joint limits
        if (spine_pos_[jnt].back() > 15 * M_PI / 180) {
            done = true;
        }
    }
    if (done) {
        if (write_to_file_) {
            // Write results to file
            write_to_file();
        }

        reset_world();
        if (++search_iteration_ == parameters_to_test_.size()) {
            // Finished
            finished_solving_ = true;
            return;
        }
        current_params_ = get_parameters();
        update_spine_joint_params();
    }
}

void SpineTrajectoryOptimiseWorld::setup_world() {
    robot_ = ester_kinematics::load_ester_robot_model();
    world_->setTimeStep(2.5e-3);
    world_->setGravity(Eigen::Vector3d(0, 0, -9.81));

    for (const auto &leg : ALL_LEG_IDS) {
        ctrls_[leg] = std::make_shared<LegController>(leg, true);
        dyns_[leg] = ctrls_[leg]->get_solver();
    }
    spine_ = std::make_shared<SpineKinematics>();

    for (const auto &id : ALL_JOINT_IDS) {
        cmd_jnt_pos_[id] = 0;
        cmd_jnt_vel_[id] = 0;
        cmd_jnt_eff_[id] = 0;
        torque_ctrl_[id] = true;
        ctrl_kp_[id] = 1.5;
        ctrl_kd_[id] = 0.02;
        robot_ctrl_.register_joint(
            id, &cmd_jnt_pos_[id], &cmd_jnt_vel_[id], &cmd_jnt_eff_[id],
            &torque_ctrl_[id]);
    }

    for (const auto &jnt : ALL_SPINE_JOINTS) {
        torque_ctrl_[full_joint_id(jnt)] = false;
    }

    // Default spine stiffness values
    ctrl_kp_[AllJointId::F_SPINE_PITCH] = 8;
    ctrl_kd_[AllJointId::F_SPINE_PITCH] = 0.25;
    ctrl_kp_[AllJointId::R_SPINE_PITCH] = 32;
    ctrl_kd_[AllJointId::R_SPINE_PITCH] = 0;
    ctrl_kp_[AllJointId::F_SPINE_YAW] = 8;
    ctrl_kd_[AllJointId::F_SPINE_YAW] = 0.8;
    ctrl_kp_[AllJointId::R_SPINE_YAW] = 8;
    ctrl_kd_[AllJointId::R_SPINE_YAW] = 0.8;

    {
        plane_ = dd::Skeleton::create("Plane");
        dd::BodyNodePtr node = plane_->createJointAndBodyNodePair<dd::WeldJoint>().second;
        const double t = 0.1;
        auto box = std::make_shared<dd::BoxShape>(Eigen::Vector3d(100, 100, t));
        auto shape_node = node->createShapeNodeWith<
            dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(box);
        shape_node->getVisualAspect()->setColor(dart::Color::Fuchsia(0.8));
        // Set plane top surface to be at z = 0
        node->getParentJoint()->setTransformFromParentBodyNode(
            Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, -t/2));
        world_->addSkeleton(plane_);
    }

    // First node is wack because dart gives default inertia
    robot_->getBodyNode(0)->setMass(1e-9);
    robot_->getBodyNode(0)->setMomentOfInertia(1e-3, 1e-3, 1e-3, 0, 0, 0);

    chassis_rbd_ = robot_->getBodyNode("spine_center_link");
    feet_[LegId::FL] = robot_->getBodyNode("front_left_foot_contact_link");
    feet_[LegId::FR] = robot_->getBodyNode("front_right_foot_contact_link");
    feet_[LegId::RL] = robot_->getBodyNode("rear_left_foot_contact_link");
    feet_[LegId::RR] = robot_->getBodyNode("rear_right_foot_contact_link");

    zeros_.resize(robot_->getNumDofs());
    zeros_.setZero();
    dof_zeros_ = zeros_;

    spine_idx_[SpineJointId::FRONT_PITCH] = 6;
    spine_idx_[SpineJointId::FRONT_YAW] = 7;
    spine_idx_[SpineJointId::REAR_PITCH] = 14;
    spine_idx_[SpineJointId::REAR_YAW] = 15;
    // FL
    leg_col_[LegId::FL] = 8;
    dof_zeros_(9) = 0.531;
    dof_zeros_(10) = -0.996;
    // FR
    leg_col_[LegId::FR] = 11;
    dof_zeros_(12) = 0.531;
    dof_zeros_(13) = -0.996;
    // RL
    leg_col_[LegId::RL] = 16;
    dof_zeros_(17) = -0.353;
    dof_zeros_(18) = 1.053;
    // RR
    leg_col_[LegId::RR] = 19;
    dof_zeros_(20) = -0.353;
    dof_zeros_(21) = 1.053;

    robot_->setPositions(dof_zeros_);

    // Start just off the ground
    robot_->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.25));
    std::cerr << "Finished loading robot '" << robot_->getName() << "'" << std::endl;
    world_->addSkeleton(robot_);

    LegMap<Eigen::Vector3d> fp, ff;
    get_foot_p_foot_f(fp, ff);
    XmlRpc::XmlRpcValue gait_config;
    auto nh = std::make_shared<ros::NodeHandle>();
    nh->getParam("/ester/gait_manager", gait_config);

    // Disable spine trajectories from GM
    gait_config["generate_spine_trajectories"] = false;
    gait_ = std::make_unique<GaitManager>(fp, gait_config, nh);
    gait_->set_gait_type(test_gait_);

    if(!ros::param::has("/mpc/mass")) {
        ros::param::set("/mpc/mass", robot_->getMass());
    }

    mpc_ = std::make_shared<MPCManager>("/mpc", robot_, nullptr, ControllerType::SRBD);

    fillMap(robot_->getPositions(), cmd_jnt_pos_);
    fillMap(robot_->getVelocities(), cmd_jnt_vel_);
    fillMap(robot_->getForces(), cmd_jnt_eff_);
}

void SpineTrajectoryOptimiseWorld::step_mpc(double dt) {
    Eigen::Vector3d pos = chassis_rbd_->getWorldTransform().translation();
    Eigen::Matrix3d rot = chassis_rbd_->getWorldTransform().rotation();
    Eigen::Vector3d vel = chassis_rbd_->getLinearVelocity();
    Eigen::Vector3d ang_vel = chassis_rbd_->getAngularVelocity(dd::Frame::World(), chassis_rbd_);

    LegMap<Eigen::Vector3d> foot_p, foot_f;
    get_foot_p_foot_f(foot_p, foot_f);

    Eigen::Vector4d sp, sv;
    sp << 
        robot_->getJoint("front_spine_pitch_joint")->getPosition(0),
        robot_->getJoint("front_spine_yaw_joint")->getPosition(0),
        robot_->getJoint("rear_spine_pitch_joint")->getPosition(0),
        robot_->getJoint("rear_spine_yaw_joint")->getPosition(0);
    sv << 
        robot_->getJoint("front_spine_pitch_joint")->getVelocity(0),
        robot_->getJoint("front_spine_yaw_joint")->getVelocity(0),
        robot_->getJoint("rear_spine_pitch_joint")->getVelocity(0),
        robot_->getJoint("rear_spine_yaw_joint")->getVelocity(0);

    auto ctrl = mpc_->control(
        pos, rot, vel, ang_vel, cmd_pos_, cmd_rot_, cmd_lin_vel_, cmd_ang_vel_,
        foot_p, foot_f, contacts_, trajs_, sp, sv);

    AllJointMap<double> all_jp, all_jv;
    LegMap<Eigen::Vector3d> tgt_pos;
    LegMap<bool> tgt_stance;
    for (const auto &leg : ALL_LEG_IDS) {
        tgt_stance[leg] = trajs_->stance[leg].front();
        if (tgt_stance[leg] && !contacts_[leg]) {
            // Apply a force down to push the foot into ground contact
            ctrl.foot_forces[leg] = Eigen::Vector3d::UnitZ() * mpc_->get_state()->mass * 2;
        }
        tgt_pos[leg] = trajs_->pos[leg].front();
    }
    fillMap(robot_->getPositions(), all_jp);
    fillMap(robot_->getVelocities(), all_jv);
    robot_ctrl_.control_robot(all_jp, tgt_pos, ctrl.foot_forces, tgt_stance, dt);

    AllJointMap<double> jnt_torques;
    for (const auto &id : ALL_JOINT_IDS) {
        if (torque_ctrl_[id]) {
            jnt_torques[id] = cmd_jnt_eff_[id];
        }
        else {
            double Kp = ctrl_kp_[id] * (cmd_jnt_pos_[id] - all_jp[id]);
            double Kd = ctrl_kd_[id] * (cmd_jnt_vel_[id] - all_jv[id]);
            jnt_torques[id] = Kp + Kd;
        }
    }
    apply_spine_feedforward_torque(jnt_torques, foot_p, ctrl.foot_forces);
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        // Fixed spine joint torque is zero because they're fixed
        if (spine_trajectory_generators_[jnt]->type() == SpineTrajectoryType::FIXED) {
            jnt_torques[full_joint_id(jnt)] = 0;
        }
    }
    Eigen::VectorXd gen_torques = toVector(jnt_torques);
    robot_->setForces(gen_torques);
    prev_cmd_ = gen_torques;
}

void SpineTrajectoryOptimiseWorld::update_trajectories(double dt) {
    Eigen::Vector6d cmd_vel;
    cmd_vel << cmd_ang_vel_, cmd_lin_vel_;
    LegMap<Eigen::Vector3d> foot_p, foot_f;
    get_foot_p_foot_f(foot_p, foot_f);
    trajs_ = gait_->update_trajectories(dt, chassis_rbd_->getSpatialVelocity(), cmd_vel, cmd_pos_.z(), foot_p);

    double current_phase = gait_->get_phase(LegId::FL);
    double phase_per_second = 1. / gait_->get_phase_dur();

    for (const auto &jnt : ALL_SPINE_JOINTS) {
        auto p = spine_trajectory_parameters_[jnt];
        switch(p->type()) {
            case SpineTrajectoryType::FIXED: {
                break;
            }
            case SpineTrajectoryType::TIME_DEP: {
                auto tdp = std::static_pointer_cast<SpineTimeDependantTrajectoryParams>(p);
                tdp->phase = current_phase;
                tdp->phase_per_second = phase_per_second;
                auto traj = std::static_pointer_cast<SpineTimeDependantTrajectory>(
                    spine_trajectory_generators_[jnt]->generate_trajectory(p));
                cmd_jnt_pos_[full_joint_id(jnt)] = traj->theta[0];
                ctrl_kp_[full_joint_id(jnt)] = std::abs(traj->stiffness);
                ctrl_kd_[full_joint_id(jnt)] = std::abs(traj->damping);
                break;
            }
            case SpineTrajectoryType::STIFFNESS: {
                auto sp = std::static_pointer_cast<SpineStiffnessTrajectoryParams>(p);
                sp->phase = current_phase;
                sp->phase_per_second = phase_per_second;
                auto traj = std::static_pointer_cast<SpineStiffnessTrajectory>(
                    spine_trajectory_generators_[jnt]->generate_trajectory(p));
                ctrl_kp_[full_joint_id(jnt)] = std::abs(traj->stiffness[0]);
                ctrl_kd_[full_joint_id(jnt)] = std::abs(traj->damping);
                break;
            }
            case SpineTrajectoryType::IMPEDANCE: {
                auto ip = std::static_pointer_cast<SpineImpedanceTrajectoryParams>(p);
                ip->phase = current_phase;
                ip->phase_per_second = phase_per_second;
                auto traj = std::static_pointer_cast<SpineImpedanceTrajectory>(
                    spine_trajectory_generators_[jnt]->generate_trajectory(p));
                ctrl_kp_[full_joint_id(jnt)] = std::abs(traj->stiffness[0]);
                ctrl_kd_[full_joint_id(jnt)] = std::abs(traj->damping[0]);
                break;
            }
            case SpineTrajectoryType::FOOT_POSITION_DEP: {
                auto fpdp = std::static_pointer_cast<SpineFootPosDependantTrajectoryParams>(p);
                // TODO: Consider future state if necessary
                fpdp->foot_pos[0] = foot_p;
                auto traj = std::static_pointer_cast<SpineFootPosDependantTrajectory>(
                    spine_trajectory_generators_[jnt]->generate_trajectory(p));
                cmd_jnt_pos_[full_joint_id(jnt)] = traj->theta[0];
                ctrl_kp_[full_joint_id(jnt)] = std::abs(traj->stiffness);
                ctrl_kd_[full_joint_id(jnt)] = std::abs(traj->damping);
                break;
            }
        }
    }
}

void SpineTrajectoryOptimiseWorld::update_contacts() {
    auto ce = world_->getConstraintSolver()->getCollisionDetector();
    auto cg_gp = ce->createCollisionGroup(plane_->getBodyNode(0));
    for (const auto &leg : ALL_LEG_IDS) {
        auto cg = ce->createCollisionGroup(feet_[leg]);
        dc::CollisionOption opt;
        dc::CollisionResult res;
        bool collision = cg_gp->collide(cg.get(), opt, &res);
        prev_contacts_[leg] = contacts_[leg];
        contacts_[leg] = collision;
    }
}

void SpineTrajectoryOptimiseWorld::reset_world() {
    world_->reset();

    robot_->clearExternalForces();
    robot_->clearInternalForces();
    robot_->setPositions(dof_zeros_);
    robot_->resetVelocities();
    robot_->resetAccelerations();
    robot_->resetCommands();

    last_mpc_update_ = 0;
    prev_cmd_ = zeros_;

    cmd_pos_ = Eigen::Vector3d(0, 0, 0.22);
    cmd_rot_ = Eigen::Matrix3d::Identity();
    cmd_lin_vel_ = Eigen::Vector3d(0, 0, 0);
    cmd_ang_vel_ = Eigen::Vector3d(0, 0, 0);

    LegMap<Eigen::Vector3d> foot_p, foot_f;
    get_foot_p_foot_f(foot_p, foot_f);
    gait_->reset(foot_p, true);
    gait_->set_gait_type(test_gait_);

    trajs_ = std::make_shared<Trajectories>();
    trajs_->timestep = 0.01;
    for (const auto &leg : ALL_LEG_IDS) {
        trajs_->pos[leg].push_back(foot_p[leg]);
        trajs_->stance[leg].push_back(true);
        contacts_[leg] = false;
        prev_contacts_[leg] = false;

        stance_start_pos_[leg].clear();
        stance_end_pos_[leg].clear();
        all_foot_forces_[leg].clear();
    }
    all_torques_.clear();
    total_power_.clear();
    total_spine_power_.clear();
    total_leg_power_.clear();
    body_pos_err_.clear();
    body_rot_err_.clear();
    body_lin_vel_err_.clear();
    body_ang_vel_err_.clear();
    auto set_joint_type = [&](auto jnt, auto type) {
        std::string name;
        switch(jnt) {
            case SpineJointId::FRONT_PITCH:
                name = "front_spine_pitch_joint";
                break;
            case SpineJointId::FRONT_YAW:
                name = "front_spine_yaw_joint";
                break;
            case SpineJointId::REAR_PITCH:
                name = "rear_spine_pitch_joint";
                break;
            case SpineJointId::REAR_YAW:
                name = "rear_spine_yaw_joint";
                break;
        }
        robot_->getJoint(name)->setActuatorType(type);
    };
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        spine_err_[jnt].clear();

        switch(spine_trajectory_generators_[jnt]->type()) {
            case SpineTrajectoryType::FIXED: {
                spine_trajectory_parameters_[jnt] = std::make_shared<SpineTrajectoryParams>();
                set_joint_type(jnt, dd::Joint::LOCKED);
                break;
            }
            case SpineTrajectoryType::FOOT_POSITION_DEP: {
                auto p = std::make_shared<SpineFootPosDependantTrajectoryParams>();
                p->prediction_window = 1;
                p->foot_pos.resize(p->prediction_window);
                p->spine_jnt_id = jnt;
                p->timestep = mpc_->get_state()->dt;
                spine_trajectory_parameters_[jnt] = p;
                set_joint_type(jnt, dd::Joint::FORCE);
                break;
            }
            case SpineTrajectoryType::IMPEDANCE: {
                auto p = std::make_shared<SpineImpedanceTrajectoryParams>();
                p->prediction_window = 1;
                p->timestep = mpc_->get_state()->dt;
                spine_trajectory_parameters_[jnt] = p;
                set_joint_type(jnt, dd::Joint::FORCE);
                break;
            }
            case SpineTrajectoryType::STIFFNESS: {
                auto p = std::make_shared<SpineStiffnessTrajectoryParams>();
                p->prediction_window = 1;
                p->timestep = mpc_->get_state()->dt;
                spine_trajectory_parameters_[jnt] = p;
                set_joint_type(jnt, dd::Joint::FORCE);
                break;
            }
            case SpineTrajectoryType::TIME_DEP: {
                auto p = std::make_shared<SpineTimeDependantTrajectoryParams>();
                p->prediction_window = 1;
                p->timestep = mpc_->get_state()->dt;
                spine_trajectory_parameters_[jnt] = p;
                if (jnt == SpineJointId::FRONT_PITCH || jnt == SpineJointId::REAR_PITCH) {
                    p->freq = 2;
                }
                set_joint_type(jnt, dd::Joint::FORCE);
                break;
            }
        }
    }
}

void SpineTrajectoryOptimiseWorld::update_readings() {
    // If contacts changed update relevant variables
    LegMap<Eigen::Vector3d> fp, ff;
    get_foot_p_foot_f(fp, ff);
    for (const auto &leg : ALL_LEG_IDS) {
        if(contacts_[leg] && !prev_contacts_[leg]) {
            // Landed
            stance_start_pos_[leg].push_back(fp[leg]);
            // Didn't release
            stance_end_pos_[leg].push_back(Eigen::Vector3d::Zero());
        }
        else if (!contacts_[leg] && prev_contacts_[leg]) {
            // Didn't land
            stance_start_pos_[leg].push_back(Eigen::Vector3d::Zero());
            // Released
            stance_end_pos_[leg].push_back(fp[leg]);
        }
        else {
            // Didn't land
            stance_start_pos_[leg].push_back(Eigen::Vector3d::Zero());
            // Didn't release
            stance_end_pos_[leg].push_back(Eigen::Vector3d::Zero());
        }
        all_foot_forces_[leg].push_back(ff[leg].norm());
    }
    Eigen::VectorXd v = robot_->getVelocities();
    // Electrical P = mech P + heat loss
    // Heat loss = I * I * R
    Eigen::ArrayXd I = prev_cmd_.array() * motor_kvs_.array();
    Eigen::ArrayXd heat_loss = I * I * motor_winding_resistance_;
    heat_loss.head<6>() = Eigen::Vector6d::Zero();
    // Mechanical P = tau * V
    Eigen::ArrayXd mech_power = (prev_cmd_.array() * v.array()).abs();
    mech_power.head<6>() = Eigen::Vector6d::Zero();
    all_torques_.push_back(prev_cmd_);
    auto total_power = mech_power + heat_loss;
    double total_spine_power = total_power(6) + total_power(7) + total_power(14) + total_power(15);
    double total_leg_power = total_power.sum() - total_spine_power;
    total_spine_power_.push_back(total_spine_power);
    total_leg_power_.push_back(total_leg_power);
    total_power_.push_back(total_spine_power + total_leg_power);
    body_pos_err_.push_back(chassis_rbd_->getWorldTransform().translation() - cmd_pos_);
    body_rot_err_.push_back(Sophus::SO3d(chassis_rbd_->getWorldTransform().linear() * cmd_rot_.transpose()).log());
    body_lin_vel_err_.push_back(chassis_rbd_->getLinearVelocity() - cmd_lin_vel_);
    body_ang_vel_err_.push_back(chassis_rbd_->getAngularVelocity() - cmd_ang_vel_);
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        switch(spine_trajectory_generators_[jnt]->type()) {
            // Err in spine target
            case SpineTrajectoryType::FOOT_POSITION_DEP:
            case SpineTrajectoryType::TIME_DEP:
                spine_err_[jnt].push_back(cmd_jnt_pos_[full_joint_id(jnt)] - robot_->getPosition(spine_idx_[jnt]));
                break;
            // Not relevant
            case SpineTrajectoryType::FIXED:
            case SpineTrajectoryType::IMPEDANCE:
            case SpineTrajectoryType::STIFFNESS:
                break;
        }
        spine_pos_[jnt].push_back(robot_->getPosition(spine_idx_[jnt]));
    }
}

void SpineTrajectoryOptimiseWorld::update_spine_joint_params() {
    size_t param_idx = 0;
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        auto p = spine_trajectory_parameters_[jnt];
        switch(p->type()) {
            case SpineTrajectoryType::FOOT_POSITION_DEP: {
                auto fpdp = std::static_pointer_cast<SpineFootPosDependantTrajectoryParams>(p);
                fpdp->A = current_params_[param_idx++];
                fpdp->B = current_params_[param_idx++];
                fpdp->C = current_params_[param_idx++];
                break;
            }
            case SpineTrajectoryType::IMPEDANCE: {
                auto ip = std::static_pointer_cast<SpineImpedanceTrajectoryParams>(p);
                ip->A = current_params_[param_idx++];
                ip->B = current_params_[param_idx++];
                ip->C = current_params_[param_idx++];
                ip->D = current_params_[param_idx++];
                ip->E = current_params_[param_idx++];
                ip->F = current_params_[param_idx++];
                ip->phase = 0;
                break;
            }
            case SpineTrajectoryType::STIFFNESS: {
                auto sp = std::static_pointer_cast<SpineStiffnessTrajectoryParams>(p);
                sp->A = current_params_[param_idx++];
                sp->B = current_params_[param_idx++];
                sp->C = current_params_[param_idx++];
                sp->D = current_params_[param_idx++];
                sp->phase = 0;
                break;
            }
            case SpineTrajectoryType::TIME_DEP: {
                auto tdp = std::static_pointer_cast<SpineTimeDependantTrajectoryParams>(p);
                tdp->A = current_params_[param_idx++];
                tdp->B = current_params_[param_idx++];
                tdp->C = current_params_[param_idx++];
                tdp->D = current_params_[param_idx++];
                tdp->phase = 0;
                break;
            }
            case SpineTrajectoryType::FIXED:
                break;
        }
    }
}

Eigen::VectorXd SpineTrajectoryOptimiseWorld::get_parameters() {
    Eigen::VectorXd params = parameters_to_test_[search_iteration_];
    return params;
}

double SpineTrajectoryOptimiseWorld::calculate_cost(const Eigen::VectorXd &parameters) {
    // If we didn't have the full number of steps return inf cost
    if (world_->getTime() < test_dur_) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    auto std_dev = [&zero](const std::vector<Eigen::Vector3d> &v) {
        std::vector<Eigen::Vector3d> non_zero;
        for (const Eigen::Vector3d &vec : v) {
            if (!vec.isApprox(Eigen::Vector3d::Zero())) {
                non_zero.push_back(vec);
            }
        }
        Eigen::Vector3d sum = std::accumulate(non_zero.begin(), non_zero.end(), zero);
        Eigen::Vector3d mean = sum / non_zero.size();
        std::vector<Eigen::Vector3d> sq(non_zero.size());
        std::transform(non_zero.begin(), non_zero.end(), sq.begin(),
            [mean](const Eigen::Vector3d &x) {return (x - mean).array().square().matrix(); });
        Eigen::Vector3d sq_sum = std::accumulate(sq.begin(), sq.end(), zero);
        Eigen::Vector3d std_dev = (sq_sum * (1. / non_zero.size())).array().sqrt();
        return std_dev;
    };

    double avg_power = std::accumulate(total_power_.begin(), total_power_.end(), 0.0) / total_power_.size();
    double avg_spine_power = std::accumulate(total_spine_power_.begin(), total_spine_power_.end(), 0.0) / total_spine_power_.size();
    double avg_leg_power = std::accumulate(total_leg_power_.begin(), total_leg_power_.end(), 0.0) / total_leg_power_.size();
    double avg_pos_err = std::accumulate(body_pos_err_.begin(), body_pos_err_.end(), zero).norm() / body_pos_err_.size();
    double avg_rot_err = std::accumulate(body_rot_err_.begin(), body_rot_err_.end(), zero).norm() / body_rot_err_.size();
    double avg_lin_vel_err = std::accumulate(body_lin_vel_err_.begin(), body_lin_vel_err_.end(), zero).norm() / body_lin_vel_err_.size();
    double avg_ang_vel_err = std::accumulate(body_ang_vel_err_.begin(), body_ang_vel_err_.end(), zero).norm() / body_ang_vel_err_.size();

    // These are accumulated since sum(X * k) = k * sum(X)
    double leg_stance_start_std_dev = 0;
    double leg_stance_end_std_dev = 0;
    double avg_force = 0;
    double peak_force = 0;
    double avg_spine_err = 0;
    double avg_peak_spine_movement = 0;
    for (const auto &leg : ALL_LEG_IDS) {
        leg_stance_start_std_dev += std_dev(stance_start_pos_[leg]).norm();
        leg_stance_end_std_dev += std_dev(stance_end_pos_[leg]).norm();
        avg_force += std::accumulate(all_foot_forces_[leg].begin(), all_foot_forces_[leg].end(), 0.0) / all_foot_forces_[leg].size();
        // Skip 1st second as robot is dropped
        peak_force += *std::max_element(all_foot_forces_[leg].begin()+400, all_foot_forces_[leg].end());
    }
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        double avg = std::accumulate(spine_pos_[jnt].begin(), spine_pos_[jnt].end(), 0.0) / spine_pos_[jnt].size();
        double avg_peak = 0;
        size_t num_peaks = 0;
        for (size_t i = 1; i < spine_pos_[jnt].size()-1; i++) {
            double k_minus = spine_pos_[jnt][i-1];
            double k = spine_pos_[jnt][i];
            double k_plus = spine_pos_[jnt][i+1];
            // Peak
            if (k_minus < k && k_plus < k) {
                avg_peak += k - avg;
                num_peaks++;
            }
            // Trough
            if (k < k_minus && k < k_plus) {
                avg_peak += avg - k;
                num_peaks++;
            }
        }
        if (num_peaks != 0) {
            avg_peak /= num_peaks;
            avg_peak_spine_movement += avg_peak;
        }
        // For those that don't record it it will just be zero
        if (spine_err_[jnt].size() == 0) {
            continue;
        }
        avg_spine_err += std::accumulate(spine_err_[jnt].begin(), spine_err_[jnt].end(), 0.0) / spine_err_[jnt].size();
        
    }
//    /*
    std::cerr << "Avg power: " << avg_power * 0.002 << " avg_pos_err: " << avg_pos_err * 50 << " avg_rot_err: " << avg_rot_err * 50
        << " avg_lin_vel_err: " << avg_lin_vel_err * 50 << " avg ang_vel_err: " << avg_ang_vel_err * 10
        << " leg_stance_start_stddev: " << leg_stance_start_std_dev * 4
        << " leg_stance_end_stddev: " << leg_stance_end_std_dev * 4
        << " avg_force: " << avg_force * 0.1 << " peak_force: " << peak_force / 800 << " avg_spine_err: " << avg_spine_err
        << " avg_spine_movement: " << avg_peak_spine_movement * 2
        << std::endl;
//        */

    // Magic numbers chosen to pick approximately similar orders of magnitude
    double cost =
        // Approx 1
        avg_pos_err * 50. + avg_rot_err * 50.
        // Approx 0.1
        + leg_stance_start_std_dev * 1.
        + leg_stance_end_std_dev * 1.
        + avg_power * 0.002
        // Approx 0.1
        + avg_lin_vel_err * 50 + avg_ang_vel_err * 10 + peak_force / 800.
        // Approx 1
        + avg_spine_err * 10.//;
        // Want to maximise this
        - avg_peak_spine_movement * 2;
    std::cerr << search_iteration_ << ": TOTAL COST: " << cost << std::endl;
    return cost;
}

double SpineTrajectoryOptimiseWorld::interpolate(
    double from, double to, size_t step, size_t total_steps)
{
    if (total_steps == 0) {
        return to;
    }
    double dx = (to - from) / (total_steps - 1);
    return from + dx * step;
}

ester_common::LegJointMap<double> SpineTrajectoryOptimiseWorld::toMap(
    const Eigen::Vector3d &v) const
{
    LegJointMap<double> map;
    map[LegJointId::HIP_ROLL] = v(0);
    map[LegJointId::HIP_PITCH] = v(1);
    map[LegJointId::KNEE] = v(2);
    return map;
}

Eigen::Vector3d SpineTrajectoryOptimiseWorld::toVector(
    const ester_common::LegJointMap<double> &lm) const
{
    Eigen::Vector3d v;
    v(0) = lm.at(LegJointId::HIP_ROLL);
    v(1) = lm.at(LegJointId::HIP_PITCH);
    v(2) = lm.at(LegJointId::KNEE);
    return v;
}

void SpineTrajectoryOptimiseWorld::fillMap(
    const Eigen::VectorXd jnt_vector,
    ester_common::AllJointMap<double> &jnt_map) const
{
    for (const auto &leg : ALL_LEG_IDS) {
        size_t idx = leg_col_.at(leg);
        jnt_map[leg + LegJointId::HIP_ROLL] = jnt_vector(idx);
        jnt_map[leg + LegJointId::HIP_PITCH] = jnt_vector(idx+1);
        jnt_map[leg + LegJointId::KNEE] = jnt_vector(idx+2);
    }
    jnt_map[AllJointId::F_SPINE_PITCH] = jnt_vector(6);
    jnt_map[AllJointId::F_SPINE_YAW] = jnt_vector(7);
    jnt_map[AllJointId::R_SPINE_PITCH] = jnt_vector(14);
    jnt_map[AllJointId::R_SPINE_YAW] = jnt_vector(15);
}

Eigen::VectorXd SpineTrajectoryOptimiseWorld::toVector(
    const ester_common::AllJointMap<double> &jm) const
{
    Eigen::VectorXd v;
    v.resize(robot_->getNumDofs());
    v.setZero();
    for (const auto &leg : ALL_LEG_IDS) {
        size_t idx = leg_col_.at(leg);
        v(idx) = jm.at(leg + LegJointId::HIP_ROLL);
        v(idx+1) = jm.at(leg + LegJointId::HIP_PITCH);
        v(idx+2) = jm.at(leg + LegJointId::KNEE);
    }
    v(6) = jm.at(AllJointId::F_SPINE_PITCH);
    v(7) = jm.at(AllJointId::F_SPINE_YAW);
    v(14) = jm.at(AllJointId::R_SPINE_PITCH);
    v(15) = jm.at(AllJointId::R_SPINE_YAW);
    return v;
}

void SpineTrajectoryOptimiseWorld::get_foot_p_foot_f(
    ester_common::LegMap<Eigen::Vector3d> &foot_p,
    ester_common::LegMap<Eigen::Vector3d> &foot_f)
{
    Eigen::VectorXd joint_pos = robot_->getPositions();
    Eigen::VectorXd joint_vel = robot_->getVelocities();
    Eigen::VectorXd joint_eff = robot_->getForces(); 

    SpineJointMap<double> sp;
    sp[SpineJointId::FRONT_PITCH] = robot_->getJoint("front_spine_pitch_joint")->getPosition(0);
    sp[SpineJointId::FRONT_YAW]   = robot_->getJoint("front_spine_yaw_joint")->getPosition(0);
    sp[SpineJointId::REAR_PITCH]  = robot_->getJoint("rear_spine_pitch_joint")->getPosition(0);
    sp[SpineJointId::REAR_YAW]    = robot_->getJoint("rear_spine_yaw_joint")->getPosition(0); 
    spine_->set_spine_joint_positions(sp);

    for (const auto &leg : ALL_LEG_IDS) {
        const size_t idx = leg_col_[leg];
        LegJointMap<double> jp, jv, je;
        jp = toMap(joint_pos.segment<3>(idx));
        jv = toMap(joint_vel.segment<3>(idx));
        je = toMap(joint_eff.segment<3>(idx));

        auto inv_hip_tf = spine_->get_shoulder_transform(leg).inverse();
        foot_p[leg] = inv_hip_tf * dyns_[leg]->solve_fk(jp).translation();
        foot_f[leg] = inv_hip_tf.linear() * dyns_[leg]->solve_fd(jp, jv, je);
    }
}

void SpineTrajectoryOptimiseWorld::apply_spine_feedforward_torque(
    AllJointMap<double> &cmd_torques,
    const LegMap<Eigen::Vector3d> &foot_p,
    const LegMap<Eigen::Vector3d> &foot_f)
{
    auto get_torque = [&](const dd::JointPtr &jnt, const LegId &leg) {
        auto base_to_jnt =
            jnt->getParentBodyNode()->getTransform(chassis_rbd_)
          * jnt->getTransformFromParentBodyNode();
        Eigen::Vector3d dist = foot_p.at(leg) - base_to_jnt.translation();
        Eigen::Vector3d torque = dist.cross(foot_f.at(leg)) * contacts_[leg];
        return torque;
    };
    auto f_spine_p = robot_->getJoint("front_spine_pitch_joint");
    auto f_spine_y = robot_->getJoint("front_spine_yaw_joint");
    auto r_spine_p = robot_->getJoint("rear_spine_pitch_joint");
    auto r_spine_y = robot_->getJoint("rear_spine_yaw_joint");

    Eigen::Vector3d f_p_eff = get_torque(f_spine_p, LegId::FL) + get_torque(f_spine_p, LegId::FR);
    Eigen::Vector3d f_y_eff = get_torque(f_spine_y, LegId::FL) + get_torque(f_spine_y, LegId::FR);
    Eigen::Vector3d r_p_eff = get_torque(r_spine_p, LegId::RL) + get_torque(r_spine_p, LegId::RR);
    Eigen::Vector3d r_y_eff = get_torque(r_spine_y, LegId::RL) + get_torque(r_spine_y, LegId::RR);

    // We add the feedforward torque here, PID is still being used to maintain
    // target positions
    cmd_torques[AllJointId::F_SPINE_PITCH] -= f_p_eff.y();
    cmd_torques[AllJointId::F_SPINE_YAW]   -= f_y_eff.z();
    cmd_torques[AllJointId::R_SPINE_PITCH] -= r_p_eff.y();
    cmd_torques[AllJointId::R_SPINE_YAW]   -= r_y_eff.z();
}

void SpineTrajectoryOptimiseWorld::write_to_file() {
    std::string file_name;
    switch(test_gait_) {
    case GaitType::WALKING:
        file_name += "walk_";
        break;
    case GaitType::TROTTING:
        file_name += "trot_";
        break;
    case GaitType::BOUNDING:
        file_name += "bound_";
        break;
    default:
        throw std::runtime_error("Unknown Gait Type");
    }
    if (tgt_ang_vel_ > 0.05 || tgt_ang_vel_ < -0.05) {
        std::ostringstream ss;
        ss << tgt_ang_vel_;
        file_name += "turning_" + ss.str() + "_";
    }
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        switch(spine_trajectory_generators_[jnt]->type()) {
        case SpineTrajectoryType::FIXED:
            file_name += "fixed_";
            break;
        case SpineTrajectoryType::STIFFNESS:
            file_name += "stiff_";
            break;
        case SpineTrajectoryType::IMPEDANCE:
            file_name += "imped_";
            break;
        case SpineTrajectoryType::FOOT_POSITION_DEP:
            file_name += "foot_";
            break;
        case SpineTrajectoryType::TIME_DEP:
            file_name += "time_";
            break;
        }
    }

    auto logger = logging::FileLoggerBuilder().begin()
        .filename(file_name)
        .starting_timestamp(0.)
        .new_field("spine_f_y")
        .new_field("spine_f_z")
        .new_field("spine_r_y")
        .new_field("spine_r_z")
        .new_field("err_spine_f_y")
        .new_field("err_spine_f_z")
        .new_field("err_spine_r_y")
        .new_field("err_spine_r_z")
        .new_field("total_torque")
        .new_field("total_power")
        .new_field("body_err_x")
        .new_field("body_err_y")
        .new_field("body_err_z")
        .new_field("rot_err_x")
        .new_field("rot_err_y")
        .new_field("rot_err_z")
        .new_field("lin_vel_err_x")
        .new_field("lin_vel_err_y")
        .new_field("lin_vel_err_z")
        .new_field("ang_vel_err_x")
        .new_field("ang_vel_err_y")
        .new_field("ang_vel_err_z")
        .new_field("fl_grf")
        .new_field("fr_grf")
        .new_field("rl_grf")
        .new_field("rr_grf")
        .new_field("fl_fd_x")
        .new_field("fl_fd_y")
        .new_field("fl_fd_z")
        .new_field("fl_lo_x")
        .new_field("fl_lo_y")
        .new_field("fl_lo_z")
        .new_field("fr_fd_x")
        .new_field("fr_fd_y")
        .new_field("fr_fd_z")
        .new_field("fr_lo_x")
        .new_field("fr_lo_y")
        .new_field("fr_lo_z")
        .new_field("rl_fd_x")
        .new_field("rl_fd_y")
        .new_field("rl_fd_z")
        .new_field("rl_lo_x")
        .new_field("rl_lo_y")
        .new_field("rl_lo_z")
        .new_field("rr_fd_x")
        .new_field("rr_fd_y")
        .new_field("rr_fd_z")
        .new_field("rr_lo_x")
        .new_field("rr_lo_y")
        .new_field("rr_lo_z")
        .build();

    for (size_t i = 0; i < body_pos_err_.size(); i++) {
        double t = world_->getTimeStep() * i;
        logger->write_field("spine_f_y", spine_pos_[SpineJointId::FRONT_PITCH][i], t);
        logger->write_field("spine_f_z", spine_pos_[SpineJointId::FRONT_YAW][i], t);
        logger->write_field("spine_r_y", spine_pos_[SpineJointId::REAR_PITCH][i], t);
        logger->write_field("spine_r_z", spine_pos_[SpineJointId::REAR_YAW][i], t);
        if (spine_err_[SpineJointId::FRONT_PITCH].size()) {
            logger->write_field("err_spine_f_y", spine_err_[SpineJointId::FRONT_PITCH][i], t);
        }
        else {
            logger->write_field("err_spine_f_y", 0., t);
        }
        if (spine_err_[SpineJointId::FRONT_YAW].size()) {
            logger->write_field("err_spine_f_z", spine_err_[SpineJointId::FRONT_YAW][i], t);
        }
        else {
            logger->write_field("err_spine_f_z", 0., t);
        }
        if (spine_err_[SpineJointId::REAR_PITCH].size()) {
            logger->write_field("err_spine_r_y", spine_err_[SpineJointId::REAR_PITCH][i], t);
        }
        else {
            logger->write_field("err_spine_r_y", 0., t);
        }
        if (spine_err_[SpineJointId::REAR_YAW].size()) {
            logger->write_field("err_spine_r_z", spine_err_[SpineJointId::REAR_YAW][i], t);
        }
        else {
            logger->write_field("err_spine_r_z", 0., t);
        }
        logger->write_field("total_torque", all_torques_[i].array().abs().sum(), t);
        // P = I * I * R
        logger->write_field("total_power", total_power_[i], t);
        logger->write_field("total_spine_power", total_spine_power_[i], t);
        logger->write_field("total_leg_power", total_leg_power_[i], t);
        logger->write_field("body_err_x", body_pos_err_[i].x(), t);
        logger->write_field("body_err_y", body_pos_err_[i].y(), t);
        logger->write_field("body_err_z", body_pos_err_[i].z(), t);
        logger->write_field("rot_err_x", body_rot_err_[i].x(), t);
        logger->write_field("rot_err_y", body_rot_err_[i].y(), t);
        logger->write_field("rot_err_z", body_rot_err_[i].z(), t);
        logger->write_field("lin_vel_err_x", body_lin_vel_err_[i].x(), t);
        logger->write_field("lin_vel_err_y", body_lin_vel_err_[i].y(), t);
        logger->write_field("lin_vel_err_z", body_lin_vel_err_[i].z(), t);
        logger->write_field("ang_vel_err_x", body_ang_vel_err_[i].x(), t);
        logger->write_field("ang_vel_err_y", body_ang_vel_err_[i].y(), t);
        logger->write_field("ang_vel_err_z", body_ang_vel_err_[i].z(), t);
        logger->write_field("fl_grf", all_foot_forces_[LegId::FL][i], t);
        logger->write_field("fr_grf", all_foot_forces_[LegId::FR][i], t);
        logger->write_field("rl_grf", all_foot_forces_[LegId::RL][i], t);
        logger->write_field("rr_grf", all_foot_forces_[LegId::RR][i], t);
        logger->write_field("fl_fd_x", stance_start_pos_[LegId::FL][i].x(), t);
        logger->write_field("fl_fd_y", stance_start_pos_[LegId::FL][i].y(), t);
        logger->write_field("fl_fd_z", stance_start_pos_[LegId::FL][i].z(), t);
        logger->write_field("fl_lo_x", stance_end_pos_[LegId::FL][i].x(), t);
        logger->write_field("fl_lo_y", stance_end_pos_[LegId::FL][i].y(), t);
        logger->write_field("fl_lo_z", stance_end_pos_[LegId::FL][i].z(), t);
        logger->write_field("fr_fd_x", stance_start_pos_[LegId::FR][i].x(), t);
        logger->write_field("fr_fd_y", stance_start_pos_[LegId::FR][i].y(), t);
        logger->write_field("fr_fd_z", stance_start_pos_[LegId::FR][i].z(), t);
        logger->write_field("fr_lo_x", stance_end_pos_[LegId::FR][i].x(), t);
        logger->write_field("fr_lo_y", stance_end_pos_[LegId::FR][i].y(), t);
        logger->write_field("fr_lo_z", stance_end_pos_[LegId::FR][i].z(), t);
        logger->write_field("rl_fd_x", stance_start_pos_[LegId::RL][i].x(), t);
        logger->write_field("rl_fd_y", stance_start_pos_[LegId::RL][i].y(), t);
        logger->write_field("rl_fd_z", stance_start_pos_[LegId::RL][i].z(), t);
        logger->write_field("rl_lo_x", stance_end_pos_[LegId::RL][i].x(), t);
        logger->write_field("rl_lo_y", stance_end_pos_[LegId::RL][i].y(), t);
        logger->write_field("rl_lo_z", stance_end_pos_[LegId::RL][i].z(), t);
        logger->write_field("rr_fd_x", stance_start_pos_[LegId::RR][i].x(), t);
        logger->write_field("rr_fd_y", stance_start_pos_[LegId::RR][i].y(), t);
        logger->write_field("rr_fd_z", stance_start_pos_[LegId::RR][i].z(), t);
        logger->write_field("rr_lo_x", stance_end_pos_[LegId::RR][i].x(), t);
        logger->write_field("rr_lo_y", stance_end_pos_[LegId::RR][i].y(), t);
        logger->write_field("rr_lo_z", stance_end_pos_[LegId::RR][i].z(), t);
    }
}

SpineTrajectoryOptimiseWorldOSG::SpineTrajectoryOptimiseWorldOSG(
    std::shared_ptr<SpineTrajectoryOptimiseWorld> optimiser)
    
    :optimiser_(std::move(optimiser)), dart::gui::osg::WorldNode(optimiser->world())
{
    // ctor
}

void SpineTrajectoryOptimiseWorldOSG::customPreStep() {
    optimiser_->pre_step();
}

void SpineTrajectoryOptimiseWorldOSG::customPostStep() {
    optimiser_->post_step();
}

} // namespace ester_mpc