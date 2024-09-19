#include "ester_mpc/mpc_manager.hpp"

// For get param etc.
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

using namespace ester_mpc;
using namespace ester_common;
using namespace ester_kinematics;

MPCManager::MPCManager(
    const std::string &param_ns,
    const dart::dynamics::SkeletonPtr &robot,
    const std::shared_ptr<RobotModel> &rm,
    ControllerType type)
    :robot_model_(rm), type_(type), strt_tgt_vel_(0, 0, 0)
{
    ros::NodeHandle nh;

    if (rm == nullptr) {
        robot_model_.reset(new RobotModel);
    }

    if (type == ControllerType::UNSPECIFIED) {
        std::string ns(param_ns);
        while (ns.back() == '/') {
            ns.pop_back();
        }
        std::string ctrl_type = "";
        std::string param = ns + '/' + "controller_type";
        if (!ros::param::param(param, ctrl_type, std::string("srbd")))
        {
            ROS_WARN_STREAM(
                "ControllerType::UNSPECIFIED received, but no controller "
                << "type specified on parameter server '" << param 
                << "'. Defaulting to " << ctrl_type);
        }
        if (ctrl_type == "srbd") {
            type_ = ControllerType::SRBD;
        }
        else {
            ROS_WARN_STREAM(
                "Requested MPC controller '" << ctrl_type
                << "' is not a member of the options: 'srbd'. Defaulting to"
                << " 'srbd'. (parameter '" << param << "')");
            type_ = ControllerType::SRBD;
        }
    }

    switch(type_) {
    case ControllerType::SRBD:
        state_ = std::make_shared<SingleRigidBodyMPC::State>();
        mpc_ = std::make_shared<SingleRigidBodyMPC>();
        break;
    case ControllerType::UNSPECIFIED:
        throw "We shouldn't be able to get here";
        break;
    }

    load_ros_params(param_ns);
}

MPCManager::~MPCManager() {
    // No special destruction
}

void MPCManager::update_robot_state(
    const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
    const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
    const AllJointMap<double> &jp,
    const AllJointMap<double> &jv,
    const AllJointMap<double> &jt,
    const std::shared_ptr<Trajectories> &traj)
{
    robot_pos_ = pos;
    robot_rot_ = rot;
    robot_vel_ = vel;
    robot_ang_vel_ = ang_vel;

    SpineJointMap<double> spine_jp;
    spine_jp[SpineJointId::FRONT_PITCH] = jp.at(AllJointId::F_SPINE_PITCH);
    spine_jp[SpineJointId::FRONT_YAW] = jp.at(AllJointId::F_SPINE_YAW);
    spine_jp[SpineJointId::REAR_PITCH] = jp.at(AllJointId::R_SPINE_PITCH);
    spine_jp[SpineJointId::REAR_YAW] = jp.at(AllJointId::R_SPINE_YAW);

    auto fill_maps = [&jp, &jv, &jt](
        auto &jpm, auto &jvm, auto &jtm, LegId leg)
    {
        auto roll_id = full_joint_id(leg, LegJointId::HIP_ROLL);
        auto pitch_id = full_joint_id(leg, LegJointId::HIP_PITCH);
        auto knee_id = full_joint_id(leg, LegJointId::KNEE);
        jpm[LegJointId::HIP_ROLL] = jp.at(roll_id);
        jpm[LegJointId::HIP_PITCH] = jp.at(pitch_id);
        jpm[LegJointId::KNEE] = jp.at(knee_id); 
        jvm[LegJointId::HIP_ROLL] = jv.at(roll_id);
        jvm[LegJointId::HIP_PITCH] = jv.at(pitch_id);
        jvm[LegJointId::KNEE] = jv.at(knee_id);
        jtm[LegJointId::HIP_ROLL] = jt.at(roll_id);
        jtm[LegJointId::HIP_PITCH] = jt.at(pitch_id);
        jtm[LegJointId::KNEE] = jt.at(knee_id);
    };

    LegJointMap<double> jpmx;
    LegJointMap<double> jvmx;
    LegJointMap<double> jtmx;
    fill_maps(jpmx, jvmx, jtmx, LegId::FL);
    robot_fp_[LegId::FL] =
        robot_model_->calculate_fp(LegId::FL, jpmx, spine_jp);
    robot_ff_[LegId::FL] =
        robot_model_->calculate_grf(LegId::FL, jpmx, jvmx, jtmx, spine_jp);

    fill_maps(jpmx, jvmx, jtmx, LegId::FR);
    robot_fp_[LegId::FR] =
        robot_model_->calculate_fp(LegId::FR, jpmx, spine_jp);
    robot_ff_[LegId::FR] =
        robot_model_->calculate_grf(LegId::FR, jpmx, jvmx, jtmx, spine_jp);

    fill_maps(jpmx, jvmx, jtmx, LegId::RL);
    robot_fp_[LegId::RL] =
        robot_model_->calculate_fp(LegId::RL, jpmx, spine_jp);
    robot_ff_[LegId::RL] =
        robot_model_->calculate_grf(LegId::RL, jpmx, jvmx, jtmx, spine_jp);

    fill_maps(jpmx, jvmx, jtmx, LegId::RR);
    robot_fp_[LegId::RR] =
        robot_model_->calculate_fp(LegId::RR, jpmx, spine_jp);
    robot_ff_[LegId::RR] =
        robot_model_->calculate_grf(LegId::RR, jpmx, jvmx, jtmx, spine_jp);

    spine_pos_(0) = spine_jp[SpineJointId::FRONT_PITCH];
    spine_pos_(1) = spine_jp[SpineJointId::FRONT_YAW];
    spine_pos_(2) = spine_jp[SpineJointId::REAR_PITCH];
    spine_pos_(3) = spine_jp[SpineJointId::REAR_YAW];

    for (const auto &leg : ALL_LEG_IDS) {
        if (!traj->stance.at(leg)[0]) {
            robot_ff_[leg] = Eigen::Vector3d::Zero();
        }
    }

    trajs_ = traj;
}

MPCControlOutput MPCManager::control(
    const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
    const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
    const Eigen::Vector3d &cmd_pos, const Eigen::Matrix3d &cmd_rot,
    const Eigen::Vector3d &cmd_lin_vel, const Eigen::Vector3d &cmd_ang_vel,
    const AllJointMap<double> &jp, const AllJointMap<double> &jv,
    const AllJointMap<double> &jt, const LegMap<bool> &foot_s,
    const std::shared_ptr<Trajectories> &traj)
{
    update_robot_state(pos, rot, vel, ang_vel, jp, jv, jt, traj);
    target_pos_ = cmd_pos;
    target_rot_ = cmd_rot;
    cmd_v_ = cmd_lin_vel;
    cmd_w_ = cmd_ang_vel;
    robot_stance_ = foot_s;
    if(!fill_state(state_)) {
        ROS_ERROR("FAILED TO FILL STATE");
        return MPCControlOutput();
    }
    auto solu = mpc_->solve(state_);
    return extract_solu(solu);
}

MPCControlOutput MPCManager::control(
    const Eigen::Vector3d &pos, const Eigen::Matrix3d &rot,
    const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel,
    const Eigen::Vector3d &cmd_pos, const Eigen::Matrix3d &cmd_rot,
    const Eigen::Vector3d &cmd_lin_vel, const Eigen::Vector3d &cmd_ang_vel,
    const LegMap<Eigen::Vector3d> &foot_p,
    const LegMap<Eigen::Vector3d> &foot_f,
    const LegMap<bool> &foot_s,
    const std::shared_ptr<Trajectories> &trajs,
    const Eigen::Vector4d &spine_pos,
    const Eigen::Vector4d &spine_vel)
{
    robot_pos_ = pos;
    robot_rot_ = rot;
    robot_vel_ = vel;
    robot_ang_vel_ = ang_vel;
    robot_fp_ = foot_p;
    robot_ff_ = foot_f;
    robot_stance_ = foot_s;
    trajs_ = trajs;
    target_pos_ = cmd_pos;
    target_rot_ = cmd_rot;
    cmd_v_ = cmd_lin_vel;
    cmd_w_ = cmd_ang_vel;
    spine_pos_ = spine_pos;
    spine_vel_ = spine_vel;
    if(!fill_state(state_)) {
        ROS_ERROR("FAILED TO FILL STATE");
        return MPCControlOutput();
    }
    solu_ = mpc_->solve(state_);
    return extract_solu(solu_);
}

bool MPCManager::fill_state(const std::shared_ptr<MPCBase::State> &state) {
    // Current variables are in an axis aligned global coordinate frame,
    // except angular velocities which are in an body frame
    // We want to be in a current pose centered inertial frame
    // By definition: zero (robot_pos_ - robot_pos_)
    state->r = Eigen::Vector3d::Zero();
    // Already in global axis-aligned inertial frame
    state->R = robot_rot_;
    // Already in global axis-aligned inertial frame
    state->g = Eigen::Vector3d(0, 0, -9.81);
    // Already in global axis-aligned inertial frame
    state->v = robot_vel_;
    // Already in body frame
    state->w = robot_ang_vel_;

    switch (type_) {
        case ControllerType::SRBD:
            return fill_state(
                std::static_pointer_cast<SingleRigidBodyMPC::State>(state));
            break;
        default:
            throw "We shouldn't get here";
    }

    return false;
}

bool MPCManager::fill_state(
    const std::shared_ptr<SingleRigidBodyMPC::State> &state)
{
    if (trajs_ == nullptr || trajs_->pos[LegId::FL].size() == 0) {
        if (trajs_ == nullptr) {
            ROS_ERROR("NO TRAJS");
        }
        else {
            ROS_ERROR("TRAJS EMPTY");
        }
        return false;
    }
    interpolate_trajectories(state->stance_state, trajs_);
    state->stance_state[0] = robot_stance_;

    for (const auto &leg : ALL_LEG_IDS) {
        // Convert to gloabl frame and account for com offset
        state->foot_positions[leg] = robot_rot_ * (robot_fp_.at(leg) - robot_com_offset_);
    }

    double acc = state->acc_limit * state->dt;
    // Already in global axis-aligned inertial frame
    Eigen::Vector3d dv = cmd_v_ - strt_tgt_vel_;
    /**
     * \todo This shouldn't be dt, this should be time since last update
     */
    if (dv.norm() > acc) {
        dv = dv.normalized() * acc;
    }
    state->v_d[0] = strt_tgt_vel_;
    strt_tgt_vel_ += dv;
    state->w_d[0] = cmd_w_;

    // Current position is always zero. Therefore current target is
    // to_robot_frame(tgt - now), next target is tgt - now + avg velocities
    state->r_d[0] = (target_pos_ - robot_pos_)
                  + (state->v + state->v_d[0]) * state->dt / 2;
    // All currently in global frame
    state->R_d[0] = Sophus::SO3d::exp(state->w_d[0] * state->dt).matrix()
                  * target_rot_;

    double sum_stance = 0;
    for (const auto & id : ALL_LEG_IDS) {
        sum_stance += state->stance_state[0].at(id) ? 1 : 0;
    }
    double foot_f = state->mass * 9.81 / sum_stance;
    // Foot forces already in body frame need to be in axis-aligned inertial
    state->current_foot_forces <<
        robot_rot_ * robot_ff_[LegId::FL],
        robot_rot_ * robot_ff_[LegId::FR],
        robot_rot_ * robot_ff_[LegId::RL],
        robot_rot_ * robot_ff_[LegId::RR];
    state->u_d[0] <<
        0, 0, state->stance_state[0].at(LegId::FL) ? foot_f : 0,
        0, 0, state->stance_state[0].at(LegId::FR) ? foot_f : 0,
        0, 0, state->stance_state[0].at(LegId::RL) ? foot_f : 0,
        0, 0, state->stance_state[0].at(LegId::RR) ? foot_f : 0;

    for (unsigned int i = 1; i < state->horizon; i++) {
        // cmd_v_ is in global frame
        dv = cmd_v_ - state->v_d[i-1];
        if (dv.norm() > acc) {
            dv = dv.normalized() * acc;
        }
        state->v_d[i] = state->v_d[i-1] + dv;
        // cmd_w_ is in body frame
        state->w_d[i] = cmd_w_;

        state->r_d[i] = state->r_d[i-1]
                      + (state->v_d[i-1] + state->v_d[i]) * state->dt / 2;
        state->R_d[i] =
            Sophus::SO3d::exp(state->w_d[i-1] * state->dt).matrix()
            * state->R_d[i-1];
        
        sum_stance = 0;
        for (const auto & id : ALL_LEG_IDS) {
            sum_stance += state->stance_state[i].at(id) ? 1 : 0;
        }
        foot_f = state->mass * 9.81 / sum_stance;
        state->u_d[i] <<
            0, 0, state->stance_state[i].at(LegId::FL) ? foot_f : 0,
            0, 0, state->stance_state[i].at(LegId::FR) ? foot_f : 0,
            0, 0, state->stance_state[i].at(LegId::RL) ? foot_f : 0,
            0, 0, state->stance_state[i].at(LegId::RR) ? foot_f : 0;
    }

    return true;
}

void MPCManager::interpolate_trajectories(
    std::vector<LegMap<bool>> &stance,
    const std::shared_ptr<Trajectories> &trajs)
{
    const double EPSILON = 1e-6;
    const unsigned int num_pts = state_->horizon;
    stance.clear();
    if (std::abs(state_->dt - trajs->timestep) < EPSILON) {
        // Treat as identical timestep
        for (unsigned int i = 0; i < num_pts && i < trajs->stance.at(LegId::FL).size(); i++) {
            LegMap<bool> st;
            for (const auto &leg : ALL_LEG_IDS) {
                st[leg] = trajs->stance.at(leg).at(i);
            }
            stance.push_back(st);
        }
    }
    else {
        // Then we need to interpolate
        double T = 0;
        unsigned int traj_i = 0;
        double T_traj = double(traj_i) * trajs->timestep;
        // Loop until enough points or out of trajectories to interpolate
        while (stance.size() != num_pts && traj_i < trajs->stance.at(LegId::FL).size() - 1) {
            LegMap<bool> st;
            for (const auto &leg : ALL_LEG_IDS) {
                st[leg] = trajs->stance.at(leg)[traj_i]
                       && trajs->stance.at(leg)[traj_i+1];
            }
            stance.push_back(st);
            T += state_->dt;
            // Update trajectory time and counter only if the current time
            // passes the boundary
            while(T + state_->dt > T_traj + trajs->timestep) {
                traj_i++;
                T_traj = double(traj_i) * trajs->timestep;
            }
        }
    }
    // Special case if trajs->pos is only size 1 we might not have added any points
    if (stance.size() == 0) {
        LegMap<bool> st;
        for (const auto &leg : ALL_LEG_IDS) {
            st[leg] = trajs->stance.at(leg).front();
        }
        stance.push_back(st);
    }
    // If the vector is not long enough, just add the final state
    while(stance.size() < num_pts) {
        stance.push_back(stance.back());
    }
}

void MPCManager::interpolate_spine(
    std::vector<Eigen::Vector4d> &spine_pos,
    std::vector<Eigen::Vector4d> &spine_vel,
    const std::shared_ptr<Trajectories> &trajs)
{
    const double EPSILON = 1e-6;
    const unsigned int num_pts = state_->horizon;
    spine_pos.clear();
    spine_vel.clear();

    if (std::abs(state_->dt - trajs->timestep) < EPSILON) {
        // Treat as identical timestep
        for (unsigned int i = 0; i < num_pts && i < trajs->spine_pos.size(); i++) {
            Eigen::Vector4d sp;
            sp.setZero();
            if (trajs->contains_spine_positions) {
                for (const auto &spine : ALL_SPINE_JOINTS) {
                    sp((unsigned int)spine) = trajs->spine_pos[i][spine];
                }
            }
            spine_pos.push_back(sp);
        }
    }
    else {
        // Then we need to interpolate
        auto interp_pos = [](
            const auto &A, const auto &B, auto TA, auto TB, auto T)
        {
            double pc = (T - TA) / (TB - TA);
            return (1-pc) * A + pc * B;
        };
        double T = 0;
        unsigned int traj_i = 0;
        double T_traj = double(traj_i) * trajs->timestep;
        // Loop until enough points or out of trajectories to interpolate
        while (spine_pos.size() != num_pts && traj_i < trajs->spine_pos.size() - 1) {
            Eigen::Vector4d sp;
            sp.setZero();
            if (trajs->contains_spine_positions) {
                for (const auto &spine : ALL_SPINE_JOINTS) {
                    sp((unsigned int)spine) = interp_pos(
                        trajs->spine_pos[traj_i][spine],
                        trajs->spine_pos[traj_i+1][spine],
                        T_traj, T_traj + trajs->timestep, T);
                }
            }
            spine_pos.push_back(sp);
            T += state_->dt;
            // Update trajectory time and counter only if the current time
            // passes the boundary
            while(T + state_->dt > T_traj + trajs->timestep) {
                traj_i++;
                T_traj = double(traj_i) * trajs->timestep;
            }
        }
    }
    // Special case if trajs->spine_pos is only size 1 we might not have added any points
    if (spine_pos.size() == 0) {
        Eigen::Vector4d sp;
        sp.setZero();
        if (trajs->contains_spine_positions) {
            for (const auto &spine : ALL_SPINE_JOINTS) {
                sp((unsigned int)spine) = trajs->spine_pos[0][spine];
            }
        }
        spine_pos.push_back(sp);
    }
    // If the vector is not long enough, just add the final state
    while(spine_pos.size() < num_pts) {
        spine_pos.push_back(spine_pos.back());
    }
    // Calculate spine velocities
    for (unsigned int i = 0; i < spine_pos.size() - 1; i++) {
        Eigen::Vector4d sv;
        sv = (spine_pos[i+1] - spine_pos[i]) / state_->dt;
        spine_vel.push_back(sv);
    }
    // One last one for the final timestep
    spine_vel.push_back(spine_vel.back());
}

MPCControlOutput MPCManager::extract_solu(
    const std::shared_ptr<MPCBase::Solution> &solu)
{
    switch (type_) {
    case ControllerType::SRBD:
        return extract_solu(
            std::static_pointer_cast<SingleRigidBodyMPC::Solution>(solu));
        break;
    default:
        throw "We shouldn't get here";
    }

    // Can't get here because of throw above, but prevent compile warnings
    return MPCControlOutput();
}

MPCControlOutput MPCManager::extract_solu(
    const std::shared_ptr<SingleRigidBodyMPC::Solution> &solu)
{
    MPCControlOutput control;
    control.contains_foot_forces = true;
    for (const auto & leg : ALL_LEG_IDS) {
        // Convert to local frame
        control.foot_forces[leg] = robot_rot_.transpose() * solu->foot_grfs[0][leg];
    }

    control.contains_spine_pos = false;
    control.contains_spine_vel = false;
    control.contains_spine_eff = false;
    return control;
}

template<typename T>
void LOAD_MPC_PARAM(
    std::string ns, std::string param, T &val,
    const T &default_val)
{
    // remove trailing "/" on ns and leading "/" on param
    while (ns.back() == '/') {
        ns.pop_back();
    }
    while (param.front() == '/') {
        param.erase(param.front());
    }
    if (!ros::param::param<T>(ns + '/' + param, val, default_val)) {
        ROS_WARN_STREAM(
            "Parameter '" << ns + '/' + param << "' not set. Defaulting to "
            << val);
    }
}

void LOAD_MPC_PARAM_MAT(
    std::string ns, std::string param, Eigen::MatrixXd &val,
    Eigen::MatrixXd &default_val)
{
    // remove trailing "/" on ns and leading "/" on param
    while (ns.back() == '/') {
        ns.pop_back();
    }
    while (param.front() == '/') {
        param.erase(param.front());
    }
    XmlRpc::XmlRpcValue config;
    if (ros::param::has(ns + '/' + param)) {
        ros::param::get(ns + '/' + param, config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        int s = default_val.rows();
        for (int i = 0; i < default_val.cols(); i++) {
            for (int j = 0; j < s; j++) {
                try {
                    std::stringstream ss;
                    ss << config[s * i + j];
                    ss >> val(i, j);
                }
                catch (XmlRpc::XmlRpcException &e) {
                    throw e;
                }
                catch(...) {
                    throw;
                }
            }
        }
    }
    else {
        val = default_val;
        ROS_WARN_STREAM(
            "Parameter '" << ns + '/' + param << "' not set. Defaulting to "
            << val);
    }
}

void LOAD_MPC_PARAM_VEC(
    std::string ns, std::string param, Eigen::VectorXd &val,
    Eigen::VectorXd &default_val)
{
    // remove trailing "/" on ns and leading "/" on param
    while (ns.back() == '/') {
        ns.pop_back();
    }
    while (param.front() == '/') {
        param.erase(param.front());
    }
    XmlRpc::XmlRpcValue config;
    if (ros::param::has(ns + '/' + param)) {
        ros::param::get(ns + '/' + param, config);
        ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        int s = default_val.rows();
        for (int i = 0; i < s; i++) {
            try {
                std::stringstream ss;
                ss << config[i];
                ss >> val(i);
            }
            catch (XmlRpc::XmlRpcException &e) {
                throw e;
            }
            catch(...) {
                throw;
            }
        }
    }
    else {
        val = default_val;
        ROS_WARN_STREAM(
            "Parameter '" << ns + '/' + param << "' not set. Defaulting to "
            << val);
    }
}

void MPCManager::load_ros_params(const std::string ns) {
    int horiz_int;
    LOAD_MPC_PARAM(ns, "horizon", horiz_int, 10);
    state_->horizon = horiz_int;
    LOAD_MPC_PARAM(ns, "time_step", state_->dt, 0.01);
    LOAD_MPC_PARAM(ns, "acc_limit", state_->acc_limit, 1.);
    LOAD_MPC_PARAM(ns, "decay_rate", state_->decay_rate, 1.0);
    LOAD_MPC_PARAM(ns, "mass", state_->mass, 3.5);
    // TODO: load from urdf
    state_->I << state_->mass * (0.2 * 0.2 + 0.25 * 0.25) / 12, 0, 0,
                 0, state_->mass * (0.4 * 0.4 + 0.25 * 0.25) / 12, 0,
                 0, 0, state_->mass * (0.4 * 0.4 + 0.2 * 0.2) / 12;
    Eigen::MatrixXd I = state_->I;
    LOAD_MPC_PARAM_MAT(ns, "inertia", I, I);
    state_->I = I;
    state_->invI = state_->I.inverse();

    Eigen::VectorXd com_offset = Eigen::Vector3d::Zero();
    LOAD_MPC_PARAM_VEC(ns, "com_offset", com_offset, com_offset);
    robot_com_offset_ = com_offset;

    if (type_ == ControllerType::SRBD) {
        auto state = std::static_pointer_cast<SingleRigidBodyMPC::State>(
            state_);
        state->stance_state.resize(state->horizon);
        state->r_d.resize(state->horizon);
        state->R_d.resize(state->horizon);
        state->v_d.resize(state->horizon);
        state->w_d.resize(state->horizon);
        state->u_d.resize(state->horizon);
        LOAD_MPC_PARAM(ns, "srbd/mu", state->mu, 0.5);

        state->Q_x.diagonal() <<
            1e5, 2e5, 3e5,
            5e2, 1e3, 1e3,
            1e3, 1e4, 8e2,
            4e1, 4e1, 1e1;
        state->Qf_x = state->Q_x;
        state->Q_u.diagonal() <<
            1e-1, 2e-1, 1e-1,
            1e-1, 2e-1, 1e-1,
            1e-1, 2e-1, 1e-1,
            1e-1, 2e-1, 1e-1;
        Eigen::VectorXd v = state->Q_x.diagonal();
        LOAD_MPC_PARAM_VEC(ns, "srbd/Q_x", v, v);
        state->Q_x.diagonal() = v;
        v = state->Qf_x.diagonal();
        LOAD_MPC_PARAM_VEC(ns, "srbd/Qf_x", v, v);
        state->Qf_x.diagonal() = v;
        v = state->Q_u.diagonal();
        LOAD_MPC_PARAM_VEC(ns, "srbd/Q_u", v, v);
        state->Q_u.diagonal() = v;
    }
}
