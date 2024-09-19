#include "ester_mpc/gait_manager.hpp"

using namespace ester_common;

namespace ester_mpc {

std::string str(const GaitType &gait) {
    switch(gait) {
    case GaitType::STATIONARY:
        return "STATIONARY";
    case GaitType::WALKING:
        return "WALKING";
    case GaitType::TROTTING:
        return "TROTTING";
    case GaitType::BOUNDING:
        return "BOUNDING";
    case GaitType::PACING:
        return "PACING";
    case GaitType::PRONKING:
        return "PRONKING";
    default:
        throw std::runtime_error("Unknown gait: " + std::to_string((int)gait));
    }
    return "";
}

std::ostream& operator<<(std::ostream &os, const GaitType &gait) {
    return os << str(gait);
}

GaitManager::GaitManager(
    const LegMap<Eigen::Vector3d> &feet_pos,
    const XmlRpc::XmlRpcValue &config,
    const std::shared_ptr<ros::NodeHandle> &nh)
    : manual_gait_control_(false)
{
    if (nh) {
        pub_to_ros_ = true;
        phase_pub_ = nh->advertise<std_msgs::Float32MultiArray>("/trajectories/foot_phases", 1);
    }
    else {
        pub_to_ros_ = false;
    }
    trajs_ = std::make_shared<Trajectories>();
    traj_future_ = 5.;
    t_params_.timestep = 0.01;
    t_params_.foot_pos = feet_pos;
    reset(feet_pos, true);

    std::array<GaitType, 6> gaits = {
        GaitType::STATIONARY, GaitType::WALKING, GaitType::TROTTING,
        GaitType::PACING, GaitType::BOUNDING, GaitType::PRONKING,
    };
    const std::string gst = "generate_spine_trajectories";
    if (config.hasMember(gst)) {
        consider_spine_positions_ = config[gst];
    }
    else {
        consider_spine_positions_ = false;
        ROS_WARN_STREAM("No config found: /ester/gait_manager/" << gst);
    }
    for (const auto &gait : gaits) {
        std::string name = str(gait);
        if (config.hasMember(name)) {
            stance_durs_map_[gait] = config[name]["stance_dur"];
            ratios_map_[gait] = config[name]["swing_ratio"];
            Kpitch_map_[gait] = config[name]["Kpitch"];
            Kyaw_map_[gait] = config[name]["Kyaw"];
            phase_map_[gait][LegId::FL] = config[name]["phases"][0];
            phase_map_[gait][LegId::FR] = config[name]["phases"][1];
            phase_map_[gait][LegId::RL] = config[name]["phases"][2];
            phase_map_[gait][LegId::RR] = config[name]["phases"][3];
        }
        else {
            ROS_WARN_STREAM("No config found: /ester/gait_manager/" << name);
        }
    }
}

const std::shared_ptr<Trajectories> &GaitManager::update_trajectories(
    double dt, const Eigen::Vector6d &current_vel,
    const Eigen::Vector6d &cmd_vel, const double cmd_height,
    const LegMap<Eigen::Vector3d> &foot_p)
{
    if (!manual_gait_control_ && current_gait_ == target_gait_) {
        // Consider the speed of the gait and determine the best gait
        switch(current_gait_) {
        case GaitType::STATIONARY:
            if (cmd_vel.squaredNorm() > 3e-3) {
                enter_walking();
            }
            break;
        case GaitType::WALKING:
            if (cmd_vel.tail<3>().squaredNorm() > 0.4) {
                enter_trotting();
            }
            // Don't exit walking unless explicitly requested
            break;
        case GaitType::TROTTING:
            if (cmd_vel.tail<3>().squaredNorm() > 1.5) {
                enter_bounding();
            }
            else if (cmd_vel.tail<3>().squaredNorm() < 0.3) {
                enter_walking();
            }
            break;
        case GaitType::BOUNDING:
            if (cmd_vel.tail<3>().squaredNorm() < 1.3) {
                enter_trotting();
            }
            break;
        default:
            // We don't know how to handle other gaits, so return to trotting
            enter_trotting();
            ROS_WARN_STREAM(
                "Unable to handle gait type " << current_gait_
                << " automatically, reverting to " << target_gait_);
            break;
        }
        if (current_gait_ != target_gait_) {
            ROS_INFO_STREAM("Switching from " << current_gait_ << " to " << target_gait_);
        }
    }
    switch(current_gait_) {
    case GaitType::STATIONARY:
        prepare_stationary_gait(dt, cmd_height, current_vel, foot_p);
        break;
    default:
        prepare_moving_gait(dt, cmd_height, current_vel, cmd_vel, foot_p);
        break;
    }
    if (current_gait_ != target_gait_) {
        update_gait_phase_transition(dt);
    }
    else {
        update_gait_phase(dt);
    }

    if (pub_to_ros_) {
        std_msgs::Float32MultiArray msg;
        for (const auto &leg : ALL_LEG_IDS) {
            msg.data.push_back(t_params_.phase_counter[leg]);
        }
        phase_pub_.publish(msg);
    }
    
    trajs_ = tp_->generate_trajectories(t_params_);
    return trajs_;
}

void GaitManager::reset(
    const LegMap<Eigen::Vector3d> &current_feet_pos, bool reset_gait)
{
    if (reset_gait) {
        current_gait_ = GaitType::STATIONARY;
        target_gait_ = GaitType::STATIONARY;

        t_params_.phase_counter[LegId::FL] = 0;
        t_params_.phase_counter[LegId::FR] = 0.5;
        t_params_.phase_counter[LegId::RL] = 0.75;
        t_params_.phase_counter[LegId::RR] = 0.25;
    }

    t_params_.foot_zeros[LegId::FL] = Eigen::Vector3d( 0.114,  0.134, -0.2);
    t_params_.foot_zeros[LegId::FR] = Eigen::Vector3d( 0.114, -0.134, -0.2);
    t_params_.foot_zeros[LegId::RL] = Eigen::Vector3d(-0.174,  0.134, -0.2);
    t_params_.foot_zeros[LegId::RR] = Eigen::Vector3d(-0.174, -0.134, -0.2);

    t_params_.stance_dur = 0.1;
    t_params_.swing_dur = 0.;
    t_params_.timestep = 0.01;
    // Request trajectories 5s into the future for visualisation
    t_params_.num_pts = 5.0 / t_params_.timestep;
    t_params_.lin_vel = Eigen::Vector2d(0.0, 0.0);
    t_params_.cmd_lin_vel = Eigen::Vector2d(0.0, 0.0);
    t_params_.max_stride_length = Eigen::Vector2d(0.11, 0.03);
    t_params_.max_leg_len = -0.32;

    t_params_.foot_pos = current_feet_pos;
    t_params_.swing_start_pos = t_params_.foot_pos;

    // TODO: Read parameters from ROS
    tp_ = std::make_unique<TrajectoryPlanner>(0.03, 0.05);

    manual_gait_control_ = false;

    enter_stationary();
}

void GaitManager::set_gait_type(const GaitType &gait) {
    enter_gait(gait);
    manual_gait_control_ = true;
}

void GaitManager::return_to_automatic_gait_control(const GaitType &gait) {
    enter_gait(gait);
    manual_gait_control_ = false;
}

void GaitManager::enter_gait(const GaitType &gait) {
    switch(gait) {
    case GaitType::BOUNDING:
        enter_bounding();
        break;
    case GaitType::PACING:
        enter_pacing();
        break;
    case GaitType::PRONKING:
        enter_pronking();
        break;
    case GaitType::STATIONARY:
        enter_stationary();
        break;
    case GaitType::TROTTING:
        enter_trotting();
        break;
    case GaitType::WALKING:
        enter_walking();
        break;
    }
}

void GaitManager::enter_stationary() {
    target_gait_ = GaitType::STATIONARY;

    target_phases_ = phase_map_[GaitType::STATIONARY];
    t_params_.calculate_spine_positions = false;
    t_params_.Kpitch = Kpitch_map_[GaitType::STATIONARY];
    t_params_.Kyaw = Kyaw_map_[GaitType::STATIONARY];
}

void GaitManager::enter_walking() {
    target_gait_ = GaitType::WALKING;

    target_phases_ = phase_map_[GaitType::WALKING];

    t_params_.calculate_spine_positions = true && consider_spine_positions_;
    t_params_.Kpitch = Kpitch_map_[GaitType::WALKING];
    t_params_.Kyaw = Kyaw_map_[GaitType::WALKING];
}

void GaitManager::enter_trotting() {
    target_gait_ = GaitType::TROTTING;

    target_phases_ = phase_map_[GaitType::TROTTING];

    t_params_.calculate_spine_positions = true && consider_spine_positions_;
    t_params_.Kpitch = Kpitch_map_[GaitType::TROTTING];
    t_params_.Kyaw = Kyaw_map_[GaitType::TROTTING];
}

void GaitManager::enter_pacing() {
    target_gait_ = GaitType::PACING;

    target_phases_ = phase_map_[GaitType::PACING];

    t_params_.calculate_spine_positions = false;
    t_params_.Kpitch = Kpitch_map_[GaitType::PACING];
    t_params_.Kyaw = Kyaw_map_[GaitType::PACING];
}

void GaitManager::enter_pronking() {
    target_gait_ = GaitType::PRONKING;

    target_phases_ = phase_map_[GaitType::PRONKING];

    t_params_.calculate_spine_positions = false;
    t_params_.Kpitch = Kpitch_map_[GaitType::PRONKING];
    t_params_.Kyaw = Kyaw_map_[GaitType::PRONKING];
}

void GaitManager::enter_bounding() {
    target_gait_ = GaitType::BOUNDING;

    target_phases_ = phase_map_[GaitType::BOUNDING];

    t_params_.calculate_spine_positions = true && consider_spine_positions_;
    t_params_.Kpitch = Kpitch_map_[GaitType::BOUNDING];
    t_params_.Kyaw = Kyaw_map_[GaitType::BOUNDING];
}

void GaitManager::update_gait_phase_transition(double dt) {
    double dphase = dt / get_phase_dur();
    const double TOL = 1e-3;
    bool transition_over = true;
    for (const auto &leg : ALL_LEG_IDS) {
        double max_delta;
        if (target_phases_[leg] > t_params_.phase_counter[leg]) {
            max_delta = target_phases_[leg] - t_params_.phase_counter[leg];
        }
        else {
            max_delta = 1 + target_phases_[leg] - t_params_.phase_counter[leg];
        }
        if (max_delta < dphase || max_delta >= 1 - TOL) {
            t_params_.phase_counter[leg] = target_phases_[leg];
            t_params_.swing_start_pos[leg] = t_params_.foot_pos.at(leg);
        }
        else {
            t_params_.phase_counter[leg] += dphase;
            transition_over = false;
        }
    }
    if (transition_over) {
        ROS_INFO_STREAM("Finished transition from " << current_gait_ << " to " << target_gait_);
        current_gait_ = target_gait_;
    }
}

void GaitManager::update_gait_phase(double dt) {
    double dphase = dt / (t_params_.stance_dur + t_params_.swing_dur);
    for (const auto &leg : ALL_LEG_IDS) {
        double max_delta;
        double old_phase = t_params_.phase_counter[leg];
        double swing_phase = 1. - t_params_.swing_dur
            / (t_params_.stance_dur + t_params_.swing_dur);
        double new_phase = old_phase + dphase;
        if (old_phase < swing_phase && new_phase >= swing_phase) {
            // New starting position is the current foot position
            t_params_.swing_start_pos[leg] = t_params_.foot_pos.at(leg);
        }
        if (new_phase > 1) {
            // Wrap around
            new_phase -= 1;
        }
        t_params_.phase_counter[leg] = new_phase;
    }
}

void GaitManager::prepare_stationary_gait(
    double dt, double cmd_height, const Eigen::Vector6d &body_vel,
    const ester_common::LegMap<Eigen::Vector3d> &foot_p)
{
    t_params_.num_pts = 1;
    t_params_.cmd_lin_vel = Eigen::Vector2d::Zero();
    t_params_.lin_vel.x() = body_vel(3);
    t_params_.lin_vel.y() = body_vel(4);

    t_params_.foot_pos = foot_p;
    for (const auto &leg : ALL_LEG_IDS) {
        t_params_.foot_zeros[leg].z() = -cmd_height;
        Eigen::Vector3d pos_err = t_params_.foot_zeros[leg] - t_params_.foot_pos[leg];
        if (pos_err.norm() > 0.05) {
            // We go into walking to re-balance the robot
            enter_walking();
        }
    }
}

void GaitManager::prepare_moving_gait(
    double dt, double cmd_height, const Eigen::Vector6d &body_vel,
    const Eigen::Vector6d &cmd_vel,
    const ester_common::LegMap<Eigen::Vector3d> &foot_p)
{
    t_params_.num_pts = (traj_future_ / t_params_.timestep) + 1;
    t_params_.cmd_lin_vel.x() = cmd_vel(3);
    t_params_.cmd_lin_vel.y() = cmd_vel(4);
    t_params_.lin_vel.x() = body_vel(3);
    t_params_.lin_vel.y() = body_vel(4);

    if (t_params_.cmd_lin_vel.norm() < 1e-3) {
        t_params_.stance_dur = stance_durs_map_[current_gait_];
    }
    else {
        double max_stance_dur_x = t_params_.max_stride_length.x() / t_params_.cmd_lin_vel.x();
        double max_stance_dur_y = t_params_.max_stride_length.y() / t_params_.cmd_lin_vel.y();

        t_params_.stance_dur = std::min(
            max_stance_dur_x,
            std::min(max_stance_dur_y, stance_durs_map_[current_gait_]));
    }

    double ratio = ratios_map_[current_gait_];
    t_params_.swing_dur = t_params_.stance_dur * (ratio / (1 - ratio));

    t_params_.foot_pos = foot_p;
    for (const auto &leg : ALL_LEG_IDS) {
        t_params_.foot_zeros[leg].z() = -cmd_height;
    }
}

} // namespace ester_mpc