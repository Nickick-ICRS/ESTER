#include "ester_mpc/trajectory_planner.hpp"

using namespace ester_common;

namespace ester_mpc {

TrajectoryPlanner::TrajectoryPlanner(double start_lift, double end_lift)
    :foot_swing_clearance_start_(start_lift),
     foot_swing_clearance_end_(end_lift)
{
    // ctor
}

const std::shared_ptr<Trajectories>
TrajectoryPlanner::generate_trajectories(
    const TrajectoryParameters &params)
{
    assert(params.timestep > 1e-6);
    assert(params.num_pts != 0);
    auto trajectories = std::make_shared<Trajectories>();
    trajectories->timestep = params.timestep;
    double total_dur = params.swing_dur + params.stance_dur;
    double swing_phase = 1. - params.swing_dur / total_dur;
    double dpc = params.timestep / total_dur;
    for (const auto &leg : ALL_LEG_IDS) {
        double t = params.phase_counter.at(leg);
        bool entered_swing = t > swing_phase;
        Eigen::Vector3d strt = params.swing_start_pos.at(leg);
        Eigen::Vector3d zero = params.foot_zeros.at(leg);
        Eigen::Vector3d end = raibert_heuristic(
                    params.stance_dur, params.max_leg_len, zero,
                    params.lin_vel, params.cmd_lin_vel,
                    params.max_stride_length);
        // Special case, if we haven't entered swing yet then end is current pos
        if (!entered_swing) {
            strt = params.foot_pos.at(leg);
            end = params.foot_pos.at(leg);
        }
        while (trajectories->pos[leg].size() < params.num_pts) {
            if (t >= 1) {
                // Switch from swing to stance, so update start, zero and end
                zero.x() += (end - strt).x();
                zero.y() += (end - strt).y();
                strt = end;
                end = raibert_heuristic(
                    params.stance_dur, params.max_leg_len, zero,
                    params.lin_vel, params.cmd_lin_vel,
                    params.max_stride_length);
                t -= 1;
                entered_swing = true;
            }
            if (t <= swing_phase) {
                // In stance - position must be where we finished the step
                trajectories->pos[leg].emplace_back(end);
                trajectories->stance[leg].emplace_back(true);
            }
            else {
                // In swing - calculate position of bezier curve
                trajectories->pos[leg].emplace_back(get_foot_pos_bezier(
                    (t - swing_phase) / (1 - swing_phase), strt, end));
                trajectories->stance[leg].emplace_back(false);
            }
            t += dpc;
        }
    }
    trajectories->contains_spine_positions = params.calculate_spine_positions; 
    if (params.calculate_spine_positions) {
        auto foot_twist = [](const Eigen::Vector3d &foot_vector) {
            Eigen::Vector2d proj_vec = foot_vector.segment<2>(0);
            double theta = std::acos(proj_vec.dot(Eigen::Vector2d::UnitY()) / proj_vec.norm());
            if (proj_vec.x() < 0) {
                return theta;
            }
            return -theta;
        };
        for (unsigned int i = 0; i < params.num_pts; i++) {
            SpineJointMap<double> spine_pos;
            LegMap<Eigen::Vector3d> feet_pos;
            LegMap<double> dx;
            for (const auto &leg : ALL_LEG_IDS) {
                feet_pos[leg] = trajectories->pos[leg][i];
                dx[leg] = params.foot_zeros.at(leg).x() - feet_pos[leg].x();
            }
            spine_pos[SpineJointId::FRONT_PITCH] =
                params.Kpitch * (dx[LegId::FL] + dx[LegId::FR]) / 2;
            spine_pos[SpineJointId::REAR_PITCH] =
                params.Kpitch * (dx[LegId::RL] + dx[LegId::RR]) / 2;
            
            spine_pos[SpineJointId::FRONT_YAW] =
                params.Kyaw * foot_twist(feet_pos[LegId::FL] - feet_pos[LegId::FR]);
            spine_pos[SpineJointId::REAR_YAW] =
                params.Kyaw * foot_twist(feet_pos[LegId::RL] - feet_pos[LegId::RR]);

            trajectories->spine_pos.push_back(spine_pos);
        }
    }
    return trajectories;
}

Eigen::Vector3d TrajectoryPlanner::get_foot_pos_bezier(
    double t, const Eigen::Vector3d &start, const Eigen::Vector3d &end) const
{
    assert(0 <= t && t <= 1);

    Eigen::Vector3d foot_pos_target;
    for (unsigned int i = 0; i < 3; i++) {
        // Apply Z offset for 3rd (z) dimension
        foot_pos_target(i) = bezier(t, start(i), end(i), i == 2 ? true : false);
    }
    return foot_pos_target;
}

double TrajectoryPlanner::bezier(
    double t, double p0, double p1, bool apply_z_offset) const
{
    const unsigned int DEGREE = 4;
    const std::array<double, DEGREE+1> coeffs {1, 4, 6, 4, 1};
    std::array<double, DEGREE+1> ctrl {p0, p0, p1, p1, p1};
    if (apply_z_offset) {
        ctrl[1] += foot_swing_clearance_start_;
        ctrl[2] += foot_swing_clearance_end_;
    }

    double x = 0;
    for (unsigned int i = 0; i <= DEGREE; i++) {
        x += coeffs[i] * std::pow(t, i) * std::pow(1 - t, DEGREE - i) * ctrl[i];
    }
    return x;
}

} // ns ester_mpc