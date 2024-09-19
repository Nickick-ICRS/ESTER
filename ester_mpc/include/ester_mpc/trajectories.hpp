#ifndef __TRAJECTORIES_HPP__
#define __TRAJECTORIES_HPP__

#include <Eigen/Geometry>

#include <ester_common/ester_enums.hpp>

#include <ros/ros.h>

namespace ester_mpc {

struct Trajectories {
    ester_common::LegMap<std::vector<Eigen::Vector3d>> pos;
    ester_common::LegMap<std::vector<bool>> stance;
    double timestep;
    bool contains_spine_positions;
    std::vector<ester_common::SpineJointMap<double>> spine_pos;
};

inline Eigen::Vector3d raibert_heuristic(
    const double t_stance,
    const double max_leg_len,
    const Eigen::Vector3d &zero,
    const Eigen::Vector2d &lin_vel,
    const Eigen::Vector2d &tgt_lin_vel,
    const Eigen::Vector2d &max_stride_length)
{
    Eigen::Vector3d tgt = zero;
    double K = std::sqrt(std::abs(tgt.z() - max_leg_len) / 9.8);
    Eigen::Vector2d delta =
        K * (lin_vel - tgt_lin_vel) + t_stance * tgt_lin_vel / 2;
    delta = delta.cwiseMin(max_stride_length/2).cwiseMax(-max_stride_length/2);
    tgt.x() += delta.x();
    tgt.y() += delta.y();
    return tgt;  
}

} // ns ester_mpc

#endif // __TRAJECTORIES_HPP__