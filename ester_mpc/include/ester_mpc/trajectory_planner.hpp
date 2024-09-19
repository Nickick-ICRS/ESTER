#ifndef __TRAJECTORY_PLANNER_HPP__
#define __TRAJECTORY_PLANNER_HPP__

#include "ester_mpc/trajectories.hpp"

namespace ester_mpc {

struct TrajectoryParameters {
    /**
     * @brief The current phase state for each leg (0-1)
     */
    ester_common::LegMap<double> phase_counter;

    /**
     * @brief Requested total duration of the stance stage (s)
     */
    double stance_dur = 0.3;

    /**
     * @brief Requested total duration of the swing stage (s)
     */
    double swing_dur = 0.1;

    /**
     * @brief Starting position of the foot swing trajectory
     */
    ester_common::LegMap<Eigen::Vector3d> swing_start_pos;

    /**
     * @brief Current position of the foot
     */
    ester_common::LegMap<Eigen::Vector3d> foot_pos;

    /**
     * @brief Rest position / center of stance position
     */
    ester_common::LegMap<Eigen::Vector3d> foot_zeros;

    /**
     * @brief Number of points required in the trajectory
     * 
     * @details If 1 then only the current position will be returned
     *          else, <num_pts> will be returned at even timespaces
     */
    unsigned int num_pts = 1;

    /**
     * @brief requested timestep (seconds) between points if num_pts > 1
     */
    double timestep = 0.01;

    /**
     * @brief Current linear velocity of the robot body
     * 
     * @details In the body frame
     */
    Eigen::Vector2d lin_vel;

    /**
     * @brief Requested linear velocity of the robot body
     * 
     * @details In the body frame
     */
    Eigen::Vector2d cmd_lin_vel;

    /**
     * @brief Maximum x and y stride length
     */
    Eigen::Vector2d max_stride_length;

    /**
     * @brief Z position of the leg when fully extended (should be negative)
     */
    double max_leg_len;

    /**
     * @brief Whether to also calculate spine positions for the trajectory
     */
    bool calculate_spine_positions = false;

    /**
     * @brief Pitch is determined proportionally to the expected distance of the
     *  feet from the zero position
     */
    double Kpitch;

    /**
     * @brief Yaw is determined proportionally to the angle of the feet to
     *        the body frame y axis
     */
    double Kyaw;
};

class TrajectoryPlanner {
public:
    /**
     * @brief Constructor
     * 
     * @param start_lift How much to lift the foot at the start of the swing
     * @param end_lift How much to lift the foot at the end of the swing
     */
    TrajectoryPlanner(double start_lift, double end_lift);

    /**
     * @brief Generate the trajectories
     */
    const std::shared_ptr<Trajectories> generate_trajectories(
        const TrajectoryParameters &params);

private:
    /**
     * @brief Calculates the swing foot position at time t
     * 
     * @details Don't call this for stance legs!
     * 
     * @param t Ranges from 0 to 1, with 0 being the start and 1 the end
     * @param start The starting position of the bezier curve
     * @param end The ending position of the bezier curve
     * 
     * @returns The position of the foot at time t of the swing
     */
    Eigen::Vector3d get_foot_pos_bezier(
        double t, const Eigen::Vector3d &start,
        const Eigen::Vector3d &end) const;

    /**
     * @brief Calculate a single bezier point in 1d
     * 
     * @param t Ranges from 0 to 1, how long along the curve we are
     * @param p0 First control point (used twice)
     * @param p1 Second control point (used thrice)
     * @param apply_z_offset If true, apply the z offset at the second and
     *                       third control points
     */
    double bezier(double t, double p0, double p1, bool apply_z_offset) const;

    /**
     * @brief Clearance to lift the foot at the start
     */
    double foot_swing_clearance_start_;

    /**
     * @brief Clearance to lift the foot at the end
     */
    double foot_swing_clearance_end_;
};

} // ns ester_mpc

#endif // __TRAJECTORY_PLANNER_HPP__