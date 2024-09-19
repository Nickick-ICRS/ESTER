#ifndef __TRAJECTORY_VISUALISER_HPP__
#define __TRAJECTORY_VISUALISER_HPP__

#include "ester_mpc/trajectories.hpp"

#include <ros/ros.h>

namespace ester_mpc {

/**
 * @brief Visualises trajectories on /trajectory_visualisation
 */
class TrajectoryVisualiser {
public:
    /**
     * @brief Constructor
     * 
     * @param nh The ROS NodeHandle
     */
    TrajectoryVisualiser(ros::NodeHandle &nh);

    /**
     * @brief Visualise the trajectories over ROS
     * 
     * @param trajectories The trajectories to visualise
     */
    void visualise(const std::shared_ptr<Trajectories> &trajectories);

private:
    /**
     * @brief The publisher
     */
    ros::Publisher pub_;
};

} // ns ester_mpc

#endif // __TRAJECTORY_VISUALISER_HPP__