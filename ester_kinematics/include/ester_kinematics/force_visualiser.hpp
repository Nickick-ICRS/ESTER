#ifndef __FORCE_VISUALISER_HPP__
#define __FORCE_VISUALISER_HPP__

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <ester_common/ester_enums.hpp>

namespace ester_kinematics {

class ForceVisualiser {
public:
    /**
     * @brief Constructor
     *
     * @param nh ROS NodeHandle
     */
    ForceVisualiser(ros::NodeHandle &nh);

    /**
     * @brief Publish an array of foot forces to RVIZ
     *
     * @param pos Position of each foot in the spine_center_link frame
     * @param grf The GRF in the spine_center_link frame
     */
    void publish_foot_forces(
        const ester_common::LegMap<Eigen::Vector3d> &pos,
        const ester_common::LegMap<Eigen::Vector3d> &grf);
private:
    /**
     * @brief ROS NodeHandle
     */
    ros::NodeHandle nh_;

    /**
     * @brief Publisher for marker arrays
     */
    ros::Publisher marker_pub_;
};

} // ns ester_kinematics

#endif // __FORCE_VISUALISER_HPP__
