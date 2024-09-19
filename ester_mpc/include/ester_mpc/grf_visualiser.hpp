#ifndef __GRF_VISUALISER_HPP__
#define __GRF_VISUALISER_HPP__

#include <ester_common/ester_enums.hpp>

#include <Eigen/Geometry>

#include <ros/ros.h>

namespace ester_mpc
{

class GRFVisualiser {
public:
    /**
     * @brief Constructor
     */
    GRFVisualiser();

    /**
     * @brief Visualise the foot GRFs
     * 
     * @param foot_pos The foot positions
     * @param foot_grfs The foot GRFs
     */
    void visualise(
        const ester_common::LegMap<Eigen::Vector3d> &foot_pos,
        const ester_common::LegMap<Eigen::Vector3d> &foot_grfs);

private:
    /**
     * @brief ROS NodeHandle
     */
    ros::NodeHandle nh_;

    /**
     * @brief ROS Publisher
     */
    ros::Publisher vis_pub_;
};

} // namespace ester_mpc


#endif // __GRF_VISUALISER_HPP__