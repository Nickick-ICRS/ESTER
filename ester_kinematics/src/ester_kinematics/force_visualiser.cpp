#include "ester_kinematics/force_visualiser.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>

using namespace ester_common;
using namespace ester_kinematics;

ForceVisualiser::ForceVisualiser(ros::NodeHandle &nh) :nh_(nh) {
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/mpc_visualisation", 1);
}

void ForceVisualiser::publish_foot_forces(
    const LegMap<Eigen::Vector3d> &pos,
    const LegMap<Eigen::Vector3d> &grf)
{
    visualization_msgs::MarkerArray msg;

    auto t = ros::Time::now();

    unsigned int idx = 0;
    for (const auto &pair : pos) {
        const auto &id = pair.first;
        visualization_msgs::Marker m;
        m.header.stamp = t;
        m.header.frame_id = "spine_center_link";
        m.ns = "grf";
        m.type = m.ARROW;
        m.action = m.ADD;
        m.id = idx++;
        m.pose.orientation.w = 1;
        geometry_msgs::Point start;
        geometry_msgs::Point end;
        tf::pointEigenToMsg(pair.second, start);
        tf::pointEigenToMsg(pair.second + grf.at(id) / 100, end);
        m.points.push_back(start);
        m.points.push_back(end);
        m.scale.x = 0.025;
        m.scale.y = 0.05;
        m.scale.z = 0;

        double mag = grf.at(id).norm();
        m.color.r = grf.at(id).x() / mag;
        m.color.g = grf.at(id).y() / mag;
        m.color.b = grf.at(id).z() / mag;
        m.color.a = 0.5;

        msg.markers.push_back(m);
    }

    marker_pub_.publish(msg);
}
