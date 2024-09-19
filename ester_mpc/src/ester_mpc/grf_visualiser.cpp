#include "ester_mpc/grf_visualiser.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

using namespace ester_mpc;
using namespace ester_common;

#define SCALE 0.025

GRFVisualiser::GRFVisualiser() {
    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/mpc_visualisation", 1);
}

void GRFVisualiser::visualise(
    const LegMap<Eigen::Vector3d> &foot_pos,
    const LegMap<Eigen::Vector3d> &foot_grfs)
{
    visualization_msgs::MarkerArray arr;
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "spine_center_link";
    unsigned int i = 0;
    for (const auto &leg : ALL_LEG_IDS) {
        visualization_msgs::Marker m;
        m.header = h;
        m.ns = "foot_grfs";
        m.id = i++;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.lifetime = ros::Duration(0);
    
        geometry_msgs::Point strt, end;
        tf::pointEigenToMsg(foot_pos.at(leg), strt);
        tf::pointEigenToMsg(foot_pos.at(leg) + SCALE * foot_grfs.at(leg), end);
        m.points.push_back(strt);
        m.points.push_back(end);
        m.scale.x = 0.02;
        m.scale.y = m.scale.x * 2;
        m.scale.z = 0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 1;
        m.color.a = 0.4;
        arr.markers.push_back(m);
    }
    vis_pub_.publish(arr);
}