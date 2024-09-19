#include "ester_mpc/trajectory_visualiser.hpp"

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

using namespace ester_common;
using namespace ester_mpc;

#define MARKER_WIDTH 0.01
#define MARKER_R 0
#define MARKER_G 0
#define MARKER_B 1
#define MARKER_A 0.5
#define MARKER_DURATION 5.0

TrajectoryVisualiser::TrajectoryVisualiser(ros::NodeHandle &nh) {
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/trajectory_visualisation", 1);
}

void TrajectoryVisualiser::visualise(
    const std::shared_ptr<Trajectories> &trajectories)
{
    visualization_msgs::MarkerArray msg;
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "spine_center_link";
    unsigned int i = 0;
    for (const auto &traj_pair : trajectories->pos) {
        visualization_msgs::Marker m;
        m.header = h;
        m.ns = "foot_trajectories";
        m.id = i++;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.scale.x = MARKER_WIDTH;
        m.lifetime = ros::Duration(MARKER_DURATION);
        m.frame_locked = false;

        for (const auto &pos : traj_pair.second) {
            geometry_msgs::Point p;
            tf::pointEigenToMsg(pos, p);
            m.points.emplace_back(p);
            std_msgs::ColorRGBA c;
            c.r = MARKER_R;
            c.g = MARKER_G;
            c.b = MARKER_B;
            c.a = MARKER_A;
            m.colors.emplace_back(c);
        }
        msg.markers.emplace_back(m);
    }
    pub_.publish(msg);
}