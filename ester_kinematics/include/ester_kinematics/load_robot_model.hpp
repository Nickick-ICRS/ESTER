#ifndef __LOAD_ROBOT_MODEL_HPP__
#define __LOAD_ROBOT_MODEL_HPP__

#include <ros/package.h>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

namespace ester_kinematics
{

inline dart::dynamics::SkeletonPtr load_ester_robot_model()
{
    std::string ester_description = ros::package::getPath(
        "ester_description");
    std::string urdf_path = ester_description + "/urdf/ester.urdf";
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
    dart::dynamics::SkeletonPtr robot = loader.parseSkeleton(dart::common::Uri(urdf_path));

    // First node is wack because dart gives default inertia
    robot->getBodyNode(0)->setMass(1e-9);
    robot->getBodyNode(0)->setMomentOfInertia(1e-6, 1e-6, 1e-6, 0, 0, 0);

    return robot;
}

} // namespace ester_kinematics

#endif // __LOAD_ROBOT_MODEL_HPP__