#include <random>
#include <ester_common/ester_enums.hpp>

#include <ros/package.h>
#include <urdf/model.h>

#include <dart/dart.hpp>
#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>


#define BIG_TEST_NUMBER 1e9

#define MM_ACCURATE 1e-3
#define NM_ACCURATE 1e-6

#define DEG_ACCURATE 0.0174533
#define TENTH_DEG_ACCURATE (DEG_ACCURATE/10.0)

inline ester_common::SpineJointMap<double> gen_rand_spine_joint_map() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1., 1.);

    ester_common::SpineJointMap<double> jm;
    jm[ester_common::SpineJointId::FRONT_PITCH] = 0;// dist(gen) * 0.3;
    jm[ester_common::SpineJointId::FRONT_YAW] = 0;// dist(gen) * 0.3;
    jm[ester_common::SpineJointId::REAR_PITCH] = 0;// dist(gen) * 0.3;
    jm[ester_common::SpineJointId::REAR_YAW] = 0;// dist(gen) * 0.3;
    return jm;
}

inline ester_common::LegJointMap<double> gen_rand_leg_joint_map(
    const ester_common::LegJointMap<double> &upper_limits,
    const ester_common::LegJointMap<double> &lower_limits)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0., 1.);

    ester_common::LegJointMap<double> jm;
    for (const auto &jnt : ester_common::ALL_LEG_JOINTS) {
        jm[jnt] = lower_limits.at(jnt) + dist(gen)
                * (upper_limits.at(jnt) - lower_limits.at(jnt));
    }
    return jm;
}

inline std::shared_ptr<urdf::Model> load_full_urdf() {
    std::string ester_description =
        ros::package::getPath("ester_description");
    std::string urdf_path = ester_description + "/urdf/ester.urdf";

    auto model = std::make_shared<urdf::Model>();
    if(!model->initFile(urdf_path)) {
        throw;
    }
    return model;
}

inline void get_jnt_limits(
    const std::shared_ptr<urdf::Model> &model,
    const ester_common::AllJointId &jnt,
    double &upper, double &lower)
{
    upper = model->getJoint(ester_common::str(jnt))->limits->upper;
    lower = model->getJoint(ester_common::str(jnt))->limits->lower;
    // URDF actually allows knees to go both ways but kinematics limits them
    if (jnt == ester_common::AllJointId::FL_KNEE
     || jnt == ester_common::AllJointId::FR_KNEE) {
        upper = -0.05;
    }
    else if(jnt == ester_common::AllJointId::RL_KNEE
         || jnt == ester_common::AllJointId::RR_KNEE) {
        lower = 0.05;
    }
    // Slightly shrink the limits
    upper = upper > 0 ? upper * 0.9 : upper * 1.1;
    lower = lower < 0 ? lower * 0.9 : lower * 1.1;
}

inline dart::dynamics::SkeletonPtr load_robot_dart() {
    std::string ester_description = ros::package::getPath(
        "ester_description");
    std::string urdf_path = ester_description + "/urdf/ester.urdf";
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
    dart::dynamics::SkeletonPtr skel = loader.parseSkeleton(
        dart::common::Uri(urdf_path), nullptr,
        dart::utils::DartLoader::Flags::FIXED_BASE_LINK);
    skel->setGravity(Eigen::Vector3d(0, 0, 0));

    /*
    // Uncomment to print DART joint numbers
    for (unsigned int i = 0; i < skel->getNumJoints(); i++) {
        ROS_ERROR_STREAM(i << ": " << skel->getJoint(i)->getName());
    }
    // Uncomment to print DART body node numbers
    for (unsigned int i = 0; i < skel->getNumBodyNodes(); i++) {
        ROS_ERROR_STREAM(i << ": " << skel->getBodyNode(i)->getName());
    }
    */

    // Set foot masses to dwarf other masses
    skel->getBodyNode(8)->setMass(BIG_TEST_NUMBER);
    skel->getBodyNode(13)->setMass(BIG_TEST_NUMBER);
    skel->getBodyNode(21)->setMass(BIG_TEST_NUMBER);
    skel->getBodyNode(26)->setMass(BIG_TEST_NUMBER);
    return skel;
}

inline std::array<unsigned int, 4> get_leg_joint_and_foot_ids(
    ester_common::LegId leg)
{
    // 3 leg joint ids + foot body node id
    std::array<unsigned int, 4> ids;
    switch(leg) {
    // hip_fe, upper_leg, lower_leg, foot_link
    case ester_common::LegId::FL:
        ids = {5, 6, 7, 8};
        break;
    case ester_common::LegId::FR:
        ids = {10, 11, 12, 13};
        break;
    case ester_common::LegId::RL:
        ids = {18, 19, 20, 21};
        break;
    case ester_common::LegId::RR:
        ids = {23, 24, 25, 26};
        break;
    }
    return ids;
}