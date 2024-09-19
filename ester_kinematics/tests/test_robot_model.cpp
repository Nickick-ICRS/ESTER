#include <random>
#include <array>

#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include <ester_common/ester_enums.hpp>
#include "test_utils.hpp"
#include "ester_kinematics/robot_model.hpp"


using namespace ester_common;
using namespace ester_kinematics;

namespace test_robot_model {

Eigen::Vector3d full_model_calc_fp(
    const dart::dynamics::SkeletonPtr &skel, LegId leg,
    const LegJointMap<double> &pos, const SpineJointMap<double> &spine_pos)
{
    auto ids = get_leg_joint_and_foot_ids(leg);

    auto set_joint = [](auto dart_joint, double pos) {
        dart_joint->setPosition(0, pos);
        dart_joint->setVelocity(0, 0);
        dart_joint->setForce(0, 0);
    };

    set_joint(skel->getJoint(2), spine_pos.at(SpineJointId::FRONT_PITCH));
    set_joint(skel->getJoint(3), spine_pos.at(SpineJointId::FRONT_YAW));
    set_joint(skel->getJoint(14), spine_pos.at(SpineJointId::REAR_PITCH));
    set_joint(skel->getJoint(15), spine_pos.at(SpineJointId::REAR_YAW));
    set_joint(skel->getJoint(ids.at(0)), pos.at(LegJointId::HIP_ROLL));
    set_joint(skel->getJoint(ids.at(1)), pos.at(LegJointId::HIP_PITCH));
    set_joint(skel->getJoint(ids.at(2)), pos.at(LegJointId::KNEE));
    
    skel->computeForwardKinematics();
    return skel->getBodyNode(ids.at(3))->getTransform(skel->getBodyNode("spine_center_link")).translation();
}

Eigen::Vector3d full_model_calc_grf(
    const dart::dynamics::SkeletonPtr &skel, LegId leg,
    const LegJointMap<double> &pos, const LegJointMap<double> &vel,
    const LegJointMap<double> &eff, const SpineJointMap<double> &spine_pos)
{
    auto ids = get_leg_joint_and_foot_ids(leg);

    auto foot = skel->getBodyNode(ids.at(3));

    auto set_joint = [](auto dart_joint, double pos, double vel, double eff)
    {
        dart_joint->setPosition(0, pos);
        dart_joint->setVelocity(0, vel);
        dart_joint->setForce(0, eff);
    };

    auto set_no_move = [](auto dart_joint) {
        dart_joint->setSpringStiffness(0, 1e20);
        dart_joint->setDampingCoefficient(0, 1e20);
    };

    // Reset joints
    for (unsigned int i = 0; i < skel->getNumJoints(); i++) {
        set_joint(skel->getJoint(i), 0, 0, 0);
    }

    set_joint(
        skel->getJoint(2), spine_pos.at(SpineJointId::FRONT_PITCH), 0, 0);
    set_joint(
        skel->getJoint(3), spine_pos.at(SpineJointId::FRONT_YAW), 0, 0);
    set_joint(
        skel->getJoint(14), spine_pos.at(SpineJointId::REAR_PITCH), 0, 0);
    set_joint(
        skel->getJoint(15), spine_pos.at(SpineJointId::REAR_YAW), 0, 0);
    // For now we just simulate a perfectly rigid spine
    set_no_move(skel->getJoint(2));
    set_no_move(skel->getJoint(3));
    set_no_move(skel->getJoint(14));
    set_no_move(skel->getJoint(15));

    set_joint(
        skel->getJoint(ids.at(0)), pos.at(LegJointId::HIP_ROLL),
        vel.at(LegJointId::HIP_ROLL), eff.at(LegJointId::HIP_ROLL));
    set_joint(
        skel->getJoint(ids.at(1)), pos.at(LegJointId::HIP_PITCH),
        vel.at(LegJointId::HIP_PITCH), eff.at(LegJointId::HIP_PITCH));
    set_joint(
        skel->getJoint(ids.at(2)), pos.at(LegJointId::KNEE),
        vel.at(LegJointId::KNEE), eff.at(LegJointId::KNEE));

    skel->computeForwardKinematics();
    skel->computeForwardDynamics();

    Eigen::Vector3d acc = foot->getCOMLinearAcceleration();

    return -1 * acc * BIG_TEST_NUMBER;
}

TEST(TestRobotModel, forwardKinematics)
{
    RobotModel robot_model;
    auto skel = load_robot_dart();

    auto urdf = load_full_urdf();
    LegJointMap<double> upper, lower;

    for (const auto &leg : ALL_LEG_IDS) {
        for (const auto &jnt : ALL_LEG_JOINTS) {
            get_jnt_limits(urdf, full_joint_id(leg, jnt), upper[jnt], lower[jnt]);
        }
        for (unsigned int i = 0; i < 100; i++) {
            auto spine_pos = gen_rand_spine_joint_map();
            auto pos = gen_rand_leg_joint_map(upper, lower);
            Eigen::Vector3d rmp = robot_model.calculate_fp(leg, pos, spine_pos);
            Eigen::Vector3d fmp = full_model_calc_fp(skel, leg, pos, spine_pos);
            ASSERT_TRUE((rmp - fmp).norm() < NM_ACCURATE) << leg << " failed" << std::endl;
        }
    }
}

TEST(TestRobotModel, forwardDynamics)
{
    RobotModel robot_model;
    auto skel = load_robot_dart();
    auto urdf = load_full_urdf();

    LegJointMap<double> upper, lower, v_upper, v_lower, e_upper, e_lower;

    for (const auto &leg : ALL_LEG_IDS) {
        for (const auto &jnt : ALL_LEG_JOINTS) {
            get_jnt_limits(urdf, full_joint_id(leg, jnt), upper[jnt], lower[jnt]);
            v_upper[jnt] = 0;
            v_lower[jnt] = 0;
            e_upper[jnt] = 2.0;
            e_lower[jnt] = -2.0;
        }
        if (leg == LegId::FL || leg == LegId::FR) {
            upper[LegJointId::KNEE] = -M_PI / 16; // Singularities near knee 0
        }
        else {
            lower[LegJointId::KNEE] = M_PI / 16; // Singularities near knee 0
        }
        for (unsigned int i = 0; i < 100; i++) {
            auto spine_pos = gen_rand_spine_joint_map();
            auto pos = gen_rand_leg_joint_map(upper, lower);
            auto vel = gen_rand_leg_joint_map(v_upper, v_lower);
            auto eff = gen_rand_leg_joint_map(e_upper, e_lower);

            Eigen::Vector3d rmf = robot_model.calculate_grf(
                leg, pos, vel, eff, spine_pos);
            Eigen::Vector3d fmf = full_model_calc_grf(
                skel, leg, pos, vel, eff, spine_pos);
            if (fmf.norm() > 500) {
                // Singularity
                i--;
                continue;
            }
            // 10 % tolerance
            double err_tol = 0.01 * fmf.norm();
            ASSERT_TRUE((rmf - fmf).norm() <= err_tol) << leg << i << " failed" << std::endl;
        }
    }
}

} // ns test_robot_model
