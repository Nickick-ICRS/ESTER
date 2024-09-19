#include <random>
#include <gtest/gtest.h>

#include <ros/package.h>
#include <dart/dart.hpp>
#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <ester_common/ester_enums.hpp>
#include "ester_kinematics/spine_kinematics.hpp"
#include "test_utils.hpp"


using namespace ester_kinematics;
using namespace ester_common;

namespace test_spine_kinematics {

TEST(TestSpineKinematics, kinematics) {
    SpineKinematics spine;

    std::string ester_description = ros::package::getPath(
        "ester_description");
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
    auto model = loader.parseSkeleton(
        dart::common::Uri(ester_description + "/urdf/ester.urdf"), nullptr,
        dart::utils::DartLoader::Flags::FIXED_BASE_LINK);
    model->setGravity(Eigen::Vector3d::Zero());

    SpineJointMap<dart::dynamics::JointPtr> dart_jnts;
    LegMap<dart::dynamics::BodyNodePtr> dart_lnks;
    dart_jnts[SpineJointId::FRONT_PITCH] =
        model->getJoint("front_spine_pitch_joint");
    dart_jnts[SpineJointId::FRONT_YAW] =
        model->getJoint("front_spine_yaw_joint");
    dart_jnts[SpineJointId::REAR_PITCH] =
        model->getJoint("rear_spine_pitch_joint");
    dart_jnts[SpineJointId::REAR_YAW] =
        model->getJoint("rear_spine_yaw_joint");
    dart_lnks[LegId::FL] = model->getBodyNode("front_left_hip_aa_link");
    dart_lnks[LegId::FR] = model->getBodyNode("front_right_hip_aa_link");
    dart_lnks[LegId::RL] = model->getBodyNode("rear_left_hip_aa_link");
    dart_lnks[LegId::RR] = model->getBodyNode("rear_right_hip_aa_link");

    unsigned int NUM_LOOPS = 100;
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        auto pos = gen_rand_spine_joint_map();
        spine.set_spine_joint_positions(pos);

        for (const auto &jnt : ALL_SPINE_JOINTS) {
            dart_jnts[jnt]->setPosition(0, pos[jnt]);
            dart_jnts[jnt]->setVelocity(0, 0);
            dart_jnts[jnt]->setForce(0, 0);
        }
        model->computeForwardKinematics();
        model->computeForwardDynamics();

        for (const auto & id : ALL_LEG_IDS) {
            Eigen::Isometry3d tf_us = spine.get_shoulder_transform(id);
            Eigen::Isometry3d tf_drt = dart_lnks[id]->getTransform().inverse();

            // Some arbitrary point that can be translated and rotated
            Eigen::Vector3d pt(1, 1, 1);

            ASSERT_LT((tf_us.translation() - tf_drt.translation()).norm(), NM_ACCURATE);
            ASSERT_LT(((tf_us.linear() * tf_drt.linear().transpose()) * pt - pt).norm(), NM_ACCURATE);
            Eigen::Vector3d v_us = tf_us.linear() * pt;
            Eigen::Vector3d v_drt = tf_drt.linear() * pt;
            ASSERT_LT((v_us - v_drt).norm(), NM_ACCURATE);

            Eigen::Vector3d p_drt = tf_drt * pt;
            ASSERT_LT((spine.point_to_shoulder_frame(id, pt) - p_drt).norm(), NM_ACCURATE);
            ASSERT_LT((spine.vector_to_shoulder_frame(id, pt) - v_drt).norm(), NM_ACCURATE);

            // base -> shoulder -> base should be identity
            ASSERT_LT((spine.point_to_base_frame(
                id, spine.point_to_shoulder_frame(id, pt)) - pt).norm(), NM_ACCURATE);
            // base -> shoulder -> base should be identity
            ASSERT_LT((spine.vector_to_base_frame(
                id, spine.vector_to_shoulder_frame(id, pt)) - pt).norm(), NM_ACCURATE);
        }
    }
}

} // ns test_spine_kinematics