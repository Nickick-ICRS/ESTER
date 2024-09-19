#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <Eigen/Geometry>

#include <ros/package.h>

#include <ester_kinematics/inertia_calculator.hpp>

#include <gtest/gtest.h>

namespace test_inertia_calculator
{

namespace dd = dart::dynamics;
using namespace ester_kinematics;

bool isApprox(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double precision) {
    if (A.cols() != B.cols() || A.rows() != B.rows()) {
        return false;
    }
    for (size_t i = 0; i < A.rows(); i++) {
        for (size_t j = 0; j < A.cols(); j++) {
            if (fabs(A(i, j) - B(i, j)) > precision) {
                return false;
            }
        }
    }
    return true;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &X) {
    Eigen::Matrix3d m;
    m <<     0, -X(2),  X(1),
          X(2),     0, -X(0),
         -X(1),  X(0),     0; 
    return m;
}

Eigen::Matrix3d get_total_inertia(
    const dd::BodyNodePtr &body, const dd::BodyNodePtr &rel_to,
    Eigen::Vector3d &com_offset)
{
    double mass = body->getMass();
    Eigen::Matrix3d inertia = body->getInertia().getMoment();
    Eigen::Matrix3d R = body->getTransform(rel_to).rotation();
    Eigen::Vector3d r = body->getCOM(rel_to) - com_offset;
    Eigen::Matrix3d skew_r = skew(r);

    Eigen::Matrix3d I = R * inertia * R.transpose() + mass * skew_r.transpose() * skew_r;

    for (size_t i = 0; i < body->getNumChildBodyNodes(); i++) {
        I += get_total_inertia(body->getChildBodyNode(i), rel_to, com_offset);
    }
    return I;
}

class TestInertiaCalculator : public EsterInertiaCalculator {
public:
    TestInertiaCalculator(const dd::SkeletonPtr &robot)
        :EsterInertiaCalculator(robot) {};
    
    Eigen::Isometry3d get_rbd_transform(size_t id, const Eigen::VectorXd &sp) {
        return calculate_rbd_transform(id, sp);
    };

    Eigen::Vector3d get_post_joint_offset(size_t id) {
        return spine_pos_post_joint_[id];
    }
};

TEST(TestInertiaCalculations, rbdTransformsMatchDart) {
    auto robot = ester_kinematics::load_ester_robot_model();
    std::vector<Eigen::Vector4d> spine_positions = {
        Eigen::Vector4d(0, 0, 0, 0),
        Eigen::Vector4d(0.25, 0, -0.25, 0),
        Eigen::Vector4d(0, 0.25, 0, -0.25),
        Eigen::Vector4d(-0.25, 0, 0.25, 0),
        Eigen::Vector4d(0, -0.25, 0, 0.25),
    };
    auto front_spine_pitch = robot->getJoint("front_spine_pitch_joint");
    auto front_spine_yaw = robot->getJoint("front_spine_yaw_joint");
    auto rear_spine_pitch = robot->getJoint("rear_spine_pitch_joint");
    auto rear_spine_yaw = robot->getJoint("rear_spine_yaw_joint");
    auto chassis = robot->getBodyNode("spine_center_link");
    auto front = robot->getBodyNode("front_spine_yaw_link");
    auto rear = robot->getBodyNode("rear_spine_yaw_link");
    ASSERT_TRUE(front_spine_pitch);
    ASSERT_TRUE(front_spine_yaw);
    ASSERT_TRUE(rear_spine_pitch);
    ASSERT_TRUE(rear_spine_yaw);
    ASSERT_TRUE(chassis);

    TestInertiaCalculator calc(robot);
    for (const auto &sp : spine_positions) {
        front_spine_pitch->setPosition(0, sp(0));
        front_spine_yaw->setPosition(0, sp(1));
        rear_spine_pitch->setPosition(0, sp(2));
        rear_spine_yaw->setPosition(0, sp(3));
        Eigen::Isometry3d front_tf = calc.get_rbd_transform(0, sp);
        Eigen::Isometry3d mid_tf = calc.get_rbd_transform(1, sp);
        Eigen::Isometry3d rear_tf = calc.get_rbd_transform(2, sp);

        Eigen::Isometry3d dart_front_tf = front->getTransform(chassis);
        Eigen::Isometry3d dart_mid_tf =
            Eigen::Isometry3d::Identity() * Eigen::Translation3d(chassis->getCOM(chassis));
        Eigen::Isometry3d dart_rear_tf = rear->getTransform(chassis);

        // Front tf and rear tf include the COM offset, which we don't want
        front_tf.translation() -= front_tf.linear() * calc.get_post_joint_offset(0);
        rear_tf.translation() -= rear_tf.linear() * calc.get_post_joint_offset(2);
        // Note that we *do* want the com offset for the mid tf

        EXPECT_TRUE(front_tf.isApprox(dart_front_tf))
            << front_tf.matrix() << std::endl << dart_front_tf.matrix() << "\n(" << sp.transpose() << ")";
        EXPECT_TRUE(mid_tf.isApprox(dart_mid_tf))
            << mid_tf.matrix() << std::endl << dart_mid_tf.matrix() << "\n(" << sp.transpose() << ")";
        EXPECT_TRUE(rear_tf.isApprox(dart_rear_tf))
            << rear_tf.matrix() << std::endl << dart_rear_tf.matrix() << "\n(" << sp.transpose() << ")";
    }
}

TEST(TestInertiaCalculations, comMatchesDART) {
    auto robot = ester_kinematics::load_ester_robot_model();
    std::vector<Eigen::Vector4d> spine_positions = {
        Eigen::Vector4d(0, 0, 0, 0),
        Eigen::Vector4d(0.25, 0, -0.25, 0),
        Eigen::Vector4d(0, 0.25, 0, -0.25),
        Eigen::Vector4d(-0.25, 0, 0.25, 0),
        Eigen::Vector4d(0, -0.25, 0, 0.25),
    };
    auto front_spine_pitch = robot->getJoint("front_spine_pitch_joint");
    auto front_spine_yaw = robot->getJoint("front_spine_yaw_joint");
    auto rear_spine_pitch = robot->getJoint("rear_spine_pitch_joint");
    auto rear_spine_yaw = robot->getJoint("rear_spine_yaw_joint");
    auto chassis = robot->getBodyNode("spine_center_link");
    ASSERT_TRUE(front_spine_pitch);
    ASSERT_TRUE(front_spine_yaw);
    ASSERT_TRUE(rear_spine_pitch);
    ASSERT_TRUE(rear_spine_yaw);
    ASSERT_TRUE(chassis);

    EsterInertiaCalculator calc(robot);
    for (const auto &sp : spine_positions) {
        front_spine_pitch->setPosition(0, sp(0));
        front_spine_yaw->setPosition(0, sp(1));
        rear_spine_pitch->setPosition(0, sp(2));
        rear_spine_yaw->setPosition(0, sp(3));
        Eigen::Vector3d dart_com = robot->getCOM(chassis);
        Eigen::Vector3d model_com = calc.calculate_inertia(sp).com;
        // Note that since we approximate the pitch rigid body as part of the yaw rigid body
        // the COM should vary slightly. Within 1mm seems to be accurate enough though
        EXPECT_TRUE(isApprox(dart_com, model_com, 1e-3))
            << "Error (dart - model): " << dart_com.transpose() - model_com.transpose()
            << " (spine: " << sp.transpose() << ")";
    }
}

TEST(TestInertiaCalculations, inertiaMatchesDART) {
    auto robot = ester_kinematics::load_ester_robot_model();
    std::vector<Eigen::Vector4d> spine_positions = {
        Eigen::Vector4d(0, 0, 0, 0),
        Eigen::Vector4d(0.25, 0, -0.25, 0),
        Eigen::Vector4d(0, 0.25, 0, -0.25),
        Eigen::Vector4d(-0.25, 0, 0.25, 0),
        Eigen::Vector4d(0, -0.25, 0, 0.25),
    };
    auto front_spine_pitch = robot->getJoint("front_spine_pitch_joint");
    auto front_spine_yaw = robot->getJoint("front_spine_yaw_joint");
    auto rear_spine_pitch = robot->getJoint("rear_spine_pitch_joint");
    auto rear_spine_yaw = robot->getJoint("rear_spine_yaw_joint");
    auto chassis = robot->getBodyNode("spine_center_link");
    ASSERT_TRUE(front_spine_pitch);
    ASSERT_TRUE(front_spine_yaw);
    ASSERT_TRUE(rear_spine_pitch);
    ASSERT_TRUE(rear_spine_yaw);
    ASSERT_TRUE(chassis);
    EsterInertiaCalculator calc(robot);
    for (const auto &sp : spine_positions) {
        front_spine_pitch->setPosition(0, sp(0));
        front_spine_yaw->setPosition(0, sp(1));
        rear_spine_pitch->setPosition(0, sp(2));
        rear_spine_yaw->setPosition(0, sp(3));
        Eigen::Vector3d com = robot->getCOM(chassis);
        Eigen::Matrix3d dart_inertia = get_total_inertia(chassis, chassis, com);
        Eigen::Matrix3d model_inertia = calc.calculate_inertia(sp).I;
        // Again, approximations to speed up high frequency calculations reduce accuracy here
        EXPECT_TRUE(isApprox(dart_inertia, model_inertia, 1e-3))
            << "Error (dart - model):\n" << dart_inertia - model_inertia
            << "\n(spine: " << sp.transpose() << ")";
    }
}

} // namespace test_inertia_calculator