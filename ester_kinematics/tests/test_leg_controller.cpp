#include <random>

#include <gtest/gtest.h>

#include <ester_common/ester_enums.hpp>
#include "ester_kinematics/leg_controller.hpp"
#include "test_utils.hpp"

using namespace ester_common;
using namespace ester_kinematics;

namespace test_leg_controller {

auto test_leg_controller = [](const LegId &id) {
    LegController ctrl(id);

    auto urdf = load_full_urdf();
    LegJointMap<double> upper, lower, zero_vel;
    for (const auto &jnt : ALL_LEG_JOINTS) {
        get_jnt_limits(urdf, full_joint_id(id, jnt), upper[jnt], lower[jnt]);
        zero_vel[jnt] = 0;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0., 1.);

    unsigned int NUM_LOOPS = 100;
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        auto cpos = gen_rand_leg_joint_map(upper, lower);
        auto tpos = gen_rand_leg_joint_map(upper, lower);

        // A random, reasonable foot GRF
        Eigen::Vector3d req_grf;
        req_grf.x() = dist(gen) * 4. - 2.;
        req_grf.y() = dist(gen) * 4. - 2.;
        req_grf.z() = 9.8 + dist(gen) * 4 - 2;

        // Some random transform
        Eigen::Isometry3d tf;
        tf = Eigen::AngleAxisd(dist(gen) - 0.5, Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(dist(gen) - 0.5, Eigen::Vector3d::UnitX())
           * Eigen::Translation3d(
                dist(gen) - 0.5, dist(gen) - 0.5, dist(gen) - 0.5);

        bool stance = dist(gen) > 0.5;
        double dt = dist(gen) * 0.1 + 0.01;

        // Get target foot position from kinematics
        Eigen::Vector3d tgt =
            (tf.inverse() * ctrl.get_solver()->solve_fk(tpos)).translation();

        req_grf = tf.inverse().linear() * req_grf;

        auto res = ctrl.control_leg(tgt, req_grf, cpos, tf, dt, stance);
        ASSERT_TRUE(res.valid_data);

        auto current_pos_err = [&cpos](auto &jp) {
            double err = 0;
            for (const auto &jnt : ALL_LEG_JOINTS) {
                err += std::abs(jp[jnt] - cpos[jnt]);
            }
            return err;
        };
        if (current_pos_err(res.pos) < current_pos_err(tpos)) {
            // Just assert target position is the same
            Eigen::Vector3d tgt_pos =
                ctrl.get_solver()->solve_fk(tpos).translation();
            Eigen::Vector3d ctrl_pos =
                ctrl.get_solver()->solve_fk(res.pos).translation();
            ASSERT_NEAR(ctrl_pos.x(), tgt_pos.x(), MM_ACCURATE);
            ASSERT_NEAR(ctrl_pos.y(), tgt_pos.y(), MM_ACCURATE);
            ASSERT_NEAR(ctrl_pos.z(), tgt_pos.z(), MM_ACCURATE);
        }
        else {
            ASSERT_NEAR(res.pos[LegJointId::HIP_ROLL], tpos[LegJointId::HIP_ROLL], MM_ACCURATE);
            ASSERT_NEAR(res.pos[LegJointId::HIP_PITCH], tpos[LegJointId::HIP_PITCH], MM_ACCURATE);
            ASSERT_NEAR(res.pos[LegJointId::KNEE], tpos[LegJointId::KNEE], MM_ACCURATE);
        }

        if (stance) {
            ASSERT_TRUE(res.use_torque_control);

            Eigen::Vector3d res_grf = tf.inverse().linear() * 
                ctrl.get_solver()->solve_fd(
                    res.pos, zero_vel, res.eff);

            ASSERT_TRUE((res_grf - req_grf).norm() < MM_ACCURATE)
                << res_grf.transpose() << " != " << req_grf.transpose();
            // Prevent compiler doing weird optimisations
            (void)res_grf;
        }

        else {
            ASSERT_FALSE(res.use_torque_control);
        }

        for (const auto &jnt : ALL_LEG_JOINTS) {
            ASSERT_NEAR(cpos[jnt] + res.vel[jnt] * dt, res.pos[jnt], NM_ACCURATE);
        }
    }
};

TEST(TestLegController, FL) {
    test_leg_controller(LegId::FL);
}

TEST(TestLegController, FR) {
    test_leg_controller(LegId::FR);
}

TEST(TestLegController, RL) {
    test_leg_controller(LegId::RL);
}

TEST(TestLegController, RR) {
    test_leg_controller(LegId::RR);
}

} // ns test_leg_controller