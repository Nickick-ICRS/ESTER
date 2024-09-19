#include <random>

#include <gtest/gtest.h>

#include <ester_common/ester_enums.hpp>
#include "ester_kinematics/leg_dynamics.hpp"
#include "test_utils.hpp"

using namespace ester_common;
using namespace ester_kinematics;

namespace test_dynamics {

auto test_leg_kinematics = [](const LegId &id) {
    LegDynamics dyn(id);

    std::shared_ptr<urdf::Model> urdf = load_full_urdf();
    LegJointMap<double> upper, lower;
    for (const auto &jnt : ALL_LEG_JOINTS) {
        get_jnt_limits(
            urdf, full_joint_id(id, jnt), upper[jnt], lower[jnt]);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-0.1, 0.1);

    unsigned int NUM_LOOPS = 100;
    auto start = std::chrono::steady_clock::now();
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        auto pos = gen_rand_leg_joint_map(upper, lower);
        // In the event of multiple valid solutions we want the one closest to
        // the current pos
        auto current_pos = pos;
        Eigen::Isometry3d fk_res = dyn.solve_fk(pos);
        auto ik_res = dyn.solve_ik(fk_res.translation(), current_pos);
        // IK is solved from fk results so
        ASSERT_TRUE(ik_res.reachable);
        for (const auto &jnt : ALL_LEG_JOINTS) {
            ASSERT_NEAR(pos[jnt],
                        ik_res.pos[jnt],
                        NM_ACCURATE*10);
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> dur = end - start;
    std::cerr << "Avg. " << dur.count() / (double)NUM_LOOPS
              << "s per IK iteration." << std::endl;
};

auto test_leg_dynamics = [](const LegId &id) {
    LegDynamics dyn(id);

    std::shared_ptr<urdf::Model> urdf = load_full_urdf();
    LegJointMap<double> upper, lower, vel_upper, vel_lower,
                        eff_upper, eff_lower;
    for (const auto &jnt : ALL_LEG_JOINTS) {
        // ID doesn't work with non-zero velocities (due to foot mass approx?)
        get_jnt_limits(urdf, full_joint_id(id, jnt), upper[jnt], lower[jnt]);
        vel_upper[jnt] = 0.0;
        vel_lower[jnt] = -0.0;
        eff_upper[jnt] = 2.0;
        eff_lower[jnt] = -2.0;
    }
    if (id == LegId::FL || id == LegId::FR) {
        upper[LegJointId::KNEE] = -M_PI / 16; // Singularities near knee 0
        lower[LegJointId::KNEE] = -M_PI / 4;
    }
    else {
        upper[LegJointId::KNEE] = M_PI / 4;
        lower[LegJointId::KNEE] = M_PI / 16; // Singularities near knee 0
    }

    unsigned int NUM_LOOPS = 100;
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        auto pos = gen_rand_leg_joint_map(upper, lower);
        auto vel = gen_rand_leg_joint_map(vel_upper, vel_lower);
        auto eff = gen_rand_leg_joint_map(eff_upper, eff_lower);
        Eigen::Vector3d foot_vel;
        auto fd_res = dyn.solve_fd(pos, vel, eff, &foot_vel);
        auto id_res = dyn.solve_id(pos, vel, foot_vel, fd_res);
        auto fd_res2 = dyn.solve_fd(pos, id_res.vel, id_res.eff);
        ASSERT_NEAR((fd_res - fd_res2).norm(), 0, MM_ACCURATE) << fd_res.transpose() << " != " << fd_res2.transpose();
        for (const auto &jnt : ALL_LEG_JOINTS) {
            ASSERT_NEAR(id_res.vel[jnt], vel[jnt], MM_ACCURATE);
            ASSERT_NEAR(id_res.eff[jnt], eff[jnt], MM_ACCURATE);
        }
    }
};

TEST(TestLegDynamics, forwardInvKinematicsFL) {
    test_leg_kinematics(LegId::FL);
}

TEST(TestLegDynamics, forwardInvKinematicsFR) {
    test_leg_kinematics(LegId::FR);
}

TEST(TestLegDynamics, forwardInvKinematicsRL) {
    test_leg_kinematics(LegId::RL);
}

TEST(TestLegDynamics, forwardInvKinematicsRR) {
    test_leg_kinematics(LegId::RR);
}

TEST(TestLegDynamics, forwardInvDynamicsFL) {
    test_leg_dynamics(LegId::FL);
}

TEST(TestLegDynamics, forwardInvDynamicsFR) {
    test_leg_dynamics(LegId::FR);
}

TEST(TestLegDynamics, forwardInvDynamicsRL) {
    test_leg_dynamics(LegId::RL);
}

TEST(TestLegDynamics, forwardInvDynamicsRR) {
    test_leg_dynamics(LegId::RR);
}

} // ns test_dynamics