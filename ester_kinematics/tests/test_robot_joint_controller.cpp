#include <random>

#include <gtest/gtest.h>

#include <ester_common/ester_enums.hpp>
#include "test_utils.hpp"
#include "ester_kinematics/leg_dynamics.hpp"
#include "ester_kinematics/robot_joint_controller.hpp"

using namespace ester_common;
using namespace ester_kinematics;

namespace test_robot_joint_controller {

void fill_random_data(
    AllJointMap<double> &pos, LegMap<bool> &stance,
    LegMap<Eigen::Vector3d> &foot_tgt, LegMap<Eigen::Vector3d> &foot_grf)
{
    static urdf::ModelSharedPtr urdf;
    if (!urdf) {
        urdf = load_full_urdf();
    }
    LegJointMap<double> upper, lower;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-1., 1.);

    auto spine_jnts = gen_rand_spine_joint_map();
    for (const auto &jnt : ALL_SPINE_JOINTS) {
        pos[full_joint_id(jnt)] = spine_jnts[jnt];
        pos[full_joint_id(jnt)] = 0;
    }

    SpineKinematics spine;
    spine.set_spine_joint_positions(spine_jnts);

    for (const auto &leg : ALL_LEG_IDS) {
        LegDynamics dyn(leg);
        for (const auto &jnt : ALL_LEG_JOINTS) {
            get_jnt_limits(urdf, full_joint_id(leg, jnt), upper[jnt], lower[jnt]);
        }
        auto strt_pos = gen_rand_leg_joint_map(upper, lower);
        auto tgt_pos = gen_rand_leg_joint_map(upper, lower);
        foot_tgt[leg] = spine.point_to_base_frame(
            leg, dyn.solve_fk(tgt_pos).translation());
        // Sanity check kinematics is feasible
        stance[leg] = dist(gen) > 0;
        foot_grf[leg].x() = dist(gen) * 2;
        foot_grf[leg].y() = dist(gen) * 2;
        foot_grf[leg].z() = (dist(gen) + 1) * 8;
        for (const auto &jnt : ALL_LEG_JOINTS) {
            pos[full_joint_id(leg, jnt)] = strt_pos[jnt];
        }
    }
}
/*
TEST(TestRobotJointController, notEnoughJoints) {
    RobotJointController rjc;

    AllJointMap<double> pos;
    AllJointMap<double> cmd_pos;
    AllJointMap<double> cmd_vel;
    AllJointMap<double> cmd_eff;
    AllJointMap<bool> trq_ctrl;
    LegMap<bool> stance;
    LegMap<Eigen::Vector3d> foot_tgt;
    LegMap<Eigen::Vector3d> foot_grf;

    for (unsigned int i = 0; i < 12; i++) {
        auto jnt = ALL_JOINT_IDS[i];
        rjc.register_joint(jnt, &cmd_pos[jnt], &cmd_vel[jnt], &cmd_eff[jnt], &trq_ctrl[jnt]);
    }

    fill_random_data(pos, stance, foot_tgt, foot_grf);

    ASSERT_ANY_THROW(rjc.control_robot(pos, foot_tgt, foot_grf, stance, 0.02));
}

TEST(TestRobotJointController, accurateVelocities) {
    RobotJointController rjc;

    AllJointMap<double> pos;
    AllJointMap<double> cmd_pos;
    AllJointMap<double> cmd_vel;
    AllJointMap<double> cmd_eff;
    AllJointMap<bool> trq_ctrl;
    LegMap<bool> stance;
    LegMap<Eigen::Vector3d> foot_tgt;
    LegMap<Eigen::Vector3d> foot_grf;

    for (const auto &jnt : ALL_JOINT_IDS) {
        cmd_pos[jnt] = 0;
        cmd_vel[jnt] = 0;
        cmd_eff[jnt] = 0;
        trq_ctrl[jnt] = false;
        rjc.register_joint(jnt, &cmd_pos[jnt], &cmd_vel[jnt], &cmd_eff[jnt], &trq_ctrl[jnt]);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(1e-3, 1e-1);
    const double NUM_LOOPS = 100;
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        double dt = dist(gen);
        fill_random_data(pos, stance, foot_tgt, foot_grf);
        rjc.control_robot(pos, foot_tgt, foot_grf, stance, dt);
        for (const auto &leg : ALL_LEG_IDS) {
            if (!stance[leg]) {
                for (const auto &jnt : ALL_LEG_JOINTS) {
                    auto fjnt = full_joint_id(leg, jnt);
                    ASSERT_NEAR(pos[fjnt] + cmd_vel[fjnt] * dt, cmd_pos[fjnt], NM_ACCURATE);
                }
            }
        }
        for (const auto &jnt : ALL_SPINE_JOINTS) {
            auto fjnt = full_joint_id(jnt);
            ASSERT_NEAR(cmd_pos[fjnt], pos[fjnt], NM_ACCURATE);
            ASSERT_NEAR(cmd_vel[fjnt], 0, NM_ACCURATE);
        }
    }
}

TEST(TestRobotJointController, respectStance) {
    RobotJointController rjc;

    AllJointMap<double> pos;
    AllJointMap<double> cmd_pos;
    AllJointMap<double> cmd_vel;
    AllJointMap<double> cmd_eff;
    AllJointMap<bool> trq_ctrl;
    LegMap<bool> stance;
    LegMap<Eigen::Vector3d> foot_tgt;
    LegMap<Eigen::Vector3d> foot_grf;

    for (const auto &jnt : ALL_JOINT_IDS) {
        cmd_pos[jnt] = 0;
        cmd_vel[jnt] = 0;
        cmd_eff[jnt] = 0;
        trq_ctrl[jnt] = false;
        rjc.register_joint(jnt, &cmd_pos[jnt], &cmd_vel[jnt], &cmd_eff[jnt], &trq_ctrl[jnt]);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(1e-3, 1e-1);
    const double NUM_LOOPS = 100;
    for (unsigned int i = 0; i < NUM_LOOPS; i++) {
        double dt = dist(gen);
        fill_random_data(pos, stance, foot_tgt, foot_grf);
        rjc.control_robot(pos, foot_tgt, foot_grf, stance, dt);
        for (const auto &leg : ALL_LEG_IDS) {
            for (const auto &jnt : ALL_LEG_JOINTS) {
                auto fjnt = full_joint_id(leg, jnt);
                ASSERT_EQ(stance[leg], trq_ctrl[fjnt]);
            }
        }
    }
}*/

} // ns test_robot_joint_controller