#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <ester_common/ester_enums.hpp>
#include <ester_common/fixed_cmd_interface.hpp>

#include <ester_kinematics/robot_model.hpp>

#include <ester_mpc/world_nodes/dart_mpc_world_osg.hpp>
#include <ester_mpc/mpc_manager.hpp>

namespace test_srbd {

using namespace ester_kinematics;
using namespace ester_common;
using namespace ester_mpc;

#define NM_ACCURATE 1e-6
#define MM_ACCURATE 1e-3
#define CM_ACCURATE 1e-2
#define ONE_PERCENT 1e-2

auto test_converge = [](
    auto &cmd, RBDState &tgt_state, RBDState initial_state, size_t num_steps,
    double X_TOL, double R_TOL, double V_TOL, double W_TOL)
{
    double start_lift = 0.03;
    double end_lift = 0.05;

    const RBDState initial_tgt = tgt_state;

    dart::simulation::WorldPtr world(new dart::simulation::World);
    osg::ref_ptr<DartMPCWorldOSG> node =
        new DartMPCWorldOSG(world, std::make_unique<TrajectoryPlanner>(
            start_lift, end_lift), cmd, tgt_state);
    node->set_initial_state(initial_state);
    
    for (unsigned int i = 0; i < num_steps; i++) {
        node->step_once();
    }

    RBDState state = node->get_state();

    EXPECT_NEAR(state.pos.x(), tgt_state.pos.x(), X_TOL);
    EXPECT_NEAR(state.pos.y(), tgt_state.pos.y(), X_TOL);
    EXPECT_NEAR(state.pos.z(), tgt_state.pos.z(), X_TOL);

    Eigen::Quaterniond state_rot, tgt_rot;
    state_rot = state.rot;
    tgt_rot = tgt_state.rot;
    Eigen::Quaterniond rot_err = state_rot * tgt_rot.conjugate();
    EXPECT_NEAR(rot_err.x(), 0, R_TOL);
    EXPECT_NEAR(rot_err.y(), 0, R_TOL);
    EXPECT_NEAR(rot_err.z(), 0, R_TOL);
    EXPECT_NEAR(rot_err.w(), 1, R_TOL);

    EXPECT_NEAR(state.lin_vel.x(), tgt_state.lin_vel.x(), V_TOL);
    EXPECT_NEAR(state.lin_vel.y(), tgt_state.lin_vel.y(), V_TOL);
    EXPECT_NEAR(state.lin_vel.z(), tgt_state.lin_vel.z(), V_TOL);

    EXPECT_NEAR(state.ang_vel.x(), tgt_state.ang_vel.x(), W_TOL);
    EXPECT_NEAR(state.ang_vel.y(), tgt_state.ang_vel.y(), W_TOL);
    EXPECT_NEAR(state.ang_vel.z(), tgt_state.ang_vel.z(), W_TOL);

    EXPECT_FALSE(::testing::Test::HasFailure()) << "Failed with state:"
        << "\nstrt_x: " << initial_state.pos.transpose()
        << "\nstrt_r: " << initial_state.rot.eulerAngles(2, 1, 0).transpose()
        << "\nstrt_v: " << initial_state.lin_vel.transpose()
        << "\nstrt_w: " << initial_state.ang_vel.transpose()
        << "\ncurr_tgt_x: " << tgt_state.pos.transpose()
        << "\ncurr_tgt_r: " << tgt_state.rot.eulerAngles(2, 1, 0).transpose()
        << "\ncurr_tgt_v: " << tgt_state.lin_vel.transpose()
        << "\ncurr_tgt_w: " << tgt_state.ang_vel.transpose()
        << "\ninit_tgt_x: " << initial_tgt.pos.transpose()
        << "\ninit_tgt_r: " << initial_tgt.rot.eulerAngles(2, 1, 0).transpose()
        << "\ninit_tgt_v: " << initial_tgt.lin_vel.transpose()
        << "\ninit_tgt_w: " << initial_tgt.ang_vel.transpose();
    return (!::testing::Test::HasFailure());
};

TEST(TestSrbdMpc, noInitialOffsetNoMovement) {
    ASSERT_TRUE(ros::master::check());
    ros::NodeHandle nh;

    RBDState tgt_state;
    Eigen::Vector3d fixed_lin_vel(0, 0, 0);
    Eigen::Vector3d fixed_ang_vel(0, 0, 0);
    std::shared_ptr<CmdInterface> cmd_interface =
        std::make_shared<FixedCmdInterface>(
            fixed_lin_vel, fixed_ang_vel, tgt_state.pos, tgt_state.rot,
            tgt_state.lin_vel, tgt_state.ang_vel);

    RBDState initial_state;
    initial_state.pos = Eigen::Vector3d::Zero();
    initial_state.rot = Eigen::Matrix3d::Identity();
    initial_state.lin_vel = Eigen::Vector3d::Zero();
    initial_state.ang_vel = Eigen::Vector3d::Zero();

    // 0.01s per step -> 10s
    size_t num_steps = 1000;
    test_converge(
        cmd_interface, tgt_state, initial_state, num_steps,
        2 * MM_ACCURATE, ONE_PERCENT, 5 * CM_ACCURATE, 5 * CM_ACCURATE);
}

TEST(TestSrbdMpc, noInitialOffsetLinMovement) {
    ASSERT_TRUE(ros::master::check());
    ros::NodeHandle nh;

    RBDState tgt_state;
    Eigen::Vector3d fixed_lin_vel(0, 0, 0);
    Eigen::Vector3d fixed_ang_vel(0, 0, 0);
    std::shared_ptr<CmdInterface> cmd_interface;

    RBDState initial_state;
    auto reset = [&]() {
        initial_state.pos = Eigen::Vector3d::Zero();
        initial_state.rot = Eigen::Matrix3d::Identity();
        initial_state.lin_vel = Eigen::Vector3d::Zero();
        initial_state.ang_vel = Eigen::Vector3d::Zero();
        tgt_state = initial_state;
        cmd_interface = std::make_shared<FixedCmdInterface>(
            fixed_lin_vel, fixed_ang_vel, tgt_state.pos, tgt_state.rot,
            tgt_state.lin_vel, tgt_state.ang_vel);
    };
    reset();

    // 0.01s per step -> 3s
    size_t num_steps = 300;
    for (double x_vel = -1.0; x_vel < 1.0; x_vel += 0.1) {
        fixed_lin_vel = Eigen::Vector3d(x_vel, 0, 0);
        reset();
        double k = std::abs(x_vel * 5);
        ASSERT_TRUE(test_converge(
            cmd_interface, tgt_state, initial_state, num_steps,
            (2+k) * MM_ACCURATE, (1+k) * ONE_PERCENT, (5+k) * CM_ACCURATE,
            (5+k) * CM_ACCURATE));
    }
    num_steps = 100;
    for (double y_vel = -0.5; y_vel < 0.5; y_vel += 0.1) {
        fixed_lin_vel = Eigen::Vector3d(0, y_vel, 0);
        reset();
        double k = std::abs(y_vel * 8);
        ASSERT_TRUE(test_converge(
            cmd_interface, tgt_state, initial_state, num_steps,
            (2+k) * MM_ACCURATE, (1+k) * ONE_PERCENT, (5+k) * CM_ACCURATE,
            (5+k) * CM_ACCURATE));
    }
}

TEST(TestSrbdMpc, noInitialOffsetAngMovement) {
    ASSERT_TRUE(ros::master::check());
    ros::NodeHandle nh;

    RBDState tgt_state;
    Eigen::Vector3d fixed_lin_vel(0, 0, 0);
    Eigen::Vector3d fixed_ang_vel(0, 0, 0);
    std::shared_ptr<CmdInterface> cmd_interface;

    RBDState initial_state;
    auto reset = [&]() {
        initial_state.pos = Eigen::Vector3d::Zero();
        initial_state.rot = Eigen::Matrix3d::Identity();
        initial_state.lin_vel = Eigen::Vector3d::Zero();
        initial_state.ang_vel = Eigen::Vector3d::Zero();
        tgt_state = initial_state;
        cmd_interface = std::make_shared<FixedCmdInterface>(
            fixed_lin_vel, fixed_ang_vel, tgt_state.pos, tgt_state.rot,
            tgt_state.lin_vel, tgt_state.ang_vel);
    };
    reset();

    // 0.01s per step -> 1s
    size_t num_steps = 100;
    for (double z_vel = -M_PI; z_vel < M_PI; z_vel += 0.1) {
        fixed_ang_vel = Eigen::Vector3d(0, 0, z_vel);
        reset();
        double k = std::abs(z_vel * 5);
        ASSERT_TRUE(test_converge(
            cmd_interface, tgt_state, initial_state, num_steps,
            (2+k) * MM_ACCURATE, (1+k) * ONE_PERCENT, (5+k) * CM_ACCURATE,
            (5+k) * CM_ACCURATE));
    }
}

TEST(TestSrbdMpc, initialOffsetNoMovement) {
    ASSERT_TRUE(ros::master::check());
    ros::NodeHandle nh;

    RBDState tgt_state;
    Eigen::Vector3d fixed_lin_vel(0, 0, 0);
    Eigen::Vector3d fixed_ang_vel(0, 0, 0);
    std::shared_ptr<CmdInterface> cmd_interface;

    RBDState initial_state;
    auto reset = [&]() {
        initial_state.pos = Eigen::Vector3d::Zero();
        initial_state.rot = Eigen::Matrix3d::Identity();
        initial_state.lin_vel = Eigen::Vector3d::Zero();
        initial_state.ang_vel = Eigen::Vector3d::Zero();
        tgt_state = initial_state;
        cmd_interface = std::make_shared<FixedCmdInterface>(
            fixed_lin_vel, fixed_ang_vel, tgt_state.pos, tgt_state.rot,
            tgt_state.lin_vel, tgt_state.ang_vel);
    };
    reset();

    // 0.01s per step -> 3s
    size_t num_steps = 300;
    for (double x_off = -0.2; x_off < 0.2; x_off += 0.05) {
        for (double y_off = -0.2; y_off < 0.2; y_off += 0.05) {
            for (double z_off = -0.1; z_off < 0.1; z_off += 0.025) {
                reset();
                initial_state.pos = Eigen::Vector3d(x_off, y_off, z_off);
                double k = 0;
                ASSERT_TRUE(test_converge(
                    cmd_interface, tgt_state, initial_state, num_steps,
                    2.5 * MM_ACCURATE, 2 * ONE_PERCENT, 5 * CM_ACCURATE,
                    5 * CM_ACCURATE));
            }
        }
    }
}

} // ns test_srbd


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_mpc");
    testing::InitGoogleTest(&argc, argv);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    return RUN_ALL_TESTS();
}