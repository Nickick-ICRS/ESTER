#include "ester_kinematics/leg_dynamics.hpp"

// For ROS_INFO etc
#include <ros/ros.h>
#include <ros/package.h>

#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#ifndef ESTER_KINEMATICS_LIBRARY_PATH
#error "Needs -DESTER_KINEMATICS_LIBRARY_PATH to be set"
#endif

using namespace ester_common;
using namespace ester_kinematics;

using ErrorMethod = dart::dynamics::InverseKinematics::ErrorMethod;

class PositionOnlyObjective : public ErrorMethod {
public:
    PositionOnlyObjective(
        dart::dynamics::InverseKinematics *ik, dart::dynamics::BodyNodePtr ee,
        const Eigen::Isometry3d &tgt, const std::string name="PositionOnly")
        : ErrorMethod(ik, name), ee_(ee), tgt_(tgt) {};
    
    virtual Eigen::Vector6d computeError() override {
        Eigen::Vector6d err = Eigen::Vector6d::Zero();
        err.segment<3>(3) = ee_->getWorldTransform().translation() - tgt_.translation();
        return err;
    };

    virtual Eigen::Isometry3d computeDesiredTransform(
        const Eigen::Isometry3d &currentTf,
        const Eigen::Vector6d &error) override
    {
        return tgt_;
    };

    virtual std::unique_ptr<ErrorMethod> clone(dart::dynamics::InverseKinematics *ik) const override {
        std::unique_ptr<ErrorMethod> ret;
        ret.reset(new PositionOnlyObjective(ik, ee_, tgt_));
        return ret;
    };

protected:
    dart::dynamics::BodyNodePtr ee_;
    Eigen::Isometry3d tgt_;
};

LegDynamics::LegDynamics(LegId id) :LegDynamicsBase(id) {
    std::string ester_description = ros::package::getPath(
        "ester_description");
    std::string leg_name;
    std::string ik_filename;
    switch (id) {
    case LegId::FL:
        leg_name = "front_left_leg";
        ik_filename = "libester_kinematics_ikfast_front_left.so";
        pos_knee_ = false;
        break;
    case LegId::FR:
        leg_name = "front_right_leg";
        ik_filename = "libester_kinematics_ikfast_front_right.so";
        pos_knee_ = false;
        break;
    case LegId::RL:
        leg_name = "rear_left_leg";
        ik_filename = "libester_kinematics_ikfast_rear_left.so";
        pos_knee_ = true;
        break;
    case LegId::RR:
        leg_name = "rear_right_leg";
        ik_filename = "libester_kinematics_ikfast_rear_right.so";
        pos_knee_ = true;
        break;
    }
    std::string urdf_path =
        ester_description + "/urdf/leg/" + leg_name + ".urdf";
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
    leg_ = loader.parseSkeleton(
        dart::common::Uri(urdf_path), nullptr,
        dart::utils::DartLoader::Flags::FIXED_BASE_LINK);
    leg_->setGravity(Eigen::Vector3d(0, 0, 0));

    // Set foot mass to sth big to swamp out other masses
    foot_ = leg_->getBodyNode(4);

    hip_roll_  = leg_->getJoint(1);
    hip_pitch_ = leg_->getJoint(2);
    knee_      = leg_->getJoint(3);

    // Force "nice" joint limits on knees
    if (id == LegId::FL || id == LegId::FR) {
        knee_->setPositionUpperLimit(0, -1e-2);
    }
    else {
        knee_->setPositionLowerLimit(0, 1e-2);
    }

    auto set_joint = [](auto &dart_joint, double pos) {
        dart_joint->setPosition(0, pos);
        dart_joint->setVelocity(0, 0);
        dart_joint->setForce(0, 0);
    };
    set_joint(hip_roll_, 0);
    set_joint(hip_pitch_, 0);
    set_joint(knee_, 0);

    leg_->computeForwardKinematics();

    auto ee = foot_->createEndEffector("ee" + str(id));
    ik_ = ee->getIK(true);
    ik_->setGradientMethod<dart::dynamics::InverseKinematics::JacobianTranspose>();
    auto solver = std::static_pointer_cast<dart::optimizer::GradientDescentSolver>(ik_->getSolver());
    solver->setTolerance(1e-6);

    auto ee2 = foot_->createEndEffector("ee_analytical" + str(id));
    ik_analytical_ = ee->getIK(true);
    std::string lib_path = ESTER_KINEMATICS_LIBRARY_PATH + std::string("/") + ik_filename;
    ROS_INFO_STREAM("[dynamics] Loading IKFast solver from " << lib_path);
    std::vector<std::size_t> active_indices{0, 1, 2};
    std::vector<std::size_t> inactive_indices{};
    ik_analytical_->setGradientMethod<dart::dynamics::SharedLibraryIkFast>(
        lib_path, active_indices, inactive_indices, "translation_"+str(id));

    ROS_INFO_STREAM(
        "[dynamics] " << leg_name << " dynamics solver set up completed");
}

LegDynamics::IdResult LegDynamics::solve_id(
    const LegJointMap<double> &ja, const LegJointMap<double> &jv,
    const Eigen::Vector3d &eef_vel, const Eigen::Vector3d &grf) const
{
    IdResult result;
    Eigen::Vector3d pos, vel;
    pos(0) = ja.at(LegJointId::HIP_ROLL);
    pos(1) = ja.at(LegJointId::HIP_PITCH);
    pos(2) = ja.at(LegJointId::KNEE);
    vel(0) = jv.at(LegJointId::HIP_ROLL);
    vel(1) = jv.at(LegJointId::HIP_PITCH);
    vel(2) = jv.at(LegJointId::KNEE);

    leg_->clearExternalForces();
    leg_->clearInternalForces();
    leg_->setPositions(pos);
    leg_->setVelocities(vel);

    auto linear_jacobian = leg_->getLinearJacobian(foot_);
    const Eigen::Vector3d jnt_vel = linear_jacobian.transpose() * eef_vel;
    const Eigen::Vector3d T = linear_jacobian.transpose() * -grf;

    result.eff[LegJointId::HIP_ROLL]  = T(0);
    result.eff[LegJointId::HIP_PITCH] = T(1);
    result.eff[LegJointId::KNEE]      = T(2);
    result.vel[LegJointId::HIP_ROLL]  = jnt_vel(0);
    result.vel[LegJointId::HIP_PITCH] = jnt_vel(1);
    result.vel[LegJointId::KNEE]      = jnt_vel(2);

    return result;
}

LegDynamics::IkResult LegDynamics::solve_ik(
    const Eigen::Vector3d &tgt, const LegJointMap<double> &current_pos) const
{
    LegJointMap<double> cp(current_pos);
    IkResult result;
    result.reachable = true;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = tgt;
    T = leg_->getJoint(0)->getTransformFromParentBodyNode().inverse() * T;
    ik_analytical_->setErrorMethod<PositionOnlyObjective>(foot_, T);
    ik_->setErrorMethod<PositionOnlyObjective>(foot_, T);

    auto zero_joint = [](auto &dart_joint)
    {
        dart_joint->setPosition(0, 0);
        dart_joint->setVelocity(0, 0);
        dart_joint->setForce(0, 0);
    };
    zero_joint(hip_roll_);
    zero_joint(hip_pitch_);
    zero_joint(knee_);

    auto tp1 = std::chrono::steady_clock::now();
    Eigen::VectorXd sols;
    result.reachable = ik_analytical_->solveAndApply(sols);
    auto solus = ((dart::dynamics::InverseKinematics::Analytical*)(&ik_analytical_->getGradientMethod()))->getSolutions();
    double best_err = std::numeric_limits<double>::infinity();
    unsigned int num_sols = 0;
    for (const auto &sol : solus) {
        if (sol.mValidity == dart::dynamics::SharedLibraryIkFast::Validity_t::VALID) {
            LegJointMap<double> pos;
            pos[LegJointId::HIP_ROLL] = sol.mConfig[0];
            pos[LegJointId::HIP_PITCH] = sol.mConfig[1];
            pos[LegJointId::KNEE] = sol.mConfig[2];
            double err = std::abs(cp[LegJointId::HIP_ROLL] - sol.mConfig[0])
                       + std::abs(cp[LegJointId::HIP_PITCH] - sol.mConfig[1])
                       + std::abs(cp[LegJointId::KNEE] - sol.mConfig[2]);
            if (err < best_err) {
                if (num_sols > 0) {
                    result.alternative_solutions.insert(
                        result.alternative_solutions.begin(), result.pos);
                }
                result.pos[LegJointId::HIP_ROLL] = sol.mConfig[0];
                result.pos[LegJointId::HIP_PITCH] = sol.mConfig[1];
                result.pos[LegJointId::KNEE] = sol.mConfig[2];
                best_err = err;
                result.reachable = true;
            }
            else {
                result.alternative_solutions.push_back(pos);
            }
            num_sols++;
        }
    }
    /*
    // Backup numerical attempt
    unsigned int attempt = 0;
    const unsigned int MAX_ATTEMPTS = 5;
    while (!result.reachable && attempt < MAX_ATTEMPTS) {
        result.reachable = ik_->solveAndApply(sols);
        result.pos[LegJointId::HIP_ROLL] = sols(0);
        result.pos[LegJointId::HIP_PITCH] = sols(1);
        result.pos[LegJointId::KNEE] = sols(2);
        auto fk_pos = solve_fk(result.pos).translation();
        if ((tgt - fk_pos).norm() < 1e-6) {
            result.reachable = true;
        }
        attempt++;
    }
    auto tp2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt_s = tp2 - tp1;
    */
    return result;
}

Eigen::Isometry3d LegDynamics::solve_fk(const LegJointMap<double> &ja) const
{

    auto set_joint = [](auto &dart_joint, double pos) {
        dart_joint->setPosition(0, pos);
        dart_joint->setVelocity(0, 0);
        dart_joint->setForce(0, 0);
    };
    set_joint(hip_roll_, ja.at(LegJointId::HIP_ROLL));
    set_joint(hip_pitch_, ja.at(LegJointId::HIP_PITCH));
    set_joint(knee_, ja.at(LegJointId::KNEE));

    leg_->computeForwardKinematics();
    return foot_->getTransform();
}

Eigen::Vector3d LegDynamics::solve_fd(
    const LegJointMap<double> &pos, const LegJointMap<double> &vel,
    const LegJointMap<double> &eff, Eigen::Vector3d *foot_vel) const
{
    auto set_joint = [](auto &dart_joint, double p, double v, double e) {
        dart_joint->setPosition(0, p);
        dart_joint->setVelocity(0, v);
        dart_joint->setForce(0, e);
    };
    set_joint(
        hip_roll_, pos.at(LegJointId::HIP_ROLL),
        vel.at(LegJointId::HIP_ROLL), eff.at(LegJointId::HIP_ROLL));
    set_joint(
        hip_pitch_, pos.at(LegJointId::HIP_PITCH),
        vel.at(LegJointId::HIP_PITCH), eff.at(LegJointId::HIP_PITCH));
    set_joint(
        knee_, pos.at(LegJointId::KNEE),
        vel.at(LegJointId::KNEE), eff.at(LegJointId::KNEE));
    
    double m = foot_->getMass();
    const double BIG_NUMBER = 1e12;
    foot_->setMass(BIG_NUMBER);

    leg_->computeForwardKinematics();
    leg_->computeForwardDynamics();

    Eigen::Vector3d acc = foot_->getCOMLinearAcceleration();

    if (foot_vel != nullptr) {
        *foot_vel = foot_->getCOMLinearVelocity();
    }

    foot_->setMass(m);

    // Also gotta flip sign, coz we expect GRF
    return -1 * acc * BIG_NUMBER;
}
