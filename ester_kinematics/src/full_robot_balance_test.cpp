#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <ros/package.h>

#include <ester_common/ester_enums.hpp>
#include "ester_kinematics/leg_dynamics.hpp"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;
namespace dc = dart::collision;

using namespace ester_common;
using namespace ester_kinematics;

class EsterRobotWorldNode :public dart::gui::osg::WorldNode {
public:
    EsterRobotWorldNode(const ds::WorldPtr &world)
        :dart::gui::osg::WorldNode(world), world_(world)
    {
        const double TIMESTEP = 2.5e-3;
        world_->setTimeStep(TIMESTEP);
        world_->setGravity(Eigen::Vector3d::UnitZ() * -9.81);

        for (const auto &leg : ALL_LEG_IDS) {
            dyns_[leg] = std::make_shared<LegDynamics>(leg);
        }
    
        setup_plane();
        setup_robot();

        reset();
    };

    virtual ~EsterRobotWorldNode() = default;

    void step_once() {
        customPreStep();
        world_->step();
        customPostStep();
    };

    void reset() {
        robot_->setPositions(dof_zeros_);
        robot_->setVelocities(zeros_);
        robot_->setForces(zeros_);

        prev_err_ = get_err();
    };

    void customPreStep() override {
        step_controller();
    };

    void customPostStep() override {
        update_contacts();
    };

private:
    void step_controller() {
        double fz = -robot_->getMass() * world_->getGravity().z();
        Eigen::Vector3d target_force(0, 0, -fz);

        Eigen::Vector6d err = get_err();

        Eigen::Vector6d derr = (err - prev_err_) / world_->getTimeStep();
        prev_err_ = err;

        const double kp_xyz = 6e2;
        const double kd_xyz = 3e1;
        target_force -= err.tail<3>() * kp_xyz + derr.tail<3>() * kd_xyz;

        const double kp_rp = 2e1;
        const double kd_rp = 1e0;
        double front_mult = 0.5 - kp_rp * err(1) - kd_rp * derr(1);
        if (front_mult > 1) front_mult = 1;
        if (front_mult < 0) front_mult = 0;
        double right_mult = 0.5 - kp_rp * err(0) - kd_rp * derr(0);
        if (right_mult > 1) right_mult = 1;
        if (right_mult < 0) right_mult = 0;

        Eigen::Vector3d front_force = front_mult * target_force;
        Eigen::Vector3d rear_force = (1 - front_mult) * target_force;

        LegMap<Eigen::Vector3d> leg_forces;
        leg_forces[LegId::FL] = front_force * (1 - right_mult);
        leg_forces[LegId::FR] = front_force * right_mult;
        leg_forces[LegId::RL] = rear_force * (1 - right_mult);
        leg_forces[LegId::RR] = rear_force * right_mult;

        // Simple Jacobian transpose torque control
        Eigen::VectorXd torques = robot_->getForces();
        LegMap<size_t> leg_col;
        leg_col[LegId::FL] = 8;
        leg_col[LegId::FR] = 11;
        leg_col[LegId::RL] = 16;
        leg_col[LegId::RR] = 19;
        for (const auto &leg : ALL_LEG_IDS) {
            auto J = robot_->getLinearJacobian(feet_[leg]).block<3, 3>(0, leg_col[leg]);
            torques.segment<3>(leg_col[leg]) = J.transpose() * leg_forces[leg];
        }
        robot_->setForces(torques);
    };

    void setup_plane() {
        plane_ = dd::Skeleton::create("Plane");
        dd::BodyNodePtr node;
            node = plane_->createJointAndBodyNodePair<dd::WeldJoint>().second;
        const double t = 0.1;
        auto box = std::make_shared<dd::BoxShape>(Eigen::Vector3d(1, 1, t));
        auto shape_node = node->createShapeNodeWith<
            dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(box);
        shape_node->getVisualAspect()->setColor(dart::Color::Fuchsia(0.8));
        // Set plane top surface to be at z = 0
        node->getParentJoint()->setTransformFromParentBodyNode(
            Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, -t/2));
        world_->addSkeleton(plane_);
    };

    void setup_robot() {
        std::string ester_description = ros::package::getPath(
            "ester_description");
        std::string urdf_path = ester_description + "/urdf/ester.urdf";
        dart::utils::DartLoader loader;
        loader.addPackageDirectory("ester_description", ester_description);
        robot_ = loader.parseSkeleton(dart::common::Uri(urdf_path));

        /*
        for (size_t i = 0; i < robot_->getNumDofs(); i++) {
            auto dof = robot_->getDof(i);
            dof->setCoulombFriction(0);
            dof->setDampingCoefficient(0);
        }
        */
        
        chassis_ = robot_->getBodyNode("spine_center_link");
        feet_[LegId::FL] = robot_->getBodyNode("front_left_foot_contact_link");
        feet_[LegId::FR] = robot_->getBodyNode("front_right_foot_contact_link");
        feet_[LegId::RL] = robot_->getBodyNode("rear_left_foot_contact_link");
        feet_[LegId::RR] = robot_->getBodyNode("rear_right_foot_contact_link");

        robot_->getJoint("front_spine_pitch_joint")->setActuatorType(dd::Joint::ActuatorType::LOCKED);
        robot_->getJoint("front_spine_yaw_joint")->setActuatorType(dd::Joint::ActuatorType::LOCKED);
        robot_->getJoint("rear_spine_pitch_joint")->setActuatorType(dd::Joint::ActuatorType::LOCKED);
        robot_->getJoint("rear_spine_yaw_joint")->setActuatorType(dd::Joint::ActuatorType::LOCKED);

        zeros_.resize(robot_->getNumDofs());
        zeros_.setZero();
        dof_zeros_ = zeros_;

        // FL
        dof_zeros_(9) = 0.531;
        dof_zeros_(10) = -0.996;
        // FR
        dof_zeros_(12) = 0.531;
        dof_zeros_(13) = -0.996;
        // RL
        dof_zeros_(17) = -0.353;
        dof_zeros_(18) = 1.053;
        // RR
        dof_zeros_(20) = -0.353;
        dof_zeros_(21) = 1.053;

        // Start in the sky
        robot_->getJoint(0)->setTransformFromParentBodyNode(
            Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.26));

        world_->addSkeleton(robot_);
    };

    void update_contacts() {
        auto ce = world_->getConstraintSolver()->getCollisionDetector();
        auto cg_gp = ce->createCollisionGroup(plane_->getBodyNode(0));
        for (const auto &leg : ALL_LEG_IDS) {
            auto cg = ce->createCollisionGroup(feet_[leg]);
            dc::CollisionOption opt;
            dc::CollisionResult res;
            bool collision = cg_gp->collide(cg.get(), opt, &res);
            contacts_[leg] = collision;
        }
    };

    Eigen::Vector6d get_err() {
        Eigen::Isometry3d tgt_state;
        tgt_state = Eigen::Translation3d(0, 0, 0.2);
        Eigen::Vector6d err = dart::math::logMap(chassis_->getWorldTransform().inverse() * tgt_state);
        return err;
    };

    ds::WorldPtr world_;

    dd::SkeletonPtr plane_;
    dd::SkeletonPtr robot_;

    LegMap<dd::BodyNodePtr> feet_;
    LegMap<std::shared_ptr<LegDynamics>> dyns_;
    LegMap<bool> contacts_;
    dd::BodyNodePtr chassis_;
    
    Eigen::VectorXd zeros_;
    Eigen::VectorXd dof_zeros_;

    Eigen::Vector6d prev_err_;
};

class EventHandler : public osgGA::GUIEventHandler {
public:
    EventHandler(const osg::ref_ptr<EsterRobotWorldNode> &node)
    {
        node_ = node;
        node_->reset();
    };

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter&) override
    {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
        {
            if (ea.getKey() == 'r' || ea.getKey() == 'R') {
                node_->reset();
            }
            else if (ea.getKey() == 's' || ea.getKey() == 'S') {
                node_->step_once();
            }
            return true;
        }
        return false;
    }

private:
    osg::ref_ptr<EsterRobotWorldNode> node_;
};

int main(int argc, char **argv) {
    dart::gui::osg::ImGuiViewer viewer;
    ds::WorldPtr world = std::make_shared<ds::World>();
    osg::ref_ptr<EsterRobotWorldNode> world_node =
        new EsterRobotWorldNode(world);
    viewer.addWorldNode(world_node);
    viewer.addEventHandler(new EventHandler(world_node));
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.getCameraManipulator()->setHomePosition(
        osg::Vec3( 2.57f,  3.14f, 1.64f),
        osg::Vec3( 0.00f,  0.00f, 0.00f),
        osg::Vec3(-0.24f, -0.25f, 0.94f));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
    return 0;
}