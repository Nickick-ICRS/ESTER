#include "ester_kinematics/leg_dynamics.hpp"

#include <ros/ros.h>

#include <dart/gui/osg/osg.hpp>

using namespace ester_common;
using namespace ester_kinematics;

class CustomWorldNode : public dart::gui::osg::WorldNode {
public:
    CustomWorldNode(
        const std::array<std::shared_ptr<LegDynamics>, 4> &dyns,
        const std::array<dart::gui::osg::InteractiveFramePtr, 4> &targets,
        const dart::simulation::WorldPtr &world = nullptr)
        : dart::gui::osg::WorldNode(world), dyns_(dyns), targets_(targets)
    {
        for (unsigned int i = 0; i < 4; i++) {
            auto tf = targets_[i]->getTransform();
            tf.translation() =
                dyns_[i]->get_leg()->getBodyNode(4)->getTransform()
                    .translation();
            targets_[i]->setTransform(tf);
        }
    };

    void solve_ik_target() {
        for (unsigned int i = 0; i < 4; i++) {
            const auto &target = targets_[i];
            const auto &dyn = dyns_[i];
            Eigen::Vector3d &prev_tgt = prev_tgts_[i];
            Eigen::Vector3d tgt = target->getTransform().translation();
            if ((prev_tgt - tgt).norm() < 1e-6) {
                continue;
            }
            prev_tgt = tgt;
            auto ik_res = dyn->solve_ik(tgt);
            auto offset = dyn->get_leg()->getJoint(0)->
                getTransformFromParentBodyNode().inverse();
            if (ik_res.reachable) {
                Eigen::Vector3d pos = dyn->solve_fk(ik_res.pos)
                    .translation();
                ROS_INFO_STREAM(
                    "\nTarget: " << (offset * tgt).transpose() << " IK / FK Error: "
                    << (pos - tgt).norm());
                ROS_INFO_STREAM(""
                    << LegJointId::HIP_ROLL << ": "
                    << ik_res.pos[LegJointId::HIP_ROLL]
                    << LegJointId::HIP_PITCH << ": "
                    << ik_res.pos[LegJointId::HIP_PITCH]
                    << LegJointId::KNEE << ": "
                    << ik_res.pos[LegJointId::KNEE]);
            }
            else {
                ROS_INFO_STREAM(
                    "\nTarget: " << (offset * tgt).transpose() << " unreachable");
            }
        }
    };

    void customPreRefresh() override {
        solve_ik_target();
    };

private:
    std::array<std::shared_ptr<LegDynamics>, 4> dyns_;
    std::array<dart::gui::osg::InteractiveFramePtr, 4> targets_;
    std::array<Eigen::Vector3d, 4> prev_tgts_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_dynamics");

    dart::simulation::WorldPtr world(new dart::simulation::World);
    std::array<std::shared_ptr<LegDynamics>, 4> dyns;
    dyns[0] = std::make_shared<LegDynamics>(LegId::FL);
    dyns[1] = std::make_shared<LegDynamics>(LegId::FR);
    dyns[2] = std::make_shared<LegDynamics>(LegId::RL);
    dyns[3] = std::make_shared<LegDynamics>(LegId::RR);
    //dyns[0]->get_leg()->getJoint(0)->setTransformFromParentBodyNode(
    //    Eigen::Isometry3d::Identity()*Eigen::Translation3d(0.5, 0.5, 0));
    dyns[1]->get_leg()->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity()*Eigen::Translation3d(0.5, -0.5, 0));
    dyns[2]->get_leg()->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity()*Eigen::Translation3d(-0.5, 0.5, 0));
    dyns[3]->get_leg()->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity()*Eigen::Translation3d(-0.5, -0.5, 0));

    std::array<dart::gui::osg::InteractiveFramePtr, 4> targets;
    for (unsigned int i = 0; i < 4; i++) {
        targets[i].reset(
            new dart::gui::osg::InteractiveFrame(
                dart::dynamics::Frame::World()));
        world->addSkeleton(dyns[i]->get_leg());
        world->addSimpleFrame(targets[i]);
    }

    osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(
        dyns, targets, world);

    dart::gui::osg::ImGuiViewer viewer;
    viewer.addWorldNode(node);
    for (unsigned int i = 0; i < 4; i++) {
        viewer.enableDragAndDrop(targets[i].get());
    }
    viewer.addEventHandler(new osgGA::GUIEventHandler());
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3( 2.57f,  3.14f, 1.64f),
        ::osg::Vec3( 0.00f,  0.00f, 0.00f),
        ::osg::Vec3(-0.24f, -0.25f, 0.94f));
    viewer.setCameraManipulator(viewer.getCameraManipulator());

    viewer.run();
    return 0;
}
