#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <ros/package.h>

namespace dd = dart::dynamics;
namespace ds = dart::simulation;
namespace dc = dart::collision;

class ApplyForceWorldNode :public dart::gui::osg::WorldNode {
public:
    ApplyForceWorldNode(const ds::WorldPtr &world)
        :dart::gui::osg::WorldNode(world), world_(world)
    {
        const double TIMESTEP = 1e-4;
        world_->setTimeStep(TIMESTEP);
        world_->setGravity(Eigen::Vector3d::Zero());
    
        setup_plane();
        setup_robot_leg();

        reset();
    };

    virtual ~ApplyForceWorldNode() = default;

    void step_once() {
        customPreStep();
        world_->step();
        customPostStep();
    }

    void reset() {
        Eigen::VectorXd zeros = robot_->getPositions();
        zeros.setZero();
        robot_->setPositions(zeros);
        robot_->setVelocities(zeros);
        robot_->setForces(zeros);

        // "Hip A"
        robot_->setPosition(0, 1.06874e-6);
        // "Hip B"
        robot_->setPosition(1, 0.811791);
        // "Knee"
        robot_->setPosition(2, -1.62481);
    };

    void customPreStep() override {
        step_controller();
    };

    void customPostStep() override {
        get_and_print_contact_forces();
    };

private:
    void step_controller() {
        // We expect a reaction force of 5 Nm in the z direction once we make contact
        Eigen::Vector3d target_force(0, 0, -5.0);
        
        // Simple Jacobian transpose torque control
        auto J = robot_->getLinearJacobian(robot_ee_);
        auto torques = J.transpose() * target_force;
        robot_->setForces(torques);

        std::cerr << "Requested torques: " << torques.transpose() << std::endl;
    };

    void get_and_print_contact_forces() {
        dart::collision::CollisionResult result;
        result = world_->getLastCollisionResult();
        bool collision = result.getNumContacts() > 0;
        if (collision) {
            // Total force should be the sum of the individual forces (ignoring wrench)
            Eigen::Vector3d total_linear_force = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < result.getNumContacts(); i++) {
                // Only one object in each group so we don't need to check that
                // we have the correct object
                auto con = result.getContact(i);
                std::cerr << con.collisionObject1->getShapeFrame()->getParentFrame()->getName()
                    << " - " << con.collisionObject2->getShapeFrame()->getParentFrame()->getName()
                    << std::endl;
                total_linear_force += result.getContact(i).force;
            }
            std::cout << "Contact force: " << total_linear_force.transpose() << std::endl;
        }
        else {
            std::cout << "No collisions" << std::endl;
        }

        std::cerr << "Joint pos: " << robot_->getPositions().transpose() << std::endl;
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

    void setup_robot_leg() {
        std::string ester_description = ros::package::getPath(
            "ester_description");
        std::string urdf_path = ester_description + "/urdf/leg/rear_left_leg.urdf";
        dart::utils::DartLoader loader;
        loader.addPackageDirectory("ester_description", ester_description);
        robot_ = loader.parseSkeleton(
            dart::common::Uri(urdf_path), nullptr,
            dart::utils::DartLoader::FIXED_BASE_LINK);
        robot_ee_ = robot_->getBodyNode(robot_->getNumBodyNodes() - 1);

        for (size_t i = 0; i < robot_->getNumDofs(); i++) {
            auto dof = robot_->getDof(i);
            dof->setCoulombFriction(0);
            dof->setDampingCoefficient(0);
        }

        // Start in the sky pointing down
        robot_->getJoint(0)->setTransformFromParentBodyNode(
            Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.2));

        world_->addSkeleton(robot_);
    };

    ds::WorldPtr world_;

    dd::SkeletonPtr plane_;
    dd::SkeletonPtr robot_;

    dd::BodyNodePtr robot_ee_;
};

class EventHandler : public osgGA::GUIEventHandler {
public:
    EventHandler(const osg::ref_ptr<ApplyForceWorldNode> &node)
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
    osg::ref_ptr<ApplyForceWorldNode> node_;
};

int main(int argc, char **argv) {
    dart::gui::osg::ImGuiViewer viewer;
    ds::WorldPtr world = std::make_shared<ds::World>();
    osg::ref_ptr<ApplyForceWorldNode> world_node =
        new ApplyForceWorldNode(world);
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