#include <Eigen/Geometry>
#include <dart/dart.hpp>
#include <dart/common/Uri.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <ros/package.h>

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew_v;
    skew_v << 0, -v.z(), v.y(),
                v.z(), 0, -v.x(),
                -v.y(), v.x(), 0;
    return skew_v;
}

Eigen::Matrix6d transform_inertia(
    const Eigen::Matrix6d &I, const Eigen::Isometry3d &T)
{
    Eigen::Matrix6d adj;
    adj << T.linear(), Eigen::Matrix3d::Zero(),
           skew(T.translation()), T.linear();
    return adj.transpose() * I * adj;
}

struct InertiaData {
    Eigen::Matrix6d I_66 = Eigen::Matrix6d::Zero();
    double mass = 0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
};

void combine_com(
    const dart::dynamics::BodyNodePtr &body,
    const dart::dynamics::BodyNodePtr &rel_to,
    Eigen::Vector3d &cumulative_com,
    double &cumulative_mass,
    bool include_children)
{
    Eigen::Vector3d com = body->getCOM(rel_to);
    double mass = body->getMass();

    cumulative_com = (cumulative_com * cumulative_mass + com * mass) / (cumulative_mass + mass);
    cumulative_mass += mass;

    if (include_children) {
        for (size_t i = 0; i < body->getNumChildBodyNodes(); i++) {
            auto child = body->getChildBodyNode(i);
            // Skip feet
            if (child->getName().find("foot") != std::string::npos) {
                continue;
            }
            combine_com(child, rel_to, cumulative_com, cumulative_mass, include_children);
        }
    }
}

Eigen::Matrix6d get_body_inertia(
    const dart::dynamics::BodyNodePtr &body,
    const dart::dynamics::BodyNodePtr &rel_to,
    const Eigen::Vector3d &com_offset,
    bool include_children)
{
    double m = body->getInertia().getMass();
    Eigen::Matrix3d I_body_com = body->getInertia().getMoment();

    Eigen::Matrix3d R = body->getTransform(rel_to).rotation();
    // com_offset is with respect to rel_to, so reverse it
    Eigen::Vector3d r = body->getCOM(rel_to) - com_offset;
    Eigen::Matrix3d skew_r = skew(r);

    Eigen::Matrix6d I_66;
    I_66 << R * I_body_com * R.transpose() + m * skew_r.transpose() * skew_r,
        m * skew_r,
        m * skew_r.transpose(),
        m * Eigen::Matrix3d::Identity();

    
    if (include_children) {
        for (size_t i = 0; i < body->getNumChildBodyNodes(); i++) {
            auto child = body->getChildBodyNode(i);
            // Skip feet
            if (child->getName().find("foot") != std::string::npos) {
                continue;
            }
            I_66 += get_body_inertia(child, rel_to, com_offset, include_children);
        }
    }
    return I_66;
}

InertiaData get_body_inertia(
    const dart::dynamics::BodyNodePtr &body,
    const dart::dynamics::BodyNodePtr &rel_to,
    bool include_children=true)
{
    InertiaData data;
    combine_com(body, rel_to, data.com, data.mass, include_children);
    data.I_66 = get_body_inertia(body, rel_to, data.com, include_children);
    return data;
};

void print_inertia(const dart::dynamics::BodyNodePtr &root,  bool combine_children) {
    InertiaData data = get_body_inertia(root, root, combine_children);

    std::cout << root->getName() << " body inertia is:\n" << data.I_66
        << "\nRelative COM is:\n" << data.com.transpose()
        << "\nMass is: " << data.mass << std::endl;
}

int main(int argc, char **argv) {
    std::string ester_description =
        ros::package::getPath("ester_description");
    std::string urdf_path = ester_description + "/urdf/ester.urdf";
    dart::utils::DartLoader loader;
    loader.addPackageDirectory("ester_description", ester_description);
 
    auto skel = loader.parseSkeleton(
        dart::common::Uri(urdf_path), nullptr,
        dart::utils::DartLoader::Flags::FIXED_BASE_LINK);
    skel->setGravity(Eigen::Vector3d(0, 0, 0));

    print_inertia(skel->getBodyNode("spine_center_link"), false);
    print_inertia(skel->getBodyNode("front_spine_pitch_link"), true);
    print_inertia(skel->getBodyNode("rear_spine_pitch_link"), true);

    std::cerr << "\n\n FULL ROBOT INERTIA: " << std::endl;
    print_inertia(skel->getBodyNode("spine_center_link"), true);
}
