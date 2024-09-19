#include <ester_kinematics/inertia_calculator.hpp>
#include <ester_kinematics/load_robot_model.hpp>

#include <ros/package.h>

namespace dd = dart::dynamics;

namespace ester_kinematics
{

Eigen::Matrix3d skew(const Eigen::Vector3d &X) {
    Eigen::Matrix3d m;
    m <<     0, -X(2),  X(1),
          X(2),     0, -X(0),
         -X(1),  X(0),     0; 
    return m;
}

std::pair<double, Eigen::Vector3d> get_total_mass_and_com(
    const dd::BodyNodePtr &body, const dd::BodyNodePtr &rel_to)
{
    double mass = body->getMass();
    Eigen::Vector3d com = body->getCOM(rel_to);
    for (size_t i = 0; i < body->getNumChildBodyNodes(); i++) {
        auto mass_and_com = get_total_mass_and_com(body->getChildBodyNode(i), rel_to);
        com = (mass * com + mass_and_com.first * mass_and_com.second) / (mass + mass_and_com.first);
        mass += mass_and_com.first;
    }
    return std::make_pair(mass, com);
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

EsterInertiaCalculator::EsterInertiaCalculator(
    const dart::dynamics::SkeletonPtr &robot)
{
    robot->setPositions(Eigen::Matrix<double, 22, 1>::Zero());
    auto chassis = robot->getBodyNode("spine_center_link");
    auto front_spine_pitch_link = robot->getBodyNode("front_spine_pitch_link");
    auto front_spine_yaw_link = robot->getBodyNode("front_spine_yaw_link");
    auto rear_spine_pitch_link = robot->getBodyNode("rear_spine_pitch_link");
    auto rear_spine_yaw_link = robot->getBodyNode("rear_spine_yaw_link");

    spine_pos_pre_joint_[0] = front_spine_pitch_link->getTransform(chassis).translation();
    spine_pos_pre_joint_[1] = chassis->getTransform(chassis).translation();
    spine_pos_pre_joint_[2] = rear_spine_pitch_link->getTransform(chassis).translation();

    yaw_pitch_offset_[0] = front_spine_yaw_link->getTransform(front_spine_pitch_link).translation();
    yaw_pitch_offset_[1] = rear_spine_yaw_link->getTransform(rear_spine_pitch_link).translation();

    auto front_mass_and_com = get_total_mass_and_com(front_spine_pitch_link, front_spine_yaw_link);
    auto rear_mass_and_com = get_total_mass_and_com(rear_spine_pitch_link, rear_spine_yaw_link);
    spine_pos_post_joint_[0] = front_mass_and_com.second;
    spine_pos_post_joint_[1] = chassis->getCOM(chassis);
    spine_pos_post_joint_[2] = rear_mass_and_com.second;

    rbd_masses_[0] = front_mass_and_com.first;
    rbd_masses_[1] = chassis->getMass();
    rbd_masses_[2] = rear_mass_and_com.first;

    rbd_inertias_[0] = get_total_inertia(
        front_spine_pitch_link, front_spine_yaw_link, front_mass_and_com.second);
    rbd_inertias_[1] = chassis->getInertia().getMoment();
    rbd_inertias_[2] = get_total_inertia(
        rear_spine_pitch_link, rear_spine_yaw_link, rear_mass_and_com.second);
}

EsterInertiaCalculator::Inertia EsterInertiaCalculator::calculate_inertia(
    const Eigen::VectorXd &spine_positions) const
{
    assert(spine_positions.size() == 4);
    Inertia inertia;
    inertia.com.setZero();
    inertia.I.setZero();

    std::array<Eigen::Isometry3d, 3> sp_tfs;

    sp_tfs[0] = calculate_rbd_transform(0, spine_positions);
    sp_tfs[1] = calculate_rbd_transform(1, spine_positions);
    sp_tfs[2] = calculate_rbd_transform(2, spine_positions);

    for (unsigned int i = 0; i < 3; i++) {
        double m = rbd_masses_[i];
        inertia.com += m * sp_tfs[i].translation();
    }
    inertia.com /= rbd_masses_[0] + rbd_masses_[1] + rbd_masses_[2];

    for (unsigned int i = 0; i < 3; i++) {
        double m = rbd_masses_[i];
        const Eigen::Matrix3d &I = rbd_inertias_[i];
        Eigen::Matrix3d skew_p = skew(sp_tfs[i].translation() - inertia.com);
        const Eigen::Matrix3d &R = sp_tfs[i].rotation();
        inertia.I += R * I * R.transpose() + m * skew_p.transpose() * skew_p;
    }

    return inertia;
}

Eigen::Matrix3d EsterInertiaCalculator::calculate_inertia_dot(
    const Eigen::VectorXd &spine_positions,
    const Eigen::VectorXd &spine_velocities) const
{
    assert(spine_positions.size() == 4);
    assert(spine_positions.size() == spine_velocities.size());
    // Numerical approach is about 2x faster than analytical in practice
    const double dt = 1e-3;
    Inertia I_now = calculate_inertia(spine_positions);
    Inertia I_next = calculate_inertia(spine_positions + spine_velocities * dt);
    return (I_next.I - I_now.I) / dt;
}

std::pair<EsterInertiaCalculator::Inertia, Eigen::Matrix3d>
EsterInertiaCalculator::calculate_inertia_and_derivative(
    const Eigen::VectorXd &spine_positions,
    const Eigen::VectorXd &spine_velocities) const
{
    assert(spine_positions.size() == 4);
    assert(spine_positions.size() == spine_velocities.size());
    // Numerical approach is about 2x faster than analytical in practice
    const double dt = 1e-3;
    Inertia I_now = calculate_inertia(spine_positions);
    Inertia I_next = calculate_inertia(spine_positions + spine_velocities * dt);
    return std::make_pair(I_now, (I_next.I - I_now.I) / dt);
}

Eigen::Isometry3d EsterInertiaCalculator::calculate_rbd_transform(
    size_t rbd_id, const Eigen::VectorXd &spine_pos) const
{
    assert(spine_pos.size() == 4);
    assert(rbd_id < 3);
    Eigen::Isometry3d tf;
    // Front, relative to middle
    if (rbd_id == 0) {
        tf = Eigen::Translation3d(spine_pos_pre_joint_[0])
           * Eigen::AngleAxisd(spine_pos(0), Eigen:: Vector3d::UnitY())
           * Eigen::Translation3d(yaw_pitch_offset_[0])
           * Eigen::AngleAxisd(spine_pos(1), Eigen::Vector3d::UnitZ())
           * Eigen::Translation3d(spine_pos_post_joint_[0]);
    }
    // Middle COM, relative to middle FRAME
    else if (rbd_id == 1) {
        tf = Eigen::Translation3d(spine_pos_post_joint_[1]);
    }
    // Rear, relative to middle
    else if (rbd_id == 2) {
        tf = Eigen::Translation3d(spine_pos_pre_joint_[2])
           * Eigen::AngleAxisd(spine_pos(2), Eigen::Vector3d::UnitY())
           * Eigen::Translation3d(yaw_pitch_offset_[1])
           * Eigen::AngleAxisd(spine_pos(3), Eigen::Vector3d::UnitZ())
           * Eigen::Translation3d(spine_pos_post_joint_[2]);
    }
    return tf;
}

} // namespace ester_kinematics