#ifndef __INERTIA_CALCULATOR_HPP__
#define __INERTIA_CALCULATOR_HPP__

#include <array>
#include <Eigen/Geometry>
#include <dart/dart.hpp>

namespace ester_kinematics
{

class InertiaCalculatorInterface {
public:
    struct Inertia {
        Eigen::Vector3d com;
        Eigen::Matrix3d I;
    };

    /**
     * @brief Calculates the inertia tensor and center of mass
     * 
     * @param spine_positions Current robot spine positions
     * @return Inertia (com and I)
     */
    virtual Inertia calculate_inertia(const Eigen::VectorXd &spine_positions) const = 0;

    /**
     * @brief Calculates the derivative of the inertia tensor
     * 
     * @param spine_positions Current robot spine positions
     * @param spine_velocities Current robot spine velocities
     * @return I_dot
     */
    virtual Eigen::Matrix3d calculate_inertia_dot(
        const Eigen::VectorXd &spine_positions,
        const Eigen::VectorXd &spine_velocities) const = 0;

    /**
     * @brief Calculates both the inertia and derivative of the inertia tensor
     * @details May be more efficient than calling the two functions above
     *          depending on implementation 
     * 
     * @param spine_positions Current robot spine positions
     * @param spine_velocities Current robot spine velocities
     * @return Pair containing inertia data and inertia_dot
     */
    virtual std::pair<Inertia, Eigen::Matrix3d> calculate_inertia_and_derivative(
        const Eigen::VectorXd &spine_positions,
        const Eigen::VectorXd &spine_velocities) const = 0;
};

class EsterInertiaCalculator : public InertiaCalculatorInterface {
public:
    /**
     * @brief Construct a new Ester Inertia Calculator object
     * 
     * @param robot An initialised model of Ester, from which inertia values,
     *              joint parameters etc. can be read
     */
    EsterInertiaCalculator(const dart::dynamics::SkeletonPtr &robot);

    Inertia calculate_inertia(const Eigen::VectorXd &spine_positions) const override;

    Eigen::Matrix3d calculate_inertia_dot(
        const Eigen::VectorXd &spine_positions,
        const Eigen::VectorXd &spine_velocities) const override;

    std::pair<Inertia, Eigen::Matrix3d> calculate_inertia_and_derivative(
        const Eigen::VectorXd &spine_positions,
        const Eigen::VectorXd &spine_velocities) const override;

    const Eigen::Matrix3d& get_rbd_com_inertia(size_t rbd_id) const { return rbd_inertias_[rbd_id]; };

    double get_mass(size_t rbd_id) const { return rbd_masses_[rbd_id]; };

    const Eigen::Vector3d& get_rbd_com_rel_jnt(size_t rbd_id) const { return spine_pos_post_joint_[rbd_id]; };

    const Eigen::Vector3d& get_rbd_jnt_rel_parent(size_t rbd_id) const { return spine_pos_pre_joint_[rbd_id]; };

protected:
    Eigen::Isometry3d calculate_rbd_transform(
        size_t rbd_id, const Eigen::VectorXd &spine_pos) const;

    std::array<Eigen::Vector3d, 3> spine_pos_pre_joint_;
    std::array<Eigen::Vector3d, 3> spine_pos_post_joint_;
    std::array<double, 3> rbd_masses_;
    std::array<Eigen::Matrix3d, 3> rbd_inertias_;
    std::array<Eigen::Vector3d, 2> yaw_pitch_offset_;
};

} // namespace ester_kinematics

#endif // __INERTIA_CALCULATOR_HPP__