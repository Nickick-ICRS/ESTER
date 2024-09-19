#include "ester_mpc/spine_trajectory_generator.hpp"

using namespace ester_common;

namespace ester_mpc
{


std::shared_ptr<SpineTrajectory> SpineTrajectoryGenerator::generate_trajectory(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert_safe(params);
    return std::make_shared<SpineTrajectory>();
}
    
void SpineTrajectoryGenerator::assert_safe(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert(params && params->type() == type());
}

std::shared_ptr<SpineTrajectory> SpineStiffnessTrajectoryGenerator::generate_trajectory(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert_safe(params);

    auto trajectory = std::make_shared<SpineStiffnessTrajectory>();
    auto m_params = std::static_pointer_cast<SpineStiffnessTrajectoryParams>(params);

    auto f = [&m_params](double phase) {
        return m_params->A * sin(2 * M_PI * phase + m_params->B) + m_params->C;
    };

    trajectory->stiffness.reserve(m_params->prediction_window);
    for (size_t i = 0; i < m_params->prediction_window; i++) {
        double phase = m_params->phase + m_params->phase_per_second * m_params->timestep * i;
        trajectory->stiffness.push_back(f(phase));
    }

    trajectory->damping = m_params->D;
    trajectory->timestep = m_params->timestep;
    return trajectory;
}

std::shared_ptr<SpineTrajectory> SpineImpedanceTrajectoryGenerator::generate_trajectory(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert_safe(params);

    auto trajectory = std::make_shared<SpineImpedanceTrajectory>();
    auto m_params = std::static_pointer_cast<SpineImpedanceTrajectoryParams>(params);

    auto f_p = [&m_params](double phase) {
        return m_params->A * sin(2 * M_PI * phase + m_params->B) + m_params->C;
    };
    auto f_d = [&m_params](double phase) {
        return m_params->D * sin(2 * M_PI * phase + m_params->E) + m_params->F;
    };

    trajectory->stiffness.reserve(m_params->prediction_window);
    trajectory->damping.reserve(m_params->prediction_window);
    for (size_t i = 0; i < m_params->prediction_window; i++) {
        double phase = m_params->phase + m_params->phase_per_second * m_params->timestep * i;
        trajectory->stiffness.push_back(f_p(phase));
        trajectory->damping.push_back(f_d(phase));
    }

    trajectory->timestep = m_params->timestep;

    return trajectory;
}

std::shared_ptr<SpineTrajectory> SpineFootPosDependantTrajectoryGenerator::generate_trajectory(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert_safe(params);

    auto trajectory = std::make_shared<SpineFootPosDependantTrajectory>();
    auto m_params = std::static_pointer_cast<SpineFootPosDependantTrajectoryParams>(params);

    auto foot_theta = [](const Eigen::Vector3d &foot_vector) {
        Eigen::Vector2d proj_vec = foot_vector.segment<2>(0);
        double theta = std::acos(proj_vec.dot(Eigen::Vector2d::UnitY()) / proj_vec.norm());
        if (proj_vec.x() < 0) {
            return theta;
        }
        return -theta;
    };

    trajectory->theta.reserve(m_params->prediction_window);
    for (size_t i = 0; i < m_params->prediction_window; i++) {
        LegMap<Eigen::Vector3d> &fp = m_params->foot_pos[i];
        switch(m_params->spine_jnt_id) {
            case SpineJointId::FRONT_PITCH: {
                double fx = fp.at(LegId::FL).x() > fp.at(LegId::FR).x() ? fp.at(LegId::FL).x() : fp.at(LegId::FR).x();
                trajectory->theta.push_back(fx * m_params->A);
                break;
            }
            case SpineJointId::FRONT_YAW: {
                Eigen::Vector3d front_line = fp.at(LegId::FL) - fp.at(LegId::FR);
                trajectory->theta.push_back(foot_theta(front_line) * m_params->A);
                break;
            }
            case SpineJointId::REAR_PITCH: {
                double rx = fp.at(LegId::RL).x() > fp.at(LegId::RR).x() ? fp.at(LegId::RL).x() : fp.at(LegId::RR).x();
                trajectory->theta.push_back(rx * m_params->A);
                break;
            }
            case SpineJointId::REAR_YAW: {
                Eigen::Vector3d rear_line = fp.at(LegId::RL) - fp.at(LegId::RR);
                trajectory->theta.push_back(foot_theta(rear_line) * m_params->A);
                break;
            }
        }
    }

    trajectory->stiffness = m_params->B;
    trajectory->damping = m_params->C;
    trajectory->timestep = m_params->timestep;

    return trajectory;
}

std::shared_ptr<SpineTrajectory> SpineTimeDependantTrajectoryGenerator::generate_trajectory(
    const std::shared_ptr<SpineTrajectoryParams> &params) const
{
    assert_safe(params);

    auto trajectory = std::make_shared<SpineTimeDependantTrajectory>();
    auto m_params = std::static_pointer_cast<SpineTimeDependantTrajectoryParams>(params);

    auto f = [&m_params](double phase) {
        return m_params->A * std::sin(m_params->freq * 2 * M_PI * phase + m_params->B);
    };

    trajectory->theta.reserve(m_params->prediction_window);
    for (size_t i = 0; i < m_params->prediction_window; i++) {
        double phase = m_params->phase + m_params->phase_per_second * m_params->timestep * i;
        trajectory->theta.push_back(f(phase));
    }
    trajectory->stiffness = m_params->C;
    trajectory->damping = m_params->D;
    trajectory->timestep = m_params->timestep;

    return trajectory;
}

} // namespace ester_mpc