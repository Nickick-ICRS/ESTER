#ifndef __SPINE_TRAJECTORY_GENERATOR_HPP__
#define __SPINE_TRAJECTORY_GENERATOR_HPP__

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "ester_common/ester_enums.hpp"

namespace ester_mpc
{

enum class SpineTrajectoryType {
    FIXED,
    STIFFNESS,
    IMPEDANCE,
    FOOT_POSITION_DEP,
    TIME_DEP,
};

struct SpineTrajectory {
    virtual SpineTrajectoryType type() const { return SpineTrajectoryType::FIXED; };
};

struct SpineTrajectoryParams {
    virtual SpineTrajectoryType type() const { return SpineTrajectoryType::FIXED; };
};

// Base class also works as a "Fixed" Spine Trajectory Generator
class SpineTrajectoryGenerator {
public:
    SpineTrajectoryGenerator() = default;
    virtual ~SpineTrajectoryGenerator() = default;

    virtual std::shared_ptr<SpineTrajectory> generate_trajectory(
        const std::shared_ptr<SpineTrajectoryParams> &params) const;
    
    void assert_safe(const std::shared_ptr<SpineTrajectoryParams> &params) const;

    virtual SpineTrajectoryType type() const { return SpineTrajectoryType::FIXED; };
    virtual size_t num_params() const { return 0; };
};

struct SpineStiffnessTrajectory : public SpineTrajectory {
    std::vector<double> stiffness;
    double damping;
    double timestep;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::STIFFNESS; };
};

struct SpineStiffnessTrajectoryParams : public SpineTrajectoryParams {
    // stiffness = A * sin(2*pi*phase + B) + C, damping = D
    double A, B, C, D;
    double phase;
    double phase_per_second;
    // in seconds
    double timestep;
    size_t prediction_window;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::STIFFNESS; };
};

// Generates a trajectory p = f(t), d = C. Spine positions set to zero
class SpineStiffnessTrajectoryGenerator : public SpineTrajectoryGenerator {
public:
    SpineStiffnessTrajectoryGenerator() = default;
    virtual ~SpineStiffnessTrajectoryGenerator() = default;

    /**
     * @param params casted to SpineStiffnessTrajectoryParams
     * @return SpineStiffnessTrajectory pointer
     */
    virtual std::shared_ptr<SpineTrajectory> generate_trajectory(
        const std::shared_ptr<SpineTrajectoryParams> &params) const override;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::STIFFNESS; };
    virtual size_t num_params() const override { return 4; };
};

struct SpineImpedanceTrajectory : public SpineTrajectory {
    std::vector<double> stiffness;
    std::vector<double> damping;
    double timestep;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::IMPEDANCE; };
};

struct SpineImpedanceTrajectoryParams : public SpineTrajectoryParams {
    // stiffness = A * sin(2*pi*phase + B) + C, damping = D * sin(2*pi*phase + E) + F
    double A, B, C, D, E, F;
    double phase;
    double phase_per_second;
    double timestep;
    size_t prediction_window;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::IMPEDANCE; };
};

class SpineImpedanceTrajectoryGenerator : public SpineTrajectoryGenerator {
public:
    SpineImpedanceTrajectoryGenerator() = default;
    virtual ~SpineImpedanceTrajectoryGenerator() = default;

    /**
     * @param params casted to SpineImpedanceTrajectoryParams
     * @return SpineImpedanceTrajectory pointer
     */
    // Returns SpineImpedanceTrajectory, takes SpineStiffnessTrajectoryParams
    virtual std::shared_ptr<SpineTrajectory> generate_trajectory(
        const std::shared_ptr<SpineTrajectoryParams> &params) const override;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::IMPEDANCE; };
    virtual size_t num_params() const override { return 6; };
};

struct SpineFootPosDependantTrajectory : public SpineTrajectory {
    std::vector<double> theta;
    double stiffness;
    double damping;
    double timestep;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::FOOT_POSITION_DEP; };
};

struct SpineFootPosDependantTrajectoryParams : public SpineTrajectoryParams {
    // theta = f(foot positions, A), stiffness = B, damping = C
    double A, B, C;
    // relative to chassis
    std::vector<ester_common::LegMap<Eigen::Vector3d>> foot_pos;
    ester_common::SpineJointId spine_jnt_id;
    double timestep;
    size_t prediction_window;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::FOOT_POSITION_DEP; };
};

class SpineFootPosDependantTrajectoryGenerator : public SpineTrajectoryGenerator {
public:
    SpineFootPosDependantTrajectoryGenerator() = default;
    virtual ~SpineFootPosDependantTrajectoryGenerator() = default;

    /**
     * @param params casted to SpineFootPosDependantTrajectoryParams
     * @return SpineFootPosDependantTrajectory pointer
     */
    virtual std::shared_ptr<SpineTrajectory> generate_trajectory(
        const std::shared_ptr<SpineTrajectoryParams> &params) const override;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::FOOT_POSITION_DEP; };
    virtual size_t num_params() const override { return 3; };
};

struct SpineTimeDependantTrajectory : public SpineTrajectory {
    std::vector<double> theta;
    double stiffness;
    double damping;
    double timestep;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::TIME_DEP; };
};

struct SpineTimeDependantTrajectoryParams : public SpineTrajectoryParams {
    // theta = A * sin(2*pi*phase + B), stiffness = C, damping = D
    double A, B, C, D;
    double phase;
    double phase_per_second;
    double timestep;
    double freq = 1;
    size_t prediction_window;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::TIME_DEP; };
};

class SpineTimeDependantTrajectoryGenerator : public SpineTrajectoryGenerator {
public:
    SpineTimeDependantTrajectoryGenerator() = default;
    virtual ~SpineTimeDependantTrajectoryGenerator() = default;

    /**
     * @param params casted to SpineTimeDependantTrajectoryParams
     * @return SpineTimeDependantTrajectory pointer
     */
    virtual std::shared_ptr<SpineTrajectory> generate_trajectory(
        const std::shared_ptr<SpineTrajectoryParams> &params) const override;

    virtual SpineTrajectoryType type() const override { return SpineTrajectoryType::TIME_DEP; };
    virtual size_t num_params() const override { return 4; };
};

} // namespace ester_mpc

#endif // __SPINE_TRAJECTORY_GENERATOR_HPP__