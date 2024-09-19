#ifndef __MPC_BASE_HPP__
#define __MPC_BASE_HPP__

#include <chrono>
#include <map>
#include <memory>

#include <Eigen/Geometry>

namespace ester_mpc {

inline void orthogonalise(Eigen::Matrix3d &rot) {
    Eigen::Quaterniond q(rot);
    rot = q.normalized().toRotationMatrix();
}

inline void orthogonalise(Eigen::Isometry3d &T) {
    Eigen::Matrix3d R = T.linear();
    orthogonalise(R);
    T.linear() = R;
}

class MPCBase {
public:
    /**
     * @brief Constructor
     */
    MPCBase();

    /**
     * @brief Destructor
     */
    virtual ~MPCBase();

    /**
     * @brief MPC Solution
     *
     * @details Inherit this with whatever relevant parameters
     */
    struct Solution {
        virtual ~Solution() {};

        /**
         * @brief The time horizon solved for (i.e. num steps)
         */
        unsigned int horizon;

        /**
         * @brief Time duration of each step
         */
        double dt;

        /**
         * @brief Time taken to solve the optimisation
         */
        std::chrono::duration<double> solve_time;
    };

    /**
     * @brief MPC current state
     *
     * @details Inherit this with whatever relevant parameters
     */
    struct State {
        virtual ~State() {};

        /**
         * @brief Time horizon, i.e. num steps
         */
        unsigned int horizon;

        /**
         * @brief Requested update time step
         */
        double dt;

        /**
         * @brief Maximum acceleration rate
         */
        double acc_limit;

        /**
         * @brief Discount future penalties
         */
        double decay_rate;

        /**
         * @brief Current position of the spine_center_link
         */
        Eigen::Vector3d r;

        /**
         * @brief Current orientation of the spine_center_link
         */
        Eigen::Matrix3d R;

        /**
         * @brief Current linear velocity
         */
        Eigen::Vector3d v;

        /**
         * @brief Current angular velocity
         */
        Eigen::Vector3d w;

        /**
         * @brief Current gravity vector in the robot frame
         */
        Eigen::Vector3d g;

        /**
         * @brief Robot inertia matrix
         */
        Eigen::Matrix3d I;

        /**
         * @brief Inverse of inertia matrix
         */
        Eigen::Matrix3d invI;

        /**
         * @brief Robot mass
         */
        double mass;
    };

    /**
     * @brief Solves the MPC problem at the current timestep
     *
     * @param state Current state of the robot, inc. any parameters
     *
     * @returns The MPC solution. Expected to be cast into the correct
     *          format for whichever solver.
     */
    virtual std::shared_ptr<MPCBase::Solution> solve(
        const std::shared_ptr<MPCBase::State> &state) = 0;
};

} // ns ester_mpc

#endif // __MPC_BASE_HPP__
