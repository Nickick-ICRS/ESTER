#ifndef __SRBDM_MPC_HPP__
#define __SRBDM_MPC_HPP__

#include "ester_mpc/mpc_base.hpp"

#include <ester_common/ester_enums.hpp>

#include <qpSWIFT/qpSWIFT.h>

namespace ester_mpc {

class SingleRigidBodyMPC : public MPCBase {
public:
    /**
     * @brief Constructor
     */
    SingleRigidBodyMPC();

    /**
     * @brief Destructor
     */
    virtual ~SingleRigidBodyMPC() = default;

    /**
     * @brief Solves the MPC problem at the current timestep
     *
     * @param state MPCBase::State Will be cast to SingleRigidBodyMPC::State
     *
     * @returns SingleRigidBodyMPC::Solution cast as MPCBase::Solution
     */
    std::shared_ptr<MPCBase::Solution> solve(
        const std::shared_ptr<MPCBase::State> &state) override;

    struct Solution : public MPCBase::Solution {
        // Horizon, dt etc. is inherited

        virtual ~Solution() {};

        /**
         * Foot force for each timestep
         */
        std::vector<ester_common::LegMap<Eigen::Vector3d>>
            foot_grfs;

        /**
         * @brief Predicted position for next k timesteps
         */
        std::vector<Eigen::Vector3d> pred_r;

        /**
         * @brief Predicted rotation for next k timesteps
         */
        std::vector<Eigen::Matrix3d> pred_R;

        /**
         * @brief Predicted angular velocity for next k timesteps
         *        body frame
         */
        std::vector<Eigen::Vector3d> pred_w;

        /**
         * @brief Predicted linear velocity for next k timesteps
         */
        std::vector<Eigen::Vector3d> pred_v;
    };

    struct State : public MPCBase::State {
        // Current state etc. is inherited

        virtual ~State() {};

        /**
         * @brief Current foot positions
         */
        ester_common::LegMap<Eigen::Vector3d> foot_positions;

        /**
         * @brief Current stance (true) and swing (false) states and future
         *        HORIZON stance states
         */
        std::vector<ester_common::LegMap<bool>> stance_state;

        /**
         * @brief Desired position for the next HORIZON steps
         */
        std::vector<Eigen::Vector3d> r_d;

        /**
         * @brief Desired orientation for the next HORIZON steps
         */
        std::vector<Eigen::Matrix3d> R_d;

        /**
         * @brief Desired linear velocity for the next HORIZON steps
         */
        std::vector<Eigen::Vector3d> v_d;

        /**
         * @brief Desired angular velocity for the next HORIZON steps
         * @brief body frame
         */
        std::vector<Eigen::Vector3d> w_d;

        /**
         * @brief Desired foot forces for the next HORIZON steps
         */
        std::vector<Eigen::Matrix<double, 12, 1>> u_d;

        /**
         * @brief Current foot forces
         *
         * @details FL: 0-2, FR: 3-5, RL: 6-8, RR: 9-11
         */
        Eigen::Matrix<double, 12, 1> current_foot_forces;

        /**
         * @brief Foot friction cone mu
         */
        double mu;

        /**
         * @brief Weight matrix for cost on x
         *
         * @details r, v, R, w
         */
        Eigen::DiagonalMatrix<double, 12> Q_x;

        /**
         * @brief Weight matrix for cost on x at final timestep
         */
        Eigen::DiagonalMatrix<double, 12> Qf_x;

        /**
         * @brief Weight matrix for cost on u
         */
        Eigen::DiagonalMatrix<double, 12> Q_u;
    };

private:
    /**
     * @brief Extract qp results into a solution variable
     *
     * @param the_qp The qp to extract results from
     * @param solu The solution to extract results into
     * @param state The current state of the robot
     */
    void extract_qp_results(
        QP *the_qp, const std::shared_ptr<Solution> &solu,
        const std::shared_ptr<State> &state);

    /**
     * @brief Prepare the qp for optimisation
     *
     * @param state Current state and MPC parameters
     *
     * @returns The QP ready to be QP_SOLVE'd
     */
    QP* prepare_qp(const std::shared_ptr<State> &state);

    /**
     * @brief Calculate equality constants for x and v
     *
     * @param Cx_x Apply to position (for position)
     * @param Cx_v Apply to velocity (for position)
     * @param Cv_v Apply to velocity (for velocity)
     * @param Cv_u Apply to forces (for velocity)
     * @param Cv_c Constant (for velocity)
     * @param u_op Foot forces at operating point
     * @param dt MPC time step
     * @param mass Robot mass
     * @param g Gravity vector
     */
    void calculate_co_xv(
        Eigen::Matrix3d &Cx_x, Eigen::Matrix3d &Cx_v,
        Eigen::Matrix3d &Cv_v, Eigen::Matrix<double, 3, 12> &Cv_u,
        Eigen::Vector3d &Cv_c,
        const Eigen::Matrix<double, 12, 1> &u_op,
        double dt, double mass, Eigen::Vector3d g);

    /**
     * @brief Calculate equality constants for eta
     *
     * @param C_c Plain constant
     * @param C_eta Apply to R3 representation of lie algebra of dR
     * @param C_w Apply to angular velocity
     * @param R The rotation matrix at the operating point
     * @param w The angular velocity at the operating point
     * @param dt MPC time step
     */
    void calculate_co_eta(
        Eigen::Vector3d &C_c, Eigen::Matrix3d &C_eta, Eigen::Matrix3d &C_w,
        const Eigen::Matrix3d &R, const Eigen::Vector3d &w, double dt);

    /**
     * @brief Calculate equality constants for w
     *
     * @param C_c Plain constant
     * @param C_dr Apply to body position r
     * @param C_eta Apply to R3 representation of lie algebra of dR
     * @param C_w Apply to angular velocity
     * @param C_du Apply to variation of control inputs
     *
     * @param r Position of the com at operating point
     * @param R Rotation of the com at operating point
     * @param w Angular velocity of com at operating point
     * @param u Foot forces at operating point (vectorised)
     * @param foot_pos Foot positions at operating point
     * @param I Inertia matrix
     * @param I_inv Inverse of inertia matrix
     * @param dt MPC time step
     */
    void calculate_co_w(
        Eigen::Vector3d &C_c, Eigen::Matrix3d &C_dr, Eigen::Matrix3d &C_eta,
        Eigen::Matrix3d &C_w, Eigen::Matrix<double, 3, 12> &C_du,
        const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
        const Eigen::Vector3d &w, const Eigen::Matrix<double, 12, 1> &u,
        const std::array<Eigen::Vector3d, 4> &foot_pos,
        const Eigen::Matrix3d &I, const Eigen::Matrix3d &I_inv,
        double dt);

    /**
     * @brief Calculate cost function constants for timestep k
     *
     * @param C_x Error matrix for x_k
     * @param C_u Error matrix for u_k
     * @param Q_x Weight matrix for x
     * @param Q_u Weight matrix for u
     * @param R_op SO3 orientation at the operating point
     * @param r_dk Desired position at timestep k
     * @param R_dk Desired SO3 orientation at timestep k
     * @param v_dk Desired velocity at timestep k
     * @param w_dk Desired angular velocity at timestep k
     */
    void calculate_co_err(
        Eigen::Matrix<double, 12, 1> &C_x,
        Eigen::Matrix<double, 12, 1> &C_u,
        const Eigen::DiagonalMatrix<double, 12> &Q_x,
        const Eigen::DiagonalMatrix<double, 12> &Q_u,
        const Eigen::Matrix3d &R_op,
        const Eigen::Vector3d &r_dk, const Eigen::Matrix3d &R_dk,
        const Eigen::Vector3d &v_dk, const Eigen::Vector3d &w_dk);

    /**
     * @brief Constructs equality and inequality matrices for qpSWIFT
     *
     * @details A_ineq * z <= b_ineq; A_eq * z = b_eq; z = [u0, x1, ..., xn]
     *          For other parameters see the various calculate_co_XX funcs
     * 
     * @param A_ineq A matrix for inequality function
     * @param b_ineq b vector for inequality function
     * @param A_eq A matrix for equality function
     * @param b_eq b matrix for equality function
     */
    void construct_eq_ineq_matrices(
        Eigen::MatrixXd &A_ineq, Eigen::VectorXd &b_ineq,
        Eigen::MatrixXd &A_eq, Eigen::VectorXd &b_eq,
        unsigned int horizon, double dt, double mu, double mass,
        const Eigen::Vector3d &g,
        const ester_common::LegMap<Eigen::Vector3d> &foot_pos,
        const std::vector<ester_common::LegMap<bool>> &stance_map,
        const Eigen::Vector3d &r_op, const Eigen::Matrix3d &R_op,
        const Eigen::Vector3d &v_op, const Eigen::Vector3d &w_op,
        const Eigen::Matrix<double, 12, 1> &u_op,
        const std::vector<Eigen::Matrix<double, 12, 1>> &u_d,
        const Eigen::Matrix3d &I, const Eigen::Matrix3d &I_inv);

    /**
     * @brief Matrix to vectorize an R3 lie algebra
     *
     * @details Note that Eigen < 3.4 does not let us brace-initialise this,
     *          so it can't be const
     */
    Eigen::Matrix<double, 9, 3> N;
    Eigen::Matrix<double, 3, 9> invN;

    /**
     * @brief QP variable
     */
    QP *the_qp;

    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;
    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;
    Eigen::Matrix<qp_real, Eigen::Dynamic, Eigen::Dynamic> P;
    Eigen::Matrix<qp_real, Eigen::Dynamic, 1> c;
};

} // ns ester_mpc

#endif // __SRBDM_MPC_HPP__
