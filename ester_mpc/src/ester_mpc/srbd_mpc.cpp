#include "ester_mpc/srbd_mpc.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

#include <sophus/so3.hpp>

// For ROS_WARN etc
#include <ros/ros.h>

using namespace ester_mpc;
using namespace ester_common;

#define hat_map_so3(v) Sophus::SO3d::hat(v)

// Vectorize a matrix
template<int cols, int rows>
Eigen::Matrix<double, cols*rows, 1> vec(
    Eigen::Matrix<double, cols, rows> &m)
{
    return Eigen::Map<Eigen::Matrix<double, cols*rows, 1>>(
        m.data(), cols*rows);
}

SingleRigidBodyMPC::SingleRigidBodyMPC()
{
    N << 0,  0,  0,
         0,  0,  1,
         0, -1,  0,
         0,  0, -1,
         0,  0,  0,
         1,  0,  0,
         0,  1,  0,
        -1,  0,  0,
         0,  0,  0;
    invN = (N.transpose() * N).inverse() * N.transpose();
}

std::shared_ptr<MPCBase::Solution> SingleRigidBodyMPC::solve(
    const std::shared_ptr<MPCBase::State> &state)
{
    auto start = std::chrono::steady_clock::now();

    std::shared_ptr<Solution> solu(new Solution);
    const std::shared_ptr<State> m_state =
        std::dynamic_pointer_cast<State>(state);

    solu->horizon = m_state->horizon;
    solu->dt = m_state->dt;

    auto the_qp = prepare_qp(m_state);
    auto qp_res = QP_SOLVE(the_qp);
    if (qp_res == QP_OPTIMAL) {
        // Optimal solution found
        extract_qp_results(the_qp, solu, m_state);
    }
    else if (qp_res == QP_MAXIT) {
        // Max iterations exceeded
        //ROS_WARN("[srbd MPC]: Maximum iterations exceeded!");
        extract_qp_results(the_qp, solu, m_state);
    }
    else if (qp_res == QP_KKTFAIL) {
        // Failed to solve LDL factorization
        ROS_ERROR("[srbd MPC]: Failed to solve LDL factorization!");
    }
    else /*if (qp_res == QP_FATAL)*/ {
        // Unknown error
        ROS_ERROR("[srbd MPC]: Unknown error!");
    }

    QP_CLEANUP_dense(the_qp);
    auto end = std::chrono::steady_clock::now();
    solu->solve_time = end - start;

    return std::static_pointer_cast<MPCBase::Solution>(solu);
}

void SingleRigidBodyMPC::extract_qp_results(
    QP *the_qp, const std::shared_ptr<Solution> &solu,
    const std::shared_ptr<State> &state)
{
    // 12 u + 12 x
    const unsigned int num_z = 24;
    // u first
    const unsigned int offset = 0;
    solu->foot_grfs.resize(solu->horizon);
    solu->pred_r.resize(solu->horizon);
    solu->pred_v.resize(solu->horizon);
    solu->pred_R.resize(solu->horizon);
    solu->pred_w.resize(solu->horizon);

    const auto &u_op = state->current_foot_forces;

    // qp solves for the *delta* u w.r.t current operating forces, so
    // need to add operating foot forces to the solution
    for (unsigned int i = 0; i < solu->horizon; i++) {
        solu->foot_grfs[i][LegId::FL][0] =
            the_qp->x[num_z*i+0+offset] + u_op[0];
        solu->foot_grfs[i][LegId::FL][1] =
            the_qp->x[num_z*i+1+offset] + u_op[1];
        solu->foot_grfs[i][LegId::FL][2] =
            the_qp->x[num_z*i+2+offset] + u_op[2];
        solu->foot_grfs[i][LegId::FR][0] =
            the_qp->x[num_z*i+3+offset] + u_op[3];
        solu->foot_grfs[i][LegId::FR][1] =
            the_qp->x[num_z*i+4+offset] + u_op[4];
        solu->foot_grfs[i][LegId::FR][2] =
            the_qp->x[num_z*i+5+offset] + u_op[5];
        solu->foot_grfs[i][LegId::RL][0] =
            the_qp->x[num_z*i+6+offset] + u_op[6];
        solu->foot_grfs[i][LegId::RL][1] =
            the_qp->x[num_z*i+7+offset] + u_op[7];
        solu->foot_grfs[i][LegId::RL][2] =
            the_qp->x[num_z*i+8+offset] + u_op[8];
        solu->foot_grfs[i][LegId::RR][0] =
            the_qp->x[num_z*i+9+offset] + u_op[9];
        solu->foot_grfs[i][LegId::RR][1] =
            the_qp->x[num_z*i+10+offset] + u_op[10];
        solu->foot_grfs[i][LegId::RR][2] =
            the_qp->x[num_z*i+11+offset] + u_op[11];
    
        solu->pred_r[i].x() = state->r.x() + the_qp->x[num_z*i+12+offset];
        solu->pred_r[i].y() = state->r.y() + the_qp->x[num_z*i+13+offset];
        solu->pred_r[i].z() = state->r.z() + the_qp->x[num_z*i+14+offset];

        solu->pred_v[i].x() = state->v.x() + the_qp->x[num_z*i+15+offset];
        solu->pred_v[i].y() = state->v.y() + the_qp->x[num_z*i+16+offset];
        solu->pred_v[i].z() = state->v.z() + the_qp->x[num_z*i+17+offset];

        Eigen::Vector3d zeta;
        zeta.x() = the_qp->x[num_z*i+18+offset];
        zeta.y() = the_qp->x[num_z*i+19+offset];
        zeta.z() = the_qp->x[num_z*i+20+offset];
        solu->pred_R[i] = state->R * Sophus::SO3d::exp(zeta).matrix();

        solu->pred_w[i].x() = state->w.x() + the_qp->x[num_z*i+21+offset];
        solu->pred_w[i].y() = state->w.y() + the_qp->x[num_z*i+22+offset];
        solu->pred_w[i].z() = state->w.z() + the_qp->x[num_z*i+23+offset];
    }
}

QP* SingleRigidBodyMPC::prepare_qp(const std::shared_ptr<State> &state) {
    const unsigned int n_x = 12;
    const unsigned int n_u = 12;
    const unsigned int n_z = n_x + n_u;

    construct_eq_ineq_matrices(
        A_ineq, b_ineq, A_eq, b_eq,
        state->horizon, state->dt, state->mu, state->mass,
        state->g, state->foot_positions, state->stance_state,
        state->r, state->R, state->v, state->w, state->current_foot_forces,
        state->u_d, state->I, state->invI);

    // num variables
    qp_int n = state->horizon * n_z;
    // num inequality constraints
    qp_int m = A_ineq.rows();
    // num equality constraints
    qp_int p = A_eq.rows();

    P.resize(n, n);
    c.resize(n);

    P.setZero();
    c.setZero();

    for (unsigned int i = 0; i < state->horizon; i++) {
        Eigen::Matrix<double, n_x, 1> C_x;
        Eigen::Matrix<double, n_u, 1> C_u;
        unsigned int pos = i * n_z;
        double decay = std::pow(state->decay_rate, i);
        if (i == state->horizon - 1) {
            calculate_co_err(
                C_x, C_u, state->Qf_x, state->Q_u, state->R,
                state->r_d[i], state->R_d[i], state->v_d[i],
                state->w_d[i]);
            P.block<n_x, n_x>(pos+n_u, pos+n_u) = state->Qf_x * decay;
            c.block<n_x, 1>(pos+n_u, 0) = C_x * decay;
        }
        else {
            calculate_co_err(
                C_x, C_u, state->Q_x, state->Q_u, state->R,
                state->r_d[i], state->R_d[i], state->v_d[i],
                state->w_d[i]);
            P.block<n_x, n_x>(pos+n_u, pos+n_u) = state->Q_x * decay;
            c.block<n_x, 1>(pos+n_u, 0) = C_x * decay;
        }
        P.block<n_u, n_u>(pos, pos) = state->Q_u * decay;
        c.block<n_u, 1>(pos, 0) =
            state->Q_u * (state->current_foot_forces - state->u_d[i])
            * decay;
    }

    // Matlab: P, c, A, b, G, h
    // Cpp: P, A, G, c, h, b
    the_qp = QP_SETUP_dense(
        n, m, p, P.data(), A_eq.data(), A_ineq.data(), c.data(),
        b_ineq.data(), b_eq.data(), NULL, COLUMN_MAJOR_ORDERING);

    the_qp->options->maxit = 50; // default 100
    the_qp->options->reltol = 1e-6; // default 1e-6
    the_qp->options->abstol = 1e-6; // default 1e-6
    //the_qp->options->SIGMA = 100; // default 100, rec not change
    //the_qp->options->VERBOSE = 1; // 1 for VERBOSE, 0 for silent

    return the_qp;
}

void SingleRigidBodyMPC::calculate_co_xv(
    Eigen::Matrix3d &Cx_x, Eigen::Matrix3d &Cx_v,
    Eigen::Matrix3d &Cv_v, Eigen::Matrix<double, 3, 12> &Cv_u,
    Eigen::Vector3d &Cv_c,
    const Eigen::Matrix<double, 12, 1> &u_op,
    double dt, double mass, Eigen::Vector3d g)
{
    Cx_x = Eigen::Matrix3d::Identity();
    Cx_v = dt * Eigen::Matrix3d::Identity();
    Cv_v = Eigen::Matrix3d::Identity();
    Cv_u << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(), 
            Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity();
    Cv_u = dt * Cv_u / mass;
    Cv_c = Cv_u * u_op + g * dt;
}

void SingleRigidBodyMPC::calculate_co_eta(
    Eigen::Vector3d &C_c, Eigen::Matrix3d &C_eta, Eigen::Matrix3d &C_w,
    const Eigen::Matrix3d &R, const Eigen::Vector3d &w, double dt)
{
    Eigen::Matrix<double, 9, 3> I_R_kron_x_N =
        Eigen::kroneckerProduct(Eigen::Matrix3d::Identity(), R) * N;
    Eigen::Matrix3d w_hat = hat_map_so3(w);
    Eigen::Matrix3d R_x_w_hat = R * w_hat;

    Eigen::Matrix<double, 3, 9> pre_mult =
        dt * invN * Eigen::kroneckerProduct(
            Eigen::Matrix3d::Identity(), R.transpose());

    C_c = pre_mult * (vec(R_x_w_hat) - I_R_kron_x_N * w);
    C_w = pre_mult * I_R_kron_x_N;
    C_eta = Eigen::Matrix3d::Identity() + pre_mult * (
        Eigen::kroneckerProduct(Eigen::Matrix3d::Identity(), R_x_w_hat)
      * N - I_R_kron_x_N * w_hat);
}

void SingleRigidBodyMPC::calculate_co_w(
    Eigen::Vector3d &C_c, Eigen::Matrix3d &C_r, Eigen::Matrix3d &C_eta,
    Eigen::Matrix3d &C_w, Eigen::Matrix<double, 3, 12> &C_u,
    const Eigen::Vector3d &r, const Eigen::Matrix3d &R,
    const Eigen::Vector3d &w, const Eigen::Matrix<double, 12, 1> &u,
    const std::array<Eigen::Vector3d, 4> &foot_pos,
    const Eigen::Matrix3d &I, const Eigen::Matrix3d &I_inv,
    double dt)
{
    Eigen::Matrix3d w_hat = hat_map_so3(w);
    Eigen::Vector3d I_x_w = I * w;
    Eigen::Matrix3d I_x_w__hat_sub_w_hat_x_I = hat_map_so3(I_x_w) - w_hat * I;
    Eigen::Matrix<double, 3, 12> foot_pos_hat;
    Eigen::Matrix<double, 3, 12> sum_u_helper;
    for (unsigned int i = 0; i < 4; i++) {
        foot_pos_hat.block<3, 3>(0, 3 * i) = hat_map_so3(foot_pos.at(i) - r);
        sum_u_helper.block<3, 3>(0, 3 * i) = Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d tau = foot_pos_hat * u;
    Eigen::Vector3d R_t_x_tau = R.transpose() * tau;
    Eigen::Vector3d sum_u = sum_u_helper * u;
    Eigen::Matrix3d sum_u_hat = hat_map_so3(sum_u);

    Eigen::Matrix3d I_R_t_x_tau_t_kron_x_n =
        Eigen::kroneckerProduct(Eigen::Matrix3d::Identity(), R_t_x_tau.transpose()) * N;

    C_r = dt * I_inv * (R.transpose() * sum_u_hat);
    C_eta = dt * I_inv * (I_R_t_x_tau_t_kron_x_n - I_x_w__hat_sub_w_hat_x_I * w_hat);
    C_w = dt * I_inv * I_x_w__hat_sub_w_hat_x_I
        + Eigen::Matrix3d::Identity();
    C_u = dt * I_inv * (R.transpose() * foot_pos_hat);
    C_c = dt * I_inv * (-w_hat * I * w + R_t_x_tau
          - I_x_w__hat_sub_w_hat_x_I * w - R.transpose() * sum_u_hat * r);
}

void SingleRigidBodyMPC::calculate_co_err(
    Eigen::Matrix<double, 12, 1> &C_x,
    Eigen::Matrix<double, 12, 1> &C_u,
    const Eigen::DiagonalMatrix<double, 12> &Q_x,
    const Eigen::DiagonalMatrix<double, 12> &Q_u,
    const Eigen::Matrix3d &R_op,
    const Eigen::Vector3d &r_dk, const Eigen::Matrix3d &R_dk,
    const Eigen::Vector3d &v_dk, const Eigen::Vector3d &w_dk)
{
    Eigen::Matrix<double, 12, 1> X;
    Eigen::Matrix3d eR = R_dk.transpose() * R_op;
    Sophus::SO3d so3_eR = Sophus::SO3d::fitToSO3(eR);
    // Note that sophus log computes vee(log(SO3))
    X << -r_dk, -v_dk, so3_eR.log(), -w_dk;
    C_x = Q_x * X;
    C_u = Q_u.diagonal();
}

void SingleRigidBodyMPC::construct_eq_ineq_matrices(
    Eigen::MatrixXd &A_ineq, Eigen::VectorXd &b_ineq,
    Eigen::MatrixXd &A_eq, Eigen::VectorXd &b_eq,
    unsigned int horizon, double dt, double mu, double mass,
    const Eigen::Vector3d &g,
    const LegMap<Eigen::Vector3d> &foot_pos,
    const std::vector<LegMap<bool>> &stance_map,
    const Eigen::Vector3d &r_op, const Eigen::Matrix3d &R_op,
    const Eigen::Vector3d &v_op,const Eigen::Vector3d &w_op,
    const Eigen::Matrix<double, 12, 1> &u_op,
    const std::vector<Eigen::Matrix<double, 12, 1>> &u_d,
    const Eigen::Matrix3d &I, const Eigen::Matrix3d &I_inv)
{
    const unsigned int n_u = 12;
    const unsigned int n_x = 12;
    const unsigned int n_z = n_u + n_x;
    // We're doing u0, x1, u1, x2, ...
    unsigned int n_eq = n_x * horizon;
    // We have upper, lower for each u
    const unsigned int n_ineq = 6;
    A_ineq.resize(4 * n_ineq * horizon, n_z * horizon); A_ineq.setZero();
    b_ineq.resize(4 * n_ineq * horizon); b_ineq.setZero();
    A_eq.resize(n_eq, n_z * horizon); A_eq.setZero();
    b_eq.resize(n_eq); b_eq.setZero();

    // Equality constraints for x and v
    Eigen::Matrix3d Cx_x;
    Eigen::Matrix3d Cx_v;
    Eigen::Matrix3d Cv_v;
    Eigen::Matrix<double, 3, 12> Cv_u;
    Eigen::Vector3d Cv_c;
    calculate_co_xv(Cx_x, Cx_v, Cv_v, Cv_u, Cv_c, u_op, dt, mass, g);

    // Equality constraints for eta
    Eigen::Vector3d Ceta_c;
    Eigen::Matrix3d Ceta_eta;
    Eigen::Matrix3d Ceta_w;
    calculate_co_eta(Ceta_c, Ceta_eta, Ceta_w, R_op, w_op, dt);

    // Equality constraints for w
    Eigen::Vector3d Cw_c;
    Eigen::Matrix3d Cw_x;
    Eigen::Matrix3d Cw_eta;
    Eigen::Matrix3d Cw_w;
    Eigen::Matrix<double, 3, 12> Cw_u;

    std::array<Eigen::Vector3d, 4> fp;
    for (unsigned int j = 0; j < 4; j++) {
        const auto &id = ALL_LEG_IDS[j];
        fp[j] = foot_pos.at(id);
    }
    calculate_co_w(
        Cw_c, Cw_x, Cw_eta, Cw_w, Cw_u,
        r_op, R_op, w_op, u_op, fp, I, I_inv, dt);

    Eigen::Matrix<double, 12, 1> q_op;
    q_op << r_op, v_op, Eigen::Vector3d::Zero(), w_op;

    Eigen::Matrix<double, n_ineq, 3> ineq_unit;
    ineq_unit
        << 1, 0, -mu, -1,  0, -mu,
           0, 1, -mu,  0, -1, -mu,
           0, 0,   1,  0,  0,  -1;

    std::vector<LegMap<double>> lb, ub;
    for (unsigned int i = 0; i < u_d.size(); i++) {
        lb.emplace_back();
        ub.emplace_back();
        lb.back()[LegId::FL] = -1 * u_d[i](2);
        ub.back()[LegId::FL] =  2 * u_d[i](2);
        lb.back()[LegId::FR] = -1 * u_d[i](5);
        ub.back()[LegId::FR] =  2 * u_d[i](5);
        lb.back()[LegId::RL] = -1 * u_d[i](8);
        ub.back()[LegId::RL] =  2 * u_d[i](8);
        lb.back()[LegId::RR] = -1 * u_d[i](11);
        ub.back()[LegId::RR] =  2 * u_d[i](11);
    }

    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 12> B;
    Eigen::Matrix<double, 12, 1> D;
    // r, v, R, w
    A << Cx_x, Cx_v, Eigen::Matrix<double, 3, 6>::Zero(),
            Eigen::Matrix3d::Zero(), Cv_v, Eigen::Matrix<double, 3, 6>::Zero(),
            Eigen::Matrix<double, 3, 6>::Zero(), Ceta_eta, Ceta_w,
            Cw_x, Eigen::Matrix3d::Zero(), Cw_eta, Cw_w;
    B << Eigen::Matrix<double, 3, 12>::Zero(),
            Cv_u,
            Eigen::Matrix<double, 3, 12>::Zero(),
            Cw_u;
    D << Eigen::Vector3d::Zero(),
            Cv_c,
            Ceta_c,
            Cw_c;

    for (unsigned int i = 0; i < horizon; i++) {
        // Fill inequality matrix
        Eigen::Matrix<double, 4 * n_ineq, n_u> Fi; Fi.setZero();
        Eigen::Matrix<double, 4 * n_ineq, 1> hi; hi.setZero();
        for (unsigned int j = 0; j < 4; j++) {
            unsigned int idx_F = j * n_ineq;
            unsigned int idx_u = j * 3;

            double z_max = ub[i][ALL_LEG_IDS[j]];
            double z_min = lb[i][ALL_LEG_IDS[j]];
            Eigen::Vector3d upper_lim, lower_lim;
            lower_lim << 
                mu * u_op(idx_u+2, 0) + u_op(idx_u, 0),
                mu * u_op(idx_u+2, 0) + u_op(idx_u+1, 0),
               -z_min + u_op(idx_u+2, 0) - u_d[i](idx_u+2, 0);
            upper_lim <<
                mu * u_op(idx_u+2, 0) - u_op(idx_u, 0),
                mu * u_op(idx_u+2, 0) - u_op(idx_u+1, 0),
                z_max - u_op(idx_u+2, 0) + u_d[i](idx_u+2, 0),

            // Compiler doing weird stuff again?
            (void)ineq_unit;
            Fi.block<n_ineq, 3>(idx_F, idx_u) = ineq_unit;
            hi.block<n_ineq, 1>(idx_F, 0) <<
                upper_lim(0), lower_lim(0),
                upper_lim(1), lower_lim(1),
                upper_lim(2), lower_lim(2);
        }
        unsigned int idx_A = i * 4 * n_ineq;
        unsigned int idx_z = i * n_z;
        A_ineq.block<4*n_ineq, n_u>(idx_A, idx_z) = Fi;
        b_ineq.segment<4*n_ineq>(idx_A) = hi;

        // Fill equality matrix
        unsigned int col = i * (n_x + n_u);
        unsigned int row = i * n_x;
        // Equations of form r_k-1 + ...k-1 - r_k = X
        // This is r_k
        A_eq.block<n_x, n_x>(row, col+n_u) =
            -Eigen::Matrix<double, n_x, n_x>::Identity();
        if (i == 0) {
            A_eq.block<n_x, n_u>(row, col) = -B;
            A_eq.block<n_x, n_x>(row, col+n_u) =
                Eigen::Matrix<double, n_x, n_x>::Identity();
            b_eq.segment<n_x>(row) = A * q_op + D;
        }
        else {
            A_eq.block<n_x, n_x>(row, col-n_x) = -A;
            A_eq.block<n_x, n_u>(row, col) = -B;
            A_eq.block<n_x, n_x>(row, col+n_u) =
                Eigen::Matrix<double, n_x, n_x>::Identity();
            b_eq.segment<n_x>(row) = D;
        }
    }
}
