#include "quadrotor_mpc/mhe/mhe_nmpc_controller.hpp"

#include "acados_solver_quadrotor_mhe_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstdio>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace quadrotor_mpc {

MheNmpcController::MheNmpcController() {
    x_bar_.setZero();
    x_bar_(6)  = 1.0;    // identity quaternion
    x_bar_(13) = 1.08;   // nominal mass
    x_bar_(14) = 0.03;   // nominal τ_rc
    P_bar_inv_.setOnes();
    P_bar_diag_.setConstant(1.0);
}

MheNmpcController::~MheNmpcController() { free(); }

bool MheNmpcController::init() {
    capsule_ = quadrotor_mhe_nmpc_acados_create_capsule();
    int status = quadrotor_mhe_nmpc_acados_create_with_discretization(capsule_, N, nullptr);
    if (status != 0) {
        std::fprintf(stderr, "[MheNmpcController] acados_create failed: %d\n", status);
        return false;
    }
    return true;
}

void MheNmpcController::rebuild_pinv_() {
    auto& pd = P_bar_diag_;
    Eigen::Matrix<double,18,1> p;
    for (int i = 0; i < 18; ++i) p(i) = std::max(pd(i), 1e-12);
    // 17D arrival error layout: p(3),v(3),log_q(3),ω(3),m,τ,d(3)
    P_bar_inv_(0)=1.0/p(0); P_bar_inv_(1)=1.0/p(1); P_bar_inv_(2)=1.0/p(2);   // p
    P_bar_inv_(3)=1.0/p(3); P_bar_inv_(4)=1.0/p(4); P_bar_inv_(5)=1.0/p(5);   // v
    P_bar_inv_(6)=1.0/p(6); P_bar_inv_(7)=1.0/p(6); P_bar_inv_(8)=1.0/p(6);   // q isotropic
    P_bar_inv_(9)=1.0/p(10); P_bar_inv_(10)=1.0/p(11); P_bar_inv_(11)=1.0/p(12); // ω
    P_bar_inv_(12)=1.0/p(13);   // m
    P_bar_inv_(13)=1.0/p(14);   // τ
    P_bar_inv_(14)=1.0/p(15); P_bar_inv_(15)=1.0/p(16); P_bar_inv_(16)=1.0/p(17); // d
}

void MheNmpcController::reset(const MheNmpcEstimate& x0_prior,
                              const Eigen::Matrix<double,18,1>& P_bar_diag)
{
    x_bar_      = x0_prior.to_vector();
    P_bar_diag_ = P_bar_diag;
    rebuild_pinv_();

    sigma_k_ = P_bar_diag_(13) + P_bar_diag_(14)
             + P_bar_diag_(15) + P_bar_diag_(16) + P_bar_diag_(17);

    Eigen::Matrix<double,13,1> y0;
    y0.head<3>() = x0_prior.pos; y0.segment<3>(3) = x0_prior.vel;
    y0.segment<4>(6) = x0_prior.quat; y0.tail<3>() = x0_prior.omega;

    Control4 u0 = Control4::Zero();
    u0(0) = x0_prior.m_hat * 9.81;   // hover thrust as initial applied control

    y_win_.clear(); u_win_.clear();
    for (int k = 0; k <= N; ++k) y_win_.push_back(y0);
    for (int k = 0; k <  N; ++k) u_win_.push_back(u0);

    ocp_nlp_config* cfg = quadrotor_mhe_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mhe_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mhe_nmpc_acados_get_nlp_out(capsule_);
    ocp_nlp_in*     in  = quadrotor_mhe_nmpc_acados_get_nlp_in(capsule_);

    double x0v[NX];
    for (int i = 0; i < NX; ++i) x0v[i] = x_bar_(i);
    for (int k = 0; k <= N; ++k) ocp_nlp_out_set(cfg, dim, out, in, k, "x", x0v);
    double w0[NU] = {0,0,0,0,0};
    for (int k = 0; k < N; ++k)  ocp_nlp_out_set(cfg, dim, out, in, k, "u", w0);
}

void MheNmpcController::push(const Eigen::Matrix<double,13,1>& y_k, const Control4& u_applied) {
    y_win_.push_back(y_k);
    u_win_.push_back(u_applied);
    if ((int)y_win_.size() > N + 1) y_win_.pop_front();
    if ((int)u_win_.size() > N)     u_win_.pop_front();
}

void MheNmpcController::set_stage_params_(int k) {
    double p[NP] = {};
    const auto& y = (k < (int)y_win_.size()) ? y_win_[k] : y_win_.back();
    for (int i = 0; i < 13; ++i) p[i] = y(i);
    const Control4& u = (k < (int)u_win_.size()) ? u_win_[k] : u_win_.back();
    for (int i = 0; i < 4; ++i) p[13 + i] = u(i);
    for (int i = 0; i < 18; ++i) p[17 + i] = x_bar_(i);
    for (int i = 0; i < 17; ++i) p[35 + i] = P_bar_inv_(i);
    quadrotor_mhe_nmpc_acados_update_params(capsule_, k, p, NP);
}

int MheNmpcController::solve() {
    for (int k = 0; k <= N; ++k) set_stage_params_(k);
    auto tic = std::chrono::steady_clock::now();
    int status = quadrotor_mhe_nmpc_acados_solve(capsule_);
    solve_time_s_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tic).count();
    return status;
}

MheNmpcEstimate MheNmpcController::get_estimate() const {
    ocp_nlp_config* cfg = quadrotor_mhe_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mhe_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mhe_nmpc_acados_get_nlp_out(capsule_);
    double x[NX];
    ocp_nlp_out_get(cfg, dim, out, N, "x", x);
    State18 xv = Eigen::Map<State18>(x);
    Quat4 q = xv.segment<4>(6);
    double qn = q.norm();
    if (qn > 1e-6) xv.segment<4>(6) = q / qn;
    return MheNmpcEstimate::from_vector(xv);
}

void MheNmpcController::propagate_prior(bool solver_ok) {
    if (solver_ok) {
        ocp_nlp_config* cfg = quadrotor_mhe_nmpc_acados_get_nlp_config(capsule_);
        ocp_nlp_dims*   dim = quadrotor_mhe_nmpc_acados_get_nlp_dims(capsule_);
        ocp_nlp_out*    out = quadrotor_mhe_nmpc_acados_get_nlp_out(capsule_);
        double x1[NX];
        ocp_nlp_out_get(cfg, dim, out, 1, "x", x1);
        bool ok = true;
        for (int i = 0; i < NX; ++i) if (!std::isfinite(x1[i])) { ok = false; break; }
        double dn2 = x1[15]*x1[15] + x1[16]*x1[16] + x1[17]*x1[17];
        if (dn2 > 225.0) ok = false;
        if (ok) {
            for (int i = 0; i < NX; ++i) x_bar_(i) = x1[i];
            Quat4 qb = x_bar_.segment<4>(6);
            double n = qb.norm(); if (n > 1e-6) x_bar_.segment<4>(6) = qb / n;
        }
    }
    // Physical states decay; parameters random-walk grow
    for (int i = 0; i < 13; ++i)
        P_bar_diag_(i) = std::max(P_bar_diag_(i) * 0.98, P_MIN_PHYS);
    P_bar_diag_(13) = P_bar_diag_(13) * 0.98 + Q_W_M   * DT;  // m
    P_bar_diag_(14) = P_bar_diag_(14) * 0.98 + Q_W_TAU * DT;  // τ
    for (int i = 15; i < 18; ++i)
        P_bar_diag_(i) = P_bar_diag_(i) * 0.98 + Q_W_D * DT;  // d
    for (int i = 0; i < 13; ++i)  P_bar_diag_(i) = std::max(P_bar_diag_(i), P_MIN_PHYS);
    for (int i = 13; i < 18; ++i) P_bar_diag_(i) = std::max(P_bar_diag_(i), P_MIN_PARAM);

    rebuild_pinv_();
    sigma_k_ = P_bar_diag_(13) + P_bar_diag_(14)
             + P_bar_diag_(15) + P_bar_diag_(16) + P_bar_diag_(17);
}

void MheNmpcController::free() {
    if (capsule_) {
        quadrotor_mhe_nmpc_acados_free(capsule_);
        quadrotor_mhe_nmpc_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

}  // namespace quadrotor_mpc
