#include "quadrotor_mpc/mhe/mhe_trans_controller.hpp"

#include "acados_solver_quadrotor_mhe_trans.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstdio>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace quadrotor_mpc {

MheTransController::MheTransController() {
    x_bar_.setZero();
    x_bar_(6) = 1.05;   // nominal mass
    P_bar_inv_.setOnes();
    P_bar_diag_.setConstant(1.0);
}
MheTransController::~MheTransController() { free(); }

bool MheTransController::init() {
    capsule_ = quadrotor_mhe_trans_acados_create_capsule();
    int s = quadrotor_mhe_trans_acados_create_with_discretization(capsule_, N, nullptr);
    if (s != 0) { std::fprintf(stderr, "[MheTrans] create failed %d\n", s); return false; }
    return true;
}

void MheTransController::reset(const MheTransEstimate& x0, const Eigen::Matrix<double,10,1>& Pd) {
    x_bar_ = x0.to_vector();
    P_bar_diag_ = Pd;
    for (int i = 0; i < 10; ++i) P_bar_inv_(i) = 1.0 / std::max(Pd(i), 1e-12);
    sigma_k_ = P_bar_diag_(6) + P_bar_diag_(7) + P_bar_diag_(8) + P_bar_diag_(9);

    Eigen::Matrix<double,6,1> y0; y0 << x0.pos, x0.vel;
    y_win_.clear(); T_win_.clear(); a_win_.clear(); sf_win_.clear(); mask_win_.clear();
    for (int k = 0; k <= N; ++k) { y_win_.push_back(y0); mask_win_.push_back(1.0); }
    for (int k = 0; k <  N; ++k) { T_win_.push_back(x0.m_hat*9.81); a_win_.push_back(Vec3(0,0,1)); sf_win_.push_back(Vec3(0,0,9.81)); }

    ocp_nlp_config* c = quadrotor_mhe_trans_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   d = quadrotor_mhe_trans_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    o = quadrotor_mhe_trans_acados_get_nlp_out(capsule_);
    ocp_nlp_in*     in = quadrotor_mhe_trans_acados_get_nlp_in(capsule_);
    double xv[NX]; for (int i = 0; i < NX; ++i) xv[i] = x_bar_(i);
    for (int k = 0; k <= N; ++k) ocp_nlp_out_set(c, d, o, in, k, "x", xv);
    double w0[NU] = {0,0,0,0};
    for (int k = 0; k <  N; ++k) ocp_nlp_out_set(c, d, o, in, k, "u", w0);
}

void MheTransController::push(const Eigen::Matrix<double,6,1>& y_k, double T, const Vec3& a,
                              const Vec3& sf_meas, double meas_mask) {
    y_win_.push_back(y_k); T_win_.push_back(T); a_win_.push_back(a); sf_win_.push_back(sf_meas);
    mask_win_.push_back(meas_mask);
    if ((int)y_win_.size() > N + 1) { y_win_.pop_front(); mask_win_.pop_front(); }
    if ((int)T_win_.size() > N)     { T_win_.pop_front(); a_win_.pop_front(); sf_win_.pop_front(); }
}

void MheTransController::set_stage_params_(int k) {
    double p[NP] = {};
    const auto& y = (k < (int)y_win_.size()) ? y_win_[k] : y_win_.back();
    for (int i = 0; i < 6; ++i) p[i] = y(i);
    double T = (k < (int)T_win_.size()) ? T_win_[k] : T_win_.back();
    Vec3   a = (k < (int)a_win_.size()) ? a_win_[k] : a_win_.back();
    p[6] = T; p[7] = a(0); p[8] = a(1); p[9] = a(2);
    for (int i = 0; i < 10; ++i) p[10 + i] = x_bar_(i);
    for (int i = 0; i < 10; ++i) p[20 + i] = P_bar_inv_(i);
    p[30] = drag_c_;
    Vec3 sf = (k < (int)sf_win_.size()) ? sf_win_[k] : (sf_win_.empty() ? Vec3(0,0,9.81) : sf_win_.back());
    p[31] = sf(0); p[32] = sf(1); p[33] = sf(2);
    p[34] = (k < (int)mask_win_.size()) ? mask_win_[k] : (mask_win_.empty() ? 1.0 : mask_win_.back());
    quadrotor_mhe_trans_acados_update_params(capsule_, k, p, NP);
}

int MheTransController::solve() {
    for (int k = 0; k <= N; ++k) set_stage_params_(k);
    auto t = std::chrono::steady_clock::now();
    int s = quadrotor_mhe_trans_acados_solve(capsule_);
    solve_time_s_ = std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();
    return s;
}

MheTransEstimate MheTransController::get_estimate() const {
    ocp_nlp_config* c = quadrotor_mhe_trans_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   d = quadrotor_mhe_trans_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    o = quadrotor_mhe_trans_acados_get_nlp_out(capsule_);
    double x[NX]; ocp_nlp_out_get(c, d, o, N, "x", x);
    return MheTransEstimate::from_vector(Eigen::Map<State10>(x));
}

void MheTransController::propagate_prior(bool ok) {
    if (ok) {
        ocp_nlp_config* c = quadrotor_mhe_trans_acados_get_nlp_config(capsule_);
        ocp_nlp_dims*   d = quadrotor_mhe_trans_acados_get_nlp_dims(capsule_);
        ocp_nlp_out*    o = quadrotor_mhe_trans_acados_get_nlp_out(capsule_);
        double x1[NX]; ocp_nlp_out_get(c, d, o, 1, "x", x1);
        bool good = true;
        for (int i = 0; i < NX; ++i) if (!std::isfinite(x1[i])) good = false;
        if (good) for (int i = 0; i < NX; ++i) x_bar_(i) = x1[i];
    }
    for (int i = 0; i < 6; ++i) P_bar_diag_(i) = std::max(P_bar_diag_(i)*0.98, P_MIN_PHYS);
    P_bar_diag_(6) = P_bar_diag_(6)*0.98 + Q_W_M*DT;
    for (int i = 7; i < 10; ++i) P_bar_diag_(i) = P_bar_diag_(i)*0.98 + Q_W_D*DT;
    for (int i = 6; i < 10; ++i) P_bar_diag_(i) = std::max(P_bar_diag_(i), P_MIN_PARAM);
    for (int i = 0; i < 10; ++i) P_bar_inv_(i) = 1.0 / std::max(P_bar_diag_(i), 1e-12);
    sigma_k_ = P_bar_diag_(6) + P_bar_diag_(7) + P_bar_diag_(8) + P_bar_diag_(9);
}

void MheTransController::freeze_mass(double m) {
    if (!capsule_) return;
    ocp_nlp_config* c = quadrotor_mhe_trans_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   d = quadrotor_mhe_trans_acados_get_nlp_dims(capsule_);
    ocp_nlp_in*     in = quadrotor_mhe_trans_acados_get_nlp_in(capsule_);
    ocp_nlp_out*    o = quadrotor_mhe_trans_acados_get_nlp_out(capsule_);
    double lb[4] = {m, -2.0, -2.0, -2.0};
    double ub[4] = {m,  2.0,  2.0,  2.0};
    for (int k = 0; k <= N; ++k) {
        ocp_nlp_constraints_model_set(c, d, in, o, k, "lbx", lb);
        ocp_nlp_constraints_model_set(c, d, in, o, k, "ubx", ub);
    }
    x_bar_(6) = m;
}

void MheTransController::freeze_disturbance() {
    if (!capsule_) return;
    ocp_nlp_config* c = quadrotor_mhe_trans_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   d = quadrotor_mhe_trans_acados_get_nlp_dims(capsule_);
    ocp_nlp_in*     in = quadrotor_mhe_trans_acados_get_nlp_in(capsule_);
    ocp_nlp_out*    o = quadrotor_mhe_trans_acados_get_nlp_out(capsule_);
    const double eps = 1e-3;
    double lb[4] = {0.5, -eps, -eps, -eps};
    double ub[4] = {3.0,  eps,  eps,  eps};
    for (int k = 0; k <= N; ++k) {
        ocp_nlp_constraints_model_set(c, d, in, o, k, "lbx", lb);
        ocp_nlp_constraints_model_set(c, d, in, o, k, "ubx", ub);
    }
    x_bar_(7) = x_bar_(8) = x_bar_(9) = 0.0;
}

void MheTransController::free() {
    if (capsule_) {
        quadrotor_mhe_trans_acados_free(capsule_);
        quadrotor_mhe_trans_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

}  // namespace quadrotor_mpc
