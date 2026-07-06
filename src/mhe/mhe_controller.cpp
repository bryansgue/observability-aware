#include "quadrotor_mpc/mhe/mhe_controller.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"

#include "acados_solver_quadrotor_mhe.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstring>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace quadrotor_mpc {

// ─── helpers ─────────────────────────────────────────────────────────────────

static inline Quat4 quat_inv(const Quat4& q) {
    double n2 = q.squaredNorm() + 1e-12;
    return Quat4(q(0), -q(1), -q(2), -q(3)) / n2;
}

static inline Quat4 quat_mult(const Quat4& a, const Quat4& b) {
    return Quat4(
        a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3),
        a(0)*b(1) + a(1)*b(0) + a(2)*b(3) - a(3)*b(2),
        a(0)*b(2) - a(1)*b(3) + a(2)*b(0) + a(3)*b(1),
        a(0)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(0)
    );
}

static inline Vec3 quat_log_map(const Quat4& q) {
    // Hemisphere correction
    Quat4 qc = (q(0) < 0.0) ? -q : q;
    Vec3  qv = qc.tail<3>();
    double nv = qv.norm();
    double theta = std::atan2(nv, qc(0));
    if (nv > 1e-9) return (2.0 * theta / nv) * qv;
    return Vec3::Zero();
}

// ─── MheController ────────────────────────────────────────────────────────────

MheController::MheController() {
    x_bar_.setZero();
    x_bar_(6) = 1.0;     // identity quaternion
    x_bar_(16) = 1.08;   // nominal mass
    x_bar_(17) = 0.03;   // nominal tau_rc
    P_bar_inv_.setOnes();  // default: unit weights (tuned in reset())
    P_bar_diag_.setZero();
    P_bar_diag_.setConstant(1.0);
}

MheController::~MheController() { free(); }

bool MheController::init() {
    capsule_ = quadrotor_mhe_acados_create_capsule();
    int status = quadrotor_mhe_acados_create_with_discretization(capsule_, N, nullptr);
    if (status != 0) {
        std::fprintf(stderr, "[MheController] acados_create failed: %d\n", status);
        return false;
    }
    return true;
}

void MheController::reset(const MheEstimate& x0_prior,
                          const Eigen::Matrix<double,21,1>& P_bar_diag)
{
    x_bar_      = x0_prior.to_vector();
    P_bar_diag_ = P_bar_diag;

    // Build P̄_inv for 20D Lie-invariant error:
    // mapping: [p(3), v(3), q→log(3), ω(3), θ, vθ, f, m, τ, d(3)] ← 20D
    // state indices: 0:6 → 0:6 (p,v), 6:10→quaternion (3D log), 10:13→ω, 13-15→virt, 16-17→params, 18:21→d
    Eigen::Matrix<double,21,1> pd = P_bar_diag;
    // Avoid division by zero
    for (int i = 0; i < 21; ++i) pd(i) = std::max(pd(i), 1e-12);

    // 20D P̄_inv: indices 0..5=p,v; 6=q(scalar, replaces 4D with 3D log);
    //             7..9=ω; 10=θ; 11=vθ; 12=f; 13=m; 14=τ; 15..17=d (→ 18 total? )
    // Wait: p(3)+v(3)+q_log(3)+ω(3)+θ+vθ+f+m+τ+d(3) = 3+3+3+3+1+1+1+1+1+3 = 20 ✓
    // Mapping from state dim to error dim:
    // error[0:3]  ← state[0:3]   (p)      weight = P_bar_diag[0..2]
    // error[3:6]  ← state[3:6]   (v)      weight = P_bar_diag[3..5]
    // error[6:9]  ← log(q_err)   (q 4→3)  weight = P_bar_diag[6] (isotropic)
    // error[9:12] ← state[10:13] (ω)      weight = P_bar_diag[10..12]
    // error[12]   ← state[13]    (θ)      weight = P_bar_diag[13]
    // error[13]   ← state[14]    (vθ)     weight = P_bar_diag[14]
    // error[14]   ← state[15]    (f)      weight = P_bar_diag[15]
    // error[15]   ← state[16]    (m)      weight = P_bar_diag[16]
    // error[16]   ← state[17]    (τ)      weight = P_bar_diag[17]
    // error[17:20]← state[18:21] (d)      weight = P_bar_diag[18..20]
    P_bar_inv_(0)  = 1.0/pd(0);  P_bar_inv_(1) = 1.0/pd(1);  P_bar_inv_(2) = 1.0/pd(2);
    P_bar_inv_(3)  = 1.0/pd(3);  P_bar_inv_(4) = 1.0/pd(4);  P_bar_inv_(5) = 1.0/pd(5);
    P_bar_inv_(6)  = 1.0/pd(6);  P_bar_inv_(7) = 1.0/pd(6);  P_bar_inv_(8) = 1.0/pd(6); // isotropic q
    P_bar_inv_(9)  = 1.0/pd(10); P_bar_inv_(10) = 1.0/pd(11); P_bar_inv_(11) = 1.0/pd(12);
    P_bar_inv_(12) = 1.0/pd(13);
    P_bar_inv_(13) = 1.0/pd(14);
    P_bar_inv_(14) = 1.0/pd(15);
    P_bar_inv_(15) = 1.0/pd(16);
    P_bar_inv_(16) = 1.0/pd(17);
    P_bar_inv_(17) = 1.0/pd(18); P_bar_inv_(18) = 1.0/pd(19); P_bar_inv_(19) = 1.0/pd(20);

    // Compute initial sigma_k from parameter block
    sigma_k_ = P_bar_diag_(16) + P_bar_diag_(17)
               + P_bar_diag_(18) + P_bar_diag_(19) + P_bar_diag_(20);

    // Fill sliding window with prior measurements
    Eigen::Matrix<double,13,1> y0;
    y0.head<3>()     = x0_prior.pos;
    y0.segment<3>(3) = x0_prior.vel;
    y0.segment<4>(6) = x0_prior.quat;
    y0.tail<3>()     = x0_prior.omega;

    Control5 u0 = Control5::Zero();
    u0(0) = 0.0;  // Δf=0

    y_win_.clear(); u_win_.clear();
    for (int k = 0; k <= N; ++k) y_win_.push_back(y0);
    for (int k = 0; k <  N; ++k) u_win_.push_back(u0);

    // Warm-start solver with prior state
    ocp_nlp_config* cfg = quadrotor_mhe_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mhe_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mhe_acados_get_nlp_out(capsule_);
    ocp_nlp_in*     in  = quadrotor_mhe_acados_get_nlp_in(capsule_);

    double x0v[NX];
    for (int i = 0; i < NX; ++i) x0v[i] = x_bar_(i);
    for (int k = 0; k <= N; ++k)
        ocp_nlp_out_set(cfg, dim, out, in, k, "x", x0v);

    double w0[NU] = {0.0, 0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < N; ++k)
        ocp_nlp_out_set(cfg, dim, out, in, k, "u", w0);
}

void MheController::push(const Eigen::Matrix<double,13,1>& y_k,
                         const Control5& u_applied)
{
    y_win_.push_back(y_k);
    u_win_.push_back(u_applied);
    if ((int)y_win_.size() > N + 1) y_win_.pop_front();
    if ((int)u_win_.size() > N)     u_win_.pop_front();
}

void MheController::set_stage_params_(int k)
{
    double p[NP] = {};

    // y_k: measurement at stage k
    const auto& y = (k < (int)y_win_.size()) ? y_win_[k] : y_win_.back();
    for (int i = 0; i < 13; ++i) p[i] = y(i);

    // u_k: applied control at stage k (use k-th if available, else last)
    const Control5& u = (k < (int)u_win_.size()) ? u_win_[k] : u_win_.back();
    for (int i = 0; i < 5; ++i) p[13 + i] = u(i);

    // x̄: prior state (only meaningful at stage 0 for arrival cost)
    for (int i = 0; i < 21; ++i) p[18 + i] = x_bar_(i);

    // P̄_inv: arrival cost weight (20D, only meaningful at stage 0)
    for (int i = 0; i < 20; ++i) p[39 + i] = P_bar_inv_(i);

    quadrotor_mhe_acados_update_params(capsule_, k, p, NP);
}

int MheController::solve()
{
    // Set parameters for all stages
    for (int k = 0; k <= N; ++k) set_stage_params_(k);

    auto tic = std::chrono::steady_clock::now();
    int status = quadrotor_mhe_acados_solve(capsule_);
    solve_time_s_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tic).count();

    return status;
}

MheEstimate MheController::get_estimate() const
{
    ocp_nlp_config* cfg = quadrotor_mhe_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mhe_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mhe_acados_get_nlp_out(capsule_);

    double x[NX];
    ocp_nlp_out_get(cfg, dim, out, N, "x", x);

    State21 xv = Eigen::Map<State21>(x);
    // Normalize quaternion
    Quat4 q = xv.segment<4>(6);
    double qn = q.norm();
    if (qn > 1e-6) xv.segment<4>(6) = q / qn;

    return MheEstimate::from_vector(xv);
}

Eigen::Matrix<double,5,1> MheController::get_param_estimate() const
{
    MheEstimate e = get_estimate();
    Eigen::Matrix<double,5,1> theta;
    theta(0) = e.m_hat;
    theta(1) = e.tau_hat;
    theta.tail<3>() = e.d_hat;
    return theta;
}

void MheController::propagate_prior(bool solver_ok)
{
    // ── Update x_bar_ from stage 1 only when solver converged ────────────
    // If solver_ok=false (MINSTEP / NaN): skip x_bar_ update to avoid
    // corrupting the prior with garbage estimates (especially d̂ which can
    // diverge and pull future solves in the wrong direction).
    if (solver_ok) {
        ocp_nlp_config* cfg = quadrotor_mhe_acados_get_nlp_config(capsule_);
        ocp_nlp_dims*   dim = quadrotor_mhe_acados_get_nlp_dims(capsule_);
        ocp_nlp_out*    out = quadrotor_mhe_acados_get_nlp_out(capsule_);

        double x1[NX];
        ocp_nlp_out_get(cfg, dim, out, 1, "x", x1);

        // Sanity check: only accept if all finite and |d| < 15 m/s²
        bool x1_ok = true;
        for (int i = 0; i < NX; ++i)
            if (!std::isfinite(x1[i])) { x1_ok = false; break; }
        double d_norm2 = x1[18]*x1[18] + x1[19]*x1[19] + x1[20]*x1[20];
        if (d_norm2 > 225.0) x1_ok = false;   // |d| > 15 m/s² → discard

        if (x1_ok) {
            for (int i = 0; i < NX; ++i) x_bar_(i) = x1[i];
            // Normalize quaternion in prior
            Quat4 qb = x_bar_.segment<4>(6);
            double qbn = qb.norm();
            if (qbn > 1e-6) x_bar_.segment<4>(6) = qb / qbn;
        }
    }

    // ── Diagonal P̄ propagation (simplified Riccati) ──────────────────
    // Physical states: decay (MHE measurement update reduces uncertainty)
    for (int i = 0; i < 16; ++i)
        P_bar_diag_(i) = std::max(P_bar_diag_(i) * 0.98, P_MIN_PHYS);

    // Parameter states: grow by process noise (random walk), then decay
    // m: P̄_m,new = P̄_m + Q_m * dt
    P_bar_diag_(16) = P_bar_diag_(16) * 0.98 + Q_W_M  * DT;
    P_bar_diag_(17) = P_bar_diag_(17) * 0.98 + Q_W_TAU * DT;
    for (int i = 18; i < 21; ++i)
        P_bar_diag_(i) = P_bar_diag_(i) * 0.98 + Q_W_D * DT;

    // Floor to prevent over-confidence
    for (int i = 0; i < 16; ++i)
        P_bar_diag_(i) = std::max(P_bar_diag_(i), P_MIN_PHYS);
    for (int i = 16; i < 21; ++i)
        P_bar_diag_(i) = std::max(P_bar_diag_(i), P_MIN_PARAM);

    // Update P̄_inv from new P̄_diag (same layout as reset())
    auto& pd = P_bar_diag_;
    P_bar_inv_(0) = 1.0/pd(0); P_bar_inv_(1) = 1.0/pd(1); P_bar_inv_(2) = 1.0/pd(2);
    P_bar_inv_(3) = 1.0/pd(3); P_bar_inv_(4) = 1.0/pd(4); P_bar_inv_(5) = 1.0/pd(5);
    // quaternion: isotropic weight from pd[6]
    P_bar_inv_(6) = 1.0/pd(6); P_bar_inv_(7) = 1.0/pd(6); P_bar_inv_(8) = 1.0/pd(6);
    P_bar_inv_(9) = 1.0/pd(10); P_bar_inv_(10) = 1.0/pd(11); P_bar_inv_(11) = 1.0/pd(12);
    P_bar_inv_(12) = 1.0/pd(13);
    P_bar_inv_(13) = 1.0/pd(14);
    P_bar_inv_(14) = 1.0/pd(15);
    P_bar_inv_(15) = 1.0/pd(16);
    P_bar_inv_(16) = 1.0/pd(17);
    P_bar_inv_(17) = 1.0/pd(18); P_bar_inv_(18) = 1.0/pd(19); P_bar_inv_(19) = 1.0/pd(20);

    // σ_k = tr(P̄_θ,k): sum of parameter variances [m, τ_rc, d(3)]
    sigma_k_ = P_bar_diag_(16) + P_bar_diag_(17)
               + P_bar_diag_(18) + P_bar_diag_(19) + P_bar_diag_(20);
}

Eigen::Matrix<double,20,1> MheController::arrival_error_(const State21& x_a) const
{
    Eigen::Matrix<double,20,1> e;
    e.head<3>()      = x_a.head<3>()       - x_bar_.head<3>();     // p
    e.segment<3>(3)  = x_a.segment<3>(3)   - x_bar_.segment<3>(3); // v
    // Lie-invariant quaternion error (log-map)
    Quat4 q_bar = x_bar_.segment<4>(6);
    double qbn = q_bar.norm(); if (qbn > 1e-6) q_bar /= qbn;
    Quat4 q    = x_a.segment<4>(6);
    double qn  = q.norm();    if (qn  > 1e-6) q     /= qn;
    Quat4 q_err = quat_mult(quat_inv(q_bar), q);  // q_bar^{-1} ⊗ q
    e.segment<3>(6)  = quat_log_map(q_err);
    e.segment<3>(9)  = x_a.segment<3>(10) - x_bar_.segment<3>(10); // ω
    e(12) = x_a(13) - x_bar_(13);  // θ
    e(13) = x_a(14) - x_bar_(14);  // v_θ
    e(14) = x_a(15) - x_bar_(15);  // f
    e(15) = x_a(16) - x_bar_(16);  // m
    e(16) = x_a(17) - x_bar_(17);  // τ_rc
    e.tail<3>() = x_a.tail<3>() - x_bar_.tail<3>();  // d
    return e;
}

void MheController::free()
{
    if (capsule_) {
        quadrotor_mhe_acados_free(capsule_);
        quadrotor_mhe_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

}  // namespace quadrotor_mpc
