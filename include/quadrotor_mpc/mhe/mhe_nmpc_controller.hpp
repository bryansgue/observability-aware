#pragma once
/**
 * MheNmpcController — Lie-invariant MHE matched to the NMPC (thrust-as-input).
 *
 * Augmented state x_a ∈ ℝ¹⁸:
 *   [p(3), v(3), q(4), ω(3), m, τ_rc, d(3)]
 *    0:3   3:6   6:10 10:13  13   14   15:18
 *
 * Process noise w ∈ ℝ⁵ (MHE "controls"): [w_m, w_τ, w_d(3)]
 *
 * Applied control (KNOWN input, fed as params): u_k = [T, ω_cmd(3)] ∈ ℝ⁴
 *
 * Runtime params p ∈ ℝ⁵² per stage:
 *   p[ 0:13] = y_k   measurement [p,v,q,ω]
 *   p[13:17] = u_k   applied control [T, ω_cmd]
 *   p[17:35] = x̄    prior state (18D) — arrival cost (stage 0)
 *   p[35:52] = P̄_inv arrival cost diagonal weights (17D Lie-invariant error)
 *
 * Unlike the 21D MPCC MHE, thrust is a KNOWN input here (not a state), so the
 * estimator is consistent with an NMPC that commands T directly.
 */

#include "quadrotor_mpc/common/types.hpp"
#include <Eigen/Dense>
#include <deque>

struct quadrotor_mhe_nmpc_solver_capsule;

namespace quadrotor_mpc {

using State18 = Eigen::Matrix<double, 18, 1>;

/// Physical state + parameter estimate at current time
struct MheNmpcEstimate {
    Vec3   pos;    Vec3   vel;    Quat4  quat;   Vec3  omega;  // physical (0:13)
    double m_hat   = 1.08;          // mass [kg]        (13)
    double tau_hat = 0.03;          // τ_rc [s]         (14)
    Vec3   d_hat   = Vec3::Zero();  // disturbance [m/s²] (15:18)

    State18 to_vector() const {
        State18 v;
        v.head<3>() = pos; v.segment<3>(3) = vel;
        v.segment<4>(6) = quat; v.segment<3>(10) = omega;
        v(13) = m_hat; v(14) = tau_hat; v.tail<3>() = d_hat;
        return v;
    }
    static MheNmpcEstimate from_vector(const State18& v) {
        MheNmpcEstimate e;
        e.pos = v.head<3>(); e.vel = v.segment<3>(3);
        e.quat = v.segment<4>(6); e.omega = v.segment<3>(10);
        e.m_hat = v(13); e.tau_hat = v(14); e.d_hat = v.tail<3>();
        return e;
    }
};

class MheNmpcController {
public:
    static const int N  = 31;   // 1.0 s window (dt=0.032, Jetson) → separates m from d
    static const int NX = 18;
    static const int NU = 5;
    static const int NP = 52;

    MheNmpcController();
    ~MheNmpcController();

    bool init();

    /// Reset with prior (x̄, diagonal P̄ of 18 variances).
    void reset(const MheNmpcEstimate& x0_prior,
               const Eigen::Matrix<double,18,1>& P_bar_diag);

    /// Push measurement y_k (13D) and applied control u_k = [T, ω_cmd] (4D).
    void push(const Eigen::Matrix<double,13,1>& y_k, const Control4& u_applied);

    /// One SQP-RTI iteration. Returns acados status (0 = success).
    int solve();

    MheNmpcEstimate get_estimate() const;
    double get_sigma() const { return sigma_k_; }
    double get_solve_time() const { return solve_time_s_; }

    /// Propagate arrival-cost prior after solve (simplified diagonal Riccati).
    void propagate_prior(bool solver_ok = true);

    void free();

private:
    quadrotor_mhe_nmpc_solver_capsule* capsule_ = nullptr;

    std::deque<Eigen::Matrix<double,13,1>> y_win_;
    std::deque<Control4>                   u_win_;

    State18                    x_bar_;
    Eigen::Matrix<double,17,1> P_bar_inv_;
    Eigen::Matrix<double,18,1> P_bar_diag_;

    double sigma_k_      = 1e6;
    double solve_time_s_ = 0.0;

    static constexpr double Q_W_M   = 1e-4;   // estimate mass
    static constexpr double Q_W_TAU = 1e-6;   // estimate τ_rc (slow drift)
    static constexpr double Q_W_D   = 1e-9;   // d LOCKED ≈0 → m absorbs structural error
    static constexpr double DT      = 0.032;  // MHE node spacing (1.0 s window / N=31)
    static constexpr double P_MIN_PHYS  = 1e-4;
    static constexpr double P_MIN_PARAM = 1e-3;

    void set_stage_params_(int k);
    void rebuild_pinv_();
};

}  // namespace quadrotor_mpc
