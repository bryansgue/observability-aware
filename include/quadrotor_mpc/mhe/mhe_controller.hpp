#pragma once
/**
 * MHE Controller — Lie-invariant Moving Horizon Estimator.
 *
 * Augmented state x_a ∈ ℝ²¹:
 *   [p(3), v(3), q(4), ω(3), θ, v_θ, f, m, τ_rc, d(3)]
 *    0:3   3:6   6:10 10:13  13   14   15  16    17   18:21
 *
 * Process noise w ∈ ℝ⁵  (MHE "controls"):
 *   [w_m, w_τ, w_d(3)]
 *
 * Runtime params p ∈ ℝ⁵⁹ per stage:
 *   p[ 0:13] = y_k   measurement [p(3), v(3), q(4), ω(3)]
 *   p[13:18] = u_k   applied control [Δf, ωx, ωy, ωz, a_θ]
 *   p[18:39] = x̄    prior state (21D) — used only in cost_type_0
 *   p[39:59] = P̄_inv arrival cost diagonal weights (20D Lie-invariant error)
 *
 * Outputs:
 *   x̂_{N_e} = estimated augmented state at current time
 *   σ_k = tr(P̄_θ,k) = parameter uncertainty → drives adaptive W_s
 */

#include "quadrotor_mpc/common/types.hpp"
#include "quadrotor_mpc/mujoco/mujoco_interface.hpp"
#include <Eigen/Dense>
#include <deque>
#include <chrono>

struct quadrotor_mhe_solver_capsule;

namespace quadrotor_mpc {

/// Full augmented state estimate at current time step
struct MheEstimate {
    Vec3   pos;     Vec3   vel;     Quat4  quat;    Vec3  omega;  // physical (0:13)
    double theta   = 0.0;   // arc-length progress  (13)
    double vtheta  = 0.0;   // progress velocity    (14)
    double f       = 10.6;  // thrust state [N]     (15)
    double m_hat   = 1.08;  // mass estimate [kg]   (16)
    double tau_hat = 0.03;  // τ_rc estimate [s]    (17)
    Vec3   d_hat   = Vec3::Zero();  // disturbance [m/s²] (18:21)

    /// Pack into 21D vector
    State21 to_vector() const {
        State21 v;
        v.head<3>()       = pos;
        v.segment<3>(3)   = vel;
        v.segment<4>(6)   = quat;
        v.segment<3>(10)  = omega;
        v(13) = theta; v(14) = vtheta; v(15) = f;
        v(16) = m_hat; v(17) = tau_hat;
        v.tail<3>()       = d_hat;
        return v;
    }

    /// Unpack from 21D vector
    static MheEstimate from_vector(const State21& v) {
        MheEstimate e;
        e.pos   = v.head<3>();
        e.vel   = v.segment<3>(3);
        e.quat  = v.segment<4>(6);
        e.omega = v.segment<3>(10);
        e.theta  = v(13); e.vtheta = v(14); e.f = v(15);
        e.m_hat  = v(16); e.tau_hat = v(17);
        e.d_hat  = v.tail<3>();
        return e;
    }
};

class MheController {
public:
    static const int N  = 20;   // horizon length (0.2s @ 100Hz)
    static const int NX = 21;   // augmented state
    static const int NU = 5;    // process noise
    static const int NP = 59;   // runtime params per stage

    MheController();
    ~MheController();

    /// Initialize acados capsule. Returns false on failure.
    bool init();

    /// Reset estimator with initial prior (x̄, diagonal P̄).
    void reset(const MheEstimate& x0_prior,
               const Eigen::Matrix<double,21,1>& P_bar_diag);

    /// Push new measurement y_k (13D) and previously applied control u_k (5D).
    /// Slides the window forward (drop oldest, add newest).
    void push(const Eigen::Matrix<double,13,1>& y_k,
              const Control5& u_applied);

    /// Run one SQP-RTI iteration. Returns solver status (0 = success).
    int solve();

    /// Get full augmented state estimate at stage N (current time).
    MheEstimate get_estimate() const;

    /// Get parameter estimate [m̂, τ̂_rc, d̂] ∈ ℝ⁵.
    Eigen::Matrix<double,5,1> get_param_estimate() const;

    /// σ_k = tr(P̄_θ,k) — scalar parameter uncertainty for adaptive W_s.
    double get_sigma() const { return sigma_k_; }

    /// Wall-clock MHE solve time [s].
    double get_solve_time() const { return solve_time_s_; }

    /// Propagate arrival cost prior after solve (simplified diagonal Riccati).
    /// @param solver_ok  true  → update x_bar_ from stage-1 estimate + grow P̄
    ///                   false → only grow P̄ uncertainty (bad solve: don't corrupt prior)
    /// Must be called after each solve() before the next push().
    void propagate_prior(bool solver_ok = true);

    /// Release acados resources.
    void free();

private:
    quadrotor_mhe_solver_capsule* capsule_ = nullptr;

    // Sliding window (length N+1 for states, N for controls)
    std::deque<Eigen::Matrix<double,13,1>> y_win_;  // measurements
    std::deque<Control5>                   u_win_;  // applied controls

    // Arrival cost prior at stage 0
    State21                        x_bar_;        // prior state mean
    Eigen::Matrix<double,20,1>     P_bar_inv_;    // diagonal P̄^{-1} (20D Lie-inv error)
    Eigen::Matrix<double,21,1>     P_bar_diag_;   // P̄ diagonal variances (for propagation)

    // Uncertainty measure for adaptive W_s
    double sigma_k_      = 1e6;  // tr(P̄_θ,k), large initially
    double solve_time_s_ = 0.0;

    // Process noise variances [m, τ, d] — used in propagation
    static constexpr double Q_W_M   = 1e-4;   // [kg²/s] mass drift variance
    static constexpr double Q_W_TAU = 1e-6;   // τ_rc drift variance
    static constexpr double Q_W_D   = 1e-3;   // [m²/s⁵] disturbance drift variance
    static constexpr double DT      = 0.01;   // [s] control period

    // Floor for P̄ diagonal (prevents over-confidence → ACADOS_MINSTEP)
    // Steady-state P̄_m ≈ Q_W_M*DT/(1-0.98) = 5e-5.  If P_MIN_PARAM is
    // smaller, P̄_inv can reach 20,000 → arrival cost stiffness → MINSTEP.
    // Floor at 1e-3 caps P̄_inv_param ≤ 1000 (std ≥ 0.032) — still informative.
    static constexpr double P_MIN_PHYS  = 1e-4;  // physical states → P̄_inv ≤ 10,000
    static constexpr double P_MIN_PARAM = 1e-3;  // parameters     → P̄_inv ≤ 1,000

    /// Pack and set runtime parameters for stage k.
    void set_stage_params_(int k);

    /// Compute 20D Lie-invariant arrival error from x_a and x̄.
    Eigen::Matrix<double,20,1> arrival_error_(const State21& x_a) const;
};

}  // namespace quadrotor_mpc
