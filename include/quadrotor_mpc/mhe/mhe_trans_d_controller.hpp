#pragma once
/**
 * MheTransDController — disturbance-only translational MHE (EXP-2).
 *
 * State x ∈ ℝ⁹ = [p(3), v(3), d(3)]   — mass is NOT a state.
 * Process noise w ∈ ℝ³ = [w_d(3)]
 * Known inputs (params): T (thrust), a = R·e3, and the FIXED mass m
 *   (identified offline in EXP-1). With m fixed, the m–d_z ambiguity disappears
 *   → d_z observable, no mass drift polluting it (Prop. 1).
 *
 * Runtime params p ∈ ℝ³³:
 *   p[0:6]   = y_k  [p(3), v(3)]
 *   p[6]     = T
 *   p[7:10]  = a
 *   p[10:19] = x̄  prior (9D)
 *   p[19:28] = P̄_inv (9D)
 *   p[28]    = c_drag
 *   p[29:32] = sf_meas
 *   p[32]    = m (FIXED mass)
 */
#include "quadrotor_mpc/common/types.hpp"
#include <Eigen/Dense>
#include <deque>

struct quadrotor_mhe_trans_d_solver_capsule;

namespace quadrotor_mpc {

using State9 = Eigen::Matrix<double, 9, 1>;

struct MheTransDEstimate {
    Vec3 pos;  Vec3 vel;
    Vec3 d_hat = Vec3::Zero();

    State9 to_vector() const {
        State9 v; v.head<3>() = pos; v.segment<3>(3) = vel; v.tail<3>() = d_hat;
        return v;
    }
    static MheTransDEstimate from_vector(const State9& v) {
        MheTransDEstimate e; e.pos = v.head<3>(); e.vel = v.segment<3>(3);
        e.d_hat = v.tail<3>(); return e;
    }
};

class MheTransDController {
public:
    static const int N  = 31;
    static const int NX = 9;
    static const int NU = 3;
    static const int NP = 33;

    /// Fixed quadratic-drag acceleration coefficient.
    void set_drag(double c) { drag_c_ = c; }
    /// Fixed (identified) mass — a parameter, not a state.
    void set_mass(double m) { m_fix_ = m; }

    MheTransDController();
    ~MheTransDController();
    bool init();

    void reset(const MheTransDEstimate& x0_prior,
               const Eigen::Matrix<double,9,1>& P_bar_diag);

    /// y = [p(3), v(3)] (6D); inputs T (thrust) and a = R·e3 (world dir).
    void push(const Eigen::Matrix<double,6,1>& y_k, double T, const Vec3& a,
              const Vec3& sf_meas = Vec3(0, 0, 9.81));

    int solve();
    MheTransDEstimate get_estimate() const;
    double get_sigma() const { return sigma_k_; }
    double get_solve_time() const { return solve_time_s_; }
    void propagate_prior(bool solver_ok = true);
    void free();

private:
    quadrotor_mhe_trans_d_solver_capsule* capsule_ = nullptr;
    std::deque<Eigen::Matrix<double,6,1>> y_win_;
    std::deque<double>                    T_win_;
    std::deque<Vec3>                      a_win_;
    std::deque<Vec3>                      sf_win_;

    State9                    x_bar_;
    Eigen::Matrix<double,9,1> P_bar_inv_;
    Eigen::Matrix<double,9,1> P_bar_diag_;

    double sigma_k_      = 1e6;
    double solve_time_s_ = 0.0;
    double drag_c_       = 0.0;
    double m_fix_        = 1.05;   // FIXED mass parameter
    static constexpr double Q_W_D = 1e-3;
    static constexpr double DT    = 0.01;
    static constexpr double P_MIN_PHYS  = 1e-4;
    static constexpr double P_MIN_PARAM = 1e-3;

    void set_stage_params_(int k);
};

}  // namespace quadrotor_mpc
