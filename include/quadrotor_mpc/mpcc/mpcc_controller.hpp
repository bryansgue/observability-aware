#pragma once
/**
 * MPCC Controller — C++ wrapper over acados-generated MPCC solver.
 *
 * State  x ∈ ℝ¹⁶ = [p(3), v(3), q(4), ω(3), θ, v_θ, f]
 * Control u ∈ ℝ⁵  = [Δf, ωx_cmd, ωy_cmd, ωz_cmd, a_θ]
 *
 * Runtime parameters p ∈ ℝ²⁰:
 *   p[ 0: 3] = Q_ec      contouring error weights
 *   p[ 3: 6] = Q_el      lag error weights
 *   p[ 6: 9] = Q_q       quaternion-log error weights
 *   p[ 9:13] = U_mat     control effort [f_err, ωx, ωy, ωz]
 *   p[13]    = Q_s       progress weight  (-Q_s * v_θ)  ← adaptive W_s goes here
 *   p[14]    = vtheta_max
 *   p[15]    = W_df      thrust rate penalty: W_df * Δf²
 *   p[16]    = m_hat     estimated mass [kg]          (v2: from MHE)
 *   p[17:20] = d_hat     estimated disturbance [m/s²] (v2: from MHE)
 *   p[20]    = k_tau     rate-ctrl gain = 1/tau_rc [1/s] (v2: from MHE)
 */

#include "quadrotor_mpc/common/types.hpp"
#include <Eigen/Dense>

struct quadrotor_mpcc_solver_capsule;

namespace quadrotor_mpc {

struct MpccWeights {
    Vec3 Q_ec   = Vec3(100.0, 100.0, 100.0);
    Vec3 Q_el   = Vec3(13.0,  13.0,  13.0);
    Vec3 Q_q    = Vec3(0.5,   0.5,   0.5);
    Eigen::Vector4d U_mat = Eigen::Vector4d(0.1, 0.3, 0.3, 0.3);
    double Q_s        = 15.0;
    double vtheta_max = 14.0;
    double W_df       = 0.001;
    // v2 adaptive model parameters (from MHE) — defaults reproduce v1 behaviour
    double m_hat      = 1.08;         // estimated mass [kg]
    Vec3   d_hat      = Vec3::Zero(); // estimated disturbance [m/s²]
    double k_tau      = 1.0 / 0.03;  // rate-ctrl gain = 1/tau_rc [1/s]
};

class MpccController {
public:
    static const int N;
    static const int NX = 16;
    static const int NU = 5;
    static const int NP = 21;

    MpccController();
    ~MpccController();

    bool     init();
    void     set_x0(const State16& x0);
    void     set_params(int stage, const MpccWeights& w);
    void     set_params_all(const MpccWeights& w);
    int      solve();
    Control5 get_u0() const;
    State16  get_x(int k) const;
    void     set_x_init(int k, const State16& x);
    void     set_u_init(int k, const Control5& u);
    double   get_solve_time() const;
    void     free();

private:
    quadrotor_mpcc_solver_capsule* capsule_ = nullptr;
    static void weights_to_param(double* p, const MpccWeights& w);
};

}  // namespace quadrotor_mpc
