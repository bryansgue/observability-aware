#pragma once

#include "quadrotor_mpc/common/types.hpp"
#include "quadrotor_mpc/common/params.hpp"

// Forward-declare the acados capsule (C linkage)
extern "C" {
    struct quadrotor_nmpc_solver_capsule;
}

namespace quadrotor_mpc {

class NmpcController {
public:
    NmpcController();
    ~NmpcController();

    // Non-copyable
    NmpcController(const NmpcController&) = delete;
    NmpcController& operator=(const NmpcController&) = delete;

    /// Create and initialise the acados solver
    bool init();

    /// Set initial state constraint x(0)
    void set_x0(const State13& x0);

    /// Set reference for a specific shooting node k ∈ [0, N-1]
    /// p_ref: desired position, q_ref: desired quaternion
    void set_reference(int k, const Vec3& p_ref, const Quat4& q_ref,
                       const Vec3& v_ref = Vec3::Zero());

    /// Set terminal reference (stage N)
    void set_reference_terminal(const Vec3& p_ref, const Quat4& q_ref,
                                const Vec3& v_ref = Vec3::Zero());

    /// Set cost weights (applied to all stages)
    void set_weights(const Vec3& Q_pos, const Vec3& Q_att,
                     const Eigen::Vector4d& R_u,
                     const Vec3& Q_vel = Vec3(10.0, 10.0, 10.0));

    /// Set adaptive model parameters (injected into the dynamics + cost).
    /// m_hat: estimated mass [kg], d_hat: disturbance accel [m/s²],
    /// k_tau: estimated 1/τ_rc [1/s]. Applied on the next set_reference() calls.
    void set_model_params(double m_hat, const Vec3& d_hat, double k_tau);

    /// Solve the OCP. Returns acados status (0 = success).
    int solve();

    /// Get optimal control at stage 0
    Control4 get_u0() const;

    /// Get predicted state at stage k
    State13 get_x(int k) const;

    /// Get solver wall-clock time [s]
    double get_solve_time() const;

    /// Free solver resources
    void free();

    /// These are read from the generated solver header at compile time.
    /// If you change dt or T_horizon, regenerate the solver and rebuild.
    static const int N;
    static constexpr int NX = 13;
    static constexpr int NU = 4;
    static constexpr int NP = 28;

private:
    quadrotor_nmpc_solver_capsule* capsule_ = nullptr;

    // Cached weight values for building parameter vector
    Vec3 Q_pos_ = Vec3(50.0, 50.0, 50.0);
    Vec3 Q_att_ = Vec3(5.0, 5.0, 5.0);
    Vec3 Q_vel_ = Vec3(10.0, 10.0, 10.0);
    Eigen::Vector4d R_u_ = Eigen::Vector4d(0.1, 0.3, 0.3, 0.3);

    // Cached adaptive model parameters (default = nominal)
    double m_hat_ = 1.08;            // [kg]
    Vec3   d_hat_ = Vec3::Zero();    // [m/s²]
    double k_tau_ = 1.0 / 0.03;      // [1/s]

    /// Build the 28-element parameter vector for a given stage
    void build_params(double* p, const Vec3& p_ref, const Quat4& q_ref,
                      const Vec3& v_ref = Vec3::Zero()) const;
};

}  // namespace quadrotor_mpc
