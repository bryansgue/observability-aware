#include "quadrotor_mpc/nmpc/nmpc_controller.hpp"

#include "acados_solver_quadrotor_nmpc.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstring>
#include <cstdio>

namespace quadrotor_mpc {

// N comes from the generated solver (QUADROTOR_NMPC_N = T_HORIZON / DT_CONTROL)
const int NmpcController::N = QUADROTOR_NMPC_N;

NmpcController::NmpcController() = default;

NmpcController::~NmpcController() {
    free();
}

bool NmpcController::init() {
    capsule_ = quadrotor_nmpc_acados_create_capsule();
    int status = quadrotor_nmpc_acados_create_with_discretization(capsule_, N, nullptr);
    if (status != 0) {
        std::fprintf(stderr, "[NmpcController] acados_create failed with status %d\n", status);
        return false;
    }
    return true;
}

void NmpcController::set_x0(const State13& x0) {
    ocp_nlp_config* config = quadrotor_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dims   = quadrotor_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in*     in     = quadrotor_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out*    out    = quadrotor_nmpc_acados_get_nlp_out(capsule_);

    double lbx0[NX], ubx0[NX];
    for (int i = 0; i < NX; ++i) {
        lbx0[i] = x0(i);
        ubx0[i] = x0(i);
    }
    ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", ubx0);
}

void NmpcController::build_params(double* p, const Vec3& p_ref, const Quat4& q_ref,
                                  const Vec3& v_ref) const {
    // p = [p_ref(3), q_ref(4), Q_pos(3), Q_att(3), R_u(4), m̂(1), d̂(3), k̂_τ(1),
    //      v_ref(3), Q_vel(3)]  → 28
    p[0]  = p_ref(0);  p[1]  = p_ref(1);  p[2]  = p_ref(2);
    p[3]  = q_ref(0);  p[4]  = q_ref(1);  p[5]  = q_ref(2);  p[6]  = q_ref(3);
    p[7]  = Q_pos_(0); p[8]  = Q_pos_(1); p[9]  = Q_pos_(2);
    p[10] = Q_att_(0); p[11] = Q_att_(1); p[12] = Q_att_(2);
    p[13] = R_u_(0);   p[14] = R_u_(1);   p[15] = R_u_(2);   p[16] = R_u_(3);
    p[17] = m_hat_;
    p[18] = d_hat_(0); p[19] = d_hat_(1); p[20] = d_hat_(2);
    p[21] = k_tau_;
    p[22] = v_ref(0);  p[23] = v_ref(1);  p[24] = v_ref(2);
    p[25] = Q_vel_(0); p[26] = Q_vel_(1); p[27] = Q_vel_(2);
}

void NmpcController::set_reference(int k, const Vec3& p_ref, const Quat4& q_ref,
                                   const Vec3& v_ref) {
    double p[NP];
    build_params(p, p_ref, q_ref, v_ref);
    quadrotor_nmpc_acados_update_params(capsule_, k, p, NP);
}

void NmpcController::set_reference_terminal(const Vec3& p_ref, const Quat4& q_ref,
                                            const Vec3& v_ref) {
    double p[NP];
    build_params(p, p_ref, q_ref, v_ref);
    quadrotor_nmpc_acados_update_params(capsule_, N, p, NP);
}

void NmpcController::set_weights(const Vec3& Q_pos, const Vec3& Q_att,
                                  const Eigen::Vector4d& R_u, const Vec3& Q_vel) {
    Q_pos_ = Q_pos;
    Q_att_ = Q_att;
    R_u_   = R_u;
    Q_vel_ = Q_vel;
}

void NmpcController::set_model_params(double m_hat, const Vec3& d_hat, double k_tau) {
    m_hat_ = m_hat;
    d_hat_ = d_hat;
    k_tau_ = k_tau;
}

int NmpcController::solve() {
    return quadrotor_nmpc_acados_solve(capsule_);
}

Control4 NmpcController::get_u0() const {
    ocp_nlp_config* config = quadrotor_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dims   = quadrotor_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out    = quadrotor_nmpc_acados_get_nlp_out(capsule_);

    double u[NU];
    ocp_nlp_out_get(config, dims, out, 0, "u", u);
    return Control4(u[0], u[1], u[2], u[3]);
}

State13 NmpcController::get_x(int k) const {
    ocp_nlp_config* config = quadrotor_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dims   = quadrotor_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out    = quadrotor_nmpc_acados_get_nlp_out(capsule_);

    double x[NX];
    ocp_nlp_out_get(config, dims, out, k, "x", x);
    return Eigen::Map<State13>(x);
}

double NmpcController::get_solve_time() const {
    ocp_nlp_solver* solver = quadrotor_nmpc_acados_get_nlp_solver(capsule_);
    double t;
    ocp_nlp_get(solver, "time_tot", &t);
    return t;
}

void NmpcController::free() {
    if (capsule_) {
        quadrotor_nmpc_acados_free(capsule_);
        quadrotor_nmpc_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

}  // namespace quadrotor_mpc
