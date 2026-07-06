#include "quadrotor_mpc/mpcc/mpcc_controller.hpp"

#include "acados_solver_quadrotor_mpcc.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstring>
#include <cstdio>

namespace quadrotor_mpc {

const int MpccController::N = QUADROTOR_MPCC_N;

MpccController::MpccController() = default;
MpccController::~MpccController() { free(); }

bool MpccController::init() {
    capsule_ = quadrotor_mpcc_acados_create_capsule();
    int status = quadrotor_mpcc_acados_create_with_discretization(capsule_, N, nullptr);
    if (status != 0) {
        std::fprintf(stderr, "[MpccController] acados_create failed: %d\n", status);
        return false;
    }
    return true;
}

void MpccController::set_x0(const State16& x0) {
    ocp_nlp_config* cfg = quadrotor_mpcc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mpcc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in*     in  = quadrotor_mpcc_acados_get_nlp_in(capsule_);
    ocp_nlp_out*    out = quadrotor_mpcc_acados_get_nlp_out(capsule_);

    double lbx[NX], ubx[NX];
    for (int i = 0; i < NX; ++i) { lbx[i] = x0(i); ubx[i] = x0(i); }
    ocp_nlp_constraints_model_set(cfg, dim, in, out, 0, "lbx", lbx);
    ocp_nlp_constraints_model_set(cfg, dim, in, out, 0, "ubx", ubx);
}

void MpccController::weights_to_param(double* p, const MpccWeights& w) {
    p[0]=w.Q_ec(0); p[1]=w.Q_ec(1); p[2]=w.Q_ec(2);
    p[3]=w.Q_el(0); p[4]=w.Q_el(1); p[5]=w.Q_el(2);
    p[6]=w.Q_q(0);  p[7]=w.Q_q(1);  p[8]=w.Q_q(2);
    p[9]=w.U_mat(0); p[10]=w.U_mat(1); p[11]=w.U_mat(2); p[12]=w.U_mat(3);
    p[13]=w.Q_s;
    p[14]=w.vtheta_max;
    p[15]=w.W_df;
    // v2: adaptive model parameters from MHE
    p[16]=w.m_hat;
    p[17]=w.d_hat(0); p[18]=w.d_hat(1); p[19]=w.d_hat(2);
    p[20]=w.k_tau;
}

void MpccController::set_params(int stage, const MpccWeights& w) {
    double p[NP]; weights_to_param(p, w);
    quadrotor_mpcc_acados_update_params(capsule_, stage, p, NP);
}

void MpccController::set_params_all(const MpccWeights& w) {
    double p[NP]; weights_to_param(p, w);
    for (int k = 0; k <= N; ++k)
        quadrotor_mpcc_acados_update_params(capsule_, k, p, NP);
}

int MpccController::solve() {
    return quadrotor_mpcc_acados_solve(capsule_);
}

Control5 MpccController::get_u0() const {
    ocp_nlp_config* cfg = quadrotor_mpcc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mpcc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mpcc_acados_get_nlp_out(capsule_);
    double u[NU];
    ocp_nlp_out_get(cfg, dim, out, 0, "u", u);
    return Control5(u[0], u[1], u[2], u[3], u[4]);
}

State16 MpccController::get_x(int k) const {
    ocp_nlp_config* cfg = quadrotor_mpcc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mpcc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mpcc_acados_get_nlp_out(capsule_);
    double x[NX];
    ocp_nlp_out_get(cfg, dim, out, k, "x", x);
    return Eigen::Map<State16>(x);
}

void MpccController::set_x_init(int k, const State16& x) {
    ocp_nlp_config* cfg = quadrotor_mpcc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mpcc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mpcc_acados_get_nlp_out(capsule_);
    ocp_nlp_in*     in  = quadrotor_mpcc_acados_get_nlp_in(capsule_);
    double xd[NX]; for (int i=0;i<NX;++i) xd[i]=x(i);
    ocp_nlp_out_set(cfg, dim, out, in, k, "x", xd);
}

void MpccController::set_u_init(int k, const Control5& u) {
    ocp_nlp_config* cfg = quadrotor_mpcc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims*   dim = quadrotor_mpcc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out*    out = quadrotor_mpcc_acados_get_nlp_out(capsule_);
    ocp_nlp_in*     in  = quadrotor_mpcc_acados_get_nlp_in(capsule_);
    double ud[NU]; for (int i=0;i<NU;++i) ud[i]=u(i);
    ocp_nlp_out_set(cfg, dim, out, in, k, "u", ud);
}

double MpccController::get_solve_time() const {
    ocp_nlp_solver* slv = quadrotor_mpcc_acados_get_nlp_solver(capsule_);
    double t; ocp_nlp_get(slv, "time_tot", &t); return t;
}

void MpccController::free() {
    if (capsule_) {
        quadrotor_mpcc_acados_free(capsule_);
        quadrotor_mpcc_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
}

}  // namespace quadrotor_mpc
