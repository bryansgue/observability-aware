"""
Generate acados C code for a Lie-invariant MHE matched to the NMPC.

This is the CLEAN estimator for the adaptive NMPC: thrust is the APPLIED CONTROL
(a known input), NOT a state. That removes the thrust-as-state impedance mismatch
that makes the 21-state MPCC MHE fail when driven by the NMPC.

Augmented state  x_a ∈ ℝ¹⁸ = [p(3), v(3), q(4), ω(3), m, τ_rc, d(3)]
                               0:3   3:6   6:10  10:13  13   14    15:18
Process noise    w   ∈ ℝ⁵  = [w_m, w_τ, w_d(3)]   (MHE "controls")

Applied control comes from runtime params (known input, like the NMPC commands):
  u_k = [T, ωx_cmd, ωy_cmd, ωz_cmd]   (4D)

Dynamics:
  v̇ = -g·e₃ + (R·[0,0,T])/m + d        (T = known input, m = estimated)
  ω̇ = (ω_cmd − ω)/τ                     (ω_cmd = known input, τ = estimated)

Runtime params per stage: p ∈ ℝ⁵²
  p[ 0:13] = y_k   measurement [p(3), v(3), q(4), ω(3)]
  p[13:17] = u_k   applied control [T, ω_cmd(3)]
  p[17:35] = x̄    prior state (18D) — arrival cost (stage 0)
  p[35:52] = P̄_inv arrival cost diagonal weights (17D Lie-invariant error)

Horizon: N = 20, dt = 0.01 s (0.2 s window). Solver: SQP_RTI, HPIPM ROBUST.
Output: ../c_generated_code_mhe_nmpc/

Run once: python3 generate_mhe_nmpc_ocp.py
"""

import os
import shutil
import numpy as np
import casadi as ca
from casadi import MX, vertcat, norm_2, if_else, atan2

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
from quad_config import MASS, G, TAU_RC

# ── MHE dimensions ────────────────────────────────────────────────────────────
NX  = 18   # [p,v,q,ω,m,τ,d]
NU  = 5    # process noise: [w_m, w_τ, w_d(3)]
NP  = 52   # [y(13), u_applied(4), x̄(18), P̄_inv(17)]
N   = 31   # MHE horizon (Jetson-runnable)
DT  = 0.032 # 1.0 s window (31×0.032≈0.99s) → separates m from d, ~31 Hz

# Fixed measurement noise inverse (diagonal R⁻¹, 12×12)
# Attitude is MEASURED, not estimated → pin q,ω TIGHTLY to the measurement (high
# R_Q, R_OM) so the quaternion state does not drift over the 1.0s window under large
# yaw (that drift was breaking the QP). Translation (p,v) moderate → mass/d come from
# how measured thrust+attitude explain the measured acceleration.
R_P, R_V, R_Q, R_OM = 20.0, 5.0, 300.0, 30.0
R_INV_DIAG = np.array([R_P]*3 + [R_V]*3 + [R_Q]*3 + [R_OM]*3)

# Fixed process noise inverse (diagonal Q_w⁻¹, 5×5)
# d̂ process noise — moderate: loose enough to TRACK a time-varying disturbance
# (wind), but not so loose it saturates and confounds the mass at low excitation.
# Nominal (no wind) used a stiff d̂; the wind experiment loosens it to track.
# d̂ HARD-LOCKED ≈0 (penalise the d random-walk noise w_d strongly + tiny bound):
# with no disturbance to explain, the MHE attributes all structural mismatch to
# MASS → m̂ converges (biased by drag, but stable, not confounded with d).
# JOINT estimation m, τ, d — all FREE. With the 1.0 s window the attitude/thrust
# vary enough to separate m from d (identifiability). d loose enough to solve.
QW_M, QW_TAU, QW_D = 1.0/0.001, 1.0/0.001, 1.0/0.01
QW_INV_DIAG = np.array([QW_M, QW_TAU, QW_D, QW_D, QW_D])

# Process noise bounds
W_M_MAX, W_TAU_MAX, W_D_MAX = 0.1, 0.01, 2.0
# Parameter bounds
M_MIN, M_MAX = 0.5, 3.0
TAU_MIN, TAU_MAX = 0.005, 0.15
D_MIN, D_MAX = -8.0, 8.0


def quat_mult(q1, q2):
    w0, x0, y0, z0 = q1[0], q1[1], q1[2], q1[3]
    w1, x1, y1, z1 = q2[0], q2[1], q2[2], q2[3]
    return vertcat(
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    )


def quat_inv(q):
    return vertcat(q[0], -q[1], -q[2], -q[3]) / (norm_2(q) + 1e-12)


def quat_log(q):
    """SO(3) log map with hemisphere correction → 3D.

    The epsilon is added INSIDE the sqrt: nv = sqrt(qv·qv + ε). This keeps the
    Jacobian finite at the identity quaternion (qv=0). Using norm_2(qv)+ε gives
    a 0/0 = NaN derivative at qv=0 → NaN Gauss-Newton Hessian → QP_FAILURE
    whenever the drone is exactly level (e.g. hover at the start).
    """
    q_hc = if_else(q[0] < 0, -q, q)
    q_v  = q_hc[1:]
    nv   = ca.sqrt(ca.dot(q_v, q_v) + 1e-12)
    theta = atan2(nv, q_hc[0])
    return 2.0 * q_v * theta / nv


def quat_err(q_ref, q_state):
    return quat_log(quat_mult(quat_inv(q_ref), q_state))


def build_model():
    model = AcadosModel()
    model.name = "quadrotor_mhe_nmpc"
    e3 = MX([0, 0, 1])

    # ── States (18) ──────────────────────────────────────────────────────
    p_s   = MX.sym("p",   3)
    v_s   = MX.sym("v",   3)
    q_s   = MX.sym("q",   4)
    om_s  = MX.sym("om",  3)
    m_s   = MX.sym("m")
    tau_s = MX.sym("tau")
    d_s   = MX.sym("d",   3)
    x = vertcat(p_s, v_s, q_s, om_s, m_s, tau_s, d_s)

    # ── Process noise (5) ────────────────────────────────────────────────
    w_m   = MX.sym("w_m")
    w_tau = MX.sym("w_tau")
    w_d   = MX.sym("w_d", 3)
    u_noise = vertcat(w_m, w_tau, w_d)

    # ── Runtime params (52) ──────────────────────────────────────────────
    p_sym = MX.sym("p_rt", NP)
    y_p, y_v, y_q, y_om = p_sym[0:3], p_sym[3:6], p_sym[6:10], p_sym[10:13]
    u_T     = p_sym[13]      # applied thrust [N]  (known input)
    u_omcmd = p_sym[14:17]   # applied ω_cmd       (known input)

    # ── Rotation matrix ──────────────────────────────────────────────────
    qn  = norm_2(q_s) + 1e-12
    qnv = q_s / qn
    q_hat = MX.zeros(3, 3)
    q_hat[0,1] = -q_s[3]; q_hat[0,2] =  q_s[2]
    q_hat[1,0] =  q_s[3]; q_hat[1,2] = -q_s[1]
    q_hat[2,0] = -q_s[2]; q_hat[2,1] =  q_s[1]
    Rot = MX.eye(3) + 2*q_hat@q_hat + 2*qnv[0]*q_hat

    # ── Dynamics (thrust + ω_cmd are KNOWN inputs) ───────────────────────
    dp = v_s
    dv = -G*e3 + (Rot @ vertcat(MX(0), MX(0), u_T)) / m_s + d_s
    om1 = vertcat(MX(0), om_s)
    w0_, x0_, y0_, z0_ = q_s[0], q_s[1], q_s[2], q_s[3]
    dq = 0.5 * vertcat(
        w0_*om1[0] - x0_*om1[1] - y0_*om1[2] - z0_*om1[3],
        w0_*om1[1] + x0_*om1[0] + y0_*om1[3] - z0_*om1[2],
        w0_*om1[2] - x0_*om1[3] + y0_*om1[0] + z0_*om1[1],
        w0_*om1[3] + x0_*om1[2] - y0_*om1[1] + z0_*om1[0],
    )
    dom  = (u_omcmd - om_s) / (tau_s + 1e-6)
    dm   = w_m
    dtau = w_tau
    dd   = w_d
    f_expl = vertcat(dp, dv, dq, dom, dm, dtau, dd)

    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x, model.xdot, model.u, model.p = x, x_dot, u_noise, p_sym
    return model, p_sym, y_p, y_v, y_q, y_om, p_s, v_s, q_s, om_s, m_s, tau_s, d_s


def build_ocp():
    ocp = AcadosOcp()
    model, p_sym, y_p, y_v, y_q, y_om, p_s, v_s, q_s, om_s, m_s, tau_s, d_s = build_model()
    ocp.model = model
    ocp.code_export_directory = os.path.join(SCRIPT_DIR, "..", "c_generated_code_mhe_nmpc")
    ocp.solver_options.N_horizon = N
    u_noise = model.u

    R_inv  = np.diag(R_INV_DIAG)
    QW_inv = np.diag(QW_INV_DIAG)

    # ── Stage cost: 12D measurement residual + process noise ─────────────
    e_meas = vertcat(p_s - y_p, v_s - y_v, quat_err(y_q, q_s), om_s - y_om)  # 12D
    ocp.cost.cost_type = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = e_meas.T @ R_inv @ e_meas + u_noise.T @ QW_inv @ u_noise

    # ── Arrival cost (stage 0): 17D Lie-invariant prior ──────────────────
    x_bar = p_sym[17:35]
    xb_p, xb_v, xb_q, xb_om = x_bar[0:3], x_bar[3:6], x_bar[6:10], x_bar[10:13]
    xb_m, xb_tau, xb_d = x_bar[13], x_bar[14], x_bar[15:18]
    Pinv = p_sym[35:52]   # 17D
    e_arr = vertcat(
        p_s  - xb_p,                  # 3
        v_s  - xb_v,                  # 3
        quat_err(xb_q, q_s),          # 3 (Lie-inv)
        om_s - xb_om,                 # 3
        m_s   - xb_m,                 # 1
        tau_s - xb_tau,               # 1
        d_s   - xb_d,                 # 3
    )  # 17D
    ocp.cost.cost_type_0 = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_0 = e_arr.T @ ca.diag(Pinv) @ e_arr

    # ── Terminal cost: measurement only ──────────────────────────────────
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = e_meas.T @ R_inv @ e_meas

    # ── Default params ───────────────────────────────────────────────────
    p_default = np.zeros(NP)
    p_default[6]      = 1.0   # y_q identity
    p_default[17+6]   = 1.0   # x̄_q identity
    p_default[35:52]  = 1.0   # P̄_inv unit
    ocp.parameter_values = p_default

    # ── State bounds: m, τ, d ────────────────────────────────────────────
    # state idx: m=13, τ=14, d=15,16,17
    lbx = np.array([M_MIN, TAU_MIN, D_MIN, D_MIN, D_MIN])
    ubx = np.array([M_MAX, TAU_MAX, D_MAX, D_MAX, D_MAX])
    idxbx = np.array([13, 14, 15, 16, 17])
    ocp.constraints.lbx,   ocp.constraints.ubx,   ocp.constraints.idxbx   = lbx, ubx, idxbx
    ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx
    ocp.constraints.lbx_0, ocp.constraints.ubx_0, ocp.constraints.idxbx_0 = lbx, ubx, idxbx

    # ── Process noise bounds ─────────────────────────────────────────────
    ocp.constraints.lbu = np.array([-W_M_MAX, -W_TAU_MAX, -W_D_MAX, -W_D_MAX, -W_D_MAX])
    ocp.constraints.ubu = np.array([ W_M_MAX,  W_TAU_MAX,  W_D_MAX,  W_D_MAX,  W_D_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4])

    # ── Solver options (same robust recipe as the 21D MHE) ───────────────
    ocp.solver_options.qp_solver            = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N     = N // 4
    ocp.solver_options.qp_solver_iter_max   = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx       = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method    = "PROJECT_REDUC_HESS"
    ocp.solver_options.levenberg_marquardt  = 1e-1
    ocp.solver_options.hpipm_mode           = "ROBUST"
    ocp.solver_options.integrator_type      = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 1
    ocp.solver_options.nlp_solver_type      = "SQP_RTI"
    ocp.solver_options.tol                  = 1e-3
    ocp.solver_options.tf                   = N * DT
    return ocp


def main():
    ocp = build_ocp()
    code_dir = ocp.code_export_directory
    if os.path.isdir(code_dir):
        shutil.rmtree(code_dir)
    json_file = os.path.join(os.path.dirname(code_dir), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(json_file):
        os.remove(json_file)
    print(f"Generating NMPC-MHE (18D) C code in {code_dir} ...")
    print(f"  NX={NX}, NU={NU}(noise), NP={NP}, N={N}, T={N*DT:.2f}s")
    print(f"  Thrust + ω_cmd are KNOWN inputs (params), not states.")
    AcadosOcpSolver(ocp, json_file=json_file)
    print(f"\nJSON: {json_file}\nNow: cd ../build && cmake .. && make -j$(nproc)")


if __name__ == "__main__":
    main()
