"""
Generate acados C code for a Lie-invariant Moving Horizon Estimator (MHE).

Augmented state  x_a ∈ ℝ²¹ = [p(3), v(3), q(4), ω(3), θ, v_θ, f, m, τ_rc, d(3)]
                               0:3   3:6   6:10  10:13  13  14   15  16   17    18:21
Process noise    w   ∈ ℝ⁵  = [w_m, w_τ, w_d(3)]   (MHE "controls")

Runtime params per stage: p ∈ ℝ⁵⁹
  p[ 0:13] = y_k   measurement [p(3), v(3), q(4), ω(3)]
  p[13:18] = u_k   applied control [Δf, ωx, ωy, ωz, a_θ]
  p[18:39] = x̄    prior state (21D) — used in arrival cost (stage 0)
  p[39:59] = P̄_inv arrival cost diagonal weights (20D Lie-invariant error)

Fixed noise covariances (baked in, not runtime params):
  R_inv: measurement noise inverse — r_p=100, r_v=10, r_q=50, r_ω=5
  Q_w_inv: process noise inverse — q_wm=1000, q_wτ=1000, q_wd=100

Horizon: N = 20, dt = 0.01 s (uniform, 100 Hz → 0.2 s estimation window)
Solver: SQP_RTI, PARTIAL_CONDENSING_HPIPM, ERK4
Output: ../c_generated_code_mhe/

Run once: python3 generate_mhe_ocp.py
"""

import os
import shutil
import numpy as np
import casadi as ca
from casadi import MX, vertcat, norm_2, if_else, atan2, dot
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# ── Local imports ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from quad_config import MASS, G, TAU_RC, T_MAX, T_MIN, W_MAX, VTHETA_MAX, S_MAX_MANUAL, THETA_MARGIN

# ── MHE dimensions ────────────────────────────────────────────────────────────
NX  = 21   # augmented state: [p,v,q,ω,θ,vθ,f,m,τ,d]
NU  = 5    # process noise: [w_m, w_τ, w_d(3)]
NP  = 59   # runtime params per stage: [y(13), u_applied(5), x̄(21), P̄_inv(20)]
N   = 20   # MHE horizon length
DT  = 0.01 # [s] control period (100 Hz)

# Fixed measurement noise inverse (diagonal R⁻¹, 12×12)
R_P  = 100.0   # position noise inverse [1/m²]
R_V  = 10.0    # velocity noise inverse [1/(m/s)²]
R_Q  = 50.0    # quaternion-log noise inverse [1/rad²]
R_OM = 5.0     # angular velocity noise inverse [1/(rad/s)²]
R_INV_DIAG = np.array([R_P]*3 + [R_V]*3 + [R_Q]*3 + [R_OM]*3)  # 12D

# Fixed process noise inverse (diagonal Q_w⁻¹, 5×5)
# w = [w_m [kg/s], w_τ [1/s], w_d [m/s³ × 3]]
# These must stay loose enough for the MHE to converge from a wrong initial
# guess (m̂_init=0.9 vs true=1.08).  Output smoothing is handled by the EMA
# filter in mhe_mpcc_sil.cpp, NOT by tightening Q_w here.
QW_M   = 1.0 / 0.001   # mass noise: std ≈ 0.032 kg/s per step
QW_TAU = 1.0 / 0.001   # τ_rc noise: std ≈ 0.032 1/s per step
QW_D   = 1.0 / 0.01    # disturbance noise: std ≈ 0.1 m/s³ per step
QW_INV_DIAG = np.array([QW_M, QW_TAU, QW_D, QW_D, QW_D])  # 5D

# Process noise bounds (limits extreme drifts)
W_M_MAX   = 0.1      # [kg/s]
W_TAU_MAX = 0.01     # [1/s]
W_D_MAX   = 2.0      # [m/s³]

# Parameter bounds
M_MIN = 0.5;    M_MAX = 3.0     # [kg]
TAU_MIN = 0.005; TAU_MAX = 0.15 # [s]
D_MIN = -8.0;   D_MAX = 8.0     # [m/s²]


# ══════════════════════════════════════════════════════════════════════════════
#  Quaternion helpers (CasADi symbolic)
# ══════════════════════════════════════════════════════════════════════════════

def quat_mult(q1, q2):
    """q1 ⊗ q2, both [qw, qx, qy, qz]."""
    w0, x0, y0, z0 = q1[0], q1[1], q1[2], q1[3]
    w1, x1, y1, z1 = q2[0], q2[1], q2[2], q2[3]
    return vertcat(
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    )


def quat_inv(q):
    """q⁻¹ = [qw, -qv] / ||q||²  (unit quaternion → conjugate)."""
    qn = norm_2(q)
    return vertcat(q[0], -q[1], -q[2], -q[3]) / (qn + 1e-12)


def quat_log(q):
    """SO(3) log map: Log(q) = 2·atan2(||qv||, qw)·qv/||qv||  (3D output).
    Hemisphere-corrected: ensures qw > 0 before computing log."""
    # Hemisphere correction: q and -q represent same rotation; enforce qw >= 0
    q_hc = if_else(q[0] < 0, -q, q)
    q_w  = q_hc[0]
    q_v  = q_hc[1:]
    # Epsilon INSIDE the sqrt → finite Jacobian at qv=0 (identity). Using
    # norm_2(qv)+1e-9 gives a 0/0=NaN derivative at qv=0 → NaN Hessian →
    # QP_FAILURE/MINSTEP whenever attitude error ≈ 0 (low-tilt / level). This
    # was making the MHE fail ~99% and the SiL fallback masked it.
    nv   = ca.sqrt(ca.dot(q_v, q_v) + 1e-12)
    theta = atan2(nv, q_w)
    return 2.0 * q_v * theta / nv


def quat_err(q_meas, q_state):
    """Lie-invariant measurement residual: Log(q_meas⁻¹ ⊗ q_state)."""
    q_rel = quat_mult(quat_inv(q_meas), q_state)
    return quat_log(q_rel)   # 3D


def arrival_quat_err(q_bar, q_state):
    """Lie-invariant arrival error: Log(q̄⁻¹ ⊗ q_state)."""
    q_rel = quat_mult(quat_inv(q_bar), q_state)
    return quat_log(q_rel)   # 3D


# ══════════════════════════════════════════════════════════════════════════════
#  Build 21-state augmented quadrotor dynamics for MHE
# ══════════════════════════════════════════════════════════════════════════════

def build_mhe_model():
    """Build 21-state augmented quadrotor model.

    State:   x_a = [p(3), v(3), q(4), ω(3), θ, vθ, f, m, τ_rc, d(3)]
    Control: w   = [w_m, w_τ, w_d(3)]  (process noise — MHE 'controls')
    Params:  p   = [y_k(13), u_k(5), x̄(21), P̄_inv(20)]  (per-stage runtime)

    Applied control u_k comes from params p[13:18]:
      p[13]    = Δf (thrust rate applied by MPCC)
      p[14:17] = ω_cmd (body rate commands)
      p[17]    = a_θ  (progress acceleration)
    """
    model = AcadosModel()
    model.name = "quadrotor_mhe"

    e3 = MX([0, 0, 1])

    # ── States (21) ─────────────────────────────────────────────────────
    p_s    = MX.sym("p",    3)   # position [m]
    v_s    = MX.sym("v",    3)   # velocity [m/s]
    q_s    = MX.sym("q",    4)   # quaternion [qw,qx,qy,qz]
    om_s   = MX.sym("om",   3)   # angular velocity [rad/s]
    theta  = MX.sym("theta")     # arc-length progress [m]
    vtheta = MX.sym("vtheta")    # progress velocity [m/s]
    f_s    = MX.sym("f")         # thrust state [N]
    m_s    = MX.sym("m")         # mass [kg]
    tau_s  = MX.sym("tau")       # rate-control time constant [s]
    d_s    = MX.sym("d",    3)   # external disturbance acceleration [m/s²]

    x = vertcat(p_s, v_s, q_s, om_s, theta, vtheta, f_s, m_s, tau_s, d_s)  # 21D

    # ── Process noise controls (5) ──────────────────────────────────────
    w_m   = MX.sym("w_m")       # mass drift rate [kg/s]
    w_tau = MX.sym("w_tau")     # τ_rc drift rate [1/s]
    w_d   = MX.sym("w_d", 3)   # disturbance drift rate [m/s³]
    u_noise = vertcat(w_m, w_tau, w_d)  # 5D

    # ── Runtime parameters (59) ─────────────────────────────────────────
    p_sym = MX.sym("p_rt", NP)
    # y_k measurement [0:13]
    y_p   = p_sym[0:3]    # position measurement
    y_v   = p_sym[3:6]    # velocity measurement
    y_q   = p_sym[6:10]   # quaternion measurement
    y_om  = p_sym[10:13]  # angular velocity measurement
    # u_k applied control [13:18]
    u_df     = p_sym[13]      # Δf (thrust rate from MPCC)
    u_om_cmd = p_sym[14:17]   # ω_cmd (body rate commands from MPCC)
    u_atheta = p_sym[17]      # a_θ (progress acceleration from MPCC)
    # x̄ prior state [18:39] (used only in arrival cost at stage 0)
    # P̄_inv diagonal [39:59] (20D Lie-invariant, used at stage 0)

    # ── Rotation matrix from quaternion ─────────────────────────────────
    qw, qx, qy, qz = q_s[0], q_s[1], q_s[2], q_s[3]
    qn  = norm_2(q_s) + 1e-12
    qnv = q_s / qn   # normalised
    q_hat = MX.zeros(3, 3)
    q_hat[0,1] = -q_s[3]; q_hat[0,2] =  q_s[2]
    q_hat[1,0] =  q_s[3]; q_hat[1,2] = -q_s[1]
    q_hat[2,0] = -q_s[2]; q_hat[2,1] =  q_s[1]
    Rot = MX.eye(3) + 2*q_hat@q_hat + 2*qnv[0]*q_hat

    # ── Dynamics ─────────────────────────────────────────────────────────
    # Translational
    dp = v_s
    dv = -G*e3 + (Rot @ vertcat(MX(0), MX(0), f_s)) / m_s + d_s

    # Quaternion kinematics: q̇ = ½ q ⊗ [0, ω]
    w0_, x0_, y0_, z0_ = q_s[0], q_s[1], q_s[2], q_s[3]
    om1 = vertcat(MX(0), om_s)
    dq = 0.5 * vertcat(
        w0_*om1[0] - x0_*om1[1] - y0_*om1[2] - z0_*om1[3],
        w0_*om1[1] + x0_*om1[0] + y0_*om1[3] - z0_*om1[2],
        w0_*om1[2] - x0_*om1[3] + y0_*om1[0] + z0_*om1[1],
        w0_*om1[3] + x0_*om1[2] - y0_*om1[1] + z0_*om1[0],
    )

    # Rate controller: ω̇ = (ω_cmd − ω) / τ_rc
    dom = (u_om_cmd - om_s) / (tau_s + 1e-6)

    # Virtual states (driven by applied MPCC control from params)
    dtheta  = vtheta
    dvtheta = u_atheta
    df      = u_df

    # Augmented parameter dynamics: random walk (driven by process noise)
    dm   = w_m
    dtau = w_tau
    dd   = w_d

    f_expl = vertcat(dp, dv, dq, dom, dtheta, dvtheta, df, dm, dtau, dd)

    # ── Implicit form for acados ─────────────────────────────────────────
    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x    = x
    model.xdot = x_dot
    model.u    = u_noise
    model.p    = p_sym

    return model, p_sym, y_p, y_v, y_q, y_om, p_s, v_s, q_s, om_s, theta, vtheta, f_s, m_s, tau_s, d_s


# ══════════════════════════════════════════════════════════════════════════════
#  Build MHE OCP
# ══════════════════════════════════════════════════════════════════════════════

def build_mhe_ocp():
    """Build the acados OCP for the Lie-invariant MHE."""

    ocp   = AcadosOcp()
    model, p_sym, y_p, y_v, y_q, y_om, p_s, v_s, q_s, om_s, theta_s, vtheta_s, f_s, m_s, tau_s, d_s = build_mhe_model()
    ocp.model = model

    # Code export
    code_dir = os.path.join(SCRIPT_DIR, "..", "c_generated_code_mhe")
    ocp.code_export_directory = code_dir

    ocp.solver_options.N_horizon = N
    u_noise = model.u   # process noise w ∈ ℝ⁵

    # ── Fixed R and Q_w matrices (baked into cost expressions) ──────────
    R_inv   = np.diag(R_INV_DIAG)   # 12×12
    QW_inv  = np.diag(QW_INV_DIAG)  # 5×5

    # ── Stage cost (stages 0..N-1): measurement residual + process noise ─
    # 12D measurement residual: [p-yp, v-yv, Log(yq⁻¹⊗q), ω-yω]
    e_meas = vertcat(
        p_s   - y_p,
        v_s   - y_v,
        quat_err(y_q, q_s),    # 3D log-map on SO(3)
        om_s  - y_om,
    )  # 12D

    stage_cost = (e_meas.T @ R_inv @ e_meas
                  + u_noise.T @ QW_inv @ u_noise)

    ocp.cost.cost_type   = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = stage_cost

    # ── Arrival cost (stage 0): Lie-invariant prior cost ─────────────────
    # Prior state x̄ from p[18:39] (21D)
    x_bar  = p_sym[18:39]
    xb_p   = x_bar[0:3]
    xb_v   = x_bar[3:6]
    xb_q   = x_bar[6:10]
    xb_om  = x_bar[10:13]
    xb_th  = x_bar[13]
    xb_vth = x_bar[14]
    xb_f   = x_bar[15]
    xb_m   = x_bar[16]
    xb_tau = x_bar[17]
    xb_d   = x_bar[18:21]

    # P̄⁻¹ diagonal (20D Lie-invariant error) from p[39:59]
    Pinv_diag = p_sym[39:59]

    # 20D Lie-invariant arrival error
    # Layout: [p_err(3), v_err(3), log_q_err(3), ω_err(3), θ_err, vθ_err, f_err, m_err, τ_err, d_err(3)]
    e_arr = vertcat(
        p_s    - xb_p,                          # 3
        v_s    - xb_v,                          # 3
        arrival_quat_err(xb_q, q_s),            # 3 (Lie-inv)
        om_s   - xb_om,                         # 3
        theta_s  - xb_th,                       # 1
        vtheta_s - xb_vth,                      # 1
        f_s    - xb_f,                          # 1
        m_s    - xb_m,                          # 1
        tau_s  - xb_tau,                        # 1
        d_s    - xb_d,                          # 3
    )  # 20D total

    arrival_cost = e_arr.T @ ca.diag(Pinv_diag) @ e_arr

    ocp.cost.cost_type_0 = "EXTERNAL"
    # IMPORTANT: cost_type_0 is evaluated ONLY at stage 0, while cost_type is evaluated
    # at stages 0..N-1. So at stage 0 the total cost is:
    #   cost_expr_ext_cost_0 + cost_expr_ext_cost
    # We put ONLY the arrival cost in cost_expr_ext_cost_0 to avoid double-counting
    # the stage cost at stage 0.
    ocp.model.cost_expr_ext_cost_0 = arrival_cost  # arrival cost only (no stage cost here)

    # ── Terminal cost (stage N): measurement only, no process noise ───────
    terminal_cost = e_meas.T @ R_inv @ e_meas

    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = terminal_cost

    # ── Default parameter values ─────────────────────────────────────────
    p_default = np.zeros(NP)
    p_default[6]  = 1.0   # y_q = identity quaternion [qw=1, qx=0, ...]
    p_default[18+6] = 1.0  # x̄_q = identity quaternion
    p_default[39:59] = 1.0  # P̄_inv = identity (unit weights)
    ocp.parameter_values = p_default

    # ── State bounds ─────────────────────────────────────────────────────
    # State layout: [p(3), v(3), q(4), ω(3), θ, vθ, f, m, τ, d(3)]
    # Indices:        0:3   3:6   6:10 10:13  13  14  15  16  17  18:21
    #
    # Constrain: θ≥0, vθ∈[0,vmax], f∈[T_min,T_max], m∈[0.5,3], τ∈[0.005,0.15], d∈[-8,8]
    lbx_vals = [0.0,        # θ
                0.0,        # vθ
                T_MIN,      # f
                M_MIN,      # m
                TAU_MIN,    # τ
                D_MIN, D_MIN, D_MIN]  # d(3)

    ubx_vals = [S_MAX_MANUAL + THETA_MARGIN,  # θ
                VTHETA_MAX,   # vθ
                T_MAX,        # f
                M_MAX,        # m
                TAU_MAX,      # τ
                D_MAX, D_MAX, D_MAX]  # d(3)

    idxbx = np.array([13, 14, 15, 16, 17, 18, 19, 20])

    ocp.constraints.lbx   = np.array(lbx_vals)
    ocp.constraints.ubx   = np.array(ubx_vals)
    ocp.constraints.idxbx = idxbx

    # Terminal state bounds (same)
    ocp.constraints.lbx_e   = np.array(lbx_vals)
    ocp.constraints.ubx_e   = np.array(ubx_vals)
    ocp.constraints.idxbx_e = idxbx

    # IMPORTANT for MHE: do NOT use ocp.constraints.x0 (equality constraint).
    # In MHE, stage 0 is the OLDEST state in the window — it is a FREE decision
    # variable that the solver optimizes. The arrival cost (P̄_inv) provides the
    # "soft" prior that attracts x_0 toward x̄. An equality constraint would
    # over-constrain the problem and cause QP infeasibility / NaN.
    # Apply the same physical box constraints at stage 0 as at other stages:
    ocp.constraints.lbx_0   = np.array(lbx_vals)
    ocp.constraints.ubx_0   = np.array(ubx_vals)
    ocp.constraints.idxbx_0 = idxbx

    # ── Process noise bounds (MHE 'controls') ────────────────────────────
    ocp.constraints.lbu = np.array([-W_M_MAX,  -W_TAU_MAX, -W_D_MAX, -W_D_MAX, -W_D_MAX])
    ocp.constraints.ubu = np.array([ W_M_MAX,   W_TAU_MAX,  W_D_MAX,  W_D_MAX,  W_D_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4])

    # ── Solver options ────────────────────────────────────────────────────
    ocp.solver_options.qp_solver            = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N     = N // 4   # = 5
    ocp.solver_options.qp_solver_iter_max   = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx       = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method    = "PROJECT_REDUC_HESS"
    ocp.solver_options.levenberg_marquardt  = 1e-2   # stronger LM damps ill-cond.
    ocp.solver_options.hpipm_mode           = "ROBUST"  # adds reg. to prevent MINSTEP
    ocp.solver_options.integrator_type      = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 1   # uniform dt=0.01s
    ocp.solver_options.nlp_solver_type      = "SQP_RTI"
    ocp.solver_options.tol                  = 1e-3
    ocp.solver_options.tf                   = N * DT   # = 0.2 s

    return ocp


def main():
    ocp = build_mhe_ocp()
    code_dir = ocp.code_export_directory

    # Clean previous build
    if os.path.isdir(code_dir):
        shutil.rmtree(code_dir)

    json_file = os.path.join(os.path.dirname(code_dir),
                              f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(json_file):
        os.remove(json_file)

    print(f"\nGenerating MHE solver C code in {code_dir} ...")
    print(f"  NX={NX}, NU={NU}(process noise), NP={NP}, N={N}, T={N*DT:.2f}s, dt={DT}s")
    print(f"  Measurement: p/v/q/ω (12D)")
    print(f"  Parameters:  m∈[{M_MIN},{M_MAX}]kg, τ∈[{TAU_MIN},{TAU_MAX}]s, d∈[{D_MIN},{D_MAX}]m/s²")

    solver = AcadosOcpSolver(ocp, json_file=json_file)

    print("\nDone! Generated files:")
    for f in sorted(os.listdir(code_dir)):
        print(f"  {f}")
    print(f"\nJSON: {json_file}")
    print(f"\nNow run: cd ../build && cmake .. && make -j$(nproc)")


if __name__ == "__main__":
    main()
