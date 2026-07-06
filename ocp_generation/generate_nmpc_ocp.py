"""
Generate acados C code for a quadrotor NMPC (waypoint tracking).

State x ∈ ℝ¹³ = [p(3), v(3), q(4), ω(3)]
Control u ∈ ℝ⁴ = [T, ωx_cmd, ωy_cmd, ωz_cmd]

Adaptive plant (model params injected at runtime from the MHE):
  v̇ = -g·e₃ + (R·[0,0,T])/m̂ + d̂      (m̂ = estimated mass, d̂ = disturbance)
  ω̇ = k̂_τ·(ω_cmd − ω)                  (k̂_τ = 1/τ̂_rc)

Cost: external, with runtime parameters for reference + weights + model params.
  p = [p_ref(3), q_ref(4), Q_pos(3), Q_att(3), R_u(4), m̂(1), d̂(3), k̂_τ(1)]  → 22 params

Run once:  python3 generate_nmpc_ocp.py
Output:    ../c_generated_code_nmpc/
"""

import os
import sys
import shutil
import numpy as np
from casadi import MX, vertcat, norm_2, if_else, atan2, Function, diag, sqrt, dot
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# ── Physical parameters ──────────────────────────────────────────────────────
MASS   = 1.08
G      = 9.81
TAU_RC = 0.03
T_MAX  = 5.0 * G
W_MAX  = 20.0

# ── OCP dimensions ───────────────────────────────────────────────────────────
NX = 13
NU = 4
DT_CONTROL = 0.01   # [s] control LOOP period (100 Hz) — independent of node dt
T_HORIZON  = 1.5    # [s] prediction horizon
N_HORIZON  = 31     # Jetson-runnable; node dt = 1.5/31 ≈ 0.048 s
N_PARAMS   = 28     # p_ref(3)+q_ref(4)+Q_pos(3)+Q_att(3)+R_u(4)+m̂(1)+d̂(3)+k̂_τ(1) + v_ref(3)+Q_vel(3)


def build_quadrotor_model():
    """Build 13-state rate-control quadrotor model."""
    model = AcadosModel()
    model.name = "quadrotor_nmpc"

    # States
    p = MX.sym("p", 3)     # position (inertial)
    v = MX.sym("v", 3)     # velocity (inertial)
    q = MX.sym("q", 4)     # quaternion [qw, qx, qy, qz]
    w = MX.sym("w", 3)     # angular velocity (body)
    x = vertcat(p, v, q, w)

    # Controls
    T      = MX.sym("T")          # thrust [N]
    w_cmd  = MX.sym("w_cmd", 3)   # rate command [rad/s]
    u = vertcat(T, w_cmd)

    # Runtime parameters: [p_ref(3), q_ref(4), Q_pos(3), Q_att(3), R_u(4), m̂, d̂(3), k̂_τ]
    p_sym = MX.sym("p_runtime", N_PARAMS)
    m_hat = p_sym[17]        # estimated mass [kg]
    d_hat = p_sym[18:21]     # estimated disturbance accel [m/s²]
    k_tau = p_sym[21]        # estimated 1/τ_rc [1/s]
    model.p = p_sym

    # Quaternion → rotation matrix
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    q_hat = MX.zeros(3, 3)
    q_hat[0,1] = -qz;  q_hat[0,2] =  qy
    q_hat[1,0] =  qz;  q_hat[1,2] = -qx
    q_hat[2,0] = -qy;  q_hat[2,1] =  qx
    qn = norm_2(q)
    q_normed = q / qn
    Rot = MX.eye(3) + 2*q_hat@q_hat + 2*q_normed[0]*q_hat

    e3 = MX([0, 0, 1])

    # Dynamics (model params m̂, d̂, k̂_τ injected at runtime from the MHE)
    dp = v
    dv = -e3 * G + (Rot @ vertcat(MX(0), MX(0), T)) / m_hat + d_hat
    # Quaternion kinematics: q̇ = ½ q ⊗ [0, ω]
    omega_quat = vertcat(MX(0), w)
    w0, x0, y0, z0 = q[0], q[1], q[2], q[3]
    w1, x1, y1, z1 = omega_quat[0], omega_quat[1], omega_quat[2], omega_quat[3]
    dq = 0.5 * vertcat(
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    )
    # Rate controller (k̂_τ = 1/τ̂_rc, linear in the parameter → well-conditioned)
    dw = k_tau * (w_cmd - w)

    f_expl = vertcat(dp, dv, dq, dw)

    # Implicit form
    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u

    return model, p_sym


def build_nmpc_ocp():
    """Build the acados OCP for NMPC waypoint tracking."""
    ocp = AcadosOcp()
    model, p_sym = build_quadrotor_model()
    ocp.model = model

    # Code export directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    code_dir = os.path.join(script_dir, "..", "c_generated_code_nmpc")
    ocp.code_export_directory = code_dir

    # Dimensions
    ocp.solver_options.N_horizon = N_HORIZON

    # Runtime parameters (p_sym created in build_quadrotor_model; model.p already set):
    # [p_ref(3), q_ref(4), Q_pos(3), Q_att(3), R_u(4), m̂(1), d̂(3), k̂_τ(1)]
    p_ref  = p_sym[0:3]
    q_ref  = p_sym[3:7]
    Q_pos  = p_sym[7:10]
    Q_att  = p_sym[10:13]
    R_u    = p_sym[13:17]
    m_hat  = p_sym[17]
    v_ref  = p_sym[22:25]   # reference velocity (feedforward → kills tracking lag at speed)
    Q_vel  = p_sym[25:28]

    # ── Cost function ────────────────────────────────────────────────────────
    ocp.cost.cost_type   = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Position error
    e_pos = model.x[0:3] - p_ref
    # Velocity error (reference-velocity feedforward → tracks the path at speed)
    e_vel = model.x[3:6] - v_ref

    # Quaternion error: q_err = q_real⁻¹ ⊗ q_desired
    q_real = model.x[6:10]
    q_real_inv = vertcat(q_real[0], -q_real[1], -q_real[2], -q_real[3]) / norm_2(q_real)
    q_err = vertcat(
        q_real_inv[0]*q_ref[0] - q_real_inv[1]*q_ref[1] - q_real_inv[2]*q_ref[2] - q_real_inv[3]*q_ref[3],
        q_real_inv[0]*q_ref[1] + q_real_inv[1]*q_ref[0] + q_real_inv[2]*q_ref[3] - q_real_inv[3]*q_ref[2],
        q_real_inv[0]*q_ref[2] - q_real_inv[1]*q_ref[3] + q_real_inv[2]*q_ref[0] + q_real_inv[3]*q_ref[1],
        q_real_inv[0]*q_ref[3] + q_real_inv[1]*q_ref[2] - q_real_inv[2]*q_ref[1] + q_real_inv[3]*q_ref[0],
    )
    # Log map. Epsilon INSIDE the sqrt → finite Jacobian at q_err = identity
    # (qv=0). With norm_2(qv)+1e-9 the derivative is 0/0 = NaN at qv=0 → NaN
    # Hessian → QP_FAILURE whenever the attitude error is ~zero (e.g. hovering
    # level with a level reference). Same fix as the MHE quat_log.
    q_err_w = if_else(q_err[0] < 0, -q_err, q_err)
    qv = q_err_w[1:]
    nqv = sqrt(dot(qv, qv) + 1e-12)   # safe norm: finite Jacobian at qv=0
    theta_q = atan2(nqv, q_err_w[0])
    log_q = 2.0 * qv * theta_q / nqv

    # ── Residual vectors ─────────────────────────────────────────────────────
    # Control deviation: u_err = [T - T_hover, ωx, ωy, ωz]
    # Center on the ESTIMATED hover thrust m̂·g so a wrong nominal mass does not
    # bias the regularisation away from the true equilibrium thrust.
    T_hover = m_hat * G
    u_err = vertcat(model.u[0] - T_hover, model.u[1], model.u[2], model.u[3])

    # ── Weight matrices (diagonal) ────────────────────────────────────────────
    #   Q_p ∈ R^{3×3},  Q_a ∈ R^{3×3},  R ∈ R^{4×4}
    Q_p = diag(Q_pos)
    Q_a = diag(Q_att)
    Q_v = diag(Q_vel)
    R   = diag(R_u)

    # ── Quadratic cost: e^T W e ───────────────────────────────────────────────
    #
    #   ℓ(x,u) = e_pos^T Q_p e_pos  +  log_q^T Q_a log_q  +  u_err^T R u_err
    #   ℓ_e(x) = e_pos^T Q_p e_pos  +  log_q^T Q_a log_q
    #
    stage_cost    = (e_pos.T  @ Q_p @ e_pos
                   + e_vel.T  @ Q_v @ e_vel
                   + log_q.T  @ Q_a @ log_q
                   + u_err.T  @ R   @ u_err)

    terminal_cost = (e_pos.T  @ Q_p @ e_pos
                   + e_vel.T  @ Q_v @ e_vel
                   + log_q.T  @ Q_a @ log_q)

    ocp.model.cost_expr_ext_cost   = stage_cost
    ocp.model.cost_expr_ext_cost_e = terminal_cost

    # Default parameter values (sane: identity quat ref, nominal model params).
    # m̂ MUST default to a nonzero value (it divides T in the dynamics).
    p_default = np.zeros(N_PARAMS)
    p_default[3]  = 1.0          # q_ref = identity quaternion [qw=1, ...]
    p_default[17] = MASS         # m̂ default = nominal mass
    p_default[21] = 1.0 / TAU_RC # k̂_τ default = 1/τ_rc nominal
    p_default[25:28] = 10.0      # Q_vel default
    ocp.parameter_values = p_default

    # ── Constraints ──────────────────────────────────────────────────────────
    ocp.constraints.lbu = np.array([0.0,  -W_MAX, -W_MAX, -W_MAX])
    ocp.constraints.ubu = np.array([T_MAX, W_MAX,  W_MAX,  W_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    x0 = np.zeros(NX)
    x0[0] = 2.5; x0[2] = 1.5; x0[6] = 1.0  # pos + identity quat
    ocp.constraints.x0 = x0

    # ── Solver options ───────────────────────────────────────────────────────
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N = N_HORIZON // 4
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method = "CONVEXIFY"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.tf = T_HORIZON

    return ocp


def main():
    ocp = build_nmpc_ocp()
    code_dir = ocp.code_export_directory

    # Clean previous build
    if os.path.isdir(code_dir):
        shutil.rmtree(code_dir)

    json_file = os.path.join(os.path.dirname(code_dir),
                              f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(json_file):
        os.remove(json_file)

    print(f"Generating NMPC solver C code in {code_dir} ...")
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    print("Done! Generated files:")
    for f in sorted(os.listdir(code_dir)):
        print(f"  {f}")
    print(f"\nJSON: {json_file}")
    print(f"\nNow run: cd .. && colcon build")


if __name__ == "__main__":
    main()
