"""
Generate acados C code for a DISTURBANCE-ONLY translational MHE (EXP-2).

Difference vs generate_mhe_trans_ocp.py (EXP-1): the mass is NOT a state here.
It was already identified in EXP-1, so it enters as a FIXED PARAMETER. The
estimator integrates only [p, v, d] and estimates ONLY the external disturbance
d. With m fixed, the m–d_z ambiguity (Prop. 1) disappears: d_z becomes
observable and cannot be polluted by a drifting mass.

State   x ∈ ℝ⁹ = [p(3), v(3), d(3)]
Noise   w ∈ ℝ³ = [w_d(3)]          (MHE "controls")
Inputs (params, known): T (thrust), a = R·e3 (world thrust direction), m (mass)

Dynamics:
  ṗ = v
  v̇ = -g·e₃ + (T/m)·a + d - c_drag·v·|v|     (m FIXED param, not a state)
  ḋ = w_d

Runtime params p ∈ ℝ³³:
  p[0:6]   = y_k  measurement [p(3), v(3)]
  p[6]     = T
  p[7:10]  = a = R·e3
  p[10:19] = x̄  prior (9D)
  p[19:28] = P̄_inv arrival weights (9D)
  p[28]    = c_drag
  p[29:32] = sf_meas (IMU specific force, world frame)
  p[32]    = m  (FIXED mass, identified offline)

Output: ../c_generated_code_mhe_trans_d/
"""
import os, shutil
import numpy as np
from casadi import MX, vertcat, sqrt, dot
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
from quad_config import G

NX = 9    # [p,v,d]  — NO mass state
NU = 3    # [w_d(3)]
NP = 33   # [y(6), T(1), a(3), x̄(9), P̄_inv(9), c_drag(1), sf_meas(3), m(1)]
N  = 31
DT = 0.01   # 10 ms grid → pure 100 Hz MHE (310 ms window)

R_P = 50.0
R_VXY, R_VZ = 10.0, 10.0
W_IMU = 2.0
QW_D    = float(os.environ.get("MHE_QW_D",  1.0e3))   # penalty on d motion
W_D_MAX = float(os.environ.get("MHE_WD_MAX", 0.5))    # rate bound on d
D_MAX   = float(os.environ.get("MHE_D_MAX",  2.0))    # d magnitude bound
D_MIN   = -D_MAX


def build_model():
    model = AcadosModel()
    model.name = "quadrotor_mhe_trans_d"
    e3 = MX([0, 0, 1])

    p_s = MX.sym("p", 3)
    v_s = MX.sym("v", 3)
    d_s = MX.sym("d", 3)
    x = vertcat(p_s, v_s, d_s)

    w_d = MX.sym("w_d", 3)
    u_noise = w_d

    p_sym = MX.sym("p_rt", NP)
    T   = p_sym[6]
    a   = p_sym[7:10]   # R·e3 (world thrust direction)
    c_drag = p_sym[28]  # quadratic drag accel coeff
    m_fix  = p_sym[32]  # FIXED mass (identified offline) — a parameter, not a state

    dp = v_s
    vmag = sqrt(dot(v_s, v_s) + 1e-6)
    dv = -G*e3 + (T / m_fix) * a + d_s - c_drag * v_s * vmag
    dd = w_d
    f_expl = vertcat(dp, dv, dd)

    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x, model.xdot, model.u, model.p = x, x_dot, u_noise, p_sym
    return model, p_sym, p_s, v_s, d_s


def build_ocp():
    ocp = AcadosOcp()
    model, p_sym, p_s, v_s, d_s = build_model()
    ocp.model = model
    ocp.code_export_directory = os.path.join(SCRIPT_DIR, "..", "c_generated_code_mhe_trans_d")
    ocp.solver_options.N_horizon = N
    u_noise = model.u

    R_inv  = np.diag([R_P]*3 + [R_VXY, R_VXY, R_VZ])   # 6x6
    QW_inv = np.diag([QW_D, QW_D, QW_D])               # 3x3

    y_p = p_sym[0:3]
    y_v = p_sym[3:6]
    e_meas = vertcat(p_s - y_p, v_s - y_v)      # 6D

    # IMU specific-force residual (momentum observer) — m is the FIXED param.
    sf_meas = p_sym[29:32]
    T_p, a_p, cdrag_p, m_fix = p_sym[6], p_sym[7:10], p_sym[28], p_sym[32]
    vmag_c = sqrt(dot(v_s, v_s) + 1e-6)
    sf_model = (T_p / m_fix) * a_p + d_s - cdrag_p * v_s * vmag_c
    r_imu = sf_meas - sf_model

    ocp.cost.cost_type = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = (e_meas.T @ R_inv @ e_meas
                                    + u_noise.T @ QW_inv @ u_noise
                                    + W_IMU * dot(r_imu, r_imu))

    x_bar = p_sym[10:19]
    Pinv  = p_sym[19:28]
    e_arr = vertcat(p_s - x_bar[0:3], v_s - x_bar[3:6], d_s - x_bar[6:9])  # 9D
    ocp.cost.cost_type_0 = "EXTERNAL"
    arrival = Pinv[0]*e_arr[0]**2
    for i in range(1, 9):
        arrival = arrival + Pinv[i]*e_arr[i]**2
    ocp.model.cost_expr_ext_cost_0 = arrival

    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = (e_meas.T @ R_inv @ e_meas
                                      + W_IMU * dot(r_imu, r_imu))

    p_default = np.zeros(NP)
    p_default[6]  = 9.81 * 1.0     # T nominal
    p_default[9]  = 1.0            # a = e3 (level)
    p_default[19:28] = 1.0         # P̄_inv unit
    p_default[28] = 0.0            # c_drag default 0
    p_default[31] = 9.81           # sf_meas nominal = g·e3
    p_default[32] = 1.05           # m fixed default (identified mass)
    ocp.parameter_values = p_default

    # bounds on d (state idx 6,7,8)
    lbx = np.array([D_MIN, D_MIN, D_MIN])
    ubx = np.array([D_MAX, D_MAX, D_MAX])
    idxbx = np.array([6, 7, 8])
    ocp.constraints.lbx,   ocp.constraints.ubx,   ocp.constraints.idxbx   = lbx, ubx, idxbx
    ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx
    ocp.constraints.lbx_0, ocp.constraints.ubx_0, ocp.constraints.idxbx_0 = lbx, ubx, idxbx

    ocp.constraints.lbu = np.array([-W_D_MAX, -W_D_MAX, -W_D_MAX])
    ocp.constraints.ubu = np.array([ W_D_MAX,  W_D_MAX,  W_D_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    ocp.solver_options.qp_solver            = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N     = N // 4
    ocp.solver_options.qp_solver_iter_max   = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx       = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method    = "PROJECT_REDUC_HESS"
    ocp.solver_options.levenberg_marquardt  = 1e-2
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
    print(f"Generating TRANS-MHE-D (9D, mass=param) in {code_dir} ...")
    AcadosOcpSolver(ocp, json_file=json_file)
    print(f"JSON: {json_file}\nNow: cd ../build && cmake .. && make")


if __name__ == "__main__":
    main()
