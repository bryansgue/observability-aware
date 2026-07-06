"""
Generate acados C code for a TRANSLATIONAL-ONLY MHE (mass + disturbance).

Attitude is MEASURED, not estimated (no quaternion state → no log-map
singularity). The thrust direction a = R·e3 and magnitude T enter as KNOWN
INPUTS. The estimator integrates only the translational dynamics.

State   x ∈ ℝ¹⁰ = [p(3), v(3), m, d(3)]
Noise   w ∈ ℝ⁴  = [w_m, w_d(3)]   (MHE "controls")
Inputs (params, known): T (thrust), a = R·e3 (world thrust direction)

Dynamics:
  ṗ = v
  v̇ = -g·e₃ + (T/m)·a + d - c·v·|v|
  ṁ = w_m,  ḋ = w_d

Runtime params p ∈ ℝ³⁴:
  p[0:6]   = y_k  [p(3), v(3)]
  p[6]     = T
  p[7:10]  = a = R·e3
  p[10:20] = x̄  prior (10D)
  p[20:30] = P̄_inv (10D)
  p[30]    = c_drag
  p[31:34] = sf_meas (accelerometer)
  p[34]    = meas_mask (1 = odometry available, 0 = IMU-only → drift)

N=31, dt=0.01 (310 ms window, 100 Hz). Output: ../c_generated_code_mhe_trans/
"""
import os, shutil
import numpy as np
from casadi import MX, vertcat, norm_2, sqrt, dot
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
from quad_config import G

NX = 10
NU = 4
NP = 35   # +1: per-node odometry measurement mask (sparse-position experiments)
N  = 31
DT = 0.01

R_P = 50.0
R_VXY, R_VZ = 10.0, 10.0
W_IMU = 2.0
QW_M    = float(os.environ.get("MHE_QW_M",  1.0/0.001))
QW_D    = float(os.environ.get("MHE_QW_D",  1.0e3))
W_D_MAX = float(os.environ.get("MHE_WD_MAX", 0.5))
D_MAX   = float(os.environ.get("MHE_D_MAX",  2.0))
W_M_MAX = float(os.environ.get("MHE_WM_MAX", 0.1))
M_MIN, M_MAX = 0.5, 3.0
D_MIN = -D_MAX


def build_model():
    model = AcadosModel()
    model.name = "quadrotor_mhe_trans"
    e3 = MX([0, 0, 1])

    p_s = MX.sym("p", 3)
    v_s = MX.sym("v", 3)
    m_s = MX.sym("m")
    d_s = MX.sym("d", 3)
    x = vertcat(p_s, v_s, m_s, d_s)

    w_m = MX.sym("w_m")
    w_d = MX.sym("w_d", 3)
    u_noise = vertcat(w_m, w_d)

    p_sym = MX.sym("p_rt", NP)
    T   = p_sym[6]
    a   = p_sym[7:10]
    c_drag = p_sym[30]

    dp = v_s
    vmag = sqrt(dot(v_s, v_s) + 1e-6)
    dv = -G*e3 + (T / m_s) * a + d_s - c_drag * v_s * vmag
    f_expl = vertcat(dp, dv, w_m, w_d)

    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x, model.xdot, model.u, model.p = x, x_dot, u_noise, p_sym
    return model, p_sym, p_s, v_s, m_s, d_s


def build_ocp():
    ocp = AcadosOcp()
    model, p_sym, p_s, v_s, m_s, d_s = build_model()
    ocp.model = model
    ocp.code_export_directory = os.path.join(SCRIPT_DIR, "..", "c_generated_code_mhe_trans")
    ocp.solver_options.N_horizon = N
    u_noise = model.u

    y_p, y_v = p_sym[0:3], p_sym[3:6]
    R_inv  = np.diag([R_P]*3 + [R_VXY, R_VXY, R_VZ])
    QW_inv = np.diag([QW_M, QW_D, QW_D, QW_D])

    e_meas = vertcat(p_s - y_p, v_s - y_v)
    ocp.cost.cost_type = "EXTERNAL"

    meas_mask = p_sym[34]   # 1 = odometry available at this node, 0 = IMU-only (drift)
    sf_meas = p_sym[31:34]
    T_p, a_p, cdrag_p = p_sym[6], p_sym[7:10], p_sym[30]
    vmag_c = sqrt(dot(v_s, v_s) + 1e-6)
    sf_model = (T_p / m_s) * a_p + d_s - cdrag_p * v_s * vmag_c
    r_imu = sf_meas - sf_model

    ocp.model.cost_expr_ext_cost = (meas_mask * (e_meas.T @ R_inv @ e_meas)
                                    + u_noise.T @ QW_inv @ u_noise
                                    + W_IMU * dot(r_imu, r_imu))

    x_bar = p_sym[10:20]
    Pinv  = p_sym[20:30]
    e_arr = vertcat(p_s - x_bar[0:3], v_s - x_bar[3:6],
                    m_s - x_bar[6], d_s - x_bar[7:10])
    ocp.cost.cost_type_0 = "EXTERNAL"
    arrival = Pinv[0]*e_arr[0]**2
    for i in range(1, NX):
        arrival = arrival + Pinv[i]*e_arr[i]**2
    ocp.model.cost_expr_ext_cost_0 = arrival

    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = (meas_mask * (e_meas.T @ R_inv @ e_meas)
                                      + W_IMU * dot(r_imu, r_imu))

    p_default = np.zeros(NP)
    p_default[6]  = 9.81 * 1.0
    p_default[9]  = 1.0
    p_default[16] = 1.0
    p_default[20:30] = 1.0
    p_default[30] = 0.0
    p_default[33] = 9.81
    p_default[34] = 1.0   # odometry measurement ON by default
    ocp.parameter_values = p_default

    lbx = np.array([M_MIN, D_MIN, D_MIN, D_MIN])
    ubx = np.array([M_MAX, D_MAX, D_MAX, D_MAX])
    idxbx = np.array([6, 7, 8, 9])
    ocp.constraints.lbx,   ocp.constraints.ubx,   ocp.constraints.idxbx   = lbx, ubx, idxbx
    ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx
    ocp.constraints.lbx_0, ocp.constraints.ubx_0, ocp.constraints.idxbx_0 = lbx, ubx, idxbx

    ocp.constraints.lbu = np.array([-W_M_MAX, -W_D_MAX, -W_D_MAX, -W_D_MAX])
    ocp.constraints.ubu = np.array([ W_M_MAX,  W_D_MAX,  W_D_MAX,  W_D_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

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
    print(f"Generating TRANS-MHE (10D: p,v,m,d) in {code_dir} ...")
    AcadosOcpSolver(ocp, json_file=json_file)
    print(f"JSON: {json_file}\nNow: cd ../build && cmake .. && make")


if __name__ == "__main__":
    main()
