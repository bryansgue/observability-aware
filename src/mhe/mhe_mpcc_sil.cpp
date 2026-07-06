/**
 * MHE-MPCC SiL — Lie-invariant MHE + Adaptive W_s + MPCC with MuJoCo feedback.
 *
 * Architecture (each step at 100 Hz):
 *   1. MHE: push measurement y_k + applied control u_{k-1}
 *   2. MHE: solve() → x̂_k (21D estimate incl. m̂, τ̂_rc, d̂)
 *   3. MHE: propagate_prior() → update P̄ diagonal + σ_k = tr(P̄_θ,k)
 *   4. Adaptive W_s: W_s(k) = W_s_max / (1 + α · σ_k)
 *   5. MPCC: set_params_all(w_adaptive)  (Q_s = W_s(k))
 *   6. MPCC: set_x0 from MHE estimate + virtual states
 *   7. MPCC: solve() → u* = [Δf, ω_cmd, a_θ]
 *   8. Euler: integrate virtual states [θ, v_θ, f] at 100 Hz
 *   9. Send: f (thrust state) + ω_cmd to MuJoCo
 *
 * Protocol: RELOAD → PD HOLD → RUN → stabilise
 *
 * CSV: extended with m_hat, tau_hat, dx/y/z_hat, sigma_k, W_s,
 *      theta, vtheta, a_theta, s_nearest, mhe_ms, mhe_status
 *
 * Usage:
 *   ./mhe_mpcc_sil <vtheta_max> [v1|v2]
 *   ./mhe_mpcc_sil 10 v1        → open-loop:  MPCC uses wrong mass (0.9 kg) always
 *   ./mhe_mpcc_sil 10 v2        → closed-loop: MPCC corrects mass via MHE (default)
 *   ./mhe_mpcc_sil 10           → same as v2
 */

#include "quadrotor_mpc/mpcc/mpcc_controller.hpp"
#include "quadrotor_mpc/mhe/mhe_controller.hpp"
#include "quadrotor_mpc/mujoco/mujoco_interface.hpp"
#include "quadrotor_mpc/mujoco/sil_protocol.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"
#include "quadrotor_mpc/common/params.hpp"
#include "quadrotor_mpc/trajectory/lissajous.hpp"
#include "quadrotor_mpc/trajectory/arc_length.hpp"
#include "quadrotor_mpc/trajectory/attitude_reference.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>

using namespace quadrotor_mpc;

// ── Adaptive W_s parameters ──────────────────────────────────────────────────
static constexpr double ALPHA_WS = 10.0;   // sensitivity: W_s ~ W_s_max when σ_k ≈ 0

// ── MHE initial prior (deliberately wrong to demonstrate convergence) ────────
static constexpr double M_HAT_INIT   = 0.9;   // [kg] — true is 1.08 (17% error)
static constexpr double TAU_HAT_INIT = 0.03;  // [s]  — same as true
// disturbance prior: zero (correct — no disturbance in nominal SiL)

// ── EMA smoothing for parameter estimates ────────────────────────────────────
// x_ema[k] = α * x_ema[k-1] + (1-α) * x_mhe[k]
// Time constant: τ = -dt/ln(α)  (dt=0.01s)
//   α=0.98 → τ≈0.50s  (mass, τ_rc — slow parameters)
//   α=0.92 → τ≈0.12s  (disturbances — faster-changing)
static constexpr double EMA_ALPHA_PARAM = 0.98;  // mass + τ_rc
static constexpr double EMA_ALPHA_DIST  = 0.92;  // disturbances

// ── Load MPCC tuned weights from JSON ───────────────────────────────────────
static bool load_weights_json(const std::string& path, MpccParams& mp,
                               QuadParams& qp, double vel) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    auto j = nlohmann::json::parse(f);
    auto w = j["weights"];
    double qec = w["Q_ec"][0];
    double qel = w["Q_el"][0];
    double qq  = w["Q_q"][0];
    double ut  = w["U_mat"][0];
    double uw  = w["U_mat"][1];
    double qs  = w["Q_s"];
    mp.Q_ec  = Vec3(qec, qec, qec);
    mp.Q_el  = Vec3(qel, qel, qel);
    mp.Q_q   = Vec3(qq, qq, qq);
    mp.U_mat = Eigen::Vector4d(ut, uw, uw, uw);
    mp.Q_s   = qs;
    qp.vtheta_max = vel;
    return true;
}

// ── Extended CSV (SilResult + MHE/MPCC extra columns) ───────────────────────
static void save_csv_extended(
    const std::string& path, const SilResult& res,
    const std::vector<double>& log_theta,
    const std::vector<double>& log_vtheta,
    const std::vector<double>& log_atheta,
    const std::vector<double>& log_s_nearest,
    const std::vector<double>& log_mhat,
    const std::vector<double>& log_tauhat,
    const std::vector<double>& log_dx,
    const std::vector<double>& log_dy,
    const std::vector<double>& log_dz,
    const std::vector<double>& log_sigma,
    const std::vector<double>& log_ws,
    const std::vector<double>& log_mhe_ms,
    const std::vector<int>&    log_mhe_status,
    const std::vector<double>& log_px_hat,
    const std::vector<double>& log_py_hat,
    const std::vector<double>& log_pz_hat,
    const std::vector<double>& log_vx_hat,
    const std::vector<double>& log_vy_hat,
    const std::vector<double>& log_vz_hat,
    const std::vector<double>& log_qw_hat,
    const std::vector<double>& log_qx_hat,
    const std::vector<double>& log_qy_hat,
    const std::vector<double>& log_qz_hat,
    const std::vector<double>& log_wx_hat,
    const std::vector<double>& log_wy_hat,
    const std::vector<double>& log_wz_hat)
{
    std::ofstream csv(path);
    csv << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,"
        << "T,wx_cmd,wy_cmd,wz_cmd,"
        << "mpcc_solve_ms,loop_ms,s,"
        << "px_ref,py_ref,pz_ref,qw_ref,qx_ref,qy_ref,qz_ref,"
        << "theta,vtheta,a_theta,s_nearest,"
        << "m_hat,tau_hat,dx_hat,dy_hat,dz_hat,"
        << "sigma_k,W_s,mhe_ms,mhe_status,mpcc_status,"
        << "px_hat,py_hat,pz_hat,vx_hat,vy_hat,vz_hat,"
        << "qw_hat,qx_hat,qy_hat,qz_hat,wx_hat,wy_hat,wz_hat\n";

    int n = res.n_steps;
    for (int i = 0; i < n; ++i) {
        csv << res.t(i);
        for (int j = 0; j < 13; ++j) csv << "," << res.x(j, i);
        for (int j = 0; j < 4;  ++j) csv << "," << res.u(j, i);
        csv << "," << res.solve_ms(i) << "," << res.loop_ms(i) << "," << res.progress(i);
        for (int j = 0; j < 3; ++j) csv << "," << res.p_ref(j, i);
        for (int j = 0; j < 4; ++j) csv << "," << res.q_ref(j, i);

        auto safe = [&](const std::vector<double>& v) -> double {
            return (i < (int)v.size()) ? v[i] : 0.0;
        };
        auto safe_i = [&](const std::vector<int>& v) -> int {
            return (i < (int)v.size()) ? v[i] : -1;
        };

        csv << "," << safe(log_theta)
            << "," << safe(log_vtheta)
            << "," << safe(log_atheta)
            << "," << safe(log_s_nearest)
            << "," << safe(log_mhat)
            << "," << safe(log_tauhat)
            << "," << safe(log_dx)
            << "," << safe(log_dy)
            << "," << safe(log_dz)
            << "," << safe(log_sigma)
            << "," << safe(log_ws)
            << "," << safe(log_mhe_ms)
            << "," << safe_i(log_mhe_status)
            << "," << res.status(i)
            << "," << safe(log_px_hat)
            << "," << safe(log_py_hat)
            << "," << safe(log_pz_hat)
            << "," << safe(log_vx_hat)
            << "," << safe(log_vy_hat)
            << "," << safe(log_vz_hat)
            << "," << safe(log_qw_hat)
            << "," << safe(log_qx_hat)
            << "," << safe(log_qy_hat)
            << "," << safe(log_qz_hat)
            << "," << safe(log_wx_hat)
            << "," << safe(log_wy_hat)
            << "," << safe(log_wz_hat)
            << "\n";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ── Parameters ──────────────────────────────────────────────────────
    QuadParams quad;
    MpccParams mpcc_p;

    // Usage: ./mhe_mpcc_sil <vtheta_max> [v1|v2]
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <vtheta_max> [v1|v2]\n", argv[0]);
        fprintf(stderr, "  v1: MPCC uses wrong mass (%.2f kg) always — open loop\n", M_HAT_INIT);
        fprintf(stderr, "  v2: MHE corrects mass online (default)\n");
        return 1;
    }
    double vel = std::atof(argv[1]);
    // Mode: v1 = open-loop (no MHE→MPCC feedback), v2 = closed-loop (default)
    bool use_v2 = true;
    std::string mode_str = "v2";
    if (argc >= 3) {
        mode_str = std::string(argv[2]);
        use_v2   = (mode_str != "v1");
    }
    std::string json_path = "../tuning/best_weights_mpcc_sil_v"
                            + std::to_string(static_cast<int>(vel)) + ".json";
    if (!load_weights_json(json_path, mpcc_p, quad, vel)) {
        fprintf(stderr, "ERROR: cannot load %s\n", json_path.c_str());
        return 1;
    }

    // ── MuJoCo interface ────────────────────────────────────────────────
    auto muj = std::make_shared<MujocoInterface>("mhe_mpcc_sil_controller");
    std::thread spin_thread([&]() { rclcpp::spin(muj); });

    RCLCPP_INFO(muj->get_logger(),
        "MHE-MPCC SiL [%s] — vtheta_max=%.1f, W_s_max=%.1f, m_nom=%.2f kg",
        mode_str.c_str(), vel, mpcc_p.Q_s, M_HAT_INIT);

    // ── Build trajectory ────────────────────────────────────────────────
    RCLCPP_INFO(muj->get_logger(), "Building trajectory (arc-length)...");
    LissajousTrajectory liss;
    std::vector<Vec3> traj_pos, traj_vel;
    std::vector<double> traj_t;
    liss.sample(2000, traj_pos, traj_vel, traj_t);

    ArcLengthPath arc_path;
    arc_path.build(traj_pos, traj_vel, traj_t);
    const double s_max   = std::min(100.0, arc_path.s_max());
    const double s_wp_end = std::min(s_max + quad.theta_margin, arc_path.s_max());

    const int N_WP = 400;
    PathWaypoints wp = build_waypoints(
        arc_path, s_wp_end, N_WP,
        quad.att_ref_speed, quad.g, quad.att_ref_max_tilt_deg);
    RCLCPP_INFO(muj->get_logger(), "Waypoints: N=%d, s_wp_end=%.1f m (s_max=%.1f)",
                N_WP, s_wp_end, s_max);

    // ── Init MPCC solver ────────────────────────────────────────────────
    RCLCPP_INFO(muj->get_logger(), "Initialising MPCC solver (N=%d)...", MpccController::N);
    MpccController mpcc;
    if (!mpcc.init()) {
        RCLCPP_ERROR(muj->get_logger(), "MPCC init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    MpccWeights weights;
    weights.Q_ec  = mpcc_p.Q_ec;
    weights.Q_el  = mpcc_p.Q_el;
    weights.Q_q   = mpcc_p.Q_q;
    weights.U_mat = mpcc_p.U_mat;
    weights.Q_s   = mpcc_p.Q_s;        // nominal W_s_max
    weights.vtheta_max = quad.vtheta_max;
    // Both v1 and v2 start with wrong mass assumption.
    // v2: MHE will correct it online once converged.
    // v1: stays at M_HAT_INIT forever (simulates miscalibrated model).
    weights.m_hat = M_HAT_INIT;        // 0.9 kg — true is 1.08 (17% error)
    weights.d_hat = Vec3::Zero();
    const double W_S_MAX = mpcc_p.Q_s; // keep reference for adaptive law

    mpcc.set_params_all(weights);

    // ── Init MHE ────────────────────────────────────────────────────────
    RCLCPP_INFO(muj->get_logger(), "Initialising MHE (N=%d, m_hat_init=%.2f kg)...",
                MheController::N, M_HAT_INIT);
    MheController mhe;
    if (!mhe.init()) {
        RCLCPP_ERROR(muj->get_logger(), "MHE init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // Initial prior state (centred at P0 = trajectory start)
    MheEstimate x0_prior;
    x0_prior.pos    = wp.pos[0];
    x0_prior.vel    = Vec3::Zero();
    x0_prior.quat   = wp.quat[0];
    x0_prior.omega  = Vec3::Zero();
    x0_prior.theta  = 0.0;
    x0_prior.vtheta = 0.0;
    x0_prior.f      = quad.mass * quad.g;
    x0_prior.m_hat  = M_HAT_INIT;
    x0_prior.tau_hat = TAU_HAT_INIT;
    x0_prior.d_hat  = Vec3::Zero();

    // Initial P̄ diagonal (uncertainties for each state)
    Eigen::Matrix<double,21,1> P_init;
    P_init.segment<3>(0).setConstant(0.05);    // pos: std≈0.22m
    P_init.segment<3>(3).setConstant(0.2);     // vel: std≈0.45m/s
    P_init.segment<4>(6).setConstant(0.02);    // quat: std≈0.14rad (log)
    P_init.segment<3>(10).setConstant(0.2);    // omega: std≈0.45rad/s
    P_init(13) = 5.0;     // θ: std≈2.24m (loose on start)
    P_init(14) = 2.0;     // vθ: std≈1.41m/s
    P_init(15) = 4.0;     // f: std≈2N
    P_init(16) = 0.1;     // m: std≈0.32kg  ← deliberately uncertain
    P_init(17) = 0.01;    // τ_rc: std≈0.10s (loose — keeps P̄_inv reasonable)
    P_init.segment<3>(18).setConstant(0.5);    // d: std≈0.71m/s²

    mhe.reset(x0_prior, P_init);
    RCLCPP_INFO(muj->get_logger(), "MHE reset — sigma_k_init=%.3e", mhe.get_sigma());

    // ── EMA state (parameter estimates, initialized to prior) ───────────
    double m_ema   = M_HAT_INIT;
    double tau_ema = TAU_HAT_INIT;
    Vec3   d_ema   = Vec3::Zero();

    // ── Virtual state trackers (Euler-integrated outside solver) ────────
    double theta_current  = 0.0;
    double vtheta_current = 0.0;
    double f_current      = quad.mass * quad.g;

    // Previous MPCC control (fed to MHE as u_applied)
    Control5 u_prev = Control5::Zero();
    u_prev(0) = 0.0;   // Δf=0 (no thrust change)

    // Extra log vectors (accumulated in callback)
    std::vector<double> log_theta, log_vtheta, log_atheta, log_s_nearest;
    std::vector<double> log_mhat, log_tauhat, log_dx, log_dy, log_dz;
    std::vector<double> log_sigma, log_ws, log_mhe_ms;
    std::vector<int>    log_mhe_status;
    // Estimated physical states (for real vs estimated comparison plots)
    std::vector<double> log_px_hat, log_py_hat, log_pz_hat;
    std::vector<double> log_vx_hat, log_vy_hat, log_vz_hat;
    std::vector<double> log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat;
    std::vector<double> log_wx_hat, log_wy_hat, log_wz_hat;

    // Warm-start MPCC with a few SQP-RTI iterations before the run
    const int N_WARMUP = 20;
    bool first_call = true;
    double s_nearest_prev = 0.0;

    // ── Configure SiL protocol ──────────────────────────────────────────
    SilConfig cfg;
    cfg.P0           = wp.pos[0];
    cfg.mass         = quad.mass;
    cfg.gravity      = quad.g;
    cfg.progress_max = s_max;   // terminate when s_nearest >= s_max
    cfg.t_final      = 85.0;
    cfg.abort_on_crash = true;

    SilProtocol proto(muj, cfg);

    proto.set_progress_tracker(
        [&](const Vec3& /*pos*/, double /*s_prev*/) -> double {
            return s_nearest_prev;
        });

    proto.set_controller(
        [&](const DroneState& ds, double /*t*/, double /*s*/) -> ControlOutput {
            using Clock = std::chrono::steady_clock;

            // ── 1. Pack measurement y_k (13D) ──────────────────────────
            Eigen::Matrix<double,13,1> y_k;
            y_k << ds.pos, ds.vel, ds.quat, ds.omega;

            // ── 2. Push to MHE window + solve ──────────────────────────
            mhe.push(y_k, u_prev);

            auto mhe_tic = Clock::now();
            int mhe_stat = mhe.solve();
            double mhe_ms = std::chrono::duration<double,std::milli>(
                Clock::now() - mhe_tic).count();

            // ── 3. Propagate arrival cost prior ────────────────────────
            // Pass solver_ok=true only when status==0 to prevent corrupting
            // x_bar_ with garbage estimates (MINSTEP / NaN).
            mhe.propagate_prior(mhe_stat == 0);

            // ── 4. Get MHE estimates (with fallback on failure) ─────────
            MheEstimate xhat = mhe.get_estimate();
            double sigma_k   = mhe.get_sigma();

            // Fallback: if MHE failed or returned NaN, use raw measurement
            bool mhe_ok = (mhe_stat == 0)
                          && xhat.pos.allFinite()
                          && xhat.vel.allFinite()
                          && xhat.quat.allFinite()
                          && std::isfinite(xhat.m_hat)
                          && std::isfinite(xhat.tau_hat)
                          && xhat.d_hat.allFinite();
            if (!mhe_ok) {
                xhat.pos    = ds.pos;
                xhat.vel    = ds.vel;
                xhat.quat   = ds.quat;
                xhat.omega  = ds.omega;
                xhat.theta  = theta_current;
                xhat.vtheta = vtheta_current;
                xhat.f      = f_current;
                // NO CHEAT: keep the last smoothed parameter estimate, NEVER the
                // true mass. Injecting quad.mass here made m̂ "converge" to 1.08
                // purely as an artifact when the solver failed every step.
                xhat.m_hat  = m_ema;
                xhat.tau_hat = tau_ema;
                xhat.d_hat  = d_ema;
                sigma_k = mhe.get_sigma();  // keep sigma for logging
            }

            // ── 5. EMA smooth on parameter estimates ────────────────────
            // Raw MHE estimates can have step-changes when the window shifts.
            // EMA low-pass filters them before logging and MPCC feedback.
            m_ema   = EMA_ALPHA_PARAM * m_ema   + (1.0 - EMA_ALPHA_PARAM) * xhat.m_hat;
            tau_ema = EMA_ALPHA_PARAM * tau_ema + (1.0 - EMA_ALPHA_PARAM) * xhat.tau_hat;
            d_ema   = EMA_ALPHA_DIST  * d_ema   + (1.0 - EMA_ALPHA_DIST)  * xhat.d_hat;

            // ── 6. Adaptive W_s ─────────────────────────────────────────
            //   W_s(k) = W_s_max / (1 + α · σ_k)
            //   - σ_k large (cold start) → W_s ≈ 0 → conservative (no rushing)
            //   - σ_k → 0 (converged)   → W_s → W_s_max → aggressive
            double W_s_k = W_S_MAX / (1.0 + ALPHA_WS * sigma_k);

            // ── 7. Update MPCC weights + adaptive model params ──────────
            MpccWeights w_adaptive = weights;  // starts with m_hat=M_HAT_INIT
            w_adaptive.Q_s = W_s_k;
            // v2 (closed-loop): feed MHE estimates into MPCC dynamics once converged.
            // v1 (open-loop):   MPCC always uses the wrong initial mass (M_HAT_INIT).
            if (use_v2 && sigma_k < 0.05) {
                w_adaptive.m_hat = m_ema;
                w_adaptive.d_hat = d_ema;
                // Convert tau_ema [s] → k_tau [1/s], clamp to avoid singularity
                w_adaptive.k_tau = 1.0 / std::max(tau_ema, 0.005);
            }
            mpcc.set_params_all(w_adaptive);

            // ── 8. Build MPCC state from MHE + virtual states ──────────
            // Physical states from MHE estimate; virtual from Euler integration
            State16 x_mpcc;
            x_mpcc.head<3>()       = xhat.pos;
            x_mpcc.segment<3>(3)   = xhat.vel;
            x_mpcc.segment<4>(6)   = xhat.quat / (xhat.quat.norm() + 1e-12);
            x_mpcc.segment<3>(10)  = xhat.omega;
            x_mpcc(13) = theta_current;
            x_mpcc(14) = vtheta_current;
            x_mpcc(15) = f_current;

            // ── 9. Warm-start MPCC on first call ───────────────────────
            if (first_call) {
                const int Nh = MpccController::N;
                double v0 = 2.0;
                for (int k = 0; k <= Nh; ++k) {
                    State16 xk = x_mpcc;
                    double tk   = k * 0.01;
                    xk(13) = std::clamp(theta_current + v0 * tk, 0.0,
                                        s_max + quad.theta_margin);
                    xk(14) = v0;
                    xk(15) = f_current;
                    mpcc.set_x_init(k, xk);
                    if (k < Nh) {
                        Control5 uk; uk.setZero();
                        mpcc.set_u_init(k, uk);
                    }
                }
                for (int i = 0; i < N_WARMUP; ++i) mpcc.solve();
                first_call = false;
            }

            // ── 10. MPCC solve ──────────────────────────────────────────
            mpcc.set_x0(x_mpcc);
            ControlOutput out;
            out.solver_status = mpcc.solve();
            out.solve_time_s  = mpcc.get_solve_time();

            // ── 11. Extract optimal control u* = [Δf, ω_cmd, a_θ] ─────
            Control5 u = mpcc.get_u0();
            u_prev = u;   // save for next MHE step

            // ── 12. Virtual state integration (Euler, 100 Hz) ──────────
            const double dt  = 0.01;
            const double th_ub = s_max + quad.theta_margin;
            double delta_f  = u(0);
            double a_theta  = u(4);

            theta_current  = std::clamp(theta_current + vtheta_current * dt, 0.0, th_ub);
            vtheta_current = std::clamp(vtheta_current + a_theta * dt, 0.0, quad.vtheta_max);
            f_current      = std::clamp(f_current + delta_f * dt, quad.T_min, quad.T_max);

            // ── 13. Commands to MuJoCo ─────────────────────────────────
            out.cmd.thrust    = f_current;       // thrust from state (smooth)
            out.cmd.omega_cmd = u.segment<3>(1); // body rate commands

            // ── 14. Geometric references for logging ────────────────────
            out.p_ref = pos_interp(theta_current, wp);
            out.q_ref = quat_interp(theta_current, wp);

            // ── 15. Find nearest arc-length (for paper metrics) ────────
            double sn = find_nearest_s(ds.pos, wp, s_nearest_prev, s_max);
            s_nearest_prev = sn;   // used by progress tracker

            // ── 16. Accumulate extra log (EMA-smoothed parameter values) ─
            log_theta.push_back(theta_current);
            log_vtheta.push_back(vtheta_current);
            log_atheta.push_back(a_theta);
            log_s_nearest.push_back(sn);
            log_mhat.push_back(m_ema);
            log_tauhat.push_back(tau_ema);
            log_dx.push_back(d_ema.x());
            log_dy.push_back(d_ema.y());
            log_dz.push_back(d_ema.z());
            log_sigma.push_back(sigma_k);
            log_ws.push_back(W_s_k);
            log_mhe_ms.push_back(mhe_ms);
            log_mhe_status.push_back(mhe_stat);
            // Estimated physical states (real vs estimated comparison)
            log_px_hat.push_back(xhat.pos.x());
            log_py_hat.push_back(xhat.pos.y());
            log_pz_hat.push_back(xhat.pos.z());
            log_vx_hat.push_back(xhat.vel.x());
            log_vy_hat.push_back(xhat.vel.y());
            log_vz_hat.push_back(xhat.vel.z());
            log_qw_hat.push_back(xhat.quat(0));
            log_qx_hat.push_back(xhat.quat(1));
            log_qy_hat.push_back(xhat.quat(2));
            log_qz_hat.push_back(xhat.quat(3));
            log_wx_hat.push_back(xhat.omega.x());
            log_wy_hat.push_back(xhat.omega.y());
            log_wz_hat.push_back(xhat.omega.z());

            // ── Periodic log ────────────────────────────────────────────
            {
                static int log_cnt = 0;
                if (++log_cnt % 500 == 0) {
                    RCLCPP_INFO(muj->get_logger(),
                        "[MHE] m̂=%.3f(raw=%.3f) kg  τ̂=%.4f s  d̂=[%.2f,%.2f,%.2f] m/s²  "
                        "σ_k=%.3e  W_s=%.2f  solve=%.2f ms",
                        m_ema, xhat.m_hat, tau_ema,
                        d_ema.x(), d_ema.y(), d_ema.z(),
                        sigma_k, W_s_k, mhe_ms);
                }
            }

            return out;
        });

    // ── Execute protocol ────────────────────────────────────────────────
    SilResult res = proto.execute();

    // ── Save CSV (basic + extended MHE columns) ─────────────────────────
    std::string csv_name = "mhe_mpcc_sil_results_" + mode_str + ".csv";
    save_csv_extended(csv_name, res,
        log_theta, log_vtheta, log_atheta, log_s_nearest,
        log_mhat, log_tauhat, log_dx, log_dy, log_dz,
        log_sigma, log_ws, log_mhe_ms, log_mhe_status,
        log_px_hat, log_py_hat, log_pz_hat,
        log_vx_hat, log_vy_hat, log_vz_hat,
        log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat,
        log_wx_hat, log_wy_hat, log_wz_hat);

    // Persistent copy
    {
        int v_int = static_cast<int>(quad.vtheta_max);
        std::system("mkdir -p ../results");
        std::string persistent = "../results/mhe_mpcc_" + mode_str + "_v"
                                 + std::to_string(v_int) + ".csv";
        save_csv_extended(persistent, res,
            log_theta, log_vtheta, log_atheta, log_s_nearest,
            log_mhat, log_tauhat, log_dx, log_dy, log_dz,
            log_sigma, log_ws, log_mhe_ms, log_mhe_status,
            log_px_hat, log_py_hat, log_pz_hat,
            log_vx_hat, log_vy_hat, log_vz_hat,
            log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat,
            log_wx_hat, log_wy_hat, log_wz_hat);
        RCLCPP_INFO(muj->get_logger(), "CSV saved: %s + %s (%d samples)",
                    csv_name.c_str(), persistent.c_str(), res.n_steps);
    }

    // Print convergence summary
    if (!log_mhat.empty()) {
        RCLCPP_INFO(muj->get_logger(),
            "[MHE convergence] m̂: %.3f → %.3f kg  (true=%.3f)",
            log_mhat.front(), log_mhat.back(), quad.mass);
        RCLCPP_INFO(muj->get_logger(),
            "[MHE convergence] σ_k: %.3e → %.3e",
            log_sigma.front(), log_sigma.back());
        RCLCPP_INFO(muj->get_logger(),
            "[Adaptive W_s] %.3f → %.3f  (max=%.1f)",
            log_ws.front(), log_ws.back(), W_S_MAX);
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
