/**
 * NMPC-MHE SiL — Lie-invariant MHE + Adaptive (offset-free) NMPC with MuJoCo.
 *
 * This is the CLEAN adaptive-estimation vehicle: a temporal trajectory-tracking
 * NMPC (no arc-length) whose model parameters [m̂, d̂, k̂_τ] are estimated online
 * by a dedicated 18-state Lie-invariant MHE (thrust-as-INPUT). Because the NMPC tracks a
 * fixed time-reference, the only thing the MHE changes is MODEL ACCURACY → the
 * tracking improvement is 100% attributable to the estimation (clean ablation).
 *
 * Architecture (100 Hz):
 *   1. MHE: push measurement y_k + applied control u_{k-1}
 *   2. MHE: solve() → x̂_k incl. m̂, τ̂, d̂  + σ_k = tr(P̄_θ)
 *   3. MHE: propagate_prior()
 *   4. EMA smooth on [m̂, τ̂, d̂]
 *   5. Gate σ_k < 0.05 → (v2 only) NMPC.set_model_params(m̂, d̂, k̂_τ)
 *   6. NMPC: set_x0 (physical state from MHE) + temporal references
 *   7. NMPC: solve() → u* = [T, ω_cmd]
 *   8. Send: T + ω_cmd to MuJoCo + feed u=[T,ω_cmd] to MHE as the applied input
 *
 * The 18-state MHE takes thrust T and ω_cmd as KNOWN INPUTS (params), exactly
 * matching the NMPC command — no thrust-state reconstruction, no Δf hack.
 *
 * Modes:
 *   ./nmpc_mhe_sil v1 [t_run]   → open-loop:  NMPC uses wrong mass (0.9 kg) always
 *   ./nmpc_mhe_sil v2 [t_run]   → closed-loop: NMPC corrects mass via MHE (default)
 *
 * CSV columns match mhe_mpcc_sil for plot reuse. Results in results/nmpc_mhe_v{1,2}.csv
 */

#include "quadrotor_mpc/nmpc/nmpc_controller.hpp"
#include "quadrotor_mpc/mhe/mhe_trans_controller.hpp"
#include "quadrotor_mpc/mhe/ekf_baseline.hpp"
#include "quadrotor_mpc/mujoco/mujoco_interface.hpp"
#include "quadrotor_mpc/mujoco/sil_protocol.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"
#include "quadrotor_mpc/common/params.hpp"
#include "quadrotor_mpc/trajectory/lissajous.hpp"
#include "quadrotor_mpc/trajectory/attitude_reference.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>
#include <random>

using namespace quadrotor_mpc;

// ── Adaptive law / MHE prior ──────────────────────────────────────────────────
// FOCUS = external disturbance rejection. Mass is assumed KNOWN/identified
// (= true 1.05, summed from the MuJoCo geom masses). The MHE estimates the
// external disturbance d̂; the NMPC injects d̂ as feedforward.
static constexpr double M_KNOWN      = 1.05;  // [kg] REAL mass (MuJoCo model total)
static constexpr double TAU_HAT_INIT = 0.03;  // [s]
static constexpr double SIGMA_GATE   = 0.05;  // inject θ̂ only when σ_k below this
static constexpr double EMA_ALPHA_PARAM = 0.995; // mass + τ_rc (heavier smoothing)
static constexpr double EMA_ALPHA_DIST  = 0.92;  // disturbances (τ≈0.12s)

// ── Extended CSV (same layout as mhe_mpcc_sil for plot reuse) ────────────────
static void save_csv_extended(
    const std::string& path, const SilResult& res,
    const std::vector<double>& log_theta, const std::vector<double>& log_vtheta,
    const std::vector<double>& log_atheta, const std::vector<double>& log_s_nearest,
    const std::vector<double>& log_mhat, const std::vector<double>& log_tauhat,
    const std::vector<double>& log_dx, const std::vector<double>& log_dy,
    const std::vector<double>& log_dz, const std::vector<double>& log_sigma,
    const std::vector<double>& log_ws, const std::vector<double>& log_mhe_ms,
    const std::vector<int>&    log_mhe_status,
    const std::vector<double>& log_px_hat, const std::vector<double>& log_py_hat,
    const std::vector<double>& log_pz_hat, const std::vector<double>& log_vx_hat,
    const std::vector<double>& log_vy_hat, const std::vector<double>& log_vz_hat,
    const std::vector<double>& log_qw_hat, const std::vector<double>& log_qx_hat,
    const std::vector<double>& log_qy_hat, const std::vector<double>& log_qz_hat,
    const std::vector<double>& log_wx_hat, const std::vector<double>& log_wy_hat,
    const std::vector<double>& log_wz_hat,
    const std::vector<double>& log_gx, const std::vector<double>& log_gy,
    const std::vector<double>& log_gz, const std::vector<double>& log_tx,
    const std::vector<double>& log_ty, const std::vector<double>& log_tz,
    const std::vector<double>& log_m_ekf, const std::vector<double>& log_dx_ekf,
    const std::vector<double>& log_dy_ekf, const std::vector<double>& log_dz_ekf,
    const std::vector<double>& log_px_ekf, const std::vector<double>& log_py_ekf,
    const std::vector<double>& log_pz_ekf, const std::vector<double>& log_vx_ekf,
    const std::vector<double>& log_vy_ekf, const std::vector<double>& log_vz_ekf,
    const std::vector<double>& log_m_ekfg, const std::vector<double>& log_sigma_m)
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
        << "qw_hat,qx_hat,qy_hat,qz_hat,wx_hat,wy_hat,wz_hat,"
        << "gx,gy,gz,tauext_x,tauext_y,tauext_z,"
        << "m_ekf,dx_ekf,dy_ekf,dz_ekf,"
        << "px_ekf,py_ekf,pz_ekf,vx_ekf,vy_ekf,vz_ekf,"
        << "m_ekfg,sigma_m\n";

    int n = res.n_steps;
    for (int i = 0; i < n; ++i) {
        csv << res.t(i);
        for (int j = 0; j < 13; ++j) csv << "," << res.x(j, i);
        for (int j = 0; j < 4;  ++j) csv << "," << res.u(j, i);
        csv << "," << res.solve_ms(i) << "," << res.loop_ms(i) << "," << res.progress(i);
        for (int j = 0; j < 3; ++j) csv << "," << res.p_ref(j, i);
        for (int j = 0; j < 4; ++j) csv << "," << res.q_ref(j, i);

        auto safe   = [&](const std::vector<double>& v){ return (i<(int)v.size())?v[i]:0.0; };
        auto safe_i = [&](const std::vector<int>&    v){ return (i<(int)v.size())?v[i]:-1;  };

        csv << "," << safe(log_theta)  << "," << safe(log_vtheta) << "," << safe(log_atheta)
            << "," << safe(log_s_nearest)
            << "," << safe(log_mhat) << "," << safe(log_tauhat)
            << "," << safe(log_dx) << "," << safe(log_dy) << "," << safe(log_dz)
            << "," << safe(log_sigma) << "," << safe(log_ws)
            << "," << safe(log_mhe_ms) << "," << safe_i(log_mhe_status)
            << "," << res.status(i)
            << "," << safe(log_px_hat) << "," << safe(log_py_hat) << "," << safe(log_pz_hat)
            << "," << safe(log_vx_hat) << "," << safe(log_vy_hat) << "," << safe(log_vz_hat)
            << "," << safe(log_qw_hat) << "," << safe(log_qx_hat) << "," << safe(log_qy_hat)
            << "," << safe(log_qz_hat) << "," << safe(log_wx_hat) << "," << safe(log_wy_hat)
            << "," << safe(log_wz_hat)
            << "," << safe(log_gx) << "," << safe(log_gy) << "," << safe(log_gz)
            << "," << safe(log_tx) << "," << safe(log_ty) << "," << safe(log_tz)
            << "," << safe(log_m_ekf) << "," << safe(log_dx_ekf)
            << "," << safe(log_dy_ekf) << "," << safe(log_dz_ekf)
            << "," << safe(log_px_ekf) << "," << safe(log_py_ekf) << "," << safe(log_pz_ekf)
            << "," << safe(log_vx_ekf) << "," << safe(log_vy_ekf) << "," << safe(log_vz_ekf)
            << "," << safe(log_m_ekfg) << "," << safe(log_sigma_m) << "\n";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Deterministic external-force profile (world frame, N). The controller commands
// this on /external_force_cmd; MuJoCo applies it and re-publishes it on
// /external_force as ground truth. Because WE define it, the disturbance is fully
// reproducible: zero for the first `warmup` s (clean takeoff + early mass ID),
// then a per-axis profile whose three axes switch on different periods so the
// estimator must track all three at once.
static Vec3 ext_force_design(double t, double amp, bool sine, double warmup = 3.0)
{
    if (t < warmup) return Vec3::Zero();
    const double tt = t - warmup;
    const double ax = amp, ay = 0.75 * amp, az = 0.55 * amp;  // per-axis amplitudes
    const double px = 7.0, py = 9.0, pz = 11.0;               // per-axis periods [s]
    if (sine) {
        const double w = 2.0 * M_PI;
        return Vec3(ax * std::sin(w * tt / px),
                    ay * std::sin(w * tt / py),
                    az * std::sin(w * tt / pz));
    }
    auto sq = [](double t, double period) {
        return (std::fmod(t, period) < 0.5 * period) ? 1.0 : -1.0;
    };
    return Vec3(ax * sq(tt, px), ay * sq(tt, py), az * sq(tt, pz));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    QuadParams quad;
    NmpcParams nmpc_p;

    // Usage: ./nmpc_mhe_sil [v1|v2] [t_run] [hover]
    //   hover → hold a fixed setpoint (no aggressive trajectory). At ~0 speed
    //   the aerodynamic drag vanishes, so d̂ captures ONLY the external gusts →
    //   clean disturbance-rejection demonstration (v2 should clearly beat v1).
    bool use_v2 = true;
    std::string mode_str = "v2";
    double t_run_arg = -1.0;
    bool hover_mode = false;
    bool slow_mode  = false;   // gentle Lissajous: moving (observable d) but low drag
    double w_override = -1.0;  // sweep: positive value overrides liss.w
    bool idmass_mode = false;  // "idmass" → start mass at 0.60 to show convergence
    double m0_override = -1.0;     // "m0=X"     → start mass identification from X kg
    double taurc0_override = -1.0; // "taurc0=X" → start τ_rc RLS from X s
    double tauf0_override  = -1.0; // "tauf0=X"  → start τ_f RLS from X s
    bool ff_mode = false;      // "ff" → inject estimated disturbance d̂ as NMPC feedforward
    bool dfix_mode = false;    // "dfix" → EXP-2 MHE-D (9D): mass fixed, estimate ONLY d
    bool outlier_mode = false; // "outlier" → corrupt the estimator-fed measurements with spikes
    int  sparse_k     = 1;     // "sparse=K" → odometry only every K steps (IMU stays 100 Hz)
    bool trans_mode   = false; // "trans" → aggressive first half (identify), hover second half (protect)
    bool probe_mode   = false; // "probe" → active observability-aware control: inject excitation when mass unobservable+unidentified
    double fext_amp   = 1.5;   // "fext=X" → deterministic external-force amplitude [N] (0 → no perturbation)
    bool   fext_sine  = false; // "fextsine" → sinusoidal profile instead of steps
    bool   pert_manual = false; // "pertmanual" → wait for the /start_perturbation service (no auto-arm)
    // Active-probe shape (info-optimal excitation study). Vertical bob modulates the
    // thrust magnitude T; lateral motion tilts the thrust axis a. sigma_tilde (the
    // inverse-CRB on the mass) depends on the dispersion of T·a, so the vertical/lateral
    // mix that maximizes sigma_tilde per unit displacement is the information-optimal probe.
    double probe_az = 0.35, probe_al = 0.20;   // [m] vertical / lateral amplitudes
    double probe_fz = 1.0,  probe_fl = 0.5;    // [Hz] vertical / lateral frequencies
    if (argc >= 2) { mode_str = std::string(argv[1]); use_v2 = (mode_str != "v1"); }
    if (argc >= 3) t_run_arg = std::atof(argv[2]);
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "hover") hover_mode = true;
        if (a == "slow")  slow_mode  = true;
        if (a == "idmass") idmass_mode = true;
        if (a.rfind("m0=", 0) == 0) { m0_override = std::atof(a.c_str() + 3); }  // seed only, keep v2 regime
        if (a.rfind("taurc0=", 0) == 0) { taurc0_override = std::atof(a.c_str() + 7); }  // seed only, keep v2 regime
        if (a.rfind("tauf0=", 0)  == 0) { tauf0_override  = std::atof(a.c_str() + 6); idmass_mode = true; }
        if (a == "ff")     ff_mode    = true;
        if (a == "dfix")   dfix_mode  = true;
        if (a == "outlier") outlier_mode = true;
        if (a == "trans")   trans_mode   = true;   // aggressive (identify) → hover (protect)
        if (a == "probe")   probe_mode   = true;   // active observability-aware excitation
        if (a.rfind("fext=", 0) == 0) fext_amp = std::atof(a.c_str() + 5);  // perturbation amplitude [N]
        if (a == "fextsine") fext_sine = true;     // sinusoidal disturbance profile
        if (a == "pertmanual") pert_manual = true; // wait for /start_perturbation service
        if (a.rfind("probe_az=", 0) == 0) probe_az = std::atof(a.c_str() + 9);
        if (a.rfind("probe_al=", 0) == 0) probe_al = std::atof(a.c_str() + 9);
        if (a.rfind("probe_fz=", 0) == 0) probe_fz = std::atof(a.c_str() + 9);
        if (a.rfind("probe_fl=", 0) == 0) probe_fl = std::atof(a.c_str() + 9);
        if (a.rfind("sparse=", 0) == 0) sparse_k = std::max(1, std::atoi(a.c_str() + 7));
        if (a.rfind("w=", 0) == 0) w_override = std::atof(a.c_str() + 2);
    }
    if (w_override > 0.0) {
        char buf[32]; std::snprintf(buf, sizeof(buf), "_w%.1f", w_override);
        mode_str += buf;  // CSV name → nmpc_mhe_v2_w2.0.csv
    }

    auto muj = std::make_shared<MujocoInterface>("nmpc_mhe_sil_controller");
    std::thread spin_thread([&]() { rclcpp::spin(muj); });

    RCLCPP_INFO(muj->get_logger(),
        "NMPC-MHE SiL [%s] — DISTURBANCE rejection, mass known=%.2f kg | "
        "MHE: %s | FEEDFORWARD d̂->NMPC: %s",
        mode_str.c_str(), M_KNOWN,
        dfix_mode ? "10D, mass FROZEN (hard bound) → estimate only d"
                  : "10D, mass is a free state (identification)",
        ff_mode ? "ON (closed-loop, d̂ injected)" : "OFF (baseline, d=0 in controller)");

    // ── Temporal Lissajous trajectory (analytic — no arc-length) ─────────
    LissajousTrajectory liss;
    if (slow_mode) liss.w = 0.5;   // 4× slower → peak speed ~3 m/s → drag ~16× less
    if (w_override > 0.0) liss.w = w_override;  // velocity-sweep override
    const double dt_ctrl = nmpc_p.dt;
    const double t_traj  = (t_run_arg > 0.0) ? std::min(t_run_arg, liss.t_final)
                                             : liss.t_final;
    RCLCPP_INFO(muj->get_logger(),
        "Trajectory: t_run=%.1f s, dt=%.3f s, N_horizon=%d",
        t_traj, dt_ctrl, NmpcController::N);

    // ── Init NMPC ────────────────────────────────────────────────────────
    NmpcController ctrl;
    if (!ctrl.init()) {
        RCLCPP_ERROR(muj->get_logger(), "NMPC init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    // Gains retuned for the COARSE N=31/1.5s (dt≈0.048s) discretization: penalise
    // body-rate commands hard (a big ω_cmd over 0.048s = huge attitude swing → crash)
    // and keep attitude tracking moderate. Stability over tightness.
    ctrl.set_weights(Vec3(180, 180, 180),       // Q_pos
                     Vec3(20, 20, 20),           // Q_att (moderate, don't over-demand)
                     Eigen::Vector4d(0.3, 5.0, 5.0, 5.0),   // R_u: strong rate penalty
                     Vec3(30, 30, 30));          // Q_vel: velocity feedforward tracking
    // Known mass, zero disturbance to start. v2 will inject d̂ once converged.
    // THRUST-LAG FEEDFORWARD test: both use the same identified model; v2 ADDS the
    // feedforward thrust-lag compensation (T_applied = T_des + τ_f·dT/dt) → isolates
    // the effect of compensating the identified thrust dynamic.
    ctrl.set_model_params(1.05, Vec3::Zero(), 1.0 / 0.056);

    // ── Init TRANSLATIONAL-ONLY MHE (10D: p,v,m,d — attitude is a measured input) ─
    MheTransController mhe;
    if (!mhe.init()) {
        RCLCPP_ERROR(muj->get_logger(), "MHE init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    // ── Baseline EKF (same 10D scope, same measurements) — PASSIVE observer ───────
    EkfBaseline ekf;   // drag=0 to match the MHE (which runs with drag_c_=0)
    // ── Observability-aware EKF (our method): same EKF + mass gate by sigma_m ──────
    EkfBaseline ekf_g;
    // Normalized identifiability threshold sigma_tilde_min (dimensionless, paper eq. snorm).
    const double EKF_GATE_THR = std::getenv("EKF_GATE_THR") ? std::atof(std::getenv("EKF_GATE_THR")) : 0.009;
    // Baseline switch: ENERGY_GATE=<thr> makes the gated EKF use a generic
    // regressor-energy dead-zone instead of the Schur-complement sigma_tilde.
    if (std::getenv("ENERGY_GATE"))
        ekf_g.enable_energy_gate(std::atof(std::getenv("ENERGY_GATE")));
    else
        ekf_g.enable_gate(EKF_GATE_THR);
    // ── Outlier injection (env-tunable, deterministic seed for reproducibility) ───
    std::mt19937 out_rng(20240617u);
    std::uniform_real_distribution<double> unif01(0.0, 1.0);
    std::uniform_int_distribution<int>     axis3(0, 2);
    std::bernoulli_distribution            coin(0.5);
    auto envd = [](const char* k, double def){ const char* v=std::getenv(k); return v?std::atof(v):def; };
    const double OUTLIER_RATE = envd("OUTLIER_RATE", 0.02);  // fraction of samples corrupted
    const double OUTLIER_MAG  = envd("OUTLIER_MAG",  25.0);  // accelerometer spike [m/s^2]
    const double OUTLIER_V    = envd("OUTLIER_V",     1.0);  // velocity glitch [m/s]
    // Real-sensor error knobs (defaults = ideal): accel scale/bias + thrust mismatch.
    const double ACCEL_SCALE  = envd("ACCEL_SCALE", 1.0);   // accelerometer scale-factor
    const double ACCEL_BIAS   = envd("ACCEL_BIAS",  0.0);   // accelerometer bias [m/s^2]
    const double THRUST_SCALE = envd("THRUST_SCALE",1.0);   // thrust-coefficient mismatch
    if (outlier_mode)
        RCLCPP_WARN(muj->get_logger(), "[OUTLIER] rate=%.3f mag=%.1f m/s^2 v=%.2f m/s (estimators only)",
                    OUTLIER_RATE, OUTLIER_MAG, OUTLIER_V);
    MheTransEstimate x0_prior;
    x0_prior.pos   = liss.position(0.0);
    x0_prior.vel   = Vec3::Zero();
    // Default: mass KNOWN (1.05) → clean d from t=0 for force-sensing. Flag "idmass":
    // start at 0.60 to show the mass-identification convergence (separate experiment).
    x0_prior.m_hat = (m0_override > 0) ? m0_override : (idmass_mode ? 0.60 : 1.05);
    x0_prior.d_hat = Vec3::Zero();

    // P̄ diagonal (14D): [p(3),v(3),m,d(3),ω(3),k_τ]
    Eigen::Matrix<double,10,1> P_init;
    P_init.segment<3>(0).setConstant(0.05);
    P_init.segment<3>(3).setConstant(0.2);
    P_init(6) = 0.1;                          // mass: estimate (EXP-1 identification)
    P_init.segment<3>(7).setConstant(1e-5);   // d locked-ish → clean mass
    mhe.reset(x0_prior, P_init);

    // ── Observability regime selected by a HARD bound on the SAME 10D OCP:
    //   "dfix"   → pin m (estimate only d)   [EXP-2 rejection]
    //   "idmass" → pin d=0 (estimate only m) [EXP-1 identification, no wind]
    if (dfix_mode)        mhe.freeze_mass(M_KNOWN);
    else if (idmass_mode) mhe.freeze_disturbance();

    // ── EMA + control-feed trackers ──────────────────────────────────────
    double m_ema   = (m0_override > 0) ? m0_override : (idmass_mode ? 0.60 : 1.05);   // match estimator init
    double tau_ema = 0.03;   // τ locked at nominal
    Vec3   d_ema   = Vec3::Zero();
    // Baseline EKF output, smoothed with the SAME EMA as the MHE (fair comparison)
    double m_ema_ekf = (m0_override > 0) ? m0_override : (idmass_mode ? 0.60 : 1.05);
    Vec3   d_ema_ekf = Vec3::Zero();
    double m_ema_ekfg = m_ema_ekf;   // observability-aware EKF (gated) output
    // Applied control fed to the MHE = [T, ω_cmd]; init to hover thrust.
    Control4 u_prev = Control4::Zero();
    u_prev(0) = quad.mass * quad.g;

    // ── Rotational k_τ estimator (recursive least squares, decoupled) ────
    // ω̇ = k_τ·(ω_cmd − ω) is LINEAR in k_τ → robust RLS, no quaternion needed.
    Vec3   omega_prev = Vec3::Zero();
    double ktau_num = 0.0, ktau_den = 1e-6;
    double ktau_est = (taurc0_override > 0) ? 1.0 / taurc0_override : 1.0 / TAU_HAT_INIT;
    double ktau_mhe = (taurc0_override > 0) ? 1.0 / taurc0_override : 1.0 / TAU_HAT_INIT;   // k_τ from MHE; seed for observability test
    const double KTAU_LAMBDA = 0.997;   // forgetting factor

    // ── Thrust dynamics k_f via RLS: ḟ = k_f·(T_cmd − f), f = m̂·|v̇+g·e₃| ──
    Vec3   vel_prev_kf = Vec3::Zero();
    double f_prev_kf   = quad.mass * quad.g;
    double kf_num = 0.0, kf_den = 1e-6;
    double kf_est = (tauf0_override > 0) ? 1.0 / tauf0_override : 112.0;  // seed ≈ 1/9ms
    const double KF_LAMBDA = 0.997;
    // Actual thrust propagated at 100 Hz with the identified τ_f (=1/kf_est):
    //   ḟ = (T_cmd − f)/τ_f.  Fed to the MHE INSTEAD of the raw jerky T_cmd → the
    //   estimator sees the real (lagged) thrust. This USES the identified τ_f.
    double f_actual = quad.mass * quad.g;
    double T_nmpc_prev = quad.mass * quad.g;   // for thrust-lag feedforward (v2)

    // ── Log vectors ──────────────────────────────────────────────────────
    std::vector<double> log_theta, log_vtheta, log_atheta, log_s_nearest;
    std::vector<double> log_mhat, log_tauhat, log_dx, log_dy, log_dz;
    std::vector<double> log_sigma, log_ws, log_mhe_ms;
    std::vector<int>    log_mhe_status;
    std::vector<double> log_px_hat, log_py_hat, log_pz_hat;
    std::vector<double> log_vx_hat, log_vy_hat, log_vz_hat;
    std::vector<double> log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat;
    std::vector<double> log_wx_hat, log_wy_hat, log_wz_hat;
    // Rotational channel: IMU gyro + GROUND-TRUTH external torque (d_w validation)
    std::vector<double> log_gx, log_gy, log_gz;        // IMU gyro [rad/s]
    std::vector<double> log_tx, log_ty, log_tz;        // ext torque GT [N·m]
    // Baseline EKF estimates (EMA-smoothed) — for the MHE-vs-EKF comparison
    std::vector<double> log_m_ekf, log_dx_ekf, log_dy_ekf, log_dz_ekf;
    // EKF physical-state estimate (raw) — for the sparse-position drift comparison
    std::vector<double> log_px_ekf, log_py_ekf, log_pz_ekf, log_vx_ekf, log_vy_ekf, log_vz_ekf;
    // Observability-aware EKF (gated): mass + identifiability metric sigma_m
    std::vector<double> log_m_ekfg, log_sigma_m;

    bool first_call = true;

    // NMPC horizon node spacing (tf/N = 1.5/31 ≈ 0.048 s) — used for the temporal
    // reference horizon. NOT the 100 Hz control period.
    const double nmpc_node_dt = 1.5 / NmpcController::N;
    // The MHE lives in the 100 Hz control loop: PUSH + SOLVE + PROPAGATE every step.
    // The OCP grid is 10 ms (DT=0.01, N=31 → 310 ms window), matching the control
    // period, so data enters at 100 Hz with no sub-sampling and the estimate is smooth.
    bool      mhe_seeded = false;   // re-seed MHE prior from the actual state at RUN start
    long      est_step   = 0;       // estimator step counter (for sparse-position gating)
    double    observ_time   = 0.0;  // cumulative time the mass has been observable (gate open)
    bool      mass_identified = false;  // latches once enough observable time accrued
    bool      probe_active = false; // active excitation currently injected
    MheTransEstimate xhat = x0_prior;
    double sigma_k  = mhe.get_sigma();
    double mhe_ms   = 0.0;
    int    mhe_stat = -1;

    // ── SiL protocol (time-based progress) ───────────────────────────────
    SilConfig cfg;
    cfg.P0           = liss.position(0.0);
    cfg.mass         = quad.mass;
    cfg.gravity      = quad.g;
    cfg.t_final      = t_traj + 10.0;
    cfg.progress_max = t_traj;
    cfg.progress_done_tol = 0.5;
    if (hover_mode || trans_mode) {
        // Holding a setpoint means ~zero velocity by design — the stall guard
        // (vel<0.3 for 5s = abort) is a false positive here. Disable it.
        // trans_mode hovers in its second half, so it would also false-trip.
        cfg.stall_timeout = 1e9;
    }
    SilProtocol proto(muj, cfg);

    proto.set_progress_tracker(
        [dt_ctrl](const Vec3&, double t_prev) -> double { return t_prev + dt_ctrl; });

    // Deterministic perturbation onset: arm the disturbance a fixed time after the
    // RUN phase begins (i.e. after the verified reset + hover), unless pertmanual
    // defers it to the /start_perturbation service. t_pert_start records the flight
    // time at arming so the injected profile always starts from its beginning.
    const double PERT_WARMUP = 3.0;   // s after RUN start
    double t_pert_start = -1.0;

    proto.set_controller(
        [&](const DroneState& ds, double /*t*/, double t_elapsed) -> ControlOutput {
            using Clock = std::chrono::steady_clock;

            // Propagate ACTUAL thrust f at 100 Hz from the last command via the
            // identified τ_f (=1/kf_est): ḟ=(T_cmd−f)/τ_f. This f (lagged, real) is
            // fed to the MHE instead of the jerky command → uses the identified τ_f.
            f_actual += (1.0 - std::exp(-dt_ctrl * kf_est)) * (u_prev(0) - f_actual);

            // ── MHE in the 100 Hz loop — PURE 100 Hz, no sub-sampling ─────────────
            // The OCP grid is 10 ms (DT=0.01, N=31 → 310 ms window), so the loop rate,
            // the measurement rate, and the node spacing ALL match at 100 Hz. Every
            // control step: push the (noisy) odometry, solve once, propagate the prior.
            // No 33 Hz cadence, no predict-to-now hacks — the newest node (stage N) is
            // always the current time, so the estimate is smooth by construction.
            // The NMPC state feedback comes from raw odometry (never the estimator).
            {
                // Thrust direction a = R(q)·e3 from the MEASURED quaternion (attitude
                // is a known input, not a state). T = applied (lagged) thrust.
                const double qw=ds.quat(0), qx=ds.quat(1), qy=ds.quat(2), qz=ds.quat(3);
                Vec3 a(2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), 1-2*(qx*qx+qy*qy));
                double T_in = f_actual;

                // IMU specific force (momentum observer): sf = R·a_imu (verified sign).
                // The accelerometer's direct measurement of (f/m)·a + d → virtual
                // force sensor. Measurement + sf are common to both MHE variants.
                Eigen::Quaterniond q_meas(qw, qx, qy, qz);
                Vec3 sf_meas = q_meas.toRotationMatrix() * ds.accel;

                // ── Real-sensor error injection (env-tunable robustness study) ────
                // Corrupt ONLY the estimator inputs (not the controller feedback) to
                // characterize the virtual sensor under accelerometer bias/scale and a
                // thrust-coefficient mismatch — the error sources a real IMU-based force
                // sensor lives or dies by, which the ideal sim does not exercise.
                sf_meas *= ACCEL_SCALE;                     // accelerometer scale-factor
                sf_meas += Vec3(ACCEL_BIAS, ACCEL_BIAS, ACCEL_BIAS);  // accelerometer bias [m/s^2]
                T_in    *= THRUST_SCALE;                    // thrust-coefficient mismatch

                Eigen::Matrix<double,6,1> y_k; y_k << ds.pos, ds.vel;  // NOISY odom

                // ── Outlier injection (sensor-fault robustness flank) ─────────────
                // Corrupt ONLY the measurements fed to the estimators — IDENTICAL
                // spike to MHE and EKF (fair) — and NOT the controller feedback (so
                // the flight stays stable and the comparison isolates estimation).
                if (outlier_mode && unif01(out_rng) < OUTLIER_RATE) {
                    int ax = axis3(out_rng);
                    sf_meas(ax) += (coin(out_rng) ? 1.0 : -1.0) * OUTLIER_MAG;  // IMU spike
                    if (coin(out_rng)) y_k(3 + axis3(out_rng)) +=               // velocity glitch
                        (coin(out_rng) ? 1.0 : -1.0) * OUTLIER_V;
                }

                // ── One 10D MHE. Regime selected by constraints: "dfix" pins the
                //    mass (hard bound) → estimate only d; else mass is a free state.
                if (!mhe_seeded) {
                    MheTransEstimate seed = x0_prior;     // keep m, d priors
                    seed.pos = ds.pos; seed.vel = ds.vel;
                    mhe.reset(seed, P_init);
                    if (dfix_mode)        mhe.freeze_mass(M_KNOWN);     // re-pin after re-seed
                    else if (idmass_mode) mhe.freeze_disturbance();
                    ekf.reset(ds.pos, ds.vel, seed.m_hat, seed.d_hat);  // same init as MHE
                    ekf_g.reset(ds.pos, ds.vel, seed.m_hat, seed.d_hat);
                    mhe_seeded = true;
                }
                // Sparse-position gating: odometry only every sparse_k steps; the
                // IMU (accelerometer) stays dense. fix=true → odom available.
                bool fix = (est_step % sparse_k == 0);
                est_step++;
                // Baseline EKF: same measurement stream, passive (does not drive control)
                ekf.step(y_k, T_in, a, sf_meas, dt_ctrl, fix);
                m_ema_ekf = EMA_ALPHA_PARAM*m_ema_ekf + (1.0-EMA_ALPHA_PARAM)*ekf.m_hat();
                d_ema_ekf = EMA_ALPHA_DIST *d_ema_ekf + (1.0-EMA_ALPHA_DIST) *ekf.d_hat();
                // observability-aware EKF (gated) on the IDENTICAL stream
                ekf_g.step(y_k, T_in, a, sf_meas, dt_ctrl, fix);
                m_ema_ekfg = EMA_ALPHA_PARAM*m_ema_ekfg + (1.0-EMA_ALPHA_PARAM)*ekf_g.m_hat();
                // Active observability-aware control: accrue observable time when the
                // mass gate is OPEN; once enough accrues, the mass is "identified".
                if (!ekf_g.gate_active()) observ_time += dt_ctrl;
                if (observ_time > 6.0) mass_identified = true;   // enough excitation to fully converge
                probe_active = probe_mode && !mass_identified;
                mhe.push(y_k, T_in, a, sf_meas, fix ? 1.0 : 0.0);
                auto mhe_tic = Clock::now();
                mhe_stat = mhe.solve();
                mhe_ms = std::chrono::duration<double,std::milli>(Clock::now()-mhe_tic).count();
                mhe.propagate_prior(mhe_stat == 0);
                MheTransEstimate e = mhe.get_estimate();
                sigma_k = mhe.get_sigma();
                bool mhe_ok = (mhe_stat == 0) && e.pos.allFinite() && e.vel.allFinite()
                              && std::isfinite(e.m_hat) && e.d_hat.allFinite();
                if (mhe_ok) {
                    xhat = e;
                    m_ema = EMA_ALPHA_PARAM*m_ema + (1.0-EMA_ALPHA_PARAM)*e.m_hat;
                    d_ema = EMA_ALPHA_DIST *d_ema + (1.0-EMA_ALPHA_DIST) *e.d_hat;
                }

            }

            // ── 5. (two-phase) model is FIXED at init — no online injection here.
            //   The MHE/RLS still run to ESTIMATE (logged), but the controller uses
            //   the fixed identified model, mirroring offline-ID → deploy.

            // ── Rotational k_τ via RLS (decoupled, every 100 Hz step) ──
            {
                Vec3 wcmd = u_prev.tail<3>();   // last NMPC body-rate command
                for (int i = 0; i < 3; ++i) {
                    double wd = (ds.omega(i) - omega_prev(i)) / dt_ctrl;  // ω̇
                    double uu = wcmd(i) - omega_prev(i);                  // ω_cmd − ω
                    ktau_num = KTAU_LAMBDA*ktau_num + wd*uu;
                    ktau_den = KTAU_LAMBDA*ktau_den + uu*uu;
                }
                // EXP-2 (dfix): τ_rc FIXED at the identified value → RLS frozen.
                if (!dfix_mode && ktau_den > 1e-2) ktau_est = std::clamp(ktau_num/ktau_den, 1.0, 200.0);
                omega_prev = ds.omega;
            }

            // ── Thrust dynamics k_f via RLS (every 100 Hz step) ────────
            {
                Vec3 vdot = (ds.vel - vel_prev_kf) / dt_ctrl;       // v̇
                double f_real = m_ema * (vdot + Vec3(0,0,quad.g)).norm();  // m̂·|v̇+g·e₃|
                double fdot = (f_real - f_prev_kf) / dt_ctrl;        // ḟ
                double uu   = u_prev(0) - f_prev_kf;                  // T_cmd − f
                kf_num = KF_LAMBDA*kf_num + fdot*uu;
                kf_den = KF_LAMBDA*kf_den + uu*uu;
                // EXP-2 (dfix): τ_f FIXED at the identified value → RLS frozen.
                if (!dfix_mode && kf_den > 1e-1) kf_est = std::clamp(kf_num/kf_den, 10.0, 500.0);
                f_prev_kf = f_real; vel_prev_kf = ds.vel;
            }

            // ── 6. NMPC: x0 from RAW ODOMETRY (not the estimator) + refs ─
            State13 x;
            x << ds.pos, ds.vel, ds.quat / (ds.quat.norm() + 1e-12), ds.omega;
            ctrl.set_x0(x);

            // CLOSED-LOOP (ff mode): inject the estimated disturbance d̂ (smoothed) as
            // NMPC feedforward → the controller predicts & cancels the external force.
            // Mass and τ stay KNOWN (1.05, identified in the offline phase). Baseline
            // (no ff) keeps d=0 in the controller. Same trajectory & wind → A/B compare.
            if (ff_mode)
                ctrl.set_model_params(M_KNOWN, d_ema_ekf, 1.0/0.056);  // feedforward from the EKF (our method); nominal k_τ

            // Publish the estimated disturbance as a force [N] (= m·d̂) on
            // /quadrotor/d_hat, same units/type as /quadrotor/external_force,
            // for real-time 1-to-1 comparison (PlotJuggler). Always on.
            muj->publish_dhat(M_KNOWN * d_ema_ekf);

            // Inject the DETERMINISTIC perturbation: we command it on
            // /external_force_cmd, MuJoCo applies it and re-publishes it as the
            // ground truth on /external_force (read back as ds.ext_force). This
            // replaces the external wind node → reproducible and reset-safe.
            // Onset is gated by the /start_perturbation service: auto-armed at
            // PERT_WARMUP past RUN start (deterministic), or by an external call.
            if (fext_amp > 0.0) {
                // Deterministic onset. For the identify→protect transition we arm the
                // disturbance only AFTER the vehicle has settled into the hover (past
                // the deceleration), so the mass is identified cleanly under excitation
                // and the gate then freezes that clean value while the plain EKF drifts.
                // Other modes arm a fixed time after takeoff.
                const double t_arm = trans_mode ? (0.5 * t_traj + 2.0) : PERT_WARMUP;
                if (!pert_manual && !muj->perturbation_on() && t_elapsed >= t_arm) {
                    muj->set_perturbation(true);   // deterministic auto-arm
                    RCLCPP_INFO(muj->get_logger(),
                        "[PERTURB] auto-armed at t=%.2f s (fext=%.2f N%s)",
                        t_elapsed, fext_amp, fext_sine ? ", sine" : ", steps");
                }
                if (muj->perturbation_on()) {
                    if (t_pert_start < 0.0) t_pert_start = t_elapsed;
                    muj->publish_ext_force_cmd(
                        ext_force_design(t_elapsed - t_pert_start, fext_amp, fext_sine, 0.0));
                } else {
                    muj->publish_ext_force_cmd(Vec3::Zero());
                }
            }

            ControlOutput out;
            Quat4 q_ref_prev = ds.quat;
            const Vec3  hover_p = liss.position(0.0);
            const Quat4 hover_q(1, 0, 0, 0);
            // Transition demo: fly the trajectory for the first half (identify the
            // mass under excitation), then hold position and hover (gate protects it).
            const double t_switch = 0.5 * t_traj;
            const bool   holding  = trans_mode && (t_elapsed >= t_switch);
            const Vec3   hold_p   = liss.position(std::min(t_switch, t_traj));
            for (int j = 0; j <= NmpcController::N; ++j) {
                Vec3  pr, vref(0, 0, 0); Quat4 qr;
                if (hover_mode || holding) {
                    pr = holding ? hold_p : hover_p; qr = hover_q;   // level hold, v_ref = 0
                    // Active excitation: while the mass is unobservable + unidentified,
                    // inject a small lateral probe → tilts the thrust axis → raises
                    // sigma_m = Var((T/m^2)a) → restores mass observability.
                    if (probe_active) {
                        // Parameterized probe: vertical bob (amp probe_az) modulates T,
                        // lateral circle (amp probe_al) tilts a. The az/al mix sets how
                        // much sigma_tilde (inverse-CRB) the probe raises per unit
                        // displacement — swept to find the information-optimal shape.
                        double tp = t_elapsed + j * nmpc_node_dt;
                        const double wz = 2.0 * M_PI * probe_fz, wl = 2.0 * M_PI * probe_fl;
                        pr += Vec3(probe_al * (std::cos(wl * tp) - 1.0),
                                   probe_al * std::sin(wl * tp),
                                   probe_az * std::sin(wz * tp));
                    }
                } else {
                    double t_j = std::min(t_elapsed + j * nmpc_node_dt, t_traj);
                    Vec3 vr = liss.velocity(t_j);
                    Vec3 ar = liss.acceleration(t_j);
                    pr = liss.position(t_j);
                    vref = vr;   // velocity feedforward → kills tracking lag at speed
                    qr = quat_from_traj(vr, ar, quad.g, quad.att_ref_max_tilt_deg, q_ref_prev);
                    q_ref_prev = qr;
                }
                if (j < NmpcController::N) ctrl.set_reference(j, pr, qr, vref);
                else                       ctrl.set_reference_terminal(pr, qr, vref);
                if (j == 0) { out.p_ref = pr; out.q_ref = qr; }
            }

            // Warm-start a few SQP-RTI iterations on the very first call
            if (first_call) {
                for (int i = 0; i < 20; ++i) ctrl.solve();
                first_call = false;
            }

            // ── 7. NMPC solve ──────────────────────────────────────────
            out.solver_status = ctrl.solve();
            out.solve_time_s  = ctrl.get_solve_time();
            Control4 u = ctrl.get_u0();

            // ── 8. Commands to MuJoCo ──────────────────────────────────
            // v2: feedforward thrust-lag compensation using identified τ_f.
            //   T_applied = T_des + τ_f·dT_des/dt  → the lagged motor reaches T_des.
            double T_cmd_out = u(0);
            if (use_v2) {
                double tau_f = 1.0 / kf_est;
                double Tdot  = (u(0) - T_nmpc_prev) / dt_ctrl;
                T_cmd_out = std::clamp(u(0) + tau_f*Tdot, quad.T_min, quad.T_max);
            }
            T_nmpc_prev = u(0);
            out.cmd.thrust    = T_cmd_out;
            out.cmd.omega_cmd = u.tail<3>();

            // ── Build MHE applied-control for next push ────────────────
            // u_applied = [T, ω_cmd] — the exact NMPC command, as a known input.
            u_prev << u(0), u(1), u(2), u(3);

            // ── Log ─────────────────────────────────────────────────────
            // cols 28-30 (theta/vtheta/atheta) repurposed for the GROUND-TRUTH external
            // force [N] (validation of d̂ against the real wind).
            log_theta.push_back(ds.ext_force.x());
            log_vtheta.push_back(ds.ext_force.y());
            log_atheta.push_back(ds.ext_force.z());
            log_s_nearest.push_back(t_elapsed);
            log_mhat.push_back(m_ema); log_tauhat.push_back(1.0 / ktau_est);  // τ̂_rc = 1/k̂_τ (RLS, side estimate)
            log_dx.push_back(d_ema.x()); log_dy.push_back(d_ema.y()); log_dz.push_back(d_ema.z());
            log_m_ekf.push_back(m_ema_ekf);
            log_dx_ekf.push_back(d_ema_ekf.x()); log_dy_ekf.push_back(d_ema_ekf.y()); log_dz_ekf.push_back(d_ema_ekf.z());
            { Vec3 pe = ekf.pos(), ve = ekf.vel();
              log_px_ekf.push_back(pe.x()); log_py_ekf.push_back(pe.y()); log_pz_ekf.push_back(pe.z());
              log_vx_ekf.push_back(ve.x()); log_vy_ekf.push_back(ve.y()); log_vz_ekf.push_back(ve.z()); }
            log_m_ekfg.push_back(m_ema_ekfg); log_sigma_m.push_back(ekf_g.sigma_m());
            log_sigma.push_back(sigma_k); log_ws.push_back(1.0 / kf_est);  // τ_f in W_s column
            log_mhe_ms.push_back(mhe_ms); log_mhe_status.push_back(mhe_stat);
            log_px_hat.push_back(xhat.pos.x()); log_py_hat.push_back(xhat.pos.y()); log_pz_hat.push_back(xhat.pos.z());
            log_vx_hat.push_back(xhat.vel.x()); log_vy_hat.push_back(xhat.vel.y()); log_vz_hat.push_back(xhat.vel.z());
            // attitude is measured (not estimated by the trans MHE) → log the measurement
            log_qw_hat.push_back(ds.quat(0)); log_qx_hat.push_back(ds.quat(1));
            log_qy_hat.push_back(ds.quat(2)); log_qz_hat.push_back(ds.quat(3));
            log_wx_hat.push_back(ds.omega.x()); log_wy_hat.push_back(ds.omega.y()); log_wz_hat.push_back(ds.omega.z());
            // Rotational channel: IMU gyro [rad/s] + GROUND-TRUTH external torque [N·m]
            log_gx.push_back(ds.gyro.x()); log_gy.push_back(ds.gyro.y()); log_gz.push_back(ds.gyro.z());
            log_tx.push_back(ds.ext_torque.x()); log_ty.push_back(ds.ext_torque.y()); log_tz.push_back(ds.ext_torque.z());

            {
                static int c = 0;
                if (++c % 500 == 0)
                    RCLCPP_INFO(muj->get_logger(),
                        "[MHE] m̂=%.3f kg  τ̂=%.4f  d̂=[%.2f,%.2f,%.2f]  σ_k=%.3e  solve=%.2f ms",
                        m_ema, tau_ema, d_ema.x(), d_ema.y(), d_ema.z(), sigma_k, mhe_ms);
            }
            return out;
        });

    // Clear any leftover external force from a previous run BEFORE the protocol
    // resets and takes off, so the RELOAD settles cleanly (no wind tilting the
    // drone on the floor) and early identification is disturbance-free.
    for (int i = 0; i < 5; ++i) { muj->publish_ext_force_cmd(Vec3::Zero()); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

    SilResult res = proto.execute();

    // Stop perturbing once the run is done (leave the sim in a clean state).
    muj->set_perturbation(false);
    for (int i = 0; i < 5; ++i) { muj->publish_ext_force_cmd(Vec3::Zero()); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }

    std::system("mkdir -p ../results");
    std::string persistent = "../results/nmpc_mhe_" + mode_str + ".csv";
    save_csv_extended(persistent, res,
        log_theta, log_vtheta, log_atheta, log_s_nearest,
        log_mhat, log_tauhat, log_dx, log_dy, log_dz,
        log_sigma, log_ws, log_mhe_ms, log_mhe_status,
        log_px_hat, log_py_hat, log_pz_hat, log_vx_hat, log_vy_hat, log_vz_hat,
        log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat, log_wx_hat, log_wy_hat, log_wz_hat,
        log_gx, log_gy, log_gz, log_tx, log_ty, log_tz,
        log_m_ekf, log_dx_ekf, log_dy_ekf, log_dz_ekf,
        log_px_ekf, log_py_ekf, log_pz_ekf, log_vx_ekf, log_vy_ekf, log_vz_ekf,
        log_m_ekfg, log_sigma_m);
    RCLCPP_INFO(muj->get_logger(), "CSV saved: %s (%d samples, completed=%s)",
                persistent.c_str(), res.n_steps, res.completed ? "YES" : "NO");

    if (!log_mhat.empty())
        RCLCPP_INFO(muj->get_logger(),
            "[MHE convergence] m̂: %.3f → %.3f kg (true=%.3f)  σ_k: %.2e → %.2e",
            log_mhat.front(), log_mhat.back(), quad.mass,
            log_sigma.front(), log_sigma.back());

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
