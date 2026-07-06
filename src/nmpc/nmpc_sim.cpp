/**
 * NMPC MiL simulation — Temporal rate-control NMPC (no ROS2, no arc-length).
 *
 * References indexed by elapsed time t ∈ [0, t_traj]:
 *   p_ref(k) = liss.position(t + k·dt)
 *   q_ref(k) = quat_from_traj(vel(t+k·dt), accel(t+k·dt))
 *
 * Termination: t >= liss.t_final
 */

#include "quadrotor_mpc/nmpc/nmpc_controller.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"
#include "quadrotor_mpc/common/integrators.hpp"
#include "quadrotor_mpc/common/params.hpp"
#include "quadrotor_mpc/trajectory/lissajous.hpp"
#include "quadrotor_mpc/trajectory/attitude_reference.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <vector>
#include <numeric>

using namespace quadrotor_mpc;

// ── Quadrotor dynamics (13-state, rate-control) ────────────────────────────
static State13 quadrotor_dynamics(const State13& x, const Control4& u) {
    const double mass   = 1.08;
    const double g      = 9.81;
    const double tau_rc = 0.03;

    Vec3  v   = x.segment<3>(3);
    Quat4 q   = x.segment<4>(6);
    Vec3  w   = x.segment<3>(10);
    double T  = u(0);
    Vec3   wc = u.segment<3>(1);

    Mat3 R = quat_to_rot(q);

    State13 xdot;
    xdot.segment<3>(0) = v;
    xdot.segment<3>(3) = -Vec3(0, 0, g) + R * Vec3(0, 0, T) / mass;
    xdot.segment<4>(6) = quat_kinematics(q, w);
    xdot.segment<3>(10) = (wc - w) / tau_rc;
    return xdot;
}

// ── Main simulation ────────────────────────────────────────────────────────
int main() {
    std::cout << "=== NMPC Quadrotor Simulation — Temporal ===" << std::endl;

    QuadParams quad;
    NmpcParams nmpc_params;

    const double dt      = nmpc_params.dt;
    const double t_traj  = 63.0;          // Lissajous duration [s] (liss.t_final)
    const int    N_steps = static_cast<int>(t_traj / dt) + 200;  // small budget overshoot

    // ── 1. Lissajous trajectory (analytic) ─────────────────────────────────
    std::cout << "Using analytic Lissajous (t_final=" << t_traj << " s)..." << std::endl;
    LissajousTrajectory liss;

    // ── 2. Initialise NMPC ─────────────────────────────────────────────────
    std::cout << "Initialising NMPC solver..." << std::endl;
    NmpcController ctrl;
    if (!ctrl.init()) {
        std::cerr << "Failed to initialise NMPC solver!" << std::endl;
        return 1;
    }
    ctrl.set_weights(nmpc_params.Q_pos, nmpc_params.Q_att, nmpc_params.R_u);

    // ── 3. Initial state — start of trajectory ─────────────────────────────
    State13 x = State13::Zero();
    x.segment<3>(0) = liss.position(0.0);
    // Attitude at t=0 from trajectory curvature
    {
        Quat4 q0 = quat_from_traj(liss.velocity(0.0), liss.acceleration(0.0),
                                   quad.g, quad.att_ref_max_tilt_deg);
        x.segment<4>(6) = q0;
    }

    // ── 4. Logging ─────────────────────────────────────────────────────────
    std::vector<double>   log_t, log_solve_ms, log_progress;
    std::vector<State13>  log_x;
    std::vector<Control4> log_u;
    std::vector<Vec3>     log_p_ref;
    std::vector<Quat4>    log_q_ref;

    log_t.reserve(N_steps);
    log_x.reserve(N_steps);
    log_u.reserve(N_steps);
    log_solve_ms.reserve(N_steps);
    log_progress.reserve(N_steps);
    log_p_ref.reserve(N_steps);
    log_q_ref.reserve(N_steps);

    auto dynamics = [](const State13& x, const Control4& u) -> State13 {
        return quadrotor_dynamics(x, u);
    };

    std::cout << "Running simulation (t_traj=" << t_traj << " s)..." << std::endl;

    int solve_fails = 0;
    for (int step = 0; step < N_steps; ++step) {
        double t = step * dt;

        // Termination: trajectory elapsed
        if (t >= t_traj) {
            std::cout << "  Trajectory completed at t = " << t << " s" << std::endl;
            break;
        }

        // Normalise quaternion
        x.segment<4>(6) = quat_normalize(x.segment<4>(6));
        ctrl.set_x0(x);

        // Set horizon references (temporal)
        Vec3  p_ref_0;
        Quat4 q_ref_0;
        Quat4 q_ref_prev = x.segment<4>(6);  // hemisphere anchor

        for (int k = 0; k <= NmpcController::N; ++k) {
            double t_k = std::min(t + k * dt, t_traj);
            Vec3  pr = liss.position(t_k);
            Vec3  vr = liss.velocity(t_k);
            Vec3  ar = liss.acceleration(t_k);
            Quat4 qr = quat_from_traj(vr, ar, quad.g,
                                      quad.att_ref_max_tilt_deg, q_ref_prev);
            q_ref_prev = qr;

            if (k < NmpcController::N)
                ctrl.set_reference(k, pr, qr);
            else
                ctrl.set_reference_terminal(pr, qr);

            if (k == 0) { p_ref_0 = pr; q_ref_0 = qr; }
        }

        // Solve
        auto t_solve_start = std::chrono::steady_clock::now();
        int status = ctrl.solve();
        double solve_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_solve_start).count();

        if (status != 0) ++solve_fails;

        Control4 u = ctrl.get_u0();

        // Log
        log_t.push_back(t);
        log_x.push_back(x);
        log_u.push_back(u);
        log_solve_ms.push_back(solve_ms);
        log_progress.push_back(t);     // progress = elapsed time
        log_p_ref.push_back(p_ref_0);
        log_q_ref.push_back(q_ref_0);

        // Integrate
        x = rk4_step<13, 4>(dynamics, x, u, dt);

        // Progress report every 10 s
        if (step % static_cast<int>(10.0 / dt) == 0) {
            Vec3 pos = x.segment<3>(0);
            Vec3 e_pos = pos - p_ref_0;
            std::printf("  t=%.1f / %.1f s  |e_pos|=%.3f m  solve=%.2f ms  fails=%d\n",
                        t, t_traj, e_pos.norm(), solve_ms, solve_fails);
        }
    }

    // ── 5. Save CSV ────────────────────────────────────────────────────────
    std::string csv_path = "nmpc_results.csv";
    std::cout << "Saving " << log_t.size() << " samples to " << csv_path << "..." << std::endl;

    std::ofstream csv(csv_path);
    csv << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,"
        << "T,wx_cmd,wy_cmd,wz_cmd,solve_ms,t_progress,"
        << "px_ref,py_ref,pz_ref,qw_ref,qx_ref,qy_ref,qz_ref\n";

    for (size_t i = 0; i < log_t.size(); ++i) {
        const State13&  xi = log_x[i];
        const Control4& ui = log_u[i];
        csv << log_t[i];
        for (int j = 0; j < 13; ++j) csv << "," << xi(j);
        for (int j = 0; j < 4;  ++j) csv << "," << ui(j);
        csv << "," << log_solve_ms[i] << "," << log_progress[i];
        csv << "," << log_p_ref[i](0) << "," << log_p_ref[i](1) << "," << log_p_ref[i](2);
        csv << "," << log_q_ref[i](0) << "," << log_q_ref[i](1)
            << "," << log_q_ref[i](2) << "," << log_q_ref[i](3);
        csv << "\n";
    }
    csv.close();

    // ── 6. Summary ─────────────────────────────────────────────────────────
    if (!log_solve_ms.empty()) {
        double avg_solve = std::accumulate(log_solve_ms.begin(), log_solve_ms.end(), 0.0)
                           / log_solve_ms.size();
        double max_solve = *std::max_element(log_solve_ms.begin(), log_solve_ms.end());

        std::cout << "\n=== Simulation Summary ===" << std::endl;
        std::cout << "  Steps:       " << log_t.size() << std::endl;
        std::cout << "  Duration:    " << log_t.back() << " / " << t_traj << " s" << std::endl;
        std::cout << "  Solve fails: " << solve_fails << " / " << log_t.size() << std::endl;
        std::cout << "  Solve time:  avg=" << avg_solve << " ms, max=" << max_solve << " ms" << std::endl;
        std::cout << "  CSV:         " << csv_path << std::endl;
    }

    return 0;
}
