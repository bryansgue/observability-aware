/**
 * NMPC SiL — Temporal rate-control NMPC with MuJoCo feedback.
 *
 * References are indexed by elapsed time t ∈ [0, t_traj]:
 *   p_ref(k) = liss.position(t_elapsed + k·dt)
 *   q_ref(k) = quat_from_traj(vel(t), accel(t))   ← curvature-consistent
 *
 * No arc-length parameterisation. Termination when t_elapsed >= t_traj.
 *
 * Protocol: RELOAD → PD HOLD → RUN (NMPC) → stabilise
 */

#include "quadrotor_mpc/nmpc/nmpc_controller.hpp"
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

using namespace quadrotor_mpc;

// ── Save SilResult to CSV ──────────────────────────────────────────────────
static void save_csv(const std::string& path, const SilResult& res) {
    std::ofstream csv(path);
    csv << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,"
        << "T,wx_cmd,wy_cmd,wz_cmd,solve_ms,loop_ms,t_progress,"
        << "px_ref,py_ref,pz_ref,qw_ref,qx_ref,qy_ref,qz_ref,status\n";

    for (int i = 0; i < res.n_steps; ++i) {
        csv << res.t(i);
        for (int j = 0; j < 13; ++j) csv << "," << res.x(j, i);
        for (int j = 0; j < 4;  ++j) csv << "," << res.u(j, i);
        csv << "," << res.solve_ms(i) << "," << res.loop_ms(i)
            << "," << res.progress(i);
        for (int j = 0; j < 3; ++j) csv << "," << res.p_ref(j, i);
        for (int j = 0; j < 4; ++j) csv << "," << res.q_ref(j, i);
        csv << "," << res.status(i) << "\n";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ── Parameters ──────────────────────────────────────────────────────
    QuadParams quad;
    NmpcParams nmpc_p;

    // Optional: ./nmpc_sil <t_run>  (default = liss.t_final)
    double t_run_arg = -1.0;
    if (argc >= 2) t_run_arg = std::atof(argv[1]);

    // ── MuJoCo interface ────────────────────────────────────────────────
    auto muj = std::make_shared<MujocoInterface>("nmpc_sil_controller");
    std::thread spin_thread([&]() { rclcpp::spin(muj); });

    // ── Lissajous trajectory (analytic — no arc-length needed) ──────────
    RCLCPP_INFO(muj->get_logger(), "Building Lissajous trajectory (temporal)...");
    LissajousTrajectory liss;
    const double dt_ctrl = nmpc_p.dt;     // control period [s]
    const double t_traj  = (t_run_arg > 0.0)
                           ? std::min(t_run_arg, liss.t_final)
                           : liss.t_final;

    RCLCPP_INFO(muj->get_logger(),
        "Trajectory: t_run=%.1f s (liss.t_final=%.1f s), dt=%.3f s, N_horizon=%d",
        t_traj, liss.t_final, dt_ctrl, NmpcController::N);

    // ── Init NMPC solver ────────────────────────────────────────────────
    RCLCPP_INFO(muj->get_logger(), "Initialising NMPC solver...");
    NmpcController ctrl;
    if (!ctrl.init()) {
        RCLCPP_ERROR(muj->get_logger(), "NMPC init failed!");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    ctrl.set_weights(nmpc_p.Q_pos, nmpc_p.Q_att, nmpc_p.R_u);

    // ── Configure SiL protocol ──────────────────────────────────────────
    SilConfig cfg;
    cfg.P0           = liss.position(0.0);   // start of trajectory
    cfg.mass         = quad.mass;
    cfg.gravity      = quad.g;
    cfg.t_final      = t_traj + 10.0;        // max run time (budget)
    cfg.progress_max = t_traj;               // done when elapsed >= t_traj
    cfg.progress_done_tol = 0.5;             // [s] tolerance

    SilProtocol proto(muj, cfg);

    // Progress tracker: elapsed time advances by dt each step
    proto.set_progress_tracker(
        [dt_ctrl](const Vec3&, double t_prev) -> double {
            return t_prev + dt_ctrl;
        });

    // Controller callback: temporal NMPC
    proto.set_controller(
        [&](const DroneState& ds, double /*t*/, double t_elapsed) -> ControlOutput {
            // Build state vector
            State13 x;
            x << ds.pos, ds.vel, ds.quat, ds.omega;
            ctrl.set_x0(x);

            // Set horizon references indexed by time
            ControlOutput out;
            Quat4 q_ref_prev = ds.quat;  // start hemisphere from current attitude

            for (int j = 0; j <= NmpcController::N; ++j) {
                double t_j = std::min(t_elapsed + j * dt_ctrl, t_traj);
                Vec3  pr = liss.position(t_j);
                Vec3  vr = liss.velocity(t_j);
                Vec3  ar = liss.acceleration(t_j);
                Quat4 qr = quat_from_traj(vr, ar, quad.g,
                                          quad.att_ref_max_tilt_deg, q_ref_prev);
                q_ref_prev = qr;

                if (j < NmpcController::N)
                    ctrl.set_reference(j, pr, qr);
                else
                    ctrl.set_reference_terminal(pr, qr);

                if (j == 0) { out.p_ref = pr; out.q_ref = qr; }
            }

            // Solve
            out.solver_status = ctrl.solve();
            out.solve_time_s  = ctrl.get_solve_time();

            // Get optimal control at stage 0
            Control4 u = ctrl.get_u0();
            out.cmd.thrust    = u(0);
            out.cmd.omega_cmd = u.tail<3>();

            return out;
        });

    // ── Execute protocol ────────────────────────────────────────────────
    SilResult res = proto.execute();

    // ── Save CSV ────────────────────────────────────────────────────────
    std::string csv_path = "nmpc_sil_results.csv";
    save_csv(csv_path, res);
    RCLCPP_INFO(muj->get_logger(),
        "CSV saved: %s (%d samples, completed=%s)",
        csv_path.c_str(), res.n_steps, res.completed ? "YES" : "NO");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
