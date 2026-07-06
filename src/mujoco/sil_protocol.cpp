#include "quadrotor_mpc/mujoco/sil_protocol.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace quadrotor_mpc {

SilProtocol::SilProtocol(std::shared_ptr<MujocoInterface> muj, const SilConfig& cfg)
    : muj_(muj), cfg_(cfg) {}

void SilProtocol::set_controller(ControllerCallback cb) {
    controller_ = std::move(cb);
}

void SilProtocol::set_progress_tracker(ProgressCallback cb) {
    progress_tracker_ = std::move(cb);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Phase 1 & 4: RELOAD
// ═══════════════════════════════════════════════════════════════════════════

bool SilProtocol::reload_() {
    RCLCPP_INFO(muj_->get_logger(), "[PROTOCOL] Phase: RELOAD");

    // Send hover thrust (mass·g) so the drone doesn't fall
    muj_->send_cmd(cfg_.mass * cfg_.gravity, 0.0, 0.0, 0.0);

    // The MuJoCo reset gremlin: the service may return success but leave the
    // drone in a bad state (flying off / not level). So RETRY and VERIFY the
    // landed state (near origin, on the floor, level) before proceeding.
    const int MAX_TRIES = 6;
    for (int attempt = 1; attempt <= MAX_TRIES; ++attempt) {
        if (!muj_->reset_sim()) {
            RCLCPP_WARN(muj_->get_logger(),
                "[PROTOCOL] reset service failed (try %d/%d), retrying...",
                attempt, MAX_TRIES);
            std::this_thread::sleep_for(300ms);
            continue;
        }
        muj_->clear_crash();
        std::this_thread::sleep_for(200ms);   // let fresh odom flow
        muj_->clear_crash();
        if (!muj_->is_connected() && !muj_->wait_for_connection()) {
            RCLCPP_WARN(muj_->get_logger(),
                "[PROTOCOL] no odom after reset (try %d/%d)", attempt, MAX_TRIES);
            continue;
        }
        // VERIFY: drone must be near origin, on the floor, level.
        DroneState s = muj_->get_state();
        bool clean = (std::fabs(s.pos.x()) < 0.6 && std::fabs(s.pos.y()) < 0.6
                      && s.pos.z() < 0.15 && std::fabs(s.quat(0)) > 0.95);
        if (clean) {
            RCLCPP_INFO(muj_->get_logger(),
                "[PROTOCOL] RELOAD complete — verified clean (try %d): "
                "z=%.3f qw=%.3f", attempt, s.pos.z(), s.quat(0));
            return true;
        }
        RCLCPP_WARN(muj_->get_logger(),
            "[PROTOCOL] reset NOT clean (try %d/%d): z=%.2f qw=%.3f — retrying",
            attempt, MAX_TRIES, s.pos.z(), s.quat(0));
        std::this_thread::sleep_for(300ms);
    }
    RCLCPP_ERROR(muj_->get_logger(),
        "[PROTOCOL] RELOAD failed — could not reach a clean state in %d tries",
        MAX_TRIES);
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Phase 2: PD HOLD
// ═══════════════════════════════════════════════════════════════════════════

bool SilProtocol::pd_hold_() {
    RCLCPP_INFO(muj_->get_logger(),
        "[PROTOCOL] Phase: PD HOLD → [%.2f, %.2f, %.2f]",
        cfg_.P0.x(), cfg_.P0.y(), cfg_.P0.z());

    muj_->start_pd_hold(cfg_.P0, cfg_.mass, cfg_.gravity);

    bool converged = muj_->wait_for_pd_convergence(
        cfg_.P0, cfg_.pd_settle_dist, cfg_.pd_settle_time, cfg_.pd_timeout);

    muj_->stop_pd_hold();

    // NOTE: crash flag is NOT cleared here. It is cleared once in
    // run_loop_() right before the controller starts, guaranteeing
    // that any collision during flight is always detected.

    if (!converged) {
        RCLCPP_WARN(muj_->get_logger(),
            "[PROTOCOL] PD did not converge — proceeding anyway");
    }

    // Verify position one more time
    DroneState st = muj_->get_state();
    double dist = (st.pos - cfg_.P0).norm();
    RCLCPP_INFO(muj_->get_logger(),
        "[PROTOCOL] PD HOLD complete — dist=%.3f m, pos=[%.2f, %.2f, %.2f]",
        dist, st.pos.x(), st.pos.y(), st.pos.z());

    return converged;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Phase 3: CONTROL LOOP
// ═══════════════════════════════════════════════════════════════════════════

SilResult SilProtocol::run_loop_() {
    RCLCPP_INFO(muj_->get_logger(), "[PROTOCOL] Phase: RUN (controller active)");

    const int N_max = static_cast<int>(cfg_.t_final / cfg_.dt);
    SilResult res;

    // Pre-allocate
    res.x.resize(13, N_max);
    res.u.resize(4, N_max);
    res.p_ref.resize(3, N_max);
    res.q_ref.resize(4, N_max);
    res.t.resize(N_max);
    res.progress.resize(N_max);
    res.solve_ms.resize(N_max);
    res.loop_ms.resize(N_max);
    res.status.resize(N_max);

    double progress    = 0.0;
    double stall_timer = 0.0;   // accumulated time with velocity near zero
    bool   stall_armed = false; // don't check stall until drone has started moving
    std::string stop_reason = "timeout";  // default if loop exhausts N_max

    using Clock = std::chrono::steady_clock;

    // ── Clear crash flag ONE TIME right before control starts ────────
    // PD hold already brought the drone to P0. Any collision from
    // takeoff/ground contact is stale. From here on, any collision
    // topic message is a REAL crash (wall or floor during flight).
    muj_->clear_crash();

    for (int k = 0; k < N_max; ++k) {
        auto tic = Clock::now();
        double t_k = k * cfg_.dt;

        // ── Read state ──────────────────────────────────────────────
        DroneState ds = muj_->get_state();
        ds.quat = ds.quat / (ds.quat.norm() + 1e-12);

        // ── Update progress ─────────────────────────────────────────
        if (progress_tracker_) {
            progress = progress_tracker_(ds.pos, progress);
        }

        // ── Check termination: trajectory complete ──────────────────
        if (progress >= cfg_.progress_max - cfg_.progress_done_tol) {
            stop_reason = "TRAJECTORY_COMPLETE";
            res.completed        = true;
            res.n_steps          = k;
            res.t_final          = t_k;
            res.progress_final   = progress;
            break;
        }

        // ── Call controller ─────────────────────────────────────────
        ControlOutput out = controller_(ds, t_k, progress);

        if (out.solver_status != 0) ++res.solve_fails;

        // ── Send command to MuJoCo ──────────────────────────────────
        muj_->send_cmd(out.cmd);

        // ── Log ─────────────────────────────────────────────────────
        res.x.col(k)       = ds.to_vector();
        res.u.col(k)       << out.cmd.thrust, out.cmd.omega_cmd;
        res.p_ref.col(k)   = out.p_ref;
        res.q_ref.col(k)   = out.q_ref;
        res.t(k)           = t_k;
        res.progress(k)    = progress;
        res.solve_ms(k)    = out.solve_time_s * 1e3;
        res.status(k)      = out.solver_status;
        res.n_steps        = k + 1;
        res.t_final        = t_k;
        res.progress_final = progress;

        // ── Rate control ────────────────────────────────────────────
        double elapsed = std::chrono::duration<double>(Clock::now() - tic).count();
        double remaining = cfg_.dt - elapsed;
        if (remaining > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining));

        double loop_ms_k = std::chrono::duration<double, std::milli>(
            Clock::now() - tic).count();
        res.loop_ms(k) = loop_ms_k;

        // ── Periodic log ────────────────────────────────────────────
        if (k % static_cast<int>(5.0 / cfg_.dt) == 0) {
            double e_pos = (ds.pos - out.p_ref).norm();
            RCLCPP_INFO(muj_->get_logger(),
                "[RUN] t=%.1f  progress=%.2f/%.1f  |e|=%.3f  solve=%.2f ms  "
                "loop=%.1f ms  fails=%d",
                t_k, progress, cfg_.progress_max, e_pos,
                out.solve_time_s*1e3, loop_ms_k, res.solve_fails);
        }

        // ── Crash check: collision topic (walls & floor) ─────────────
        // Flag was cleared once before loop. Never cleared again during
        // RUN — any collision sets it permanently until next trial.
        if (cfg_.abort_on_crash && muj_->is_crashed()) {
            stop_reason = "COLLISION (MuJoCo contact)";
            RCLCPP_WARN(muj_->get_logger(),
                "[STOP] %s at t=%.1f s  pos=[%.2f, %.2f, %.2f]",
                stop_reason.c_str(), t_k, ds.pos.x(), ds.pos.y(), ds.pos.z());
            res.crashed = true;
            res.n_steps = k + 1;
            break;
        }

        // ── Floor crash (z too low during flight) ───────────────────
        if (ds.pos.z() < cfg_.z_min) {
            stop_reason = "FLOOR_CRASH (z < " + std::to_string(cfg_.z_min) + "m)";
            RCLCPP_WARN(muj_->get_logger(),
                "[STOP] %s  z=%.3f at t=%.1f s",
                stop_reason.c_str(), ds.pos.z(), t_k);
            res.crashed = true;
            res.n_steps = k + 1;
            break;
        }

        // ── Height check (escaped workspace) ────────────────────────
        if (ds.pos.z() > cfg_.z_max) {
            stop_reason = "CEILING_CRASH (z > " + std::to_string(cfg_.z_max) + "m)";
            RCLCPP_WARN(muj_->get_logger(),
                "[STOP] %s  z=%.1f at t=%.1f s",
                stop_reason.c_str(), ds.pos.z(), t_k);
            res.crashed = true;
            res.n_steps = k + 1;
            break;
        }

        // ── Stall detection (velocity near zero for too long) ────────
        {
            double v_norm = ds.vel.norm();
            // Arm stall detection once the drone reaches meaningful speed
            if (!stall_armed && v_norm > 1.0) {
                stall_armed = true;
            }
            if (stall_armed) {
                if (v_norm < cfg_.stall_vel_thr) {
                    stall_timer += cfg_.dt;
                } else {
                    stall_timer = 0.0;
                }
                if (stall_timer >= cfg_.stall_timeout) {
                    stop_reason = "STALL (v < " + std::to_string(cfg_.stall_vel_thr)
                                + " m/s for " + std::to_string(cfg_.stall_timeout) + "s)";
                    RCLCPP_WARN(muj_->get_logger(),
                        "[STOP] %s  v=%.3f at t=%.1f s (progress=%.2f)",
                        stop_reason.c_str(), v_norm, t_k, progress);
                    res.stalled = true;
                    res.n_steps = k + 1;
                    break;
                }
            }
        }
    }

    // Store stop reason in result
    res.stop_reason = stop_reason;

    RCLCPP_INFO(muj_->get_logger(),
        "[PROTOCOL] RUN ended — reason: %s  t=%.1f s  progress=%.2f",
        stop_reason.c_str(), res.t_final, res.progress_final);

    // PD stabilization: drone is tilted & fast → active attitude+position control
    muj_->start_pd_hold(cfg_.P0, cfg_.mass, cfg_.gravity);
    std::this_thread::sleep_for(2000ms);   // 2s to decelerate and level out
    muj_->stop_pd_hold();

    // Trim logs to actual steps
    int n = res.n_steps;
    res.x.conservativeResize(Eigen::NoChange, n);
    res.u.conservativeResize(Eigen::NoChange, n);
    res.p_ref.conservativeResize(Eigen::NoChange, n);
    res.q_ref.conservativeResize(Eigen::NoChange, n);
    res.t.conservativeResize(n);
    res.progress.conservativeResize(n);
    res.solve_ms.conservativeResize(n);
    res.loop_ms.conservativeResize(n);
    res.status.conservativeResize(n);

    return res;
}

// ═══════════════════════════════════════════════════════════════════════════
//  execute(): full protocol
// ═══════════════════════════════════════════════════════════════════════════

SilResult SilProtocol::execute() {
    RCLCPP_INFO(muj_->get_logger(),
        "╔══════════════════════════════════════════╗\n"
        "║       SiL PROTOCOL — START               ║\n"
        "╚══════════════════════════════════════════╝");

    // ── Phase 1: RELOAD ─────────────────────────────────────────────
    if (!reload_()) {
        RCLCPP_ERROR(muj_->get_logger(), "[PROTOCOL] RELOAD failed — aborting");
        return {};
    }

    // ── Phase 2: PD HOLD ────────────────────────────────────────────
    pd_hold_();

    // ── Phase 3: RUN ────────────────────────────────────────────────
    SilResult res = run_loop_();

    // No reload here — drone stays visible. Next execute() starts with reload.
    RCLCPP_INFO(muj_->get_logger(), "[PROTOCOL] Drone stabilised at P0 — ready");

    // ── Summary ─────────────────────────────────────────────────────
    int n = res.n_steps;
    if (n > 0) {
        RCLCPP_INFO(muj_->get_logger(),
            "\n╔══════════════════════════════════════════════════╗\n"
            "║       SiL PROTOCOL — SUMMARY                     ║\n"
            "╠══════════════════════════════════════════════════╣\n"
            "║  Stop reason: %-34s ║\n"
            "║  Steps:       %6d                               ║\n"
            "║  Duration:    %6.1f s                              ║\n"
            "║  Progress:    %5.2f / %.2f                         ║\n"
            "║  Completed:   %s                                   ║\n"
            "║  Crashed:     %s                                   ║\n"
            "║  Stalled:     %s                                   ║\n"
            "║  Solve fails: %d / %d                              ║\n"
            "║  Solve avg:   %.2f ms                              ║\n"
            "║  Solve max:   %.2f ms                              ║\n"
            "╚══════════════════════════════════════════════════╝",
            res.stop_reason.c_str(),
            n, res.t_final, res.progress_final, cfg_.progress_max,
            res.completed ? "YES" : "NO",
            res.crashed   ? "YES" : "NO",
            res.stalled   ? "YES" : "NO",
            res.solve_fails, n,
            res.solve_ms.mean(), res.solve_ms.maxCoeff());
    }

    return res;
}

}  // namespace quadrotor_mpc
