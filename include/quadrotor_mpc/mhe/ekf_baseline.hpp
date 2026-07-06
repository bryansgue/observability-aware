#pragma once
/**
 * EkfBaseline — augmented extended Kalman filter for joint state + parameter
 * estimation, the recursive baseline compared against the OA-MHE.
 *
 * State x ∈ ℝ¹⁰ = [p(3), v(3), m, d(3)]   (identical scope to MheTransController)
 * Known inputs per step: T (thrust), a = R·e3 (thrust axis), dt.
 * Measurements:
 *   y = [p, v] ∈ ℝ⁶            (odometry, linear)
 *   s = (T/m)·a + d − c·|v|·v   (accelerometer specific force, nonlinear)
 *
 * Dynamics (continuous):
 *   ṗ = v,  v̇ = −g·e3 + (T/m)·a + d − c·|v|·v,  ṁ = 0,  ḋ = 0   (random walks)
 *
 * Runs as a PASSIVE observer alongside the MHE: it sees the exact same
 * measurement stream, so the estimation comparison is on identical data. It does
 * not drive the controller.
 *
 * Measurement covariances are matched to the MHE weights for a fair comparison:
 *   R_p = 1/50, R_v = 1/10  (MHE R_inv = diag([50,50,50,10,10,10]))
 *   R_s = 1/W_IMU = 1/2.0   (MHE accelerometer weight)
 */
#include "quadrotor_mpc/common/types.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <deque>

namespace quadrotor_mpc {

class EkfBaseline {
public:
    using Vec10 = Eigen::Matrix<double, 10, 1>;
    using Mat10 = Eigen::Matrix<double, 10, 10>;

    void set_drag(double c) { drag_c_ = c; }
    // Enable the observability-aware mass gate: freeze the mass update when the
    // NORMALIZED identifiability sigma_tilde = sigma / sum||regressor||^2 in [0,1]
    // over the window is below thr. Normalizing by the regressor energy makes the
    // threshold dimensionless and maneuver-independent (the m^4 factor of the
    // (T/m^2)a regressor cancels, so sigma_tilde = sigma/sum(T^2), the paper's
    // eq. (snorm)); thr ~= 0.012 separates hover from maneuver and probe alike.
    void enable_gate(double thr) { gate_ = true; sigma_thr_ = thr; }
    // Baseline: gate on the generic regressor energy (a conventional PE proxy that
    // does not isolate the mass), instead of the Schur-complement sigma_tilde.
    void enable_energy_gate(double thr) { gate_ = true; energy_gate_ = true; energy_thr_ = thr; }

    void reset(const Vec3& p0, const Vec3& v0, double m0, const Vec3& d0) {
        x_.setZero();
        x_.head<3>() = p0; x_.segment<3>(3) = v0; x_(6) = m0; x_.tail<3>() = d0;
        P_.setZero();
        P_.diagonal() << 1e-2,1e-2,1e-2, 1e-2,1e-2,1e-2, 1e-1, 1e-1,1e-1,1e-1;
        creg_win_.clear(); sigma_m_ = 0.0; high_count_ = 0; gate_now_ = false; seeded_ = true;
    }

    bool seeded() const { return seeded_; }
    double m_hat() const { return x_(6); }
    double sigma_m() const { return sigma_m_; }       // raw mass identifiability (N*Var of regressor)
    double sigma_norm() const { return sigma_norm_; } // normalized identifiability sigma_tilde in [0,1]
    bool   gate_active() const { return gate_now_; }   // freeze mass (computed in step with hysteresis)
    Vec3   d_hat() const { return x_.tail<3>(); }
    Vec3   pos()   const { return x_.head<3>(); }
    Vec3   vel()   const { return x_.segment<3>(3); }

    // One predict + (optional) odom-update + accelerometer-update.
    // use_odom=false → IMU-only step (sparse-position experiments): the filter
    // dead-reckons on the accelerometer + model and drifts until the next fix.
    void step(const Eigen::Matrix<double,6,1>& y, double T, const Vec3& a,
              const Vec3& s, double dt, bool use_odom = true) {
        const double g = 9.81;
        const Vec3 e3(0, 0, 1);

        // ── Observability metric: sigma_m = Var((T/m^2)a) over a sliding window ─
        // This is the Schur complement of the mass in the identifiability Gramian
        // (Prop. 1): zero at hover (constant thrust axis), grows under maneuvers.
        {
            double mm = std::max(x_(6), 0.2);
            creg_win_.push_back((T / (mm * mm)) * a);
            if ((int)creg_win_.size() > WIN_) creg_win_.pop_front();
            Vec3 mean = Vec3::Zero();
            double sumsq = 0.0;
            for (const auto& c : creg_win_) { mean += c; sumsq += c.squaredNorm(); }
            mean /= (double)creg_win_.size();
            double s = 0.0;
            for (const auto& c : creg_win_) s += (c - mean).squaredNorm();
            sigma_m_    = s;                                  // = N * Var(regressor)
            sigma_norm_ = (sumsq > 1e-12) ? s / sumsq : 0.0;  // normalized in [0,1]
            sumsq_      = sumsq;                               // regressor energy (generic PE proxy)
        }
        // Hysteresis: only declare the mass observable (open the gate) after
        // SUSTAINED excitation. The decision normally uses the normalized
        // sigma_tilde (the Schur complement isolating the mass). In energy_gate_
        // mode it instead uses the raw regressor energy sumsq_ — a GENERIC
        // persistency-of-excitation proxy that does NOT isolate the mass--vertical-
        // force coupling, included as a baseline: in hover the energy is high
        // (constant high thrust) so this proxy never freezes and the mass drifts.
        const double gate_sig = energy_gate_ ? sumsq_       : sigma_norm_;
        const double gate_thr = energy_gate_ ? energy_thr_  : sigma_thr_;
        if (gate_sig > gate_thr) ++high_count_; else high_count_ = 0;
        gate_now_ = gate_ && (high_count_ < SUSTAIN_);
        const bool gate_now = gate_now_;

        // ── Predict ───────────────────────────────────────────────────────────
        Vec3 v = x_.segment<3>(3);
        double m = std::max(x_(6), 0.2);
        Vec3 d = x_.tail<3>();
        double vmag = std::sqrt(v.dot(v) + 1e-6);

        Vec3 acc = -g * e3 + (T / m) * a + d - drag_c_ * vmag * v;
        Vec10 xp = x_;
        xp.head<3>() += v * dt;
        xp.segment<3>(3) += acc * dt;
        // m, d unchanged (random walk mean)

        // Continuous Jacobian A = ∂f/∂x, then F = I + A·dt
        Mat10 A = Mat10::Zero();
        A.block<3,3>(0,3) = Eigen::Matrix3d::Identity();          // ∂ṗ/∂v
        Eigen::Matrix3d dragJ = drag_c_ *
            (vmag * Eigen::Matrix3d::Identity() + (v * v.transpose()) / vmag);
        A.block<3,3>(3,3) = -dragJ;                                // ∂v̇/∂v
        A.block<3,1>(3,6) = -(T / (m*m)) * a;                      // ∂v̇/∂m
        A.block<3,3>(3,7) = Eigen::Matrix3d::Identity();           // ∂v̇/∂d
        Mat10 F = Mat10::Identity() + A * dt;

        // When the mass is unobservable (gate active), freeze its covariance too:
        // no process noise on mass → P(6,6) stays bounded, so a brief gate-off
        // spike cannot trigger a giant ill-conditioned mass jump.
        Mat10 Q = Mat10::Zero();
        Q.diagonal() << q_p_,q_p_,q_p_, q_v_,q_v_,q_v_, (gate_now ? 0.0 : q_m_), q_d_,q_d_,q_d_;
        P_ = F * P_ * F.transpose() + Q;
        x_ = xp;

        // ── Update 1: odometry y = [p, v] (linear H = [I6 | 0]) ────────────────
        if (use_odom) {
            Eigen::Matrix<double,6,10> H = Eigen::Matrix<double,6,10>::Zero();
            H.block<6,6>(0,0) = Eigen::Matrix<double,6,6>::Identity();
            Eigen::Matrix<double,6,1> r = y - x_.head<6>();
            Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
            R.diagonal() << R_p_,R_p_,R_p_, R_v_,R_v_,R_v_;
            Eigen::Matrix<double,6,6> S = H * P_ * H.transpose() + R;
            Eigen::Matrix<double,10,6> K = P_ * H.transpose() * S.inverse();
            if (gate_now) K.row(6).setZero();   // freeze mass: unobservable → don't update
            x_ += K * r;
            P_ = (Mat10::Identity() - K * H) * P_;
        }

        // ── Update 2: accelerometer s = (T/m)a + d − c|v|v (nonlinear) ─────────
        {
            Vec3 v2 = x_.segment<3>(3);
            double m2 = std::max(x_(6), 0.2);
            Vec3 d2 = x_.tail<3>();
            double vm2 = std::sqrt(v2.dot(v2) + 1e-6);
            Vec3 h = (T / m2) * a + d2 - drag_c_ * vm2 * v2;

            Eigen::Matrix<double,3,10> H = Eigen::Matrix<double,3,10>::Zero();
            Eigen::Matrix3d dragJ2 = drag_c_ *
                (vm2 * Eigen::Matrix3d::Identity() + (v2 * v2.transpose()) / vm2);
            H.block<3,3>(0,3) = -dragJ2;               // ∂s/∂v
            H.block<3,1>(0,6) = -(T / (m2*m2)) * a;    // ∂s/∂m
            H.block<3,3>(0,7) = Eigen::Matrix3d::Identity();  // ∂s/∂d
            Vec3 r = s - h;
            Eigen::Matrix3d R = R_s_ * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d S = H * P_ * H.transpose() + R;
            Eigen::Matrix<double,10,3> K = P_ * H.transpose() * S.inverse();
            if (gate_now) K.row(6).setZero();   // freeze mass: unobservable → don't update
            x_ += K * r;
            P_ = (Mat10::Identity() - K * H) * P_;
        }

        // ── Bounds (match MHE constraints) ─────────────────────────────────────
        x_(6) = std::clamp(x_(6), 0.5, 3.0);
        for (int i = 7; i < 10; ++i) x_(i) = std::clamp(x_(i), -2.0, 2.0);
    }

private:
    Vec10 x_ = Vec10::Zero();
    Mat10 P_ = Mat10::Identity();
    bool   seeded_  = false;
    double drag_c_  = 0.0;
    // observability-aware mass gate
    std::deque<Vec3> creg_win_;            // window of the mass regressor (T/m^2)a
    double sigma_m_    = 0.0;
    double sigma_norm_ = 0.0;              // normalized identifiability sigma_tilde in [0,1]
    bool   gate_         = false;
    bool   energy_gate_  = false;   // generic-PE-proxy baseline mode
    double energy_thr_   = 0.0;
    double sumsq_        = 0.0;      // regressor energy over the window
    bool   gate_now_  = false;
    double sigma_thr_ = 0.0;
    int    high_count_ = 0;                // consecutive steps with sigma_m > thr
    static const int WIN_     = 100;       // sigma_m window (1 s) — long enough to span a probe cycle
    static const int SUSTAIN_ = 25;        // steps of sustained excitation to open the gate (~0.25 s):
                                           // rejects isolated sigma spikes while letting the sustained
                                           // active probe re-open the gate quickly enough to identify.
    // measurement covariances (matched to MHE weights)
    static constexpr double R_p_ = 1.0 / 50.0;
    static constexpr double R_v_ = 1.0 / 10.0;
    static constexpr double R_s_ = 1.0 / 2.0;
    // process noise (per step) — env-overridable for fair tuning vs the MHE
    static double env_(const char* k, double def) {
        const char* v = std::getenv(k); return v ? std::atof(v) : def;
    }
    const double q_p_ = 1e-6;
    const double q_v_ = 1e-3;
    const double q_m_ = env_("EKF_QM", 1e-5);   // matched to MHE mass process noise
    const double q_d_ = env_("EKF_QD", 1e-2);   // disturbance random-walk (fair tuning)
};

}  // namespace quadrotor_mpc
