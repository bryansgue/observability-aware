#pragma once

namespace quadrotor_mpc {

struct QuadParams {
    // Physical
    double mass    = 1.05;      // [kg] REAL mass = sum of MuJoCo geom masses (core .85 + arms .14 + thrusters .048 + props .012)
    double g       = 9.81;      // [m/s²]
    double tau_rc  = 0.03;      // [s] first-order rate lag

    // Thrust limits
    double T_max = 5.0 * 9.81;  // ≈49.05 N
    double T_min = 0.0;

    // Rate command limits
    double W_max = 20.0;         // [rad/s] max body rate command

    // MPCC / progress dynamics limits
    double vtheta_max   = 14.0;       // [m/s]
    double vtheta_min   = 0.0;
    double atheta_max   = 4.0 * 9.81; // [m/s²]
    double atheta_min   = -4.0 * 9.81;
    double df_max       = 500.0;      // [N/s] max thrust rate

    // Geometric constraints
    double D_max_lag    = 0.5;   // [m] lag constraint in MPCC
    double theta_margin = 30.0;  // [m] extra θ headroom beyond s_max

    // Attitude reference
    double att_ref_max_tilt_deg = 60.0; // [deg]
    double att_ref_speed        = 15.0; // [m/s] nominal speed for attitude ref
};

struct SimParams {
    double dt      = 0.01;    // [s] control period (100 Hz)
    double t_final = 85.0;    // [s] max simulation time budget
};

struct MpccParams {
    double dt           = 0.01;  // [s] control loop period (100 Hz)
    double t_prediction = 1.5;   // [s] prediction horizon (non-uniform, N=50)

    // Cost weights (runtime-tunable via acados params)
    Vec3 Q_ec   = Vec3(100.0, 100.0, 100.0);   // contouring error
    Vec3 Q_el   = Vec3(13.0,  13.0,  13.0);    // lag error
    Vec3 Q_q    = Vec3(0.5,   0.5,   0.5);     // attitude (log map)
    Eigen::Vector4d U_mat = Eigen::Vector4d(0.1, 0.3, 0.3, 0.3); // control effort
    double Q_s  = 15.0;    // progress weight (-Q_s * v_θ)
    double W_df = 0.001;   // thrust rate penalty (W_df * Δf²)
};

struct NmpcParams {
    double dt           = 0.01;   // [s] control loop period (100 Hz)
    double t_prediction = 1.0;    // [s] prediction horizon

    /// N is computed automatically: N = t_prediction / dt
    int N_horizon() const { return static_cast<int>(t_prediction / dt + 0.5); }

    // Cost weights (default, can be overridden at runtime)
    Vec3 Q_pos   = Vec3(150.0, 150.0, 150.0);
    Vec3 Q_att   = Vec3(50.0, 50.0, 50.0);
    Eigen::Vector4d R_u = Eigen::Vector4d(0.1, 0.5, 0.5, 0.5);
};


}  // namespace quadrotor_mpc
