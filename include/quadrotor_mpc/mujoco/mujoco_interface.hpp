#pragma once
/**
 * MujocoInterface — Reusable ROS2 node for MuJoCo quadrotor SiL.
 *
 * Subscribes  /quadrotor/odom        (nav_msgs/Odometry)
 * Publishes   /quadrotor/trpy_cmd    (quadrotor_msgs/TRPYCommand)
 * Subscribes  /quadrotor/collision   (std_msgs/Bool)
 *
 * Thread-safe: state protected by mutex so the control loop (main thread)
 * can read while the ROS2 executor writes from callbacks.
 *
 * Reusable by NMPC, MPCC, DQ-MPCC — controller-agnostic.
 */

#include "quadrotor_mpc/common/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mujoco_ros_utils/msg/external_force.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <quadrotor_msgs/msg/trpy_command.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace quadrotor_mpc {

/// Full 13-state snapshot from MuJoCo odometry
struct DroneState {
    Vec3  pos   = Vec3::Zero();       // world frame [m]
    Vec3  vel   = Vec3::Zero();       // world frame [m/s]
    Quat4 quat  = Quat4(1,0,0,0);    // [qw,qx,qy,qz]
    Vec3  omega = Vec3::Zero();       // body frame  [rad/s]
    Vec3  accel = Vec3::Zero();       // body frame raw IMU specific force [m/s²] (NOISY)
    Vec3  gyro  = Vec3::Zero();       // body frame IMU gyro [rad/s] (rotational measurement)
    Vec3  ext_force  = Vec3::Zero();   // world-frame GROUND-TRUTH external force  [N]   (validation)
    Vec3  ext_torque = Vec3::Zero();   // world-frame GROUND-TRUTH external torque [N·m] (validation)

    /// Pack into 13-element vector [p, v, q, ω]
    Eigen::Matrix<double,13,1> to_vector() const {
        Eigen::Matrix<double,13,1> x;
        x << pos, vel, quat, omega;
        return x;
    }
};

/// Acro-mode command: thrust [N] + desired body rates [rad/s]
struct AcroCommand {
    double thrust = 0.0;
    Vec3   omega_cmd = Vec3::Zero();  // [wx, wy, wz]
};

class MujocoInterface : public rclcpp::Node {
public:
    explicit MujocoInterface(
        const std::string& node_name  = "mujoco_controller",
        const std::string& odom_topic = "/quadrotor/odom",
        const std::string& cmd_topic  = "/quadrotor/trpy_cmd");

    // ── State access (thread-safe) ──────────────────────────────────────
    DroneState get_state() const;
    bool is_connected() const;
    bool is_crashed() const;
    void clear_crash();

    // ── Command publishing ──────────────────────────────────────────────
    void send_cmd(const AcroCommand& cmd);
    void send_cmd(double thrust, double wx, double wy, double wz);
    void send_zero();

    // ── Estimated disturbance publishing (validation / real-time) ───────
    // Publishes the estimated external force on /quadrotor/d_hat using the
    // SAME message type and units (N, world frame) as the ground-truth
    // /quadrotor/external_force, for a 1-to-1 comparison in PlotJuggler.
    void publish_dhat(const Vec3& force_world_N);

    // ── Deterministic perturbation injection (controller-defined) ───────
    // Publishes a commanded external wrench on /quadrotor/external_force_cmd.
    // MuJoCo applies it and re-publishes it on /quadrotor/external_force, which
    // is the ground truth the estimator validates against. Because WE define the
    // signal, the disturbance is fully deterministic and reproducible: send zero
    // during reset/takeoff, then the designed profile during flight.
    void publish_ext_force_cmd(const Vec3& force_world_N,
                               const Vec3& torque_world_Nm = Vec3::Zero());

    // ── Perturbation trigger (deterministic, service-controlled) ────────
    // The controller exposes /quadrotor/start_perturbation (std_srvs/SetBool):
    // true → begin injecting the designed force, false → stop. The protocol
    // calls it at a verified point (after reset + hover) so the disturbance
    // onset is deterministic and logged; it is also callable externally for
    // manual demos. The injection itself is gated on perturbation_on().
    void set_perturbation(bool on) { pert_on_.store(on); }
    bool perturbation_on() const { return pert_on_.load(); }

    // ── PD position hold (background thread) ────────────────────────────
    struct PdGains {
        double kp_xy  = 4.0,  kd_xy  = 2.5;
        double kp_z   = 8.0,  kd_z   = 4.0;
        double kp_att = 6.0,  kp_yaw = 2.0;
    };

    void start_pd_hold(const Vec3& target, double mass, double g,
                       const PdGains& gains);
    void start_pd_hold(const Vec3& target, double mass, double g = 9.81);
    void stop_pd_hold();

    // ── Simulator control ───────────────────────────────────────────────
    bool reset_sim(double timeout_sec = 5.0);

    // ── Convergence wait ────────────────────────────────────────────────
    bool wait_for_connection(double timeout_sec = 10.0) const;
    bool wait_for_pd_convergence(const Vec3& target,
                                  double settle_dist = 0.30,
                                  double settle_time = 1.0,
                                  double timeout_sec = 15.0) const;

private:
    // Callbacks
    void odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_cb_(const sensor_msgs::msg::Imu::SharedPtr msg);
    void extforce_cb_(const mujoco_ros_utils::msg::ExternalForce::SharedPtr msg);
    void collision_cb_(const std_msgs::msg::Bool::SharedPtr msg);

    // State
    mutable std::mutex state_mtx_;
    DroneState state_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> crashed_{false};

    // PD hold
    std::atomic<bool> pd_active_{false};
    std::thread pd_thread_;
    void pd_loop_(Vec3 target, double mass, double g, PdGains gains);

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mujoco_ros_utils::msg::ExternalForce>::SharedPtr extforce_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    rclcpp::Publisher<quadrotor_msgs::msg::TRPYCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<mujoco_ros_utils::msg::ExternalForce>::SharedPtr dhat_pub_;
    rclcpp::Publisher<mujoco_ros_utils::msg::ExternalForce>::SharedPtr extcmd_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_cli_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_pert_srv_;
    std::atomic<bool> pert_on_{false};
};

}  // namespace quadrotor_mpc
