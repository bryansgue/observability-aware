#pragma once

#include "quadrotor_mpc/common/types.hpp"
#include "quadrotor_mpc/trajectory/arc_length.hpp"
#include <vector>

namespace quadrotor_mpc {

/// Waypoint set: uniformly sampled in arc-length with attitude reference.
struct PathWaypoints {
    std::vector<double> s;      // arc-length knots
    std::vector<Vec3>   pos;    // positions
    std::vector<Vec3>   tang;   // unit tangents
    std::vector<Quat4>  quat;   // attitude quaternions [qw,qx,qy,qz]
    double s_max = 0.0;
};

/// Build waypoints with curvature-compatible attitude reference.
/// Implements eqs. (12)-(14) from the paper:
///   b3 = (a_lat + g*e3) / ||...||       (thrust direction)
///   b1 = (I - b3*b3^T)*tang / ||...||   (tangent projected)
///   b2 = b3 × b1
PathWaypoints build_waypoints(
    const ArcLengthPath& path,
    double s_max,
    int n_waypoints,
    double reference_speed,
    double gravity = 9.81,
    double max_tilt_deg = 60.0);

/// Interpolate quaternion at arc-length s from waypoint set (piecewise-linear SLERP).
Quat4 quat_interp(double s, const PathWaypoints& wp);

/// Interpolate position at arc-length s from waypoint set.
Vec3 pos_interp(double s, const PathWaypoints& wp);

/// Interpolate tangent at arc-length s from waypoint set.
Vec3 tang_interp(double s, const PathWaypoints& wp);

/// Find arc-length of the geometrically nearest path point.
/// Searches in [s_hint, s_hint+window] with step ds.
/// Returns s_nearest >= s_hint (monotonic).
inline double find_nearest_s(const Vec3& pos, const PathWaypoints& wp,
                              double s_hint, double s_max,
                              double window = 5.0, double ds = 0.05)
{
    double best_s = s_hint;
    double best_d2 = 1e18;
    const double s_end = std::min(s_hint + window, s_max);
    for (double s = s_hint; s <= s_end; s += ds) {
        Vec3 dp = pos - pos_interp(s, wp);
        double d2 = dp.squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best_s = s; }
    }
    return std::max(best_s, s_hint);
}

// ─────────────────────────────────────────────────────────────────────────────
// Temporal attitude reference (no arc-length required)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute attitude quaternion from velocity and acceleration at a trajectory point.
///
/// Implements the b3/b1/b2 frame construction (eqs. 12–14):
///   b3 = normalize(accel + g·e3)          (thrust direction)
///   b1 = normalize(vel − (vel·b3)·b3)     (heading projected ⊥ b3)
///   b2 = b3 × b1
///
/// @param vel        Trajectory velocity [m/s]
/// @param accel      Trajectory acceleration [m/s²]
/// @param g          Gravity magnitude [m/s²]
/// @param max_tilt_deg  Maximum tilt angle [deg]
/// @param q_prev     Previous quaternion (for hemisphere continuity)
Quat4 quat_from_traj(const Vec3& vel, const Vec3& accel,
                     double g, double max_tilt_deg,
                     const Quat4& q_prev = Quat4(1, 0, 0, 0));

}  // namespace quadrotor_mpc
