#pragma once

#include "quadrotor_mpc/common/types.hpp"
#include <vector>

namespace quadrotor_mpc {

/// 3D Lissajous trajectory with frequency ratio 1:2:3.
///   x(t) = Ax*sin(1*w*t) + Cx
///   y(t) = Ay*sin(2*w*t)
///   z(t) = Az*sin(3*w*t) + Cz
class LissajousTrajectory {
public:
    double w  = 2.0;    // base frequency [rad/s]
    double Ax = 5.0, Ay = 1.25, Az = 0.75;
    double Cx = 2.5, Cz = 1.5;
    double t_final = 63.0;  // parametric interval [s]

    /// Position at parameter t
    Vec3 position(double t) const;

    /// Velocity dp/dt at parameter t
    Vec3 velocity(double t) const;

    /// Acceleration d²p/dt² at parameter t
    Vec3 acceleration(double t) const;

    /// Sample N_samples points over [0, t_final]
    /// Returns positions (3 x N) and velocities (3 x N)
    void sample(int N_samples,
                std::vector<Vec3>& positions,
                std::vector<Vec3>& velocities,
                std::vector<double>& t_values) const;
};

}  // namespace quadrotor_mpc
