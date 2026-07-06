"""
quad_config.py — Physical and OCP constants for C++ solver generation.

Self-contained: no imports from the parent Python project.
These values must match params.hpp in the C++ codebase.
"""

import numpy as np

# ── Gravity ──────────────────────────────────────────────────────────────────
G = 9.81  # [m/s^2]

# ── Quadrotor physical parameters ────────────────────────────────────────────
MASS   = 1.08   # [kg]
TAU_RC = 0.03   # [s] rate-controller time constant

# ── Thrust limits [N] ───────────────────────────────────────────────────────
T_MAX = 5 * G   # ~49.05 N
T_MIN = 0.0

# ── Angular rate command limits [rad/s] ─────────────────────────────────────
W_MAX = 20.0

# ── Progress dynamics limits ────────────────────────────────────────────────
VTHETA_MIN =  0.0        # [m/s]
VTHETA_MAX = 14.0         # [m/s]
ATHETA_MIN = -4 * G       # [m/s^2]
ATHETA_MAX =  4 * G       # [m/s^2]

# ── Thrust rate limits [N/s] ───────────────────────────────────────────────
DF_MAX = 500.0            # [N/s] max thrust rate of change

# ── Lag constraint ──────────────────────────────────────────────────────────
D_MAX_LAG = 0.5  # [m] max arc-length gap between θ and drone position

# ── Trajectory ──────────────────────────────────────────────────────────────
TRAJ_VALUE = 2.0          # [rad/s] Lissajous base angular frequency
TRAJECTORY_T_FINAL = 63   # [s] parameter interval for path geometry
FREC = 100                # [Hz] control frequency
N_WAYPOINTS = 400         # B-spline interpolation knots
S_MAX_MANUAL = 100        # [m] arc-length limit
THETA_MARGIN = 30.0       # [m] extra headroom for θ beyond s_max (≈ vtheta_max * T_horizon)

# ── Attitude reference construction ─────────────────────────────────────────
ATTITUDE_REF_SPEED = 15.0        # [m/s] nominal speed for attitude reference
ATTITUDE_REF_MAX_TILT_DEG = 60.0 # [deg] max tilt in reference quaternion


def trayectoria():
    """Return 6 lambdas: (xd, yd, zd, xdp, ydp, zdp).

    Lissajous 3D, frequency ratio 1:2:3.
    x(t) = 5.0*sin(w*t) + 2.5
    y(t) = 1.25*sin(2*w*t)
    z(t) = 0.75*sin(3*w*t) + 1.5
    """
    w = TRAJ_VALUE
    xd  = lambda t: 5.0  * np.sin(1.0 * w * t) + 2.5
    yd  = lambda t: 1.25 * np.sin(2.0 * w * t)
    zd  = lambda t: 0.75 * np.sin(3.0 * w * t) + 1.5
    xdp = lambda t: 5.0  * 1.0 * w * np.cos(1.0 * w * t)
    ydp = lambda t: 1.25 * 2.0 * w * np.cos(2.0 * w * t)
    zdp = lambda t: 0.75 * 3.0 * w * np.cos(3.0 * w * t)
    return xd, yd, zd, xdp, ydp, zdp
