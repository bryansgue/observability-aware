#!/usr/bin/env python3
"""
plot_current_results.py — current state of the IMU momentum-observer MHE.

results/current_results.png (3x2):
  (a) position filtering     — noisy odom vs MHE estimate (zoom)
  (b) velocity filtering     — noisy odom vs MHE estimate (zoom)
  (c) disturbance d(t)       — virtual force sensor (momentum observer)
  (d) mass m̂(t)             — true mass = 1.05 kg (sum of MuJoCo geom masses)
  (e) rate constant τ_rc(t)  — RLS estimate
  (f) thrust constant τ_f(t) — RLS estimate
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 9.5, "axes.labelsize": 9.5,
    "axes.titlesize": 10.5, "legend.fontsize": 8, "figure.dpi": 150,
    "lines.linewidth": 1.4, "axes.grid": True, "grid.alpha": 0.3,
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
a = np.loadtxt(os.path.join(RES, "nmpc_mhe_v2_w2.0.csv"), delimiter=",", skiprows=1)
t = a[:, 0]

PX, VX, PXh, VXh = 1, 4, 42, 45
M_HAT, TAU_RC, TAU_F = 32, 33, 38
DX, DY, DZ = 34, 35, 36

TRUE_M    = 1.05     # REAL mass = sum of MuJoCo geom masses (verified from the XML)
TAU_RC_ID = 0.055    # identified converged value
TAU_F_ID  = 0.007
zoom = (t > 30.3) & (t < 30.9)
run  = t < 58

fig, axes = plt.subplots(3, 2, figsize=(11, 10))

# (a) position filtering
ax = axes[0, 0]
ax.plot(t[zoom], a[zoom, PX],  ".-", color="gray", ms=4, lw=0.7, label="odom medido (sucio)")
ax.plot(t[zoom], a[zoom, PXh], "-",  color="#1f77b4", lw=2, label="MHE filtrado")
ax.set_ylabel("px [m]"); ax.set_xlabel("t [s]")
ax.set_title("(a) Filtrado de posición"); ax.legend(loc="best")

# (b) velocity filtering
ax = axes[0, 1]
ax.plot(t[zoom], a[zoom, VX],  ".-", color="gray", ms=4, lw=0.7, label="odom medido (sucio)")
ax.plot(t[zoom], a[zoom, VXh], "-",  color="#ff7f0e", lw=2, label="MHE filtrado")
ax.set_ylabel("vx [m/s]"); ax.set_xlabel("t [s]")
ax.set_title("(b) Filtrado de velocidad"); ax.legend(loc="best")

# (c) disturbance (virtual force sensor)
ax = axes[1, 0]
ax.plot(t[run], a[run, DX], color="#d62728", label="d_x")
ax.plot(t[run], a[run, DY], color="#2ca02c", label="d_y")
ax.plot(t[run], a[run, DZ], color="#9467bd", label="d_z")
ax.axhline(0, color="k", lw=0.6, ls=":")
ax.set_ylabel(r"$\hat d$ [m/s²]"); ax.set_xlabel("t [s]")
ax.set_title(r"(c) Sensor virtual de fuerza $\hat d$ (sin viento → ≈0)")
ax.legend(loc="best", ncol=3)

# (d) mass
ax = axes[1, 1]
ax.axhline(TRUE_M, color="k", ls="--", lw=1.0, label=f"real m={TRUE_M} (MuJoCo)")
ax.plot(t[run], a[run, M_HAT], color="#1f77b4", label=r"$\hat m$")
ax.scatter([0], [0.60], color="#d62728", s=30, zorder=5, label="init=0.60")
ax.set_ylabel(r"$\hat m$ [kg]"); ax.set_xlabel("t [s]"); ax.set_ylim(0.5, 1.2)
ax.set_title("(d) Masa  (m̂→1.05 = exacto)"); ax.legend(loc="lower right")

# (e) rate constant tau_rc
ax = axes[2, 0]
ax.axhline(TAU_RC_ID, color="k", ls="--", lw=1.0, label=f"identif. ≈{TAU_RC_ID}")
ax.plot(t[run], a[run, TAU_RC], color="#8c564b", label=r"$\hat\tau_{rc}$")
ax.set_ylabel(r"$\hat\tau_{rc}$ [s]"); ax.set_xlabel("t [s]"); ax.set_ylim(0, 0.12)
ax.set_title("(e) Constante de body-rate  τ_rc"); ax.legend(loc="best")

# (f) thrust constant tau_f
ax = axes[2, 1]
ax.axhline(TAU_F_ID, color="k", ls="--", lw=1.0, label=f"identif. ≈{TAU_F_ID}")
ax.plot(t[run], a[run, TAU_F], color="#17becf", label=r"$\hat\tau_f$")
ax.set_ylabel(r"$\hat\tau_f$ [s]"); ax.set_xlabel("t [s]"); ax.set_ylim(0, 0.05)
ax.set_title("(f) Constante de empuje  τ_f"); ax.legend(loc="best")

fig.suptitle("MHE sensor virtual (IMU): filtra estado + sensa fuerza + estima masa, τ_rc, τ_f",
             fontsize=12, y=1.00)
plt.tight_layout()
out = os.path.join(RES, "current_results.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"[OK] saved → {out}")

s = (t > 20) & (t < 55)
print(f"mass   = {np.mean(a[s,32]):.3f} kg (real 1.05)")
print(f"tau_rc = {np.mean(a[s,33]):.4f} s")
print(f"tau_f  = {np.mean(a[s,38]):.4f} s")
print(f"d      = [{np.mean(a[s,34]):.3f}, {np.mean(a[s,35]):.3f}, {np.mean(a[s,36]):.3f}] m/s²")
