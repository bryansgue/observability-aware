#!/usr/bin/env python3
"""
plot_convergence.py — time-domain convergence of the three estimates on the
agile run, from a deliberately-wrong initial guess to the true values.

Reads results/nmpc_mhe_v2_w3.0.csv (agile, ~12 m/s) and produces
results/convergence.png : m̂(t), τ̂_rc(t), τ̂_f(t) over the RUN phase.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 10, "axes.labelsize": 10,
    "axes.titlesize": 11, "legend.fontsize": 9, "figure.dpi": 150,
    "lines.linewidth": 1.6, "axes.grid": True, "grid.alpha": 0.3,
})

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RES_DIR    = os.path.join(SCRIPT_DIR, "..", "results")

COL_T, COL_M, COL_TAU, COL_TAUF = 0, 32, 33, 38
TRUE_M, TRUE_TAU, TRUE_TAUF = 1.08, 0.056, 0.009
INIT_M, INIT_TAU, INIT_TAUF = 0.60, 0.030, 0.10

csv = os.path.join(RES_DIR, "nmpc_mhe_v2_w3.0.csv")
a = np.loadtxt(csv, delimiter=",", skiprows=1)
t = a[:, COL_T]
run = t < 58.0   # drop the post-trajectory PD landing

fig, axes = plt.subplots(3, 1, figsize=(6.0, 6.2), sharex=True)

ax = axes[0]
ax.axhline(TRUE_M, color="k", ls="--", lw=1.0, label=fr"true $m={TRUE_M}$")
ax.plot(t[run], a[run, COL_M], color="#1f77b4", label=r"$\hat m$")
ax.scatter([0], [INIT_M], color="#d62728", zorder=5, s=30,
           label=fr"init $={INIT_M}$ ($-44\%$)")
ax.set_ylabel(r"$\hat m$ [kg]"); ax.set_ylim(0.5, 1.2)
ax.legend(loc="lower right", fontsize=8)
ax.set_title("Convergence from wrong initial guess (agile run, $v_{peak}\\approx 12$ m/s)")

ax = axes[1]
ax.axhline(TRUE_TAU, color="k", ls="--", lw=1.0, label=fr"true $\tau_{{rc}}\approx {TRUE_TAU}$")
ax.plot(t[run], a[run, COL_TAU], color="#ff7f0e", label=r"$\hat\tau_{rc}$")
ax.set_ylabel(r"$\hat\tau_{rc}$ [s]"); ax.set_ylim(0.02, 0.12)
ax.legend(loc="upper right", fontsize=8)

ax = axes[2]
ax.axhline(TRUE_TAUF, color="k", ls="--", lw=1.0, label=fr"true $\tau_f\approx {TRUE_TAUF}$")
ax.plot(t[run], a[run, COL_TAUF], color="#2ca02c", label=r"$\hat\tau_f$")
ax.set_ylabel(r"$\hat\tau_f$ [s]"); ax.set_ylim(0.0, 0.12)
ax.set_xlabel("t [s]")
ax.legend(loc="upper right", fontsize=8)

plt.tight_layout()
out = os.path.join(RES_DIR, "convergence.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"[OK] saved → {out}")
