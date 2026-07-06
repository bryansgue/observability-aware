#!/usr/bin/env python3
"""
plot_regimes.py — 3-regime convergence overlay (hover / slow / agile)

Inputs: results/{hover_v2,slow_v2,nmpc_mhe_v2}.csv
Outputs:
  results/regime_convergence.png   — 3-panel overlay (m, tau_rc, tau_f vs t)
  results/regime_summary.txt       — table of final converged values
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 10,
    "axes.labelsize": 10,
    "axes.titlesize": 11,
    "legend.fontsize": 9,
    "figure.dpi": 150,
    "lines.linewidth": 1.5,
    "axes.grid": True,
    "grid.alpha": 0.3,
})

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.join(SCRIPT_DIR, "..")
RES_DIR    = os.path.join(BASE_DIR, "results")

# CSV col indices (0-based): see paper.tex sec II
COL_T      = 0
COL_M_HAT  = 32   # m_hat
COL_TAU    = 33   # tau_rc = tau_hat
COL_TAUF   = 38   # tau_f in W_s column (nmpc_mhe_sil v2)

REGIMES = [
    ("hover",  "hover_v2.csv",     "0 m/s (hover)"),
    ("slow",   "slow_v2.csv",      "~3 m/s (slow Lissajous)"),
    ("agile",  "nmpc_mhe_v2.csv",  "~12 m/s (agile Lissajous)"),
]

COLORS = {"hover": "#d62728", "slow": "#ff7f0e", "agile": "#1f77b4"}

# ── Load ──────────────────────────────────────────────────────────────────────
data = {}
for key, fname, _label in REGIMES:
    path = os.path.join(RES_DIR, fname)
    arr  = np.loadtxt(path, delimiter=",", skiprows=1)
    data[key] = arr

# ── Plot 3-panel overlay ──────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(6.0, 6.5), sharex=True)

# Panel 1: mass
ax = axes[0]
ax.axhline(1.05, color="k", linestyle="--", linewidth=1.0,
           label=r"true $m=1.05$")
for key, _f, label in REGIMES:
    a = data[key]
    ax.plot(a[:, COL_T], a[:, COL_M_HAT], color=COLORS[key], label=label)
ax.set_ylabel(r"$\hat m$ [kg]")
ax.set_ylim(0.4, 1.7)
ax.legend(loc="lower right", ncol=2, fontsize=8)
ax.set_title("Parameter estimation across regimes (decoupled multi-rate)")

# Panel 2: tau_rc
ax = axes[1]
ax.axhline(0.056, color="k", linestyle="--", linewidth=1.0,
           label=r"true $\tau_{rc}\approx 0.056$")
for key, _f, label in REGIMES:
    a = data[key]
    ax.plot(a[:, COL_T], a[:, COL_TAU], color=COLORS[key])
ax.set_ylabel(r"$\hat\tau_{rc}$ [s]")
ax.set_ylim(0.02, 0.08)
ax.legend(loc="lower right", fontsize=8)

# Panel 3: tau_f (only agile has meaningful data)
ax = axes[2]
ax.axhline(0.009, color="k", linestyle="--", linewidth=1.0,
           label=r"true $\tau_f\approx 0.009$")
for key, _f, label in REGIMES:
    a = data[key]
    y = a[:, COL_TAUF]
    if np.nanmax(np.abs(y)) < 1e-6:
        continue   # skip flat/zero traces (hover/slow had RLS-thrust disabled)
    ax.plot(a[:, COL_T], y, color=COLORS[key])
ax.set_ylabel(r"$\hat\tau_f$ [s]")
ax.set_ylim(0.0, 0.025)
ax.set_xlabel("t [s]")
ax.legend(loc="upper right", fontsize=8)

plt.tight_layout()
out_fig = os.path.join(RES_DIR, "regime_convergence.png")
plt.savefig(out_fig, dpi=150, bbox_inches="tight")
print(f"[OK] saved figure → {out_fig}")

# ── Final values table ────────────────────────────────────────────────────────
def final_window_mean(arr, col, window_frac=0.5):
    n = len(arr)
    return float(np.nanmean(arr[n // 2:, col]))

lines = []
lines.append(f"{'Regime':<25} {'m_hat':>8} {'tau_rc':>10} {'tau_f':>10}")
lines.append("-" * 60)
for key, _f, label in REGIMES:
    a = data[key]
    m  = final_window_mean(a, COL_M_HAT)
    tr = final_window_mean(a, COL_TAU)
    tf = final_window_mean(a, COL_TAUF)
    lines.append(f"{label:<25} {m:>8.3f} {tr:>10.4f} {tf:>10.4f}")
out_txt = os.path.join(RES_DIR, "regime_summary.txt")
with open(out_txt, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"[OK] saved summary → {out_txt}")
print("\n".join(lines))
