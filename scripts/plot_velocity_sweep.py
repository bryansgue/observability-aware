#!/usr/bin/env python3
"""
plot_velocity_sweep.py — Pattern of converged estimates vs peak speed.

Reads results/nmpc_mhe_v2_wX.X.csv for w in {0.5, 1.0, 1.5, 2.0, 2.5, 3.0}
and produces results/velocity_sweep.png with three subplots vs v_peak.

For each run, peak speed is computed from the actual velocity column
(vx, vy, vz → ||v||_max).
"""
import os
import re
import glob
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
    "lines.linewidth": 1.4,
    "axes.grid": True,
    "grid.alpha": 0.3,
})

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.join(SCRIPT_DIR, "..")
RES_DIR    = os.path.join(BASE_DIR, "results")

COL_T, COL_VX, COL_VY, COL_VZ = 0, 4, 5, 6
COL_M_HAT, COL_TAU, COL_TAUF = 32, 33, 38

TRUE_M    = 1.08
TRUE_TAU  = 0.056
TRUE_TAUF = 0.009

# ── Discover runs ─────────────────────────────────────────────────────────────
pattern = re.compile(r"nmpc_mhe_v2_w([\d.]+)\.csv$")
runs = []
for path in sorted(glob.glob(os.path.join(RES_DIR, "nmpc_mhe_v2_w*.csv"))):
    m = pattern.search(path)
    if not m:
        continue
    w = float(m.group(1))
    arr = np.loadtxt(path, delimiter=",", skiprows=1)
    if len(arr) < 200:
        print(f"[skip] {path} too short ({len(arr)} rows)")
        continue
    # Use the steady RUN phase only: skip the convergence transient (t<20 s) and
    # the post-trajectory PD landing (t>55 s), where near-hover makes the params
    # unobservable and contaminates the converged value.
    t = arr[:, COL_T]
    run = (t > 20.0) & (t < 55.0)
    if run.sum() < 100:
        print(f"[skip] {path} crashed/too short (only {run.sum()} RUN-phase samples)")
        continue
    # peak speed over the run
    v_norm = np.sqrt(arr[:, COL_VX]**2 + arr[:, COL_VY]**2 + arr[:, COL_VZ]**2)
    v_peak = float(np.percentile(v_norm[run], 95))
    # converged values: mean over the steady RUN window
    m_hat = float(np.nanmean(arr[run, COL_M_HAT]))
    tau_hat = float(np.nanmean(arr[run, COL_TAU]))
    tauf_hat = float(np.nanmean(arr[run, COL_TAUF]))
    runs.append({
        "w": w, "v_peak": v_peak,
        "m_hat": m_hat, "tau_hat": tau_hat, "tauf_hat": tauf_hat,
    })
    print(f"w={w:.1f}  v_peak={v_peak:5.2f}  m̂={m_hat:.3f}  τ̂_rc={tau_hat:.4f}  τ̂_f={tauf_hat:.4f}")

if not runs:
    print("[ERR] no nmpc_mhe_v2_w*.csv files found")
    raise SystemExit(1)

runs.sort(key=lambda r: r["v_peak"])
v_arr   = np.array([r["v_peak"]   for r in runs])
m_arr   = np.array([r["m_hat"]    for r in runs])
t_arr   = np.array([r["tau_hat"]  for r in runs])
tf_arr  = np.array([r["tauf_hat"] for r in runs])

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(6.0, 6.5), sharex=True)

ax = axes[0]
ax.axhline(TRUE_M, color="k", linestyle="--", linewidth=1.0,
           label=fr"true $m={TRUE_M}$")
ax.plot(v_arr, m_arr, "o-", color="#1f77b4", markersize=7,
        label=r"$\hat m$")
ax.set_ylabel(r"$\hat m$ [kg]")
ax.legend(loc="best", fontsize=8)
ax.set_title("Identification pattern vs peak speed")

ax = axes[1]
ax.axhline(TRUE_TAU, color="k", linestyle="--", linewidth=1.0,
           label=fr"true $\tau_{{rc}}\approx {TRUE_TAU}$")
ax.plot(v_arr, t_arr, "o-", color="#ff7f0e", markersize=7,
        label=r"$\hat\tau_{rc}$")
ax.set_ylabel(r"$\hat\tau_{rc}$ [s]")
ax.legend(loc="best", fontsize=8)

ax = axes[2]
ax.axhline(TRUE_TAUF, color="k", linestyle="--", linewidth=1.0,
           label=fr"true $\tau_f\approx {TRUE_TAUF}$")
# show only points where the RLS-thrust actually captured something
mask = tf_arr > 1e-4
ax.plot(v_arr[mask], tf_arr[mask], "o-", color="#2ca02c", markersize=7,
        label=r"$\hat\tau_f$")
ax.set_ylabel(r"$\hat\tau_f$ [s]")
ax.set_xlabel(r"peak speed [m/s]")
ax.legend(loc="best", fontsize=8)

plt.tight_layout()
out_fig = os.path.join(RES_DIR, "velocity_sweep.png")
plt.savefig(out_fig, dpi=150, bbox_inches="tight")
print(f"[OK] saved figure → {out_fig}")

# ── Save numeric table ────────────────────────────────────────────────────────
out_txt = os.path.join(RES_DIR, "velocity_sweep.txt")
with open(out_txt, "w") as f:
    f.write(f"{'w':>5} {'v_peak':>8} {'m_hat':>8} {'tau_rc':>10} {'tau_f':>10}\n")
    for r in runs:
        f.write(f"{r['w']:>5.1f} {r['v_peak']:>8.2f} "
                f"{r['m_hat']:>8.3f} {r['tau_hat']:>10.4f} "
                f"{r['tauf_hat']:>10.4f}\n")
print(f"[OK] saved table  → {out_txt}")
