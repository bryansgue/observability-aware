#!/usr/bin/env python3
"""
plot_force_corr.py — virtual-force-sensor accuracy vs. excitation (fig_force_corr.png).

Reuses the rejection battery (results/bat_*.csv). The ground-truth external force
is logged in the theta/vtheta/a_theta columns (world frame, N); the estimated
disturbance is dx_hat/dy_hat/dz_hat. For each run the per-axis Pearson
correlation between estimate and ground truth is computed over the steady-state
segment, then averaged per speed (feedforward and baseline pooled).

The figure shows the central observability result: the horizontal force is read
accurately at every speed, while the VERTICAL force is unobservable at hover
(mass-d_z ambiguity, Proposition 1) and becomes observable only as excitation
grows.

Usage:  python3 plot_force_corr.py [out.png]
"""
import os, sys, glob, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 12,
    "axes.titlesize": 13, "legend.fontsize": 10, "xtick.labelsize": 10,
    "ytick.labelsize": 10, "figure.dpi": 200, "lines.linewidth": 1.8,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
T_SKIP = 3.0
# (estimate column, ground-truth column) per axis — EKF estimate (paper method)
PAIRS = [("dx_ekf", "theta"), ("dy_ekf", "vtheta"), ("dz_ekf", "a_theta")]


def cols(f):
    h = open(f).readline().strip().split(",")
    return {n: i for i, n in enumerate(h)}


def run_corr(f):
    """Per-axis correlation and peak speed for one run (NaN if GT is flat)."""
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    c = cols(f)
    m = d[:, c["t"]] > T_SKIP
    out = []
    for est, gt in PAIRS:
        g = d[m, c[gt]]
        out.append(np.corrcoef(d[m, c[est]], g)[0, 1] if np.std(g) > 1e-6 else np.nan)
    speed = np.max(np.sqrt(d[m, c["vx"]]**2 + d[m, c["vy"]]**2 + d[m, c["vz"]]**2))
    return out, speed


def discover_speeds():
    tags = set()
    for f in glob.glob(os.path.join(RES, "bat_*_noff_*.csv")):
        mm = re.match(r"bat_(.+)_(?:ff|noff)_\d+\.csv", os.path.basename(f))
        if mm:
            tags.add(mm.group(1))
    return sorted(tags, key=lambda t: (0, 0.0) if t == "hover" else (1, float(t)))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "fig_force_corr.png")
    speeds = discover_speeds()
    if not speeds:
        print("No battery CSVs in results/."); return

    xpos = []
    mean = {0: [], 1: [], 2: []}
    std = {0: [], 1: [], 2: []}
    n_used = []
    for tag in speeds:
        per = {0: [], 1: [], 2: []}
        pk = []
        for f in sorted(glob.glob(os.path.join(RES, f"bat_{tag}_*_*.csv"))):
            try:
                cs, s = run_corr(f); pk.append(s)
                for k in range(3):
                    if not np.isnan(cs[k]):
                        per[k].append(cs[k])
            except Exception as ex:
                print(f"  skip {os.path.basename(f)}: {ex}")
        if not pk:
            continue
        xpos.append(0.0 if tag == "hover" else float(np.mean(pk)))
        n_used.append(len(pk))
        for k in range(3):
            a = np.array(per[k])
            mean[k].append(a.mean()); std[k].append(a.std())
        print(f"  {tag:>5}: peak={xpos[-1]:5.2f}  "
              f"x={mean[0][-1]:.3f}±{std[0][-1]:.3f}  "
              f"y={mean[1][-1]:.3f}±{std[1][-1]:.3f}  "
              f"z={mean[2][-1]:.3f}±{std[2][-1]:.3f}  N={len(pk)}")

    xpos = np.array(xpos)
    N = int(np.median(n_used))

    fig, ax = plt.subplots(figsize=(7.0, 3.1))
    style = [("$d_x$ (horizontal)", "#1f77b4", "o", "-"),
             ("$d_y$ (horizontal)", "#2ca02c", "s", "-"),
             ("$d_z$ (vertical)",   "#c0392b", "^", "-")]
    # small x-offset per axis so overlapping candles stay readable
    dx = (xpos.max() - xpos.min()) * 0.012 if len(xpos) > 1 else 0.1
    off = [-dx, 0.0, dx]
    for k, (lab, col, mk, ls) in enumerate(style):
        ax.errorbar(xpos + off[k], mean[k], yerr=std[k], marker=mk, ms=5.5,
                    color=col, ls=ls, mec="white", mew=0.6, capsize=2.5,
                    capthick=1.0, elinewidth=1.0, label=lab, zorder=3)

    ax.set_xlabel("peak speed [m/s]")
    ax.set_ylabel(r"$\rho(\hat{d}_i,\, f_{e,i})$")
    # title omitted — info goes in the LaTeX caption (IEEE convention)
    ax.set_ylim(0.88, 1.0)
    ticks = list(xpos)
    labels = ["hover" if abs(t) < 1e-6 else f"{round(t)}" for t in ticks]
    ax.set_xticks(ticks); ax.set_xticklabels(labels)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.01), ncol=3, frameon=False, columnspacing=1.2, handletextpad=0.5)
    # N and method named in the LaTeX caption (IEEE convention) — no in-axes metadata

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
