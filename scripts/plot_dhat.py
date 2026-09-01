#!/usr/bin/env python3
"""
plot_dhat.py — virtual force sensor in action (fig_dhat.png).

Time-domain comparison of the EKF disturbance estimate against the simulator
ground truth, per axis, on a single wind run. Shows that the estimate tracks the
applied force, not just that it correlates with it.

Ground truth force is logged in the theta/vtheta/a_theta columns (world frame, N).
The EKF estimate dx_ekf/dy_ekf/dz_ekf is a specific force (m/s^2); it is scaled
by the true mass to compare in newtons.

Usage:  python3 plot_dhat.py [csv] [out.png]
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 11,
    "axes.titlesize": 12, "legend.fontsize": 9, "xtick.labelsize": 9,
    "ytick.labelsize": 9, "figure.dpi": 200, "lines.linewidth": 1.4,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE = 1.05


def main():
    csv = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "nmpc_mhe_v2_w1.5.csv")
    if not os.path.isabs(csv):
        csv = os.path.join(RES, csv)
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RES, "fig_dhat.png")

    d = np.loadtxt(csv, delimiter=",", skiprows=1)
    h = open(csv).readline().strip().split(",")
    c = {n: i for i, n in enumerate(h)}
    t = d[:, c["t"]]

    axes = [("x", "theta", "dx_ekf"), ("y", "vtheta", "dy_ekf"), ("z", "a_theta", "dz_ekf")]
    fig, axx = plt.subplots(3, 1, figsize=(7.0, 5.2), sharex=True)
    for ax, (lab, gt, est) in zip(axx, axes):
        ax.plot(t, d[:, c[gt]], color="k", lw=1.6, label="ground truth")
        ax.plot(t, M_TRUE * d[:, c[est]], color="#c0392b", ls="--",
                label=r"EKF $m\hat{d}$")
        r = np.corrcoef(d[t > 3, c[est]], d[t > 3, c[gt]])[0, 1]
        ax.set_ylabel(rf"$f_{{e,{lab}}}$ [N]")
        ax.text(0.99, 0.04, rf"correlation $= {r:.2f}$", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=8.5, color="0.3")
    # title omitted — info in caption
    axx[0].legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2,
                  frameon=False, borderaxespad=0)
    axx[-1].set_xlabel("time [s]")

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
