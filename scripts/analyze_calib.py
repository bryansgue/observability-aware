#!/usr/bin/env python3
"""
analyze_calib.py — virtual force sensor calibration curve (fig_calib.png).

A proper sensor characterization: the estimated force (m*d_hat, in newtons) against
the ground-truth applied force, per axis, with a linear fit (slope, intercept, R^2).
An ideal force sensor has slope 1, intercept 0, R^2 = 1. Uses a sinusoidal-wind run
that sweeps the force over its range.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 6.5, "axes.labelsize": 6.5,
    "legend.fontsize": 7.5, "figure.dpi": 200, "savefig.bbox": "tight",
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
})
RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE = 1.05


def main():
    f = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "demo_sine.csv")
    if not os.path.isabs(f):
        f = os.path.join(RES, f)
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(","); c = {n: i for i, n in enumerate(h)}
    m = d[:, c["t"]] > 5.0
    axes = [("x", "dx_ekf", "theta", "#1f77b4"),
            ("y", "dy_ekf", "vtheta", "#2ca02c"),
            ("z", "dz_ekf", "a_theta", "#c0392b")]

    fig, axs = plt.subplots(1, 3, figsize=(3.6, 1.5))
    for ax, (lbl, de, gt, col) in zip(axs, axes):
        est = M_TRUE * d[m, c[de]]      # estimated force [N]
        g = d[m, c[gt]]                  # ground-truth force [N]
        a, b = np.polyfit(g, est, 1)     # est = a*g + b
        r2 = 1 - np.sum((est - (a*g+b))**2) / np.sum((est - est.mean())**2)
        lo, hi = min(g.min(), est.min()), max(g.max(), est.max())
        ax.plot([lo, hi], [lo, hi], "k:", lw=1.2, label="ideal (slope 1)")
        ax.scatter(g, est, s=4, alpha=0.25, color=col)
        xx = np.linspace(lo, hi, 50)
        ax.plot(xx, a*xx+b, color=col, lw=1.8, label=f"fit: slope {a:.2f}")
        ax.set_xlabel(f"$f_{{e,{lbl}}}$ [N]", labelpad=1)
        ax.set_ylabel(f"$m\\hat{{d}}_{lbl}$ [N]", labelpad=1)
        ax.set_title(["(a)","(b)","(c)"][["x","y","z"].index(lbl)] + f" slope {a:.2f}", fontsize=6.5, pad=2)
        from matplotlib.ticker import MaxNLocator
        ax.xaxis.set_major_locator(MaxNLocator(3)); ax.yaxis.set_major_locator(MaxNLocator(3)); ax.tick_params(labelsize=6, pad=1)
        print(f"  axis {lbl}: slope={a:.3f} intercept={b:+.3f} N  R2={r2:.4f}")
    fig.tight_layout()
    out = os.path.join(RES, "fig_calib.png")
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
