#!/usr/bin/env python3
"""
plot_traj.py — trajectory tracking under wind, with and without the force
feedforward (fig_traj.png). Two stacked orthogonal projections (XY top, XZ
bottom) sharing the common x axis.

With the estimated-force feedforward the vehicle stays on the reference; without
it the wind pushes it off the path. Reads two runs under the same wind:
  bat_<w>_ff_1.csv   (feedforward on)
  bat_<w>_noff_1.csv (feedforward off)

Usage:  python3 plot_traj.py [w]      (default w=1.0)
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 10, "axes.labelsize": 11,
    "axes.titlesize": 11, "legend.fontsize": 9, "xtick.labelsize": 9,
    "ytick.labelsize": 9, "figure.dpi": 200, "lines.linewidth": 1.6,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
C_REF, C_FF, C_NO = "k", "#1f77b4", "#c0392b"


def load(name):
    f = os.path.join(RES, name)
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    c = {n: i for i, n in enumerate(h)}
    m = d[:, c["t"]] > 1.0
    return lambda k: d[m, c[k]]


def main():
    w = sys.argv[1] if len(sys.argv) > 1 else "1.0"
    out = os.path.join(RES, "fig_traj.png")
    ff = load(f"bat_{w}_ff_1.csv")
    no = load(f"bat_{w}_noff_1.csv")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.4, 4.6), sharex=True)

    # XY (top view)
    ax1.plot(ff("px_ref"), ff("py_ref"), C_REF, ls=":", lw=1.3, label="reference")
    ax1.plot(no("px"), no("py"), C_NO, ls="--", label="no feedforward")
    ax1.plot(ff("px"), ff("py"), C_FF, ls="-", label="with feedforward")
    ax1.set_ylabel("y [m]")   # XY panel — identified by axis labels + caption

    # XZ (side view)
    ax2.plot(ff("px_ref"), ff("pz_ref"), C_REF, ls=":", lw=1.3)
    ax2.plot(no("px"), no("pz"), C_NO, ls="--")
    ax2.plot(ff("px"), ff("pz"), C_FF, ls="-")
    ax2.set_ylabel("z [m]"); ax2.set_xlabel("x [m]")   # XZ panel

    ax1.legend(loc="upper center", ncol=3, framealpha=0.95,
               bbox_to_anchor=(0.5, 1.32))
    fig.tight_layout()
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
