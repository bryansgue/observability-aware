#!/usr/bin/env python3
"""
plot_massconv.py — clean mass identification (fig_massconv.png).

A single disturbance-free trajectory from a deliberately wrong mass prior
(0.60 kg, 43% low). Under the excitation of the trajectory the EKF mass converges
to the ground truth, with the external force correctly near zero. This is the
identification result: given excitation and no confounding disturbance, the mass
is recovered accurately.

Reads results/demo_massid.csv. Usage: python3 plot_massconv.py [csv] [out.png]
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 12,
    "axes.titlesize": 13, "legend.fontsize": 10, "xtick.labelsize": 10,
    "ytick.labelsize": 10, "figure.dpi": 200, "lines.linewidth": 1.9,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE, M0 = 1.05, 0.60


def main():
    csv = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "demo_massid.csv")
    if not os.path.isabs(csv):
        csv = os.path.join(RES, csv)
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RES, "fig_massconv.png")

    d = np.loadtxt(csv, delimiter=",", skiprows=1)
    h = open(csv).readline().strip().split(",")
    c = {n: i for i, n in enumerate(h)}
    t = d[:, c["t"]]

    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    ax.axhline(M_TRUE, color="k", ls=":", lw=1.4, label=f"true mass ({M_TRUE} kg)")
    ax.plot(t, d[:, c["m_ekf"]], color="#1f77b4", ls="-",
            label="EKF mass estimate")
    ax.scatter([0], [M0], color="#c0392b", zorder=5, s=30, label=f"wrong prior ({M0} kg)")
    mf = np.mean(d[t > 18, c["m_ekf"]])   # converged value (reported in the caption)
    # Result values (converged mass, errors) stated in the LaTeX caption, not on canvas.
    ax.set_xlabel("time [s]")
    ax.set_ylabel("mass estimate [kg]")
    # title omitted — info in caption
    ax.set_ylim(0.55, 1.12)
    ax.legend(loc="lower right", framealpha=0.95)

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}  (final {mf:.3f} kg)")


if __name__ == "__main__":
    main()
