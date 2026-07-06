#!/usr/bin/env python3
"""
plot_sensitivity.py — gate robustness to the threshold (fig_sensitivity.png).

Reads transition runs (aggressive then hover, under wind) taken at a sweep of
gate thresholds sigma_min, saved as results/sweep_thr_<thr>.csv. For each
threshold it reports the gated mass during the aggressive phase (should be
identified, ~1.05) and during the hover phase (should be held, not drift). The
gate is robust over the wide range between the hover and maneuver values of
sigma, so the threshold is not a sensitive tuning knob.

Usage:  python3 plot_sensitivity.py [out.png]
"""
import os, sys, glob, re
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
M_TRUE = 1.05


def hover_vals(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    c = {n: i for i, n in enumerate(h)}
    t = d[:, c["t"]]
    m = (t >= 32) & (t < 38)   # protection (hover) phase
    return np.mean(d[m, c["m_ekfg"]]), np.mean(d[m, c["m_ekf"]])  # gated, plain


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "fig_sensitivity.png")
    # group reps by threshold (files sweep_thr_<thr>[_<rep>].csv)
    by_thr = {}
    for f in glob.glob(os.path.join(RES, "sweep_thr_*.csv")):
        m = re.search(r"sweep_thr_([\d.]+)(?:_\d+)?\.csv", os.path.basename(f))
        if m:
            by_thr.setdefault(float(m.group(1)), []).append(f)
    if not by_thr:
        print("No sweep_thr_*.csv found."); return

    thr = sorted(by_thr)
    gm, gs, pm = [], [], []
    for x in thr:
        g = [hover_vals(f) for f in by_thr[x]]
        gated = np.array([a for a, _ in g]); plain = np.array([b for _, b in g])
        gm.append(gated.mean()); gs.append(gated.std()); pm.append(plain.mean())
        print(f"  thr={x:6.0f}  gated={gated.mean():.3f}±{gated.std():.3f}  "
              f"plain={plain.mean():.3f}  N={len(g)}")
    thr = np.array(thr); gm = np.array(gm); gs = np.array(gs); pm = np.array(pm)

    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    ax.axhspan(M_TRUE - 0.02, M_TRUE + 0.02, color="0.85", alpha=0.6, zorder=0,
               label=r"$\pm2\%$ of true mass")
    ax.axhline(M_TRUE, color="k", ls=":", lw=1.4, label=f"true mass ({M_TRUE} kg)")
    ax.semilogx(thr, pm, marker="^", ms=6, color="#c0392b", ls="--",
                label="plain EKF (no gate)")
    ax.errorbar(thr, gm, yerr=gs, marker="s", ms=6, color="#117a65", ls="-",
                capsize=3, label="gated EKF (ours)")
    ax.set_xlabel(r"gate threshold $\sigma_{\min}$")
    ax.set_ylabel("hover mass estimate [kg]")
    ax.set_title("Gate robustness to the threshold")
    ax.legend(loc="best", framealpha=0.95, fontsize=8.5)
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
