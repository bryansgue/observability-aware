#!/usr/bin/env python3
"""
plot_mass_id.py — mass-identification accuracy vs. excitation (fig_mass_id.png).

Reuses the rejection battery (results/bat_*.csv). The mass is a free state in the
MHE in every run, so both feedforward and baseline runs are pooled per speed.
The figure shows the steady-state mass estimate (mean ± std) against the measured
peak speed, with the true mass as a reference line. The key observation is that
the estimate band SHRINKS with excitation: at hover the mass is weakly observable
(Proposition 1, mass-d_z ambiguity) so the estimate is loose, while an agile
flight excites the dynamics and sharpens it.

Usage:  python3 plot_mass_id.py [out.png]
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
M_TRUE = 1.05
T_SKIP = 3.0   # let the mass settle before measuring steady state


def cols(f):
    h = open(f).readline().strip().split(",")
    return {n: i for i, n in enumerate(h)}


def run_stats(f):
    """Steady-state mean mass (MHE, EKF) and peak speed for one run."""
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    c = cols(f)
    m = d[:, c["t"]] > T_SKIP
    mhat = np.mean(d[m, c["m_hat"]])
    mekf = np.mean(d[m, c["m_ekf"]]) if "m_ekf" in c else np.nan
    speed = np.max(np.sqrt(d[m, c["vx"]]**2 + d[m, c["vy"]]**2 + d[m, c["vz"]]**2))
    return mhat, mekf, speed


def discover_speeds():
    tags = set()
    for f in glob.glob(os.path.join(RES, "bat_*_noff_*.csv")):
        m = re.match(r"bat_(.+)_(?:ff|noff)_\d+\.csv", os.path.basename(f))
        if m:
            tags.add(m.group(1))
    return sorted(tags, key=lambda t: (0, 0.0) if t == "hover" else (1, float(t)))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "fig_mass_id.png")
    speeds = discover_speeds()
    if not speeds:
        print("No battery CSVs in results/."); return

    xpos, m_mean, m_std, n_used = [], [], [], []
    e_mean, e_std = [], []
    for tag in speeds:
        mh, ek, pk = [], [], []   # mh = EKF (primary), ek = MHE (baseline)
        for f in sorted(glob.glob(os.path.join(RES, f"bat_{tag}_*_*.csv"))):
            try:
                a, e, b = run_stats(f)   # a = MHE m_hat, e = EKF m_ekf
                mh.append(e); pk.append(b)
                if not np.isnan(a):
                    ek.append(a)
            except Exception as ex:
                print(f"  skip {os.path.basename(f)}: {ex}")
        if not mh:
            continue
        x = 0.0 if tag == "hover" else float(np.mean(pk))
        xpos.append(x); m_mean.append(np.mean(mh)); m_std.append(np.std(mh))
        e_mean.append(np.mean(ek) if ek else np.nan)
        e_std.append(np.std(ek) if ek else 0.0)
        n_used.append(len(mh))
        err = 100.0 * (np.mean(mh) - M_TRUE) / M_TRUE
        ek_s = f"  MHE={np.mean(ek):.3f}±{np.std(ek):.3f}" if ek else ""
        print(f"  {tag:>5}: peak={x:5.2f} m/s  EKF={np.mean(mh):.3f} ± "
              f"{np.std(mh):.3f} kg  ({err:+.1f}%){ek_s}  N={len(mh)}")

    xpos = np.array(xpos); m_mean = np.array(m_mean); m_std = np.array(m_std)
    e_mean = np.array(e_mean); e_std = np.array(e_std)
    has_ekf = np.any(~np.isnan(e_mean))
    N = int(np.median(n_used))

    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    ax.axhline(M_TRUE, color="k", ls=":", lw=1.4, label=f"true mass ({M_TRUE} kg)")
    # std band (light) + thin candle error bars
    ax.fill_between(xpos, m_mean - m_std, m_mean + m_std, color="#c0392b",
                    alpha=0.10, zorder=1)
    # Plot only the plain EKF (the baseline): biased in hover, accurate under
    # excitation. The MHE comparison lives in the operational figure (fig_trans)
    # and the summary table; its per-speed hover behavior differs by scenario, so
    # mixing it here would conflict with the transition figure.
    ax.errorbar(xpos, m_mean, yerr=m_std, marker="o", ms=5.5, color="#c0392b",
                ls="--", mec="white", mew=0.6, capsize=2.5, capthick=1.0,
                elinewidth=1.0, zorder=3, label=r"ungated EKF (observability probe)")
    for x, mm, ss in zip(xpos, m_mean, m_std):
        ax.annotate(rf"$\pm${ss*1000:.0f} g", xy=(x, mm + ss),
                    xytext=(0, 6), textcoords="offset points",
                    ha="center", va="bottom", fontsize=8.5, color="#117a65")

    ax.set_xlabel("peak speed [m/s]")
    ax.set_ylabel("mass estimate [kg]")
    # title omitted — info in caption
    ticks = list(xpos)
    labels = ["hover" if abs(t) < 1e-6 else f"{round(t)}" for t in ticks]
    ax.set_xticks(ticks); ax.set_xticklabels(labels)
    ax.legend(loc="lower right", framealpha=0.95)
    # N (and the N=9 hover note) stated in the LaTeX caption, not on canvas.

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
