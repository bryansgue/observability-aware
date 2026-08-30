#!/usr/bin/env python3
"""
plot_trans.py — identify-then-protect transition figure (fig_trans.png).

Reads "trans" runs (aggressive first half -> hover second half). Shows the mass
estimate of the plain EKF vs. the identifiability-aware (gated) EKF over time, plus
the NORMALIZED identifiability sigma_tilde = sigma / sum(T^2) in [0,1]. During the
aggressive phase both identify the mass; in hover the plain EKF corrupts it
(vertical wind misread as a mass change, Prop. 1) while the gated EKF detects the
loss of identifiability and freezes it.

If several seeded runs (results/trans_seed*.csv) are present they are aggregated as
mean +/- std; otherwise the single demo file is used.

Usage:  python3 plot_trans.py [csv_or_glob] [out.png]
"""
import os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 12,
    "axes.titlesize": 13, "legend.fontsize": 9.5, "xtick.labelsize": 10,
    "ytick.labelsize": 10, "figure.dpi": 200, "lines.linewidth": 1.8,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE = 1.05
SNORM_THR = 0.009   # normalized identifiability gate threshold (dimensionless)
WIN = 100           # sigma window [samples] = 1 s at 100 Hz


def load(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    return d, {n: i for i, n in enumerate(h)}


def snorm_series(d, c):
    """Normalized identifiability sigma_tilde = sigma/sum(T^2) over a sliding window."""
    T = d[:, c["T"]]
    qw, qx, qy, qz = d[:, c["qw"]], d[:, c["qx"]], d[:, c["qy"]], d[:, c["qz"]]
    ax = 2*(qx*qz + qw*qy); ay = 2*(qy*qz - qw*qx); az = 1 - 2*(qx**2 + qy**2)
    a = np.stack([ax, ay, az], 1)
    n = len(T); sn = np.full(n, np.nan)
    for k in range(WIN, n):
        Tk = T[k-WIN:k]; ak = a[k-WIN:k]; s2 = np.sum(Tk**2)
        if s2 > 0:
            sn[k] = (s2 - (np.sum(Tk[:, None]*ak, 0)**2).sum()/WIN) / s2
    return sn


def collect(files):
    """Return common time grid and stacked [m_ekf, m_ekfg, m_hat, snorm] across runs."""
    runs = []
    for f in files:
        d, c = load(f)
        runs.append((d[:, c["t"]], d[:, c["m_ekf"]], d[:, c["m_ekfg"]],
                     d[:, c["m_hat"]], snorm_series(d, c)))
    n = min(len(r[0]) for r in runs)
    t = runs[0][0][:n]
    M = lambda j: np.stack([r[j][:n] for r in runs], 0)
    return t, M(1), M(2), M(3), M(4)


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    if arg and not os.path.isabs(arg):
        arg = os.path.join(RES, arg)
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RES, "fig_trans.png")

    seeds = sorted(glob.glob(os.path.join(RES, "trans_seed*.csv")))
    if arg and os.path.exists(arg):
        files = [arg]
    elif seeds:
        files = seeds
    else:
        files = [os.path.join(RES, "demo_trans.csv")]
    N = len(files)
    print(f"[trans] {N} run(s): {[os.path.basename(f) for f in files]}")

    t, m_ekf, m_ekfg, m_hat, sn = collect(files)
    speed_ref = None  # detect hover phase from the first run's gated mass settle

    # Detect the hover phase from the normalized sigma dropping and staying low.
    sn_mean = np.nanmean(sn, 0)
    lowt = (sn_mean < SNORM_THR) & (t > 2.0)
    # require sustained low (>0.5 s) to mark the hover start
    k = 50
    sust = np.convolve(lowt.astype(float), np.ones(k)/k, "same") > 0.8
    idx = np.where(sust)[0]
    t_switch = t[idx[0]] if len(idx) else 0.5*t[-1]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True,
                                   gridspec_kw={"height_ratios": [2.2, 1]})
    for ax in (ax1, ax2):
        ax.axvspan(t_switch, t[-1], color="0.85", alpha=0.5, zorder=0)

    def band(ax, t, M, color, ls, label):
        mu = np.nanmean(M, 0)
        ax.plot(t, mu, color=color, ls=ls, label=label)
        if M.shape[0] > 1:
            sd = np.nanstd(M, 0)
            ax.fill_between(t, mu-sd, mu+sd, color=color, alpha=0.15, zorder=1)

    ax1.axhline(M_TRUE, color="k", ls=":", lw=1.4, label=f"true mass ({M_TRUE} kg)")
    band(ax1, t, m_hat,  "#e08214", "-.", "MHE")
    band(ax1, t, m_ekf,  "#c0392b", "--", "plain EKF")
    band(ax1, t, m_ekfg, "#1f77b4", "-",  "identifiability-aware EKF (ours)")
    ax1.set_ylabel("mass estimate [kg]")
    # title omitted — description and N go in the LaTeX caption (IEEE convention)
    ax1.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), framealpha=0.95,
               ncol=4, fontsize=8.5, borderaxespad=0)
    ax1.text(t_switch*0.5, ax1.get_ylim()[1], "aggressive\n(identifiable)",
             ha="center", va="top", fontsize=9, color="0.35")
    ax1.text((t_switch + t[-1])*0.5, ax1.get_ylim()[1], "hover\n(unidentifiable)",
             ha="center", va="top", fontsize=9, color="0.35")

    band(ax2, t, sn, "#117a65", "-", None)
    ax2.set_yscale("log")
    ax2.axhline(SNORM_THR, color="k", ls="--", lw=1.0)
    ax2.text(t[-1], SNORM_THR, r"  $\tilde\sigma_{\min}$", va="center", ha="left",
             fontsize=9, color="0.3")
    ax2.set_ylabel(r"$\tilde\sigma$ (norm.)")
    ax2.set_xlabel("time [s]")
    ax2.set_ylim(1e-3, 1.0)

    # report the hover-phase plain-EKF drift for the caption/text
    hov = t > t_switch
    drift = 100.0*(np.nanmean(m_ekf[:, hov][:, -1]) - M_TRUE)/M_TRUE
    held = 100.0*(np.nanmean(m_ekfg[:, hov][:, -1]) - M_TRUE)/M_TRUE
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}  (hover ~{t_switch:.1f}s | plain EKF drift {drift:+.1f}% | "
          f"gated {held:+.1f}%)")


if __name__ == "__main__":
    main()
