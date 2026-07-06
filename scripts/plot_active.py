#!/usr/bin/env python3
"""
plot_active.py — active observability-aware control demo (fig_active.png).

Hover runs from a wrong mass prior (0.70 kg, true 1.05):
  - passive (no probe): mass unobservable in hover -> frozen at the wrong prior.
  - active (probe):      controller injects excitation -> mass becomes observable
                         -> identified -> excitation stops -> estimate protected.

Aggregates seeded runs results/active_{noprobe,probe}_seed*.csv as mean +/- std if
present, else falls back to the single demo files. The bottom panel shows the
NORMALIZED identifiability sigma_tilde = sigma/sum(T^2) of the probe run against
the single dimensionless threshold sigma_tilde_min.

Usage:  python3 plot_active.py [out.png]
"""
import os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 12,
    "axes.titlesize": 13, "legend.fontsize": 9.5, "xtick.labelsize": 10,
    "ytick.labelsize": 10, "figure.dpi": 200, "lines.linewidth": 1.9,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE, M0, SNORM_THR, WIN = 1.05, 0.70, 0.009, 100


def load(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    return d, {n: i for i, n in enumerate(h)}


def snorm_series(d, c):
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


def collect(files, want_snorm=False):
    runs = []
    for f in files:
        d, c = load(f)
        sn = snorm_series(d, c) if want_snorm else None
        runs.append((d[:, c["t"]], d[:, c["m_ekfg"]], sn))
    n = min(len(r[0]) for r in runs)
    t = runs[0][0][:n]
    M = np.stack([r[1][:n] for r in runs], 0)
    S = np.stack([r[2][:n] for r in runs], 0) if want_snorm else None
    return t, M, S


def pick(tag, demo):
    seeds = sorted(glob.glob(os.path.join(RES, f"active_{tag}_seed*.csv")))
    return seeds if seeds else [os.path.join(RES, demo)]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "fig_active.png")
    fn = pick("noprobe", "demo_noprobe.csv")
    fp = pick("probe", "demo_probe.csv")
    N = min(len(fn), len(fp))
    print(f"[active] passive {len(fn)} run(s), probe {len(fp)} run(s)")

    tn, mn, _ = collect(fn)
    tp, mp, sp = collect(fp, want_snorm=True)

    sp_mean = np.nanmean(sp, 0)
    hi = np.convolve((sp_mean > SNORM_THR).astype(float), np.ones(20)/20, "same") > 0.5
    idx = np.where(hi & (tp > 1))[0]
    p0, p1 = (tp[idx[0]], tp[idx[-1]]) if len(idx) else (0, 0)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True,
                                   gridspec_kw={"height_ratios": [2.2, 1]})
    for ax in (ax1, ax2):
        ax.axvspan(p0, p1, color="#f9e79f", alpha=0.55, zorder=0)
    ax1.text((p0+p1)/2, M_TRUE, "active probe", ha="center", va="bottom",
             fontsize=9, color="#b9770e")

    def band(ax, t, M, color, ls, label):
        mu = np.nanmean(M, 0)
        ax.plot(t, mu, color=color, ls=ls, label=label)
        if M.shape[0] > 1:
            sd = np.nanstd(M, 0)
            ax.fill_between(t, mu-sd, mu+sd, color=color, alpha=0.15, zorder=1)

    ax1.axhline(M_TRUE, color="k", ls=":", lw=1.4, label=f"true mass ({M_TRUE})")
    ax1.axhline(M0, color="0.6", ls=":", lw=1.0)
    band(ax1, tn, mn, "#c0392b", "--", "passive (no probe)")
    band(ax1, tp, mp, "#1f77b4", "-",  "active (probe)")
    ax1.set_ylabel("mass estimate [kg]")
    # title omitted — description and N go in the LaTeX caption (IEEE convention)
    ax1.set_ylim(0.6, 1.15)
    ax1.legend(loc="center right", framealpha=0.95)

    band(ax2, tp, sp, "#117a65", "-", None)
    ax2.set_yscale("log")
    ax2.axhline(SNORM_THR, color="k", ls="--", lw=1.0)
    ax2.text(tp[-1], SNORM_THR, r"  $\tilde\sigma_{\min}$", va="center", ha="left",
             fontsize=9, color="0.3")
    ax2.set_ylabel(r"$\tilde\sigma$ (probe run)")
    ax2.set_xlabel("time [s]")
    ax2.set_ylim(1e-3, 1.0)

    err_p = 100.0*(np.nanmean(mp[:, -1]) - M_TRUE)/M_TRUE
    err_n = 100.0*(np.nanmean(mn[:, -1]) - M_TRUE)/M_TRUE
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}  (probe ~{p0:.1f}-{p1:.1f}s | active err {err_p:+.1f}% | "
          f"passive err {err_n:+.1f}%)")


if __name__ == "__main__":
    main()
