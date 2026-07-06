#!/usr/bin/env python3
"""
analyze_probe_sweep.py — information-optimal excitation analysis (fig_probe_opt.png).

For each probe shape (vertical/lateral mix at fixed displacement) computes the achieved
normalized identifiability sigma_tilde (= inverse-CRB on the mass) and the resulting mass
recovery error from a wrong 0.70 kg prior. Shows that the recovery error falls as the probe
raises sigma_tilde, and identifies the information-optimal mix (max sigma_tilde).
"""
import os, glob, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 11, "axes.labelsize": 12,
    "legend.fontsize": 9.5, "figure.dpi": 200, "lines.linewidth": 1.8,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--", "savefig.bbox": "tight",
})
RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE, WIN = 1.05, 100
ORDER = ["lateral", "mix30", "mix45", "mix60", "vert"]
NICE = {"lateral": "lateral", "mix30": "30°", "mix45": "45°", "mix60": "60° (fixed)", "vert": "vertical"}


def load(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    return d, {n: i for i, n in enumerate(h)}


def snorm_mean(d, c):
    """Mean normalized sigma_tilde over the probe window (t>8 s)."""
    T = d[:, c["T"]]; qw, qx, qy, qz = d[:, c["qw"]], d[:, c["qx"]], d[:, c["qy"]], d[:, c["qz"]]
    ax = 2*(qx*qz+qw*qy); ay = 2*(qy*qz-qw*qx); az = 1-2*(qx**2+qy**2)
    a = np.stack([ax, ay, az], 1); t = d[:, c["t"]]
    sn = []
    for k in range(WIN, len(T)):
        if t[k] < 8: continue
        Tk = T[k-WIN:k]; ak = a[k-WIN:k]; s2 = np.sum(Tk**2)
        if s2 > 0: sn.append((s2-(np.sum(Tk[:, None]*ak, 0)**2).sum()/WIN)/s2)
    return np.median(sn) if sn else np.nan


def main():
    rows = []
    for tag in ORDER:
        files = sorted(glob.glob(os.path.join(RES, f"probe_{tag}_seed*.csv")))
        if not files: continue
        sn, err = [], []
        for f in files:
            d, c = load(f)
            sn.append(snorm_mean(d, c))
            err.append(100*abs(d[-1, c["m_ekfg"]]-M_TRUE)/M_TRUE)
        rows.append((tag, np.nanmean(sn), np.nanmean(err), np.nanstd(err), len(files)))

    if not rows:
        print("No probe_*.csv found."); return
    print(f"{'probe':>10} {'sigma_t':>9} {'|err|%':>8} {'std':>6} {'N':>3}")
    for tag, s, e, es, n in rows:
        star = "  <-- info-optimal" if s == max(r[1] for r in rows) else ""
        print(f"{NICE[tag]:>10} {s:9.4f} {e:8.1f} {es:6.1f} {n:3d}{star}")

    tags = [r[0] for r in rows]; sig = np.array([r[1] for r in rows])
    err = np.array([r[2] for r in rows]); es = np.array([r[3] for r in rows])
    opt = int(np.argmax(sig))

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    cols = ["#1f77b4", "#2ca02c", "#9467bd", "#8c564b", "#e08214"]
    for i, tag in enumerate(tags):
        ax.errorbar(sig[i], err[i], yerr=es[i], fmt="o", ms=8, color=cols[i % len(cols)],
                    capsize=3, elinewidth=1.0, mec="white", mew=0.6, zorder=3,
                    label=NICE[tag])
    ax.scatter([sig[opt]], [err[opt]], s=210, facecolors="none",
               edgecolors="#c0392b", linewidths=2.0, zorder=4,
               label="family-optimal")
    ax.set_xlabel(r"achieved identifiability $\tilde\sigma$ (probe)")
    ax.set_ylabel("mass recovery error [%]")
    ax.legend(loc="upper center", ncol=3, framealpha=0.95, fontsize=9)
    out = os.path.join(RES, "fig_probe_opt.png")
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")
    print(f"\ninfo-optimal = {NICE[tags[opt]]}: sigma_t={sig[opt]:.4f}, err={err[opt]:.1f}% "
          f"| fixed (60°)={err[tags.index('mix60')] if 'mix60' in tags else '?':.1f}%")


if __name__ == "__main__":
    main()
