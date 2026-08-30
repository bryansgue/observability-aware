#!/usr/bin/env python3
"""
plot_rejection.py — disturbance-rejection-vs-speed figure (fig_rejection.png).

Reads the rejection battery produced by batch_rejection.sh:
    results/bat_<speed>_<ff|noff>_<rep>.csv
For each speed it averages the per-rep tracking RMSE for the two configurations
(baseline = no feedforward, with feedforward d̂) and plots mean ± std, N reps.
The x-axis is the MEASURED peak speed (read from the CSVs, not the filename), so
the points sit at the physically true speed. The rejection % annotation is
(RMSE_baseline − RMSE_ff) / RMSE_baseline.

Usage:
    python3 plot_rejection.py [out.png]
Default out: results/fig_rejection.png
"""
import os, sys, glob, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "figure.dpi": 200,
    "lines.linewidth": 1.8,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
T, PX, PY, PZ, VX, VY, VZ = 0, 1, 2, 3, 4, 5, 6
PXr, PYr, PZr = 21, 22, 23          # position reference columns
T_SKIP = 2.0                         # drop startup transient


def rmse_and_peak(csv):
    """Tracking-position RMSE [cm] and peak speed [m/s] for one run."""
    d = np.loadtxt(csv, delimiter=",", skiprows=1)
    m = d[:, T] > T_SKIP
    ex = d[m, PX] - d[m, PXr]
    ey = d[m, PY] - d[m, PYr]
    ez = d[m, PZ] - d[m, PZr]
    e = np.sqrt(ex**2 + ey**2 + ez**2)
    rmse_cm = np.sqrt(np.mean(e**2)) * 100.0
    speed = np.sqrt(d[m, VX]**2 + d[m, VY]**2 + d[m, VZ]**2)
    return rmse_cm, np.max(speed)


def collect(speed_tag, mode):
    """All reps for one (speed, mode): returns (rmse_list, peak_list)."""
    pat = os.path.join(RES, f"bat_{speed_tag}_{mode}_*.csv")
    rm, pk = [], []
    for f in sorted(glob.glob(pat)):
        try:
            r, p = rmse_and_peak(f)
            rm.append(r); pk.append(p)
        except Exception as ex:
            print(f"  skip {os.path.basename(f)}: {ex}")
    return np.array(rm), np.array(pk)


def discover_speeds():
    """Speed tags present in the battery, ordered hover-first then numeric."""
    tags = set()
    for f in glob.glob(os.path.join(RES, "bat_*_noff_*.csv")):
        m = re.match(r"bat_(.+)_(?:ff|noff)_\d+\.csv", os.path.basename(f))
        if m:
            tags.add(m.group(1))
    def key(t):
        return (0, 0.0) if t == "hover" else (1, float(t))
    return sorted(tags, key=key)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "fig_rejection.png")
    speeds = discover_speeds()
    if not speeds:
        print("No battery CSVs found in results/."); return

    xpos, ff_m, ff_s, nf_m, nf_s, rej, n_used = [], [], [], [], [], [], []
    for tag in speeds:
        ff_r, ff_p = collect(tag, "ff")
        nf_r, nf_p = collect(tag, "noff")
        if len(ff_r) == 0 or len(nf_r) == 0:
            print(f"  {tag}: missing data (ff={len(ff_r)} noff={len(nf_r)}) — skipped")
            continue
        # x = measured peak speed (0 for hover)
        x = 0.0 if tag == "hover" else float(np.mean(np.concatenate([ff_p, nf_p])))
        xpos.append(x)
        ff_m.append(np.mean(ff_r)); ff_s.append(np.std(ff_r))
        nf_m.append(np.mean(nf_r)); nf_s.append(np.std(nf_r))
        rej.append(100.0 * (np.mean(nf_r) - np.mean(ff_r)) / np.mean(nf_r))
        n_used.append(min(len(ff_r), len(nf_r)))
        print(f"  {tag:>5}: peak={x:5.2f} m/s  base={nf_m[-1]:5.1f}±{nf_s[-1]:.1f}  "
              f"ff={ff_m[-1]:5.1f}±{ff_s[-1]:.1f} cm  →  {rej[-1]:+.0f}%  (N={n_used[-1]})")

    xpos = np.array(xpos)
    ff_m, ff_s = np.array(ff_m), np.array(ff_s)
    nf_m, nf_s = np.array(nf_m), np.array(nf_s)
    N = int(np.median(n_used))

    fig, ax = plt.subplots(figsize=(7.0, 3.1))
    C_BASE, C_FF = "#c0392b", "#1f77b4"

    ax.fill_between(xpos, ff_m, nf_m, color=C_FF, alpha=0.08, zorder=1)
    bar = dict(capsize=2.5, capthick=1.0, elinewidth=1.0)
    ax.errorbar(xpos, nf_m, yerr=nf_s, marker="s", ms=5.5, color=C_BASE,
                ls="--", mec="white", mew=0.6, label="baseline (no feedforward)",
                zorder=3, **bar)
    ax.errorbar(xpos, ff_m, yerr=ff_s, marker="o", ms=5.5, color=C_FF,
                ls="-", mec="white", mew=0.6, label=r"with feedforward ($\hat{d}$)",
                zorder=3, **bar)

    # error reduction between the two curves (a reduction, so no leading '+')
    for x, yf, yn, r in zip(xpos, ff_m, nf_m, rej):
        ax.annotate(f"{r:.0f}%", xy=(x, 0.5 * (yf + yn)),
                    xytext=(0, 0), textcoords="offset points",
                    ha="center", va="center", fontsize=9.5,
                    color="#117a65", fontweight="bold")

    ax.set_xlabel("peak speed [m/s]")
    ax.set_ylabel("tracking RMSE [cm]")   # N stated in the caption
    ax.set_ylim(bottom=0)

    # nice x ticks: hover label at 0, integers elsewhere
    ticks = list(xpos)
    labels = ["hover" if abs(t) < 1e-6 else f"{round(t)}" for t in ticks]
    ax.set_xticks(ticks); ax.set_xticklabels(labels)

    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2,
              framealpha=0.95, borderaxespad=0)
    # explanatory text (rejection shrinks with speed) is in the LaTeX caption

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
