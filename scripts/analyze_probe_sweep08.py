#!/usr/bin/env python3
"""Probe shaping at the rho = 0.8 m budget (fig_probe_opt.png, Section VII-E).
Per shape (5 seeds): identifiability sigma_tilde achieved while the probe is active
(median over 2-12 s, Eq. snorm from logged T and attitude), time for the mass error
to settle below 2 %, and the error at the end of the 50-s run."""
import os, glob, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "analyze_sens.py")).read().split("def run_metrics")[0])
plt.rcParams.update({"font.size": 11, "axes.labelsize": 11, "xtick.labelsize": 10, "ytick.labelsize": 10, "legend.fontsize": 10, "lines.linewidth": 1.8})
M = 1.05
SHAPES = [("lateral", "lateral\n(0°)"), ("mix30", "30°"), ("mix45", "45°"), ("mix60", "60°"), ("vert", "vertical\n(90°)")]
res = {}
for tag, _ in SHAPES:
    sn, t2, ee = [], [], []
    for f in sorted(glob.glob(os.path.join(RES, f"probe08_{tag}_seed*.csv"))):
        d, c = load(f); t = d[:, c["t"]]; m = d[:, c["m_ekfg"]]; s = snorm(d, c)
        w = (t > 2) & (t < 12); sn.append(np.nanmedian(s[w]))
        err = np.abs(m - M)/M*100
        k = [i for i in np.where((err < 2.0) & (t > 1))[0] if np.all(err[i:] < 2.0)]
        t2.append(t[k[0]] if k else np.nan); ee.append(err[t > t[-1]-1].mean())
    res[tag] = (np.array(sn), np.array(t2), np.array(ee))
    print(f"{tag:8s} sigma_tilde {np.mean(sn):.4f}+-{np.std(sn):.4f}  t<2% {np.nanmean(t2):5.1f}+-{np.nanstd(t2):.1f} s  err end {np.mean(ee):.1f}+-{np.std(ee):.1f}%  N={len(sn)}")
fig, ax1 = plt.subplots(figsize=(7.0, 4.2))
x = np.arange(len(SHAPES)); labels = [l for _, l in SHAPES]
sn_m = [res[t][0].mean() for t, _ in SHAPES]; sn_s = [res[t][0].std() for t, _ in SHAPES]
ax1.bar(x, sn_m, yerr=sn_s, color="#9ecae1", edgecolor="#1f77b4", capsize=2, width=0.6, label=r"$\tilde\sigma$ during probe")
ax1.axhline(0.009, color="k", ls="--", lw=1); ax1.text(len(x)-0.6, 0.0105, r"$\tilde\sigma_{\min}$", fontsize=10, ha="right")
ax1.set_ylabel(r"$\tilde\sigma$ (probe)", color="#1f77b4"); ax1.set_ylim(0, 0.06)
ax1.set_xticks(x); ax1.set_xticklabels(labels, fontsize=10); ax1.set_xlabel("vertical/lateral mix at budget $\\rho=0.8$ m")
ax2 = ax1.twinx()
t2_m = [np.nanmean(res[t][1]) for t, _ in SHAPES]; t2_s = [np.nanstd(res[t][1]) for t, _ in SHAPES]
ax2.errorbar(x, t2_m, t2_s, fmt="o-", color="#c0392b", capsize=3, ms=6, lw=1.8, label="time to 2 % error")
ax2.set_ylabel("time to 2 % error [s]", color="#c0392b"); ax2.set_ylim(0, 18); ax2.grid(False)
h1, l1 = ax1.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
ax1.legend(h1+h2, l1+l2, loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False, fontsize=10)
fig.savefig(os.path.join(FIGS, "fig_probe_opt.png"), dpi=300, bbox_inches="tight")
print("[OK] fig_probe_opt.png")
