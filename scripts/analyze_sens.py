#!/usr/bin/env python3
"""
analyze_sens.py — gate sensitivity study (Reviewer 1, concern 4): fig_sens.png.

Reads results/sens_<cfg>_seed<k>.csv from batch_sens.sh plus the baseline
trans_seed<k>.csv, and reports for each configuration the mass error at the end
of the windy hover for the gated (ours) and the plain EKF, mean +- std over seeds,
and the fraction of hover time during which the gate was open.
"""
import os, sys, glob, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 8, "axes.labelsize": 8,
    "axes.titlesize": 8, "legend.fontsize": 7.5, "xtick.labelsize": 7,
    "ytick.labelsize": 7, "figure.dpi": 200, "lines.linewidth": 1.1,
    "lines.markersize": 4, "axes.grid": True, "grid.alpha": 0.25,
    "grid.linestyle": "--", "savefig.bbox": "tight",
})
HERE = os.path.dirname(os.path.abspath(__file__))
RES  = os.path.join(HERE, "..", "results")
FIGS = os.path.join(HERE, "..", "paper", "figs")
M_TRUE, THR, WIN = 1.05, 0.009, 100

def load(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    return d, {n: i for i, n in enumerate(h)}

def snorm(d, c, win=WIN):
    T = d[:, c["T"]]
    qw, qx, qy, qz = (d[:, c[k]] for k in ("qw", "qx", "qy", "qz"))
    a = np.stack([2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx**2 + qy**2)], 1)
    n = len(T); sn = np.full(n, np.nan)
    for k in range(win, n):
        Tk, ak = T[k-win:k], a[k-win:k]; s2 = np.sum(Tk**2)
        if s2 > 0: sn[k] = (s2 - (np.sum(Tk[:, None]*ak, 0)**2).sum()/win)/s2
    return sn

def run_metrics(f, thr=THR):
    d, c = load(f); t = d[:, c["t"]]
    sn = snorm(d, c)
    low = (sn < thr) & (t > 2.0)
    sust = np.convolve(low.astype(float), np.ones(50)/50, "same") > 0.8
    idx = np.where(sust & (t > 10.0))[0]
    t_sw = t[idx[0]] if len(idx) else 20.0
    hov = t > t_sw
    # error at the END of the hover (last 1 s mean), in % of the true mass
    tail = t > t[-1] - 1.0
    e_g = 100*(np.mean(d[tail, c["m_ekfg"]]) - M_TRUE)/M_TRUE
    e_p = 100*(np.mean(d[tail, c["m_ekf"]]) - M_TRUE)/M_TRUE
    # RMS mass error over the settled hover (t > t_sw + 2 s): captures wandering, not only the end value
    hv = t > t_sw + 2.0
    r_g = 100*np.sqrt(np.mean((d[hv, c["m_ekfg"]] - M_TRUE)**2))/M_TRUE
    r_p = 100*np.sqrt(np.mean((d[hv, c["m_ekf"]] - M_TRUE)**2))/M_TRUE
    # force reading error over the hover: m*d_hat(EKF) vs applied ground truth (theta,vtheta,a_theta cols)
    gt = np.stack([d[hv, c["theta"]], d[hv, c["vtheta"]], d[hv, c["a_theta"]]], 1)
    fh = M_TRUE*np.stack([d[hv, c["dx_ekf"]], d[hv, c["dy_ekf"]], d[hv, c["dz_ekf"]]], 1)
    f_rmse = float(np.mean(np.sqrt(np.mean((fh - gt)**2, 0))))
    # high-frequency noise of the force reading (sample-to-sample std, per axis mean) [N]
    f_hf = float(np.mean(np.std(np.diff(fh, axis=0), 0)/np.sqrt(2)))
    # identification error at the hover onset (gated), i.e. what the gate holds
    at = (t > t_sw - 1.0) & (t <= t_sw)
    e_id = 100*(np.mean(d[at, c["m_ekfg"]]) - M_TRUE)/M_TRUE
    # peak speed
    v = np.sqrt(d[:, c["vx"]]**2 + d[:, c["vy"]]**2 + d[:, c["vz"]]**2)
    return dict(e_g=e_g, e_p=e_p, r_g=r_g, r_p=r_p, f_rmse=f_rmse, f_hf=f_hf, e_id=e_id, t_sw=t_sw, vpk=np.max(v[t > 3.0]))

def group(cfg):
    files = sorted(glob.glob(os.path.join(RES, f"sens_{cfg}_seed*.csv")))
    if not files: return None
    thr = float(cfg.split("_")[1]) if cfg.startswith("thr_") else THR
    m = [run_metrics(f, thr) for f in files]
    out = {k: (np.mean([x[k] for x in m]), np.std([x[k] for x in m]), len(m)) for k in m[0]}
    return out

# ── panels: (title, x-label, [(cfg, x-value)], log-x) ─────────────────────────
PANELS = [
    ("(a) threshold", r"$\tilde\sigma_{\min}$",
     [("thr_0.003", .003), ("thr_0.006", .006), ("base", .009), ("thr_0.015", .015), ("thr_0.03", .03), ("thr_0.06", .06)], True),
    ("(b) dwell", r"$t_d$ [s]",
     [("dwell_10", .10), ("base", .25), ("dwell_50", .50), ("dwell_100", 1.0)], True),
    ("(c) window", "window [s]",
     [("win_50", .5), ("base", 1.0), ("win_200", 2.0)], True),
    ("(d) $q_m$", r"$q_m$ (per step)",
     [("qm_1e-6", 1e-6), ("base", 1e-5), ("qm_1e-4", 1e-4)], True),
    ("(e) $q_d$", r"$q_d$ (per step)",
     [("qd_1e-3", 1e-3), ("base", 1e-2), ("qd_1e-1", 1e-1)], True),
    ("(f) accel. noise", r"added std [m/s$^2$]",
     [("base", 0.0), ("anoise_0.3", 0.3), ("anoise_0.7", 0.7)], False),
    ("(g) maneuver intensity", "peak speed [m/s]",
     [("speed_0.5", None), ("base", None), ("speed_1.5", None)], False),
]
YMAX = {"(g)": 12.0, "(e)": 15.0}   # clip; off-scale points are annotated

lines = []
def log(s=""):
    print(s); lines.append(s)

fig, axs = plt.subplots(1, 6, figsize=(7.2, 1.75))
axs = axs.ravel()
for k, (title, xl, items, logx) in enumerate(PANELS):
    ax = axs[k] if k < 6 else None
    xs, g, gs, p, ps = [], [], [], [], []
    log(f"--- {title}")
    for cfg, x in items:
        r = group(cfg)
        if r is None: log(f"  {cfg:12s} (missing)"); continue
        if x is None: x = r["vpk"][0]
        xs.append(x); g.append(r["r_g"][0]); gs.append(r["r_g"][1]); p.append(r["r_p"][0]); ps.append(r["r_p"][1])
        log(f"  {cfg:12s} x={x:<8.4g} N={r['e_g'][2]}  RMS gated {r['r_g'][0]:5.1f}+-{r['r_g'][1]:.1f}%  plain {r['r_p'][0]:5.1f}+-{r['r_p'][1]:.1f}%  |  "
            f"end gated {r['e_g'][0]:+5.1f}+-{r['e_g'][1]:.1f}%  plain {r['e_p'][0]:+5.1f}+-{r['e_p'][1]:.1f}%  |  "
            f"force rmse {r['f_rmse'][0]:.2f}+-{r['f_rmse'][1]:.2f} N hf-noise {r['f_hf'][0]:.3f} N  id@hover {r['e_id'][0]:+4.1f}%  vpk={r['vpk'][0]:.1f}")
    if not xs or ax is None: continue
    o = np.argsort(xs); xs = np.array(xs)[o]; g = np.array(g)[o]; gs = np.array(gs)[o]; p = np.array(p)[o]; ps = np.array(ps)[o]
    ymax = YMAX.get(title[:3])
    ax.errorbar(xs, p, ps, fmt="s--", color="#c0392b", capsize=1.5, ms=3, lw=1.1, label="plain EKF")
    ax.errorbar(xs, g, gs, fmt="o-", color="#1f77b4", capsize=1.5, ms=3, lw=1.1, label="gated (ours)")
    if ymax:
        ax.set_ylim(0, ymax)
        for x, gv, pv in zip(xs, g, p):
            if gv > ymax or pv > ymax:
                ax.annotate(f"{pv:.0f} / {gv:.0f} %", (x, ymax*0.93), ha="left", va="top", fontsize=6.5,
                            xytext=(3, 0), textcoords="offset points")
                ax.plot([x], [ymax*0.98], marker="^", color="0.3", ms=4, clip_on=False)
    else:
        ax.set_ylim(0, None)
    if logx:
        ax.set_xscale("log"); ax.set_xticks(xs); ax.minorticks_off()
        if title.startswith("(d)") or title.startswith("(e)"):
            ax.set_xticklabels([r"$10^{%d}$" % int(np.round(np.log10(x))) for x in xs])
        elif title.startswith("(a)"):
            ax.set_xticklabels([f"{x:g}".lstrip("0") for x in xs], rotation=45, ha="right", rotation_mode="anchor")
        else:
            ax.set_xticklabels([f"{x:g}" for x in xs])
    ax.set_title(title, fontsize=7, pad=2); ax.set_xlabel(xl, labelpad=1)
    ax.tick_params(labelsize=6.5, pad=1)
axs[0].set_ylabel("hover mass RMS error [%]", fontsize=7)
fig.legend(*axs[0].get_legend_handles_labels(), loc="upper center", ncol=2, frameon=False, fontsize=7.5, bbox_to_anchor=(0.5, 1.12))
fig.tight_layout(rect=(0, 0, 1, 0.98), w_pad=0.6)
os.makedirs(FIGS, exist_ok=True)
fig.savefig(os.path.join(FIGS, "fig_sens.png"), dpi=300)
open(os.path.join(RES, "sens_analysis.txt"), "w").write("\n".join(lines) + "\n")
print("[OK] fig_sens.png")
