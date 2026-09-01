#!/usr/bin/env python3
"""
analyze_r1.py — offline Fisher-information analyses for the resubmission
(Reviewer 1, concerns 2–4; Reviewer 5, concern 4).

From the logged thrust T and attitude q (a = R(q) e3) of the existing runs:
  A. Cramer–Rao bound on the mass vs the empirical across-run spread of the
     plain-EKF mass, per trajectory speed (bat_*_noff_*.csv, bat_hover_*).
  B. Exact marginal Fisher information on beta when d is a random walk of
     per-step variance q_d (the implemented EKF model) vs the constant-d Schur
     complement sigma: ratio sigma_eff/sigma against window length and q_d.
  C. sigma_tilde against window length in hover vs maneuver (trans runs).
  D. Table: sigma_tilde (classification) vs sigma/r_s (information) per regime.
  E. Threshold <-> CRB: mass std that a 1-s window at sigma_tilde_min bounds.

Outputs: results/r1_analysis.txt, paper/figs/fig_fisher.png
"""
import os, sys, glob, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 9, "axes.labelsize": 9,
    "axes.titlesize": 9, "legend.fontsize": 7.5, "xtick.labelsize": 8,
    "ytick.labelsize": 8, "figure.dpi": 200, "lines.linewidth": 1.4,
    "lines.markersize": 4,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--",
    "savefig.bbox": "tight",
})
HERE = os.path.dirname(os.path.abspath(__file__))
RES  = os.path.join(HERE, "..", "results")
FIGS = os.path.join(HERE, "..", "paper", "figs")
M_TRUE, G, DT = 1.05, 9.81, 0.01
R_S   = 0.5          # accelerometer variance assumed by the filter [(m/s^2)^2]
Q_D   = 1e-2         # EKF per-step random-walk variance on d
THR   = 0.009
WIN   = 100          # 1 s
SPEED = {"hover": 0.0, "0.3": 2.1, "0.6": 3.8, "1.0": 6.1, "1.4": 8.2, "1.8": 10.2}
out_lines = []
def log(s=""):
    print(s); out_lines.append(s)

def load(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(",")
    return d, {n: i for i, n in enumerate(h)}

def thrust_axis(d, c):
    T = d[:, c["T"]]
    qw, qx, qy, qz = (d[:, c[k]] for k in ("qw", "qx", "qy", "qz"))
    a = np.stack([2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx**2 + qy**2)], 1)
    return T, a

def sigma_window(T, a):
    """Schur complement sigma and its normalization over one window."""
    s2 = np.sum(T**2)
    sig = s2 - np.sum(np.sum(T[:, None]*a, 0)**2)/len(T)
    return sig, (sig/s2 if s2 > 0 else 0.0)

def sigma_series(T, a, win):
    n = len(T); sig = np.full(n, np.nan); sn = np.full(n, np.nan)
    for k in range(win, n):
        sig[k], sn[k] = sigma_window(T[k-win:k], a[k-win:k])
    return sig, sn

def sigma_eff(T, a, q_d, r_s=R_S):
    """Exact marginal Fisher information on beta (times r_s, so it equals the
    Schur complement sigma when q_d = 0) for measurements
        s_k = T_k a_k beta + d_k + n_k,   d_k = d_{k-1} + w_k,
    w_k ~ N(0, q_d I3), n_k ~ N(0, r_s I3), flat prior on d_0 (nuisance)."""
    N = len(T)
    t = (T[:, None]*a).reshape(-1)                      # 3N
    M = np.tile(np.eye(3), (N, 1))                      # 3N x 3  (d_0 column)
    # covariance of the accumulated increments: q_d * min(k,l) (k,l = 0..N-1)
    idx = np.arange(N)
    K = q_d*np.minimum(idx[:, None], idx[None, :])
    Sigma = np.kron(K, np.eye(3)) + r_s*np.eye(3*N)
    X = np.linalg.solve(Sigma, np.column_stack([t, M]))   # Sigma^{-1}[t M]
    Jbb = t @ X[:, 0]
    Jbd = t @ X[:, 1:]
    Jdd = M.T @ X[:, 1:]
    return r_s*(Jbb - Jbd @ np.linalg.lstsq(Jdd, Jbd, rcond=None)[0])

# ─────────────────────────────── A. CRB vs empirical std ───────────────────────
log("=== A. Cramer-Rao bound vs empirical mass spread (plain EKF, per speed) ===")
crb_rows = []
for tag, v in SPEED.items():
    files = sorted(glob.glob(os.path.join(RES, f"bat_{tag}_*_*.csv")))
    if not files: continue
    m_final, sig_run, sn_1s = [], [], []
    for f in files:
        d, c = load(f); T, a = thrust_axis(d, c)
        tt = d[:, c["t"]]
        k0 = np.searchsorted(tt, 3.0)                     # skip take-off transient
        m_final.append(np.mean(d[tt > 3.0, c["m_ekf"]]))   # same statistic as Table (plot_mass_id)
        s, _ = sigma_window(T[k0:], a[k0:])
        sig_run.append(s)
        sg, sn = sigma_series(T[k0:], a[k0:], WIN)
        sn_1s.append(np.nanmedian(sn))
    m_final = np.array(m_final)
    emp_std = m_final.std(ddof=0)   # population estimator, as in Table 4
    sig_tot = np.mean(sig_run)
    crb_std = M_TRUE**2*np.sqrt(R_S/sig_tot) if sig_tot > 1e-9 else np.inf
    # exact random-walk information ratio at the paper's q_d over 1-s windows
    rr = []
    for f in files[:3]:
        d, c = load(f); T, a = thrust_axis(d, c); tt = d[:, c["t"]]
        for t0 in (8.0, 15.0, 22.0, 29.0):
            k0 = np.searchsorted(tt, t0)
            if k0 + WIN > len(T): continue
            Tw, aw = T[k0:k0+WIN], a[k0:k0+WIN]; sg, _ = sigma_window(Tw, aw)
            if sg > 1e-6: rr.append(sigma_eff(Tw, aw, Q_D)/sg)
    rw_ratio = float(np.mean(rr)) if rr else np.nan
    crb_rows.append((v, tag, len(files), emp_std, crb_std, sig_tot, np.median(sn_1s), rw_ratio))
    log(f"  {v:5.1f} m/s  N={len(files):2d}  emp std={emp_std*1e3:6.1f} g   "
        f"CRB std={crb_std*1e3:8.2f} g   CRB_rw std={crb_std*1e3/np.sqrt(rw_ratio):8.2f} g "
        f"(ratio {rw_ratio:.3f})   sigma_run={sig_tot:9.1f} N^2   "
        f"median sigma_tilde(1s)={np.median(sn_1s):.4f}")

# ─────────────────────────── B. random-walk exact information ─────────────────
log("\n=== B. sigma_eff/sigma (d random walk, flat d_0) vs window and q_d ===")
tf = sorted(glob.glob(os.path.join(RES, "trans_seed*.csv")))
d, c = load(tf[0]); T, a = thrust_axis(d, c); tt = d[:, c["t"]]
_, sn_full = sigma_series(T, a, WIN)
# maneuver segment: highest 1-s sigma_tilde; hover segment: t > t_switch
low = (sn_full < THR) & (tt > 2.0)
sust = np.convolve(low.astype(float), np.ones(50)/50, "same") > 0.8
t_switch = tt[np.where(sust)[0][0]]
sn_man = np.where((tt > 4.0) & (tt < t_switch - 4.0), sn_full, np.nan)   # exclude take-off
kman = int(np.nanargmax(sn_man))
wins = [25, 50, 100, 200, 400]
qds  = [0.0, 1e-4, 1e-3, 1e-2, 1e-1]
ratio = np.zeros((len(qds), len(wins))); sn_eff = np.zeros_like(ratio); sn_plain = np.zeros(len(wins))
for j, w in enumerate(wins):
    k1 = min(kman + w//2, len(T)); k0 = k1 - w
    Tw, aw = T[k0:k1], a[k0:k1]
    sig, sn = sigma_window(Tw, aw); sn_plain[j] = sn
    for i, q in enumerate(qds):
        se = sigma_eff(Tw, aw, q)
        ratio[i, j] = se/sig; sn_eff[i, j] = se/np.sum(Tw**2)
log("  window[s]:      " + "  ".join(f"{w*DT:6.2f}" for w in wins))
log("  sigma_tilde:    " + "  ".join(f"{x:6.4f}" for x in sn_plain))
for i, q in enumerate(qds):
    log(f"  q_d={q:<7.0e} ratio " + "  ".join(f"{x:6.3f}" for x in ratio[i]) +
        "   sn_eff " + "  ".join(f"{x:6.4f}" for x in sn_eff[i]))
# hover check: sigma_eff stays ~0 in hover for every q_d
kh = np.searchsorted(tt, t_switch + 5.0)
Th, ah = T[kh:kh+WIN], a[kh:kh+WIN]
sh, snh = sigma_window(Th, ah)
log(f"  hover 1-s window: sigma_tilde={snh:.5f}, sigma_tilde_eff(q_d=1e-2)="
    f"{sigma_eff(Th, ah, Q_D)/np.sum(Th**2):.5f}")

# ─────────────────────────── C. sigma_tilde vs window length ──────────────────
log("\n=== C. sigma_tilde vs window: hover vs maneuver (trans runs, 5 seeds) ===")
wins_c = [25, 50, 100, 200, 400]
hov_med, man_med, hov_p95 = [], [], []
for w in wins_c:
    hv, mv, hp = [], [], []
    for f in tf:
        d, c = load(f); T, a = thrust_axis(d, c); tt = d[:, c["t"]]
        _, sn = sigma_series(T, a, w)
        hov = (tt > t_switch + 2.0); man = (tt > 3.0) & (tt < t_switch - 3.0)
        hv.append(np.nanmedian(sn[hov])); hp.append(np.nanpercentile(sn[hov], 95))
        mv.append(np.nanmedian(sn[man]))
    hov_med.append(np.mean(hv)); hov_p95.append(np.mean(hp)); man_med.append(np.mean(mv))
    log(f"  win={w*DT:4.2f}s  hover median={np.mean(hv):.5f} p95={np.mean(hp):.5f}   "
        f"maneuver median={np.mean(mv):.4f}   ratio={np.mean(mv)/np.mean(hv):7.1f}")

# ─────────────────────────── D. sigma_tilde vs sigma/r_s per regime ───────────
log("\n=== D. classification signal vs information content (1-s window) ===")
log("  regime          sigma_tilde   sigma [N^2]   sigma/r_s   CRB std(m) per 1-s window [g]")
regimes = []
for v, tag, n, es, cs, st, sn1, _rw in crb_rows:
    files = sorted(glob.glob(os.path.join(RES, f"bat_{tag}_*_*.csv")))
    sig1 = []
    for f in files:
        d, c = load(f); T, a = thrust_axis(d, c); tt = d[:, c["t"]]
        k0 = np.searchsorted(tt, 3.0); sg, _ = sigma_series(T[k0:], a[k0:], WIN)
        sig1.append(np.nanmedian(sg))
    regimes.append((f"{v:4.1f} m/s" if v > 0 else "hover", sn1, np.median(sig1)))
# active probe (lateral) — probe phase = where sigma_tilde exceeds threshold
for f in sorted(glob.glob(os.path.join(RES, "active_probe_seed*.csv")))[:1]:
    d, c = load(f); T, a = thrust_axis(d, c)
    sg, sn = sigma_series(T, a, WIN)
    on = sn > THR
    regimes.append(("probe (lateral)", np.nanmedian(sn[on]), np.nanmedian(sg[on])))
for name, sn1, sg1 in regimes:
    crb = M_TRUE**2*np.sqrt(R_S/sg1)*1e3 if sg1 > 0 else np.inf
    log(f"  {name:16s}  {sn1:9.4f}   {sg1:10.1f}   {sg1/R_S:10.1f}   {crb:8.1f}")

# ─────────────────────────── E. threshold <-> CRB ─────────────────────────────
log("\n=== E. threshold <-> allowable mass variance ===")
T_h = M_TRUE*G
sum_T2 = WIN*T_h**2
for thr in [0.003, 0.006, 0.009, 0.015, 0.03]:
    sig = thr*sum_T2
    std_m = M_TRUE**2*np.sqrt(R_S/sig)
    log(f"  sigma_tilde_min={thr:.3f}: 1-s window CRB std(m) = {std_m*1e3:5.1f} g "
        f"({100*std_m/M_TRUE:4.1f} %);  after 10 s of such data: {100*std_m/M_TRUE/np.sqrt(10):4.1f} %")

# ─────────────────────────── figure ───────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.0, 2.7))
v  = np.array([r[0] for r in crb_rows]); es = np.array([r[3] for r in crb_rows])*1e3
cs = np.array([r[4] for r in crb_rows])*1e3
ax1.semilogy(v, es, "o-", color="#c0392b", label="empirical std, plain EKF")
rw = np.array([r[7] for r in crb_rows])
ax1.semilogy(v, np.clip(cs, 1e-3, 1e4), "s--", color="#1f77b4", label=r"CRB, constant $d$")
ax1.semilogy(v, np.clip(cs/np.sqrt(rw), 1e-3, 1e4), "^:", color="#2e8b57", label=r"CRB, random-walk $d$")
ax1.set_xlabel("peak speed [m/s]\n(a)"); ax1.set_ylabel(r"mass std [g]")

ax1.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=1, frameon=False, fontsize=6, borderaxespad=0.15, handlelength=1.2, labelspacing=0.15)
for i, q in enumerate(qds):
    ax2.plot(np.array(wins)*DT, ratio[i], "o-", ms=3, lw=1.4,
             label=("const. $d$" if q == 0 else fr"$q_d\!=\!{q:g}$"))
ax2.set_xscale("log"); ax2.set_xticks(np.array(wins)*DT)
ax2.set_xticklabels([f"{w*DT:g}" for w in wins]); ax2.minorticks_off()
ax2.set_xlabel("window length [s]\n(b)")
ax2.set_ylabel(r"$\sigma_{\mathrm{eff}}/\sigma$")
ax2.set_ylim(0, 1.05)
ax2.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False,
           fontsize=6, borderaxespad=0.15, columnspacing=0.6, handlelength=1.0,
           handletextpad=0.3, labelspacing=0.15)
fig.tight_layout(w_pad=2.5)
os.makedirs(FIGS, exist_ok=True)
fig.savefig(os.path.join(FIGS, "fig_fisher.png"), dpi=300)
open(os.path.join(RES, "r1_analysis.txt"), "w").write("\n".join(out_lines) + "\n")
print("[OK] fig_fisher.png + r1_analysis.txt")
