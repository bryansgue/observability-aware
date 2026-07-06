#!/usr/bin/env python3
"""
plot_paper_figures.py — publication-quality figures for the MHE virtual-sensor paper.

Each result is a SEPARATE figure (IEEE single/double-column friendly), English labels.

Usage:
    python3 plot_paper_figures.py filtering  <csv>
    python3 plot_paper_figures.py force      <csv> <out.png> "Title"
    python3 plot_paper_figures.py params     <csv>     # needs a mass-convergence run (init 0.60)
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 12,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "figure.dpi": 200,
    "lines.linewidth": 1.6,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linestyle": "--",
    "savefig.bbox": "tight",
})

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE = 1.05

# CSV columns
T = 0
PX, PY, PZ, VX, VY, VZ = 1, 2, 3, 4, 5, 6
M_HAT, TAU_RC, TAU_F = 32, 33, 38
DX, DY, DZ = 34, 35, 36
FX, FY, FZ = 28, 29, 30   # ground-truth external force [N]
PXh, PYh, PZh, VXh, VYh, VZh = 42, 43, 44, 45, 46, 47

C_MEAS = "0.55"
C_EST  = "#c0392b"
C_TRUE = "k"


def load(csv):
    p = csv if os.path.isabs(csv) else os.path.join(RES, csv)
    return np.loadtxt(p, delimiter=",", skiprows=1)


# ── Figure: state filtering ───────────────────────────────────────────────────
def fig_filtering(csv):
    a = load(csv); t = a[:, T]
    # HF residual computed on the FULL signal (no edge artifacts), then sliced.
    def hfres(col, w=7):
        x = a[:, col]; k = np.ones(w)/w; return x - np.convolve(x, k, "same")
    z = (t > 28.0) & (t < 34.0)
    s = (t > 20) & (t < 55)
    rp_meas = np.std(hfres(PX)[s])*1e3; rp_est = np.std(hfres(PXh)[s])*1e3
    rv_meas = np.std(hfres(VX)[s])*1e3; rv_est = np.std(hfres(VXh)[s])*1e3
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    ax[0].plot(t[z], hfres(PX)[z]*1e3, "-", color=C_MEAS, lw=1.0,
               label=f"noisy odometry  ({rp_meas:.1f} mm rms)")
    ax[0].plot(t[z], hfres(PXh)[z]*1e3, "-", color="#1f6fb4", lw=1.8,
               label=f"MHE estimate  ({rp_est:.1f} mm rms)")
    ax[0].axhline(0, color="k", lw=0.5, ls=":")
    ax[0].set_ylabel(r"$p_x$ noise [mm]")
    ax[0].legend(loc="upper right", fontsize=9, framealpha=0.9)
    ax[1].plot(t[z], hfres(VX)[z]*1e3, "-", color=C_MEAS, lw=1.0,
               label=f"noisy odometry  ({rv_meas:.0f} mm/s rms)")
    ax[1].plot(t[z], hfres(VXh)[z]*1e3, "-", color="#e67e22", lw=1.8,
               label=f"MHE estimate  ({rv_est:.0f} mm/s rms)")
    ax[1].axhline(0, color="k", lw=0.5, ls=":")
    ax[1].set_ylabel(r"$v_x$ noise [mm/s]"); ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="upper right", fontsize=9, framealpha=0.9)
    ax[0].set_title(f"State filtering: noise reduction "
                    f"$p$ −{100*(1-rp_est/rp_meas):.0f}%, $v$ −{100*(1-rv_est/rv_meas):.0f}%")
    out = os.path.join(RES, "paper_filtering.png")
    fig.savefig(out, dpi=300); print(f"[OK] {out}")
    # numeric
    s = (t > 20) & (t < 55)
    def hf(x, w=7):
        k = np.ones(w)/w; sm = np.convolve(x, k, "same"); return np.std((x-sm)[w:-w])
    for nm, cm, ce in [("px",1,42),("vx",4,45)]:
        print(f"   {nm}: noise {hf(a[s,cm])*1e3:.1f} -> {hf(a[s,ce])*1e3:.1f} (-{100*(1-hf(a[s,ce])/hf(a[s,cm])):.0f}%)")


# ── Figure: external force estimation (virtual sensor) ────────────────────────
def fig_force(csv, out, title):
    a = load(csv); t = a[:, T]; run = t < 58
    s = (t > 1) & (t < 58)
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    for k, (cf, cd, lab) in enumerate([(FX, DX, "x"), (FY, DY, "y")]):
        dtrue = a[run, cf] / M_TRUE; dhat = a[run, cd]
        c = np.corrcoef(a[s, cf]/M_TRUE, a[s, cd])[0, 1]
        ax[k].plot(t[run], dtrue, "-", color=C_TRUE, lw=2.2, label="ground-truth wind / m")
        ax[k].plot(t[run], dhat, "-", color=C_EST, lw=1.4, alpha=0.85,
                   label=fr"MHE estimate $\hat d_{lab}$  ($\rho={c:.2f}$)")
        ax[k].set_ylabel(fr"$d_{lab}$ [m/s$^2$]")
        ax[k].legend(loc="upper right", ncol=1, framealpha=0.9)
    ax[1].set_xlabel("time [s]")
    ax[0].set_title(title)
    o = os.path.join(RES, out)
    fig.savefig(o, dpi=300); print(f"[OK] {o}")
    for cf, cd, lab in [(FX, DX, "x"), (FY, DY, "y")]:
        c = np.corrcoef(a[s, cf]/M_TRUE, a[s, cd])[0, 1]
        print(f"   corr d_{lab} = {c:.3f}")


# ── Figure: parameter identification (needs init-0.60 run) ────────────────────
def fig_params(csv):
    a = load(csv); t = a[:, T]; run = t < 58
    fig, ax = plt.subplots(3, 1, figsize=(3.5, 5.2), sharex=True)
    ax[0].axhline(M_TRUE, color="k", ls="--", lw=1.2, label=f"true = {M_TRUE}")
    ax[0].plot(t[run], a[run, M_HAT], color="#1f6fb4", label=r"$\hat m$")
    ax[0].set_ylabel(r"$\hat m$ [kg]"); ax[0].set_ylim(0.5, 1.2); ax[0].legend(loc="lower right")
    ax[1].axhline(0.055, color="k", ls="--", lw=1.2, label="identified")
    ax[1].plot(t[run], a[run, TAU_RC], color="#8c564b", label=r"$\hat\tau_{rc}$")
    ax[1].set_ylabel(r"$\hat\tau_{rc}$ [s]"); ax[1].set_ylim(0, 0.12); ax[1].legend(loc="best")
    ax[2].axhline(0.007, color="k", ls="--", lw=1.2, label="identified")
    ax[2].plot(t[run], a[run, TAU_F], color="#17a2b8", label=r"$\hat\tau_f$")
    ax[2].set_ylabel(r"$\hat\tau_f$ [s]"); ax[2].set_ylim(0, 0.05); ax[2].set_xlabel("time [s]")
    ax[2].legend(loc="best")
    out = os.path.join(RES, "paper_params.png")
    fig.savefig(out, dpi=300); print(f"[OK] {out}")


# ── Figure: position hold under disturbance (rejection in hover) ──────────────
def fig_pos(csv, out, title):
    """Per-axis position deviation from the setpoint vs the applied force.
    Shows how the controller holds (or not) the hover point while perturbed."""
    PXr, PYr, PZr = 21, 22, 23  # setpoint columns
    a = load(csv); t = a[:, T]; run = t < 58
    s = (t > 1) & (t < 58)
    e = np.sqrt((a[s, PX]-a[s, PXr])**2 + (a[s, PY]-a[s, PYr])**2
                + (a[s, PZ]-a[s, PZr])**2)
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
    # top: per-axis deviation from setpoint [cm]
    for col, cref, lab, c in [(PX, PXr, "x", "#c0392b"),
                              (PY, PYr, "y", "#1f6fb4"),
                              (PZ, PZr, "z", "#27ae60")]:
        ax[0].plot(t[run], (a[run, col]-a[run, cref])*100, "-", lw=1.4,
                   color=c, label=fr"$\Delta {lab}$")
    ax[0].axhline(0, color="k", lw=0.5, ls=":")
    ax[0].set_ylabel("deviation [cm]")
    ax[0].legend(loc="upper right", ncol=3, framealpha=0.9)
    ax[0].set_title(f"{title}   (RMSE = {e.mean()*100:.1f} cm, max = {e.max()*100:.1f} cm)")
    # bottom: applied ground-truth force [N]
    for col, lab, c in [(FX, "x", "#c0392b"), (FY, "y", "#1f6fb4"), (FZ, "z", "#27ae60")]:
        ax[1].plot(t[run], a[run, col], "-", lw=1.2, color=c, label=fr"$F_{lab}$")
    ax[1].axhline(0, color="k", lw=0.5, ls=":")
    ax[1].set_ylabel("applied force [N]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="upper right", ncol=3, framealpha=0.9)
    o = os.path.join(RES, out)
    fig.savefig(o, dpi=300); print(f"[OK] {o}")
    print(f"   RMSE pos = {e.mean()*100:.1f} cm   max = {e.max()*100:.1f} cm")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "filtering":
        fig_filtering(sys.argv[2])
    elif cmd == "force":
        fig_force(sys.argv[2], sys.argv[3], sys.argv[4])
    elif cmd == "pos":
        fig_pos(sys.argv[2], sys.argv[3], sys.argv[4])
    elif cmd == "params":
        fig_params(sys.argv[2])
