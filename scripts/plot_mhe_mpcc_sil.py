#!/usr/bin/env python3
"""
plot_mhe_mpcc_sil.py — Visualización completa del experimento MHE + Adaptive W_s + MPCC.

Uso:
    python3 plot_mhe_mpcc_sil.py [vtheta_max]
    python3 plot_mhe_mpcc_sil.py 10

Genera figuras en: figures_mhe_mpcc_sil/v{X}/
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
try:
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    pass

# ── Estilo ────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 10,
    "axes.labelsize": 10,
    "axes.titlesize": 11,
    "legend.fontsize": 9,
    "figure.dpi": 150,
    "lines.linewidth": 1.4,
    "axes.grid": True,
    "grid.alpha": 0.3,
})

BLUE   = "#1f77b4"
ORANGE = "#ff7f0e"
GREEN  = "#2ca02c"
RED    = "#d62728"
PURPLE = "#9467bd"
GRAY   = "#7f7f7f"

# ── Argumentos ────────────────────────────────────────────────────────────────
vel = int(sys.argv[1]) if len(sys.argv) > 1 else 10

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.join(SCRIPT_DIR, "..")
CSV_PATH   = os.path.join(BASE_DIR, "results", f"mhe_mpcc_sil_v{vel}.csv")
FIG_DIR    = os.path.join(BASE_DIR, f"figures_mhe_mpcc_sil", f"v{vel}")
os.makedirs(FIG_DIR, exist_ok=True)

if not os.path.exists(CSV_PATH):
    print(f"ERROR: no se encontró {CSV_PATH}")
    sys.exit(1)

# ── Carga de datos ─────────────────────────────────────────────────────────────
d = np.genfromtxt(CSV_PATH, delimiter=",", names=True)
t  = d["t"]
N  = len(t)

# Estados físicos
px, py, pz = d["px"], d["py"], d["pz"]
vx, vy, vz = d["vx"], d["vy"], d["vz"]
qw, qx_d, qy_d, qz_d = d["qw"], d["qx"], d["qy"], d["qz"]
wx, wy, wz = d["wx"], d["wy"], d["wz"]

# Referencia en θ
px_ref, py_ref, pz_ref = d["px_ref"], d["py_ref"], d["pz_ref"]
qw_ref = d["qw_ref"]

# MPCC virtual states
theta   = d["theta"]
vtheta  = d["vtheta"]
a_theta = d["a_theta"]
s_near  = d["s_nearest"]

# Comandos
T       = d["T"]
wx_cmd  = d["wx_cmd"]
wy_cmd  = d["wy_cmd"]
wz_cmd  = d["wz_cmd"]

# MHE estimaciones
m_hat   = d["m_hat"]
tau_hat = d["tau_hat"]
dx_hat  = d["dx_hat"]
dy_hat  = d["dy_hat"]
dz_hat  = d["dz_hat"]
sigma_k = d["sigma_k"]
W_s     = d["W_s"]

# Tiempos de cómputo
mpcc_ms = d["mpcc_solve_ms"]
mhe_ms  = d["mhe_ms"]
loop_ms = d["loop_ms"]

# Errores de tracking (posición)
ep = np.sqrt((px - px_ref)**2 + (py - py_ref)**2 + (pz - pz_ref)**2)
ev = np.sqrt(vx**2 + vy**2 + vz**2)

# Velocidad total [m/s]
speed = ev.copy()

# Errores de contouring (perpendicular al tangente en s_nearest)
# Aproximamos con distancia euclidea entre drone y ref en theta
ec_approx = ep  # contouring es subconjunto del error posición

print(f"Datos: {N} pasos, t=[0, {t[-1]:.2f}]s, s_max={s_near.max():.1f}m")
print(f"m_hat: {m_hat[0]:.3f} → {m_hat[-1]:.3f} kg  (true=1.080)")
print(f"sigma_k: {sigma_k[0]:.3e} → {sigma_k[-1]:.3e}")
print(f"W_s: {W_s[0]:.3f} → {W_s[-1]:.3f}")
print(f"RMSE posición: {np.sqrt(np.mean(ep**2)):.4f} m")


def savefig(fig, name):
    path = os.path.join(FIG_DIR, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  → {path}")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 1 — Trayectoria 3D (o 2D proyecciones si 3D no está disponible)
# ══════════════════════════════════════════════════════════════════════════════
try:
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(px_ref, py_ref, pz_ref, "--", color=GRAY, lw=1.0, label="Referencia γ(θ)")
    ax.plot(px, py, pz, color=BLUE, lw=1.2, label="Drone (MHE-MPCC)")
    ax.scatter([px[0]], [py[0]], [pz[0]], color=GREEN, s=40, zorder=5, label="Inicio")
    ax.scatter([px[-1]], [py[-1]], [pz[-1]], color=RED, s=40, zorder=5, label="Fin")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title(f"Trayectoria 3D — MHE-MPCC, $v_{{\\theta,\\max}}={vel}$ m/s")
    ax.legend(loc="upper right")
    savefig(fig, "01_trajectory_3d.pdf")
except Exception:
    # Fallback: 2D projections
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(px_ref, py_ref, "--", color=GRAY, lw=1.0, label="Ref")
    axes[0].plot(px, py, color=BLUE, lw=1.2, label="Drone")
    axes[0].scatter([px[0]], [py[0]], color=GREEN, s=40, label="Inicio")
    axes[0].scatter([px[-1]], [py[-1]], color=RED, s=40, label="Fin")
    axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]")
    axes[0].set_title("Plano XY"); axes[0].legend()
    axes[1].plot(px_ref, pz_ref, "--", color=GRAY, lw=1.0, label="Ref")
    axes[1].plot(px, pz, color=BLUE, lw=1.2, label="Drone")
    axes[1].scatter([px[0]], [pz[0]], color=GREEN, s=40)
    axes[1].scatter([px[-1]], [pz[-1]], color=RED, s=40)
    axes[1].set_xlabel("x [m]"); axes[1].set_ylabel("z [m]")
    axes[1].set_title("Plano XZ"); axes[1].legend()
    fig.suptitle(f"Trayectoria — MHE-MPCC, $v_{{\\theta,\\max}}={vel}$ m/s")
    fig.tight_layout()
    savefig(fig, "01_trajectory_3d.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 2 — Tracking posición (x, y, z)
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
for ax, pos, ref, lbl in zip(axes,
        [px, py, pz], [px_ref, py_ref, pz_ref], ["x", "y", "z"]):
    ax.plot(t, ref, "--", color=GRAY, lw=1.0, label="Ref")
    ax.plot(t, pos, color=BLUE, label=f"${lbl}$")
    ax.set_ylabel(f"${lbl}$ [m]")
    ax.legend(loc="upper right", ncol=2)
axes[-1].set_xlabel("Tiempo [s]")
fig.suptitle("Tracking de posición — MHE-MPCC")
fig.tight_layout()
savefig(fig, "02_position_tracking.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 3 — Error de posición y velocidad de avance
# ══════════════════════════════════════════════════════════════════════════════
fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
a1.plot(t, ep, color=RED, label="|e_p| [m]")
a1.axhline(np.sqrt(np.mean(ep**2)), color=RED, ls="--", lw=1.0,
           label=f"RMSE={np.sqrt(np.mean(ep**2)):.4f} m")
a1.set_ylabel("Error posición [m]")
a1.legend()
a2.plot(t, vtheta, color=BLUE, label="$v_\\theta$ [m/s]")
a2.axhline(vel, color=GRAY, ls="--", lw=1.0, label=f"$v_{{\\theta,\\max}}={vel}$ m/s")
a2.set_ylabel("Velocidad progreso [m/s]")
a2.set_xlabel("Tiempo [s]")
a2.legend()
fig.suptitle("Error de posición y velocidad de progreso")
fig.tight_layout()
savefig(fig, "03_error_vtheta.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 4 — MHE: convergencia de masa m̂
# ══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(9, 4))
ax.plot(t, m_hat, color=BLUE, label=r"$\hat{m}(t)$ [kg]")
ax.axhline(1.08, color=GREEN, ls="--", lw=1.5, label="$m_{\\rm true}=1.08$ kg")
ax.axhline(0.9,  color=RED,   ls=":",  lw=1.2, label="$\\hat{m}_{\\rm init}=0.90$ kg")
ax.set_xlabel("Tiempo [s]")
ax.set_ylabel("Masa estimada [kg]")
ax.set_title("MHE — Convergencia de masa")
ax.legend()
ax.set_ylim([0.4, 1.4])
savefig(fig, "04_mhat_convergence.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 5 — MHE: incertidumbre σ_k y W_s adaptativo
# ══════════════════════════════════════════════════════════════════════════════
fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

# σ_k en escala log
a1.semilogy(t, sigma_k, color=PURPLE, label=r"$\sigma_k = \mathrm{tr}(\bar{P}_{\theta,k})$")
a1.set_ylabel(r"$\sigma_k$ [log]")
a1.set_title(r"Incertidumbre paramétrica $\sigma_k$ y ganancia adaptativa $W_s(k)$")
a1.legend()

# W_s
W_s_max = d["W_s"].max()
a2.plot(t, W_s, color=ORANGE, label=r"$W_s(k) = W_{s,\max}/(1+\alpha\sigma_k)$")
a2.axhline(W_s_max, color=GRAY, ls="--", lw=1.0, label=f"$W_{{s,\\max}}={W_s_max:.2f}$")
a2.set_xlabel("Tiempo [s]")
a2.set_ylabel(r"$W_s$ [—]")
a2.legend()

fig.tight_layout()
savefig(fig, "05_sigma_Ws.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 6 — Panel MHE completo: m̂, τ̂, d̂, σ_k
# ══════════════════════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(11, 8))
gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

# m̂
ax = fig.add_subplot(gs[0, 0])
ax.plot(t, m_hat, color=BLUE)
ax.axhline(1.08, color=GREEN, ls="--", lw=1.2, label="$m_{\\rm true}$")
ax.set_ylabel("$\\hat{m}$ [kg]"); ax.set_title("Masa estimada"); ax.legend()

# τ̂
ax = fig.add_subplot(gs[0, 1])
ax.plot(t, tau_hat * 1e3, color=ORANGE)
ax.axhline(30.0, color=GREEN, ls="--", lw=1.2, label="$\\tau_{\\rm true}=30$ ms")
ax.set_ylabel("$\\hat{\\tau}_{rc}$ [ms]"); ax.set_title("Constante de tiempo estimada"); ax.legend()

# d̂
ax = fig.add_subplot(gs[1, :])
ax.plot(t, dx_hat, color=BLUE,   label="$\\hat{d}_x$")
ax.plot(t, dy_hat, color=ORANGE, label="$\\hat{d}_y$")
ax.plot(t, dz_hat, color=GREEN,  label="$\\hat{d}_z$")
ax.axhline(0, color=GRAY, ls="--", lw=0.8)
ax.set_ylabel("$\\hat{d}$ [m/s²]"); ax.set_title("Perturbación externa estimada"); ax.legend(ncol=3)

# σ_k (log)
ax = fig.add_subplot(gs[2, 0])
ax.semilogy(t, sigma_k, color=PURPLE)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\sigma_k$"); ax.set_title("Incertidumbre paramétrica")

# W_s
ax = fig.add_subplot(gs[2, 1])
ax.plot(t, W_s, color=ORANGE)
ax.axhline(W_s_max, color=GRAY, ls="--", lw=1.0)
ax.set_xlabel("t [s]"); ax.set_ylabel("$W_s$"); ax.set_title("Ganancia adaptativa de progreso")

fig.suptitle(f"MHE — Estado estimado completo ($v_{{\\theta,\\max}}={vel}$ m/s)", fontsize=12)
savefig(fig, "06_mhe_full_panel.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 7 — Progreso de trayectoria: θ, v_θ, s_nearest
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
axes[0].plot(t, theta,  color=BLUE,   label="$\\theta$ (virtual)")
axes[0].plot(t, s_near, color=GREEN,  ls="--", label="$s_{\\rm nearest}$ (geométrico)")
axes[0].set_ylabel("Arco [m]"); axes[0].legend()
axes[0].set_title("Progreso de trayectoria")

axes[1].plot(t, vtheta, color=ORANGE)
axes[1].axhline(vel, color=GRAY, ls="--", lw=1.0, label=f"$v_{{\\theta,\\max}}={vel}$")
axes[1].set_ylabel("$v_\\theta$ [m/s]"); axes[1].legend()

axes[2].plot(t, a_theta, color=RED)
axes[2].axhline(0, color=GRAY, ls="--", lw=0.8)
axes[2].set_ylabel("$a_\\theta$ [m/s²]"); axes[2].set_xlabel("Tiempo [s]")

fig.tight_layout()
savefig(fig, "07_progress_states.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 8 — Empuje y tasas de rotación comandadas
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
axes[0].plot(t, T, color=BLUE)
axes[0].axhline(1.08*9.81, color=GRAY, ls="--", lw=0.9, label="hover")
axes[0].set_ylabel("$f$ [N]"); axes[0].set_title("Empuje (estado)"); axes[0].legend()

for ax, cmd, lbl, col in zip(axes[1:],
        [wx_cmd, wy_cmd, wz_cmd], ["$\\omega_x$", "$\\omega_y$", "$\\omega_z$"],
        [ORANGE, GREEN, RED]):
    ax.plot(t, cmd, color=col, label=lbl + " cmd")
    ax.axhline(0, color=GRAY, ls="--", lw=0.6)
    ax.set_ylabel("[rad/s]"); ax.legend()

axes[-1].set_xlabel("Tiempo [s]")
fig.suptitle("Comandos de control — MPCC")
fig.tight_layout()
savefig(fig, "08_control_inputs.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 9 — Tiempos de cómputo
# ══════════════════════════════════════════════════════════════════════════════
fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
a1.plot(t, mpcc_ms, color=BLUE, lw=0.8, label="MPCC")
a1.axhline(mpcc_ms.mean(), color=BLUE, ls="--", lw=1.0,
           label=f"avg={mpcc_ms.mean():.2f} ms")
a1.set_ylabel("Solver [ms]"); a1.set_title("Tiempo de cómputo MPCC"); a1.legend()

a2.plot(t, mhe_ms, color=ORANGE, lw=0.8, label="MHE")
a2.axhline(mhe_ms.mean(), color=ORANGE, ls="--", lw=1.0,
           label=f"avg={mhe_ms.mean():.2f} ms")
a2.set_ylabel("Solver [ms]"); a2.set_title("Tiempo de cómputo MHE"); a2.legend()

a3.plot(t, loop_ms, color=PURPLE, lw=0.8, label="Loop total")
a3.axhline(10.0, color=RED, ls="--", lw=1.0, label="Budget 10 ms")
a3.set_ylabel("[ms]"); a3.set_xlabel("Tiempo [s]")
a3.set_title("Tiempo total por iteración"); a3.legend()

fig.tight_layout()
savefig(fig, "09_timing.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 10 — Figura de paper: σ_k + W_s + m̂ en un panel compacto
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))

# m̂ convergencia
ax = axes[0]
ax.plot(t, m_hat, color=BLUE, lw=1.5, label=r"$\hat{m}(t)$")
ax.axhline(1.08, color=GREEN, ls="--", lw=1.5, label=r"$m_{\rm true}$")
ax.fill_between(t, m_hat, 1.08, alpha=0.12, color=BLUE)
ax.set_xlabel("t [s]"); ax.set_ylabel("[kg]")
ax.set_title(r"(a) Convergencia de $\hat{m}$")
ax.legend(); ax.set_ylim([0.4, 1.4])

# σ_k decay (log)
ax = axes[1]
ax.semilogy(t, sigma_k, color=PURPLE, lw=1.5)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\sigma_k = \mathrm{tr}(\bar{P}_{\theta,k})$")
ax.set_title(r"(b) Decaimiento de $\sigma_k$")

# W_s adaptativo
ax = axes[2]
ax.plot(t, W_s, color=ORANGE, lw=1.5, label=r"$W_s(k)$")
ax.axhline(W_s_max, color=GRAY, ls="--", lw=1.2, label=f"$W_{{s,\\max}}={W_s_max:.1f}$")
ax.fill_between(t, 0, W_s, alpha=0.12, color=ORANGE)
ax.set_xlabel("t [s]"); ax.set_ylabel(r"$W_s$")
ax.set_title(r"(c) Ganancia adaptativa $W_s(k)$")
ax.legend(); ax.set_ylim([0, W_s_max * 1.15])

fig.suptitle(r"MHE + Adaptive $W_s$ — SiL $v_{\theta,\max}=" + str(vel) + r"$ m/s",
             fontsize=12, y=1.02)
fig.tight_layout()
savefig(fig, "10_paper_mhe_Ws.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 11 — Figura de paper: trayectoria XY + XZ + error
# ══════════════════════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(12, 4))
gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.35)

# XY
ax = fig.add_subplot(gs[0])
ax.plot(px_ref, py_ref, "--", color=GRAY, lw=1.0, label="Ref")
ax.plot(px, py, color=BLUE, lw=1.2, label="Drone")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("(a) Plano XY"); ax.legend()
ax.set_aspect("equal")

# XZ
ax = fig.add_subplot(gs[1])
ax.plot(px_ref, pz_ref, "--", color=GRAY, lw=1.0, label="Ref")
ax.plot(px, pz, color=BLUE, lw=1.2, label="Drone")
ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
ax.set_title("(b) Plano XZ"); ax.legend()

# Error vs progreso
ax = fig.add_subplot(gs[2])
ax.plot(s_near, ep, color=RED, lw=0.8, alpha=0.7)
ax.axhline(np.sqrt(np.mean(ep**2)), color=RED, ls="--", lw=1.2,
           label=f"RMSE={np.sqrt(np.mean(ep**2)):.4f} m")
ax.set_xlabel("s [m]"); ax.set_ylabel("|e_p| [m]")
ax.set_title("(c) Error vs arco"); ax.legend()

fig.suptitle(f"Tracking de trayectoria — MHE-MPCC, $v_{{\\theta,\\max}}={vel}$ m/s",
             fontsize=12, y=1.02)
savefig(fig, "11_paper_trajectory.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Estimated states (from MHE output)
# ══════════════════════════════════════════════════════════════════════════════
px_h = d["px_hat"]; py_h = d["py_hat"]; pz_h = d["pz_hat"]
vx_h = d["vx_hat"]; vy_h = d["vy_hat"]; vz_h = d["vz_hat"]
qw_h = d["qw_hat"]; qx_h = d["qx_hat"]; qy_h = d["qy_hat"]; qz_h = d["qz_hat"]
wx_h = d["wx_hat"]; wy_h = d["wy_hat"]; wz_h = d["wz_hat"]

# Mask for converged MHE (skip initialization transient, sigma < 1.0)
# The first few MHE solves may have wild estimates before the window fills
conv_mask = sigma_k < 1.0   # conservative: allows full comparison

# For velocity (can have large initial spike), clip outliers for display
V_CLIP = 20.0   # m/s — physical max is ~15 m/s at v=10

# ══════════════════════════════════════════════════════════════════════════════
# Fig 12 — Real vs Estimado: Posición y Velocidad
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex=True)
fig.suptitle(r"MHE — Estado estimado vs real: Posición y Velocidad", fontsize=12)

pairs_p = [(px, px_h, "x"), (py, py_h, "y"), (pz, pz_h, "z")]
pairs_v = [(vx, vx_h, "x"), (vy, vy_h, "y"), (vz, vz_h, "z")]

for col, (real, est, lbl) in enumerate(pairs_p):
    ax = axes[0, col]
    ax.plot(t, real, color=BLUE,   lw=1.3, label=f"$p_{lbl}$ real")
    ax.plot(t, est,  color=ORANGE, lw=1.0, ls="--", label=f"$\\hat{{p}}_{lbl}$ MHE")
    rmse = np.sqrt(np.mean((real[conv_mask] - est[conv_mask])**2))
    ax.set_ylabel(f"$p_{lbl}$ [m]")
    ax.set_title(f"$p_{lbl}$ — RMSE={rmse:.4f} m (conv.)")
    ax.legend(loc="upper right", ncol=2)

for col, (real, est, lbl) in enumerate(pairs_v):
    ax = axes[1, col]
    est_clipped = np.clip(est, -V_CLIP, V_CLIP)
    ax.plot(t, real, color=BLUE,   lw=1.3, label=f"$v_{lbl}$ real")
    ax.plot(t, est_clipped, color=ORANGE, lw=1.0, ls="--", label=f"$\\hat{{v}}_{lbl}$ MHE")
    rmse = np.sqrt(np.mean((real[conv_mask] - est[conv_mask])**2))
    ax.set_ylabel(f"$v_{lbl}$ [m/s]")
    ax.set_title(f"$v_{lbl}$ — RMSE={rmse:.4f} m/s (conv.)")
    ax.set_ylim([-V_CLIP, V_CLIP])
    ax.set_xlabel("Tiempo [s]")
    ax.legend(loc="upper right", ncol=2)

fig.tight_layout()
savefig(fig, "12_state_pos_vel_comparison.pdf")

# ══════════════════════════════════════════════════════════════════════════════
# Fig 13 — Real vs Estimado: Velocidad angular y Cuaternión
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 4, figsize=(15, 6), sharex=True)
fig.suptitle(r"MHE — Estado estimado vs real: Velocidad angular y Cuaternión", fontsize=12)

# Row 0: quaternion components
pairs_q = [
    (qw, qw_h, "w"),
    (qx_d, qx_h, "x"),
    (qy_d, qy_h, "y"),
    (qz_d, qz_h, "z"),
]
for col, (real, est, lbl) in enumerate(pairs_q):
    ax = axes[0, col]
    ax.plot(t, real, color=BLUE,   lw=1.3, label=f"$q_{lbl}$ real")
    ax.plot(t, est,  color=ORANGE, lw=1.0, ls="--", label=f"$\\hat{{q}}_{lbl}$ MHE")
    rmse = np.sqrt(np.mean((real[conv_mask] - est[conv_mask])**2))
    ax.set_ylabel(f"$q_{lbl}$ [—]")
    ax.set_title(f"$q_{lbl}$ — RMSE={rmse:.5f}")
    ax.legend(loc="upper right", ncol=2, fontsize=7)

# Row 1: angular velocity (3 panels, last panel = norm error)
pairs_w = [(wx, wx_h, "x"), (wy, wy_h, "y"), (wz, wz_h, "z")]
for col, (real, est, lbl) in enumerate(pairs_w):
    ax = axes[1, col]
    ax.plot(t, real, color=BLUE,   lw=1.3, label=f"$\\omega_{lbl}$ real")
    ax.plot(t, est,  color=ORANGE, lw=1.0, ls="--", label=f"$\\hat{{\\omega}}_{lbl}$ MHE")
    rmse = np.sqrt(np.mean((real[conv_mask] - est[conv_mask])**2))
    ax.set_ylabel(f"$\\omega_{lbl}$ [rad/s]")
    ax.set_title(f"$\\omega_{lbl}$ — RMSE={rmse:.4f} rad/s")
    ax.set_xlabel("Tiempo [s]")
    ax.legend(loc="upper right", ncol=2, fontsize=7)

# Last panel: total estimation error norm (angular velocity)
ax = axes[1, 3]
err_w = np.sqrt((wx-wx_h)**2 + (wy-wy_h)**2 + (wz-wz_h)**2)
err_w_clipped = np.clip(err_w, 0, 10.0)
ax.plot(t, err_w_clipped, color=RED, lw=1.0)
err_conv = err_w[conv_mask].mean()
ax.axhline(err_conv, color=RED, ls="--", lw=1.2,
           label=f"avg={err_conv:.4f} rad/s (conv.)")
ax.set_ylabel(r"$\|\omega - \hat{\omega}\|$ [rad/s]")
ax.set_title(r"Error $\omega$ (norma)")
ax.set_xlabel("Tiempo [s]")
ax.legend()

fig.tight_layout()
savefig(fig, "13_state_quat_omega_comparison.pdf")

print(f"\nGeneradas {13} figuras en: {FIG_DIR}/")
print(f"\nResumen:")
print(f"  t_total    = {t[-1]:.2f} s")
print(f"  s_max      = {s_near.max():.1f} m")
print(f"  RMSE pos   = {np.sqrt(np.mean(ep**2)):.4f} m")
print(f"  m_hat      : {m_hat[0]:.3f} → {m_hat[-1]:.3f} kg  (Δ={abs(m_hat[-1]-1.08)*1000:.1f} g de error)")
print(f"  sigma_k    : {sigma_k[0]:.3e} → {sigma_k[-1]:.3e}  (÷{sigma_k[0]/sigma_k[-1]:.0f}x)")
print(f"  W_s        : {W_s[0]:.3f} → {W_s[-1]:.3f}  (max={W_s_max:.2f})")
print(f"  MPCC avg   = {mpcc_ms.mean():.2f} ms  max={mpcc_ms.max():.2f} ms")
print(f"  MHE  avg   = {mhe_ms.mean():.2f} ms  max={mhe_ms.max():.2f} ms")
print(f"  Loop avg   = {loop_ms.mean():.2f} ms  max={loop_ms.max():.2f} ms")

# State estimation quality
print(f"\n  Estado estimado vs real (tras convergencia σ<1.0):")
cm = conv_mask
print(f"    pos RMSE : px={np.sqrt(np.mean((px[cm]-px_h[cm])**2)):.4f}  py={np.sqrt(np.mean((py[cm]-py_h[cm])**2)):.4f}  pz={np.sqrt(np.mean((pz[cm]-pz_h[cm])**2)):.4f} m")
print(f"    vel RMSE : vx={np.sqrt(np.mean((vx[cm]-vx_h[cm])**2)):.4f}  vy={np.sqrt(np.mean((vy[cm]-vy_h[cm])**2)):.4f}  vz={np.sqrt(np.mean((vz[cm]-vz_h[cm])**2)):.4f} m/s")
print(f"    omega RMSE: wx={np.sqrt(np.mean((wx[cm]-wx_h[cm])**2)):.4f}  wy={np.sqrt(np.mean((wy[cm]-wy_h[cm])**2)):.4f}  wz={np.sqrt(np.mean((wz[cm]-wz_h[cm])**2)):.4f} rad/s")
