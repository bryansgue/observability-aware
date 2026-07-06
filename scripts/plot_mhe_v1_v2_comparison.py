#!/usr/bin/env python3
"""
plot_mhe_v1_v2_comparison.py — Comparación MHE-MPCC v1 (open-loop) vs v2 (closed-loop).

Escenario: ambos modos parten con masa incorrecta m_nom=0.9 kg (verdadera=1.08 kg, error 17%).
  v1: MPCC siempre usa m=0.9 kg — nunca corrige.
  v2: MHE estima m̂ → 1.08 kg y lo realimenta al MPCC cuando σ_k < 0.05.

Uso:
    python3 plot_mhe_v1_v2_comparison.py [vtheta_max]
    python3 plot_mhe_v1_v2_comparison.py 10

Genera figuras en: figures_mhe_comparison/v{X}/
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

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

vel = int(sys.argv[1]) if len(sys.argv) > 1 else 10

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR   = os.path.join(SCRIPT_DIR, "..")
FIG_DIR    = os.path.join(BASE_DIR, "figures_mhe_comparison", f"v{vel}")
os.makedirs(FIG_DIR, exist_ok=True)

# ── Carga de datos ─────────────────────────────────────────────────────────────
def load(mode):
    path = os.path.join(BASE_DIR, "results", f"mhe_mpcc_{mode}_v{vel}.csv")
    if not os.path.exists(path):
        print(f"ERROR: {path} no encontrado — corre primero ./mhe_mpcc_sil {vel} {mode}")
        sys.exit(1)
    return np.genfromtxt(path, delimiter=",", names=True)

d1 = load("v1")
d2 = load("v2")

t1, t2 = d1["t"], d2["t"]

# Error de posición
ep1 = np.sqrt((d1["px"]-d1["px_ref"])**2 + (d1["py"]-d1["py_ref"])**2 + (d1["pz"]-d1["pz_ref"])**2)
ep2 = np.sqrt((d2["px"]-d2["px_ref"])**2 + (d2["py"]-d2["py_ref"])**2 + (d2["pz"]-d2["pz_ref"])**2)

rmse1 = np.sqrt(np.mean(ep1**2))
rmse2 = np.sqrt(np.mean(ep2**2))
mejora = (rmse1 - rmse2) / rmse1 * 100

print(f"v1: RMSE={rmse1:.4f} m  max={ep1.max():.4f} m")
print(f"v2: RMSE={rmse2:.4f} m  max={ep2.max():.4f} m")
print(f"Mejora: {mejora:.1f}%")

def savefig(fig, name):
    path = os.path.join(FIG_DIR, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  → {path}")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 1 — Masa en MPCC: v1 fija en 0.9 vs v2 corrige a 1.08
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(12, 4))

# Masa estimada por MHE (ambos la estiman igual — la diferencia es qué le pasan al MPCC)
ax = axes[0]
ax.plot(t1, d1["m_hat"], color=RED,  lw=1.4, ls="--", label="v1: m̂ MHE (no se usa en MPCC)")
ax.plot(t2, d2["m_hat"], color=BLUE, lw=1.4,           label="v2: m̂ MHE → MPCC")
ax.axhline(1.08, color=GREEN, ls="--", lw=1.5, label=r"$m_{\rm true}=1.08$ kg")
ax.axhline(0.9,  color=GRAY,  ls=":",  lw=1.2, label=r"$m_{\rm nom}=0.90$ kg (v1 fija)")
# Mark σ_k threshold crossing in v2
sigma2 = d2["sigma_k"]
idx_conv = np.where(sigma2 < 0.05)[0]
if len(idx_conv) > 0:
    ax.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5,
               label=f"v2: σ_k<0.05 @ t={t2[idx_conv[0]]:.1f}s")
ax.set_xlabel("Tiempo [s]")
ax.set_ylabel("[kg]")
ax.set_title("(a) Masa en modelo MPCC")
ax.legend(fontsize=8)
ax.set_ylim([0.5, 1.3])

# σ_k (igual en ambos — solo depende del MHE)
ax = axes[1]
ax.semilogy(t2, d2["sigma_k"], color=PURPLE, lw=1.4, label=r"$\sigma_k = \mathrm{tr}(\bar{P}_{\theta,k})$")
if len(idx_conv) > 0:
    ax.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5,
               label=f"σ_k=0.05 @ t={t2[idx_conv[0]]:.1f}s\n→ v2 activa corrección")
ax.axhline(0.05, color=ORANGE, ls="--", lw=1.0, label="Umbral σ_k=0.05")
ax.set_xlabel("Tiempo [s]")
ax.set_ylabel(r"$\sigma_k$ [log]")
ax.set_title(r"(b) Incertidumbre paramétrica $\sigma_k$")
ax.legend(fontsize=8)

fig.suptitle(f"MHE — Convergencia de masa y activación de realimentación ($v_{{\\theta,\\max}}={vel}$ m/s)",
             fontsize=11)
fig.tight_layout()
savefig(fig, "01_mass_sigma.pdf")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 2 — Error de posición en el tiempo: v1 vs v2
# ══════════════════════════════════════════════════════════════════════════════
fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6), sharex=False)

# Error vs tiempo
a1.plot(t1, ep1, color=RED,  lw=0.9, alpha=0.8, label=f"v1 (masa fija 0.9 kg)  RMSE={rmse1:.4f} m")
a1.plot(t2, ep2, color=BLUE, lw=0.9, alpha=0.8, label=f"v2 (MHE+MPCC cerrado)  RMSE={rmse2:.4f} m")
a1.axhline(rmse1, color=RED,  ls="--", lw=1.2)
a1.axhline(rmse2, color=BLUE, ls="--", lw=1.2)
if len(idx_conv) > 0:
    a1.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5,
               label=f"v2: corrección activa @ t={t2[idx_conv[0]]:.1f}s")
a1.set_ylabel("|e_p| [m]")
a1.set_title(f"Error de posición: v1 vs v2  (mejora={mejora:.1f}%)")
a1.legend(fontsize=8)
a1.set_xlabel("Tiempo [s]")

# Diferencia de error |ep1 - ep2| > 0 significa que v2 es mejor
t_common = np.linspace(0, min(t1[-1], t2[-1]), min(len(t1), len(t2)))
ep1_r = np.interp(t_common, t1, ep1)
ep2_r = np.interp(t_common, t2, ep2)
diff = ep1_r - ep2_r   # positivo = v2 mejor

a2.fill_between(t_common, 0, diff,
                where=(diff > 0), color=BLUE,  alpha=0.4, label="v2 mejor")
a2.fill_between(t_common, 0, diff,
                where=(diff < 0), color=RED,   alpha=0.4, label="v1 mejor")
a2.axhline(0, color=GRAY, lw=0.8)
if len(idx_conv) > 0:
    a2.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5)
a2.set_ylabel(r"$|e_{p,v1}| - |e_{p,v2}|$ [m]")
a2.set_title("Diferencia de error (azul = v2 gana)")
a2.set_xlabel("Tiempo [s]")
a2.legend(fontsize=8)

fig.tight_layout()
savefig(fig, "02_position_error_comparison.pdf")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 3 — Trayectorias 2D: v1 vs v2 vs referencia
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(13, 5))

ax = axes[0]
ax.plot(d1["px_ref"], d1["py_ref"], "--", color=GRAY, lw=1.0, label="Ref γ(θ)")
ax.plot(d1["px"],     d1["py"],     color=RED,  lw=1.0, alpha=0.85,
        label=f"v1 (m=0.9 kg)  RMSE={rmse1:.3f} m")
ax.plot(d2["px"],     d2["py"],     color=BLUE, lw=1.0, alpha=0.85,
        label=f"v2 (MHE→MPCC)  RMSE={rmse2:.3f} m")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("(a) Plano XY")
ax.legend(fontsize=8)
ax.set_aspect("equal")

ax = axes[1]
ax.plot(d1["px_ref"], d1["pz_ref"], "--", color=GRAY, lw=1.0, label="Ref")
ax.plot(d1["px"],     d1["pz"],     color=RED,  lw=1.0, alpha=0.85, label="v1")
ax.plot(d2["px"],     d2["pz"],     color=BLUE, lw=1.0, alpha=0.85, label="v2")
ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
ax.set_title("(b) Plano XZ")
ax.legend(fontsize=8)

fig.suptitle(f"Trayectorias v1 vs v2 ($v_{{\\theta,\\max}}={vel}$ m/s, $m_{{\\rm nom}}=0.9$ kg, $m_{{\\rm true}}=1.08$ kg)",
             fontsize=11)
fig.tight_layout()
savefig(fig, "03_trajectories_comparison.pdf")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 4 — Ganancia adaptativa W_s: v1 vs v2
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(12, 4))

ax = axes[0]
ax.plot(t1, d1["W_s"], color=RED,  lw=1.4, label="v1")
ax.plot(t2, d2["W_s"], color=BLUE, lw=1.4, label="v2")
w_max = max(d1["W_s"].max(), d2["W_s"].max())
ax.axhline(w_max, color=GRAY, ls="--", lw=1.0, label=f"W_s_max={w_max:.1f}")
ax.set_xlabel("Tiempo [s]"); ax.set_ylabel(r"$W_s(k)$")
ax.set_title(r"(a) Ganancia adaptativa $W_s(k)$")
ax.legend()

ax = axes[1]
ax.plot(t1, d1["vtheta"], color=RED,  lw=1.2, label="v1")
ax.plot(t2, d2["vtheta"], color=BLUE, lw=1.2, label="v2")
ax.axhline(vel, color=GRAY, ls="--", lw=1.0, label=f"v_θ_max={vel} m/s")
ax.set_xlabel("Tiempo [s]"); ax.set_ylabel(r"$v_\theta$ [m/s]")
ax.set_title(r"(b) Velocidad de progreso $v_\theta$")
ax.legend()

fig.suptitle(f"Adaptivo W_s y progreso ($v_{{\\theta,\\max}}={vel}$ m/s)", fontsize=11)
fig.tight_layout()
savefig(fig, "04_Ws_vtheta_comparison.pdf")


# ══════════════════════════════════════════════════════════════════════════════
# Fig 5 — Panel de paper: compacto (3 paneles clave)
# ══════════════════════════════════════════════════════════════════════════════
fig = plt.figure(figsize=(14, 4))
gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.38)

# Panel (a): masa en MPCC
ax = fig.add_subplot(gs[0])
ax.plot(t2, d2["m_hat"], color=BLUE, lw=1.6, label=r"$\hat{m}$ MHE (v2)")
ax.axhline(1.08, color=GREEN, ls="--", lw=1.5, label=r"$m_{\rm true}=1.08$ kg")
ax.axhline(0.90, color=RED,   ls=":",  lw=1.5, label=r"$m_{\rm v1}=0.90$ kg (fija)")
if len(idx_conv) > 0:
    ax.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5,
               label=f"corrección @ t={t2[idx_conv[0]]:.1f}s")
ax.fill_between(t2, d2["m_hat"], 1.08, alpha=0.12, color=BLUE)
ax.set_xlabel("t [s]"); ax.set_ylabel("[kg]")
ax.set_title(r"(a) Masa estimada $\hat{m}(t)$")
ax.legend(fontsize=7.5); ax.set_ylim([0.6, 1.25])

# Panel (b): error posición
ax = fig.add_subplot(gs[1])
ax.plot(t1, ep1, color=RED,  lw=1.0, alpha=0.85,
        label=f"v1  RMSE={rmse1:.3f} m")
ax.plot(t2, ep2, color=BLUE, lw=1.0, alpha=0.85,
        label=f"v2  RMSE={rmse2:.3f} m")
ax.axhline(rmse1, color=RED,  ls="--", lw=1.0)
ax.axhline(rmse2, color=BLUE, ls="--", lw=1.0)
if len(idx_conv) > 0:
    ax.axvline(t2[idx_conv[0]], color=ORANGE, ls=":", lw=1.5)
ax.set_xlabel("t [s]"); ax.set_ylabel("|e_p| [m]")
ax.set_title(f"(b) Error posición (−{mejora:.0f}%)")
ax.legend(fontsize=8)

# Panel (c): trayectoria XY
ax = fig.add_subplot(gs[2])
ax.plot(d1["px_ref"], d1["py_ref"], "--", color=GRAY, lw=1.0, label="Ref")
ax.plot(d1["px"],     d1["py"],     color=RED,  lw=0.9, alpha=0.85, label="v1")
ax.plot(d2["px"],     d2["py"],     color=BLUE, lw=0.9, alpha=0.85, label="v2")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("(c) Trayectoria XY")
ax.legend(fontsize=8)
ax.set_aspect("equal")

fig.suptitle(
    rf"MHE+MPCC: open-loop v1 vs closed-loop v2 — $m_{{nom}}=0.90$ kg, $m_{{true}}=1.08$ kg, $v_{{\theta,\max}}={vel}$ m/s",
    fontsize=11, y=1.02
)
fig.tight_layout()
savefig(fig, "05_paper_v1_v2_comparison.pdf")


# ══════════════════════════════════════════════════════════════════════════════
# Resumen numérico
# ══════════════════════════════════════════════════════════════════════════════
print(f"\n{'='*55}")
print(f"  Comparación v1 vs v2 — vtheta_max={vel} m/s")
print(f"  Escenario: m_nom=0.9 kg, m_true=1.08 kg (error 17%)")
print(f"{'='*55}")
print(f"  {'Métrica':<25} {'v1':>10} {'v2':>10} {'Mejora':>10}")
print(f"  {'-'*55}")
print(f"  {'RMSE pos [m]':<25} {rmse1:>10.4f} {rmse2:>10.4f} {mejora:>9.1f}%")
print(f"  {'Max err pos [m]':<25} {ep1.max():>10.4f} {ep2.max():>10.4f} {(ep1.max()-ep2.max())/ep1.max()*100:>9.1f}%")
print(f"  {'m̂ final [kg]':<25} {'0.900':>10} {d2['m_hat'][-1]:>10.4f}  (true=1.080)")
print(f"  {'σ_k final':<25} {d1['sigma_k'][-1]:>10.3e} {d2['sigma_k'][-1]:>10.3e}")
print(f"  {'W_s final':<25} {d1['W_s'][-1]:>10.3f} {d2['W_s'][-1]:>10.3f}")
print(f"  {'Steps':<25} {len(t1):>10} {len(t2):>10}")
print(f"{'='*55}")
print(f"\nFiguras generadas en: {FIG_DIR}/")
