#!/usr/bin/env python3
"""
plot_system.py — system architecture schematic (fig_system.png, paper Fig. 1).

Block diagram of the observability-aware loop: the quadrotor's sensors feed the
EKF, which produces the state, the mass m_hat, the force d_hat, and the online
identifiability sigma. sigma gates the mass update inside the EKF and triggers the
active probe in the NMPC; the NMPC feeds back thrust and body-rate commands, with
d_hat injected as a feedforward.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams.update({"font.family": "serif", "font.size": 10, "savefig.bbox": "tight"})
RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")


def box(ax, xy, w, h, text, fc):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.04",
                                linewidth=1.3, edgecolor="#333", facecolor=fc, zorder=2))
    ax.text(x + w/2, y + h/2, text, ha="center", va="center", zorder=3, fontsize=9.5)


def arrow(ax, p0, p1, color="#333", style="-|>", ls="-", rad=0.0, lw=1.3):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=12,
                                 lw=lw, color=color, linestyle=ls,
                                 connectionstyle=f"arc3,rad={rad}", zorder=1))


def main():
    out = os.path.join(RES, "fig_system.png")
    fig, ax = plt.subplots(figsize=(7.4, 3.0))
    ax.set_xlim(0, 13.4); ax.set_ylim(0, 4.4); ax.axis("off")

    yb, hb = 1.5, 1.5            # box bottom, height (tops at 3.0)
    box(ax, (0.2, yb), 2.0, hb, "Quadrotor\n(MuJoCo)", "#eaeaea")
    box(ax, (3.0, yb), 2.0, hb, "Odometry\n+ IMU", "#eaeaea")
    box(ax, (6.0, yb), 2.7, hb,
        "Observability-\naware EKF\n" + r"[$\tilde\sigma$ gate on $\hat m$]", "#d6e4f0")
    box(ax, (10.2, yb), 2.7, hb,
        "NMPC\n+ active probe\n" + r"[$\hat{d}$ feedforward]", "#d8efd8")
    ym = yb + hb/2              # 2.25, arrow mid-height

    # forward path
    arrow(ax, (2.2, ym), (3.0, ym))                   # drone -> sensors
    arrow(ax, (5.0, ym), (6.0, ym))                   # sensors -> EKF
    ax.text(5.5, ym + 0.22, r"$y,s$", ha="center", fontsize=9)
    arrow(ax, (8.7, ym), (10.2, ym))                  # EKF -> NMPC
    ax.text(9.45, ym + 0.22, r"$\hat{x},\hat m,\hat{d}$", ha="center", fontsize=9)

    # command feedback (top loop, above boxes)
    arrow(ax, (11.55, 3.0), (11.55, 3.7))
    arrow(ax, (11.55, 3.7), (1.2, 3.7))
    arrow(ax, (1.2, 3.7), (1.2, 3.0))
    ax.text(6.3, 3.88, r"thrust $T$, body-rate $\omega_{\mathrm{cmd}}$", ha="center", fontsize=9)

    # sigma signal: EKF -> NMPC probe (bottom dashed)
    arrow(ax, (7.35, yb), (7.35, 0.65), color="#b9770e", ls="--")
    arrow(ax, (7.35, 0.65), (11.55, 0.65), color="#b9770e", ls="--")
    arrow(ax, (11.55, 0.65), (11.55, yb), color="#b9770e", ls="--")
    ax.text(9.45, 0.42, r"$\tilde\sigma$: gate $\to$ probe trigger",
            ha="center", fontsize=9, color="#b9770e")

    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
