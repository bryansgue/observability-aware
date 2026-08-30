#!/usr/bin/env python3
"""
analyze_robust.py — virtual-sensor robustness to real-sensor errors (fig_robust.png).

For each injected error level (accelerometer bias, accelerometer scale-factor, thrust
mismatch) computes the force absolute error (N RMS, all axes) and the mass error, and
plots graceful-degradation curves. Shows the virtual sensor and its self-calibration
tolerate realistic sensor errors that the ideal simulation does not exercise.
"""
import os, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "font.size": 8, "axes.labelsize": 8,
    "legend.fontsize": 7.5, "figure.dpi": 200, "lines.linewidth": 1.8,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linestyle": "--", "savefig.bbox": "tight",
})
RES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
M_TRUE = 1.05
# config tag -> (sweep, x-value)
CFG = {"base": ("all", 0.0),
       "bias1": ("bias", 0.1), "bias2": ("bias", 0.2), "bias4": ("bias", 0.4),
       "scl2": ("scale", 2.0), "scl5": ("scale", 5.0), "scl10": ("scale", 10.0),
       "thr2": ("thrust", 2.0), "thr5": ("thrust", 5.0), "thr10": ("thrust", 10.0)}


def metrics(f):
    d = np.loadtxt(f, delimiter=",", skiprows=1)
    h = open(f).readline().strip().split(","); c = {n: i for i, n in enumerate(h)}
    m = d[:, c["t"]] > 5.0
    # force: estimate m*d_ekf [N] vs ground truth (theta/vtheta/a_theta)
    ferr = []
    for de, gt in [("dx_ekf", "theta"), ("dy_ekf", "vtheta"), ("dz_ekf", "a_theta")]:
        est = M_TRUE * d[m, c[de]]; g = d[m, c[gt]]
        ferr.append(np.sqrt(np.mean((est - g) ** 2)))
    fabs = float(np.mean(ferr))
    merr = 100.0 * abs(d[-1, c["m_ekf"]] - M_TRUE) / M_TRUE
    return fabs, merr


def collect():
    data = {}  # tag -> (fabs_mean, merr_mean)
    for tag in CFG:
        fs = sorted(glob.glob(os.path.join(RES, f"robust_{tag}_seed*.csv")))
        if not fs:
            continue
        fa, me = zip(*[metrics(f) for f in fs])
        data[tag] = (np.mean(fa), np.mean(me))
    return data


def main():
    data = collect()
    if "base" not in data:
        print("No robust_base CSVs."); return
    base_f, base_m = data["base"]
    print(f"{'cfg':>6} {'force[N]':>9} {'mass%':>7}")
    for tag, (fa, me) in data.items():
        print(f"{tag:>6} {fa:9.3f} {me:7.1f}")

    fig, axs = plt.subplots(3, 1, figsize=(3.5, 3.6))
    sweeps = [("bias", "accelerometer bias [m/s$^2$]"),
              ("scale", "accel scale-factor error [%]"),
              ("thrust", "thrust mismatch [%]")]
    for k, (ax, (sw, xlab)) in enumerate(zip(axs, sweeps)):
        ax.set_title(f"({chr(97+k)})", fontsize=8, loc="center", pad=1)
        xs = [0.0] + sorted(v for t, (s, v) in CFG.items() if s == sw and t in data)
        tags = ["base"] + [t for t, (s, v) in sorted(CFG.items(), key=lambda kv: kv[1][1])
                           if s == sw and t in data]
        fa = [data[t][0] for t in tags]; me = [data[t][1] for t in tags]
        ax.plot(xs, fa, "o-", color="#1f77b4", ms=3, lw=1.1, label="force error [N]")
        ax.set_xlabel(xlab); ax.set_ylabel("force abs. error [N]", color="#1f77b4")
        ax.tick_params(axis="y", labelcolor="#1f77b4")
        ax2 = ax.twinx()
        ax2.plot(xs, me, "s--", color="#c0392b", ms=3, lw=1.1, label="mass error [%]")
        ax2.set_ylabel("mass error [%]", color="#c0392b")
        ax2.tick_params(axis="y", labelcolor="#c0392b")
        ax2.grid(False)
        from matplotlib.ticker import MaxNLocator
        ax.yaxis.set_major_locator(MaxNLocator(4)); ax2.yaxis.set_major_locator(MaxNLocator(4)); ax.xaxis.set_major_locator(MaxNLocator(5))
        ax.set_ylabel("force error [N]", color="#1f77b4"); ax2.set_ylabel("mass error [%]", color="#c0392b")
    fig.tight_layout(h_pad=0.5)
    out = os.path.join(RES, "fig_robust.png")
    fig.savefig(out, dpi=300)
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
