# Observability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors

A quadrotor rarely carries a force sensor, but flying in wind or carrying a
payload needs the external force. This code turns the onboard **IMU into a
virtual force sensor**: the external specific force is read from the
accelerometer residual of a moving-horizon estimator (MHE).

The catch is calibration — the residual gives force only if the mass is known,
and near hover the mass is *unobservable* (a vertical force and a mass change
look identical). So the estimator is **observability-aware and self-calibrating**:

- a **Fisher-information measure** on the mass gates the calibration and
  **freezes the mass when it cannot be seen**;
- when the flight lacks excitation, the controller injects a **minimal
  information-optimal probe**.

The sensed force is fed to an NMPC as feedforward, improving tracking under wind.
All results are MuJoCo software-in-the-loop at 100 Hz.

---

## Results

| Claim | Result |
|---|---|
| Force from IMU vs. ground truth | per-axis correlation **0.93** under wind |
| Gate holds mass through windy hover | **3%** error vs **7%** for an ungated EKF |
| Probe recovers mass from a wrong prior | identifies what hover alone cannot |
| Sensed force → NMPC feedforward | **34–80%** less tracking error under wind |

Force estimate + ground truth | Disturbance rejection (NMPC feedforward)
:---:|:---:
![force](docs/figs/fig_force_corr.png) | ![rejection](docs/figs/fig_rejection.png)
**Gated vs. plain EKF** (hover mass) | **Probe recovers the mass**
![trans](docs/figs/fig_trans.png) | ![active](docs/figs/fig_active.png)

---

## Build

Needs ROS 2 Humble, [acados](https://github.com/acados/acados), Eigen, and the
MuJoCo workspace (`~/mujoco_ws`).

```bash
cd build
source /opt/ros/humble/setup.bash
source ~/mujoco_ws/install/setup.bash
cmake --build . --target nmpc_mhe_sil -j$(nproc)
```

## Reproduce

Start the noisy scene + wind node (publishes ground truth on
`/quadrotor/external_force`) in another terminal:

```bash
MUJOCO_ODOM_NOISE=1 sim_gate_collideroff
```

Single runs (from `build/`):

```bash
./nmpc_mhe_sil v2 60 w=2.0          # force sensing (known mass)
./nmpc_mhe_sil v2 60 w=2.0 ff       # + feedforward d̂ → NMPC (rejection)
./nmpc_mhe_sil v2 60 w=2.0 idmass   # wrong mass prior → identification
./nmpc_mhe_sil v2 60 hover ff       # windy hover (max rejection)
```

Paper figures (each battery = one experiment group):

```bash
scripts/batch_rejection.sh   # speeds × {ff, baseline}    → fig_rejection
scripts/batch_active.sh      # probe vs no-probe recovery  → fig_active
scripts/batch_probe_sweep.sh # information-optimal probe    → fig_probe_opt
scripts/batch_robust.sh      # accel/thrust error sweep     → fig_robust
scripts/batch_trans.sh       # gated vs plain EKF in hover  → fig_trans
```

`v1` = open loop (fixed wrong mass), `v2` = closed loop. The NMPC tracks a fixed
time reference, so any tracking change is the estimator alone — a clean ablation.

## How the NMPC uses it

The MHE outputs the disturbance `d̂` (100 Hz) plus `σ_k` (confidence). `d̂` is
EMA-smoothed and **gated** (injected only while `σ_k < 0.05`), then enters the
NMPC dynamics as feedforward:

```
v̇ = (T / m̂) · R e₃ + d̂
```

The controller anticipates the wind instead of only reacting to it. Enable with
the `ff` flag; mass, `d̂`, and `k̂_τ` are runtime solver parameters (no rebuild).

## Layout

```
src/mhe/        Lie-invariant MHE + EKF baseline
src/nmpc/       adaptive NMPC + nmpc_mhe_sil (main driver)
src/mujoco/     MuJoCo SiL interface + reset/verify
scripts/        batch_*.sh runners + plot_*.py
paper/          LaTeX sources
```

Everything is SiL; hardware validation is future work.
