# Observability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors

A quadrotor rarely carries a force sensor, yet many tasks — flying in wind,
carrying a payload — need the external force on the airframe. This repository
turns the onboard **IMU into a virtual force sensor**: the external specific
force is read directly from the accelerometer residual of a moving-horizon
estimator (MHE).

The obstacle is calibration. The residual gives force only if the vehicle mass
is known, and near hover the mass is *unobservable* — a vertical force and a
mass change look identical to the accelerometer. A plain filter silently
corrupts the mass there. We make the estimator **observability-aware and
self-calibrating**:

- an online **Fisher-information measure** on the mass gates the calibration and
  **freezes the mass when it cannot be seen**;
- when the flight lacks the excitation to recalibrate, the controller injects a
  **minimal information-optimal probe** shaped by the same measure.

The sensed force is then fed to a model-predictive controller as a feedforward
term, which improves tracking under wind.

> Companion code for the paper of the same title. All results below are
> simulation (MuJoCo software-in-the-loop) at 100 Hz.

---

## Results at a glance

| Claim | Result |
|---|---|
| Force read from IMU residual vs. MuJoCo ground truth | per-axis correlation **0.93** under continuous wind |
| Gate holds mass calibration through a windy hover | **3%** error, vs **7%** drift for an ungated EKF |
| Probe recovers mass from a wrong prior | identifies mass that hover alone cannot |
| Sensed force → NMPC feedforward (tracking error) | **34–80%** reduction under wind, largest where wind dominates |
| Robustness | degrades gracefully under injected accelerometer / thrust errors |
| Real time | full stack runs at **100 Hz** |

Force estimate is weakest on the vertical axis — the mass–vertical-force
ambiguity is fundamental (Proposition 1 in the paper), not an implementation
gap.

---

## Repository layout

```
src/
  mhe/          Lie-invariant MHE (18-state, thrust-as-input) + EKF baseline
  nmpc/         adaptive NMPC + nmpc_mhe_sil (main SiL driver)
  mpcc/         MPCC controller (legacy contour-tracking variant)
  mujoco/       MuJoCo SiL interface + reset/verify protocol
  trajectory/   Lissajous path, attitude reference
  common/       quaternion algebra, params
ocp_generation/ acados OCP generators (regenerate only if the formulation changes)
scripts/        batch runners (batch_*.sh) + plotting (plot_*.py)
paper/          LaTeX sources (RA-L + IEEE Access versions)
results/        CSV logs and figures (gitignored)
```

The estimator and controller are **decoupled**: the MHE estimates state, the
external disturbance `d̂`, and the mass; a separate law handles actuator
constants. This is a deliberate design choice, not a limitation.

---

## Build

Requires ROS 2 Humble, [acados](https://github.com/acados/acados), Eigen, and
the MuJoCo simulation workspace (`~/mujoco_ws`).

```bash
cd build
source /opt/ros/humble/setup.bash
source ~/mujoco_ws/install/setup.bash
cmake --build . --target nmpc_mhe_sil -j$(nproc)
```

To regenerate the acados solvers (only after changing the OCP formulation):

```bash
cd ocp_generation
python3 generate_mhe_ocp.py     # → c_generated_code_mhe*
python3 generate_mpcc_ocp.py    # → c_generated_code_mpcc
cd ../build && cmake .. && cmake --build . -j$(nproc)
```

---

## Reproducing the results

### 1. Start the simulator

In a separate terminal, launch the noisy MuJoCo scene and the wind node that
publishes the force ground truth on `/quadrotor/external_force`:

```bash
MUJOCO_ODOM_NOISE=1 sim_gate_collideroff
```

### 2. Run the virtual sensor (single runs)

From `build/`:

```bash
./nmpc_mhe_sil v2 60 w=2.0          # force sensing (known mass 1.05 kg, clean d)
./nmpc_mhe_sil v2 60 w=2.0 ff       # + feedforward d̂ → NMPC (disturbance rejection)
./nmpc_mhe_sil v2 60 w=2.0 idmass   # start mass at 0.60 kg → identification demo
./nmpc_mhe_sil v2 60 hover ff       # windy hover (maximum rejection benefit)
```

`v1` = open loop (NMPC uses a fixed wrong mass), `v2` = closed loop (NMPC
corrects via the MHE). Because the NMPC tracks a fixed time reference, any
tracking change is attributable to the estimator alone — a clean ablation.

CSVs land in `results/`.

### 3. Reproduce the paper figures (batteries + plots)

Each battery is one experiment group; each has a plotting script:

```bash
scripts/batch_rejection.sh      # speeds × {ff, baseline} × N reps  → fig_rejection
scripts/batch_active.sh         # probe vs no-probe mass recovery    → fig_active
scripts/batch_probe_sweep.sh    # information-optimal probe shape     → fig_probe_opt
scripts/batch_robust.sh         # accel/thrust error sweep            → fig_robust
scripts/batch_trans.sh          # aggressive→hover, gated vs plain EKF → fig_trans

# individual figures from one CSV
python3 scripts/plot_paper_figures.py {filtering|force|params} <csv> [out] [title]
```

If a reset misbehaves (a known MuJoCo quirk), `python3 scripts/reset_verify.py`
does a robust reset-and-verify.

---

## How the sensed force is used in the NMPC

The MHE runs at 100 Hz and outputs the external disturbance `d̂` (external
specific force, m/s²) together with `σ_k`, the trace of the parameter
covariance — an online confidence signal.

1. `d̂` is smoothed by an EMA and **gated**: it is only injected while
   `σ_k < 0.05`, so the controller never acts on an unconverged estimate.
2. The gated `d̂` enters the NMPC dynamics as a **feedforward** term in the
   translational acceleration:

   ```
   v̇ = (T / m̂) · R e₃ + d̂
   ```

   The controller anticipates the disturbance instead of only reacting to the
   tracking error it causes. This is what turns a wind gust from a tracking
   error into a modeled input, and is the source of the 34–80% improvement.

Enable it at runtime with the `ff` flag; leave it off for the open-loop
baseline. No recompilation needed to switch — mass, `d̂`, and `k̂_τ = 1/τ̂` are
runtime parameters of the solver.

---

## Notes

- Everything here is software-in-the-loop against MuJoCo. Hardware validation is
  future work.
- The "real" mass is **1.05 kg** (sum of the geom masses in the MuJoCo XML).
- `results/` and figure folders are gitignored; regenerate them with the
  commands above.

## Citing

If this code is useful, please cite the accompanying paper (see `paper/`).

## License

See repository. Author: Bryan Guevara.
