#!/bin/bash
# batch_trans.sh — N seeded "trans" runs (aggressive -> hover) for fig_trans.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# Identify the mass under excitation, then hover under a DETERMINISTIC external
# force injected by the binary itself (fext=) on /external_force_cmd. The plain
# EKF drifts in hover (vertical force misread as mass, Prop. 1); the gated EKF
# freezes it. Different MuJoCo-noise realizations per run give the N-run spread.
#
# PREREQ (user, separate terminal — PLAIN sim, NO wind arg):
#   MUJOCO_ODOM_NOISE=1 sim_gate_collideroff
# The binary injects the perturbation; it sends ZERO force during RELOAD so the
# reset settles cleanly. NO wind node, no leaks.
# NO `set -u`: ROS setup.bash references unbound vars.
cd "$(dirname "$0")/../build"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
export EKF_GATE_THR=0.009   # normalized sigma_tilde_min (paper eq. snorm)
export ENERGY_GATE=1000   # PE-proxy baseline: generic regressor-energy dead-zone (paper Sec. Results)

RES=../results
REPS=${1:-5}
TRUN=40            # s — half aggressive, half hover
SPEED=1.0          # liss.w (peak ~6 m/s — gentle decel for a clean freeze)
FEXT=1.5           # deterministic perturbation amplitude [N] (|d|~2 N)
MINSAMP=3000
SRC="$RES/nmpc_mhe_v2_w$SPEED.csv"

# Sim-launch protocol: ensure exactly one healthy sim (reuse or relaunch).
"$SCRIPT_DIR/sim_up.sh" || { echo "[batch_transpe] sim_up failed — abort"; exit 1; }

# Verify the perturbation GT was actually applied (|ext_force|>0.5 N on >50 samples).
gt_applied() { awk -F, 'NR>1 && ($29*$29+$30*$30+$31*$31)>0.25{c++} END{exit (c>50)?0:1}' "$1"; }

run_one() {  # $1=rep
  local rep=$1
  for attempt in 1 2 3; do
    timeout $((TRUN+30)) ./nmpc_mhe_sil v2 $TRUN w=$SPEED trans fext=$FEXT \
      >/tmp/transpe_$rep.log 2>&1
    local n=$(wc -l < "$SRC" 2>/dev/null || echo 0)
    if [ "$n" -ge "$MINSAMP" ] && gt_applied "$SRC"; then
      cp "$SRC" "$RES/transpe_seed${rep}.csv"
      echo "trans seed $rep: OK ($n samples, GT applied)"
      return
    fi
    echo "trans seed $rep: bad (samples=$n, GT=$(gt_applied "$SRC" && echo ok || echo MISSING)) — retry $attempt"
  done
  echo "trans seed $rep: FAILED"
}

for rep in $(seq 1 $REPS); do run_one "$rep"; done
echo "=== TRANS-PE BATTERY DONE ($REPS reps) ==="
echo "Now: python3 ../scripts/plot_trans.py"
