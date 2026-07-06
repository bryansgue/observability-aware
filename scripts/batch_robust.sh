#!/bin/bash
# batch_robust.sh — virtual-sensor robustness to real-sensor errors.
# Sweeps accelerometer bias, accelerometer scale-factor, and thrust-coefficient
# mismatch (one at a time) and logs the force estimate + mass under each, to show
# the virtual sensor degrades gracefully — the dominant "sim-only" objection.
# A w=1.0 maneuver (mass observable) with a deterministic ~2 N perturbation
# (force ground truth present). PREREQ: PLAIN sim. NO set -u.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/../build"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
export EKF_GATE_THR=0.009

RES=../results
REPS=${1:-2}
TRUN=30
SPEED=1.0
FEXT=1.5
MINSAMP=2200
SRC="$RES/nmpc_mhe_v2_w$SPEED.csv"

"$SCRIPT_DIR/sim_up.sh" || { echo "[robust] sim_up failed"; exit 1; }

# tag  ACCEL_SCALE  ACCEL_BIAS  THRUST_SCALE
CONFIGS=(
  "base   1.00 0.00 1.00"
  "bias1  1.00 0.10 1.00"
  "bias2  1.00 0.20 1.00"
  "bias4  1.00 0.40 1.00"
  "scl2   1.02 0.00 1.00"
  "scl5   1.05 0.00 1.00"
  "scl10  1.10 0.00 1.00"
  "thr2   1.00 0.00 1.02"
  "thr5   1.00 0.00 1.05"
  "thr10  1.00 0.00 1.10"
)

for cfg in "${CONFIGS[@]}"; do
  set -- $cfg; tag=$1; ascl=$2; abias=$3; tscl=$4
  for rep in $(seq 1 $REPS); do
    for attempt in 1 2 3; do
      ACCEL_SCALE=$ascl ACCEL_BIAS=$abias THRUST_SCALE=$tscl \
        timeout $((TRUN+30)) ./nmpc_mhe_sil v2 $TRUN w=$SPEED fext=$FEXT \
        >/tmp/robust_${tag}_$rep.log 2>&1
      n=$(wc -l < "$SRC" 2>/dev/null || echo 0)
      if [ "$n" -ge "$MINSAMP" ]; then
        cp "$SRC" "$RES/robust_${tag}_seed${rep}.csv"
        echo "robust $tag (scl=$ascl bias=$abias thr=$tscl) seed $rep: OK"
        break
      fi
      echo "robust $tag seed $rep: bad ($n) retry $attempt"
    done
  done
done
echo "=== ROBUST SWEEP DONE ==="
echo "Now: python3 ../scripts/analyze_robust.py"
