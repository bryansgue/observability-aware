#!/bin/bash
# batch_probe_sweep.sh — information-optimal excitation study.
# Sweeps the active-probe shape (vertical bob vs lateral tilt) at a FIXED displacement
# budget. The vertical/lateral mix that maximizes the achieved sigma_tilde (= minimizes
# the mass CRB) per unit displacement is the information-optimal probe. For each mix we
# run the hover recovery from a wrong prior (0.70 kg) and log sigma_tilde + final mass.
# PREREQ: PLAIN sim. NO set -u.
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
REPS=${1:-5}
export PROBE_OPEN_TIME=15
TRUN=50
M0=0.70
MINSAMP=2600
SRC="$RES/nmpc_mhe_v2.csv"

"$SCRIPT_DIR/sim_up.sh" || { echo "[probe_sweep] sim_up failed"; exit 1; }

# (tag, az, al) — fixed displacement budget sqrt(az^2+al^2) ~ 0.40 m, theta sweep.
CONFIGS=(
  "lateral 0.00 0.80"
  "mix30   0.40 0.69"
  "mix45   0.57 0.57"
  "mix60   0.69 0.40"   # ~ the original fixed probe
  "vert    0.80 0.00"
)

for cfg in "${CONFIGS[@]}"; do
  set -- $cfg; tag=$1; az=$2; al=$3
  for rep in $(seq 1 $REPS); do
    for attempt in 1 2 3; do
      timeout $((TRUN+30)) ./nmpc_mhe_sil v2 $TRUN hover m0=$M0 fext=0 probe \
        probe_az=$az probe_al=$al >/tmp/probe_${tag}_$rep.log 2>&1
      n=$(wc -l < "$SRC" 2>/dev/null || echo 0)
      if [ "$n" -ge "$MINSAMP" ]; then
        cp "$SRC" "$RES/probe08_${tag}_seed${rep}.csv"
        echo "probe $tag (az=$az al=$al) seed $rep: OK"
        break
      fi
      echo "probe $tag seed $rep: bad ($n) retry $attempt"
    done
  done
done
echo "=== PROBE SWEEP DONE ==="
echo "Now: python3 ../scripts/analyze_probe_sweep.py"
