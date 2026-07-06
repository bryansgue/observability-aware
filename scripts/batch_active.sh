#!/bin/bash
# batch_active.sh — N seeded active-excitation runs for fig_active.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# From a wrong mass prior (0.70 kg) in hover, under a DETERMINISTIC external force
# injected by the binary (fext=):
#   passive (no probe): mass unobservable -> frozen at prior.
#   active  (probe):    controller excites -> mass observable -> identified.
# Different MuJoCo-noise realizations per run give the N-run spread.
#
# PREREQ (user): PLAIN sim, NO wind arg:
#   MUJOCO_ODOM_NOISE=1 sim_gate_collideroff
# NO `set -u`.
cd "$(dirname "$0")/../build"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
export EKF_GATE_THR=0.009   # normalized sigma_tilde_min (paper eq. snorm)

RES=../results
REPS=${1:-5}
TRUN=35
M0=0.70
# fext=0: the active-excitation demo is about OBSERVABILITY, not rejection. In
# clean hover the mass is genuinely unobservable, so the passive filter stays
# frozen at the wrong prior; the probe is what restores observability. A strong
# disturbance would tilt the vehicle, raise sigma_tilde and spuriously open the
# gate — defeating the very point of the figure.
FEXT=0
MINSAMP=2600
SRC="$RES/nmpc_mhe_v2.csv"

# Sim-launch protocol: ensure exactly one healthy sim (reuse or relaunch).
"$SCRIPT_DIR/sim_up.sh" || { echo "[batch_active] sim_up failed — abort"; exit 1; }

run_one() {  # $1=tag(noprobe|probe) $2=rep $3=extra-arg
  local tag=$1 rep=$2 extra=$3
  for attempt in 1 2 3; do
    timeout $((TRUN+30)) ./nmpc_mhe_sil v2 $TRUN hover m0=$M0 fext=$FEXT $extra \
      >/tmp/active_${tag}_$rep.log 2>&1
    local n=$(wc -l < "$SRC" 2>/dev/null || echo 0)
    if [ "$n" -ge "$MINSAMP" ]; then
      cp "$SRC" "$RES/active_${tag}_seed${rep}.csv"
      echo "active $tag seed $rep: OK ($n samples)"
      return
    fi
    echo "active $tag seed $rep: bad (samples=$n) — retry $attempt"
  done
  echo "active $tag seed $rep: FAILED"
}

for rep in $(seq 1 $REPS); do run_one noprobe "$rep" ""; done
for rep in $(seq 1 $REPS); do run_one probe   "$rep" "probe"; done
echo "=== ACTIVE BATTERY DONE ($REPS reps each) ==="
echo "Now: python3 ../scripts/plot_active.py"
