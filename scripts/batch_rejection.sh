#!/bin/bash
# Robust rejection battery: speeds × {ff, baseline} × N reps.
# The binary's INTERNAL RELOAD does the verified reset (retries until z<0.15,qw>0.95
# and prints "RELOAD complete — verified clean"). We check completion via #samples.
# NO `set -u`: ROS setup.bash references unbound vars → would exit 1 on source.
cd "$(dirname "$0")/../build"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

RES=../results
REPS=5
# Combined MHE+EKF battery, dense ~every 2 m/s (peaks ~0,2,4,6,8,10 m/s).
# Each run logs both estimators (m_hat/d_hat and m_ekf/d_ekf) on the same stream.
# All 1-decimal so the %.1f CSV name has no rounding collision.
SPEEDS=(hover 0.3 0.6 1.0 1.4 1.8)
MINSAMP=1800

run_one() {  # $1=speed $2=mode $3=rep
  local w=$1 mode=$2 rep=$3 arg ffarg src
  [ "$w" = "hover" ] && arg="hover" || arg="w=$w"
  [ "$mode" = "ff" ] && ffarg="ff" || ffarg=""
  [ "$w" = "hover" ] && src="$RES/nmpc_mhe_v2.csv" || src="$RES/nmpc_mhe_v2_w$w.csv"
  for attempt in 1 2; do
    # binary self-resets (verified RELOAD) at startup
    out=$(timeout 95 ./nmpc_mhe_sil v2 24 $arg $ffarg 2>&1)
    local clean=$(echo "$out" | grep -c "verified clean")
    local n=$(wc -l < "$src" 2>/dev/null || echo 0)
    if [ "$clean" -ge 1 ] && [ "$n" -ge "$MINSAMP" ]; then
      cp "$src" "$RES/bat_${w}_${mode}_${rep}.csv"
      echo "$w $mode $rep: OK (reset verified, $n samples)"
      return
    fi
    echo "$w $mode $rep: bad (clean=$clean samples=$n) — retry $attempt"
  done
  echo "$w $mode $rep: FAILED"
}

for w in "${SPEEDS[@]}"; do
  for mode in ff noff; do
    for rep in $(seq 1 $REPS); do run_one "$w" "$mode" "$rep"; done
  done
done
echo "=== BATTERY DONE ==="
