#!/bin/bash
# batch_sens.sh — gate sensitivity study (Reviewer 1, concern 4).
# Re-runs the "trans" experiment (aggressive -> hover under a deterministic wind,
# same protocol as batch_trans.sh) while sweeping one gate/filter knob at a time:
#   dwell t_d, sigma window, q_m, q_d, injected accelerometer noise,
#   threshold sigma_tilde_min, and maneuver intensity (liss.w).
# Output: results/sens_<cfg>_seed<k>.csv   (analyze with scripts/analyze_sens.py)
#
# PREREQ: sim_up.sh launches/reuses the headless MuJoCo sim (noise ON, no wind node).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/../build"
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH

RES=../results
REPS=${REPS:-5}
TRUN=40
FEXT=1.5
MINSAMP=3000

# name | env assignments (space separated, may be empty) | liss.w
CONFIGS=(
  "base        -                            1.0"
  "dwell_10    EKF_DWELL=10                 1.0"
  "dwell_50    EKF_DWELL=50                 1.0"
  "dwell_100   EKF_DWELL=100                1.0"
  "win_50      EKF_WIN=50                   1.0"
  "win_200     EKF_WIN=200                  1.0"
  "qm_1e-6     EKF_QM=1e-6                  1.0"
  "qm_1e-4     EKF_QM=1e-4                  1.0"
  "qd_1e-3     EKF_QD=1e-3                  1.0"
  "qd_1e-1     EKF_QD=1e-1                  1.0"
  "anoise_0.3  ACCEL_NOISE=0.3              1.0"
  "anoise_0.7  ACCEL_NOISE=0.7              1.0"
  "thr_0.003   EKF_GATE_THR=0.003           1.0"
  "thr_0.006   EKF_GATE_THR=0.006           1.0"
  "thr_0.015   EKF_GATE_THR=0.015           1.0"
  "thr_0.03    EKF_GATE_THR=0.03            1.0"
  "thr_0.06    EKF_GATE_THR=0.06            1.0"
  "speed_0.5   -                            0.5"
  "speed_1.5   -                            1.5"
  "pe_gate     ENERGY_GATE=1000             1.0"
)

# SAFETY: this battery must run on the SAME simulator build as the paper data
# (June-18 MujocoRosUtils plugin, launched manually). Never relaunch via sim_up.sh:
# a fresh launch would pick the current (post-06-28) plugin, whose IMU differs.
sim_check() {
  pgrep -x headless_runner >/dev/null || { echo "[batch_sens] simulator DOWN — abort (relaunch June-plugin runner manually)"; exit 1; }
  timeout 6 ros2 topic echo /quadrotor/odom --once >/dev/null 2>&1 || { echo "[batch_sens] odom silent — abort"; exit 1; }
}
sim_check

gt_applied() { awk -F, 'NR>1 && ($56*$56+$57*$57+$58*$58)>0.25{c++} END{exit (c>50)?0:1}' "$1"; }

run_one() {  # $1=name $2=envs $3=w $4=rep
  local name=$1 envs=$2 w=$3 rep=$4
  local src="$RES/nmpc_mhe_v2_w$w.csv"
  local out="$RES/sens_${name}_seed${rep}.csv"
  [ -s "$out" ] && { echo "[$name/$rep] exists, skip"; return; }
  for attempt in 1 2 3; do
    sim_check
    rm -f "$src"
    if [ "$envs" = "-" ]; then envs=""; fi
    env EKF_GATE_THR=0.009 $envs timeout $((TRUN+40)) ./nmpc_mhe_sil v2 $TRUN w=$w trans fext=$FEXT \
      >/tmp/sens_${name}_$rep.log 2>&1
    local n=$(wc -l < "$src" 2>/dev/null || echo 0)
    if [ "$n" -ge "$MINSAMP" ] && gt_applied "$src"; then
      cp "$src" "$out"; echo "[$name/$rep] OK ($n samples)"; return
    fi
    echo "[$name/$rep] bad (samples=$n) — retry $attempt"
  done
  echo "[$name/$rep] FAILED"
}

for cfg in "${CONFIGS[@]}"; do
  set -- $cfg
  name=$1; w=${@: -1}; envs="${@:2:$(($#-2))}"
  for rep in $(seq 1 $REPS); do run_one "$name" "$envs" "$w" "$rep"; done
done
echo "=== SENSITIVITY BATTERY DONE ==="
