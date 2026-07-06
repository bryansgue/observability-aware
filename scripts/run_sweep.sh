#!/usr/bin/env bash
# run_sweep.sh — robust velocity sweep with reset+verify before every run.
# Each (w, trial) gets a clean verified sim state before launching the SiL.
#
# Usage:  ./run_sweep.sh "0.5 1.0 1.5 2.0 2.5 3.0"  [n_trials]
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/../build"
RES_DIR="$SCRIPT_DIR/../results"

WS="${1:-0.5 1.0 1.5 2.0 2.5 3.0}"
N_TRIALS="${2:-1}"

cd "$BUILD_DIR" || exit 1

for w in $WS; do
  for trial in $(seq 1 "$N_TRIALS"); do
    echo "================ w=$w  trial=$trial/$N_TRIALS ================"
    # 1) reset + verify clean state (retries internally)
    if ! python3 "$SCRIPT_DIR/reset_verify.py" --tries 6 2>&1 | grep -E "CLEAN|already clean|FAILED"; then
      echo "[run_sweep] reset failed for w=$w trial=$trial — skipping"
      continue
    fi
    sleep 1
    # 2) run the SiL
    timeout 80 ./nmpc_mhe_sil v2 60 "w=$w" 2>&1 | grep -E "MHE convergence|completed=" | tail -2
    # 3) archive trial CSV if doing statistics
    if [ "$N_TRIALS" -gt 1 ]; then
      cp "$RES_DIR/nmpc_mhe_v2_w$w.csv" "$RES_DIR/nmpc_mhe_v2_w${w}_t${trial}.csv" 2>/dev/null
    fi
    rows=$(wc -l < "$RES_DIR/nmpc_mhe_v2_w$w.csv" 2>/dev/null || echo 0)
    echo "[run_sweep] w=$w trial=$trial → $rows rows"
    sleep 1
  done
done
echo "================ sweep done ================"
