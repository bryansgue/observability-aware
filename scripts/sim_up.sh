#!/bin/bash
# sim_up.sh — SINGLETON MuJoCo launcher (sim-launch protocol).
# Guarantees exactly ONE healthy simulator:
#   - clears stray wind_publisher nodes ALWAYS;
#   - if a healthy sim is already running → REUSE it;
#   - if a stale/broken sim is running → kill it, then launch fresh;
#   - never leaves two sims active; verifies odom before returning.
# Idempotent: safe to call before every battery. NO `set -u` (ROS setup).
#
# IMPORTANT: process detection uses `pgrep -x headless_runner` (exact executable),
# NOT `pgrep -f <pattern>` — a -f pattern also matches our own shell command line
# (which contains these strings), which would make sim_alive() always true.
#
# Scene: gates (collide off), headless realtime, sensor noise ON, NO wind node
# (the controller injects the deterministic perturbation via /external_force_cmd).
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/robotics/msgs_ws/install/setup.bash 2>/dev/null
source ~/mujoco_ws/install/setup.bash 2>/dev/null
source ~/uav_ws/install/setup.bash 2>/dev/null

QUAD=${1:-quadrotor}
PGID_FILE=/tmp/sim_up_${QUAD}.pgid

sim_alive()  { pgrep -x headless_runner >/dev/null 2>&1; }      # exact: real sim only
odom_alive() {                                                  # retry: discovery is slow
  for _t in 1 2 3 4 5; do
    timeout 4 ros2 topic echo "/$QUAD/odom" --once >/dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}
kill_sim() {
  [ -f "$PGID_FILE" ] && kill -9 -"$(cat "$PGID_FILE")" 2>/dev/null  # our launch's whole group
  pkill -x headless_runner 2>/dev/null                              # catch-all for the runner
  rm -f "$PGID_FILE"
  sleep 2
}

# 1) ALWAYS clear stray wind nodes (these accumulated and broke the sim before).
pkill -x wind_publisher 2>/dev/null

# 2) Reuse a healthy sim.
if sim_alive && odom_alive; then
  echo "[sim_up] REUSE — healthy sim already running, $QUAD/odom publishing."
  exit 0
fi

# 3) Kill any stale/broken sim so we never end up with two.
if sim_alive; then
  echo "[sim_up] sim process alive but odom silent → broken, killing."
  kill_sim
fi

# 4) Launch fresh in its OWN process group (setsid) so we can kill the whole tree.
echo "[sim_up] launching fresh sim (noise ON, no wind node)..."
MUJOCO_ODOM_NOISE=1 setsid ros2 launch drone_teleop mujoco_headless.launch.py \
  scene:=gates gates_collide:=off realtime:=true quad_name:="$QUAD" \
  >/tmp/sim_up.log 2>&1 &
echo $! > "$PGID_FILE"   # setsid → child is its own group leader; $! is the PGID

# 5) Verify it actually came up.
sleep 6
for _round in 1 2 3 4; do
  if sim_alive && odom_alive; then
    echo "[sim_up] OK — sim up, $QUAD/odom publishing."
    exit 0
  fi
  sleep 2
done
echo "[sim_up] FAILED — sim did not come up (see /tmp/sim_up.log)."
exit 1
