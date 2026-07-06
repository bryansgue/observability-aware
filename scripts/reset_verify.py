#!/usr/bin/env python3
"""
reset_verify.py — robust MuJoCo reset + state verification layer.

Resets the MuJoCo quadrotor and verifies it landed in a clean state
(near origin, on the floor, level attitude) before any SiL run. Handles
the reset gremlin (drone flying off to z=10000) by retrying.

Uses rclpy directly (own context) to avoid the `ros2` CLI
"rcl node's context is invalid" bug.

Exit code 0 = clean state verified, 1 = could not recover.

Usage:  python3 reset_verify.py [--tries N]
"""
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

RESET_SRV = "/quadrotor/sim/reset"
ODOM_TOP  = "/quadrotor/odom"

# Clean-state thresholds
POS_TOL   = 0.6     # |x|,|y| < 0.6 m  (near origin)
Z_MAX     = 0.10    # z < 0.10 m       (on the floor)
QW_MIN    = 0.95    # |qw| > 0.95      (level attitude)


class ResetVerifier(Node):
    def __init__(self):
        super().__init__("reset_verifier")
        self.cli = self.create_client(Trigger, RESET_SRV)
        self.last_odom = None
        self.create_subscription(Odometry, ODOM_TOP, self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _get_fresh_odom(self, timeout=3.0):
        self.last_odom = None
        t0 = time.time()
        while self.last_odom is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last_odom

    def call_reset(self, timeout=5.0):
        if not self.cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"reset service {RESET_SRV} unavailable")
            return False
        fut = self.cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.done()

    def is_clean(self):
        odom = self._get_fresh_odom()
        if odom is None:
            self.get_logger().warn("no odom received")
            return False, None
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        ok = (abs(p.x) < POS_TOL and abs(p.y) < POS_TOL
              and p.z < Z_MAX and abs(q.w) > QW_MIN)
        info = f"x={p.x:.2f} y={p.y:.2f} z={p.z:.3f} qw={q.w:.3f}"
        return ok, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tries", type=int, default=5)
    args = ap.parse_args()

    rclpy.init()
    node = ResetVerifier()
    try:
        # If already clean, nothing to do.
        ok, info = node.is_clean()
        if ok:
            node.get_logger().info(f"already clean: {info}")
            return 0

        for i in range(1, args.tries + 1):
            node.get_logger().info(f"reset attempt {i}/{args.tries} (state: {info})")
            # double reset (gremlin sometimes needs two)
            node.call_reset()
            time.sleep(0.8)
            node.call_reset()
            time.sleep(1.2)
            ok, info = node.is_clean()
            if ok:
                node.get_logger().info(f"CLEAN after {i} reset(s): {info}")
                return 0

        node.get_logger().error(f"FAILED to reach clean state: {info}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
