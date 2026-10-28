#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在主机上快速体检 /odom 与 /cmd_vel：帧率、时间戳偏差、当前速度。"""
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    rclpy.init()
    node = Node("lat_check")
    odom = []
    cmd = []
    node.create_subscription(
        Odometry, "/odom",
        lambda m: odom.append((time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z,
                               m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), 50)
    node.create_subscription(
        Twist, "/cmd_vel",
        lambda m: cmd.append((time.time(), m.linear.x, m.angular.z)), 50)

    t0 = time.time()
    dur = 3.0
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)

    print("odom  msgs=%d  %.2f Hz" % (len(odom), len(odom) / dur))
    print("cmd   msgs=%d  %.2f Hz" % (len(cmd), len(cmd) / dur))
    for t, vx, wz, st in odom[:3]:
        print("  odom t=%6.3f vx=%+.4f wz=%+.4f  arr-stamp=%.1f ms"
              % (t - t0, vx, wz, (t - st) * 1000.0))
    if odom:
        bad = [abs((t - st) * 1000.0) for t, _, _, st in odom]
        print("  arr-stamp 中位 %.1f ms" % sorted(bad)[len(bad) // 2])
    if cmd:
        print("  cmd 末值 vx=%+.3f wz=%+.3f" % (cmd[-1][1], cmd[-1][2]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
