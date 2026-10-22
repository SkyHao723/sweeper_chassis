#!/usr/bin/env python3
"""车端闭环自测：/cmd_vel -> 底盘 -> /odom 验证（需要 tb01_chassis 节点已启动）。

用法（车端，已 source ROS 环境和 tb01_ws）：
    ROS_DOMAIN_ID=7 python3 cmd_vel_odom_test.py [--vx 0.2] [--wz 0.3]

会依次：直行、停车、原地转、停车，打印 /odom 帧率/位移/转角、/chassis_diag 内容、
/imu/data_raw 频率、/PowerVoltage。结束时确保发零速。
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


class CmdVelOdomTest(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_odom_test")
        self.odom: Odometry | None = None
        self.diag: dict | None = None
        self.diag_raw: str | None = None
        self.imu_count = 0
        self.power: float | None = None
        self.odom_times: list[float] = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/chassis_diag", self._on_diag, 10)
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, 10)
        self.create_subscription(Float32, "/PowerVoltage", self._on_power, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_times.append(time.monotonic())

    def _on_diag(self, msg: String) -> None:
        self.diag_raw = msg.data
        try:
            self.diag = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            self.diag = None

    def _on_imu(self, msg: Imu) -> None:
        self.imu_count += 1

    def _on_power(self, msg: Float32) -> None:
        self.power = msg.data

    # ------------------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def drive(self, vx: float, wz: float, seconds: float, rate_hz: float = 20.0) -> None:
        period = 1.0 / rate_hz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(vx, wz)
            rclpy.spin_once(self, timeout_sec=period * 0.5)
            time.sleep(period * 0.2)

    def stop_and_settle(self, seconds: float = 1.0) -> None:
        self.publish(0.0, 0.0)
        self.spin_for(seconds)

    # ------------------------------------------------------------------
    def pose(self) -> tuple[float, float, float]:
        if self.odom is None:
            return (0.0, 0.0, 0.0)
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    def odom_rate(self, seconds: float = 2.0) -> float:
        self.odom_times.clear()
        self.spin_for(seconds)
        if len(self.odom_times) < 2:
            return 0.0
        return (len(self.odom_times) - 1) / (self.odom_times[-1] - self.odom_times[0])

    def twist(self) -> tuple[float, float]:
        if self.odom is None:
            return (0.0, 0.0)
        return (self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.2, help="直行速度 m/s")
    ap.add_argument("--vx-seconds", type=float, default=1.5)
    ap.add_argument("--wz", type=float, default=0.3, help="原地转角速度 rad/s")
    ap.add_argument("--wz-seconds", type=float, default=1.5)
    ap.add_argument("--no-motion", action="store_true", help="只检查话题，不发速度")
    args = ap.parse_args()

    rclpy.init()
    node = CmdVelOdomTest()
    results: dict = {}
    try:
        node.spin_for(2.0)
        rate = node.odom_rate(2.0)
        imu_rate = node.imu_count / 4.0
        results["odom_hz"] = round(rate, 1)
        results["imu_hz"] = round(imu_rate, 1)
        results["power_voltage"] = node.power
        results["diag"] = node.diag
        print(f"/odom 实测 {rate:.1f} Hz | /imu {imu_rate:.1f} Hz | /PowerVoltage={node.power}")
        if node.diag is None:
            print("!! 没收到 /chassis_diag")
        else:
            print("chassis_diag:", json.dumps(node.diag, ensure_ascii=False))
        if node.odom is not None:
            print(f"/odom frame_id={node.odom.header.frame_id} child={node.odom.child_frame_id}")

        if not args.no_motion:
            x0, y0, th0 = node.pose()
            print(f"\n[直行] vx={args.vx} m/s × {args.vx_seconds}s ...")
            node.drive(args.vx, 0.0, args.vx_seconds)
            node.stop_and_settle(1.2)
            x1, y1, th1 = node.pose()
            dist = math.hypot(x1 - x0, y1 - y0)
            results["forward"] = {
                "cmd_vx": args.vx,
                "cmd_seconds": args.vx_seconds,
                "expected_m": round(args.vx * args.vx_seconds, 3),
                "measured_m": round(dist, 3),
                "ratio": round(dist / (args.vx * args.vx_seconds), 3) if args.vx and args.vx_seconds else None,
                "odom_twist_vx": round(node.twist()[0], 3),
                "diag_cmd_vx": node.diag.get("cmd_vx") if node.diag else None,
            }
            print("  直行结果:", json.dumps(results["forward"], ensure_ascii=False))

            x2, y2, th2 = node.pose()
            print(f"\n[原地转] wz={args.wz} rad/s × {args.wz_seconds}s ...")
            node.drive(0.0, args.wz, args.wz_seconds)
            node.stop_and_settle(1.2)
            x3, y3, th3 = node.pose()
            dth = math.atan2(math.sin(th3 - th2), math.cos(th3 - th2))
            results["turn"] = {
                "cmd_wz": args.wz,
                "cmd_seconds": args.wz_seconds,
                "expected_rad": round(args.wz * args.wz_seconds, 3),
                "measured_rad": round(dth, 3),
                "ratio": round(dth / (args.wz * args.wz_seconds), 3) if args.wz and args.wz_seconds else None,
                "odom_twist_wz": round(node.twist()[1], 3),
                "diag_cmd_wz": node.diag.get("cmd_wz") if node.diag else None,
            }
            print("  原地转结果:", json.dumps(results["turn"], ensure_ascii=False))

            print("\n[停车] 发零速并等待 1.5s ...")
            node.stop_and_settle(1.5)
            results["after_stop"] = {
                "odom_twist_vx": round(node.twist()[0], 3),
                "odom_twist_wz": round(node.twist()[1], 3),
                "diag_cmd_timeout": node.diag.get("cmd_timeout") if node.diag else None,
            }
            print("  停车后:", json.dumps(results["after_stop"], ensure_ascii=False))
    finally:
        try:
            node.publish(0.0, 0.0)
            node.spin_for(0.6)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    print("\n===== SUMMARY =====")
    print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
