#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘验收测试 —— 通过 ROS 发 /cmd_vel, 用 /odom 核对实测值。

这是"底盘做好没有"的可重复判据: 每次改完固件/参数跑一遍, 看有没有退步。

前置: 厂商底盘节点 + EKF 已经在跑(它们占着串口), 也就是
      ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

    ⚠️ 轮子必须悬空, 或者至少保证前方有足够空间。每条测试之间会自动停车。
    ⚠️ 没有雷达也能跑这个 —— 它只用 /cmd_vel 和 /odom。

用法:
    python3 tools/chassis_check.py                 # 跑全套
    python3 tools/chassis_check.py --linear-only   # 只测直线
    python3 tools/chassis_check.py --turn-only     # 只测转向
"""

import argparse
import math
import sys
import time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    sys.exit("需要在 ROS 2 环境里运行, 先 source /opt/ros/humble/setup.bash")

PUB_HZ = 20.0
SETTLE_S = 0.6          # 发命令后等它动起来再开始算距离
STOP_S = 1.0            # 停车后等它真的停稳再读 /odom


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ChassisCheck(object):
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.cur = None
        node.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.cur = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin_for(self, seconds, cmd=None, trace=None):
        """边发命令边转 ROS, 持续 seconds 秒。
        trace 非 None 时, 每帧把 (t, x, y, yaw) 记进去。"""
        n = max(1, int(seconds * PUB_HZ))
        for _ in range(n):
            if cmd is not None:
                self.pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=1.0 / PUB_HZ)
            if trace is not None and self.cur is not None:
                trace.append((time.monotonic(),) + self.cur)

    def stop(self):
        z = Twist()
        self.spin_for(STOP_S, z)

    def run_case(self, name, vx, wz, dur):
        self.stop()
        if self.cur is None:
            print("%-22s 收不到 /odom" % name)
            return None

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz

        self.spin_for(SETTLE_S, cmd)      # 起步(不计量, 避开加速段)
        trace = []
        x0, y0, th0 = self.cur
        t0 = time.monotonic()
        self.spin_for(dur, cmd, trace)    # 计时段, 逐帧采样
        xe, ye, the = self.cur
        self.stop()

        if len(trace) < 8:
            print("%-22s 采样太少" % name)
            return None

        # 车体系位移
        dx, dy = xe - x0, ye - y0
        fwd = dx * math.cos(th0) + dy * math.sin(th0)
        lat = -dx * math.sin(th0) + dy * math.cos(th0)
        dth = wrap(the - th0)

        # ---- 稳态: 取窗口后 40% 的平均速度 ----
        k = int(len(trace) * 0.6)
        tail = trace[k:]
        dt = tail[-1][0] - tail[0][0]
        if dt > 0.05:
            ddx = tail[-1][1] - tail[0][1]
            ddy = tail[-1][2] - tail[0][2]
            sfwd = ddx * math.cos(th0) + ddy * math.sin(th0)
            sdth = wrap(tail[-1][3] - tail[0][3])
            v_steady = sfwd / dt
            w_steady = sdth / dt
        else:
            v_steady = w_steady = 0.0

        # ---- 爬到 80% 目标速度用了多久 ----
        t80 = None
        ref = vx if abs(vx) > 1e-6 else None
        if ref:
            for i in range(1, len(trace)):
                d = trace[i][0] - trace[i - 1][0]
                if d <= 1e-6:
                    continue
                a = (trace[i][1] - trace[i - 1][1]) * math.cos(th0) + \
                    (trace[i][2] - trace[i - 1][2]) * math.sin(th0)
                inst = a / d
                if abs(inst) >= 0.8 * abs(ref):
                    t80 = trace[i][0] - t0
                    break

        exp_fwd = vx * dur
        exp_dth = wz * dur
        ratio_all = (fwd / exp_fwd * 100.0) if abs(exp_fwd) > 1e-6 else 0.0
        # 稳态比例: 直线看 vx, 转向看 wz
        if abs(wz) > 1e-6:
            ratio_ss = w_steady / wz * 100.0
        elif abs(vx) > 1e-6:
            ratio_ss = v_steady / vx * 100.0
        else:
            ratio_ss = 0.0

        print("%-20s 全程 %+6.3f/%-6.3f m (%3.0f%%) | 稳态 %3.0f%% | 到80%%用 %s | 横漂 %+.3f | 航向 %+5.1f°"
              % (name, fwd, exp_fwd, ratio_all, ratio_ss,
                 ("%.2fs" % t80) if t80 is not None else "  >窗口",
                 lat, math.degrees(dth)))
        return dict(name=name, fwd=fwd, exp_fwd=exp_fwd, lat=lat,
                    dth=dth, exp_dth=exp_dth, ratio_all=ratio_all,
                    ratio_ss=ratio_ss, t80=t80)

    def run(self, cases):
        print("=" * 108)
        print("底盘验收: /cmd_vel 命令 vs /odom 实测")
        print("=" * 108)
        results = []
        for name, vx, wz, dur in cases:
            r = self.run_case(name, vx, wz, dur)
            if r:
                results.append(r)

        if not results:
            return
        print("-" * 108)
        lin = [r for r in results if abs(r["exp_fwd"]) > 1e-6]
        if lin:
            ss = [r["ratio_ss"] for r in lin]
            al = [r["ratio_all"] for r in lin]
            print("直线稳态比例: 平均 %.0f%%  最小 %.0f%%  最大 %.0f%%"
                  % (sum(ss) / len(ss), min(ss), max(ss)))
            print("直线全程比例: 平均 %.0f%%   <- 比稳态低说明起步慢" % (sum(al) / len(al)))
            t80s = [r["t80"] for r in lin if r["t80"] is not None]
            if t80s:
                print("爬到 80%% 速度用时: 平均 %.2fs  最长 %.2fs"
                      % (sum(t80s) / len(t80s), max(t80s)))
        turns = [r for r in results if abs(r["exp_dth"]) > 1e-6]
        if turns:
            ss = [r["ratio_ss"] for r in turns]
            print("转向稳态比例: 平均 %.0f%%  (正负号直接看上面)" % (sum(ss) / len(ss)))
        print()
        print("怎么用这些数:")
        print("  * 稳态比例明显不是 100% -> 改 config/ekf*.yaml 的 odom_*_scale 按比例修正")
        print("  * 全程比稳态低很多 / 到80%用很久 -> 起步慢, 见 README 的'低速拖死'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--linear-only", action="store_true")
    ap.add_argument("--turn-only", action="store_true")
    ap.add_argument("--dur", type=float, default=3.0, help="每条测多久(秒)")
    args = ap.parse_args()

    cases = []
    if not args.turn_only:
        cases += [
            ("前进 0.15 m/s", 0.15, 0.0, args.dur),
            ("前进 0.30 m/s", 0.30, 0.0, args.dur),
            ("前进 0.50 m/s", 0.50, 0.0, args.dur),
            ("后退 -0.20 m/s", -0.20, 0.0, args.dur),
        ]
    if not args.linear_only:
        cases += [
            ("原地左转 0.5 r/s", 0.0, 0.5, args.dur),
            ("原地右转 -0.5 r/s", 0.0, -0.5, args.dur),
            ("曲线 0.3/0.4", 0.30, 0.4, args.dur),
        ]

    rclpy.init()
    node = rclpy.create_node("chassis_check")
    chk = ChassisCheck(node)
    try:
        chk.spin_for(1.0)          # 先收几帧 /odom
        chk.run(cases)
    except KeyboardInterrupt:
        pass
    finally:
        chk.stop()
        print("\n已发零速停车。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
