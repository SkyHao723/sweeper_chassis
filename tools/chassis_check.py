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

    def wait_for_odom(self, timeout_s=15.0):
        """等 /odom 出现。

        ★ 别只 spin 一两秒就判"收不到" —— CH340 偶尔会掉线让底盘节点崩掉,
          而 launch 里配了 respawn, 节点 2 秒后会自己回来。正好在那个窗口里
          启动脚本就会误报"收不到 /odom", 让人以为是自己的问题。
        """
        t0 = time.monotonic()
        nxt = 0.0
        while self.cur is None and time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - t0 > nxt:
                nxt += 3.0
                print("  等 /odom ... 已等 %.0f 秒 (底盘节点在不在跑? 串口在不在?)"
                      % (time.monotonic() - t0))
        return self.cur is not None

    def run_to_distance(self, vx, target_m, timeout_s=40.0):
        """往前开, 直到 /odom 报的位移达到 target_m 就停。

        用途: 验证里程计刻度。**"里程计报的距离"被脚本钉死在 target_m**,
        所以偏差全部体现在"车实际停在哪"上:
            车正好停在 target_m 处  -> 里程计准
            车停在 target_m/2 处    -> 里程计多报一倍
            车停在 target_m*2 处    -> 里程计少报一半
        比"掐时间"或"看见就按 Ctrl-C"精确得多, 而且不需要卷尺。
        """
        self.stop()
        if not self.wait_for_odom():
            print("!! 等了 15 秒还是收不到 /odom。检查:")
            print("   ros2 node list | grep wheeltec      # 底盘节点在不在")
            print("   fuser -v /dev/wheeltec_controller   # 串口被谁占着")
            print("   ros2 topic echo /odom --once        # 话题有没有数据")
            return
        x0, y0, _ = self.cur
        cmd = Twist()
        cmd.linear.x = vx

        t0 = time.monotonic()
        d = 0.0
        while True:
            self.pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=1.0 / PUB_HZ)
            if self.cur is not None:
                d = math.hypot(self.cur[0] - x0, self.cur[1] - y0)
                if d >= target_m:
                    break
            if time.monotonic() - t0 > timeout_s:
                print("!! 超时: 里程计只走到 %.3f m (目标 %.2f m) —— 车根本没动?"
                      % (d, target_m))
                break

        self.stop()
        print("里程计位移 = %.3f m (目标 %.3f m), 用了 %.1f 秒"
              % (d, target_m, time.monotonic() - t0))
        print()
        print("现在看车实际停在哪 (起点到车同一个参照点的直线距离):")
        print("  正好 %.2f m  ->  里程计准" % target_m)
        print("  约 %.2f m    ->  里程计多报一倍" % (target_m / 2))
        print("  约 %.2f m    ->  里程计少报一半" % (target_m * 2))

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

        # ---- 期望值必须按**窗口**时长算, 不能按整条命令的时长 ----
        # ★ 踩过: SETTLE_S 秒的起步段被排除在测量窗之外了, 但期望值原来仍按
        #   dur 算, 于是"全程比例"凭空低 (dur-SETTLE)/dur —— 0.6/2.0 就是少 30%,
        #   还让脚本一直打印"比稳态低说明起步慢", 完全是误导。
        win = max(0.1, dur - SETTLE_S)
        exp_fwd = vx * win
        exp_dth = wz * win
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
            print("直线全程比例: 平均 %.0f%%  (同一窗口内的平均速度, 和稳态比可看出加速段占比)"
                  % (sum(al) / len(al)))
            t80s = [r["t80"] for r in lin if r["t80"] is not None]
            if t80s:
                print("爬到 80%% 速度用时: 平均 %.2fs  最长 %.2fs"
                      % (sum(t80s) / len(t80s), max(t80s)))
        turns = [r for r in results if abs(r["exp_dth"]) > 1e-6]
        if turns:
            print("转向 (按窗口时长算的期望值):")
            for r in turns:
                print("   %-18s 实测 %+6.1f° / 期望 %+6.1f°  =  %3.0f%%"
                      % (r["name"], math.degrees(r["dth"]),
                         math.degrees(r["exp_dth"]),
                         r["dth"] / r["exp_dth"] * 100.0))
            # 左右对不对称: 同样的角速度、相反的符号, 幅度应该接近
            if len(turns) >= 2:
                mags = [abs(r["dth"] / r["exp_dth"]) for r in turns]
                print("   左右对称性: 最大 %.0f%% / 最小 %.0f%%  (差得多说明两轮出力不匀)"
                      % (max(mags) * 100, min(mags) * 100))
        print()
        print("怎么用这些数:")
        print("  * 稳态比例明显不是 100% -> 看是不是随速度变化: 低速偏低+高速偏高")
        print("    说明是速度环的增益问题, 不是刻度问题, **不要**用单一系数去补")
        print("  * 到80%用很久 / 全程远低于稳态 -> 起步慢")
        print("  * 转向左右不对称 -> 两轮出力不匀(速度环振荡的相位差)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--linear-only", action="store_true")
    ap.add_argument("--turn-only", action="store_true")
    ap.add_argument("--dur", type=float, default=3.0, help="每条测多久(秒)")
    ap.add_argument("--once", type=float, metavar="VX",
                    help="只跑一条直线 vx m/s, 用 --dur 指定秒数。"
                         "专门用来做\"实际距离 vs /odom\"的对比: 拿卷尺量车实际"
                         "走了多远, 和脚本报的 /odom 位移比 —— 这个比值就是"
                         "里程计的标定系数, **和控制准不准无关**。")
    ap.add_argument("--drive-to", type=float, metavar="米",
                    help="往前开直到 /odom 报的位移达到这个米数就停 —— 用来"
                         "验证里程计刻度(见 run_to_distance 的说明)。"
                         "配套 --vx 指定速度, 默认 0.20 m/s。")
    ap.add_argument("--vx", type=float, default=0.20,
                    help="--drive-to 用的速度, 默认 0.20 m/s")
    args = ap.parse_args()

    if args.drive_to is not None:
        rclpy.init()
        node = rclpy.create_node("chassis_check")
        chk = ChassisCheck(node)
        try:
            chk.spin_for(1.0)
            chk.run_to_distance(args.vx, args.drive_to)
        except KeyboardInterrupt:
            pass
        finally:
            chk.stop()
            node.destroy_node()
            rclpy.shutdown()
        return

    cases = []
    if args.once is not None:
        cases = [("单次 前进 %.2f m/s" % args.once, args.once, 0.0, args.dur)]
    elif not args.turn_only:
        cases += [
            ("前进 0.15 m/s", 0.15, 0.0, args.dur),
            ("前进 0.30 m/s", 0.30, 0.0, args.dur),
            ("前进 0.50 m/s", 0.50, 0.0, args.dur),
            ("后退 -0.20 m/s", -0.20, 0.0, args.dur),
        ]
    if args.once is None and not args.linear_only:
        cases += [
            ("原地左转 0.5 r/s", 0.0, 0.5, args.dur),
            ("原地右转 -0.5 r/s", 0.0, -0.5, args.dur),
            ("曲线 0.3/0.4", 0.30, 0.4, args.dur),
        ]

    rclpy.init()
    node = rclpy.create_node("chassis_check")
    chk = ChassisCheck(node)
    try:
        chk.wait_for_odom()        # 先确保 /odom 有数据(节点可能正在 respawn)
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
