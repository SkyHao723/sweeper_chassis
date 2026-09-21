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
import os
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
    """同时盯两路里程计 —— 这个区别很关键:

      /odom           纯轮速积分(编码器), **完全不含 IMU**。
                      厂商节点的 Robot_Pos.X += Vx*cos(psi)*dt 就是这么算的。
      /odom_combined  EKF 融合输出 = 轮速 + IMU 的 yaw 角速度。

    只测 /odom 会得出"漂得很厉害"的结论, 但那是纯轮速的漂移 —— **不是 EKF 的**。
    EKF 的航向主要信的正是 IMU(协方差 2.5e-3 对轮速 5e-2, IMU 权重高 20 倍),
    所以 /odom_combined 的航向通常好得多。要判断"融合数据真不真"必须看后者。
    """

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.cur = None        # /odom           纯轮速
        self.comb = None       # /odom_combined  EKF 融合
        node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        node.create_subscription(Odometry, "/odom_combined", self._on_comb, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.cur = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_comb(self, msg):
        p = msg.pose.pose
        self.comb = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin_for(self, seconds, cmd=None, trace=None, ctrace=None):
        """边发命令边转 ROS, 持续 seconds 秒。
        trace/ctrace 非 None 时, 每帧把 (t, x, y, yaw) 记进去。"""
        n = max(1, int(seconds * PUB_HZ))
        for _ in range(n):
            if cmd is not None:
                self.pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=1.0 / PUB_HZ)
            if trace is not None and self.cur is not None:
                trace.append((time.monotonic(),) + self.cur)
            if ctrace is not None and self.comb is not None:
                ctrace.append((time.monotonic(),) + self.comb)

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

    def check_cmd_vel_exclusive(self):
        """★ 污染自检: /cmd_vel 上除了本脚本还有别的发布者就中止。

        底盘节点会同时收到多条速度命令, 谁后到听谁的 —— 于是测出来的是一个
        随机混合的指令, 但数据看着像模像样, 极难察觉。实测踩到过:
        decode_diag.py --drive 的进程在读线程报错后僵住不退出, 而它的发送
        线程一直在发 wz=+0.5, 于是"外环到底振不振"根本没法定论, 白追一轮。
        宁可在这里挡住, 也不要拿假数据去下结论。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        n = self.node.count_publishers("/cmd_vel")
        if n > 1:
            print("!! /cmd_vel 上有 %d 个发布者 —— 还有别人在发速度, 数据不可信。"
                  % n)
            print("   先查是谁: ps -eo pid,etimes,cmd | grep -E "
                  "'decode_diag|turn_truth|chassis_check' | grep -v grep")
            print("   僵死的直接 kill -9。注意卡住的进程可能还开着串口,")
            print("   两个进程读同一个 tty 会随机分走字节, 那样里程计也不可信。")
            return False
        return True

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

        # ★ 计时段必须是 win = dur - SETTLE_S, 不是 dur。
        #   下面算期望位移用的是 vx*win, 而分子是这段窗口里量到的位移 ——
        #   两者必须是同一段时间, 否则比例被系统性地放大 dur/win 倍
        #   (--once 时 dur=3.0/win=2.4, 也就是所有"全程比例"凭空高 25%)。
        #   上一版把分母从 dur 改成 win 时漏了这里, 于是 bug 从"偏低 30%"
        #   变成了"偏高 25%", 一样是错的。
        win = max(0.1, dur - SETTLE_S)
        self.spin_for(SETTLE_S, cmd)      # 起步(不计量, 避开加速段)
        trace = []
        ctrace = []
        x0, y0, th0 = self.cur
        cx0, cy0, cth0 = self.comb if self.comb else self.cur
        t0 = time.monotonic()
        self.spin_for(win, cmd, trace, ctrace)   # 计时段, 逐帧采样
        xe, ye, the = self.cur
        cxe, cye, cthe = self.comb if self.comb else self.cur
        self.stop()

        if len(trace) < 8:
            print("%-22s 采样太少" % name)
            return None

        # ---- 位姿连续性检查 ------------------------------------------
        # 为什么需要: "稳态/全程"全是从 /odom 的**位姿差分**算出来的。只要位姿
        # 中途跳变一次(底盘节点 respawn、看门狗重启栈、别的发布者抢 /cmd_vel),
        # 整段就是垃圾 —— 而现象往往只是"数字有点怪", 极难察觉, 实测被坑过好几次。
        # 逐帧看瞬时速度并把不合理的直接报出来, 让数据自己说明可不可信。
        # 用 0.25 秒的窗口算瞬时速度, 不然位姿的毫米级噪声会把它淹掉。
        W = 5
        vmax = max(abs(vx), 0.05)
        n_back = 0          # 命令前进却在后退
        n_fast = 0          # 比命令快 3 倍以上
        worst = 0.0
        for i in range(W, len(trace)):
            d = trace[i][0] - trace[i - W][0]
            if d <= 1e-6:
                continue
            a = ((trace[i][1] - trace[i - W][1]) * math.cos(th0) +
                 (trace[i][2] - trace[i - W][2]) * math.sin(th0))
            inst = a / d
            if vx > 0.01 and inst < -0.3 * vmax:
                n_back += 1
            if abs(inst) > 3.0 * vmax:
                n_fast += 1
                worst = max(worst, abs(inst))
        if n_back or n_fast:
            print("   ⚠ 位姿异常: 反向 %d 次 / 瞬时超速 %d 次 (最大 %.2f m/s, 命令 %.2f)"
                  % (n_back, n_fast, worst, vx))
            print("     -> 这组数字不可信。查: 有没有别的进程在发 /cmd_vel、"
                  "底盘节点是不是 respawn 过")

        # 车体系位移
        dx, dy = xe - x0, ye - y0
        fwd = dx * math.cos(th0) + dy * math.sin(th0)
        lat = -dx * math.sin(th0) + dy * math.cos(th0)
        dth = wrap(the - th0)
        # EKF 那一路
        cdx, cdy = cxe - cx0, cye - cy0
        cfwd = cdx * math.cos(cth0) + cdy * math.sin(cth0)
        clat = -cdx * math.sin(cth0) + cdy * math.cos(cth0)
        cdth = wrap(cthe - cth0)

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

        # ---- 期望值必须和"实际计量的那段时间"严格一致 ----
        # ★ 这个 bug 修过两次, 因为第一次只改了一半:
        #   第一版: 起步段 SETTLE_S 被排除在测量窗之外, 期望值却仍按 dur 算,
        #           于是"全程比例"凭空低 (dur-SETTLE)/dur —— 0.6/2.0 少 30%。
        #   第二版: 把分母改成 win = dur - SETTLE_S 了, **但计时段还在跑 dur 秒**,
        #           于是 bug 从偏低 30% 变成偏高 dur/win —— --once 时高 25%。
        #   现在: 计时段本身就是 win 秒(见上面 spin_for(win, ...)), 两边对齐。
        #   教训: 改比例公式时, 分子和分母必须一起看, 只改一半等于换个方向错。
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
        # 第二行专门打印 EKF 那一路 —— 用它和上一行对比, 就能看出 IMU 到底
        # 有没有帮上忙: 航向差得多说明 EKF 把纯轮速的漂移纠正掉了(IMU 生效);
        # 两行几乎一样说明 IMU 没被用上, 那才需要去查协方差配置。
        print("%-20s   └ EKF: 全程 %+6.3f m | 横漂 %+.3f | 航向 %+5.1f°%s"
              % ("", cfwd, clat, math.degrees(cdth),
                 ("   (比轮速少 %.1f°)" % (math.degrees(dth) - math.degrees(cdth)))
                 if abs(math.degrees(dth) - math.degrees(cdth)) > 0.3 else
                 "   (和轮速几乎一致 —— IMU 可能没起作用)"))
        return dict(name=name, fwd=fwd, exp_fwd=exp_fwd, lat=lat,
                    dth=dth, exp_dth=exp_dth, ratio_all=ratio_all,
                    ratio_ss=ratio_ss, t80=t80,
                    cfwd=cfwd, clat=clat, cdth=cdth)

    def run(self, cases):
        print("=" * 108)
        print("底盘验收: /cmd_vel 命令 vs 里程计实测")
        print("  上一行 = /odom 纯轮速(不含 IMU)   下一行 = /odom_combined EKF 融合")
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
        # ★ 用户最关心的一个对比: 走直线时(命令 wz=0)两路各自的航向漂移。
        #   纯轮速那一路的漂移来自"左右轮编码器报的转速不相等";
        #   EKF 那一路主要信 IMU, 所以如果 IMU 真的在起作用, 这里应该小得多。
        st = [r for r in results if abs(r["exp_dth"]) < 1e-6 and abs(r["exp_fwd"]) > 1e-6]
        if st:
            print("走直线时的航向漂移 (命令 wz=0, 理想都是 0):")
            print("   %-18s %12s %12s" % ("", "/odom(纯轮速)", "/odom_combined"))
            for r in st:
                d = math.hypot(r["fwd"], r["lat"]) or abs(r["fwd"])
                print("   %-18s %+8.1f° (%+5.1f°/m) %+8.1f° (%+5.1f°/m)"
                      % (r["name"], math.degrees(r["dth"]),
                         math.degrees(r["dth"]) / d if d > 0.05 else 0,
                         math.degrees(r["cdth"]),
                         math.degrees(r["cdth"]) / d if d > 0.05 else 0))
            raw_m = sum(abs(math.degrees(r["dth"])) for r in st) / len(st)
            ekf_m = sum(abs(math.degrees(r["cdth"])) for r in st) / len(st)
            print("   平均: 纯轮速 %.1f°  vs  EKF %.1f°" % (raw_m, ekf_m))
            if raw_m > 1.0 and ekf_m < raw_m * 0.6:
                print("   -> **EKF 明显改善了航向, IMU 在起作用** ✓")
            elif raw_m > 1.0:
                print("   -> EKF 和纯轮速差不多 —— **IMU 可能没被用上**, 去查")
                print("      /imu/data_raw 的 angular_velocity.z 有没有数据、")
                print("      符号对不对(逆时针转应该是正的)、以及 ekf.yaml 的 imu0_config")
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
        if not chk.check_cmd_vel_exclusive():
            return                 # 有别的发布者, 测了也是废数据
        chk.run(cases)
    except KeyboardInterrupt:
        pass
    finally:
        chk.stop()
        print("\n已发零速停车。")
        node.destroy_node()
        rclpy.shutdown()

    # ★ 强制退出: rclpy/DDS 的关闭偶尔会卡住, 卡住的进程会一直占着资源,
    #   更糟的是如果它还在发 /cmd_vel, 后面的测量就全废了。
    os._exit(0)


if __name__ == "__main__":
    main()
