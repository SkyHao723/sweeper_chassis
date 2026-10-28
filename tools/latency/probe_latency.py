#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测量「/cmd_vel 发布 → 轮子实际转动」的链路延迟（软件侧, 不改固件）。

链路: /cmd_vel → 厂商 wheeltec_robot_node → 串口 11B 帧 → STM32
      → CAN → 达妙驱动器速度环 → 轮子

方法
----
本脚本同时是发布者和 /odom 订阅者, 用同一个时钟 (time.time) 给每个事件打时间戳:

  trial i:
     发 0            ── 静置 settle 秒（等轮子彻底停、等固件看门狗把目标清零）
     记录 t_step; 发 ±speed  ── 一个阶跃（只发一条, STM32 有 800 ms 看门狗会自己停）
     等 hold 秒; 发 0

  方向交替 (+,-,+,-...) 让车在原地小幅前后摆, 不会跑远。

  所有 /odom 样本都记下来:
     t   = 收到的时刻 (主机时钟)
     st  = 消息里的 header.stamp (节点收到该串口帧并发布的时刻, 主机时钟)
     vx, wz

  轮子转速代理量: vwheel = |vx| + |wz| * TRACK/2
     —— 不管是往前跑还是原地打转, 只要轮子在转, 这个量就 > 0。

输出: /tmp/latency.json
  {"config":..., "published":[{t,vx,tag}], "samples":[{t,st,vx,wz}], "check":{...}}

配套:
  run_latency.sh   —— 停 car_web、起节点（带 strace 抓串口 write 时刻）、跑本脚本
  analyze_latency.py —— 把 /tmp/latency.json + /tmp/strace_node.log 一起分析出分段延迟
"""

import argparse
import json
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

TRACK = 0.456          # 洗地车轮距; 扫地车 0.716 —— 只影响 wz 当量换算, 不影响判定
WHEEL_R = 0.0855


class Probe(Node):
    def __init__(self):
        super().__init__("latency_probe")
        qos = QoSProfile(
            depth=300,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.samples = []
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.published = []

    # ---------- 回调 ----------
    def _on_odom(self, msg: Odometry):
        st = msg.header.stamp
        self.samples.append(
            {
                "t": time.time(),
                "st": st.sec + st.nanosec * 1e-9,
                "vx": msg.twist.twist.linear.x,
                "wz": msg.twist.twist.angular.z,
            }
        )

    # ---------- 工具 ----------
    def spin_for(self, seconds, dt=0.002):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=dt)

    def spin_until(self, cond, timeout=10.0, dt=0.002):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=dt)
            if cond():
                return True
        return False

    def send(self, vx, wz=0.0, tag=""):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        t = time.time()
        self.pub.publish(m)
        self.published.append({"t": t, "vx": float(vx), "wz": float(wz), "tag": tag})
        return t

    def vwheel(self, s):
        """轮子转速代理量 m/s"""
        return abs(s["vx"]) + abs(s["wz"]) * TRACK * 0.5

    def mean_vwheel(self, t_from, t_to):
        vals = [self.vwheel(s) for s in self.samples if t_from <= s["t"] <= t_to]
        return statistics.fmean(vals) if vals else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.20, help="阶跃后的目标线速度 m/s")
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--settle", type=float, default=1.00, help="每 trial 前的静置时间 s")
    ap.add_argument("--hold", type=float, default=0.60, help="阶跃保持时间 s")
    ap.add_argument("--gap", type=float, default=0.20, help="trial 之间额外间隔 s")
    ap.add_argument("--out", default="/tmp/latency.json")
    ap.add_argument("--check-only", action="store_true", help="只做方向自检, 不跑 trial")
    args = ap.parse_args()

    rclpy.init()
    p = Probe()

    print("[probe] 等待 /odom ...", flush=True)
    if not p.spin_until(lambda: len(p.samples) > 0, timeout=15.0):
        print("[probe] !! 收不到 /odom, 检查厂商节点是否起来了", flush=True)
        rclpy.shutdown()
        return 2

    # 先量 1 秒静止噪声
    n0 = len(p.samples)
    t_noise0 = time.time()
    p.spin_for(1.0)
    noise = [p.vwheel(s) for s in p.samples[n0:]]
    noise_max = max(noise) if noise else 0.0
    print("[probe] 静止 1 s: 样本 %d 个, vwheel 最大 %.4f m/s, 中位 %.4f"
          % (len(noise), noise_max, statistics.median(noise) if noise else 0), flush=True)

    # ---------- 方向自检 ----------
    print("[probe] --- 方向自检: 发 vx=+0.10 保持 0.5 s ---", flush=True)
    p.send(0.0, tag="check0")
    p.spin_for(0.6)
    t_chk = p.send(0.10, tag="check")
    p.spin_for(0.5)
    p.send(0.0, tag="check_stop")
    p.spin_for(0.8)
    seg = [s for s in p.samples if s["t"] >= t_chk + 0.15]
    vx = statistics.fmean([s["vx"] for s in seg]) if seg else 0.0
    wz = statistics.fmean([s["wz"] for s in seg]) if seg else 0.0
    print("[probe] 自检结果: /odom vx 平均 %+.3f m/s, wz 平均 %+.3f rad/s"
          % (vx, wz), flush=True)
    verdict = "unknown"
    if vx > 0.06:
        verdict = "forward"          # 命令正 vx → odom 正 vx
    elif abs(wz) > 0.15:
        verdict = "spin"             # 原地打转 = 左右方向系数错
    elif vx < -0.06:
        verdict = "backward"
    print("[probe] 自检判读: %s" % verdict, flush=True)
    print("[probe] (forward=方向正常; spin=左右方向系数反了; backward=整体反了)", flush=True)

    if args.check_only:
        data = {"config": vars(args), "published": p.published, "samples": p.samples,
                "check": {"vx": vx, "wz": wz, "verdict": verdict,
                          "noise_max": noise_max}}
        with open(args.out, "w") as f:
            json.dump(data, f)
        print("[probe] 已写 %s" % args.out, flush=True)
        rclpy.shutdown()
        return 0

    # ---------- 正式 trial ----------
    print("[probe] --- 正式测量: %d 个 trial, 目标 ±%.2f m/s ---"
          % (args.trials, args.speed), flush=True)
    for i in range(args.trials):
        sign = 1.0 if (i % 2 == 0) else -1.0
        p.send(0.0, tag="t%d_pre" % i)
        p.spin_for(args.settle)
        t_step = p.send(sign * args.speed, tag="t%d_step" % i)
        p.spin_for(args.hold)
        p.send(0.0, tag="t%d_stop" % i)
        p.spin_for(args.gap)
        print("[probe] trial %2d/%d  %s%.2f m/s  t=%.4f"
              % (i + 1, args.trials, "+" if sign > 0 else "-", args.speed, t_step),
              flush=True)

    # 收尾: 确保停住
    p.send(0.0, tag="final_stop")
    p.spin_for(0.8)

    data = {
        "config": vars(args),
        "track": TRACK,
        "published": p.published,
        "samples": p.samples,
        "check": {"vx": vx, "wz": wz, "verdict": verdict, "noise_max": noise_max},
    }
    with open(args.out, "w") as f:
        json.dump(data, f)
    print("[probe] 完成: %d 个发布, %d 个 /odom 样本 -> %s"
          % (len(p.published), len(p.samples), args.out), flush=True)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
