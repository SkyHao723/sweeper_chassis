#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证固件"命令超时 = 软停"语义（心跳机制已于 2026-11-07 移除）。

模拟键盘 teleop 的两段行为：
  阶段 A（按住 1.2 s，30 Hz 持续发 vx）→ 车应稳定跟随 vx
  阶段 B（松手：一条命令都不发）      → 800 ms 看门狗触发，按
                                       RAMP_TIMEOUT_DEC=0.70 m/s² 软停
对照组 C：只发一条 vx 就停手 → 同样应在 ~0.8 s + 斜坡后停下（不再锁存）

用法（在 192.168.5.7 上，先启动底盘节点）：
    python3 cmd_timeout_test.py [vx]
退出码 0 = 两段都符合预期，1 = 不符合。
"""
from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
CMD_HZ = 30.0
HOLD_S = 1.2                       # 按住阶段
SILENCE_S = 1.8                    # 松手后观察（800 ms 看门狗 + 斜坡）
CMD_TIMEOUT_S = 0.80               # 固件 CMD_TIMEOUT_MS
TIMEOUT_DEC = 0.70                 # 固件 RAMP_TIMEOUT_DEC (m/s²)

rclpy.init()
node = rclpy.create_node("cmd_timeout_test")
pub = node.create_publisher(Twist, "/cmd_vel", 10)

samples: list[tuple[float, float]] = []


def cb(msg):                        # /odom 回调
    samples.append((time.time(), msg.twist.twist.linear.x))


node.create_subscription(Odometry, "/odom", cb, 10)


def spin_until(t_end: float) -> None:
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)


def send(vx: float) -> None:
    tw = Twist()
    tw.linear.x = vx
    pub.publish(tw)


def mean_in(t_from: float, t_to: float) -> float | None:
    sel = [v for (ts, v) in samples if t_from <= ts <= t_to]
    return sum(sel) / len(sel) if sel else None


def fmt(x: float | None) -> str:
    return "  n/a" if x is None else "%+.3f" % x


# ---- 等 /odom 上来 ----
t0 = time.time()
while not samples and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if not samples:
    print("FAIL: 没收到 /odom（底盘节点起来了吗？ROS_DOMAIN_ID 对吗？）")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(2)

# ---- 阶段 A：按住 1.2 s，30 Hz 持续发 ----
send(0.0)
spin_until(time.time() + 0.3)
t_hold = time.time()
next_tx = t_hold
while time.time() < t_hold + HOLD_S:
    now = time.time()
    if now >= next_tx:
        send(VX)
        next_tx += 1.0 / CMD_HZ
    rclpy.spin_once(node, timeout_sec=0.005)
t_release = time.time()

# ---- 阶段 B：松手，一条都不发 ----
spin_until(t_release + SILENCE_S)

# ---- 分析 ----
a_mid = mean_in(t_release - 0.5, t_release)              # 按住末尾
b_early = mean_in(t_release + 0.5, t_release + CMD_TIMEOUT_S)   # 超时前（还在走）
b_late = mean_in(t_release + CMD_TIMEOUT_S + 0.6, t_release + SILENCE_S)  # 应已停
expect_stop_s = CMD_TIMEOUT_S + VX / TIMEOUT_DEC          # 理论停下时刻

print("命令 vx=%+.3f m/s，按住 %.1f s（%.0f Hz），然后松手" %
      (VX, HOLD_S, CMD_HZ))
print("  t(s)   vx(/odom)")
for k in range(0, 15):
    t = k * 0.2
    print("  %4.1f   %s" % (t, fmt(mean_in(t_release + t - 0.1,
                                          t_release + t + 0.1))))
print()
print("A 按住末尾 (t=-0.5..0)      : %s   期望 ≈ %+.3f" % (fmt(a_mid), VX))
print("B 松手后 0.5..0.8 s（超时前）: %s   期望 > 0（还在走）" % fmt(b_early))
print("C 松手后 1.4..1.8 s（应已停）: %s   期望 ≈ 0（理论停下 %.2f s）"
      % (fmt(b_late), expect_stop_s))

ok_hold = a_mid is not None and a_mid > VX * 0.5
ok_stop = b_late is not None and abs(b_late) < max(0.02, VX * 0.25)
print()
print("按住段跟随: %s" % ("PASS" if ok_hold else "FAIL（/odom 速度没跟上命令）"))
print("松手自动软停: %s" % ("PASS" if ok_stop
                       else "FAIL（松手 1.8 s 后还在走 → 看门狗没生效）"))
verdict = "PASS" if (ok_hold and ok_stop) else "FAIL"
print("结论: %s" % verdict)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if verdict == "PASS" else 1)
