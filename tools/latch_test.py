#!/usr/bin/env python3
# 验证"心跳模式 = 目标锁存": 只发一条 /cmd_vel, 看车是否继续走超过 800 ms
# 用法: python3 latch_test.py [vx]
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.06
HOLD_S = 1.6          # 单条命令后观察时长 (> 800 ms 老看门狗)
TAIL_S = 1.2          # 发 0 之后观察时长

rclpy.init()
node = rclpy.create_node("latch_test")
pub = node.create_publisher(Twist, "/cmd_vel", 10)

samples = []


def cb(msg):
    samples.append((time.time(), msg.twist.twist.linear.x))


node.create_subscription(Odometry, "/odom", cb, 10)

# 等 /odom 上来
t0 = time.time()
while not samples and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if not samples:
    print("FAIL: 没收到 /odom")
    sys.exit(2)

tw = Twist()
tw.linear.x = VX
pub.publish(tw)
t_cmd = time.time()

t_end = t_cmd + HOLD_S
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.02)

tw2 = Twist()
tw2.linear.x = 0.0
pub.publish(tw2)
t_zero = time.time()

t_end = t_zero + TAIL_S
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.02)

print(f"命令 vx={VX:+.3f} m/s 只发了一条, 之后不再发任何命令")
print("  t(s)   vx(/odom)")
for k in range(0, 16):
    t = k * 0.2
    sel = [v for (ts, v) in samples if t_cmd + t - 0.1 <= ts <= t_cmd + t + 0.1]
    if sel:
        print(f"  {t:4.1f}   {sum(sel)/len(sel):+.3f}")
print(f"发 0 后 (相对发 0 时刻):")
for k in range(0, 7):
    t = k * 0.2
    sel = [v for (ts, v) in samples if t_zero + t - 0.1 <= ts <= t_zero + t + 0.1]
    if sel:
        print(f"  {t:4.1f}   {sum(sel)/len(sel):+.3f}")

# 判定: 单条命令后 1.0~1.5 s 还在动 => 目标锁存
late = [v for (ts, v) in samples if t_cmd + 1.0 <= ts <= t_cmd + 1.5]
if late:
    mean_late = sum(late) / len(late)
    verdict = "LOCKED(目标锁存 OK)" if mean_late > VX * 0.5 else "NOT_LOCKED(还在动? 否)"
    print(f"1.0~1.5 s 平均 vx={mean_late:+.3f} -> {verdict}")
else:
    print("没有 1.0~1.5 s 的样本")
print("样例数:", len(samples))
node.destroy_node()
rclpy.shutdown()
