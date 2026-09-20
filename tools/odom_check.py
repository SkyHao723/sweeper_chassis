#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程计标定：用手推车走一段量好的距离，比对 /odom 报了多少。

                        ⚠️ 本车实测：这个方法**无效**，先看下面 ⚠️

============================== 重要 ==============================
本车（两驱差速底盘 + FOC 驱动器）实测：**用手推车时两路轮速不会同时上报，
永远只有一路在报数**。推 1 米采到 344 个有轮速的帧，其中
    341 帧只有左轮有数，3 帧只有右轮，**0 帧两路都有**。

后果是两个现象一起出现，而且数值能对上：
    路径长度 = (左 + 0)/2  ->  正好少一半   （实测 45%~55%）
    航向     = (0 - 左)/b  ->  巨大的假转向 （实测 -68.6°）
"走 0.55 米却算出右轮比左轮多走 1.11 米"在物理上不可能，所以必然有一路
是坏的。**所以手推标定测出来的不是里程计误差，不能用。**

正常行驶（发命令让车自己走）时两路都是好的，所以这个问题特定于
"被外部反拖"这个场景。

正确的标定方法：**直接量轮子**
    轮胎上贴个标记，慢慢推正好 2.00 米，数轮子转了几圈。
    真实轮周长 = 2.00 / 圈数，再和固件里的 WHEEL_DIAMETER_MM 比。
只依赖眼睛和卷尺，完全绕开上报不可靠的问题。

**另外**：也不能用"发命令让车走，比命令值 vs /odom"来标轮径 ——
命令换算和里程计换算用的是**同一个轮径常数**，比值恒等于 1，测不出轮径
对不对（这个方法论错误踩过）。发命令只能测"控制误差"。

本脚本保留下来是为了留证据/复现那个"只有一路在报数"的现象。

注意两点：
  1. 运行期间**持续发零速喂看门狗** —— 否则 800ms 后看门狗会切断电机继电器，
     驱动器断电后编码器就不出数了，里程计直接停住。
  2. 推**慢一点**。待机状态是动态制动，但制动力随转速下降（"慢慢推很轻"）。

用法：
    python3 tools/odom_check.py --seconds 20
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


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Track(object):
    """记录一条轨迹: 路径长度(累积)、起终点直线距离、航向变化。"""

    def __init__(self):
        self.pts = []

    def add(self, x, y, yaw):
        self.pts.append((x, y, yaw))

    def path_len(self):
        n = 0.0
        for i in range(1, len(self.pts)):
            dx = self.pts[i][0] - self.pts[i - 1][0]
            dy = self.pts[i][1] - self.pts[i - 1][1]
            n += math.hypot(dx, dy)
        return n

    def straight(self):
        if len(self.pts) < 2:
            return 0.0
        dx = self.pts[-1][0] - self.pts[0][0]
        dy = self.pts[-1][1] - self.pts[0][1]
        return math.hypot(dx, dy)

    def dyaw(self):
        return wrap(self.pts[-1][2] - self.pts[0][2]) if len(self.pts) >= 2 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--seconds", type=float, default=20.0,
                    help="采集多少秒(这期间推车), 默认 20")
    ap.add_argument("--actual", type=float, default=None,
                    help="用卷尺量到的实际前进距离(米), 给了就直接算比例系数")
    ap.add_argument("--turn", type=float, default=None,
                    help="如果做的是原地转圈, 这里填实际转过的角度(度)")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("odom_check")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    raw = Track()      # /odom        车轮里程计
    ekf = Track()      # /odom_combined

    def on_odom(m):
        p = m.pose.pose
        raw.add(p.position.x, p.position.y, yaw_of(p.orientation))

    def on_comb(m):
        p = m.pose.pose
        ekf.add(p.position.x, p.position.y, yaw_of(p.orientation))

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(Odometry, "/odom_combined", on_comb, 10)

    print("=" * 72)
    print("里程计标定  —  现在用手把车推过一段量好的距离")
    print("  持续 %.0f 秒。推慢一点, 让轮子滚动别打滑。" % args.seconds)
    print("  期间会持续发零速喂看门狗(否则继电器会断电、编码器停摆)。")
    print("=" * 72)

    zero = Twist()
    n = int(args.seconds * PUB_HZ)
    for i in range(n):
        pub.publish(zero)              # 喂看门狗, 不然会触发失效保护
        rclpy.spin_once(node, timeout_sec=1.0 / PUB_HZ)
        if i % int(PUB_HZ * 5) == 0 and i:
            print("  ... 还剩 %.0f 秒   当前 /odom 路径 %.3f m"
                  % (args.seconds - i / PUB_HZ, raw.path_len()))

    print()
    if len(raw.pts) < 5:
        print("!! /odom 数据太少, 检查底盘节点在不在跑")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("-" * 72)
    print("%-16s %12s %12s" % ("", "/odom(轮速)", "/odom_combined(EKF)"))
    print("%-16s %12.3f %12.3f" % ("路径长度 m", raw.path_len(), ekf.path_len()))
    print("%-16s %12.3f %12.3f" % ("起终点直线 m", raw.straight(), ekf.straight()))
    print("%-16s %12.1f %12.1f" % ("航向变化 deg",
                                   math.degrees(raw.dyaw()), math.degrees(ekf.dyaw())))
    print("-" * 72)

    if args.actual:
        d = raw.path_len()
        print("实际推了 %.3f m, /odom 报 %.3f m  ->  比例 %.4f"
              % (args.actual, d, d / args.actual))
        print("想让 /odom 报准, 把 `odom_x_scale` 乘上 %.4f"
              % (args.actual / d))
        print("(在 launch/base_serial.launch.py 里, 现在 odom_x_scale=1.0)")
    else:
        print("拿卷尺量一下实际推了多远, 然后:")
        print("  比例 = /odom 报的路径长度 / 实际距离")
        print("  或者重跑一次并加上 --actual <实际米数> 直接算系数")

    if abs(math.degrees(raw.dyaw())) > 5:
        print()
        print("注意: 这次航向变了 %.1f 度 —— 如果本意是走直线, 那说明推歪了,"
              % math.degrees(raw.dyaw()))
        print("      路径长度会偏大。想要纯直线标定请重新推直一点。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
