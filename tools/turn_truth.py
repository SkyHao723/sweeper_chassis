#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原地转向的"真值"对照: 里程计说的转速 vs IMU 实测的转速

为什么要这个
------------
chassis_check.py 说原地转 0.5 rad/s 只转出了 43%, 但直接看 STM32 的遥测,
轮子确实在按 ±21 RPM 转, 轮速反推的 wz 也有 92%~129%。两边对不上。

轮速到 /odom 的链路是:
    STM32 轮速 -> 24B 遥测 [6-7]=wz -> 厂商节点直接取用 -> /odom.twist.angular.z
也就是 /odom 的 wz 就是 STM32 自己算的那个数, 中间没有别的加工。所以光看
轮子和里程计是分不出谁错的 —— 它们本来就是同一个数。

必须找一个跟轮子完全无关的参照: **IMU 陀螺仪**。
    IMU 积分 ~68°  -> 车真的转了, 是 /odom 报少了   (软件/标定问题)
    IMU 积分 ~29°  -> 车真的没转够, 是轮子出力不够  (驱动器/控制问题)

顺便验证陀螺仪 Z 轴符号: 原地左转(wz>0, 俯视逆时针), IMU 的
angular_velocity.z 应该**为正**。

用法
----
    python3 ~/chassis_tools/turn_truth.py 0.5 4      # 左转, 4 秒
    python3 ~/chassis_tools/turn_truth.py -0.5 4     # 右转
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SAMPLE_DT = 0.01      # 采样周期
PUB_HZ = 20.0

# rows 的列顺序 —— 第 0 列是时间戳, 数据从第 1 列开始。
# ★ 这里踩过坑: 直接用 0/1/2 当数据列, 结果把时间戳当成 /odom 的角速度,
#   积分出 4e11 度这种鬼数。别再直接写数字了。
COL_T = 0
COL_ODOM = 1
COL_EKF = 2
COL_IMU = 3


class Rec(Node):
    def __init__(self):
        super().__init__('turn_truth')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rows = []                      # (t, odom_wz, comb_wz, imu_wz)
        self.n = [0, 0, 0]
        self.cur = [None, None, None]
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Odometry, '/odom_combined', self.cb_comb, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 50)
        self.create_timer(SAMPLE_DT, self.sample)

    def cb_odom(self, m):
        self.n[0] += 1
        self.cur[0] = m.twist.twist.angular.z

    def cb_comb(self, m):
        self.n[1] += 1
        self.cur[1] = m.twist.twist.angular.z

    def cb_imu(self, m):
        self.n[2] += 1
        self.cur[2] = m.angular_velocity.z

    def sample(self):
        self.rows.append((time.time(), self.cur[0], self.cur[1], self.cur[2]))


def integrate(rows, col, t0, t1):
    """梯形积分 rows[col] 在 [t0,t1] 上的积分; 返回 (积分值, 有效采样数)"""
    pts = [(r[0], r[col]) for r in rows if t0 <= r[0] <= t1 and r[col] is not None]
    if len(pts) < 2:
        return None, len(pts)
    s = 0.0
    for i in range(1, len(pts)):
        dt = pts[i][0] - pts[i - 1][0]
        s += 0.5 * (pts[i][1] + pts[i - 1][1]) * dt
    return s, len(pts)


def mean_rate(rows, col, t0, t1):
    pts = [r[col] for r in rows if t0 <= r[0] <= t1 and r[col] is not None]
    if not pts:
        return None
    return sum(pts) / len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('wz', type=float, help='角速度命令 rad/s (左转为正)')
    ap.add_argument('dur', type=float, nargs='?', default=4.0, help='持续时间 秒')
    args = ap.parse_args()

    rclpy.init()
    node = Rec()

    # --- 等数据到位 (绝不假设 1 秒就够 —— DDS 发现可能要好几秒) ---
    t_wait = time.time()
    while time.time() - t_wait < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.n[0] > 5 and node.n[2] > 5:
            break
    print('数据到位: /odom %d 条, /odom_combined %d 条, /imu/data_raw %d 条'
          % (node.n[0], node.n[1], node.n[2]))
    if node.n[0] == 0:
        print('!! /odom 收不到, 底盘栈是不是没起来?')
    if node.n[2] == 0:
        print('!! /imu/data_raw 收不到 —— 陀螺仪对照做不了')
        print('   先看看话题名: ros2 topic list | grep -i imu')
    if node.n[0] == 0 or node.n[2] == 0:
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # --- ★ 污染自检 ---------------------------------------------------
    # 如果 /cmd_vel 上除了本脚本还有别的**在发**的发布者, 这次测量毫无意义 ——
    # 底盘节点会同时收到两条互相冲突的速度命令, 谁后到听谁的, 结果是一个随机
    # 混合的指令。
    # ★ 但判据**不能只看发布者数量**: 实测 chassis-web.service 开机自启, 它一
    #   启动就 create_publisher, 却只在"网页使能 + 心跳新鲜"时才真发。按数量判
    #   会平白无故中止一次正常的测量(已踩过)。正确做法是**在本脚本还没发任何
    #   东西之前, 先听 2 秒**: 有消息才是真有人在发。
    t_disc = time.time()
    heard = []
    node.create_subscription(Twist, '/cmd_vel', lambda m: heard.append(m), 10)
    while time.time() - t_disc < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if heard:
        print()
        print('!! 本脚本还没开始发, /cmd_vel 上就来了 %d 条消息 —— 有别人在发速度。'
              % len(heard))
        print('   这样测出来的东西没有意义, 已中止。先找出是谁:')
        print('     ros2 topic info /cmd_vel --verbose')
        print('     ros2 node list')
        node.destroy_node()
        rclpy.shutdown()
        return 2
    print('污染自检: 静止 2 秒内 /cmd_vel 上 0 条消息 -> 没有别人在发, 可以测')

    # 先空转 1 秒, 把静止时的基线收进去
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    cmd = Twist()
    cmd.angular.z = args.wz
    period = 1.0 / PUB_HZ
    t_start = time.time()
    t_next = t_start
    while time.time() - t_start < args.dur:
        now = time.time()
        if now >= t_next:
            node.pub.publish(cmd)
            t_next += period
        rclpy.spin_once(node, timeout_sec=0.005)
    t_end = time.time()

    # 停车: 按 20Hz 发零速 1.5 秒(和正常控制一个节奏), 再看残余
    stop = Twist()
    t_brk = time.time()
    t_next = t_brk
    while time.time() - t_brk < 1.5:
        now = time.time()
        if now >= t_next:
            node.pub.publish(stop)
            t_next += period
        rclpy.spin_once(node, timeout_sec=0.005)
    t_stop = time.time()
    t_res = t_stop - 0.5      # 残余只看最后 0.5 秒, 避开刹车那段过渡

    # --- 汇总 ---
    win = t_end - t_start
    exp = args.wz * win
    print()
    print('=' * 96)
    print('原地转向真值对照   命令 wz = %+.3f rad/s, 持续 %.2f s, 期望转过 %+.1f°'
          % (args.wz, win, exp * 57.2958))
    print('=' * 96)
    print('%-22s %14s %12s %12s' % ('来源', '积分角度(°)', '占期望', '窗口内平均速率'))
    print('-' * 96)
    for col, name in ((COL_ODOM, '/odom  (纯轮速)'),
                      (COL_EKF, '/odom_combined (EKF)'),
                      (COL_IMU, '/imu/data_raw (陀螺仪)')):
        val, npts = integrate(node.rows, col, t_start, t_end)
        mr = mean_rate(node.rows, col, t_start, t_end)
        if val is None:
            print('%-22s %14s %12s %12s   (只有 %d 个采样)'
                  % (name, '无数据', '-', '-', npts))
        else:
            pct = (val / exp * 100.0) if abs(exp) > 1e-9 else float('nan')
            print('%-22s %+14.1f %11.0f%% %+12.3f'
                  % (name, val * 57.2958, pct, mr if mr is not None else float('nan')))

    print('-' * 96)
    # 稳态: 最后 1 秒
    ss0 = t_end - 1.0
    for col, name in ((COL_ODOM, '/odom'), (COL_IMU, 'IMU  ')):
        mr = mean_rate(node.rows, col, ss0, t_end)
        if mr is not None:
            print('最后 1 秒平均速率 %s: %+.3f rad/s  (命令的 %+.0f%%)'
                  % (name, mr, mr / args.wz * 100.0 if abs(args.wz) > 1e-9 else float('nan')))

    # 逐秒明细, 直接看爬升用了多久
    print()
    print('逐秒平均角速率 (rad/s)  —— 看爬升和刹车的形状')
    print('%6s %12s %12s %12s' % ('秒', '/odom', 'EKF', 'IMU'))
    k = 0
    while True:
        a = t_start + k
        b = min(a + 1.0, t_end)
        if a >= t_end:
            break
        o = mean_rate(node.rows, COL_ODOM, a, b)
        c = mean_rate(node.rows, COL_EKF, a, b)
        m = mean_rate(node.rows, COL_IMU, a, b)
        f = lambda v: ('%+12.3f' % v) if v is not None else '%12s' % '-'
        print('%6.1f %s %s %s' % (k, f(o), f(c), f(m)))
        k += 1

    # 停车后的残余 (1 秒)
    print()
    for col, name in ((COL_ODOM, '/odom'), (COL_IMU, 'IMU  ')):
        mr = mean_rate(node.rows, col, t_res, t_stop)
        if mr is not None:
            print('停车 1 秒后(取最后 0.5 秒)平均速率 %s: %+.4f rad/s  (应当接近 0)'
                  % (name, mr))

    print()
    print('怎么读:')
    print('  * IMU 和 /odom 都远低于期望 -> 车真的没转够, 是驱动器低速出力的问题,')
    print('    不是里程计的问题。这种情况下 EKF 也会跟着偏, 因为它的偏航角速度')
    print('    主要就来自这个 IMU。')
    print('  * IMU 接近期望而 /odom 偏低 -> 车转了但里程计少报, 是软件侧的问题。')
    print('  * IMU 积分和 /odom 符号相反 -> 陀螺仪 Z 轴装反了, 改 IMU_GYRO_Z_SIGN。')

    node.destroy_node()
    rclpy.shutdown()

    # ★ 强制退出。rclpy/DDS 的关闭偶尔会卡住, 而一个卡住的进程会一直占着
    #   DDS 资源和订阅 —— 更糟的是如果它还在发 /cmd_vel, 后面的测量就全废。
    #   一次性诊断脚本不值得为"优雅关闭"冒这个风险。
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
