#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直线质量评估: 量化"忽快忽慢"和"跑偏"

为什么不用卷尺
--------------
命令 wz=0 时, **IMU 陀螺仪积分出来的偏航角就是"车实际歪了多少度"** —— 它是
独立物理量, 比"量横向偏移再除以距离"更直接。横向偏移量的是结果, IMU 量的是
原因, 而且 4 米偏离 20cm 到底是"一路慢慢歪"还是"起步那一下歪了然后直的",
只有看时间序列才能分清。

速度那边直接从 /odom 的 twist.linear.x 看(那是驱动器回传的实际轮速正解出来的,
就是"车真实在跑多快"的电气侧真相)。

输出的三组数, 分别回答三个问题
------------------------------
  1. 真实偏航(IMU 积分) + 轮速里程计偏航
       -> 车实际歪了多少; 里程计报的偏航离真值多远(里程计的诚实度)
  2. 速度 均值/标准差/极值 + 主振荡频率
       -> "忽快忽慢"到底有多大, 周期多长
         如果频率在 0.2~1Hz, 那就是外环和驱动器速度环互相激励的慢极限环;
         如果频率更高(几 Hz), 更像驱动器自己的速度环在振。
  3. IMU 的 wz 均值
       -> 直行时理论上应该≈0。明显非零 = 陀螺零偏, 而零偏会伪装成"跑偏"。

用法
----
    python3 ~/chassis_tools/straight_quality.py 0.30 10     # 0.30m/s 跑 10 秒
    python3 ~/chassis_tools/straight_quality.py 0.30 10 --trim-off   # (见下)

★ 跑之前先确认 /cmd_vel 上没有别的发布者, 脚本会自检。
"""
import argparse
import collections
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SAMPLE_DT = 0.01
PUB_HZ = 20.0

COL_T, COL_ODX, COL_ODZ, COL_CBX, COL_CBZ, COL_IMU, COL_ACC = range(7)


class Rec(Node):
    def __init__(self):
        super().__init__('straight_quality')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rows = []
        self.n = [0, 0, 0]
        # 第 6 列记加速度模长。它的唯一用途是**判断 IMU 是不是死的** —— 见 main 里
        # 的检查。真实加速度计无论如何都会读到重力(约 9.8), 所以"三轴全零"是
        # 铁证; 而陀螺仪在静止时输出 0 是这颗模块的正常行为, **不能**用来判死活。
        self.cur = [None] * 6
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.create_subscription(Odometry, '/odom_combined', self.cb_comb, 20)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, 50)
        self.create_timer(SAMPLE_DT, self.sample)

    def cb_odom(self, m):
        self.n[0] += 1
        self.cur[0] = m.twist.twist.linear.x
        self.cur[1] = m.twist.twist.angular.z

    def cb_comb(self, m):
        self.n[1] += 1
        self.cur[2] = m.twist.twist.linear.x
        self.cur[3] = m.twist.twist.angular.z

    def cb_imu(self, m):
        self.n[2] += 1
        self.cur[4] = m.angular_velocity.z
        a = m.linear_acceleration
        self.cur[5] = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)

    def sample(self):
        self.rows.append((time.time(),) + tuple(self.cur))


def integrate(rows, col, t0, t1):
    """梯形积分; 返回 (积分值, 采样数)"""
    pts = [(r[COL_T], r[col]) for r in rows
           if t0 <= r[COL_T] <= t1 and r[col] is not None]
    if len(pts) < 2:
        return None, len(pts)
    s = 0.0
    for i in range(1, len(pts)):
        s += 0.5 * (pts[i][1] + pts[i - 1][1]) * (pts[i][0] - pts[i - 1][0])
    return s, len(pts)


def stats(rows, col, t0, t1):
    v = [r[col] for r in rows if t0 <= r[COL_T] <= t1 and r[col] is not None]
    if len(v) < 4:
        return None
    m = sum(v) / len(v)
    var = sum((x - m) ** 2 for x in v) / len(v)
    s = sorted(v)

    def pct(p):
        return s[min(len(s) - 1, int(p * len(s)))]

    return {'n': len(v), 'mean': m, 'std': math.sqrt(var),
            'min': min(v), 'max': max(v), 'series': v,
            # ★ 分位数是**稳健统计**: 峰峰值和标准差都会被单帧残值撑大。
            #   实测踩到: 轮速里偶尔有一帧精确 0(驱动器回码超过 RPM_STALE_MS
            #   时固件把实际转速当成 0 上报 —— 那是残值不是真停), 于是
            #   min=0、峰峰值虚高、连"均值穿越次数"估出来的振荡频率都被带偏。
            'p05': pct(0.05), 'p50': pct(0.50), 'p95': pct(0.95),
            'n_zero': sum(1 for x in v if abs(x) < 1e-6)}


def osc_freq(st, dt):
    """用均值穿越次数估主振荡频率。只对"确实在振"的信号有意义。"""
    if st is None or st['std'] < 1e-4:
        return 0.0
    v = st['series']
    m = st['mean']
    cross = 0
    for i in range(1, len(v)):
        if (v[i - 1] - m) * (v[i] - m) < 0:
            cross += 1
    dur = len(v) * dt
    return cross / (2.0 * dur) if dur > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('vx', type=float, help='直行速度 m/s')
    ap.add_argument('dur', type=float, nargs='?', default=10.0, help='持续秒数')
    args = ap.parse_args()

    rclpy.init()
    node = Rec()

    t_wait = time.time()
    while time.time() - t_wait < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.n[0] > 5 and node.n[2] > 5:
            break
    print('数据到位: /odom %d, /odom_combined %d, /imu/data_raw %d'
          % (node.n[0], node.n[1], node.n[2]))
    if node.n[0] == 0 or node.n[2] == 0:
        print('!! 收不到 /odom 或 /imu/data_raw, 底盘栈起来了吗?')
        node.destroy_node(); rclpy.shutdown(); return 1

    # ★ IMU 死活自检 —— 这一步不能省。
    #   本脚本拿 IMU 积分当"偏航真值", 而 IMU 一旦死了, 厂商节点发出来的
    #   angular_velocity.z 就是常量 0 —— 于是脚本会算出"真实偏航 +0.00°",
    #   报出一条完美直线。**拿死 IMU 去验证里程计, 结论会完全反过来。**
    #   判据必须用**加速度计**: 真实加速度计无论如何都读到重力(约 9.8),
    #   三轴全零是铁证。**不能**用陀螺仪判 —— 这颗模块静止时本来就输出 0。
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    accs = [r[COL_ACC] for r in node.rows if r[COL_ACC] is not None]
    if not accs or max(accs) < 1.0:
        print()
        print('!! IMU 没有有效数据: 加速度模长最大只有 %.2f m/s^2'
              % (max(accs) if accs else 0.0))
        print('   真实加速度计静止时也该读到约 9.8(重力), 读不到就是 IMU 死了。')
        print('   **这次测量没有意义** —— 偏航真值就是这颗 IMU, 它算出来会是 0,')
        print('   看着像"完美直线", 结论完全反了。已中止。')
        print('   查: python3 ~/chassis_tools/decode_diag.py -p /dev/wheeltec_controller')
        print('       看 imu= 状态码 (4=无应答 -> 供电/接线/模块本身)')
        node.destroy_node(); rclpy.shutdown(); return 3
    print('IMU 自检: 加速度模长 %.1f~%.1f m/s^2 (读到重力, 活着)'
          % (min(accs), max(accs)))

    # ★ 污染自检 —— 但**绝不能只看发布者的数量**。
    #   实测踩到: chassis-web.service 开机自启, 它一启动就 create_publisher,
    #   但设计上**只在"网页使能 + 心跳新鲜"时才真发**(见 chassis_web.py 的说明:
    #   "否则本页面 20Hz 的零速会和 Nav2 抢 /cmd_vel, 车会一抖一抖")。
    #   于是"有 2 个发布者"里有一个是完全静默的, 按数量判断会**平白无故中止
    #   一次本来正常的测量**。
    #   正确判据: **在本脚本还没发任何东西之前, 先听 2 秒 /cmd_vel。**
    #     收到消息 -> 确实有别人在发, 中止
    #     一条都没有 -> 那个发布者是静的, 可以测
    n_pub = node.count_publishers('/cmd_vel')
    heard = []
    node.create_subscription(Twist, '/cmd_vel', lambda m: heard.append(m), 10)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if heard:
        print()
        print('!! 本脚本还没开始发, /cmd_vel 上就已经来了 %d 条消息 —— '
              '确实有别人在发速度。' % len(heard))
        print('   两个源同时发会让底盘"谁后到听谁的", 表现是忽快忽慢 + 偏航乱,')
        print('   而且完全不可复现。已中止。先查是谁:')
        print('     ros2 topic info /cmd_vel --verbose   # 看发布者节点名')
        print('     ros2 node list')
        node.destroy_node(); rclpy.shutdown(); return 2
    print('污染自检: /cmd_vel 上有 %d 个发布者, 但静止 2 秒内 0 条消息'
          ' -> 那个是静的, 可以测' % n_pub)

    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    cmd = Twist()
    cmd.linear.x = args.vx
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

    win = t_end - t_start
    print()
    print('=' * 92)
    print('直线质量: 命令 vx = %+.3f m/s, 持续 %.2f s (t_wall %.2f s)'
          % (args.vx, args.dur, win))
    print('=' * 92)

    # ---- 1. 偏航 ----
    yaw_imu, _ = integrate(node.rows, COL_IMU, t_start, t_end)
    yaw_od, _ = integrate(node.rows, COL_ODZ, t_start, t_end)
    yaw_cb, _ = integrate(node.rows, COL_CBZ, t_start, t_end)
    print('【偏航】命令 wz=0, 所以下面每个数都是"车歪了多少"')
    for name, v in (('真实偏航 (IMU 陀螺积分)', yaw_imu),
                    ('/odom 报的偏航 (纯轮速)', yaw_od),
                    ('/odom_combined (EKF)', yaw_cb)):
        if v is None:
            print('  %-28s 无数据' % name)
        else:
            print('  %-28s %+8.2f°' % (name, v * 57.2958))
    if yaw_imu is not None and yaw_od is not None:
        print('  %-28s %+8.2f°   (里程计偏航相对真值的误差)'
              % ('  两者之差', (yaw_od - yaw_imu) * 57.2958))

    imu_st = stats(node.rows, COL_IMU, t_start, t_end)
    if imu_st:
        # ★ 均值是**零偏**, 标准差是**噪声** —— 这两个要分开看:
        #   零偏会把直行积成一条弧线(可以扣), 噪声不会造成系统性偏差, 但会决定
        #   滤波器该信它多少。厂商给 EKF 的 angular_velocity_covariance[8] 是
        #   2.5e-3(标准差 0.05 rad/s); 实测行驶中陀螺尖峰能到 ±0.5 rad/s,
        #   如果这里的标准差远大于 0.05, 说明 EKF 把这个噪声源信得太死了。
        print('  IMU wz 均值 %+.4f rad/s (零偏; 直行时应≈0)'
              % imu_st['mean'])
        print('  IMU wz 标准差 %.4f rad/s (噪声; 厂商给 EKF 的是 0.05)'
              % imu_st['std'])
        print('    -> 按实测该填的协方差 = 标准差^2 = %.2e'
              % (imu_st['std'] ** 2))
        if imu_st['std'] > 0.10:
            print('    ★ 噪声明显大于厂商假设的 0.05 -> EKF 把 IMU 信得太死,')
            print('      而它的偏航权重还是轮速的 20 倍。值得按实测改协方差。')

    # ---- 2. 速度 ----
    print()
    print('【速度】这就是"忽快忽慢"的量化')
    od_st = stats(node.rows, COL_ODX, t_start, t_end)
    cb_st = stats(node.rows, COL_CBX, t_start, t_end)
    for name, st in (('/odom (驱动器回传轮速)', od_st),
                     ('/odom_combined (EKF)', cb_st)):
        if st is None:
            print('  %-24s 无数据' % name)
            continue
        print('  %-24s 均值 %+.3f m/s (%+5.0f%%)  标准差 %.3f  '
              '5~95%%区间 %+.3f~%+.3f'
              % (name, st['mean'], st['mean'] / args.vx * 100.0,
                 st['std'], st['p05'], st['p95']))
        if st['n_zero']:
            print('      ⚠ 其中 %d/%d 帧是精确 0 —— 多半是驱动器回码超时'
                  '(RPM_STALE_MS=200ms)' % (st['n_zero'], st['n']))
            print('        被固件当成"实际转速 0"上报的**残值**, 不是车真停了。'
                  '它会撑大 min/max 和标准差。')
    if od_st:
        rng = od_st['max'] - od_st['min']
        rng95 = od_st['p95'] - od_st['p05']
        f = osc_freq(od_st, SAMPLE_DT)
        print('    -> 峰峰值 %.3f m/s (命令的 %.0f%%)  ← 会被单帧残值撑大'
              % (rng, rng / abs(args.vx) * 100.0))
        print('       5~95%% 区间宽度 %.3f m/s (命令的 %.0f%%)  ← 这个更可信'
              % (rng95, rng95 / abs(args.vx) * 100.0))
        print('       主振荡约 %.2f Hz  ← 单帧残值会让这个数虚高, 配合上面看'
              % f)
        print('    判据: 5~95% 宽度 <20% 命令 -> 算稳; 20~50% -> 明显在忽快忽慢;')
        print('          >50% -> 是极限环, 必须治')
        if 0.15 <= f <= 1.2:
            print('    ★ 这个频段本来最像外环(轮速修正)的积分造成的慢极限环 ——')
            print('      那个已经修掉了(TRIM_LIN_BAND_RPM: 误差大就不积分),')
            print('      验证方式是 decode_diag 的"目标-理论"那一列变平(实测 ±1 RPM)。')
            print('      **如果它还是平的而这里仍在振, 那就不是外环, 是驱动器自己')
            print('      的速度环** —— 外环够不着那一层, 只能走扭矩模式。')
            print('      另外先确认这个 f 不是被单帧残值撑出来的(看上面的 5~95% 宽度)。')
        elif f > 1.2:
            print('    ★ 频率偏高: 更像驱动器自己的速度环在振, 外环够不着这一层。')

    # ---- 3. 距离 ----
    print()
    print('【距离】')
    for name, col in (('/odom 轮速里程', COL_ODX),
                      ('/odom_combined', COL_CBX)):
        d, _ = integrate(node.rows, col, t_start, t_end)
        if d is not None:
            print('  %-24s %+.3f m   (命令 %.3f m, %+.0f%%)'
                  % (name, d, args.vx * win, d / (args.vx * win) * 100.0))
    print('  ★ 里程计距离**不能**自己验证刻度(它和命令同源), 要量真实距离只能用卷尺。')
    print('    这里只用来和 /odom 互相对照。')

    # ---- 逐秒 ----
    print()
    print('逐秒平均 (m/s)   —— 看"忽快忽慢"是不是周期性的')
    print('%6s %10s %10s' % ('秒', '/odom', 'IMU'))
    k = 0
    while t_start + k < t_end:
        a = t_start + k
        b = min(a + 1.0, t_end)
        o = stats(node.rows, COL_ODX, a, b)
        i = stats(node.rows, COL_IMU, a, b)
        print('%6.1f %10s %10s'
              % (k, '%+.3f' % o['mean'] if o else '-',
                 '%+.3f' % i['mean'] if i else '-'))
        k += 1

    print()
    print('停车 1 秒后 平均速率 /odom %s'
          % ('%+.4f rad/s' % (stats(node.rows, COL_ODZ, t_stop - 0.5, t_stop) or
                              {'mean': float('nan')})['mean']))

    node.destroy_node()
    rclpy.shutdown()
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
