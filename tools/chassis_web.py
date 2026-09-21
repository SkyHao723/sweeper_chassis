#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘数据网页 —— 在 RK3588 上跑, 局域网浏览器直接打开看

    ssh 到车之后:  python3 ~/chassis_tools/chassis_web.py
    然后浏览器开:  http://192.168.5.17:8080

================================ 一个刻意的取舍 ==================================

**只用标准库, 不装任何包。**
板子上没有 flask, 而装包要么走 apt 要么走 pip, 都可能和 ROS 的 python 环境
打架(这台车已经因为环境问题绕过好几次)。http.server + json 足够, 单文件就能跑。

================================ 为什么首页主角是那两个数 ======================

这一路查下来, 真正有信息量的就两个量:

  A. **达成率** = /odom 实测车体速度 / /cmd_vel 命令速度
     它直接回答"车有没有照我说的做"。我们花了很久才发现原地转只到 41%,
     又花了更久才发现直线的锅在驱动器内环 —— 都是它先露出来的。

  B. **航向差的"变化量"** = (Δ/odom_combined 偏航) − (Δ/odom 偏航), 基准是页面打开时
     它回答"EKF 到底有没有在纠偏"。两者几乎重合时说明 IMU 没起作用(或没被融合)。

     ★ 这里必须用**变化量**, 不能用绝对差 —— 两个话题的坐标系原点本来就不一样
       (各自从自己的初始位姿起算), 实测 /odom 偏航 -3.03 rad 而 /odom_combined
       -0.005 rad, 绝对差 173°, 看着像出了大问题, 其实只是原点不同。
       **一个没有意义的数摆在首页, 比不摆更糟** —— 所以服务端干脆不算它,
       由前端拿历史里的两列偏航自己减。

所以它们不是"顺便显示", 而是页面上最大、最显眼的两块。

================================ 关于运动控制 ==================================

这个页面**能控车**, 但它是照"绝不允许出现'浏览器不发了车还在走'"来设计的。
这台车因为控制层的 bug 真失控过一次, 而网页遥控最容易出的就是那一类问题
(标签页切走、WiFi 断、手指离开、笔记本合盖)。

所以:

1. **服务端绝不替浏览器"保持"运动。** 浏览器必须以 10Hz 持续发心跳; 超过
   CMD_TIMEOUT(0.5s) 没收到, 服务端立刻按停车处理。三条独立的兜底:
     浏览器松手 -> 立刻发 0;
     浏览器卡死/断网 -> 服务端 0.5s 超时 -> 0;
     服务端也挂了 -> STM32 自己的 800ms 运动看门狗 -> 0。
   三层都失效才可能出事, 而这个概率不用赌。

2. **必须显式"使能"。** 默认禁止运动, 要点一下才能动; 丢失心跳会自动回到禁止。

3. **限幅在服务端做, 不信浏览器。** vx/wz 都被夹住, 界面上改不了上限。

4. **看不到车就不许动。** /odom 超过 ODOM_MAX_AGE(1s) 没更新, 一律停车 ——
   底盘链路断了(CH340 掉线那种)时, 你的操作是没有反馈的, 那就别动。

5. **不主动霸占 /cmd_vel。** 只在"使能 + 心跳新鲜"时发布; 松开时补发几帧零速
   (STOP_BURST) 就闭嘴。否则本页面 20Hz 的零速会和 Nav2 抢 /cmd_vel, 车会一抖
   一抖。同时页面会显示 /cmd_vel 上还有没有别的发布者。

6. **只读了半天, 关键的两个数还是主角**: 你一边动, 一边就能看到达成率和航向差
   怎么变 —— 这比单独一个遥控器有用得多。

================================ 关于诊断帧 ====================================

STM32 每 50ms 还发一帧 36 字节的扩展诊断帧(轮速目标/实际、驱动器电流、故障码、
继电器状态)。**网页看不到它** —— 那一帧在串口上, 而串口被 wheeltec_robot_node
独占, 厂商节点也不解析它。想在网页里显示, 得先给厂商节点加解析+转发(和我们加
/chassis_relay 是同一套做法)。这是明确的下一步, 不是"忘了"。
"""
import argparse
import collections
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32

HIST_MAX = 600          # 滚动保留 600 帧 ≈ 30 秒 @20Hz
WINDOW = 200            # 每次给网页最近 200 帧 ≈ 10 秒
RATE_WINDOW = 5.0       # 频率统计窗口(秒)

RANGE_TOPICS = ["/ultrasonic_data_" + c for c in "ABCDEF"]

# ---- 运动控制的安全参数 (都写死在服务端, 界面上改不了) ----
CMD_TIMEOUT = 0.5        # 浏览器这么久没发心跳 -> 立刻按停车处理
ODOM_MAX_AGE = 1.0       # /odom 这么久没更新 -> 不许动 (底盘链路可能断了)
CTRL_HZ = 20             # 发布 /cmd_vel 的频率
STOP_BURST = 8           # 松开/失能时补发几帧零速再闭嘴 (8 帧 @20Hz = 0.4s)
PUB_CHECK_EVERY = 1.0    # 多久查一次 /cmd_vel 上还有没有别的发布者
VX_LIMIT_DEFAULT = 0.35  # m/s。实测工作包线 0.25~0.35 最好, >0.4 超速
WZ_LIMIT_DEFAULT = 1.2   # rad/s

# 历史每行的列序。**前 PAGE_N 列是页面画图用的, 顺序不能改** —— 前端按列号取值。
# 后面的列只给导出用, 所以页面轮询时会被裁掉(见 payload), 免得白传一堆数。
PAGE_N = 13
EXPORT_HEADER = [
    "时间(s)",
    "命令vx(m/s)", "轮速vx(m/s)", "EKFvx(m/s)", "前进达成率(%)",
    "命令wz(rad/s)", "轮速wz(rad/s)", "EKFwz(rad/s)", "转向达成率(%)",
    "轮速x(m)", "轮速y(m)", "轮速偏航(deg)",
    "EKFx(m)", "EKFy(m)", "EKF偏航(deg)",
    "航向差(deg,相对本表首行)",
    "陀螺x(rad/s)", "陀螺y(rad/s)", "陀螺z(rad/s)",
    "加速度x(m/s2)", "加速度y(m/s2)", "加速度z(m/s2)",
    "电池(V)",
]
# 导出里各数值保留几位小数由下面每列自己写, 不用表来配 —— 派生列夹在中间,
# 配表反而更难对。见 export_csv()。

# 版本标记。改控制逻辑时**顺手加一**, 这样"测试到底连的是哪份代码"一眼能看出来 ——
# 曾经因为旧实例没杀干净, 测试连了旧代码, 报了一堆假 FAIL, 白查半天。
BUILD_ID = "webctl-3"


def yaw_of(q):
    """四元数 -> 偏航角(rad)。只在平面运动下用, 所以不用完整旋转矩阵。"""
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Collector(Node):
    """把 ROS 话题收成一坨给网页看的快照 + 一段滚动历史, 顺便管运动控制。"""

    def __init__(self, vx_max=VX_LIMIT_DEFAULT, wz_max=WZ_LIMIT_DEFAULT,
                 allow_control=True, pub_topic="/cmd_vel"):
        super().__init__("chassis_web")
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.vx_max = float(vx_max)
        self.wz_max = float(wz_max)
        self.allow_control = bool(allow_control)
        # --dry-run 时改发到别的主题: 限幅/超时/刹车补发这些逻辑能完整验证,
        # 而真车一动不动。想安全地改控制代码, 这个开关很有用。
        self.pub_topic = pub_topic

        # 频率统计: 话题 -> [计数, 窗口起点, Hz]
        self.rate = {}
        self.last_rx = {}          # 话题 -> 最后收到的时刻(墙上时间)

        self.latest = {
            "cmd_vx": None, "cmd_wz": None, "cmd_age": None,
            "odom_vx": None, "odom_wz": None,
            "odom_x": None, "odom_y": None, "odom_yaw": None,
            "ekf_vx": None, "ekf_wz": None,
            "ekf_x": None, "ekf_y": None, "ekf_yaw": None,
            "gyro": None, "accel": None, "imu_age": None,
            "bat": None, "bat_age": None,
            "ranges": [None] * len(RANGE_TOPICS),
        }
        # 每收到一帧 /odom 就记一行, 让曲线上各条线时间轴严格对齐
        self.hist = collections.deque(maxlen=HIST_MAX)

        # ---- 运动控制状态 ----
        # ★ 浏览器只"表达意愿", 真正发布由本节点的定时器做 —— 这样 HTTP 线程和
        #   rclpy 线程不会互相踩。所有字段都在 self.lock 下读写。
        self.armed = False          # 必须显式使能
        self.want_vx = 0.0          # 浏览器最近一次表达的意愿
        self.want_wz = 0.0
        self.cmd_rx_t = 0.0         # 最近一次收到心跳的时刻 (0 = 从没收到)
        self.stop_burst = 0         # 还要补发几帧零速
        self.was_live = False       # 上一拍是不是真在发布(用来检测"刚停下"这个边沿)
        self.out_vx = 0.0           # 实际发出去的值 (给界面显示)
        self.out_wz = 0.0
        self.last_pub_live = False  # 上一次是不是真在发布(而不是沉默)
        self.other_pub = 0          # /cmd_vel 上别的发布者数量
        self._pub_check_t = 0.0

        self.pub = self.create_publisher(Twist, self.pub_topic, 10)
        self.create_timer(1.0 / CTRL_HZ, self._drive_tick)

        # 传感器数据用 sensor QoS (BEST_EFFORT) —— 厂商节点就是那样发的,
        # 用默认的 RELIABLE 会一个都收不到, 而且是静默收不到。
        self.create_subscription(Odometry, "/odom", self.cb_odom, 20)
        self.create_subscription(Odometry, "/odom_combined", self.cb_ekf, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 20)
        self.create_subscription(Imu, "/imu/data_raw", self.cb_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, "/PowerVoltage", self.cb_bat, 10)
        for i, t in enumerate(RANGE_TOPICS):
            self.create_subscription(
                Range, t, lambda m, k=i: self.cb_range(k, m),
                qos_profile_sensor_data)

    # ---------------- 频率/新鲜度 ----------------
    def _tick(self, topic):
        now = time.time()
        self.last_rx[topic] = now
        r = self.rate.get(topic)
        if r is None:
            self.rate[topic] = [1, now, 0.0]
            return
        r[0] += 1
        dt = now - r[1]
        if dt >= RATE_WINDOW:
            r[2] = r[0] / dt
            r[0] = 0
            r[1] = now

    def _age(self, topic):
        t = self.last_rx.get(topic)
        return None if t is None else round(time.time() - t, 2)

    # ---------------- 回调 ----------------
    def cb_cmd(self, m):
        with self.lock:
            self.latest["cmd_vx"] = round(m.linear.x, 4)
            self.latest["cmd_wz"] = round(m.angular.z, 4)
            self._tick("/cmd_vel")

    def cb_ekf(self, m):
        with self.lock:
            self.latest["ekf_vx"] = round(m.twist.twist.linear.x, 4)
            self.latest["ekf_wz"] = round(m.twist.twist.angular.z, 4)
            self.latest["ekf_x"] = round(m.pose.pose.position.x, 4)
            self.latest["ekf_y"] = round(m.pose.pose.position.y, 4)
            self.latest["ekf_yaw"] = round(yaw_of(m.pose.pose.orientation), 4)
            self._tick("/odom_combined")

    def cb_odom(self, m):
        with self.lock:
            l = self.latest
            l["odom_vx"] = round(m.twist.twist.linear.x, 4)
            l["odom_wz"] = round(m.twist.twist.angular.z, 4)
            l["odom_x"] = round(m.pose.pose.position.x, 4)
            l["odom_y"] = round(m.pose.pose.position.y, 4)
            l["odom_yaw"] = round(yaw_of(m.pose.pose.orientation), 4)
            self._tick("/odom")

            # 一帧一行, 各条线共用同一时间轴。
            # 列序见 EXPORT_HEADER —— **前 13 列是页面画图要用的**, 顺序别动,
            # 否则前端的列号就全错位了(见 PAGE_COLS)。
            # IMU 也是 20Hz, 但和 /odom 不是同一时刻的回调, 所以这里取的是
            # "这一帧 /odom 到达时最新的 IMU 值", 可能差几十毫秒。
            # 电池只有 ~1.8Hz, 是**保持上一个值**(采样保持), 不是同一时刻采的。
            g = l["gyro"] or [None, None, None]
            a = l["accel"] or [None, None, None]
            self.hist.append([
                round(time.time() - self.t0, 3),
                l["cmd_vx"], l["odom_vx"], l["ekf_vx"],
                l["cmd_wz"], l["odom_wz"], l["ekf_wz"],
                l["odom_x"], l["odom_y"],
                l["ekf_x"], l["ekf_y"],
                l["odom_yaw"], l["ekf_yaw"],
                g[0], g[1], g[2],
                a[0], a[1], a[2],
                l["bat"],
            ])

    def cb_imu(self, m):
        with self.lock:
            self.latest["gyro"] = [round(m.angular_velocity.x, 4),
                                   round(m.angular_velocity.y, 4),
                                   round(m.angular_velocity.z, 4)]
            self.latest["accel"] = [round(m.linear_acceleration.x, 3),
                                    round(m.linear_acceleration.y, 3),
                                    round(m.linear_acceleration.z, 3)]
            self._tick("/imu/data_raw")

    def cb_bat(self, m):
        with self.lock:
            self.latest["bat"] = round(m.data, 2)
            self._tick("/PowerVoltage")

    def cb_range(self, i, m):
        with self.lock:
            v = m.range
            # 超声波没量到目标时常常报 inf 或 0, 0 当"无效"更安全
            self.latest["ranges"][i] = None if (v is None or v <= 0.0
                                                or v != v or v == float("inf")) \
                else round(v, 3)
            self._tick(RANGE_TOPICS[i])

    # ---------------- 运动控制 ----------------
    # 分工: HTTP 线程只"表达意愿"(set_cmd/set_arm), 真正的发布只发生在
    # _drive_tick 里。这样 rclpy 和 HTTP 两个线程不会互相踩, 而且所有安全判断
    # 集中在一处, 不会散落到 HTTP 处理里被漏掉。
    def set_arm(self, on):
        with self.lock:
            self.armed = bool(on) and self.allow_control
            if not self.armed:
                self.want_vx = 0.0
                self.want_wz = 0.0
                self.cmd_rx_t = 0.0
            self.stop_burst = STOP_BURST
        return self.armed

    def set_cmd(self, vx, wz):
        """浏览器的心跳+意愿。没使能就当没收到 —— 不报错, 因为超时和失能本来
        就会让浏览器的请求落在空处, 那是正常现象, 不该刷一堆错误。"""
        with self.lock:
            if not (self.allow_control and self.armed):
                return False
            self.want_vx = float(vx)
            self.want_wz = float(wz)
            self.cmd_rx_t = time.time()
            return True

    def set_stop(self):
        with self.lock:
            self.want_vx = 0.0
            self.want_wz = 0.0
            self.armed = False
            self.cmd_rx_t = 0.0
            self.stop_burst = STOP_BURST

    def _drive_tick(self):
        """★ 唯一发布 /cmd_vel 的地方。三层兜底里的第二层就是这里。"""
        now = time.time()
        with self.lock:
            armed = self.armed
            hb_age = (now - self.cmd_rx_t) if self.cmd_rx_t else None
            want_vx, want_wz = self.want_vx, self.want_wz
            was_live = self.was_live

            if (now - self._pub_check_t) >= PUB_CHECK_EVERY:
                self._pub_check_t = now
                try:
                    # 自己也是一个发布者, 所以 >1 才叫"还有别人"
                    self.other_pub = max(0,
                        self.count_publishers(self.pub_topic) - 1)
                except Exception:
                    self.other_pub = 0

        odom_age = self._age("/odom")
        live = bool(armed and self.allow_control
                    and hb_age is not None and hb_age <= CMD_TIMEOUT
                    and odom_age is not None and odom_age <= ODOM_MAX_AGE)

        with self.lock:
            if live:
                self.was_live = True
            elif was_live:
                # ★ 刚才还在发布、现在不能发了(超时 / 失能 / 看不到 odom) ——
                #   必须**立刻补一段零速**。绝不能就这么沉默: 沉默的话 STM32 要等
                #   它自己的 800ms 看门狗才停, 那 0.8 秒车还在按最后一条命令走。
                #   这是死手设计里最容易漏的一条缝。
                self.stop_burst = STOP_BURST
                self.was_live = False

            if live:
                # 限幅在服务端做 —— 浏览器那边改了也不算数
                vx = max(-self.vx_max, min(self.vx_max, want_vx))
                wz = max(-self.wz_max, min(self.wz_max, want_wz))
            elif self.stop_burst > 0:
                self.stop_burst -= 1
                vx = wz = 0.0
            else:
                # 既不活跃、刹车余量也发完了: **一个字都不发**。
                # 否则本节点 20Hz 的零速会和 Nav2 抢 /cmd_vel, 车会一抖一抖。
                self.out_vx = self.out_wz = 0.0
                self.last_pub_live = False
                return

            self.out_vx, self.out_wz = float(vx), float(wz)
            self.last_pub_live = live

        # 发布放在锁外, 免得网络/中间件卡住时把 HTTP 线程也堵住
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        try:
            self.pub.publish(m)
        except Exception:
            pass

    # ---------------- 打包给网页 ----------------
    def payload(self):
        with self.lock:
            now_hb = time.time()
            l = dict(self.latest)
            l["cmd_age"] = self._age("/cmd_vel")
            l["imu_age"] = self._age("/imu/data_raw")
            l["bat_age"] = self._age("/PowerVoltage")
            rates = {k: round(v[2], 1) for k, v in self.rate.items()}
            ages = {t: self._age(t) for t in
                    ("/odom", "/odom_combined", "/imu/data_raw",
                     "/cmd_vel", "/PowerVoltage")}
            full = list(self.hist)
            # 只把前 PAGE_N 列发给页面(画图够用), 后面的 IMU/电池列留给导出,
            # 免得 5Hz 轮询白传一堆数
            hist = [list(r[:PAGE_N]) for r in full[-WINDOW:]]
            n_total = len(full)
            # 缓冲区里到底攒了多久 —— 导出前让用户知道能拿到多少, 别以为是"永久"
            span = round(full[-1][0] - full[0][0], 1) if len(full) > 1 else 0.0

            ctrl = {
                "allow": self.allow_control,     # 服务端是否允许控制(--no-control)
                "armed": self.armed,
                "want_vx": round(self.want_vx, 3),
                "want_wz": round(self.want_wz, 3),
                "out_vx": round(self.out_vx, 3),   # 实际发出去的
                "out_wz": round(self.out_wz, 3),
                "live": self.last_pub_live,        # 正在发布(而不是沉默)
                "hb_age": None if not self.cmd_rx_t else round(now_hb - self.cmd_rx_t, 2),
                "odom_age": self._age("/odom"),
                "other_pub": self.other_pub,
                "vx_max": self.vx_max,
                "wz_max": self.wz_max,
                "cmd_timeout": CMD_TIMEOUT,
                "odom_max_age": ODOM_MAX_AGE,
                # 这两个是为了能看出来"到底发生了什么": 刹车还剩几帧、上一拍在不在发。
                # 顺带也是个版本标记 —— 测试脚本靠它确认自己连的是新代码而不是
                # 某个没杀干净的旧实例(踩过: 旧实例占着端口, 新实例起不来,
                # 测试连了旧代码还给出了一堆假 FAIL)。
                "stop_burst": self.stop_burst,
                "was_live": self.was_live,
                "build": BUILD_ID,
            }

        # 达成率: 只在命令足够大时算, 否则"0/0"没意义还会刷出一堆假数
        def ratio(meas, cmd, eps):
            if cmd is None or meas is None or abs(cmd) < eps:
                return None
            return round(meas / cmd, 3)

        out = dict(l)
        out["ratio_vx"] = ratio(l["odom_vx"], l["cmd_vx"], 0.02)
        out["ratio_wz"] = ratio(l["odom_wz"], l["cmd_wz"], 0.05)
        # ★ 故意不在这里算 yaw 差: 两路位姿的原点不同, 绝对差没有意义(实测能到
        #   173°)。有意义的是"变化量之差", 由前端拿 hist 里的第 11/12 列自己算。
        #   摆一个没意义的数在首页比不摆更糟。

        return {
            "ok": True,
            "t": round(time.time() - self.t0, 2),
            "uptime": round(time.time() - self.t0, 1),
            "latest": out,
            "rates": rates,
            "ages": ages,
            "hist": hist,
            "hist_total": n_total,
            "hist_span": span,
            "hist_max": HIST_MAX,
            "ctrl": ctrl,
            "range_names": [c for c in "ABCDEF"],
        }

    # ---------------- 导出 ----------------
    def export_csv(self):
        """把最近 HIST_MAX 帧(≈30 秒 @20Hz)导成 CSV。

        为什么是 CSV + UTF-8 BOM:
          - `.csv` 双击就进 Excel / WPS / LibreOffice, 不用装任何库;
          - **BOM 一个字节都不能省** —— 不带的话 Windows 版 Excel 会按本地代码页
            解, 中文表头直接变乱码。这就是"能打开"和"打开是一堆问号"的区别。
          - 行尾用 CRLF, 也是给 Excel 面子。
        """
        import math as _m
        with self.lock:
            rows = list(self.hist)

        def f(v, nd):
            return "" if v is None else ("%.*f" % (nd, v))

        # 相对航向差的基准 = 本表第一行, 和页面上"相对打开页面时"是同一口径
        base_o = rows[0][11] if rows else None
        base_e = rows[0][12] if rows else None

        out = ["\ufeff" + ",".join(EXPORT_HEADER)]
        for r in rows:
            ratio_vx = None
            if r[1] is not None and r[2] is not None and abs(r[1]) >= 0.02:
                ratio_vx = r[2] / r[1] * 100.0
            ratio_wz = None
            if r[4] is not None and r[5] is not None and abs(r[4]) >= 0.05:
                ratio_wz = r[5] / r[4] * 100.0
            ydiff = None
            if (base_o is not None and base_e is not None
                    and r[11] is not None and r[12] is not None):
                ydiff = _m.degrees((r[12] - base_e) - (r[11] - base_o))
                while ydiff > 180.0:
                    ydiff -= 360.0
                while ydiff < -180.0:
                    ydiff += 360.0

            out.append(",".join([
                f(r[0], 3),
                f(r[1], 4), f(r[2], 4), f(r[3], 4), f(ratio_vx, 1),
                f(r[4], 4), f(r[5], 4), f(r[6], 4), f(ratio_wz, 1),
                f(r[7], 4), f(r[8], 4),
                "" if r[11] is None else f(_m.degrees(r[11]), 2),
                f(r[9], 4), f(r[10], 4),
                "" if r[12] is None else f(_m.degrees(r[12]), 2),
                f(ydiff, 2),
                f(r[13], 4), f(r[14], 4), f(r[15], 4),
                f(r[16], 3), f(r[17], 3), f(r[18], 3),
                f(r[19], 2),
            ]))
        return "\r\n".join(out) + "\r\n"


NODE = None
HTML_PATH = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = urlparse(self.path).path
        if p in ("/", "/index.html"):
            try:
                with open(HTML_PATH, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError as e:
                self._send(500, ("读不到界面文件 %s: %s" % (HTML_PATH, e))
                           .encode("utf-8"), "text/plain; charset=utf-8")
            return
        if p == "/api/data":
            try:
                body = json.dumps(NODE.payload()).encode("utf-8")
            except Exception as e:                      # 别让一个异常打死服务
                body = json.dumps({"ok": False, "err": str(e)}).encode("utf-8")
            self._send(200, body, "application/json; charset=utf-8")
            return
        if p == "/api/export.csv":
            try:
                body = NODE.export_csv().encode("utf-8")
            except Exception as e:
                self._send(500, ("导出失败: %s" % e).encode("utf-8"),
                           "text/plain; charset=utf-8")
                return
            # 文件名带时间戳, 连续导出几次不互相覆盖
            fn = "chassis_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Content-Disposition",
                             'attachment; filename="%s"' % fn)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def log_message(self, fmt, *args):
        pass            # 默认每个请求都打一行, 5Hz 轮询会把终端刷爆

    # ---------------- 控制接口 (只有这三个会改变车的行为) ----------------
    def _json(self, obj):
        self._send(200, json.dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8")

    def do_POST(self):
        global NODE
        p = urlparse(self.path).path
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        raw = self.rfile.read(n) if n > 0 else b"{}"
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except Exception:
            body = {}

        if p == "/api/arm":
            on = NODE.set_arm(bool(body.get("on")))
            self._json({"ok": True, "armed": on})
        elif p == "/api/cmd":
            try:
                vx = float(body.get("vx", 0.0))
                wz = float(body.get("wz", 0.0))
            except (TypeError, ValueError):
                vx, wz = 0.0, 0.0
            acc = NODE.set_cmd(vx, wz)
            self._json({"ok": True, "accepted": acc})
        elif p == "/api/stop":
            NODE.set_stop()
            self._json({"ok": True})
        else:
            self._send(404, b"not found", "text/plain; charset=utf-8")


def main():
    global NODE, HTML_PATH

    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0",
                    help="监听地址, 默认 0.0.0.0 也就是局域网都能访问")
    ap.add_argument("--html", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "chassis_web.html"))
    ap.add_argument("--vx-max", type=float, default=VX_LIMIT_DEFAULT,
                    help="前进速度上限 m/s (默认 %.2f)。实测工作包线 0.25~0.35 "
                         "最好, >0.4 会超速" % VX_LIMIT_DEFAULT)
    ap.add_argument("--wz-max", type=float, default=WZ_LIMIT_DEFAULT,
                    help="转向角速度上限 rad/s (默认 %.2f)" % WZ_LIMIT_DEFAULT)
    ap.add_argument("--no-control", action="store_true",
                    help="彻底关掉运动控制当纯监视用 —— 想只看看数据、"
                         "绝对不给任何发速度的可能时用这个")
    ap.add_argument("--dry-run", action="store_true",
                    help="把要发的速度发到 /cmd_vel_dryrun 而不是 /cmd_vel。"
                         "用来安全地验限幅/超时/刹车补发 —— 逻辑全跑, 车不动")
    args = ap.parse_args()

    HTML_PATH = args.html
    if not os.path.isfile(HTML_PATH):
        sys.exit("找不到界面文件: %s\n"
                 "(它应该和本脚本放在同一个目录里)" % HTML_PATH)

    rclpy.init()
    NODE = Collector(vx_max=args.vx_max, wz_max=args.wz_max,
                     allow_control=not args.no_control,
                     pub_topic="/cmd_vel_dryrun" if args.dry_run else "/cmd_vel")

    # rclpy 的 spin 必须在自己的线程里 —— 主线程留给 HTTP 服务。
    # 所有共享状态都用 NODE.lock 保护, 回调里不会碰 HTTP 的东西。
    th = threading.Thread(target=rclpy.spin, args=(NODE,), daemon=True)
    th.start()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("底盘数据网页已启动")
    print("  本机:     http://127.0.0.1:%d" % args.port)
    print("  局域网:   http://<本机IP>:%d   (这台车是 192.168.5.17)" % args.port)
    print("  界面文件: %s" % HTML_PATH)
    if args.no_control:
        print("  运动控制: **已关闭** (--no-control), 纯监视")
    else:
        print("  运动控制: 开启, 上限 vx=%.2f m/s  wz=%.2f rad/s" %
              (args.vx_max, args.wz_max))
        print("            必须在网页上显式使能才会动; 心跳断 %.1fs 自动停" %
              CMD_TIMEOUT)
        if args.dry_run:
            print("            ★ dry-run: 速度发到 /cmd_vel_dryrun, 真车不会动")
    print("  Ctrl-C 退出")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C, 退出")
    finally:
        srv.shutdown()
        NODE.destroy_node()
        rclpy.shutdown()
        os._exit(0)     # 和别的工具一致: 宁可强退, 也不留一个占着端口的僵尸


if __name__ == "__main__":
    main()
