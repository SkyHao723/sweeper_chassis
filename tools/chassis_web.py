#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘数据网页 —— 在 RK3588 上跑, 局域网浏览器直接打开看

    ssh 到车之后:  python3 ~/chassis_tools/chassis_web.py
    然后浏览器开:  http://192.168.5.17:8080

================================ 两个刻意的取舍 ================================

1. **只用标准库, 不装任何包。**
   板子上没有 flask, 而装包要么走 apt 要么走 pip, 都可能和 ROS 的 python 环境
   打架(这台车已经因为环境问题绕过好几次)。http.server + json 足够, 单文件就能跑。

2. **只读, 网页不提供任何"发速度"的入口。**
   这台车因为控制层的 bug 真失控过。一个能从浏览器点着走的东西, 要负责的是急停、
   指令超时、权限、误触 —— 那是另一个东西, 不该混进"简单数据前端"里。想看数据
   和想开车是两件事, 这一版只做前者。

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


def yaw_of(q):
    """四元数 -> 偏航角(rad)。只在平面运动下用, 所以不用完整旋转矩阵。"""
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Collector(Node):
    """把 ROS 话题收成一坨给网页看的快照 + 一段滚动历史。"""

    def __init__(self):
        super().__init__("chassis_web")
        self.lock = threading.Lock()
        self.t0 = time.time()

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

            # 一帧一行, 各条线共用同一时间轴
            # [11][12] 两列偏航是给前端算"相对漂移"用的 —— 绝对差没意义, 见文件头
            self.hist.append([
                round(time.time() - self.t0, 3),
                l["cmd_vx"], l["odom_vx"], l["ekf_vx"],
                l["cmd_wz"], l["odom_wz"], l["ekf_wz"],
                l["odom_x"], l["odom_y"],
                l["ekf_x"], l["ekf_y"],
                l["odom_yaw"], l["ekf_yaw"],
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

    # ---------------- 打包给网页 ----------------
    def payload(self):
        with self.lock:
            l = dict(self.latest)
            l["cmd_age"] = self._age("/cmd_vel")
            l["imu_age"] = self._age("/imu/data_raw")
            l["bat_age"] = self._age("/PowerVoltage")
            rates = {k: round(v[2], 1) for k, v in self.rate.items()}
            ages = {t: self._age(t) for t in
                    ("/odom", "/odom_combined", "/imu/data_raw",
                     "/cmd_vel", "/PowerVoltage")}
            hist = list(self.hist)[-WINDOW:]
            n_total = len(self.hist)

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
            "range_names": [c for c in "ABCDEF"],
        }


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
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def log_message(self, fmt, *args):
        pass            # 默认每个请求都打一行, 5Hz 轮询会把终端刷爆


def main():
    global NODE, HTML_PATH

    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0",
                    help="监听地址, 默认 0.0.0.0 也就是局域网都能访问")
    ap.add_argument("--html", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "chassis_web.html"))
    args = ap.parse_args()

    HTML_PATH = args.html
    if not os.path.isfile(HTML_PATH):
        sys.exit("找不到界面文件: %s\n"
                 "(它应该和本脚本放在同一个目录里)" % HTML_PATH)

    rclpy.init()
    NODE = Collector()

    # rclpy 的 spin 必须在自己的线程里 —— 主线程留给 HTTP 服务。
    # 所有共享状态都用 NODE.lock 保护, 回调里不会碰 HTTP 的东西。
    th = threading.Thread(target=rclpy.spin, args=(NODE,), daemon=True)
    th.start()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print("底盘数据网页已启动")
    print("  本机:     http://127.0.0.1:%d" % args.port)
    print("  局域网:   http://<本机IP>:%d   (这台车是 192.168.5.17)" % args.port)
    print("  界面文件: %s" % HTML_PATH)
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
