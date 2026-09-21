#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全验证网页的运动控制 —— 全程走 --dry-run, 真车一动不动。
验的是四条最容易出错、而且出事最严重的性质:
  1. 没使能就发速度 -> 必须被拒, 且一个字节都不许发出去
  2. 超限请求 -> 必须被**服务端**夹住(不信浏览器)
  3. 心跳停了 -> 必须**立刻补发刹车**, 不能沉默着等 STM32 的 800ms 看门狗
  4. 刹车补发完之后 -> 必须闭嘴, 不能一直霸占 /cmd_vel (否则和 Nav2 打架)
"""
import json
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BASE = "http://127.0.0.1:8090"
TOPIC = "/cmd_vel_dryrun"


def post(path, obj=None):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(obj or {}).encode(),
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read().decode())


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=3) as r:
        return json.loads(r.read().decode())


rclpy.init()
node = rclpy.create_node("dryrun_probe")
msgs = []
node.create_subscription(
    Twist, TOPIC,
    lambda m: msgs.append((time.time(), round(m.linear.x, 4),
                           round(m.angular.z, 4))), 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
time.sleep(1.2)

ok = True


def check(name, cond, extra=""):
    global ok
    print(("  [OK]   " if cond else "  [FAIL] ") + name +
          ("   " + extra if extra else ""))
    if not cond:
        ok = False


def clear():
    del msgs[:]


def win(t0, t1):
    return [m for m in msgs if t0 <= m[0] <= t1]


print("=== 0) 先确认连的是新代码 (不是没杀干净的旧实例) ===")
try:
    c0 = get("/api/data")["ctrl"]
    print("  build =", c0.get("build"))
    check("payload 里有 build 标记(旧代码没有)", "build" in c0, str(c0.get("build")))
    check("有 stop_burst / was_live 字段(旧代码没有)",
          "stop_burst" in c0 and "was_live" in c0)
except Exception as e:
    print("  !! /api/data 取不到:", e)
    check("能取到 /api/data", False)
if not ok:
    print("\n连的不是预期版本的实例, 后面的结论都没意义 —— 先清掉旧实例再来。")
    import sys
    sys.stdout.flush()
    import os
    os._exit(1)

print("=== 1) 未使能时发速度: 必须被拒, 且一个字节都不发 ===")
post("/api/stop")
time.sleep(0.8)
clear()
r = post("/api/cmd", {"vx": 0.3, "wz": 1.0})
time.sleep(0.8)
check("接口返回 accepted=False", r.get("accepted") is False, str(r))
check("dryrun 主题上零帧", len(msgs) == 0, "收到 %d 帧" % len(msgs))

print("=== 2) 使能 ===")
r = post("/api/arm", {"on": True})
check("armed 变 True", r.get("armed") is True, str(r))

print("=== 3) 超限请求必须被服务端夹住 ===")
clear()
t0 = time.time()
post("/api/cmd", {"vx": 99.0, "wz": 99.0})
t_hb = time.time()          # ★ 最后一次心跳的**真实**时刻, 就在这里
time.sleep(0.5)
c = get("/api/data")["ctrl"]
check("out_vx 夹到上限", abs(c["out_vx"] - c["vx_max"]) < 1e-6,
      "out_vx=%s 上限=%s" % (c["out_vx"], c["vx_max"]))
check("out_wz 夹到上限", abs(c["out_wz"] - c["wz_max"]) < 1e-6,
      "out_wz=%s 上限=%s" % (c["out_wz"], c["wz_max"]))
got = win(t0, time.time())
check("主题上确实是夹住后的值",
      any(abs(m[1] - c["vx_max"]) < 1e-6 for m in got), "%d 帧" % len(got))

print("=== 4) 心跳停 0.5s 之后: 必须立刻补发刹车(而不是沉默) ===")
# 时间关系(都以 t_hb 为基准):
#   [t_hb,      t_hb+0.5]  心跳还新鲜 -> 正常发布夹住后的值
#   [t_hb+0.5,  t_hb+0.9]  超时 -> was_live 边沿 -> 补发 8 帧零速
#   t_hb+0.9 之后          闭嘴, 一个字都不发
# ★ 前两版这里都写错了: 我拿"断言之后"的时刻当基准, 整段窗口错过去了,
#   把一个正常工作的补发判成了 FAIL。基准必须是**心跳那一刻**。
time.sleep(1.3)
c = get("/api/data")["ctrl"]
check("live 变 False", c["live"] is False,
      "live=%s hb_age=%s" % (c["live"], c["hb_age"]))
check("hb_age 是真实年龄(不是 Unix 时间戳)",
      c["hb_age"] is None or c["hb_age"] < 10, "hb_age=%s" % c["hb_age"])
burst = win(t_hb + 0.6, t_hb + 1.0)
check("超时窗口内有帧发出(不是沉默着等 800ms 看门狗)", len(burst) > 0,
      "窗口内 %d 帧" % len(burst))
check("而且发出去的全是 0",
      all(abs(m[1]) < 1e-9 and abs(m[2]) < 1e-9 for m in burst),
      "非零帧 %d 个" % len([m for m in burst if abs(m[1]) > 1e-9]))
print("      (以 t_hb 为基准: 0~0.5s=%d 帧[心跳新鲜, 非零]  "
      "0.5~0.9s=%d 帧[补发刹车, 全零]  0.9s以后=%d 帧[应当闭嘴])"
      % (len(win(t_hb, t_hb + 0.5)), len(win(t_hb + 0.5, t_hb + 0.9)),
         len(win(t_hb + 0.9, time.time()))))

print("=== 5) 刹车补发完就该闭嘴 ===")
time.sleep(1.5)
clear()
time.sleep(1.0)
check("静默期零帧(不霸占 /cmd_vel)", len(msgs) == 0, "收到 %d 帧" % len(msgs))

print("=== 6) 急停 = 停 + 撤使能 ===")
post("/api/arm", {"on": True})
time.sleep(0.3)
post("/api/cmd", {"vx": 0.0, "wz": 0.0})
time.sleep(0.3)
post("/api/stop")
time.sleep(0.4)
c = get("/api/data")["ctrl"]
check("armed 已撤", c["armed"] is False, "armed=%s" % c["armed"])
check("want 归零", abs(c["want_vx"]) < 1e-9 and abs(c["want_wz"]) < 1e-9)

print()
print("结论:", "全部通过" if ok else "**有不通过项**")
node.destroy_node()
rclpy.shutdown()
import os
import sys
# ★ os._exit 不会 flush 缓冲区! 输出是管道(非 tty)时 Python 用块缓冲,
#   不显式 flush 的话上面所有打印都会丢掉 —— 实测把整份测试输出吞了。
sys.stdout.flush()
sys.stderr.flush()
os._exit(0 if ok else 1)
