#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘 teleop 全链路延迟探针（在底盘主机上运行）。

做的事情，全部在同一台主机、同一个 time.time() 时钟下打时间戳：
  1) 在 pty 里起 `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
  2) 主线程按脚本"按"键，并在写入 pty 前记录键时刻 t_key
  3) 另一个线程订阅 /cmd_vel 与 /odom，记录到达时刻
  4) 把 {key 事件, /cmd_vel 样本, /odom 样本} 写成 JSON

配合 LD_PRELOAD 钩子（writets.so，记录底盘节点每次写 11 B 命令帧的时刻）
即可把全链路切成：按键→cmd_vel→串口写出→轮子开始转。

用法（示例）:
  python3 teleop_key_probe.py --trials 20 --speed 0.20 --out /tmp/key_probe.json
  python3 teleop_key_probe.py --check-only          # 只做方向自检，不动 20 trial
"""
import argparse
import json
import os
import pty
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

TRACK = 0.456       # 2026-10 新底盘轮距
WHEEL_R = 0.0855    # 171 mm 轮径

FORWARD_KEY = "i"
BACKWARD_KEY = ","


class Recorder(Node):
    def __init__(self):
        super().__init__("teleop_key_probe")
        self.lock = threading.Lock()
        self.cmd = []
        self.odom = []
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 200)
        self.create_subscription(Odometry, "/odom", self._on_odom, 200)

    def _on_cmd(self, m):
        t = time.time()
        with self.lock:
            self.cmd.append([t, m.linear.x, m.linear.y, m.angular.z])

    def _on_odom(self, m):
        t = time.time()
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        tw = m.twist.twist
        with self.lock:
            self.odom.append([t, tw.linear.x, tw.angular.z, st])

    def snapshot(self):
        with self.lock:
            return list(self.cmd), list(self.odom)


def spin_forever(node, stop_evt):
    while not stop_evt.is_set():
        rclpy.spin_once(node, timeout_sec=0.02)


def spawn_teleop(speed, turn, log_path):
    """在 pty 里起 teleop_twist_keyboard，返回 (proc, master_fd, log_file)。"""
    master, slave = pty.openpty()
    cmd = [
        "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
        "--ros-args", "-p", "speed:=%.3f" % speed, "-p", "turn:=%.3f" % turn,
    ]
    logf = open(log_path, "wb")
    proc = subprocess.Popen(
        cmd, stdin=slave, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid, close_fds=True,
    )
    os.close(slave)
    return proc, master, logf


def send_key(master, ch, events, trial, label):
    t = time.time()
    events.append({"t": t, "key": ch, "trial": trial, "label": label})
    os.write(master, ch.encode())
    return t


def motion_window(odom, t0, t1):
    """[t0, t1] 内 /odom 样本的速度统计。"""
    sel = [s for s in odom if t0 <= s[0] <= t1]
    if not sel:
        return None
    vx = [s[1] for s in sel]
    wz = [s[2] for s in sel]
    return {
        "n": len(sel),
        "vx_mean": sum(vx) / len(vx),
        "vx_min": min(vx),
        "vx_max": max(vx),
        "wz_mean": sum(wz) / len(wz),
        "wz_max": max(abs(w) for w in wz),
    }


def judge(vx_mean, vx_peak, wz_max):
    if wz_max > 0.15:
        return "spin"
    if vx_mean > 0.03:
        return "forward"
    if vx_mean < -0.03:
        return "backward"
    if vx_peak > 0.03:
        return "forward"
    return "unknown"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--speed", type=float, default=0.20, help="teleop speed 参数 (m/s)")
    ap.add_argument("--turn", type=float, default=0.60)
    ap.add_argument("--settle", type=float, default=1.5, help="每 trial 前静置 s")
    ap.add_argument("--hold", type=float, default=1.0, help="按下去后观察 s")
    ap.add_argument("--out", default="/tmp/key_probe.json")
    ap.add_argument("--teleop-log", default="/tmp/teleop_keyboard.log")
    ap.add_argument("--check-only", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    rec = Recorder()
    stop_evt = threading.Event()
    th = threading.Thread(target=spin_forever, args=(rec, stop_evt), daemon=True)
    th.start()

    proc, master, logf = spawn_teleop(args.speed, args.turn, args.teleop_log)
    events = []
    print("[probe] teleop pid=%d, speed=%.3f, 等待 3 s 建立订阅" % (proc.pid, args.speed))
    time.sleep(3.0)

    if proc.poll() is not None:
        print("[probe] teleop 已退出, rc=%s, 日志:" % proc.returncode)
        logf.flush()
        with open(args.teleop_log, "rb") as f:
            print(f.read().decode("utf-8", "replace"))
        stop_evt.set()
        th.join(timeout=2.0)
        rclpy.shutdown()
        return 2

    # ---- 静止噪声 ----
    t_noise0 = time.time()
    time.sleep(1.0)
    cmd0, odom0 = rec.snapshot()
    noise = motion_window(odom0, t_noise0, time.time())
    print("[probe] 静止噪声: %s" % noise)

    # 先发一次 stop，纯粹为了确认 /cmd_vel 链路通
    send_key(master, "k", events, -1, "stop-init")
    time.sleep(0.6)
    cmd1, _ = rec.snapshot()
    print("[probe] 试发 'k' 后 /cmd_vel 累计 %d 条" % len(cmd1))
    if len(cmd1) == 0:
        print("[probe] 致命: /cmd_vel 收不到, teleop 没在发 (域名/节点问题?)")
        proc.terminate()
        stop_evt.set()
        th.join(timeout=2.0)
        rclpy.shutdown()
        return 3

    # ---- 方向自检 ----
    t_chk = send_key(master, FORWARD_KEY, events, 0, "check")
    time.sleep(args.hold)
    _, odom1 = rec.snapshot()
    win = motion_window(odom1, t_chk + 0.20, t_chk + args.hold)
    j = "unknown"
    if win:
        j = judge(win["vx_mean"], win["vx_max"], win["wz_max"])
    print("[probe] 方向自检(按 'i'): %s  win=%s" % (j, win))
    if args.check_only:
        send_key(master, "k", events, -1, "stop")
        time.sleep(0.3)
        _dump(args, events, rec, proc, master, logf, stop_evt, j, noise, th)
        return 0

    # ---- 正式 trials ----
    for i in range(args.trials):
        send_key(master, "k", events, -1, "settle")
        time.sleep(args.settle)
        key = FORWARD_KEY if (i % 2 == 0) else BACKWARD_KEY
        send_key(master, key, events, i, "step")
        time.sleep(args.hold)
        print("[probe] trial %2d/%d key=%r" % (i + 1, args.trials, key))
    send_key(master, "k", events, -1, "stop")
    time.sleep(0.5)

    _dump(args, events, rec, proc, master, logf, stop_evt, j, noise, th)
    return 0


def _dump(args, events, rec, proc, master, logf, stop_evt, judge_txt, noise, spin_th=None):
    cmd, odom = rec.snapshot()
    data = {
        "meta": {
            "speed": args.speed,
            "turn": args.turn,
            "trials": args.trials,
            "settle": args.settle,
            "hold": args.hold,
            "track": TRACK,
            "wheel_r": WHEEL_R,
            "judge": judge_txt,
            "noise": noise,
            "t_start": events[0]["t"] if events else None,
        },
        "keys": events,
        "cmd": cmd,
        "odom": odom,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f)
    print("[probe] 写出 %s : keys=%d cmd=%d odom=%d" %
          (args.out, len(events), len(cmd), len(odom)))
    try:
        os.write(master, b"\x03")
    except OSError:
        pass
    time.sleep(0.2)
    if proc.poll() is None:
        proc.terminate()
    logf.close()
    stop_evt.set()
    if spin_th is not None:
        spin_th.join(timeout=2.0)   # 先停 spin 线程再 shutdown，否则 rclpy 会 abort
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    sys.exit(main())
