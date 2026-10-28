#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_latency.py 的自测：造一份「已知真值」的假数据，看分析脚本能不能反推出来。

真值设定:
  T1 (发布 → 串口 write)  ~ U(0, 50 ms)      <- 模拟厂商节点 spin_some 20 Hz 的调度抖动
  T2 (串口 write → 轮子开始转) = 30 ms       <- 模拟固件 20 ms 周期 + CAN + 驱动器速度环
  轮子速度: 从 T2 起用 120 ms 线性升到 ±0.20 m/s, 然后保持
  /odom   : 20 Hz (50 ms 网格), **网格相位独立于发布时刻**（真实情况: 由 STM32 的 20 Hz 时钟决定）
            速度噪声 ±0.004 m/s

用法:
  python selftest_analyze.py            # 生成 + 自测
  python selftest_analyze.py --keep     # 只生成
"""

import argparse
import json
import os
import random
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime

TRACK = 0.456
SPEED = 0.20
TRIALS = 20
T1_MAX = 0.050
T2_TRUE = 0.030
RISE = 0.120


class Gen:
    def __init__(self):
        random.seed(20261028)
        self.t0 = time.time() + 5.0
        self.pubs = []
        self.writes = []      # 与 pubs 一一对应
        self.cmds = []        # (write_time, target)  按时间递增
        self.samples = []

    def cmd(self, t_pub, target, tag):
        self.pubs.append({"t": t_pub, "vx": float(target), "wz": 0.0, "tag": tag})
        w = t_pub + random.uniform(0.0, T1_MAX)
        self.writes.append(w)
        self.cmds.append((w, float(target)))
        return w

    def speed_at(self, ts):
        """最近一条命令生效后, 轮子此刻的转速 (线性 120 ms 上升)"""
        cur = None
        for w, tgt in self.cmds:
            if w <= ts:
                cur = (w, tgt)
            else:
                break
        if cur is None:
            return 0.0
        w, tgt = cur
        if tgt == 0.0:
            return 0.0
        elapsed = ts - (w + T2_TRUE)
        if elapsed <= 0:
            return 0.0
        return tgt * min(1.0, elapsed / RISE)

    def make_samples(self, t_end, phase=0.007):
        ts = self.t0 + phase
        while ts < t_end:
            v = self.speed_at(ts) + random.uniform(-0.004, 0.004)
            self.samples.append({
                "t": ts + 0.008,           # 本机收到 /odom 的时刻
                "st": ts,                  # header.stamp
                "vx": v,
                "wz": random.uniform(-0.004, 0.004),
            })
            ts += 0.050


def build(outdir):
    g = Gen()
    t = g.t0

    # --- 方向自检 ---
    g.cmd(t, 0.0, "check0");            t += 0.6
    g.cmd(t, 0.10, "check");            t += 0.5
    g.cmd(t, 0.0, "check_stop");        t += 0.8

    # --- 正式 trial ---
    for i in range(TRIALS):
        sign = 1.0 if i % 2 == 0 else -1.0
        g.cmd(t, 0.0, "t%d_pre" % i);              t += 1.0
        g.cmd(t, sign * SPEED, "t%d_step" % i);    t += 0.6
        g.cmd(t, 0.0, "t%d_stop" % i);             t += 0.2
    g.cmd(t, 0.0, "final_stop");                   t_end = t + 0.8

    g.make_samples(t_end)

    data = {
        "config": {"speed": SPEED, "trials": TRIALS, "settle": 1.0, "hold": 0.6},
        "track": TRACK,
        "published": g.pubs,
        "samples": g.samples,
        "check": {"vx": 0.10, "wz": 0.0, "verdict": "forward", "noise_max": 0.006},
    }

    # strace 日志: 写入时刻 + 噪声行
    lines = []
    for k, w in enumerate(g.writes):
        dt = datetime.fromtimestamp(w)
        lines.append('%d %02d:%02d:%02d.%06d write(3, "\\x7b\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x7d", 11) = 11 <0.000021>'
                     % (8123, dt.hour, dt.minute, dt.second, dt.microsecond))
        if k % 3 == 0:
            dt2 = datetime.fromtimestamp(w + 0.001)
            lines.append('%d %02d:%02d:%02d.%06d write(1, "[INFO] odom\\n", 12) = 12 <0.000003>'
                         % (8123, dt2.hour, dt2.minute, dt2.second, dt2.microsecond))

    jp = os.path.join(outdir, "latency_selftest.json")
    sp = os.path.join(outdir, "strace_selftest.log")
    with open(jp, "w") as f:
        json.dump(data, f)
    with open(sp, "w") as f:
        f.write("\n".join(lines) + "\n")

    # 自检内部真值统计
    t2s = []
    for i in range(TRIALS):
        step = g.pubs[[k for k, p in enumerate(g.pubs) if p["tag"] == "t%d_step" % i][0]]
        pass
    print("真值: T1 ~ U(0, %.0f ms) mean=%.1f ms, T2 = %.0f ms, 上升 %.0f ms, odom 20 Hz"
          % (T1_MAX * 1e3, T1_MAX * 0.5e3, T2_TRUE * 1e3, RISE * 1e3))
    return jp, sp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", action="store_true")
    args = ap.parse_args()

    outdir = tempfile.gettempdir()
    jp, sp = build(outdir)
    print("生成: %s\n      %s" % (jp, sp))
    if args.keep:
        return 0

    here = os.path.dirname(os.path.abspath(__file__))
    r = subprocess.run([sys.executable, os.path.join(here, "analyze_latency.py"), jp, sp])
    print("\n[自测判读] 期望:")
    print("  T1 全部:  0~50 ms, mean≈25 ms, 100%% 落在 0~50 ms")
    print("  T2 下界:  ≤ 30 ms（某次得到 ≈ 30-50 = -20~30）")
    print("  T2 上界:  min ≈ 30~33 ms（N=20 时量化误差 ≈ 50/21≈2.4 ms）")
    print("  T2 上升沿外推: ≈ 30 ms ± 5")
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
