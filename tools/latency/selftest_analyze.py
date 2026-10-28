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

故意掺的干扰（用来验证"按内容配对"是否真的起作用）:
  每条真命令帧前面 2 ms, 插一条同样 11 字节、同样 0x7B 开头的 **LED 设置帧**
  （厂商源码 L98-109 就是这种帧）, 它解出来的 (vx,wz) 是永远不会下发的 (5.0, 0.0)。
  如果分析脚本退回"窗口内最早一帧", T1 会变成负数 —— 所以 T1 全部落在 0~50 ms
  是"内容配对生效"的硬判据。

用法:
  python selftest_analyze.py            # 生成 + 自测
  python selftest_analyze.py --keep     # 只生成
"""

import argparse
import json
import os
import random
import subprocess
import sys
import tempfile
import time

TRACK = 0.456
SPEED = 0.20
TRIALS = 20
T1_MAX = 0.050
T2_TRUE = 0.030
RISE = 0.120

DECOY_VX = 5.0          # LED 帧解出来的假速度, 实际永远不会下发


def enc_cmd(vx, wz):
    """按厂商帧布局造 11 B 命令帧: [0]0x7B [3..4]vx*1000 BE [5..6]vy*1000 [7..8]wz*1000
       [9]BCC [10]0x7D"""
    def s16(v):
        i = int(round(v * 1000.0))
        i = max(-32768, min(32767, i))
        return (i & 0xFFFF).to_bytes(2, "big")
    b = bytearray(11)
    b[0] = 0x7B
    b[1] = 0x00
    b[2] = 0x00
    b[3:5] = s16(vx)
    b[5:7] = s16(0.0)
    b[7:9] = s16(wz)
    b[9] = 0x00
    b[10] = 0x7D
    return b.hex()


def enc_decoy():
    """LED 帧: 同长度、同帧头, 但解出来是 (DECOY_VX, 0)"""
    return enc_cmd(DECOY_VX, 0.0)


class Gen:
    def __init__(self):
        random.seed(20261028)
        self.t0 = time.time() + 5.0
        self.pubs = []
        self.wlines = []      # (write_time, payload_hex, size)
        self.cmds = []        # (write_time, target)  按时间递增
        self.samples = []

    def cmd(self, t_pub, target, tag):
        self.pubs.append({"t": t_pub, "vx": float(target), "wz": 0.0, "tag": tag})
        w = t_pub + random.uniform(0.0, T1_MAX)
        self.wlines.append((w - 0.002, enc_decoy(), 11))   # 干扰帧, 更早
        self.wlines.append((w, enc_cmd(target, 0.0), 11))
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

    # writets.so 新格式: "<epoch>.<usec> <fd> <size> <payloadhex>"
    lines = []
    for w, payload, size in sorted(g.wlines, key=lambda x: x[0]):
        lines.append("%.6f %d %d %s" % (w, 3, size, payload))
    # 再掺几条非 11 字节的写（日志/其他 fd）
    lines.append("%.6f %d %d %s" % (g.t0, 1, 12, "00" * 12))
    lines.sort(key=lambda s: float(s.split()[0]))

    jp = os.path.join(outdir, "latency_selftest.json")
    sp = os.path.join(outdir, "writets_selftest.log")
    with open(jp, "w") as f:
        json.dump(data, f)
    with open(sp, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("真值: T1 ~ U(0, %.0f ms) mean=%.1f ms, T2 = %.0f ms, 上升 %.0f ms, odom 20 Hz"
          % (T1_MAX * 1e3, T1_MAX * 0.5e3, T2_TRUE * 1e3, RISE * 1e3))
    print("干扰: %d 条 LED 假帧(解码出 vx=%.1f m/s), 均在真命令帧前 2 ms" % (len(g.pubs), DECOY_VX))
    print("      -> 若 T1 有任何一个为负, 说明配对退回了'按时间', 按内容配对失效")
    return jp, sp, len(g.pubs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", action="store_true")
    args = ap.parse_args()

    outdir = tempfile.gettempdir()
    jp, sp, n_pub = build(outdir)
    print("生成: %s\n      %s" % (jp, sp))
    if args.keep:
        return 0

    here = os.path.dirname(os.path.abspath(__file__))
    r = subprocess.run([sys.executable, os.path.join(here, "analyze_latency.py"), jp, sp],
                       capture_output=True, text=True)
    out = r.stdout
    print(out)
    if r.returncode != 0:
        print("[自测] 分析脚本退出码 %d" % r.returncode)
        print(r.stderr)
        return 1

    # ---- 硬判据 ----
    bad = []
    want = "按内容 %d, 按时间兜底 0, 没配上 0" % n_pub
    if want not in out:
        bad.append("配对统计不符: 期望 '%s'" % want)
    if "落在 0~50 ms: %d/%d = 100%%" % (n_pub, n_pub) not in out:
        bad.append("T1 没有 100%% 落在 0~50 ms（内容配对没生效?）")
    if "T2 = " not in out:
        bad.append("没算出 T2")
    else:
        import re
        m = re.search(r">> T2 = ([\d.]+) ms", out)
        if not m:
            bad.append("T2 汇总行缺失")
        else:
            t2 = float(m.group(1))
            if abs(t2 - T2_TRUE * 1e3) > 6.0:
                bad.append("T2 = %.1f ms, 偏离真值 %.0f ms 超过 6 ms" % (t2, T2_TRUE * 1e3))
    print("\n[自测判读] 期望: T1 0~50 ms/100%%、T2 ≈ 30 ms ± 5、配对全部按内容")
    if bad:
        print("[自测] FAILED:")
        for b in bad:
            print("  - " + b)
        return 1
    print("[自测] PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
