#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 tools/ramp_test.py 的输出：验证固件斜坡是否真的生效。

判据（固件宏 RAMP_VX_ACC=1.0 / RAMP_VX_DEC=1.5 m/s²，控制拍 20 ms）：
  - 0 → 0.20 m/s：目标 ω 每拍 +0.234 rad/s（= 23.4 个 x100 计数），
    10 拍 = 200 ms 到顶
  - 0.20 → 0：每拍 -0.351（= 35.1 计数），约 7 拍 = 140 ms 归零
  - 反向 +0.20 → -0.20：一律用减速斜率，约 14 拍 = 280 ms
轮半径 0.0855 m。
"""
import json
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # pragma: no cover
    pass

R_WHEEL = 0.0855
EXPECT_UP_STEP = 1.0 * 0.020 / R_WHEEL * 100.0      # 每拍目标 ω 增量(x100)
EXPECT_DOWN_STEP = 1.5 * 0.020 / R_WHEEL * 100.0


def load(path):
    with open(path) as fh:
        return json.load(fh)


def find_phase(phases, prefix, cycle):
    name = "%s%d" % (prefix, cycle)
    for idx, row in enumerate(phases):
        if row[0] == name:
            return idx, row[3]        # idx, t_start
    return None, None


def sample_at_edge(samples, t_start):
    """t_start 之后第一条诊断帧。"""
    for s in samples:
        if s[0] >= t_start:
            return s
    return None


def analyze(data):
    phases = data["phases"]
    samples = data["samples"]
    meta = data["meta"]
    print("=" * 72)
    print("mata: 端口=%s 目标速度=%d mm/s 循环=%d 保持=%.2fs 帧数=%d 发送帧=%d"
          % (meta["port"], meta["speed_mm"], meta["cycles"], meta["hold_s"],
             meta["diag_count"], meta["tx_count"]))
    vers = sorted({s[7] for s in samples})
    hb = sorted({(s[8] >> 7) & 1 for s in samples})
    flags = sorted({s[6] for s in samples})
    print("固件版本[30]=%s   心跳模式 bit7=%s   flags=%s" % (vers, hb, flags))
    print("=" * 72)

    cycles = meta["cycles"]
    for cyc in range(cycles):
        print("\n--- 第 %d 轮 ---" % (cyc + 1))
        # 上冲段
        idx, t0 = find_phase(phases, "up", cyc)
        if idx is None:
            continue
        s_first = sample_at_edge(samples, t0)
        seg = [s for s in samples if t0 <= s[0] < t0 + phases[idx][2]]
        if not seg:
            continue
        tgt0 = abs(s_first[2]) if s_first else 0
        peak = max(abs(s[2]) for s in seg)
        # 到 95% 的时间
        t95 = None
        for s in seg:
            if abs(s[2]) >= 0.95 * peak:
                t95 = (s[0] - t0) * 1000.0
                break
        # 线性段斜率（相邻帧目标增量）
        steps = []
        for a, b in zip(seg, seg[1:]):
            dt = (b[0] - a[0]) * 1000.0
            dv = abs(b[2]) - abs(a[2])
            if 10.0 < dt < 80.0 and dv > 0.5:
                steps.append(dv / dt * 20.0)     # 折算成"每 20 ms 增量"
        act_peak = max(abs(s[4]) for s in seg)
        print("  起步 0→%.2f m/s: 目标ω峰值=%.1f (期望 %.1f)  到95%%用时=%.0f ms"
              % (meta["speed_mm"] / 1000.0, peak,
                 meta["speed_mm"] / 1000.0 / R_WHEEL * 100.0, t95 or -1))
        if steps:
            print("    目标每拍增量: 中位 %.1f 计数 (期望 %.1f) → 实测斜率 %.2f m/s²"
                  % (sorted(steps)[len(steps) // 2],
                     EXPECT_UP_STEP,
                     sorted(steps)[len(steps) // 2] / 100.0 * R_WHEEL / 0.020))
        print("    实际ω峰值=%.1f（跟随率 %.0f%%）"
              % (act_peak, 100.0 * act_peak / peak if peak else 0))

        # 回零段
        idx2, t2 = find_phase(phases, "stop", cyc) if cyc >= 0 else (None, None)
        if idx2 is None:
            continue
        seg2 = [s for s in samples if t2 <= s[0] < t2 + phases[idx2][2]]
        if not seg2:
            continue
        start_val = abs(seg2[0][2])
        t_zero = None
        for s in seg2:
            if abs(s[2]) <= 0.5:
                t_zero = (s[0] - t2) * 1000.0
                break
        print("  回零 %.2f→0: 起始目标ω=%.1f 到 0 用时=%s ms"
              % (start_val / 100.0 * R_WHEEL, start_val,
                 "%.0f" % t_zero if t_zero is not None else "未到 0"))
        print("    实际ω 到 5%% 用时=%s ms" % (
            "%.0f" % next(((s[0] - t2) * 1000.0 for s in seg2
                           if abs(s[4]) <= 5.0), -1)))

    # 全段概要
    print("\n" + "=" * 72)
    print("全段目标 ω 序列（前 60 帧，x100 rad/s，左轮）:")
    print("  " + " ".join("%.0f" % s[2] for s in samples[:60]))
    print("全段实际 ω 序列（前 60 帧）:")
    print("  " + " ".join("%.0f" % s[4] for s in samples[:60]))


if __name__ == "__main__":
    analyze(load(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ramp_test.json"))
