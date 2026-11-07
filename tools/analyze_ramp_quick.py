#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""快速核对固件斜坡是否按预期斜率工作（目标 ω 侧，不看实际轮速）。

判据来自宏：
  加速 RAMP_VX_ACC = 1.0 m/s^2 -> 每 20 ms 控制拍 +0.02 m/s
       = 0.02/0.0855 = 0.2339 rad/s = 23.39 (计数, x100)
  减速 RAMP_VX_DEC = 0.40 m/s^2 -> 每拍 -0.008 m/s = 9.36 计数
所以 0 -> 0.20 m/s 应 ~10 拍(~200 ms) 到 233；233 -> 0 应 ~25 拍(~500 ms)。

用法: python tools/analyze_ramp_quick.py <ramp.json>
"""
import sys
import json

sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def absv(x):
    return abs(x)


def main(path):
    d = json.load(open(path, encoding="utf-8"))
    ph = d["phases"]
    smp = d["samples"]
    print("总帧 %d" % len(smp))

    # 只看左轮目标（两轮同幅值，scrubber DM_DIR=-1 所以是负数）
    for name, vx, hold, t0 in ph:
        if name.startswith("pre") or name.startswith("post"):
            continue
        seg = [x for x in smp if t0 <= x[0] < t0 + hold]
        if len(seg) < 5:
            continue
        tg = [absv(x[2]) for x in seg]
        # 找第一个 >5 的帧（起步）和随后第一次到达 95% 的帧
        i0 = next((i for i, v in enumerate(tg) if v > 5), None)
        if i0 is None:
            continue
        peak = max(tg)
        i95 = next((i for i, v in enumerate(tg) if v >= 0.95 * peak), None)
        dt_up = (seg[i95][0] - seg[i0][0]) * 1000 if i95 else float("nan")
        # 减速：从峰值第一次回落到 <=5 的帧
        iz = next((i for i in range(i95 or 0, len(tg)) if tg[i] <= 5), None)
        dt_dn = (seg[iz][0] - seg[i95][0]) * 1000 if iz else float("nan")
        print("")
        print("[%s] t0=%.2f 目标峰 %.1f" % (name, t0, peak))
        print("  起步 %d -> 95%%: %.0f ms (期望 ~200)" % (tg[i0], dt_up))
        if iz is not None:
            print("  95%% -> 归零:    %.0f ms (期望 ~500)" % dt_dn)
        # 前 12 拍的目标序列，看斜率是否 ~23.4/拍 上升、~9.4/拍 下降
        print("  序列: %s" % [int(v) for v in tg[:14]])

    # 检查每拍增量（上升段应 ~23，下降段应 ~9）
    print("")
    print("=== 每拍目标增量统计（诊断帧 50 ms 间隔，故乘 2.5 才是 20 ms 一拍的增量）===")
    ups, dns = [], []
    for i in range(1, len(smp)):
        dv = absv(smp[i][2]) - absv(smp[i - 1][2])
        dt = smp[i][0] - smp[i - 1][0]
        if dt <= 0:
            continue
        rate = dv / dt / 2.5  # -> 每 20 ms 拍的增量
        if dv > 3 and absv(smp[i][2]) < 240:
            ups.append(rate)
        elif dv < -3:
            dns.append(rate)
    if ups:
        print("  上升增量: n=%d 中位 %.1f (期望 23.4)" % (len(ups), sorted(ups)[len(ups) // 2]))
    if dns:
        print("  下降增量: n=%d 中位 %.1f (期望 -9.4)" % (len(dns), sorted(dns)[len(dns) // 2]))


if __name__ == "__main__":
    main(sys.argv[1])
