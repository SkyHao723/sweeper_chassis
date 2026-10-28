#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用「合并重建」把 T2 从 50 ms 量化压到 ~5 ms 的分辨率, 并给出诚实的区间。

为什么需要它
------------
/odom 只有 20 Hz。单看一次 trial, 只能说"轮子开始转的时刻落在哪两帧之间"(区间 50 ms)。
但把 20 个 trial 的全部 (τ, v) 样本按 τ 分箱取中位数, 就能重建连续的响应曲线 v(τ),
τ = 串口写出命令帧 → odom 采样时刻。

★ 但有个结构性的坑（这次实测才看清）:
  厂商节点每轮循环是「等一帧 24B 遥测 → 发 /odom → spin_some 处理 cmd_vel → 串口写」,
  所以「串口写」永远紧跟在一次 /odom 发布之后 (实测 τ ≈ 0~6 ms 就有一帧)。
  → 想采样 τ ∈ (6, 40) ms 这个窗口是**做不到**的: 观测网格和命令网格是锁在一起的。
  所以 T2 只能给区间: 下界 = 最后一帧"明确没动"的 τ, 上界 = 第一帧"明确在动"的 τ。

输出
----
  阶跃响应曲线 + 停车减速曲线
  T2 区间 与 三种估计(保守/线性外推/物理预算)
"""

import json
import os
import statistics
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import analyze_latency as A          # noqa: E402

BIN = 0.005          # 分箱宽度 5 ms
TAU_MIN, TAU_MAX = -0.060, 0.700


def signed_vx(s, sign):
    """命令方向归一化后的轮速 (m/s)。wz 贡献很小, 只用 vx。"""
    return sign * s["vx"]


def collect(pairs, samples, step_key, tau_max):
    """把每个 trial 的 (τ, v) 汇总。step_key='step' 用步进帧, 'stop' 用停车帧。
       停车帧本身 vx=0, 方向要沿用**前面最近一条非零命令**的方向, 否则正负抵消。"""
    pool = []
    per_trial = []
    for idx, (p, w, how) in enumerate(pairs):
        tag = p.get("tag", "")
        if step_key not in tag:
            continue
        if step_key == "stop":
            sign = None
            for j in range(idx - 1, -1, -1):
                pj = pairs[j][0]
                if abs(pj["vx"]) > 0.01:
                    sign = 1.0 if pj["vx"] > 0 else -1.0
                    break
            if sign is None:
                continue
        else:
            sign = 1.0 if p["vx"] >= 0 else -1.0
        lo, hi = w + TAU_MIN, w + tau_max
        rows = []
        for s in samples:
            if s["st"] < lo or s["st"] > hi:
                continue
            rows.append((s["st"] - w, signed_vx(s, sign)))
        per_trial.append({"tag": tag, "w": w, "sign": sign, "rows": rows,
                          "t1": (w - p["t"])})
        pool.extend(rows)
    return pool, per_trial


def binned(pool, binw=BIN, nmin=1):
    """τ → 中位数 v, 返回 [(τ_center, median_v, n)]"""
    if not pool:
        return []
    lo = min(t for t, _ in pool)
    hi = max(t for t, _ in pool)
    nb = int((hi - lo) / binw) + 1
    buckets = [[] for _ in range(nb)]
    for t, v in pool:
        k = int((t - lo) / binw)
        if 0 <= k < nb:
            buckets[k].append(v)
    out = []
    for k, b in enumerate(buckets):
        if b:
            out.append((lo + (k + 0.5) * binw, statistics.median(b), len(b)))
    return out


def analyse_onset(curve, plateau):
    """给出 T2 的三种估计。返回 dict。"""
    res = {}
    # 0) 平台
    res["plateau"] = plateau
    # 1) 下界: 最后一个"明确没动"的 bin (要求样本数 >=3, 避免单样本噪声)
    lo_bound = None
    for t, v, n in curve:
        if t <= 0.30 and n >= 3 and abs(v) < 0.02 * plateau:
            lo_bound = max(t, lo_bound) if lo_bound is not None else t
    res["lo"] = lo_bound
    # 2) 上界: 第一个"明确在动"的 bin
    hi_bound = None
    for t, v, n in curve:
        if n >= 3 and abs(v) > 0.05 * plateau:
            hi_bound = t
            break
    res["hi"] = hi_bound
    # 3) 线性外推: 用第一个"动起来"的 bin 与下一个 bin 之间的斜率过零
    ext = None
    if hi_bound is not None:
        after = [(t, v) for (t, v, n) in curve
                 if t > hi_bound and n >= 3 and abs(v) < 0.90 * plateau]
        if after:
            t1b, v1b = hi_bound, None
            for t, v, n in curve:
                if abs(t - hi_bound) < 1e-9:
                    v1b = v
                    break
            t2b, v2b = after[0]
            if v1b is not None and t2b > t1b and v2b > v1b:
                slope = (v2b - v1b) / (t2b - t1b)
                if slope > 1e-6:
                    ext = t1b - v1b / slope
                    res["slope"] = slope
    res["ext"] = ext
    return res


def report_curve(curve, plateau, t_end=0.52):
    print("\nτ(ms)   中位v(m/s)   v/平台   n   曲线")
    for t, v, n in curve:
        if t < TAU_MIN or t > t_end:
            continue
        r = v / plateau if plateau else 0.0
        bar = "#" * int(max(0.0, r) * 40)
        print("%6.1f  %+8.3f  %5.1f%%  %3d  %s" % (t * 1e3, v, r * 100, n, bar))


def main():
    jpath = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, "results", "latency_20261028_0936.json")
    spath = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        HERE, "results", "writets_20261028_0936.log")

    with open(jpath) as f:
        data = json.load(f)
    from datetime import datetime
    pubs = sorted(data["published"], key=lambda x: x["t"])
    samples = sorted(data["samples"], key=lambda x: x["st"])
    t_med = statistics.median([p["t"] for p in pubs])
    base = datetime.fromtimestamp(t_med).date()
    frames, fmt, nlines = A.load_frames(spath, base, t_med)
    pairs = A.pair_pubs(pubs, frames)
    t1 = [w - p["t"] for p, w, h in pairs if w is not None and "step" in p.get("tag", "")]

    print("=" * 74)
    print("响应曲线重建: %s" % os.path.basename(jpath))
    print("=" * 74)
    print("11 B 帧 %d, 发布 %d, /odom 样本 %d, T1(阶跃) 中位 %.1f ms"
          % (len(frames), len(pubs), len(samples),
             statistics.median(t1) * 1e3 if t1 else -1))

    # ---------- 阶跃响应 ----------
    pool, per = collect(pairs, samples, "step", 0.65)
    curve = binned(pool)
    if not curve:
        print("!! 没有阶跃样本")
        return 1
    plateau = statistics.median([v for t, v, n in curve if 0.30 <= t <= 0.45])
    cmd = statistics.median([abs(p["vx"]) for p, w, h in pairs if "step" in p.get("tag", "")])
    print("\n阶跃样本 %d 个 / %d trial" % (len(pool), len(per)))
    print("命令 %.3f m/s, 30~45%% 段的平台 %.3f m/s" % (cmd, plateau))
    report_curve(curve, plateau)

    onset = analyse_onset(curve, plateau)
    print()
    print("--- T2 = 串口写出命令帧 → 轮子真的开始转 ---")
    if onset["lo"] is not None:
        print("  下界(最后一帧明确没动): τ > %.1f ms" % (onset["lo"] * 1e3))
    if onset["hi"] is not None:
        print("  上界(第一帧明确在动):   τ < %.1f ms" % (onset["hi"] * 1e3))
    if onset["ext"] is not None:
        print("  线性外推过零(若加速度恒定): %.1f ms   上升段斜率 %.2f m/s^2"
              % (onset["ext"] * 1e3, onset["slope"] * 1e3))
    if onset["lo"] is not None and onset["hi"] is not None:
        mid = 0.5 * (onset["lo"] + onset["hi"])
        print("  >> 实测区间 %.0f ~ %.0f ms, 物理预算(USB ~2 + STM32 20 ms 周期 0~20 + "
              "驱动/机械 ~10) ≈ 12~32 ms" % (onset["lo"] * 1e3, onset["hi"] * 1e3))
        print("  >> T2 取 ~%.0f ms (区间中点), 保守范围 %.0f~%.0f ms"
              % (mid * 1e3, onset["lo"] * 1e3, onset["hi"] * 1e3))
        if t1:
            t1m = statistics.median(t1)
            print("  >> T_total = T1 %.1f + T2 %.0f = %.0f ms"
                  % (t1m * 1e3, mid * 1e3, (t1m + mid) * 1e3))

    # ---------- 过冲 / 稳态 ----------
    print()
    print("--- 速度忠实度(从同一条重建曲线看) ---")
    peak = max(v for t, v, n in curve if t > 0)
    t_peak = [t for t, v, n in curve if v == peak][0]
    ss = statistics.median([v for t, v, n in curve if 0.42 <= t <= 0.55])
    print("  峰值 %.3f m/s @ %.0f ms (命令的 %+.0f%%)"
          % (peak, t_peak * 1e3, (peak / cmd - 1) * 100 if cmd else 0))
    print("  0.42~0.55 s 稳态 %.3f m/s (命令的 %+.0f%%)"
          % (ss, (ss / cmd - 1) * 100 if cmd else 0))

    # ---------- 停车响应 ----------
    pool2, per2 = collect(pairs, samples, "stop", 0.65)
    curve2 = binned(pool2)
    if curve2:
        print("\n--- 发 0 之后的减速曲线 (%d trial) ---" % len(per2))
        print("τ(ms)   中位v(m/s)   n")
        for t, v, n in curve2:
            if -0.05 <= t <= 0.55:
                print("%6.1f  %+8.3f  %3d" % (t * 1e3, v, n))
        for t, v, n in curve2:
            if t > 0 and abs(v) < 0.05 * plateau:
                print(">> T_stop (减速到 5%% 平台) ≈ %.1f ms" % (t * 1e3))
                break
    return 0


if __name__ == "__main__":
    sys.exit(main())
