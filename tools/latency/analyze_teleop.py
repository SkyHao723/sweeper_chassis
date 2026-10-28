#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 teleop_key_probe.py 的结果，把全链路切成四段。

输入:
  --json   teleop_key_probe.py 写出的 /tmp/key_probe.json
  --hook   writets.so 的日志（<epoch> <fd> <size> <payloadhex>）

输出四段:
  T0  按键 -> /cmd_vel 到达（teleop 进程 + DDS）
  T1  /cmd_vel 到达 -> 底盘节点把 11 B 命令帧写出串口（20 Hz 阻塞大循环量化）
  T2  串口写出 -> 轮子开始转（只能给区间：/odom 网格 20 Hz）
  T_total 按键 -> 轮子开始转（= T0 + T1 + T2）

用法:
  python analyze_teleop.py --json key_probe.json --hook writets_key.log
"""
import argparse
import json
import statistics
import sys

MOVING_VX = 0.02      # m/s，判"轮子开始转"的速度阈值（静止噪声 0.002，命令 0.2）
PAIR_TOL = 0.02       # m/s，命令帧内容配对容差


def pct(vals, q):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = (len(s) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def load_hook(path):
    frames = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            p = ln.split()
            if len(p) < 4:
                continue
            try:
                t = float(p[0])
            except ValueError:
                continue
            if p[2] != "11":
                continue
            hx = p[3]
            if len(hx) != 22:
                continue
            b = bytes.fromhex(hx)
            if b[0] != 0x7B or b[10] != 0x7D:
                continue
            vx = int.from_bytes(b[3:5], "big", signed=True) / 1000.0
            vy = int.from_bytes(b[5:7], "big", signed=True) / 1000.0
            wz = int.from_bytes(b[7:9], "big", signed=True) / 1000.0
            frames.append({"t": t, "vx": vx, "vy": vy, "wz": wz})
    frames.sort(key=lambda x: x["t"])
    return frames


def fmt(vals, unit="ms"):
    if not vals:
        return "n=0"
    return ("n=%d min %.1f p25 %.1f p50 %.1f p75 %.1f p95 %.1f max %.1f mean %.1f %s"
            % (len(vals), min(vals), pct(vals, .25), pct(vals, .5), pct(vals, .75),
               pct(vals, .95), max(vals), statistics.fmean(vals), unit))


def hist(vals, lo, hi, step):
    nb = int((hi - lo) / step) + 1
    bins = [0] * nb
    for v in vals:
        i = int((v - lo) / step)
        i = max(0, min(nb - 1, i))
        bins[i] += 1
    return " ".join("%d" % b for b in bins), [
        "%.0f~%.0f" % (lo + i * step, lo + (i + 1) * step) for i in range(nb)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--hook", required=True)
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    with open(args.json, "r", encoding="utf-8") as f:
        d = json.load(f)
    keys = d["keys"]
    cmd = d["cmd"]
    odom = d["odom"]
    meta = d["meta"]
    cmd.sort(key=lambda x: x[0])
    odom.sort(key=lambda x: x[0])
    frames = load_hook(args.hook)

    st = [s[3] for s in odom]
    d_st = [b - a for a, b in zip(st, st[1:]) if 0.005 < b - a < 0.5]
    grid = statistics.median(d_st) if d_st else float("nan")
    arr_off = [(s[0] - s[3]) * 1000 for s in odom if 0 < s[0] - s[3] < 1.0]

    print("=" * 78)
    print("数据: keys=%d  /cmd_vel=%d  /odom=%d  钩子命令帧=%d"
          % (len(keys), len(cmd), len(odom), len(frames)))
    print("/odom 观测网格: 中位 %.1f ms (%.2f Hz)" % (grid * 1000, 1.0 / grid))
    print("/odom 到达-时间戳: 中位 %.1f ms (p25 %.1f p95 %.1f)"
          % (statistics.median(arr_off), pct(arr_off, .25), pct(arr_off, .95)))
    print("命令: speed=%.3f m/s  trials=%d  settle=%.2f hold=%.2f 方向自检=%s"
          % (meta["speed"], meta["trials"], meta["settle"], meta["hold"], meta["judge"]))

    steps = [k for k in keys if k["label"] == "step"]
    print("按键事件: %d 个 step" % len(steps))

    # ---------- T0: 按键 -> /cmd_vel 到达 ----------
    t0s, pairs = [], []
    for k in steps:
        tk = k["t"]
        cand = [c for c in cmd if c[0] >= tk - 0.002]
        if not cand:
            continue
        c = cand[0]
        if c[0] - tk > 0.5:
            continue
        t0s.append((c[0] - tk) * 1000)
        pairs.append({"trial": k["trial"], "t_key": tk, "cmd": c})

    # ---------- T1: cmd_vel -> 串口写出 ----------
    t1s = []
    for p in pairs:
        tc = p["cmd"][0]
        want_vx, want_wz = p["cmd"][1], p["cmd"][3]
        hit = None
        for fr in frames:
            if fr["t"] < tc - 0.003:
                continue
            if abs(fr["vx"] - want_vx) <= PAIR_TOL and abs(fr["wz"] - want_wz) <= PAIR_TOL:
                hit = fr
                break
            if fr["t"] - tc > 0.30:
                break
        if hit is None:
            p["frame"] = None
            continue
        p["frame"] = hit
        t1s.append((hit["t"] - tc) * 1000)

    # ---------- T2: 串口写出 -> 轮子开始转 ----------
    t2_lo, t2_hi, totals_hi, rows = [], [], [], []
    for idx, p in enumerate(pairs):
        if p.get("frame") is None:
            continue
        tw = p["frame"]["t"]
        want_sign = 1.0 if p["cmd"][1] > 0 else -1.0
        t_lim = pairs[idx + 1]["t_key"] if idx + 1 < len(pairs) else tw + 1.5
        win = [s for s in odom if s[3] > tw - 0.30 and s[0] < t_lim]
        first_mv = None
        last_still = None
        for s in win:
            moving = (s[1] * want_sign) > MOVING_VX
            if s[3] <= tw:
                last_still = s
                continue
            if moving:
                first_mv = s
                break
            last_still = s
        if first_mv is None:
            rows.append((p["trial"], None, None, None, None, False))
            continue
        pre = [s for s in win if s[3] <= tw]
        vx_pre = pre[-1][1] if pre else None
        tau_lo = (last_still[3] - tw) * 1000 if last_still is not None else 0.0
        tau_hi = (first_mv[3] - tw) * 1000
        # "写串口时确实静止"：写之前速度归零，且写后 20 ms 内马上确认过没动
        at_rest = (vx_pre is not None) and (abs(vx_pre) <= MOVING_VX) and (tau_lo <= 20.0)
        # 首个动样本相对按键的总延迟（上界，因为真实起始在两者之间）
        tot = (first_mv[3] - p["t_key"]) * 1000
        if at_rest:
            t2_lo.append(tau_lo)
            t2_hi.append(tau_hi)
        totals_hi.append(tot)
        rows.append((p["trial"], (p["cmd"][0] - p["t_key"]) * 1000,
                     (tw - p["cmd"][0]) * 1000, tau_lo, tau_hi, at_rest))

    print("-" * 78)
    print("T0  按键 -> /cmd_vel 到达        : " + fmt(t0s))
    print("T1  /cmd_vel -> 串口命令帧写出   : " + fmt(t1s))
    if t1s:
        h, labels = hist(t1s, 0, 60, 5)
        print("    T1 5 ms 直方图(0~60): " + h)
    print("    T0+T1 软件段 (按键->写出)     : "
          + fmt([a + b for a, b in zip(t0s, t1s)]))
    print("-" * 78)
    print("T2  串口写出 -> 轮子开始转       : 只能给区间，观测网格 %.1f ms" % (grid * 1000))
    if t2_lo:
        print("    最后一帧明确没动 tau <= %s" % fmt(t2_lo))
        print("    第一帧明确在动 tau >  %s" % fmt(t2_hi))
        print("    => 真实起点落在 (%.1f, %.1f) ms 内 中位区间"
              % (pct(t2_lo, .5), pct(t2_hi, .5)))
    print("-" * 78)
    print("T_total 按键 -> 首个动样本(%s): %s"
          % ("上界", fmt(totals_hi)))
    if totals_hi and t1s:
        print("    => 按键->轮子开始转 估计 %d~%d ms（T0+T1 中位 + T2 区间）"
              % (int(pct([a + b for a, b in zip(t0s, t1s)], .5) + pct(t2_lo, .5)),
                 int(pct([a + b for a, b in zip(t0s, t1s)], .5) + pct(t2_hi, .5))))

    print("-" * 78)
    print("逐 trial (T0/T1 单位 ms; T2 为写串口后的区间; * = 写串口时车还没停稳，不计入 T2)")
    print("  #   T0     T1    T2lo   T2hi")
    for r in rows:
        if r[1] is None:
            print("  %2d  ----  ----  ----  ----(没测到开始动)" % r[0])
        else:
            print("  %2d  %5.1f  %5.1f  %5.1f  %5.1f%s"
                  % (r[0], r[1], r[2], r[3], r[4], "" if r[5] else "  *"))

    # 逐 trial 锯齿（验证 20 Hz 量化）
    t12 = [r[1] + r[2] for r in rows if r[1] is not None]
    if len(t12) >= 5:
        print("-" * 78)
        print("T0+T1 逐 trial 序列 (看是否锯齿/被 %0.1f ms 网格量化):" % (grid * 1000))
        print("  " + " -> ".join("%.1f" % v for v in t12))


if __name__ == "__main__":
    main()
