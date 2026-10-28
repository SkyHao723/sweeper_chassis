#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析「/cmd_vel 发布 → 轮子实际转动」的分段延迟。

输入:
  1) /tmp/latency.json   —— probe_latency.py 产出（发布时刻 + 全部 /odom 样本）
  2) /tmp/writets.log    —— LD_PRELOAD 钩子产出（每次 11 B 命令帧写串口的时刻 + 内容）
     （也兼容老的 strace -tt 文本）

分段:
  T1 = 发布时刻 → 节点把 11 B 命令帧 write 到串口
       µs 级精确。厂商节点 Control() 是阻塞大循环 + spin_some 每轮一次,
       回调只能在每轮被服务 → 期望 T1 在 0~50 ms 之间均匀分布, 均值 25 ms。
  T2 = 串口写出 → /odom 首次报告轮子在转
       固件 20 ms 周期 + CAN + 达妙速度环 + 机械, 另加 odom 的 50 ms 量化。
       用「上升沿两帧外推」压到 ~±3 ms。
  T_total = T1 + T2

用法:
  python analyze_latency.py [latency.json] [writets.log]
"""

import json
import os
import re
import statistics
import sys
from datetime import datetime

FRAME_SIZE = 11
FRAME_HEAD = 0x7B

# writets.so:  "<epoch>.<usec> <fd> <size> [<hexpayload>]"
RE_WRITETS = re.compile(r"^(?P<epoch>\d+\.\d+)\s+(?P<fd>\d+)\s+(?P<size>\d+)\s*(?P<hex>[0-9a-fA-F]*)\s*$")

# strace -tt:  "<pid> HH:MM:SS.ffffff write(3, "...", 11) = 11 <...>"
RE_STRACE = re.compile(
    r"^(?P<pid>\d+)\s+(?P<tod>\d{2}:\d{2}:\d{2}\.\d+)\s+"
    r"(?P<call>write|writev)\((?P<args>.*)\)\s*=\s*(?P<ret>-?\d+)"
)
RE_SIZE_TAIL = re.compile(r",\s*(\d+)\s*$")
RE_IOV = re.compile(r"iov_len=(\d+)")


def decode_cmd(hexstr):
    """从 11 B 命令帧内容解出 (vx, wz)，不是命令帧返回 None。
       厂商帧: [0]0x7B [1]AutoRecharge [2]SecurityPLY
               [3..4]vx*1000 BE  [5..6]vy*1000 BE  [7..8]wz*1000 BE
               [9]BCC [10]0x7D
    """
    if not hexstr or len(hexstr) != FRAME_SIZE * 2:
        return None
    try:
        b = bytes.fromhex(hexstr)
    except ValueError:
        return None
    if len(b) != FRAME_SIZE or b[0] != FRAME_HEAD:
        return None
    vx = int.from_bytes(b[3:5], "big", signed=True) / 1000.0
    wz = int.from_bytes(b[7:9], "big", signed=True) / 1000.0
    return (vx, wz)


def tod_candidates(tod, base_date):
    base = datetime(base_date.year, base_date.month, base_date.day).timestamp()
    return [base + d * 86400.0 + tod for d in (-1, 0, 1)]


def load_frames(path, base_date=None, t_ref=None):
    """读「11 B 命令帧写出的时刻」。
       返回 (frames, fmt, nlines)，frames = [(epoch, (vx,wz) or None)]，按时间排序。
    """
    frames = []
    nlines = 0
    fmt = None
    with open(path, "r", errors="replace") as f:
        for ln in f:
            if not ln.strip():
                continue
            nlines += 1
            m = RE_WRITETS.match(ln)
            if m:
                fmt = fmt or "writets"
                if int(m.group("size")) != FRAME_SIZE:
                    continue
                frames.append((float(m.group("epoch")), decode_cmd(m.group("hex"))))
                continue
            m = RE_STRACE.match(ln)
            if m:
                fmt = fmt or "strace"
                if m.group("call") == "write":
                    sm = RE_SIZE_TAIL.search(m.group("args"))
                    size = int(sm.group(1)) if sm else -1
                else:
                    lens = RE_IOV.findall(m.group("args"))
                    size = sum(int(x) for x in lens) if lens else -1
                if size != FRAME_SIZE or int(m.group("ret")) != FRAME_SIZE:
                    continue
                hh, mm, ss = m.group("tod").split(":")
                tod = int(hh) * 3600 + int(mm) * 60 + float(ss)
                cands = tod_candidates(tod, base_date)
                frames.append((min(cands, key=lambda c: abs(c - t_ref)), None))
    frames.sort(key=lambda x: x[0])
    return frames, fmt or "?", nlines


def pair_pubs(pubs, frames, window=0.200, tol=0.02):
    """把每次 /cmd_vel 发布配到它那次串口写出。
       优先按帧内容（vx/wz 一致）匹配, 内容对不上时退回"时间窗内最早的一帧"。
       返回 [(pub, epoch_or_None, how)]，how ∈ {'payload','time',None}
    """
    used = [False] * len(frames)
    out = []
    for p in pubs:
        t0 = p["t"]
        vx_cmd = float(p.get("vx", 0.0))
        wz_cmd = float(p.get("wz", 0.0))
        best = None
        best_how = None
        for i, (ft, cmd) in enumerate(frames):
            if ft < t0:
                continue
            if ft > t0 + window:
                break
            if used[i]:
                continue
            if cmd is not None:
                if abs(cmd[0] - vx_cmd) <= tol and abs(cmd[1] - wz_cmd) <= tol:
                    best, best_how = i, "payload"
                    break
                continue                      # 内容不符: 跳过这帧（可能是别的帧）
            if best is None:                  # 没有内容可比: 先记下最早的一帧
                best, best_how = i, "time"
        if best is None:
            out.append((p, None, None))
        else:
            used[best] = True
            out.append((p, frames[best][0], best_how))
    return out


def fmt_stats(v, unit=1e3, suffix="ms"):
    if not v:
        return "无样本"
    v = sorted(v)
    n = len(v)
    g = lambda q: v[min(n - 1, int(q * (n - 1)))]
    return ("n=%d  min=%.1f  p25=%.1f  p50=%.1f  p75=%.1f  p95=%.1f  max=%.1f  mean=%.1f %s"
            % (n, v[0] * unit, g(0.25) * unit, statistics.median(v) * unit,
               g(0.75) * unit, g(0.95) * unit, v[-1] * unit,
               statistics.fmean(v) * unit, suffix))


def main():
    jpath = sys.argv[1] if len(sys.argv) > 1 else "/tmp/latency.json"
    spath = sys.argv[2] if len(sys.argv) > 2 else "/tmp/writets.log"

    if not os.path.exists(jpath) or not os.path.exists(spath):
        print("!! 找不到输入文件:")
        print("   数据: %s  %s" % (jpath, "OK" if os.path.exists(jpath) else "缺失"))
        print("   写入日志: %s  %s" % (spath, "OK" if os.path.exists(spath) else "缺失"))
        print("   先把主机上的文件拉下来:")
        print("     python tools/rsh.py 192.168.5.11 123456 --get /tmp/latency.json ./latency.json")
        print("     python tools/rsh.py 192.168.5.11 123456 --get /tmp/writets.log  ./writets.log")
        print("   再跑: python tools/latency/analyze_latency.py ./latency.json ./writets.log")
        return 2

    with open(jpath) as f:
        data = json.load(f)
    pubs = sorted(data["published"], key=lambda x: x["t"])
    samples = sorted(data["samples"], key=lambda x: x["st"])
    track = data.get("track", 0.456)
    cfg = data.get("config", {})
    chk = data.get("check", {})
    if not pubs or not samples:
        print("!! 数据为空: published=%d samples=%d" % (len(pubs), len(samples)))
        return 1

    def vwheel(s):
        return abs(s["vx"]) + abs(s["wz"]) * track * 0.5

    t_med = statistics.median([p["t"] for p in pubs])
    base_date = datetime.fromtimestamp(t_med).date()
    frames, fmt, nlines = load_frames(spath, base_date, t_med)

    print("=" * 76)
    print("链路延迟分析: /cmd_vel 发布 → 轮子实际转动")
    print("=" * 76)
    print("参数: %s" % {k: cfg.get(k) for k in ("speed", "trials", "settle", "hold")})
    print("写入日志(%s): %d 行, 11 B 帧 %d 个, 其中能解出命令 %d 个"
          % (fmt, nlines, len(frames), sum(1 for _, c in frames if c is not None)))
    print("probe: %d 次发布, %d 个 /odom 样本" % (len(pubs), len(samples)))
    if chk:
        print("方向自检: vx=%+.3f m/s wz=%+.3f rad/s -> %s  (静止 vwheel 最大 %.4f)"
              % (chk.get("vx", 0), chk.get("wz", 0), chk.get("verdict"), chk.get("noise_max", 0)))

    pairs = pair_pubs(pubs, frames)
    n_pay = sum(1 for _, _, h in pairs if h == "payload")
    n_time = sum(1 for _, _, h in pairs if h == "time")
    n_none = sum(1 for _, _, h in pairs if h is None)
    t1_all = [w - p["t"] for p, w, h in pairs if w is not None]
    t1_step = [w - p["t"] for p, w, h in pairs if w is not None and "step" in p.get("tag", "")]

    print()
    print("配对: 按内容 %d, 按时间兜底 %d, 没配上 %d" % (n_pay, n_time, n_none))
    print()
    print("--- T1: /cmd_vel 发布 → 节点把 11 B 命令帧写到串口 ---")
    print("  全部: " + fmt_stats(t1_all))
    print("  阶跃: " + fmt_stats(t1_step))
    if t1_all:
        bins = [0] * 10
        for v in t1_all:
            bins[min(9, max(0, int(v * 1e3 / 5)))] += 1
        print("  5 ms 直方图(0..50ms): " + " ".join("%d" % b for b in bins))
        lo = sum(1 for v in t1_all if 0 <= v <= 0.050)
        print("  落在 0~50 ms: %d/%d = %.0f%%  (厂商节点阻塞大循环 + spin_some 20 Hz 的直接证据)"
              % (lo, len(t1_all), 100.0 * lo / len(t1_all)))

    # ---------- T2 ----------
    noise = float(chk.get("noise_max", 0.0) or 0.0)
    thresh = max(0.010, 2.0 * noise)
    step_pairs = [(p, w) for p, w, h in pairs
                  if w is not None and "step" in p.get("tag", "")]
    res = []
    for p, w in step_pairs:
        cand = [s for s in samples if s["st"] >= w]
        k = None
        for idx in range(len(cand) - 1):
            if vwheel(cand[idx]) > thresh and vwheel(cand[idx + 1]) > thresh * 0.5:
                k = idx
                break
        if k is None:
            continue
        ub = cand[k]["st"]
        lb = cand[k - 1]["st"] if k > 0 else w
        v1 = vwheel(cand[k])
        v2 = vwheel(cand[k + 1]) if k + 1 < len(cand) else v1
        plat = [vwheel(s) for s in cand if ub + 0.15 <= s["st"] <= ub + 0.35]
        plateau = statistics.median(plat) if plat else 0.0
        rise = None
        if v2 > v1 > 0 and plateau > 0 and v1 < 0.9 * plateau:
            dt = cand[k + 1]["st"] - ub
            if dt > 1e-6:
                slope = (v2 - v1) / dt
                if slope > 1e-6:
                    rise = ub - v1 / slope
        res.append({
            "tag": p["tag"], "t1": w - p["t"], "lb": lb - w, "ub": ub - w,
            "rise": (rise - w) if rise is not None else None,
            "plateau": plateau, "cmd": p["vx"],
        })

    print()
    print("--- T2: 节点写出命令帧 → /odom 首次报告轮子在转 (阈值 %.3f m/s) ---" % thresh)
    if not res:
        print("  !! 没有有效 trial —— 车是不是没动?")
    else:
        lbs = [r["lb"] for r in res]
        ubs = [r["ub"] for r in res]
        rises = [r["rise"] for r in res if r["rise"] is not None]
        print("  下界(上一帧): " + fmt_stats(lbs))
        print("  上界(该帧)  : " + fmt_stats(ubs))
        print("  上升沿外推  : " + fmt_stats(rises))
        print("  (下界/上界含'轮子要转过门限才报动'的偏置, 只当区间证据;)")
        print("  (真值看上升沿外推: 假设 50 ms 内速度线性上升, 由首两帧求过零点)")
        if rises:
            s = sorted(rises)
            print("  >> T2 = %.1f ms  (p25~p75 %.1f~%.1f, n=%d)"
                  % (statistics.median(rises) * 1e3,
                     s[int(0.25 * (len(s) - 1))] * 1e3,
                     s[int(0.75 * (len(s) - 1))] * 1e3, len(s)))
        pl = statistics.median([r["plateau"] for r in res])
        cm = statistics.median([abs(r["cmd"]) for r in res])
        print("  平台速度: 实测 %.3f / 命令 %.3f m/s (%s)"
              % (pl, cm, "OK" if cm > 0 and abs(pl / cm - 1) < 0.3 else "!! 偏差大"))
        print()
        print("--- T_total: 发布 → odom 报告轮子转动 ---")
        if rises and t1_step:
            t1m = statistics.median(t1_step)
            t2m = statistics.median(rises)
            print("  >> T_total = T1 %.1f + T2 %.1f = %.1f ms" % (t1m * 1e3, t2m * 1e3, (t1m + t2m) * 1e3))
        print("  (量化上界参考: " + fmt_stats([r["t1"] + r["ub"] for r in res]) + ")")
        print()
        print("--- 逐 trial ---")
        print("  %-9s %8s %8s %8s %8s %7s" % ("tag", "T1/ms", "回码前", "回码后", "上升沿", "平台"))
        for r in res:
            print("  %-9s %8.1f %8.1f %8.1f %8s %7.3f"
                  % (r["tag"], r["t1"] * 1e3, r["lb"] * 1e3, r["ub"] * 1e3,
                     ("%.1f" % (r["rise"] * 1e3)) if r["rise"] is not None else "-",
                     r["plateau"]))

    obs = [s["t"] - s["st"] for s in samples if 0 <= s["t"] - s["st"] < 1.0]
    print()
    print("--- 观测链路: 串口帧到达本机 → probe 收到 /odom ---")
    print("  " + fmt_stats(obs))
    print()
    print("=" * 76)
    print("T1 = 主机侧(ROS 回调调度 + DDS + 串口写字节)")
    print("T2 = 固件(20 ms 控制周期) + CAN + 达妙驱动器速度环 + 机械")
    if t1_all:
        print("T1 若呈 0~50 ms 均匀分布 -> 证实厂商节点 spin_some 20 Hz 是主机侧主延迟,")
        print("  改多线程 executor / 单独线程收 cmd_vel 就能省掉均值约 25 ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
