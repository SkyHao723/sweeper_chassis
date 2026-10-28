#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析 /tmp/latency.json + /tmp/strace_node.log，得出「/cmd_vel 发布 → 轮子转动」的分段延迟。

分段:
  T1 = 发布时刻 → 节点把 11 B 命令帧 write 到串口的时刻
       （strace 抓的墙钟，µs 级精确。厂商节点 Control() 是阻塞大循环 + spin_some，
        回调只能在每轮跑一次 → 期望 T1 在 0~50 ms 之间均匀分布，均值 25 ms）
  T2 = 串口写出 → /odom 首次报告轮子在转
       （odom 20 Hz → 50 ms 量化；用 N 次取最小 + 线性拟合外推求亚采样估计）
  T_total = T1 + T2

用法:
  python analyze_latency.py [latency.json] [strace_node.log]
"""

import json
import re
import statistics
import sys
from datetime import datetime

LINE = re.compile(
    r"^(?P<pid>\d+)\s+(?P<tod>\d{2}:\d{2}:\d{2}\.\d+)\s+"
    r"(?P<call>write|writev)\((?P<args>.*)\)\s*=\s*(?P<ret>-?\d+)"
)
SIZE_TAIL = re.compile(r",\s*(\d+)\s*$")
IOV = re.compile(r"iov_len=(\d+)")


def parse_strace(path):
    """返回 [(pid, tod_seconds, size, ret)]"""
    out = []
    with open(path, "r", errors="replace") as f:
        for ln in f:
            m = LINE.match(ln)
            if not m:
                continue
            hh, mm, ss = m.group("tod").split(":")
            tod = int(hh) * 3600 + int(mm) * 60 + float(ss)
            call = m.group("call")
            args = m.group("args")
            if call == "write":
                sm = SIZE_TAIL.search(args)
                size = int(sm.group(1)) if sm else -1
            else:
                lens = IOV.findall(args)
                size = sum(int(x) for x in lens) if lens else -1
            out.append((int(m.group("pid")), tod, size, int(m.group("ret"))))
    return out


def tod_candidates(tod, base_date):
    base = datetime(base_date.year, base_date.month, base_date.day).timestamp()
    return [base + d * 86400.0 + tod for d in (-1, 0, 1)]


def main():
    jpath = sys.argv[1] if len(sys.argv) > 1 else "/tmp/latency.json"
    spath = sys.argv[2] if len(sys.argv) > 2 else "/tmp/strace_node.log"

    import os
    if not os.path.exists(jpath) or not os.path.exists(spath):
        print("!! 找不到输入文件:")
        print("   数据: %s  %s" % (jpath, "OK" if os.path.exists(jpath) else "缺失"))
        print("   strace: %s  %s" % (spath, "OK" if os.path.exists(spath) else "缺失"))
        print("   先把主机上的 /tmp/latency.json 和 /tmp/strace_node.log 拉下来:")
        print("     python tools/rsh.py 192.168.5.11 123456 --get /tmp/latency.json  ./latency.json")
        print("     python tools/rsh.py 192.168.5.11 123456 --get /tmp/strace_node.log ./strace_node.log")
        print("   再跑: python tools/latency/analyze_latency.py ./latency.json ./strace_node.log")
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

    # ---------- strace: 11 B 帧的 write 时刻 ----------
    raw = parse_strace(spath)
    writes = []
    for pid, tod, size, ret in raw:
        if size != 11 or ret != 11:
            continue
        cands = tod_candidates(tod, base_date)
        writes.append(min(cands, key=lambda c: abs(c - t_med)))
    writes.sort()

    print("=" * 72)
    print("链路延迟分析")
    print("=" * 72)
    print("参数: %s" % {k: cfg.get(k) for k in ("speed", "trials", "settle", "hold")})
    print("strace: %d 行 write/writev, 其中 11 B 且成功 = %d 次" % (len(raw), len(writes)))
    print("probe : %d 次发布, %d 个 /odom 样本" % (len(pubs), len(samples)))
    if chk:
        print("方向自检: vx=%+.3f m/s wz=%+.3f rad/s -> %s  (静止噪声 max %.4f)"
              % (chk.get("vx", 0), chk.get("wz", 0), chk.get("verdict"), chk.get("noise_max", 0)))
    else:
        print("方向自检: (无)")

    # ---------- 配对: 每次发布 → 对应那次 write ----------
    pairs = []
    j = 0
    for p in pubs:
        while j < len(writes) and writes[j] <= p["t"]:
            j += 1
        if j >= len(writes):
            pairs.append((p, None))
            continue
        dt = writes[j] - p["t"]
        if dt <= 0.200:
            pairs.append((p, writes[j]))
            j += 1
        else:
            pairs.append((p, None))

    t1_all = [w - p["t"] for p, w in pairs if w is not None and (w - p["t"]) >= 0]
    t1_step = [w - p["t"] for p, w in pairs
               if w is not None and "step" in p.get("tag", "") and (w - p["t"]) >= 0]

    def stats(v):
        if not v:
            return "无样本"
        v = sorted(v)
        return "n=%d  min=%.1f  p25=%.1f  p50=%.1f  p75=%.1f  p95=%.1f  max=%.1f  mean=%.1f ms" % (
            len(v), v[0] * 1e3, v[int(0.25 * (len(v) - 1))] * 1e3, statistics.median(v) * 1e3,
            v[int(0.75 * (len(v) - 1))] * 1e3, v[min(len(v) - 1, int(0.95 * len(v)))] * 1e3,
            v[-1] * 1e3, statistics.fmean(v) * 1e3)

    print()
    print("--- T1: /cmd_vel 发布 → 节点把 11 B 帧 write 到串口 ---")
    print("  全部: " + stats(t1_all))
    print("  阶跃: " + stats(t1_step))
    if t1_all:
        bins = [0] * 10
        for v in t1_all:
            b = min(9, int(v * 1e3 / 5))
            bins[b] += 1
        print("  5 ms 直方图 (0..50ms): " + " ".join("%d" % b for b in bins))
        lo = sum(1 for v in t1_all if 0 <= v <= 0.050)
        print("  落在 0~50 ms 内: %d/%d = %.0f%%  (厂商节点 Control() 阻塞大循环 + spin_some 的直接证据)"
              % (lo, len(t1_all), 100.0 * lo / len(t1_all)))

    # ---------- T2: write → odom 首次报动 ----------
    thresh = max(0.015, 2.0 * float(chk.get("noise_max", 0.0) or 0.0))
    step_pubs = [(p, w) for p, w in pairs if w is not None and "step" in p.get("tag", "")]
    res = []
    for p, w in step_pubs:
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
        # 平台值: onset 之后 0.15~0.35 s
        plat = [vwheel(s) for s in cand if ub + 0.15 <= s["st"] <= ub + 0.35]
        plateau = statistics.median(plat) if plat else 0.0
        # 上升沿两点外推: 假设这段速度线性上升, v1 = slope*(ub-onset)
        rise = None
        if v2 > v1 > 0 and v1 < 0.9 * plateau:
            dt = cand[k + 1]["st"] - ub
            if dt > 1e-6:
                slope = (v2 - v1) / dt
                if slope > 1e-6:
                    rise = ub - v1 / slope
        res.append({
            "i": p["tag"],
            "t_pub": p["t"],
            "t_write": w,
            "t1": w - p["t"],
            "lb": lb - w,
            "ub": ub - w,
            "rise": (rise - w) if rise is not None else None,
            "plateau": plateau,
            "cmd": p["vx"],
        })

    print()
    print("--- T2: 节点写出 11 B 帧 → /odom 首次报告轮子在转 (阈值 %.3f m/s) ---" % thresh)
    if not res:
        print("  !! 没有有效的 trial 样本 —— 检查车是否真动了 / 方向是否打转")
    else:
        lbs = [r["lb"] for r in res if r["lb"] >= 0]
        ubs = [r["ub"] for r in res if r["ub"] >= 0]
        rises = [r["rise"] for r in res if r["rise"] is not None and r["rise"] >= 0]
        print("  下界(上一帧): " + stats(lbs))
        print("  上界(该帧)  : " + stats(ubs))
        print("  上升沿外推  : " + stats(rises))
        print("  (下界/上界含'轮子转过门限后才报动'的偏置, 只能当区间证据;")
        print("   真值用上升沿外推: 假设 50 ms 内速度线性上升, 由 v1 与 v2 求过零点)")
        if rises:
            med = statistics.median(rises)
            lo = sorted(rises)[int(0.25 * (len(rises) - 1))]
            hi = sorted(rises)[int(0.75 * (len(rises) - 1))]
            print("  >> T2 估计 = %.1f ms  (p25~p75: %.1f ~ %.1f, n=%d)"
                  % (med * 1e3, lo * 1e3, hi * 1e3, len(rises)))
        pl = [r["plateau"] for r in res]
        cm = [abs(r["cmd"]) for r in res]
        print("  平台速度检查: 实测中位 %.3f m/s / 命令 %.3f m/s -> %s"
              % (statistics.median(pl), statistics.median(cm),
                 "OK" if statistics.median(cm) > 0 and
                 abs(statistics.median(pl) / statistics.median(cm) - 1) < 0.25 else "!! 偏差大, 可能没真正跑起来"))
        tm = [r["t1"] + r["ub"] for r in res]
        print()
        print("--- T_total: 发布 → odom 报告轮子转动 (含 20 Hz 量化, 上界) ---")
        print("  " + stats(tm))
        if rises and t1_step:
            print("  >> T_total 估计 = T1中位 %.1f + T2 %.1f = %.1f ms"
                  % (statistics.median(t1_step) * 1e3, statistics.median(rises) * 1e3,
                     (statistics.median(t1_step) + statistics.median(rises)) * 1e3))
        print()
        print("--- 逐 trial ---")
        print("  %-8s %8s %8s %8s %8s %7s" % ("tag", "T1/ms", "回码前", "回码后", "上升沿", "平台"))
        for r in res:
            print("  %-8s %8.1f %8.1f %8.1f %8s %7.3f"
                  % (r["i"], r["t1"] * 1e3, r["lb"] * 1e3, r["ub"] * 1e3,
                     ("%.1f" % (r["rise"] * 1e3)) if r["rise"] is not None else "  -",
                     r["plateau"]))

    # ---------- 观测链路自身延迟 ----------
    obs = [s["t"] - s["st"] for s in samples if s["t"] > s["st"] and s["t"] - s["st"] < 1.0]
    print()
    print("--- 观测链路: 串口帧到达本机 → 本进程收到 /odom (不影响上面 T2 的口径) ---")
    print("  " + stats(obs))

    print()
    print("=" * 72)
    print("读法:")
    print("  T1 = 主机侧(ROS 回调调度 + DDS + 串口写)")
    print("  T2 = 固件(20 ms 控制周期) + CAN + 达妙驱动器速度环 + 机械, 另加 odom 50 ms 量化")
    print("  真值区间: T2 在 [下界, 上界] 之间; N 次取最小逐渐逼近真值;")
    print("  若 T1 呈 0~50 ms 均匀分布 -> 证实厂商节点 spin_some 20 Hz 是主机侧主延迟")
    print("  (改多线程 executor / 单独线程收 cmd_vel 即可省掉均值 25 ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
