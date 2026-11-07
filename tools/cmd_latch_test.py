#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令锁存测试（固件 v4）：发一条速度，然后**彻底不发**，看车还走不走。

为什么需要这个工具
------------------
固件 v4 把命令看门狗关掉了（`CMD_TIMEOUT_ENABLE = 0`）：没收到新命令就一直
执行最后一条命令。这个行为没法用"眼睛看"确认——车在走可能只是斜坡还没走完，
也可能是真的锁存了。所以用诊断帧里的 **目标 ω** 和 **实际 ω** 直接判：

  * 目标 ω 在"停发命令"之后**没有回零** → 锁存生效（这是关键证据，
    比看车动不动更硬：目标回零了但车还在滑行也骗不过去）；
  * 实际 ω 在停发 2 s 后仍然 > 50% 命令值 → 真的还在执行。

用法
----
    # 直连串口（不需要 ROS，推荐：排除厂商节点的干扰）
    python3 cmd_latch_test.py --port /dev/wheeltec_controller

    # 将来把看门狗改回 CMD_TIMEOUT_ENABLE=1 时，期望"松手就停"
    python3 cmd_latch_test.py --expect stop

判定
----
    keep（默认）: 停发后 [1.8, 2.4] s 的 |目标 ω| > 50% 命令值 且
                  |实际 ω| 均值 > 50% 命令值          -> PASS
    stop        : 停发后 [1.8, 2.4] s 的 |目标 ω| == 0 且
                  |实际 ω| 均值 < 25% 命令值          -> PASS

固件身份也会打出来：诊断帧 `[30]` 版本号、`[41]` bit6 = 命令锁存标志。
如果 `[41].bit6 = 0` 但 `[30] = 4`，说明宏被改回去了，本工具的 keep 判据会
FAIL —— 这正是期望的（工具不替固件猜）。
"""

import argparse
import os
import sys
import time

import serial

# Windows 下 pwsh 用 GBK 解 stdout，中文/° 会炸；强制 UTF-8（坏字节替换掉）
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ramp_test import FrameScanner, build_cmd, i16be  # noqa: E402

WHEEL_R_M = 0.0855


def counts_from_mm_s(speed_mm):
    """mm/s -> 诊断帧里的 ω 计数（rad/s x100）。"""
    return abs(speed_mm) / 1000.0 / WHEEL_R_M * 100.0


def window_mean(samples, t0, t1, idx):
    vals = [s[idx] for s in samples if t0 <= s[0] <= t1]
    if not vals:
        return None, 0
    return sum(vals) / float(len(vals)), len(vals)


def window_abs_mean(samples, t0, t1, idx):
    vals = [abs(s[idx]) for s in samples if t0 <= s[0] <= t1]
    if not vals:
        return None, 0
    return sum(vals) / float(len(vals)), len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/wheeltec_controller")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed-mm", type=int, default=200,
                    help="测试速度 mm/s（默认 200 = 0.2 m/s）")
    ap.add_argument("--hold", type=float, default=1.0,
                    help="按住/连续发命令的秒数")
    ap.add_argument("--silence", type=float, default=2.6,
                    help="彻底不发命令的秒数（要覆盖 800 ms 看门狗 + 2 s 反馈超时）")
    ap.add_argument("--expect", choices=["keep", "stop"], default="keep",
                    help="keep=没命令也继续跑（v4 默认）; stop=超时自动软停（老的看门狗行为）")
    args = ap.parse_args()

    sp = serial.Serial(args.port, args.baud, timeout=0)
    scanner = FrameScanner()
    samples = []          # [t, tgtL, tgtR, actL, actR, flags, ver, f41]
    t00 = time.time()
    last_tx = 0.0
    tx_count = 0
    phase = "pre"
    phase_end = t00 + 1.5

    def pump(now, send_mm):
        """读串口；send_mm 为 None 表示这一拍**不发**。"""
        nonlocal last_tx, tx_count
        data = sp.read(4096)
        for kind, frame, ts in scanner.feed(data):
            if kind == "diag":
                samples.append([
                    ts - t00,
                    i16be(frame, 10), i16be(frame, 12),   # 目标 ω x100 L/R
                    i16be(frame, 6), i16be(frame, 8),     # 实际 ω x100 L/R
                    frame[1],
                    frame[30],
                    frame[41],
                ])
        if send_mm is not None and now - last_tx >= 0.020:
            sp.write(build_cmd(send_mm, 0))
            last_tx = now
            tx_count += 1

    # ---- 预检：发 0，确认链路和固件还活着 ----
    while time.time() < phase_end:
        pump(time.time(), 0)
        time.sleep(0.002)
    if not samples:
        sys.stderr.write("1.5 s 内没收到任何 0x7E 诊断帧，检查串口/固件\n")
        sp.close()
        return 2

    ver = samples[-1][6]
    f41 = samples[-1][7]
    latch_bit = bool(f41 & 0x40)
    print("固件版本 [30] = %d,  [41] = 0x%02X,  命令锁存 bit6 = %s"
          % (ver, f41, "1 (锁存)" if latch_bit else "0 (有看门狗)"))
    if args.expect == "keep" and not latch_bit:
        print("!! 固件报的是'有看门狗'，与本测试期望的 keep 不一致 —— 继续跑，"
              "但预计 FAIL（说明车上不是 v4 固件）")

    # ---- 阶段 1：连续发目标，让车走起来 ----
    phase = "hold"
    t_hold0 = time.time()
    t_last_send = t_hold0
    while time.time() < t_hold0 + args.hold:
        now = time.time()
        pump(now, args.speed_mm)
        t_last_send = now
        time.sleep(0.002)

    # ---- 阶段 2：彻底不发（关键） ----
    phase = "silence"
    t_sil0 = time.time()
    while time.time() < t_sil0 + args.silence:
        pump(time.time(), None)
        time.sleep(0.002)

    # ---- 阶段 3：显式发 0，确认软停这条路是通的 ----
    phase = "stop"
    t_stop0 = time.time()
    while time.time() < t_stop0 + 1.4:
        pump(time.time(), 0)
        time.sleep(0.002)
    sp.close()

    # ---- 分析（时间都相对"最后一次发命令"）----
    t_rel = lambda s: s[0] - (t_last_send - t00)   # noqa: E731

    expect = counts_from_mm_s(args.speed_mm)
    hold_tgt, n1 = window_abs_mean(samples, -args.hold + 0.15, 0.0, 1)
    early_tgt, n2 = window_abs_mean(samples, 0.35, 0.75, 1)
    late_tgt, n3 = window_abs_mean(samples, 1.8, 2.4, 1)
    late_act, n4 = window_abs_mean(samples, 1.8, 2.4, 3)
    stop_tgt, n5 = window_abs_mean(samples, args.silence + 0.6, args.silence + 1.2, 1)
    stop_act, n6 = window_abs_mean(samples, args.silence + 0.6, args.silence + 1.2, 3)

    def fmt(v, n):
        return "    -   " if v is None else "%7.1f (%d帧)" % (v, n)

    print("")
    print("命令速度 = %.0f mm/s -> 期望 ω = %.1f 计数(rad/s x100)"
          % (args.speed_mm, expect))
    print("发送帧数 = %d（按住 %.1f s @50Hz）" % (tx_count, args.hold))
    print("")
    print("                     目标 ω        实际 ω")
    print("  按住末段         %s  %s" % (fmt(hold_tgt, n1), fmt(-1, 0)))
    print("  停发 0.35~0.75s  %s  %s" % (fmt(early_tgt, n2), fmt(-1, 0)))
    print("  停发 1.80~2.40s  %s  %s" % (fmt(late_tgt, n3), fmt(late_act, n4)))
    print("  显式发 0 之后    %s  %s" % (fmt(stop_tgt, n5), fmt(stop_act, n6)))
    print("")

    ok = True
    if args.expect == "keep":
        if late_tgt is None:
            print("[FAIL] 停发后没采到诊断帧")
            ok = False
        elif late_tgt < 0.5 * expect:
            print("[FAIL] 停发 1.8~2.4 s 后目标 ω = %.1f，只有命令值的 %.0f%% "
                  "-> 看门狗还在（不是 v4 固件，或 CMD_TIMEOUT_ENABLE 被改成 1）"
                  % (late_tgt, 100.0 * late_tgt / expect))
            ok = False
        elif (late_act or 0.0) < 0.5 * expect:
            print("[FAIL] 目标锁存住了（%.1f）但实际 ω 只有 %.1f（%.0f%%）"
                  "-> 车没在走，检查使能/ERR/是否被上层清零"
                  % (late_tgt, late_act or 0.0,
                     100.0 * (late_act or 0.0) / expect))
            ok = False
        else:
            print("[PASS] 停发 %.1f s 后目标 ω 仍是 %.0f%% 命令值、实际 ω %.0f%% "
                  "-> 命令锁存生效，没命令也一直按最后一条跑"
                  % (args.silence, 100.0 * late_tgt / expect,
                     100.0 * late_act / expect))
    else:  # stop
        if late_tgt is None:
            print("[FAIL] 停发后没采到诊断帧")
            ok = False
        elif late_tgt > 0.1 * expect:
            print("[FAIL] 停发后目标 ω 仍是 %.1f（期望回到 0）-> 看门狗没生效"
                  % late_tgt)
            ok = False
        elif (late_act or 0.0) > 0.25 * expect:
            print("[FAIL] 目标已回零但实际 ω 仍有 %.1f（%.0f%%）-> 没停住"
                  % (late_act or 0.0, 100.0 * (late_act or 0.0) / expect))
            ok = False
        else:
            print("[PASS] 停发后目标回零、实际 ω 掉到 %.0f%% -> 看门狗软停生效"
                  % (100.0 * (late_act or 0.0) / expect))

    # 附带：显式发 0 必须能停下来（这条与锁存无关，任何版本都该过）
    if stop_act is not None and stop_act > 0.25 * expect:
        print("[WARN] 显式发 0 之后实际 ω 仍有 %.1f -> '发 0 就能停'这条不成立，"
              "先别把车放到地上跑" % stop_act)
        ok = False
    elif stop_act is not None:
        print("[ok] 显式发 0 之后实际 ω = %.1f（目标 %.1f）-> 显式停车通路正常"
              % (stop_act, stop_tgt or 0.0))

    print("")
    print("提示：车真的会一直跑。要停就显式发 0（键盘按 k / 网页松手 / "
          "cmd_vel 发 0），或者断电机电源。")
    print("结论: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
