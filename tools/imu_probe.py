#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静态探测：诊断帧里的 IMU 状态位 + 24B 遥测帧里的六轴原始值。

用途：搞清楚"24B 遥测帧的 accel/gyro 全是 0"到底是 IMU 没工作，还是打包 bug。
不动车（只发 0 速度），读 3 s 就退出。

用法（在主机上）：
    python3 /tmp/imu_probe.py --port /dev/wheeltec_controller --secs 3
"""
import argparse
import sys
import time

import serial

import ramp_test as rt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/wheeltec_controller")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--secs", type=float, default=3.0)
    args = ap.parse_args()

    sp = serial.Serial(args.port, args.baud, timeout=0)
    sc = rt.FrameScanner()
    diags, tels = [], []
    t_end = time.time() + args.secs
    last_tx = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - last_tx >= 0.020:
            sp.write(rt.build_cmd(0, 0))
            last_tx = now
        for kind, frame, _ts in sc.feed(sp.read(4096)):
            if kind == "diag":
                diags.append(frame)
            elif kind == "tel":
                tels.append(frame)
        time.sleep(0.002)
    sp.close()

    print("诊断帧 %d 个，遥测帧 %d 个" % (len(diags), len(tels)))
    if diags:
        print("\n诊断帧里的 IMU 字段（应恒定）:")
        for off, name in ((31, "IMU status"), (38, "IMU addr"),
                          (39, "IMU probe"), (40, "IMU version")):
            vals = sorted({f[off] for f in diags})
            print("  [%2d] %-12s = %s" % (off, name, vals))
        print("  [30] 版本      = %s" % sorted({f[30] for f in diags}))
        print("  [41] 阶段/心跳 = %s" % sorted({f[41] for f in diags}))
        print("  [1]  flags     = %s" % sorted({f[1] for f in diags}))
    if tels:
        print("\n24B 遥测帧字段（raw int16 BE）:")
        names = ["vx mm/s", "vy mm/s", "wz mrad/s",
                 "ax", "ay", "az", "gx", "gy", "gz", "bat mV"]
        offs = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
        for name, off in zip(names, offs):
            vals = [rt.i16be(f, off) for f in tels]
            lo, hi = min(vals), max(vals)
            print("  %-10s = %+7d .. %+7d" % (name, lo, hi))
    return 0


if __name__ == "__main__":
    sys.exit(main())
