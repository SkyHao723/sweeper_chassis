#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""洗地车辅助按钮直连测试（不经 ROS / 不经 car_web）。

固件 v5 新增功能帧 f[1]=0x09：让 PB0/PB1 从"上拉输入"切成"推挽输出低"
（等于按一下物理按钮），保持 f[3]*10 ms（0=默认 1000 ms，上限 5000 ms）
后自动恢复上拉输入。功能帧**不改变速度目标**。

本脚本做的事：
  1. 打开底盘串口（默认 /dev/wheeltec_controller @115200）
  2. 先发 30 帧零速命令，确认车是停的（安全）
  3. 发一条功能帧（默认 mask=1 → PB0，保持 1000 ms）
  4. 之后只发零速命令 + 收帧，打印状态帧 [30]固件版本 / [39] 的
     bit3(正在按) bit4(最近按过 PB0) bit5(最近按过 PB1) 随时间的变化
  5. 判据：功能帧发出后 ≤200 ms 应看到 bit3=1；1100~1500 ms 后 bit3 应回 0

用法（在 RK3588 上，先停掉占串口的进程：car_web / wheeltec_robot_node）：

    python3 aux_btn_test.py                       # 按 PB0 1 s
    python3 aux_btn_test.py --mask 2              # 按 PB1
    python3 aux_btn_test.py --mask 3 --hold-ms 2000
    python3 aux_btn_test.py --port /dev/ttyUSB0 --cycles 3

注意：
  - 固件 < v5（扫地车或老版本）会把这个帧当普通命令帧吃掉，看不到 bit3 变化；
    脚本会打印 [30] 版本号，>=5 才是支持辅助按钮的固件。
  - 功能帧不改变速度；但为安全起见脚本全程只发零速命令。
"""

from __future__ import annotations

import argparse
import sys
import time

from ramp_test import FrameScanner, bcc8, build_cmd

CMD_SIZE = 11
CMD_HEAD = 0x7B
CMD_TAIL = 0x7D

FUNC_AUX_BTN = 0x09
AUX_NAMES = {0x01: "PB0", 0x02: "PB1", 0x03: "PB0+PB1"}
AUX_BTN_DEFAULT_MS = 1000
AUX_BTN_MAX_MS = 5000

BIT_ACTIVE = 0x08     # 正在按
BIT_PB0 = 0x10        # 最近按过 PB0
BIT_PB1 = 0x20        # 最近按过 PB1


def build_aux_btn_frame(mask: int, hold_ms: int = 0) -> bytes:
    """功能帧：f[1]=0x09, f[2]=掩码, f[3]=保持时间(单位 10 ms)。"""
    mask = int(mask) & 0x03
    if mask == 0:
        raise ValueError("mask 至少选一个按钮（1=PB0, 2=PB1, 3=两个）")
    hold = int(hold_ms) if hold_ms else AUX_BTN_DEFAULT_MS
    hold = max(10, min(int(hold), AUX_BTN_MAX_MS))
    hold_10ms = min(255, int(round(hold / 10.0)))

    f = bytearray(CMD_SIZE)
    f[0] = CMD_HEAD
    f[1] = FUNC_AUX_BTN
    f[2] = mask
    f[3] = hold_10ms
    f[9] = bcc8(f[0:9])
    f[10] = CMD_TAIL
    return bytes(f)


def bits_text(f39: int) -> str:
    return "bit3=%d(按下中) bit4=%d(PB0) bit5=%d(PB1)" % (
        1 if f39 & BIT_ACTIVE else 0,
        1 if f39 & BIT_PB0 else 0,
        1 if f39 & BIT_PB1 else 0,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="洗地车辅助按钮直连测试")
    ap.add_argument("--port", default="/dev/wheeltec_controller")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mask", type=int, default=1, help="1=PB0, 2=PB1, 3=两个")
    ap.add_argument("--hold-ms", type=int, default=0, help="0=固件默认 1000")
    ap.add_argument("--cycles", type=int, default=1, help="重复按几次")
    ap.add_argument("--settle", type=float, default=0.6, help="按下后观察多久（s）")
    args = ap.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        print("缺 pyserial：sudo apt install python3-serial  或 pip3 install pyserial",
              flush=True)
        return 2

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as exc:  # noqa: BLE001
        print("打开 %s 失败：%s" % (args.port, exc), flush=True)
        print("提示：先 pkill -f car_web.py / pkill -f wheeltec_robot_node 释放串口，"
              "并确认 /dev/wheeltec_controller 存在", flush=True)
        return 2

    scan = FrameScanner()
    t0 = time.time()
    last_ver = None
    rows = []          # (相对时间, f39, 原始帧时间戳)

    def pump(seconds: float, send_period: float = 0.02,
             extra: bytes | None = None) -> None:
        """发零速命令 + 收帧若干秒；extra 只在第一拍写出去。"""
        nonlocal last_ver
        end = time.time() + seconds
        next_tx = time.time()
        first = True
        while time.time() < end:
            now = time.time()
            if now >= next_tx:
                try:
                    ser.write(build_cmd(0, 0))
                    if first and extra is not None:
                        ser.write(extra)
                        first = False
                except Exception as exc:      # noqa: BLE001
                    print("写串口失败：%s" % exc, flush=True)
                    return
                next_tx = now + send_period
            try:
                chunk = ser.read(256)
            except Exception as exc:          # noqa: BLE001
                print("读串口失败：%s" % exc, flush=True)
                return
            if chunk:
                for kind, frame, ts in scan.feed(chunk):
                    if kind != "diag":
                        continue
                    if last_ver != frame[30]:
                        last_ver = frame[30]
                        print("固件版本 [30] = %d%s" % (
                            frame[30],
                            "" if frame[30] >= 5 else
                            "（<5，没有辅助按钮功能，看不到下面 bit3 的变化）"),
                            flush=True)
                    rows.append((ts - t0, frame[39], ts, bytes(frame)))

    print("端口 %s @ %d，mask=%d（%s），hold=%s ms"
          % (args.port, args.baud, args.mask & 0x03,
             AUX_NAMES.get(args.mask & 0x03, "?"),
             args.hold_ms or AUX_BTN_DEFAULT_MS), flush=True)

    # 1) 先静置，确认链路通、车是停的
    pump(1.0)
    if not rows:
        print("没收到 44 字节状态帧：检查接线/波特率/固件是否在跑", flush=True)
        ser.close()
        return 3
    print("链路正常：1 s 内收到 %d 帧状态帧，最新 %s"
          % (len(rows), bits_text(rows[-1][1])), flush=True)

    ok_all = True
    for i in range(max(1, args.cycles)):
        print("\n--- 第 %d/%d 次：按 %s ---"
              % (i + 1, max(1, args.cycles),
                 AUX_NAMES.get(args.mask & 0x03, "?")), flush=True)
        mark = len(rows)
        hold = args.hold_ms or AUX_BTN_DEFAULT_MS
        pump(0.4, extra=build_aux_btn_frame(args.mask, args.hold_ms))
        pump(max(0.8, hold / 1000.0 + 0.4))

        seg = rows[mark:]
        if not seg:
            print("这段时间没收到状态帧", flush=True)
            ok_all = False
            continue

        # 只看功能帧发出之后的样本；按下应在 ≤200 ms 内出现
        first_active = None
        last_active = None
        for rel, f39, _ts, _raw in seg:
            if f39 & BIT_ACTIVE:
                if first_active is None:
                    first_active = rel
                last_active = rel
        on_delay = None if first_active is None else (first_active - seg[0][0]) * 1000.0
        on_len = None if first_active is None else (last_active - first_active) * 1000.0

        print("样本 %d 个；首次 bit3=1 延迟 %s ms；bit3 持续 %s ms（期望 ≈%d ms）"
              % (len(seg),
                 "—" if on_delay is None else "%.0f" % on_delay,
                 "—" if on_len is None else "%.0f" % on_len, hold), flush=True)
        print("最后一次回读：%s" % bits_text(seg[-1][1]), flush=True)
        for rel, f39, _ts, _raw in seg[:6]:
            print("  t=%+6.0f ms  f39=0x%02X  %s"
                  % (rel * 1000.0, f39, bits_text(f39)), flush=True)

        ok = (first_active is not None
              and on_delay is not None and on_delay <= 250.0
              and on_len is not None and abs(on_len - hold) <= 400.0
              and not (seg[-1][1] & BIT_ACTIVE))
        ok_all = ok_all and ok
        print("判据：%s" % ("PASS" if ok else "FAIL"), flush=True)

    ser.close()
    print("\n结论：%s"
          % ("PASS —— 功能帧生效，按下时长与恢复都符合预期" if ok_all
             else "FAIL —— 见上面每组数字（固件 <v5、引脚没接、或帧格式不对）"),
          flush=True)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
