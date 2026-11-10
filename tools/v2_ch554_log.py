#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v2_ch554_log.py —— 看 CH554G 的调试日志（UART1，P1.7=TXD1，115200 8N1）

背景
----
CH554 固件（firmware/ch554-gamepad）有两条串口：
  * 数据口 UART0 P3.1 -> STM32 PB11（发给 STM32 的 15 字节手柄报告）
  * 调试口 UART1 P1.7 -> USB-TTL 的 RXD（本工具看的就是这条）

这条调试口和 **CH554 的 ISP 下载口是同一组脚**（P1.6/P1.7/GND），所以：
烧完 CH554 不用拔线，直接用本工具读日志最省事。

用法
----
    python tools/v2_ch554_log.py --port COM19
    python tools/v2_ch554_log.py --port COM19 --seconds 10
    python tools/v2_ch554_log.py --list          # 列出串口

能看到的日志（固件 v2 起）
--------------------------
    [ch554] gamepad host v2 (uart0 p3.1/p3.0 115200)   上电横幅
    [usb] connect                                      检测到设备插入
    [usb] enum=<s>                                     枚举结果（0=成功）
    [usb] status=<n> type=<hh> vid=<hhhh> pid=<hhhh>   设备状态/类/VID/PID
    [pad] found, loc=<n> endp=<hh>                     找到中断 IN 端点
    [pad] first report                                 第一次收到 15 字节报告
    [pad] len=<n> head=<hhhhhhhh>                      收到的包长度不是 15
    [pad] lost, err=<hh>                               设备掉了
    [stat] pad=<0|1> pkt=<n> sent=<n> len=<n>          每 500 ms 一行统计
"""
import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    print("需要 pyserial：python -m pip install pyserial")
    raise SystemExit(2)


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到串口")
        return
    for p in ports:
        print("%-8s %s  [%s]" % (p.device, p.description, p.hwid))


def main(argv=None):
    ap = argparse.ArgumentParser(description="CH554G 调试日志查看器（UART1, 115200）")
    ap.add_argument("--port", help="串口号，如 COM19")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0.0, help="读多少秒后退出（0=一直读）")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    args = ap.parse_args(argv)

    if args.list or not args.port:
        list_serial_ports()
        if not args.port:
            return 0

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:  # noqa: BLE001
        print("打开 %s 失败：%s" % (args.port, exc))
        return 1

    print("监听 %s @%d（Ctrl-C 退出）" % (args.port, args.baud))
    buf = bytearray()
    t0 = time.time()
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    print(text)
            if args.seconds and (time.time() - t0) >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
