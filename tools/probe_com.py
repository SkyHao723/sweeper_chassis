#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探测 COM11 是什么设备。★ 打开前把 DTR/RTS 置 False, 免得误复位 ESP32。"""
import serial, time, sys

PORT = "COM11"
DUR  = 4.0

ser = serial.Serial()
ser.port = PORT
ser.baudrate = 115200
ser.timeout = 0.2
ser.dtr = False
ser.rts = False
try:
    ser.open()
except Exception as e:
    print("★ 打不开 %s: %s" % (PORT, e))
    sys.exit(1)

print("# 已打开 %s @115200, 采集 %.0f 秒 …" % (PORT, DUR))
t0 = time.time()
buf = b""
while time.time() - t0 < DUR:
    try:
        c = ser.read(4096)
    except Exception as e:
        print("★ 读失败: %s" % e); break
    if c:
        buf += c
ser.close()

print("# 收到 %d 字节" % len(buf))
if not buf:
    print("# ★ 一个字节都没有 —— 可能: 设备没发数据 / 波特率不对 / 线只接了 TX 没接 RX")
    sys.exit(0)

print("\n# 前 64 字节(hex):")
print("  " + buf[:64].hex(" "))

print("\n# 可打印文本(前 600 字符):")
txt = "".join(chr(b) if 32 <= b < 127 or b in (10, 13) else "." for b in buf[:600])
print(txt)

# 判断是哪一种
n7e = buf.count(0x7E); n7b = buf.count(0x7B); n7d = buf.count(0x7D)
print("\n# 判据:")
print("  0x7E 出现 %d 次 / 0x7B %d 次 / 0x7D %d 次" % (n7e, n7b, n7d))
if b"ESP-ROM" in buf or b"ESP32" in buf or b"====" in buf:
    print("  -> **这是 ESP32**(看到启动屏/我的程序输出)")
elif n7e > 3 or b"seq=" in buf:
    print("  -> **这是 STM32**(诊断帧 0x7E / 或 decode 后的 seq=)")
    open("/tmp_win_probe.bin", "wb").write(buf)
else:
    print("  -> 认不出, 可能是别的设备或波特率不对")

open("probe_com11.bin", "wb").write(buf)
print("\n# 原始数据已存到 probe_com11.bin (共 %d 字节)" % len(buf))
