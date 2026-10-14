#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位 ESP32 再读串口(Windows 版)。
★ 为什么要复位: 探活固件开机打印一次就把话说完, 之后"只听不说话";
  只有复位才能重新看到它的启动横幅和那 10 秒监听结果。
   ESP32 自动复位(经 CH340): RTS->EN, DTR->GPIO0
   正常启动: DTR=False(IO0 保持高) + RTS 脉冲(EN 拉低再放开)
"""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM11"
DUR  = float(sys.argv[2]) if len(sys.argv) > 2 else 16.0
BAUD = 115200

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.2
ser.dtr = False
ser.rts = False
try:
    ser.open()
except Exception as e:
    print("★ 打不开 %s: %s" % (PORT, e)); sys.exit(1)

# 正常启动: IO0 高, EN 脉冲
ser.dtr = False
ser.rts = True
time.sleep(0.15)
ser.rts = False
time.sleep(0.05)

print("# 已复位 %s, 采集 %.0f 秒 …" % (PORT, DUR))
t0 = time.time()
buf = b""
while time.time() - t0 < DUR:
    try:
        c = ser.read(4096)
    except Exception as e:
        print("★ 读失败: %s" % e); break
    if c:
        buf += c
        sys.stdout.write(c.decode("utf-8", "replace").replace("\r", ""))
        sys.stdout.flush()
ser.close()

print("\n# 共 %d 字节" % len(buf))
if len(buf) == 0:
    print("# ★ 还是 0 字节。可能:")
    print("#   1) 这个口不是 ESP32(而是 STM32 或别的)")
    print("#   2) ESP32 没在跑固件 / 没上电")
    print("#   3) 板子的 CH340 和 ESP32 之间没有 EN/IO0 自动复位电路(有些板没有),")
    print("#      那就手动按一下板上的 RST 键, 我再读")
else:
    open("probe_uart.txt", "wb").write(buf)
    print("# 已存到 probe_uart.txt")
