#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动找 STM32 的串口并读一段: 遍历所有非虚拟 COM 口, 看哪个吐诊断帧(0x7E/0x7B)。"""
import serial, serial.tools.list_ports, time, sys

SKIP = {"COM3", "COM4"}      # 蓝牙 / ELTIMA 虚拟口

ports = [p.device for p in serial.tools.list_ports.comports()]
print("# 本机串口: %s" % (", ".join(ports) if ports else "(无)"))

def try_port(name, seconds=2.5):
    try:
        s = serial.Serial()
        s.port = name; s.baudrate = 115200; s.timeout = 0.2
        s.dtr = False; s.rts = False
        s.open()
    except Exception as e:
        print("  %-8s 打不开: %s" % (name, e))
        return None
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        try: buf += s.read(4096)
        except Exception: break
    s.close()
    n7e = buf.count(0x7E); n7b = buf.count(0x7B)
    return buf, n7e, n7b

found = None
for name in ports:
    if name.upper() in SKIP:
        continue
    r = try_port(name)
    if not r:
        continue
    buf, n7e, n7b = r
    print("  %-8s 收到 %5d 字节 | 0x7E x%3d  0x7B x%3d" % (name, len(buf), n7e, n7b))
    if n7e > 2 or n7b > 2:
        found = name
        open("stm_dump.bin", "wb").write(buf)
        print("     -> ★ 这是 STM32! 原始数据已存 stm_dump.bin")
        break

if not found:
    print("\n# 没找到 STM32 的串口。检查: 串口线插了吗 / 驱动 / 换一个口")
    sys.exit(1)

print("\n# 用 %s 再读 4 秒, 打印解出来的帧 …" % found)
s = serial.Serial()
s.port = found; s.baudrate = 115200; s.timeout = 0.2
s.dtr = False; s.rts = False
s.open()
buf = b""; t0 = time.time()
while time.time() - t0 < 4.0:
    buf += s.read(4096)
s.close()
open("stm_dump.bin", "wb").write(buf)
print("# 存了 %d 字节到 stm_dump.bin" % len(buf))
print("# 端口名 = %s" % found)
