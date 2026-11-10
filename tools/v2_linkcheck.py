"""V2 调试口通断 + 协议解析自测（USART1 中断接收版）。

读 STM32 RAM 里的计数器判断：
  - PC->MCU：发 N 条帧后 s_host_cmds / s_func_frames 是否增加、g_mode 是否改变
  - RX 有没有丢：s_rx_overrun（正常 0）、环形缓冲 pending
  - MCU->PC：监听串口能收到多少字节

用法：
  python tools/v2_linkcheck.py --port COM18
"""
import argparse
import subprocess
import time

import serial

OPENOCD = r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe"

# 符号地址（firmware/stm32-chassis-v2/Listings/chassis-v2/chassis-v2.map）
SYM = {
    "rx_head": 0x20000014,
    "rx_tail": 0x20000016,
    "rx_overrun": 0x20000018,
    "g_mode": 0x2000001A,
    "host_vx": 0x20000020,
    "host_wz": 0x20000022,
    "host_cmds": 0x20000024,
    "bad_frames": 0x20000028,
    "func_frames": 0x2000002A,
    "sent": 0x2000002E,
}
BASE = 0x20000014
WORDS = 8


def bcc(data):
    v = 0
    for x in data:
        v ^= x
    return v


def frame_speed(vx_mm_s=100, wz_mrad_s=0):
    body = bytes([0x7B, 0x00, 0x00,
                  (vx_mm_s >> 8) & 0xFF, vx_mm_s & 0xFF,
                  0x00, 0x00,
                  (wz_mrad_s >> 8) & 0xFF, wz_mrad_s & 0xFF])
    return body + bytes([bcc(body), 0x7D])


def frame_set_mode(mode):
    body = bytes([0x7A, 0x02, mode & 0xFF, 0, 0, 0])
    return body + bytes([bcc(body), 0x7D])


def read_ram():
    args = [
        OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f1x.cfg",
        "-c", "adapter speed 500", "-c", "init", "-c", "halt",
        "-c", "mdw 0x%08X %d" % (BASE, WORDS), "-c", "resume", "-c", "shutdown",
    ]
    out = subprocess.run(args, capture_output=True, text=True, timeout=45)
    txt = (out.stdout or "") + (out.stderr or "")
    for line in txt.splitlines():
        s = line.strip()
        if s.startswith("0x%08X:" % BASE):
            # openocd 打印的是 "0x20000014: 00000000 00000000 ..."（字面量不带 0x 前缀）
            toks = [t for t in s.split(":", 1)[1].split() if t]
            words = []
            for t in toks:
                try:
                    words.append(int(t, 16))
                except ValueError:
                    break
            if len(words) < WORDS:
                continue
            raw = b"".join(w.to_bytes(4, "little") for w in words)
            return {
                "rx_head": int.from_bytes(raw[0x00:0x02], "little"),
                "rx_tail": int.from_bytes(raw[0x02:0x04], "little"),
                "rx_overrun": int.from_bytes(raw[0x04:0x06], "little"),
                "g_mode": raw[0x06],
                "host_vx": int.from_bytes(raw[0x0C:0x0E], "little", signed=True),
                "host_wz": int.from_bytes(raw[0x0E:0x10], "little", signed=True),
                "host_cmds": int.from_bytes(raw[0x10:0x12], "little"),
                "bad_frames": int.from_bytes(raw[0x14:0x16], "little"),
                "func_frames": int.from_bytes(raw[0x16:0x18], "little"),
                "sent": int.from_bytes(raw[0x1A:0x1C], "little"),
            }
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM18")
    ap.add_argument("--count", type=int, default=100)
    args = ap.parse_args()

    a = read_ram()
    print("A) 发之前:", a)

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    t0 = time.time()
    total = ser.in_waiting
    data = b""
    while time.time() - t0 < 1.0:
        chunk = ser.read(256)
        if chunk:
            data += chunk
    print("B) 监听 1s: %d 字节 (MCU->PC)%s" % (len(data), "  样例 " + data[:16].hex(" ") if data else ""))

    for _ in range(args.count):
        ser.write(frame_speed(100))
        time.sleep(0.004)
    for _ in range(args.count):
        ser.write(frame_set_mode(2))
        time.sleep(0.004)
    ser.close()
    time.sleep(0.3)

    b = read_ram()
    print("C) 发之后:", b)

    if a and b:
        print("增量: host_cmds +%d, func_frames +%d, bad_frames +%d, rx_overrun +%d, sent +%d"
              % (b["host_cmds"] - a["host_cmds"], b["func_frames"] - a["func_frames"],
                 b["bad_frames"] - a["bad_frames"], b["rx_overrun"] - a["rx_overrun"],
                 b["sent"] - a["sent"]))
        print("g_mode: %d -> %d" % (a["g_mode"], b["g_mode"]))
        print("host_cmd 解析值: vx=%d wz=%d" % (b["host_vx"], b["host_wz"]))
        ok = (b["host_cmds"] - a["host_cmds"]) >= args.count
        print("=> PC->MCU %s" % ("OK（全部帧都收到了）" if ok else "不正常（有丢帧）"))
    if not data:
        print("=> MCU->PC 仍然一个字节都收不到（TX 线/CH343 RXD 侧硬件问题）")


if __name__ == "__main__":
    main()
