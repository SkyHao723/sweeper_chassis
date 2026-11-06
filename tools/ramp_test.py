#!/usr/bin/env python3
"""斜坡（缓起缓停）实测脚本 —— 直连底盘串口，不经过 ROS。

用途
----
固件 v3 在 `Control_Tick()` 里、差速逆解之前对 vx/wz 做线性斜坡
（RAMP_VX_ACC=1.0 / RAMP_VX_DEC=1.5 m/s²）。本脚本直接发 11 字节命令帧，
并解析 44 字节诊断帧里的 **目标 ω**（[10..13]）与 **实际 ω**（[6..9]），
用 20 Hz 的采样去验证：
  1. 目标 ω 是否按斜坡爬升（0→0.2 m/s 应该 200 ms 到顶，每拍 +23.4）
  2. 目标 ω 是否按减速斜坡回零（0.2→0 应该 140 ms）
  3. 实际 ω 是否跟随（含驱动器自身的延迟/过冲）

在 192.168.5.7 上跑，前提是 `wheeltec_robot_node` / `car_web` 都没占用
`/dev/wheeltec_controller`。

帧格式（与固件一致）
-------------------
命令 11 B：0x7B | f1=0 | f2=0 | vx(int16 BE, mm/s) | vy | wz(int16 BE, mrad/s) | BCC | 0x7D
诊断 44 B：0x7E | ... | [42]=BCC | [43]=0x7D
遥测 24 B：0x7B | ... | [22]=BCC | [23]=0x7D
"""

import argparse
import json
import struct
import sys
import time

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    sys.stderr.write("缺少 pyserial：pip3 install pyserial\n")
    raise

CMD_SIZE = 11
CMD_HEAD = 0x7B
CMD_TAIL = 0x7D
DIAG_HEAD = 0x7E
DIAG_SIZE = 44
TEL_HEAD = 0x7B
TEL_SIZE = 24


def bcc8(buf):
    value = 0
    for byte in buf:
        value ^= byte
    return value


def build_cmd(vx_mm, wz_mrad):
    f = bytearray(CMD_SIZE)
    f[0] = CMD_HEAD
    f[1] = 0            # 普通速度帧
    f[2] = 0
    f[3] = (vx_mm >> 8) & 0xFF
    f[4] = vx_mm & 0xFF
    f[5] = 0
    f[6] = 0
    f[7] = (wz_mrad >> 8) & 0xFF
    f[8] = wz_mrad & 0xFF
    f[9] = bcc8(f[0:9])
    f[10] = CMD_TAIL
    return bytes(f)


def i16be(buf, off):
    return struct.unpack_from(">h", buf, off)[0]


class FrameScanner:
    """同时识别 44 B 诊断帧与 24 B 遥测帧；靠 帧头+帧尾+BCC 三重校验。"""

    def __init__(self):
        self.buf = bytearray()
        self.diag = []
        self.tel = []

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while True:
            moved = False
            for i in range(len(self.buf)):
                head = self.buf[i]
                if head == DIAG_HEAD and i + DIAG_SIZE <= len(self.buf):
                    frame = bytes(self.buf[i:i + DIAG_SIZE])
                    if frame[43] == CMD_TAIL and bcc8(frame[0:42]) == frame[42]:
                        out.append(("diag", frame, time.time()))
                        del self.buf[: i + DIAG_SIZE]
                        moved = True
                        break
                elif head == TEL_HEAD and i + TEL_SIZE <= len(self.buf):
                    frame = bytes(self.buf[i:i + TEL_SIZE])
                    if frame[23] == CMD_TAIL and bcc8(frame[0:22]) == frame[22]:
                        out.append(("tel", frame, time.time()))
                        del self.buf[: i + TEL_SIZE]
                        moved = True
                        break
            if not moved:
                # 丢掉不可能再组成帧的前缀，避免缓冲无限增长
                if len(self.buf) > 4096:
                    del self.buf[: len(self.buf) - 128]
                break
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/wheeltec_controller")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed-mm", type=int, default=200,
                    help="阶跃幅值 mm/s（默认 200 = 0.2 m/s）")
    ap.add_argument("--cycles", type=int, default=3, help="±交替的循环次数")
    ap.add_argument("--hold", type=float, default=1.6, help="每个阶段保持秒数")
    ap.add_argument("--out", default="/tmp/ramp_test.json")
    args = ap.parse_args()

    phases = []
    for k in range(args.cycles):
        phases.append(("up%d" % k, args.speed_mm, args.hold))
        phases.append(("stop%d" % k, 0, args.hold))
        phases.append(("down%d" % k, -args.speed_mm, args.hold))
        phases.append(("stop%d" % k, 0, args.hold))

    sp = serial.Serial(args.port, args.baud, timeout=0)
    scanner = FrameScanner()
    samples = []
    tel = []
    t00 = time.time()
    last_tx = 0.0
    tx_count = 0
    cur_phase = -1
    phase_t0 = 0.0
    pending = []            # (phase_idx_raw, vx_mm) 待发
    script = [("pre", 0, 0.60)] + phases + [("post", 0, 0.60)]
    phase_log = []

    def pump(now):
        """读串口 + 按 50 Hz 发当前阶段命令。"""
        nonlocal last_tx, tx_count, cur_phase, phase_t0
        data = sp.read(4096)
        for kind, frame, ts in scanner.feed(data):
            if kind == "diag":
                samples.append([
                    ts - t00,
                    0 if cur_phase < 0 else cur_phase,
                    i16be(frame, 10), i16be(frame, 12),   # 目标 ω x100 L/R
                    i16be(frame, 6), i16be(frame, 8),     # 实际 ω x100 L/R
                    frame[1],
                    frame[30],
                    frame[41],
                ])
            elif kind == "tel":
                # 24 B 遥测：vx(mm/s) + accel/gyro 六轴。accel 是**独立于轮速**
                # 的物理量，用来交叉验证"轮速振荡到底是真抖动还是反馈噪声"。
                tel.append([
                    ts - t00,
                    i16be(frame, 2),      # vx mm/s
                    i16be(frame, 8),      # ax raw
                    i16be(frame, 10),     # ay raw
                    i16be(frame, 12),     # az raw
                    i16be(frame, 14),     # gx raw
                    i16be(frame, 16),     # gy raw
                    i16be(frame, 18),     # gz raw
                ])
        if cur_phase >= 0 and cur_phase < len(script):
            _, vx_mm_now, _ = script[cur_phase]
            if now - last_tx >= 0.020:
                sp.write(build_cmd(vx_mm_now, 0))
                last_tx = now
                tx_count += 1

    # ---- 预检：先发 0，等第一帧诊断帧，确认链路活着 ----
    t_end = time.time() + 1.5
    while time.time() < t_end:
        pump(time.time())
        time.sleep(0.002)
    if not samples:
        sys.stderr.write("1.5 s 内没收到任何 0x7E 诊断帧，检查串口/固件\n")
        sp.close()
        return 2
    sys.stderr.write("链路正常：已收到 %d 帧，开始阶跃...\n" % len(samples))

    # ---- 正式流程 ----
    for idx, (name, vx_mm, hold) in enumerate(script):
        cur_phase = idx
        phase_t0 = time.time()
        phase_log.append([name, vx_mm, hold, phase_t0 - t00])
        while time.time() - phase_t0 < hold:
            pump(time.time())
            time.sleep(0.002)

    cur_phase = len(script)
    # 收尾：连发 0.5 s 的 0
    t_end = time.time() + 0.5
    while time.time() < t_end:
        sp.write(build_cmd(0, 0))
        data = sp.read(4096)
        for kind, frame, ts in scanner.feed(data):
            if kind == "diag":
                samples.append([ts - t00, len(script),
                                i16be(frame, 10), i16be(frame, 12),
                                i16be(frame, 6), i16be(frame, 8),
                                frame[1], frame[30], frame[41]])
            elif kind == "tel":
                tel.append([ts - t00, i16be(frame, 2),
                            i16be(frame, 8), i16be(frame, 10), i16be(frame, 12),
                            i16be(frame, 14), i16be(frame, 16), i16be(frame, 18)])
        time.sleep(0.02)
    sp.close()

    payload = {
        "meta": {
            "port": args.port,
            "baud": args.baud,
            "speed_mm": args.speed_mm,
            "cycles": args.cycles,
            "hold_s": args.hold,
            "t0_unix": t00,
            "tx_count": tx_count,
            "diag_count": len(samples),
            "tel_count": len(tel),
        },
        "phases": phase_log,
        "samples": samples,
        "tel": tel,
    }
    with open(args.out, "w") as fh:
        json.dump(payload, fh)
    sys.stderr.write("完成：%d 帧 -> %s\n" % (len(samples), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
