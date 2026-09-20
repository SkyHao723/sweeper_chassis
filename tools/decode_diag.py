#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解码 STM32 底盘板的扩展诊断帧 (0x7E), 用来查"某个轮子到底有没有劲"。

配合 firmware/stm32-chassis 使用。STM32 每 50ms 发一帧标准 24 字节遥测,
紧跟一帧 32 字节诊断帧。厂商 ROS 节点会把诊断帧整帧跳过(帧头是 0x7E 不是
0x7B), 所以这个脚本和厂商节点**不能同时开** —— 串口是独占的。
看诊断数据时先把底盘节点停掉:

    ros2 launch turn_on_wheeltec_robot base_serial.launch.py   # 先 Ctrl-C 停掉
    python3 tools/decode_diag.py --port /dev/wheeltec_controller

看完再把它起回来。

用法:
    # 实时看 (需要 pyserial)
    python3 tools/decode_diag.py -p /dev/wheeltec_controller
    # 只看有问题的轮子, 别的刷屏不管
    python3 tools/decode_diag.py -p /dev/wheeltec_controller --only-problems
    # 原始十六进制, 确认帧本身对不对
    python3 tools/decode_diag.py -p /dev/ttyUSB0 --raw
    # 事后分析: 把原始字节存下来再喂给它
    python3 tools/decode_diag.py -p /dev/ttyUSB0 --dump raw.bin
    python3 tools/decode_diag.py -f raw.bin

诊断帧布局 (32 字节, 多字节高字节在前):
    [0]     0x7E
    [1]     flags  bit0 曾收到命令 bit1 看门狗已停车 bit2 IMU有效 bit3 陀螺仪有效
    [2-3]   车体 vx   mm/s          [4-5]  车体 wz   mrad/s
    [6-7]   左轮 实际 RPM           [8-9]  右轮 实际 RPM
    [10-11] 左轮 目标 RPM           [12-13] 右轮 目标 RPM
    [14-15] 1号驱动器 输出电流 A*100   [16-17] 2号驱动器 输出电流 A*100
    [18]    1号驱动器 故障码        [19]   2号驱动器 故障码
    [20]    1号驱动器 模式          [21]   2号驱动器 模式
    [22]    CAN 错误计数            [23]   UART 坏帧
    [24]    UART 有效命令           [25]   UART 溢出
    [26]    继电器 bit0电机 bit1水泵  [27]  IMU 诊断码
    [28]    IMU I2C 地址            [29]   帧序号
    [30]    BCC                     [31]   0x7D
"""

import argparse
import struct
import sys
import time

CMD_TAIL = 0x7D
TEL_HEAD = 0x7B
DIAG_HEAD = 0x7E
TEL_SIZE = 24
DIAG_SIZE = 32

# 故障码表: docs/foc-driver-protocol-v1.2.pdf 第 11 页
FAULTS = {
    0: "正常",
    1: "驱动器故障",
    5: "过流",
    6: "过压",
    7: "欠压",
    8: "过温",
    23: "2号霍尔故障",
    24: "1号霍尔故障",
    25: "2号堵转",
    26: "1号堵转",
    27: "UART通讯故障",
    28: "RS485通讯故障",
    29: "CAN通讯故障",
    30: "摇杆故障",
    31: "转把故障",
}
MODES = {1: "电压", 4: "力矩", 5: "速度", 6: "位置"}

IMU_STATUS = {
    0: "正常",
    1: "SCL拉不高",
    2: "SDA拉不高",
    3: "总线死",
    4: "无应答",
    5: "读出错",
}

# 判定"没劲"的阈值
RPM_CMD_EPS = 5      # 目标转速绝对值超过它就认为"命令要求这个轮子转"
RPM_ACT_EPS = 3      # 实际转速低于它就认为"没转起来"
CUR_EPS = 30         # 电流 0.30 A (单位是 A*100)


def i16(b, i):
    return struct.unpack(">h", bytes(b[i:i + 2]))[0]


def fault_name(code):
    if code == 0:
        return "正常"
    return "%s(%d)" % (FAULTS.get(code, "未知故障"), code)


def mode_name(code):
    return MODES.get(code, "0x%02X" % code)


class Diag(object):
    __slots__ = ("flags", "vx", "wz", "wl", "wr", "tl", "tr",
                 "c1", "c2", "f1", "f2", "m1", "m2", "can_err",
                 "bad", "ok", "ovf", "relay", "imu_st", "imu_addr", "seq")

    def __init__(self, b):
        self.flags = b[1]
        self.vx, self.wz = i16(b, 2), i16(b, 4)
        self.wl, self.wr = i16(b, 6), i16(b, 8)
        self.tl, self.tr = i16(b, 10), i16(b, 12)
        self.c1, self.c2 = i16(b, 14), i16(b, 16)
        self.f1, self.f2 = b[18], b[19]
        self.m1, self.m2 = b[20], b[21]
        self.can_err, self.bad, self.ok = b[22], b[23], b[24]
        self.ovf, self.relay = b[25], b[26]
        self.imu_st, self.imu_addr = b[27], b[28]
        self.seq = b[29]

    def problems(self):
        """返回这个轮子当前的可疑描述; 没问题就返回 None。"""
        out = []
        for tag, tgt, act, cur, flt in (
                ("1号", self.tl, self.wl, self.c1, self.f1),
                ("2号", self.tr, self.wr, self.c2, self.f2)):
            if flt != 0:
                out.append("%s驱动器报故障: %s" % (tag, fault_name(flt)))
            elif abs(tgt) > RPM_CMD_EPS and abs(act) < RPM_ACT_EPS:
                if abs(cur) < CUR_EPS:
                    out.append("%s 命令 %+d RPM 但实际 %+d RPM 且电流仅 %.2fA"
                               " -> 驱动器没给力(查故障码/使能/接线)"
                               % (tag, tgt, act, cur / 100.0))
                else:
                    out.append("%s 命令 %+d RPM 但实际 %+d RPM 而电流 %.2fA"
                               " -> 给了力却被堵住/拖住"
                               % (tag, tgt, act, cur / 100.0))
        return out or None

    def line(self):
        return ("seq=%-3d vx=%+5d wz=%+5d | "
                "1号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s | "
                "2号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s | "
                "can_err=%d 坏帧=%d 溢出=%d relay=%d imu=%s"
                % (self.seq, self.vx, self.wz,
                   self.tl, self.wl, self.c1 / 100.0,
                   mode_name(self.m1), fault_name(self.f1),
                   self.tr, self.wr, self.c2 / 100.0,
                   mode_name(self.m2), fault_name(self.f2),
                   self.can_err, self.bad, self.ovf, self.relay,
                   IMU_STATUS.get(self.imu_st, "?%d" % self.imu_st)))


def iter_frames(stream):
    """按厂商那套帧同步规则切帧: 上一字节是帧尾(0x7D/0x7F)、本字节是帧头才入帧。
    故意和厂商 ROS 节点保持一致 —— 我们看到的帧边界就是它看到的。
    返回 (kind, bytes), kind 是 'tel' 或 'diag'。"""
    prev = None
    buf = bytearray()
    need = 0
    kind = None
    while True:
        chunk = stream.read(1)
        if not chunk:
            return
        b = chunk[0]
        if need == 0:
            after_tail = prev in (CMD_TAIL, 0x7F)
            if after_tail and b == TEL_HEAD:
                buf, need, kind = bytearray([b]), TEL_SIZE, "tel"
            elif after_tail and b == DIAG_HEAD:
                buf, need, kind = bytearray([b]), DIAG_SIZE, "diag"
            prev = b
            continue
        buf.append(b)
        prev = b
        if len(buf) < need:
            continue
        frame = bytes(buf)
        buf, need, k = bytearray(), 0, kind
        kind = None
        if frame[-1] != CMD_TAIL:
            continue
        bcc = 0
        for x in frame[:-2]:
            bcc ^= x
        if bcc != frame[-2]:
            continue
        yield k, frame


def open_stream(args):
    if args.file:
        return open(args.file, "rb"), None
    try:
        import serial
    except ImportError:
        sys.exit("需要 pyserial: pip3 install pyserial  "
                 "(或者先用 --dump 存文件, 再用 -f 离线分析)")
    sp = serial.Serial(args.port, args.baud, timeout=1)
    dump = open(args.dump, "wb") if args.dump else None
    return sp, dump


def main():
    ap = argparse.ArgumentParser(
        description="解码 STM32 扩展诊断帧 (0x7E)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/wheeltec_controller",
                    help="串口设备 (默认 /dev/wheeltec_controller)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-f", "--file", help="从文件读原始字节, 而不是串口")
    ap.add_argument("--dump", help="顺便把原始字节存到这个文件")
    ap.add_argument("--raw", action="store_true", help="打印每帧的十六进制")
    ap.add_argument("--only-problems", action="store_true",
                    help="只在发现问题时输出")
    ap.add_argument("--seconds", type=float, default=0,
                    help="跑这么久就退出 (0 = 一直跑)")
    args = ap.parse_args()

    stream, dump = open_stream(args)
    t0 = time.time()
    last_diag = None
    dropped = 0
    seen_seq = None

    try:
        for kind, frame in iter_frames(stream):
            if dump:
                dump.write(frame)
                dump.flush()
            if args.raw:
                print("%-4s %s" % (kind, frame.hex(" ")))
                continue
            if kind != "diag":
                continue

            d = Diag(frame)
            if seen_seq is not None:
                gap = (d.seq - seen_seq - 1) & 0xFF
                if 0 < gap < 128:
                    dropped += gap
            seen_seq = d.seq

            probs = d.problems()
            if args.only_problems and not probs:
                continue

            last_diag = d
            print(d.line())
            for p in (probs or []):
                print("    !! " + p)
            if dropped:
                print("    (累计丢帧 %d — 链路不稳, 数值要打折看)" % dropped)
            sys.stdout.flush()

            if args.seconds and (time.time() - t0) > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if last_diag is None:
            print("没有收到任何诊断帧。检查:\n"
                  "  - 底盘节点是不是还占着串口 (占着就收不到数据)\n"
                  "  - 设备名对不对 (/dev/wheeltec_controller)\n"
                  "  - STM32 固件里 Send_DiagFrame 有没有被调用")
        if dump:
            dump.close()
        try:
            stream.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
