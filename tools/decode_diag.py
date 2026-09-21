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

诊断帧布局 (36 字节, 多字节高字节在前):
    [0]     0x7E
    [1]     flags  bit0 曾收到命令 bit1 看门狗已停车 bit2 IMU有效 bit3 陀螺仪有效
                   bit4 上次复位是 IWDG(主循环卡死过)
    [2-3]   车体 vx   mm/s          [4-5]  车体 wz   mrad/s
    [6-7]   左轮 实际 RPM           [8-9]  右轮 实际 RPM
    [10-11] 左轮 最终命令 RPM       [12-13] 右轮 最终命令 RPM
            ★ 这两个是**含外环修正**的最终命令, 不是主机下发的理论目标。
              用 --drive 时本脚本知道理论目标, 会自动把修正量还原出来显示。
    [14-15] 1号驱动器 输出扭矩电流 A*10   [16-17] 2号驱动器 输出扭矩电流 A*10
    [18]    1号驱动器 故障码        [19]   2号驱动器 故障码
    [20]    1号驱动器 模式          [21]   2号驱动器 模式
    [22]    CAN 错误计数            [23]   UART 坏帧
    [24]    UART 有效命令           [25]   UART 溢出
    [26]    继电器 bit0电机 bit1水泵  [27]  IMU 诊断码
    [28]    IMU I2C 地址            [29]   帧序号
    [30]    IMU 寄存器体检标志位: bit0 版本 bit1 陀螺仪 bit2 磁力计
                                bit3 四元数 bit4 欧拉角
    [31]    IMU 版本号(主)
    [32-33] IMU 内部融合的偏航角 int16 0.01rad
    [34]    BCC                     [35]   0x7D
"""

import argparse
import os
import struct
import sys
import threading
import time

CMD_SIZE = 11
CMD_HEAD = 0x7B
CMD_TAIL = 0x7D
TEL_HEAD = 0x7B
DIAG_HEAD = 0x7E
TEL_SIZE = 24
DIAG_SIZE = 36
SEND_HZ = 20          # 和 STM32 的 CAN_PERIOD_MS 对齐, 上位机一般也是这个量级
STOP_BURST = 15       # 退出前连发几帧零速, 保证车真的停下

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
# 注: 6 = "陀螺仪恒0" 已经删掉 —— 那个判定是错的。这颗 YbImu 在检测到静止时
# 本来就把角速度输出归零(实测: 静止 478 帧全零, 转动 321 帧都有数据),
# 所以"三轴全零"是正常现象, 不是故障。判断陀螺仪死活要看下面 gyro= 那几个数
# 在转动时变不变。

# 判定"没劲"的阈值
RPM_CMD_EPS = 5      # 目标转速绝对值超过它就认为"命令要求这个轮子转"
RPM_ACT_EPS = 3      # 实际转速低于它就认为"没转起来"
# ★ 电流的单位是 A×10, 不是 A×100。
#   协议原文(第 5 页): "DATA4 当前输出扭矩电流 int16 A ... 放大10 倍，保留1 位小数";
#   力矩模式命令那边也写着"写入 -100~0~100 对应 -10.0A~0~10.0A", 同样是 ×10。
#   一开始按 ×100 解读, 结果所有电流都少算了 10 倍, 差点把"电机在使劲"误判成"没给力"。
CUR_EPS = 10         # = 1.0A。低于它认为驱动器没在使劲。

# 协议规定的陀螺仪换算系数(±500dps 量程): rad/s = 原始值 * 0.00026644
GYRO_RATIO = 0.00026644

# 底盘几何 —— 必须和固件 main.c 里的一致, 否则还原出来的外环修正量是错的。
# 用途: 固件上报的"目标"是**含外环修正的最终命令**, 拿这里算出的理论目标一减,
# 就得到外环到底加了多少。见 main.c 的 Trim_Apply。
WHEEL_DIAMETER_MM = 205.0
TRACK_WIDTH_MM = 930.0
WHEEL_CIRC_M = WHEEL_DIAMETER_MM / 1000.0 * 3.141592653589793
TRACK_WIDTH_M = TRACK_WIDTH_MM / 1000.0


def nominal_rpm(vx, wz):
    """主机下发的 (vx, wz) -> 左右轮理论物理转速, 和固件 Drive_Apply 同一套公式。"""
    v_l = vx - wz * TRACK_WIDTH_M * 0.5
    v_r = vx + wz * TRACK_WIDTH_M * 0.5
    return (v_l / WHEEL_CIRC_M * 60.0, v_r / WHEEL_CIRC_M * 60.0)


def i16(b, i):
    return struct.unpack(">h", bytes(b[i:i + 2]))[0]


def tel_battery_mv(frame):
    """24 字节主遥测帧 [20-21] = 电池电压 mV。
    带载时的电压比静态值有意义得多 —— 一加速就塌下去说明电池/线路撑不住。"""
    return struct.unpack(">H", bytes(frame[20:22]))[0]


def tel_gyro_raw(frame):
    """24 字节主遥测帧 [14-15][16-17][18-19] = 陀螺仪三轴原始值。
    单位是协议规定的 ±500dps 量程原始值: rad/s = 原始值 * 0.00026644。

    ★ 为什么要把这三个数显出来: 陀螺仪"恒为 0"有两种可能 ——
      (a) 器件坏了
      (b) 它在静止时本来就输出零, 是我们的 gyro_all_zero 判定在误报
      分不清就可能在修一个根本没坏的东西。**转动一下板子看这三个数变不变**
      是一秒就能定性的测试。"""
    return (i16(frame, 14), i16(frame, 16), i16(frame, 18))


def build_cmd(vx, wz):
    """构造 11 字节速度命令帧, 格式见 docs/chassis-serial-protocol.md。
    和 STM32 的 Cmd_Verify() 对齐: BCC = XOR(f[0..8]), 放在 f[9], f[10]=0x7D。
    单位是 mm/s 和 mrad/s, 所以要 ×1000。"""
    vxi = max(-32768, min(32767, int(round(vx * 1000.0))))
    wzi = max(-32768, min(32767, int(round(wz * 1000.0))))
    f = bytearray(CMD_SIZE)
    f[0] = CMD_HEAD
    f[1] = 0                       # AutoRecharge, 正常走车 = 0
    f[2] = 0                       # SecurityPLY,  正常走车 = 0
    struct.pack_into(">h", f, 3, vxi)
    struct.pack_into(">h", f, 5, 0)     # vy, 差速车用不到
    struct.pack_into(">h", f, 7, wzi)
    bcc = 0
    for x in f[:CMD_SIZE - 2]:
        bcc ^= x
    f[CMD_SIZE - 2] = bcc
    f[CMD_SIZE - 1] = CMD_TAIL
    return bytes(f)


class Sender(threading.Thread):
    """后台线程: 按 SEND_HZ 持续发同一条速度命令。
    必须持续发 —— STM32 有 800ms 看门狗, 停发就自动刹车。"""

    def __init__(self, port, vx, wz):
        threading.Thread.__init__(self, daemon=True)
        self.port = port
        self.vx = vx
        self.wz = wz
        self.done = threading.Event()
        self.error = None

    def run(self):
        period = 1.0 / SEND_HZ
        while not self.done.is_set():
            try:
                self.port.write(build_cmd(self.vx, self.wz))
            except Exception as exc:            # 串口掉了之类
                self.error = exc
                return
            self.done.wait(period)

    def halt(self):
        """停发, 然后连发一串零速把车按住。"""
        self.done.set()
        self.join(timeout=1.0)
        for _ in range(STOP_BURST):
            try:
                self.port.write(build_cmd(0.0, 0.0))
            except Exception:
                return
            time.sleep(0.02)


def fault_name(code):
    if code == 0:
        return "正常"
    return "%s(%d)" % (FAULTS.get(code, "未知故障"), code)


def mode_name(code):
    return MODES.get(code, "0x%02X" % code)


class Diag(object):
    __slots__ = ("flags", "vx", "wz", "wl", "wr", "tl", "tr",
                 "c1", "c2", "f1", "f2", "m1", "m2", "can_err",
                 "bad", "ok", "ovf", "relay", "imu_st", "imu_addr", "seq",
                 "probe", "imu_ver", "euler_yaw")

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
        self.probe = b[30]          # 各功能块活没活
        self.imu_ver = b[31]
        self.euler_yaw = i16(b, 32) / 100.0    # rad

    def probe_text(self):
        """把 IMU 体检标志位翻成人话 —— 陀螺仪恒 0 时靠它定位问题。

        ★ 只列**真的去读过**的功能块。四元数(0x16)和欧拉角(0x26)目前故意
          不读(读 16/12 字节的长块会把模块卡成"总线死", 见 YbImu.c), 所以
          绝不能把它们报成"死" —— 那是没测, 不是坏。分不清这两者会把人
          带偏。"""
        if self.probe == 0:
            return "没做体检"
        names = ((0x01, "版本"), (0x02, "陀螺"), (0x04, "磁力"))
        alive = [n for bit, n in names if self.probe & bit]
        dead = [n for bit, n in names if not (self.probe & bit)]
        extra = ""
        if self.probe & 0x18:
            extra = " 四元数/欧拉也有数据"
        return "活:%s 死:%s ver=%d%s" % (
            "/".join(alive) or "-", "/".join(dead) or "-",
            self.imu_ver, extra)

    def problems(self):
        """返回 [(key, 描述), ...]; 没问题就返回空列表。
        key 用来做"连续出现多久"的统计, 见 ProblemTracker。"""
        out = []
        if self.flags & 0x10:
            # 这条最要紧: IWDG 不会无缘无故触发。触发过就说明主循环卡死过,
            # STM32 静默了一秒 —— 而驱动器没有命令超时, 那一秒里车是在按
            # 最后一条命令跑的。必须查清楚卡在哪。
            out.append(("iwdg",
                        "STM32 上次复位是看门狗引起的 —— 主循环卡死过! "
                        "驱动器没有命令超时, 卡死期间车会按最后一条命令继续跑, "
                        "必须查出卡在哪 (看是不是 I2C/USART 死等)"))
        if self.imu_st != 0:
            out.append(("imuother",
                        "IMU 读取异常: %s（体检: %s）"
                        % (IMU_STATUS.get(self.imu_st, "?%d" % self.imu_st),
                           self.probe_text())))
        for tag, tgt, act, cur, flt in (
                ("1号", self.tl, self.wl, self.c1, self.f1),
                ("2号", self.tr, self.wr, self.c2, self.f2)):
            if flt != 0:
                out.append(("fault" + tag,
                            "%s驱动器报故障: %s" % (tag, fault_name(flt))))
            elif abs(tgt) > RPM_CMD_EPS and abs(act) < RPM_ACT_EPS:
                if abs(cur) < CUR_EPS:
                    out.append(("weak" + tag,
                                "%s 命令 %+d RPM 但实际 %+d RPM 且电流仅 %.2fA"
                                " -> 驱动器没给力(查故障码/使能/接线)"
                                % (tag, tgt, act, cur / 10.0)))
                else:
                    out.append(("stall" + tag,
                                "%s 命令 %+d RPM 但实际 %+d RPM 而电流 %.2fA"
                                " -> 给了力却被堵住/拖住"
                                % (tag, tgt, act, cur / 10.0)))
        return out

    def line(self, bat_mv=0, gyro=None, nom=None):
        bat = (" bat=%.1fV" % (bat_mv / 1000.0)) if bat_mv else ""
        imu = IMU_STATUS.get(self.imu_st, "?%d" % self.imu_st)
        if self.imu_st != 0:
            imu += "[%s]" % self.probe_text()
        fl = flags_text(self.flags)
        fl = (" fl=%s" % fl) if fl else ""
        g = ""
        if gyro:
            g = " gyro=(%+d,%+d,%+d)=%.3f,%.3f,%.3frad/s" % (
                gyro[0], gyro[1], gyro[2],
                gyro[0] * GYRO_RATIO, gyro[1] * GYRO_RATIO, gyro[2] * GYRO_RATIO)
        # 上报的"目标"是含外环修正的最终命令; 知道主机下发的理论目标就能把它
        # 还原出来。只在非零时显示, 免得占地方。
        trim = ""
        if nom is not None:
            dl = int(round(self.tl - nom[0]))
            dr = int(round(self.tr - nom[1]))
            if dl or dr:
                trim = " 外环修正%+d/%+d" % (dl, dr)
        return ("seq=%-3d vx=%+5d wz=%+5d | "
                "1号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s | "
                "2号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s%s | "
                "can_err=%d 坏帧=%d 溢出=%d relay=%d imu=%s%s%s%s"
                % (self.seq, self.vx, self.wz,
                   self.tl, self.wl, self.c1 / 10.0,
                   mode_name(self.m1), fault_name(self.f1),
                   self.tr, self.wr, self.c2 / 10.0,
                   mode_name(self.m2), fault_name(self.f2), trim,
                   self.can_err, self.bad, self.ovf, self.relay, imu, fl, bat, g))


def flags_text(f):
    """把诊断帧 [1] 的 flags 翻成人话。只列真正置位的, 没置的不占地方。"""
    out = []
    if f & 0x01:
        out.append("已连过")
    if f & 0x02:
        out.append("看门狗停车")
    if f & 0x04:
        out.append("IMU加速OK")
    if f & 0x08:
        out.append("陀螺有数据")
    if f & 0x10:
        # 这条最重要: IWDG 不会无缘无故触发, 触发过就说明主循环卡死过
        out.append("**上次复位是看门狗(主循环卡死过!)**")
    return "/".join(out)


class ProblemTracker(object):
    """同一个问题要连续出现够久才报。

    为什么需要: 加速斜坡生效期间, 目标转速是慢慢爬上去的, 实际转速必然滞后
    一两帧, 电流也还没建立 —— 每一帧都符合"命令有、实际 0、电流≈0", 直接报
    就会在起步和换向时刷屏, 真正的故障反而被淹掉。PERSIST 帧(20Hz 下约 0.5s)
    之后还成立, 才认为是真问题。
    """

    PERSIST = 10

    def __init__(self):
        self.counts = {}
        self.reported = set()

    def update(self, probs):
        """传 [(key, 描述)]; 返回这一帧应该真正打印出来的描述列表。"""
        out = []
        seen = set()
        for key, text in probs:
            seen.add(key)
            n = self.counts.get(key, 0) + 1
            self.counts[key] = n
            if n >= self.PERSIST and key not in self.reported:
                self.reported.add(key)      # 只报一次, 不刷屏
                out.append(text)
        # 条件消失就清零, 下次复发还能再报一次
        for key in list(self.counts):
            if key not in seen:
                del self.counts[key]
                self.reported.discard(key)
        return out


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
    # ★ write_timeout 不能省。CH340 在 USB 层掉线之后, 无超时的 write() 会一直
    #   卡住 —— 而 halt() 里连发零速用的就是 write。实测踩到过: decode_diag 在
    #   读线程报 SerialException 之后卡在 halt() 的 write 上, **进程僵了 19 分钟
    #   不退出、还一直开着串口**, 于是后面所有测量都在"两个进程抢同一个 tty"的
    #   状态下做的, 数据全废。诊断工具自己必须先保证不僵死。
    sp = serial.Serial(args.port, args.baud, timeout=1, write_timeout=0.3)
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
    ap.add_argument("--drive", nargs=2, type=float, metavar=("VX", "WZ"),
                    help="边看边下发速度命令 (m/s 与 rad/s), 20Hz 持续发。"
                         "台架测试用: 否则只能下令或看数据二选一, 因为"
                         "串口是独占的。退出时自动连发零速停车。")
    args = ap.parse_args()

    stream, dump = open_stream(args)

    sender = None
    # 只有 --drive 时才知道主机下发的理论目标, 才能把外环修正量还原出来
    nom = nominal_rpm(args.drive[0], args.drive[1]) if args.drive else None
    if args.drive:
        if args.file:
            sys.exit("--drive 需要真实串口, 不能和 -f 一起用")
        if not hasattr(stream, "write"):
            sys.exit("--drive 需要真实串口 (当前读的是文件)")
        sender = Sender(stream, args.drive[0], args.drive[1])
        sender.start()
        print("# 正在下发 vx=%.3f m/s  wz=%.3f rad/s  (Ctrl-C 停止)"
              % (args.drive[0], args.drive[1]))
        print("# 理论轮速目标: 左 %+.1f RPM  右 %+.1f RPM"
              "  —— 固件上报的'目标'含外环修正, 相减就是修正量" % nom)

    t0 = time.time()
    last_diag = None
    dropped = 0
    seen_seq = None
    tracker = ProblemTracker()
    bat_mv = 0
    gyro_raw = None

    try:
        for kind, frame in iter_frames(stream):
            if dump:
                dump.write(frame)
                dump.flush()
            if args.raw:
                print("%-4s %s" % (kind, frame.hex(" ")))
                continue
            if kind == "tel":
                bat_mv = tel_battery_mv(frame)
                gyro_raw = tel_gyro_raw(frame)
                continue
            if kind != "diag":
                continue

            d = Diag(frame)
            if seen_seq is not None:
                gap = (d.seq - seen_seq - 1) & 0xFF
                if 0 < gap < 128:
                    dropped += gap
            seen_seq = d.seq

            probs = tracker.update(d.problems())
            if args.only_problems and not probs:
                continue

            last_diag = d
            print(d.line(bat_mv, gyro_raw, nom))
            for p in probs:
                print("    !! " + p)
            if dropped:
                print("    (累计丢帧 %d — 链路不稳, 数值要打折看)" % dropped)
            sys.stdout.flush()

            if sender and sender.error:
                print("!! 下发失败: %s" % sender.error)
                break
            if args.seconds and (time.time() - t0) > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if sender:
            sender.halt()           # 无论如何都要把车按住再退出
            print("# 已发零速停车", file=sys.stderr)
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

        # ★ 最后一道保险: 不管上面哪一步卡住, 都强制退出。
        #   诊断工具绝不允许僵在那里占着串口 —— 那会让后面所有测量都作废
        #   (两个进程抢同一个 tty, 字节被随机分走), 而且极难察觉。
        #   实测踩到过一次, 白测了一整轮。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
