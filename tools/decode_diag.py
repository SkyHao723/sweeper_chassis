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

诊断帧布局 (44 字节, 多字节高字节在前):
    [0]     0x7E
    [1]     flags  bit0 曾收到命令 bit1 看门狗已停车 bit2 IMU有效 bit3 陀螺仪有效
                   bit4 上次复位是 IWDG(主循环卡死过)
                   bit5 驱动器未就绪(在等回码, 此时只发刹车)
                   bit6 起步助推正在加力(轮子还没转起来)
                   bit7 助推加到上限还不转(轮子被顶住? 已进入给/歇循环, 直到目标归零才清)
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
    [26]    继电器 bit0滚刷 bit1水泵  [27]  IMU 诊断码
    [28]    IMU I2C 地址            [29]   帧序号
    [30]    IMU 寄存器体检标志位: bit0 版本 bit1 陀螺仪 bit2 磁力计
                                bit3 四元数 bit4 欧拉角
                                ★ bit5-6 = **车型号** (1=扫地机 2=洗地机)
                                  几何选错 -> 转向增益差 2.19 倍, 所以固件把它
                                  报出来; ARMCC5 不支持 #pragma message, 编译
                                  日志里留不下记号, 只能走这里
    [31]    IMU 版本号(主)
    [32-33] IMU 内部融合的偏航角 int16 0.01rad
    ---- 以下 4 项 2026-10-13 新增(帧 36 -> 44 字节), 目的是"多给主机一些数据帮它调参" ----
    [34]    ★ 饱和/保护状态 **当前拍**: 见 SAT_BITS
    [35]    ★ 同上, 但**本上电周期发生过没有**(锁存)
              位: bit0 vx指令被夹 bit1 wz指令被夹
                  bit2/3 左右轮目标被夹到 MAX_RPM
                  bit4 就绪联锁生效(只刹车) bit5 failsafe锁存
                  bit6 运动看门狗生效 bit7 链路看门狗生效
    [36-37] 底盘时基低 16 位 ms (65.5s 回绕) —— 给主机更好的时标
    [38-39] ★ 底盘**收到的**指令 vx  mm/s
    [40-41] ★ 底盘**收到的**指令 wz  mrad/s
              这两项让主机能回答它以前答不了的两个问题:
                ① "我发的命令到底到了没有"(和自己发出去的值比)
                ② "到了之后有没有被夹"(和 MAX_LIN_SPEED/MAX_YAW_RATE 比)
    [42]    BCC                     [43]   0x7D
"""

import argparse
import collections
import os
# ★ re 是为了 parse_saved_log(离线复核历史日志要解析自己打印的文本格式)。
#   这个工具主体是**手工按字节解析**的, 所以一开始没有 re —— 别以为它到处都是。
import re
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
DIAG_SIZE = 44
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
#
# ★★ 2026-10-12 修: 以前这里写死扫地机的 205/930, 于是**洗地机上这一列恒偏 +10 RPM**
#    (0.3 m/s: 扫地机名义 27.9 RPM, 洗地机 38.2 RPM), 看起来像"外环一直在加 10",
#    把助推/外环的分析带偏了一晚上。
#    现在改成**按诊断帧里的车型号自动选几何**(车型号是固件上报的, 见 vehicle_text)。
GEOM = {                       # 车型号 -> (轮径 mm, 轮距 mm)
    1: (205.0, 930.0),         # 扫地机
    2: (150.0, 425.0),         # 洗地机
}
DEFAULT_GEOM = (205.0, 930.0)  # 旧固件(f[30] bit5-6 = 0)只能按扫地机算


def nominal_rpm(vx, wz, vehicle=0):
    """主机下发的 (vx, wz) -> 左右轮理论物理转速, 和固件 Drive_Apply 同一套公式。

    ★ vehicle 必须来自诊断帧的车型号; 传 0(旧固件)就只能退回扫地机几何,
      这时候洗地机上的结果会偏, 提示一下比默默算错强。"""
    d_mm, t_mm = GEOM.get(vehicle, DEFAULT_GEOM)
    circ = d_mm / 1000.0 * 3.141592653589793
    track = t_mm / 1000.0
    v_l = vx - wz * track * 0.5
    v_r = vx + wz * track * 0.5
    return (v_l / circ * 60.0, v_r / circ * 60.0)


def i16(b, i):
    return struct.unpack(">h", bytes(b[i:i + 2]))[0]


def u16(b, i):
    return struct.unpack(">H", bytes(b[i:i + 2]))[0]


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


def tel_accel_ms2(frame):
    """24 字节主遥测帧 [8-9][10-11][12-13] = 加速度计三轴原始值。
    单位是协议规定的 ±2g 量程原始值: m/s^2 = 原始值 / 1671.84。

    ★ 这个量最大的用途是给"轮速到底是不是真在抖"当**独立裁判**:
      驱动器回码在 0~71 RPM 之间暴力来回冲时, 有两种可能 ——
        (a) 车真的在一窜一窜 -> 纵向加速度必然跟着摆, 幅度约
            A = v_pp * pi * f, 就算 0.76m/s 峰峰值 / 1Hz 也有 ±2.4 m/s^2;
        (b) 只是驱动器的转速估计在乱跳, 车其实走得很顺 -> 加速度是平的,
            只剩路面噪声(约 0.1~0.3 m/s^2)。
      加速度计和轮速完全独立, 所以这一眼就能定性。"""
    return (i16(frame, 8) / 1671.84, i16(frame, 10) / 1671.84,
            i16(frame, 12) / 1671.84)


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


def build_relay(mask):
    """构造 11 字节**继电器**帧 (本工程自定义扩展, 和速度帧同头同尾同长度,
    靠 f[1]=FUNC_RELAY 区分)。f[2] = 掩码: bit0 滚刷, bit1 水泵。

    ★ 别把它当速度帧发: f[3..8] 会被解析成 vx/wz。STM32 那边先判 f[1], 所以
      只要帧本身合法就不会误解析 —— 但第三方上位机不认识这条帧, 别指望它们发。

    ★ 必须**持续发**: STM32 的断链保护(RELAY_OFF_ON_LINK_LOSS)在 800ms 收不到
      任何合法帧时会把两个继电器都断开。所以"开滚刷"是"持续喂帧"而不是"发一次"。"""
    f = bytearray(CMD_SIZE)
    f[0] = CMD_HEAD
    f[1] = 0x05                    # FUNC_RELAY
    f[2] = mask & 0x03
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


#========================= 测试有效性判定 (堵转 / 偏航失控) =========================
# ★ 为什么要有这一段 —— 2026-10-13
#
# 之前所有"走直/转准"的测试都被同一个问题污染过: **车到底有没有顶到东西?**
#   实测出现过"轮子 0 RPM 却吃 20~43A", 这个特征有两个同样成立的解释:
#       (a) 驱动器/相线故障           (b) 轮子被顶住了(车头顶到墙 -> 该轮堵转)
#   分不清这两者, 任何结论都不成立 —— 我为此已经收回过一次"硬件故障"的判断。
#
# 所以把它变成**可判定、且自动终止**的: 跑的时候实时认, 认出来立刻停车,
# 并把这次判为**无效**, 免得又拿脏数据下结论。
#
# 两个判据(都要求"持续一段时间", 为的是躲开起步瞬态和噪声):
#   堵转: |目标| >= 20 RPM 且 |实际| < 5 RPM 且 |电流| >= 2.0A, 持续 >= 400ms
#         (起步时轮子还没破静摩擦, 也会有零点几秒的这种状态 —— 所以要设时长)
#   失控: |实际偏航率 - 命令偏航率| >= 0.45 rad/s, 持续 >= 200ms
#         (直行时命令是 0, 于是它就是"车在自己乱转")
#
# ★ 判出来后还要分"两轮都堵"还是"只有一轮堵":
#     两轮都堵 -> 整车被挡住了(或者总电源/被抬起来悬空)
#     只有一轮 -> 可能是车角顶住, 也可能是**那一条通道的故障** —— 两种情况
#                 用同一次测试分不开, 需要"把驱动轮垫起来"的空转测试来定
#============================================================================
STALL_TGT_RPM = 20.0     # 目标大于它才算"应该转"
STALL_ACT_RPM = 5.0      # 实际小于它才算"没转"
STALL_CUR_A   = 2.0      # 电流大于它才算"在使劲"(没使劲的 0 RPM 只是没收到命令)
STALL_MS      = 400      # 持续这么久才算堵转
RUNAWAY_WZ    = 0.45     # rad/s, 偏离命令多少算失控
# ★ RUNAWAY_MS 从 200ms 提到 600ms —— 是拿真实日志定出来的:
#   最好那次(每米偏航 +1.36, 净偏航只有 2 度)也有 8 帧(400ms)越过 0.45,
#   但那是起步/停车的瞬态, 判它"无效"就把好数据扔了;
#   而真正的失控(那次 -55 deg/m)是 21 帧(1.05s)连续越阈。
#   => 600ms 能把两者分开。改这个值前请先拿这两份日志复核。
RUNAWAY_MS    = 600
FRAME_MS      = 50.0     # 诊断帧 20Hz


def make_row(diag, gz=0.0):
    """把一帧诊断 + 当前陀螺仪读数摊平成判定器要的字典。
       c1/c2 换算成安培(协议是 A x10)。"""
    return {"tl": float(diag.tl), "tr": float(diag.tr),
            "wl": float(diag.wl), "wr": float(diag.wr),
            "c1": diag.c1 / 10.0, "c2": diag.c2 / 10.0,
            "vx": float(diag.vx), "gz": float(gz)}


def classify(rows, wz_cmd=0.0):
    """rows = [make_row(...), ...] -> 判定字典。无副作用, 可以离线喂历史日志。"""
    res = {"verdict": "有效", "reason": "", "frames": len(rows),
           "stall_l": 0, "stall_r": 0, "runaway": 0,
           "distance": 0.0, "yaw": 0.0, "yaw_per_m": 0.0}
    if len(rows) < 5:
        res["verdict"] = "无数据"
        return res

    need_stall = max(1, int(STALL_MS / FRAME_MS))
    need_run   = max(1, int(RUNAWAY_MS / FRAME_MS))
    run_l = run_r = run_dev = 0
    best_l = best_r = best_dev = 0

    for r in rows:
        # ---- 堵转(左右各判各的) ----
        for side in ("l", "r"):
            t = abs(r.get("t" + side, 0.0))
            a = abs(r.get("w" + side, 0.0))
            c = abs(r.get("c1" if side == "l" else "c2", 0.0))
            if t >= STALL_TGT_RPM and a < STALL_ACT_RPM and c >= STALL_CUR_A:
                if side == "l":
                    run_l += 1; best_l = max(best_l, run_l)
                else:
                    run_r += 1; best_r = max(best_r, run_r)
            else:
                if side == "l":
                    run_l = 0
                else:
                    run_r = 0
        # ---- 偏航失控 ----
        dev = abs(r.get("gz", 0.0) - wz_cmd)
        run_dev = run_dev + 1 if dev >= RUNAWAY_WZ else 0
        best_dev = max(best_dev, run_dev)

    res["stall_l"], res["stall_r"], res["runaway"] = best_l, best_r, best_dev
    res["distance"] = sum(r.get("vx", 0.0) for r in rows) * FRAME_MS / 1000.0 / 1000.0
    res["yaw"]      = sum(r.get("gz", 0.0) for r in rows) * FRAME_MS / 1000.0
    if res["distance"] > 0.1:
        res["yaw_per_m"] = res["yaw"] / res["distance"] * 57.2958

    if best_l >= need_stall or best_r >= need_stall:
        both = best_l >= need_stall and best_r >= need_stall
        res["verdict"] = "★无效(轮子堵转)"
        res["reason"] = ("1号 %d 帧 / 2号 %d 帧 有目标却不转且有大电流。%s"
                         % (best_l, best_r,
                            "**两个轮子都堵** -> 整车被挡住了(或驱动轮悬空), "
                            "不是单通道问题"
                            if both else
                            "**只有一轮堵** -> 可能是车角顶住, 也可能是该通道故障;"
                            "要分清必须做'驱动轮垫起来空转'的测试"))
    elif best_dev >= need_run:
        res["verdict"] = "★无效(偏航失控)"
        res["reason"] = ("实际偏航率偏离命令 %.2f rad/s 以上持续 %d 帧 —— "
                         "车在自己乱转, 这次数据不能用" % (RUNAWAY_WZ, best_dev))
    return res


def print_verdict(res, prefix="  "):
    print("%s判定: %s" % (prefix, res["verdict"]))
    if res["reason"]:
        print("%s  %s" % (prefix, res["reason"]))
    print("%s  里程 %.2f m | 偏航积分 %+.3f rad | 每米偏航 %+.2f deg/m"
          % (prefix, res["distance"], res["yaw"], res["yaw_per_m"]))
    print("%s  堵转帧: 1号 %d / 2号 %d | 失控帧: %d  (共 %d 帧)"
          % (prefix, res["stall_l"], res["stall_r"], res["runaway"], res["frames"]))


def selftest_classify():
    """★ 判定器的自检 —— 用构造数据验证逻辑, 不需要硬件。
       这是本轮"可验证"的部分: 三种场面必须分别判对。"""
    def mk(n, tl, tr, wl, wr, c1, c2, gz=0.0, vx=300.0):
        return [{"tl": tl, "tr": tr, "wl": wl, "wr": wr,
                 "c1": c1, "c2": c2, "vx": vx, "gz": gz}] * n

    cases = [
        ("正常直行",            mk(60, 38, 38, 39, 34, 5.0, -3.0), 0.0, "有效"),
        ("2号堵转(0RPM+大电流)", mk(60, 38, 49, 36, 0.0, 6.0, -22.6), 0.0, "★无效(轮子堵转)"),
        ("两轮都堵",            mk(60, 38, 38, 0.0, 0.0, 12.0, -12.0), 0.0, "★无效(轮子堵转)"),
        ("偏航失控",            mk(60, 38, 38, 39, 34, 5.0, -3.0, gz=1.05), 0.0, "★无效(偏航失控)"),
        ("短瞬态不算堵转",       mk(4, 38, 38, 0.0, 0.0, 12.0, -12.0) + mk(56, 38, 38, 39, 34, 5.0, -3.0), 0.0, "有效"),
        ("正常转弯(命令 0.6)",   mk(60, 30, 46, 31, 45, 5.0, 4.0, gz=0.60), 0.6, "有效"),
    ]
    ok = True
    for name, rows, wz, want in cases:
        got = classify(rows, wz)["verdict"]
        good = (got == want)
        ok = ok and good
        print("  [%s] %-22s -> %-14s (期望 %s)" % ("通过" if good else "失败", name, got, want))
    print("自检%s" % ("全部通过 ✓" if ok else "★有不通过"))
    return 0 if ok else 1


def parse_saved_log(path):
    """把以前保存的文本日志还原成 rows —— 用于离线复核历史数据。
       ★ 这是解析我们自己打印的格式, 格式改了这里也要改(所以放在同一个文件里)。"""
    rows = []
    for line in open(path, encoding="utf-8", errors="replace"):
        if not line.startswith("seq="):
            continue
        r = {}
        m = re.search(r"vx=\s*([-+]?\d+)", line)
        if m:
            r["vx"] = float(m.group(1))
        for side, pat in (("l", r"1号 目标\s*([-+]?\d+) 实际\s*([-+]?\d+)\s*([-+]?[\d.]+)A"),
                          ("r", r"2号 目标\s*([-+]?\d+) 实际\s*([-+]?\d+)\s*([-+]?[\d.]+)A")):
            m = re.search(pat, line)
            if m:
                r["t" + side] = float(m.group(1))
                r["w" + side] = float(m.group(2))
                r["c" + ("1" if side == "l" else "2")] = float(m.group(3))
        m = re.search(r"gyro=\([^)]*\)=([-+\d.]+),([-+\d.]+),([-+\d.]+)rad/s", line)
        if m:
            r["gz"] = float(m.group(3))
        # ★ 必须**所有字段都齐**才收: 日志里偶尔会缺一段(比如那帧还没收到陀螺仪
        #   遥测), 少字段的行混进来会让判定器 KeyError 崩掉 —— 实测就崩过一次。
        need = ("vx", "tl", "wl", "c1", "tr", "wr", "c2", "gz")
        if all(k in r for k in need):
            rows.append(r)
    return rows


def classify_saved_log(path, wz_cmd=0.0):
    rows = parse_saved_log(path)
    print("文件: %s   解析出 %d 帧" % (path, len(rows)))
    if not rows:
        print("★ 一帧都没解析出来 —— 文件格式不对, 或者当时就没抓到数据")
        return 1
    print_verdict(classify(rows, wz_cmd))
    return 0


class LiveGuard(object):
    """在跑的时候实时认"堵转/偏航失控"。增量维护连续计数, 不用每帧重扫全部历史。

    ★ 触发就停车, 不是"记下来等跑完再说" —— 实测见过被堵住的轮子持续吃 43A,
      那纯粹在发热; 而且既然这次已经无效, 早点停也省得车一直往墙上顶。"""

    def __init__(self, wz_cmd=0.0):
        self.wz = wz_cmd
        self.run_l = self.run_r = self.run_dev = 0
        self.tripped = None       # None / "stall" / "runaway"
        self.detail = ""

    def feed(self, row):
        if self.tripped:
            return
        need_stall = max(1, int(STALL_MS / FRAME_MS))
        need_run   = max(1, int(RUNAWAY_MS / FRAME_MS))

        for side in ("l", "r"):
            t = abs(row["t" + side])
            a = abs(row["w" + side])
            c = abs(row["c1" if side == "l" else "c2"])
            if (t >= STALL_TGT_RPM) and (a < STALL_ACT_RPM) and (c >= STALL_CUR_A):
                if side == "l":
                    self.run_l += 1
                else:
                    self.run_r += 1
            else:
                if side == "l":
                    self.run_l = 0
                else:
                    self.run_r = 0

        for side, cnt in (("1号", self.run_l), ("2号", self.run_r)):
            if cnt >= need_stall:
                self.tripped = "stall"
                self.detail = ("%s轮有目标(%d RPM)却不转(实际 %.0f)且电流 %.1fA, "
                               "持续 %dms —— 车可能顶到东西了, 或者该通道故障"
                               % (side, STALL_TGT_RPM, STALL_ACT_RPM,
                                  STALL_CUR_A, STALL_MS))
                return

        dev = abs(row["gz"] - self.wz)
        self.run_dev = self.run_dev + 1 if dev >= RUNAWAY_WZ else 0
        if self.run_dev >= need_run:
            self.tripped = "runaway"
            self.detail = ("偏航率偏离命令 %.2f rad/s 以上(实测 %.2f, 命令 %.2f) "
                           "持续 %dms —— 车在自己乱转"
                           % (RUNAWAY_WZ, row["gz"], self.wz, RUNAWAY_MS))


def mode_name(code):
    return MODES.get(code, "0x%02X" % code)


class Diag(object):
    __slots__ = ("flags", "vx", "wz", "wl", "wr", "tl", "tr",
                 "c1", "c2", "f1", "f2", "m1", "m2", "can_err",
                 "bad", "ok", "ovf", "relay", "imu_st", "imu_addr", "seq",
                 "probe", "imu_ver", "euler_yaw", "vehicle", "selftest_nomove",
                 "sat_now", "sat_latch", "tick_ms", "cmd_vx", "cmd_wz")

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
        self.probe = b[30] & 0x1F     # IMU 体检位, **只取低 5 位**
        # ★ 高三位是别的用途, 必须在这里掩掉: 否则下面 probe_text() 的
        #   `if self.probe == 0: return "没做体检"` 永远不成立(车型号恒 >=1),
        #   "没做体检"这个真实状态会被吃掉。
        self.vehicle = (b[30] >> 5) & 0x03          # bit5-6 车型
        self.selftest_nomove = (b[30] >> 7) & 0x01  # bit7 上电自检没转
        self.imu_ver = b[31]
        self.euler_yaw = i16(b, 32) / 100.0    # rad
        # ---- [34] 起: 2026-10-13 新增的"饱和/保护"区 (帧 36 -> 44 字节) ----
        self.sat_now   = b[34]      # 当前拍(瞬时), 位定义见 sat_text()
        self.sat_latch = b[35]      # 本上电周期发生过没有(锁存)
        self.tick_ms   = u16(b, 36) # 底盘时基低16位, 65.5s 回绕
        self.cmd_vx    = i16(b, 38) # ★底盘**收到的**指令 vx (mm/s)
        self.cmd_wz    = i16(b, 40) # ★底盘**收到的**指令 wz (mrad/s)

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
        if self.flags & 0x80:
            out.append(("booststuck",
                        "起步助推已经加到上限(KICK_MAX_RPM)还是没转起来 -> 固件"
                        "进入'给 3 秒 / 歇 2 秒'的循环(不是永久放弃), 并锁存了这个"
                        "标志(直到目标归零才清)。轮子被东西顶住, 或者驱动器根本没"
                        "使上劲 —— 对比电流: 电流大=真堵转, 电流小=驱动器没给力"))
        for tag, tgt, act, cur, flt in (
                ("1号", self.tl, self.wl, self.c1, self.f1),
                ("2号", self.tr, self.wr, self.c2, self.f2)):
            if flt != 0:
                out.append(("fault" + tag,
                            "%s驱动器报故障: %s" % (tag, fault_name(flt))))
            elif abs(tgt) > RPM_CMD_EPS and abs(act) < RPM_ACT_EPS:
                # 命令里含助推量, 所以"助推在给还不动"就是最硬的静摩擦/堵转证据
                boost = ""
                if self.flags & 0x40:
                    boost = ("（这一帧固件正在给起步助推, 说明它已经知道轮子没转 "
                             "—— 是起步门槛/堵转, 不是命令没送到）")
                if abs(cur) < CUR_EPS:
                    out.append(("weak" + tag,
                                "%s 命令 %+d RPM 但实际 %+d RPM 且电流仅 %.2fA"
                                " -> 驱动器没给力(查故障码/使能/接线)%s"
                                % (tag, tgt, act, cur / 10.0, boost)))
                else:
                    out.append(("stall" + tag,
                                "%s 命令 %+d RPM 但实际 %+d RPM 而电流 %.2fA"
                                " -> 给了力却被堵住/拖住%s"
                                % (tag, tgt, act, cur / 10.0, boost)))
        return out

    def line(self, bat_mv=0, gyro=None, nom=None, accel=None):
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
        a = ""
        if accel:
            a = " acc=(%+.2f,%+.2f,%+.2f)m/s2" % accel
        # 上报的"目标"是含外环修正**和起步助推**的最终命令。
        # ★ 这里显示的是 目标 - 主机理论目标, 也就是"修正 + 助推"的合计,
        #   不是积分的修正量本身。实测踩过: 看到 "+13" 以为积分已经顶到
        #   比例限幅(±13.95)了, 其实其中 12 是助推, 积分只剩 1 —— 判断完全
        #   反了。诊断帧没有空余字节放助推量, 所以固件在 flags bit6 上给了一个
        #   "助推正在加力"的位: **想知道这 +13 里有多少是助推, 看 bit6 就行了**
        #   (bit6 置位 = 这一帧的差值里包含助推; 没置位 = 全是外环积分)。
        trim = ""
        if nom is not None:
            dl = int(round(self.tl - nom[0]))
            dr = int(round(self.tr - nom[1]))
            if dl or dr:
                trim = " 目标-理论%+d/%+d(含助推)" % (dl, dr)
        return ("seq=%-3d %s%s vx=%+5d wz=%+5d | "
                "1号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s | "
                "2号 目标%+4d 实际%+4d %+6.2fA %-4s %-4s%s | "
                "can_err=%d 坏帧=%d 溢出=%d relay=%d imu=%s%s%s%s%s%s"
                % (self.seq, vehicle_text(self.vehicle),
                   selftest_text(self.selftest_nomove), self.vx, self.wz,
                   self.tl, self.wl, self.c1 / 10.0,
                   mode_name(self.m1), fault_name(self.f1),
                   self.tr, self.wr, self.c2 / 10.0,
                   mode_name(self.m2), fault_name(self.f2), trim,
                   self.can_err, self.bad, self.ovf, self.relay, imu, fl, bat, g, a,
                   ((" " + self.drive_text()) if self.drive_text() else "") +
                   # ★ 2026-10-13 新增: 底盘**收到的**指令 + 底盘时基。
                   #   让主机(或人)能确认"我发的命令到底到了没有"。没人发命令时
                   #   这里应该是 0/0, tick 一直在涨。
                   " rxcmd=%+.3f/%+.3f tick=%d" % (
                       self.cmd_vx / 1000.0, self.cmd_wz / 1000.0, self.tick_ms) +
                   sat_text(self.sat_now, self.sat_latch)))

    def drive_text(self):
        """诊断帧 [26] 的高位: 驱动模式 + 扭矩保护锁存。

        ★ 这两位是 2026-10 加"力矩模式"实验时新占的 —— 速度模式下**恒为 0**,
          所以老固件/老日志在这里只会印出空串, 不会误报。

        为什么值得单独印出来: "这块板烧的是速度模式还是力矩模式"上车前必须能一眼
        看出来 —— 和车型号同理。力矩模式的参数是没标定的, 而且保护一旦锁存就只刹车
        (车表现为"不走"), 不印出来就只能猜。"""
        out = []
        if self.relay & 0x04:
            out.append("★力矩模式固件")
        if self.relay & 0x08:
            out.append("**扭矩保护已锁存(超速/堵转, 只刹车)**")
        return "/".join(out)


SAT_BITS = ((0x01, "vx指令被夹"), (0x02, "wz指令被夹"),
            (0x04, "左轮目标被夹"), (0x08, "右轮目标被夹"),
            (0x10, "驱动器未就绪只刹车"), (0x20, "failsafe锁存"),
            (0x40, "运动看门狗生效"), (0x80, "链路看门狗生效"))


def sat_text(now, latch):
    """诊断帧 [34]/[35] (2026-10-13 新增, 帧从 36 字节扩到 44)。

    ★ 为什么要印出来: 主机要调自己的控制器, 必须知道**底盘做没做到它要的**。
      以前只有"目标 vs 实际", 看不出"目标是主机要的还是已经被夹过的"。
      典型: 主机发 wz=0.5, 底盘只能给 0.35 —— 主机不知道就一直积分(windup),
      表现为"越纠越偏"或"松手后猛冲"。

    印法: **[34] 当前**用 `!` 标出, **[35] 曾经发生过**用 `~` 标出。
      当前才在生效的(比如就绪联锁)看 [34]; 只想知道"这一趟出过什么事"看 [35]。"""
    now_s   = [n for m, n in SAT_BITS if now & m]
    latch_s = [n for m, n in SAT_BITS if (latch & m) and not (now & m)]
    if not now_s and not latch_s:
        return ""
    out = []
    if now_s:
        out.append("sat[现在] " + ",".join(now_s))
    if latch_s:
        out.append("sat[曾] " + ",".join(latch_s))
    return " " + " ".join(out)


def vehicle_text(v):
    """诊断帧 [30] 的 bit5-6: 固件是按哪辆车的几何编的。

    ★ 这个必须一眼看得见: 几何选错时, 转向增益会错 2.19 倍(扫地机 930mm 的
      固件烧到 425mm 的洗地机上 -> 转得过头一倍多), 里程计偏航也会同样错,
      导航会直接跑飞。ARMCC5 不支持 #pragma message, 编译日志留不下记号,
      所以只能靠诊断帧上报 —— 上车前先看这一项。"""
    return {0: "未标(旧固件)", 1: "扫地机205/930", 2: "洗地机150/425"}.get(
        v, "未知%d" % v)


def selftest_text(nomove):
    """诊断帧 [30] 的 bit7: 上电自检(延迟3秒后往前点动)的结果。

    ★ bit7 = 0 **有歧义**: 可能是"自检通过(动了一下)", 也可能是"压根没跑" ——
      驱动器没就绪时放弃、上位机先接管了(ever_linked)、或者上次是看门狗复位
      所以故意跳过。只有一个空位, 分不出这三种。实际判断办法是"上电 3 秒内
      有没有看到车往前动一下"。"""
    return "★上电自检:两轮都没转!" if nomove else ""


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
    if f & 0x20:
        # 驱动器还没回码 -> 只发刹车, 车不动是正常的(不是故障)
        out.append("驱动器未就绪(等回码, 车被刹住)")
    if f & 0x40:
        # 这一位的含义**随驱动模式变**(固件里由 Drive_ActiveFlag() 给):
        #   速度模式 = 起步助推正在加力。这是"小转向命令愣在原地"和"转向太敏感"
        #             这两个症状共同的现场证据: 助推在给 = 轮子还没转起来。
        #   力矩模式 = 扭矩环正在出力(|扭矩指令| > 0.05A)。
        # 台架上很好用: 命令了却一直不亮 = 环没在给(或保护锁存); 亮了却不动 =
        # 在给但推不动(电流不够 / 被卡住)。
        out.append("起步助推中/扭矩出力中")
    if f & 0x80:
        # 速度模式: 助推加到上限、KICK_MAX_MS 内轮子仍然没转 -> 被东西顶住
        # 力矩模式: 这条恒为 0(力矩模式走 Torque_Guard, 堵转由 f[26] bit3 报)
        out.append("**助推加到上限还不转(轮子被顶住?)**")
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
    ap.add_argument("--relay", type=int, metavar="MASK",
                    help="继电器保持模式: 只发继电器帧、不发速度。"
                         "1=滚刷 2=水泵 3=两个都开 0=都关。"
                         "必须持续发 —— STM32 断链 800ms 会自己把继电器断开。"
                         "Ctrl-C 退出时自动发 0 关掉。")
    ap.add_argument("--no-abort", action="store_true",
                    help="关掉「堵转/偏航失控」自动停车。默认是开的: 一旦认出这两种场面"
                         "(见 classify 的说明)就立刻发零速停车并把这次判为**无效**,"
                         "免得又拿「车顶到墙」那种脏数据下结论。")
    ap.add_argument("--classify", metavar="LOG",
                    help="离线判定: 读一个以前保存的日志文件, 给出这次测试有效/无效。"
                         "不需要硬件, 用来复核历史数据。")
    ap.add_argument("--classify-selftest", action="store_true",
                    help="判定器自检: 用构造数据验证堵转/失控/正常三种场面判得对(不需要硬件)。")
    args = ap.parse_args()

    if args.classify_selftest:
        return selftest_classify()
    if args.classify:
        return classify_saved_log(args.classify)

    stream, dump = open_stream(args)

    # ---- 继电器保持模式: 只发继电器帧, 绝不发速度帧 ----
    # 为什么只发继电器帧就够了、而且是**更好**的:
    #   STM32 里有两条独立的看门狗(见 main.c) ——
    #     运动看门狗: 800ms 没收到**速度帧** -> target 归零, 车停住;
    #     链路看门狗: 800ms 没收到**任何合法帧** -> 断开两个继电器。
    #   所以只发继电器帧的效果正好是"车停着、滚刷保持转", 不会误动车。
    if args.relay is not None:
        if args.file:
            sys.exit("--relay 需要真实串口, 不能和 -f 一起用")
        mask = args.relay & 0x03
        label = {0: "两个都关", 1: "滚刷开", 2: "水泵开", 3: "滚刷+水泵都开"}[mask]
        frame = build_relay(mask)
        print("# 继电器保持模式: %s (mask=%d)" % (label, mask))
        print("# 每 50ms 发一帧, 不发速度帧 -> 车保持停住。Ctrl-C 退出并关掉。")
        try:
            while True:
                stream.write(frame)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        except Exception as exc:
            print("!! 发送失败: %s" % exc)
        finally:
            try:
                for _ in range(STOP_BURST):
                    stream.write(build_relay(0))
                    time.sleep(0.02)
                print("# 已发 0, 两个继电器都断开", file=sys.stderr)
            except Exception:
                pass
            try:
                stream.close()
            except Exception:
                pass
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)

    sender = None
    # 只有 --drive 时才知道主机下发的理论目标, 才能把外环修正量还原出来。
    # ★ 注意这里先留 None: 几何要按诊断帧里的车型号算, 第一帧到了才能算(见下面)。
    nom = None
    if args.drive:
        if args.file:
            sys.exit("--drive 需要真实串口, 不能和 -f 一起用")
        if not hasattr(stream, "write"):
            sys.exit("--drive 需要真实串口 (当前读的是文件)")
        sender = Sender(stream, args.drive[0], args.drive[1])
        sender.start()
        print("# 正在下发 vx=%.3f m/s  wz=%.3f rad/s  (Ctrl-C 停止)"
              % (args.drive[0], args.drive[1]))
        print("# 理论轮速目标按诊断帧里的**车型号**算 "
              "(几何错了这一列会整体偏移, 以前就是这么被误导过)")

    t0 = time.time()
    last_diag = None
    dropped = 0
    seen_seq = None
    tracker = ProblemTracker()
    bat_mv = 0
    gyro_raw = None
    accel = None
    # ★ 实时有效性守卫 + 收集本轮的 rows(跑完给判定器用)
    guard = None
    drive_rows = []
    aborted = None
    if args.drive and not args.no_abort:
        guard = LiveGuard(args.drive[1])
    # 加速度峰峰值追踪: 这是判断"轮速是真在抖还是只是回码在抖"的独立裁判,
    # 见 tel_accel_ms2 的说明。取最近 2 秒(20Hz x 40 帧)的峰峰值。
    acc_hist = collections.deque(maxlen=40)
    n_diag = 0

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
                accel = tel_accel_ms2(frame)
                continue
            if kind != "diag":
                continue

            d = Diag(frame)
            if seen_seq is not None:
                gap = (d.seq - seen_seq - 1) & 0xFF
                if 0 < gap < 128:
                    dropped += gap
            seen_seq = d.seq

            # ★ 理论轮速要按**诊断帧上报的车型号**算几何(见 nominal_rpm 的说明)。
            #   所以不能提前算, 得等第一帧拿到车型号。
            if (nom is None) and args.drive:
                nom = nominal_rpm(args.drive[0], args.drive[1], d.vehicle)

            probs = tracker.update(d.problems())
            if args.only_problems and not probs:
                continue

            last_diag = d
            print(d.line(bat_mv, gyro_raw, nom, accel))
            for p in probs:
                print("    !! " + p)

            # ===== ★ 实时有效性守卫: 认出"堵转/偏航失控"就立刻停车 =====
            #  为什么必须实时: (1) 再往堵住的电机里灌电流就是纯发热(实测见过 0 RPM 吃 43A);
            #  (2) 抓到了就说明这次数据不能用, 早停早省事。
            if guard is not None:
                # tel_gyro_raw 给的是**原始值**, 换算成 rad/s (协议 ±500dps)
                # ★ 注意: 这里用的是原始符号, 没套固件里的 IMU_GYRO_Z_SIGN。
                #   直行(wz_cmd=0)时判据取绝对值, 符号无所谓;
                #   转弯测试时若符号约定和固件不一致, 需要手动核对。
                gz = (gyro_raw[2] * GYRO_RATIO) if gyro_raw else 0.0
                row = make_row(d, gz)
                drive_rows.append(row)
                guard.feed(row)
                if guard.tripped:
                    print("!! ★ %s" % guard.detail)
                    print("!! ★ 立刻零速停车, **本次测试判为无效** "
                          "(车可能顶到东西了, 或该通道故障 —— 见 classify 的说明)")
                    aborted = guard.tripped
                    if sender:
                        sender.halt()
                    break

            # 每 1 秒报一次加速度峰峰值 —— 用来判断轮速抖动是真是假
            # ★ 必须丢掉"不可能"的样本: 真实加速度计永远读到重力, 模长不可能接近 0。
            #   STM32 刚上电还没读到 IMU 时会发全 0, 那几帧会把峰峰值污染成 9.7,
            #   于是**每一次**开头的峰峰值都误报"车在一窜一窜"。实测被它骗过。
            #   用 L1 范数就够(判断"是不是接近 0"), 不必引入 math。
            if accel is not None and (abs(accel[0]) + abs(accel[1]) + abs(accel[2])) > 1.0:
                acc_hist.append(accel)
            n_diag += 1
            if n_diag % 20 == 0 and len(acc_hist) >= 20:
                pp = [max(a[i] for a in acc_hist) - min(a[i] for a in acc_hist)
                      for i in range(3)]
                print("    └ 加速度峰峰值(最近%d帧): x=%.2f y=%.2f z=%.2f m/s2"
                      % (len(acc_hist), pp[0], pp[1], pp[2]))
                print("      ^ 轮速在跳时看这个: 有 1 以上 -> 车真的在一窜一窜;"
                      " 只有 0.1~0.3 -> 车走得挺顺, 只是回码在乱跳")
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
    except Exception:
        # ★★★ 这个 except 是必须的, 不是多余 —— 2026-10-13 踩到:
        #   finally 里有 os._exit(0), 它会**跳过解释器打印 traceback**,
        #   于是任何解析异常都表现成"一帧都没收到、退出码 0、零输出"。
        #   那次是 line() 里少了一个格式化参数(TypeError), 却让我花了好几轮
        #   去怀疑串口/固件/帧长 —— 全是白费。
        #   **诊断工具自己出错时必须能说出来。**
        import traceback
        traceback.print_exc()
        print("★ 工具内部异常(不是没数据!) —— 上面的 traceback 才是真因",
              file=sys.stderr)
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
        # ★ 本次测试到底算不算数 —— 跑完必须给个结论, 否则又会拿脏数据下结论
        if args.drive and drive_rows:
            print()
            print("===== 本次测试有效性判定 =====")
            if aborted:
                print("  (被实时守卫提前终止: %s)" % aborted)
            print_verdict(classify(drive_rows, args.drive[1]))
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
