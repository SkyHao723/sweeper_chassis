#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小车遥控器 Web 应用（RK3576 → STM32 → 达妙 DM-H65）。

数据通路（和旧的 web_driver 完全不同）：
    浏览器 --WebSocket/HTTP--> 本程序 --串口 115200--> STM32 桥
    STM32 --CAN--> 两个达妙轮毂电机；STM32 再把遥测/状态原路回传。

为什么只有标准库 + pyserial
    目标机是 RK3576（aarch64）上的现场设备，装 FastAPI/uvicorn 要联网拉包、
    还要管 systemd 里的 uvicorn 命令行；stdlib 的 ThreadingHTTPServer 足够
    应付"一个遥控器 + 一两个看客"，而且出问题时 `python3 car_web.py` 就能起，
    不用猜虚拟环境。WebSocket 的握手/分帧在下面自己实现（约 120 行），
    也正因为我们只需要"收小 JSON、推状态"这一种用法。

并发模型（硬性要求）
    串口只有一根：**只由 LinkWorker 这一个线程读写**。HTTP/WebSocket 线程
    只碰 CarHub 里加锁的共享状态，不碰串口。这样不会出现两个线程交错
    写命令帧、或者一边 read 一边 close 的经典崩法。

安全层次（三层，任何一层失效都还能停住）
    1. 前端：松手/回中 → 发 0（后端软停）；切后台/失焦/关页 → 发 0 + hard:true
       （后端立即归零）；WebSocket 断了就不再发。
    2. 本程序：500 ms 没收到网页指令 → 分级衰减（300 ms 70%、380 ms 50%、
       460 ms 30%）并在 500 ms 归零；急停锁存后只发 0；
       |vx|<=2.0 m/s、|wz|<=1.5 rad/s 硬夹 + 档位夹（固件上限 2.565 m/s）。
    3. STM32：800 ms 收不到合法命令帧 → 目标清零（固件里已实现）。
    ★ 分级衰减只让"网络抖动"的顿挫变柔和，绝不延长归零时间：
      硬上限仍是 500 ms 开始/完成归零，比固件 800 ms 看门狗更早。

缓启缓停（默认开，加减速分开）
    正常行驶指令在 step_output() 里按 vx 加速 1.0 / 减速 0.40 m/s²、
    wz 加速 4.0 / 减速 1.50 rad/s² 逼近目标：
      * 急停、心跳超时、前端断连、页面隐藏/失焦（hard:true）**立即归零**；
      * 松手、摇杆回中、主动点“停止”、/api/stop 是**正常停车**，按减速斜率软停。
    开关状态走 HTTP /api/ramp 或 WebSocket {"t":"ramp","on":...}，
    /api/state 与 /status 都会回显。

方向约定
    固件逆解 v_left=vx-wz*TRACK/2、v_right=vx+wz*TRACK/2，所以 wz>0 = 左转。
    "右推 = 右转"必须发 wz<0。唯一取反点是 static/app.js 的 WZ_CMD_SIGN=-1；
    本文件不做符号变换，遥测/轮速也按实际值原样显示。

自测
    python3 car_web.py --sim            # 不开串口，内部仿真产生 24/44 字节帧
    python3 selftest.py                 # 52 项断言（需要浏览器；否则 --no-browser）
    python3 car_web.py                  # 真机：/dev/wheeltec_controller @115200
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import select
import socket
import struct
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import unquote, urlparse

# ---------------------------------------------------------------------------
# 1. 协议常量与编解码
# ---------------------------------------------------------------------------

# 页面/接口版本：改前端静态资源时一定要一起改，index.html 的 ?v= 会跟着变，
# 否则浏览器会拿缓存的旧 app.js 跑新 HTML（现场排查过好几次）。
VERSION = "1.4.0"

CMD_HEAD = 0x7B
CMD_TAIL = 0x7D
CMD_SIZE = 11

TEL_HEAD = 0x7B          # 24 字节主遥测帧（含 IMU 六轴）
TEL_TAIL = 0x7D
TEL_SIZE = 24

ST_HEAD = 0x7E           # 44 字节状态帧
ST_TAIL = 0x7D
ST_SIZE = 44

# IMU 缩放：固件把 ±16g/±2000dps 换算成上位机约定的 ±2g/±500dps 原始值，
# 所以这里的系数必须和 firmware/stm32-chassis/user/main.c 里的完全一致。
ACC_LSB = 1671.84        # raw / 1671.84 = m/s²
GYRO_LSB = 0.00026644    # raw * 0.00026644 = rad/s

# 底盘几何（只用于界面换算显示，不参与控制）
# 轮距 456 mm 是 2026-10 新底盘实测值；旧值是 425 mm。
# ⚠ 真正决定实车转弯/里程的是固件 firmware/stm32-chassis/user/main.c 里的
#   TRACK_WIDTH_M；本文件的常量只影响 --sim 仿真和界面换算。固件没同步成
#   0.456 之前，真机回转半径/里程仍按 0.425 计算。
WHEEL_R_M = 0.0855
TRACK_WIDTH_M = 0.456
WHEEL_OMEGA_MAX = 30.0   # 固件限幅，界面条形图按这个满量程画

# 速度档位（6 档）：vx 上限 0.15 → 1.60 m/s；wz 同步收紧，
# 否则低速档一推满摇杆就原地甩尾，手感很危险。
# 默认仍是 low（0.15 m/s）：页面一打开、第一次推杆不会蹿出去。
GEARS = {
    "low":    {"name": "低速", "vx": 0.15, "wz": 0.50},
    "mid":    {"name": "中速", "vx": 0.30, "wz": 0.75},
    "high":   {"name": "高速", "vx": 0.50, "wz": 1.00},
    "fast":   {"name": "快速", "vx": 0.80, "wz": 1.15},
    "sprint": {"name": "极速", "vx": 1.20, "wz": 1.30},
    "ultra":  {"name": "超快", "vx": 1.60, "wz": 1.50},
}
GEAR_ORDER = ("low", "mid", "high", "fast", "sprint", "ultra")

# 硬上限：档位只能更小。**绝不能超过固件能力**：
#   WHEEL_OMEGA_MAX 30 rad/s × WHEEL_R_M 0.0855 m = 2.565 m/s。
# 这里取 2.0 m/s 留 0.565 m/s 余量；转向最坏情况
#   |vx| + |wz|*TRACK/2 = 2.0 + 1.5*0.228 = 2.342 m/s → 27.4 rad/s < 30，
# 所以极限组合也不会被固件削。改大之前先把这条算式重算一遍。
HARD_VX_LIMIT = 2.00
HARD_WZ_LIMIT = 1.50
FIRMWARE_VX_MAX = WHEEL_OMEGA_MAX * WHEEL_R_M   # 2.565 m/s，只用于注释/自测对照
CMD_TIMEOUT_MS = 500     # 网页心跳超时（< 固件的 800 ms，保证先由我们清零）

# ---- 心跳分级衰减（1.4.0）-----------------------------------------------
# 只用来说明"500 ms 之前"的阶段：手机 WiFi 抖动时，指令可能 300~500 ms 才到。
# 旧行为是"到 500 ms 一步从全速跳到 0"，车会明显"冲一下-停-再冲"。
# 现在按 (年龄阈值, 目标缩放) 分 3 档衰减，让减速更柔和；但 500 ms 仍然
# 强制归零（不延长），固件 800 ms 看门狗仍在更外层兜底。
#
# ★ 安全边界（改之前先读）：
#   * 最晚 300 ms 就开始降速，500 ms 必须到 0 —— 比原来的 500 ms 更早减速；
#   * 衰减只乘"目标"，实际的下降仍由 RAMP_VX_DEC / RAMP_WZ_DEC 限制；
#   * 急停 / hard:true / 断连永远不会走这条路径，仍是立即归零。
CMD_DEGRADE_STEPS = ((300.0, 0.70), (380.0, 0.50), (460.0, 0.30))
CMD_DIAG_WINDOW_MS = 2000  # 诊断滚动窗口，和前端 app.js 的 DIAG_WINDOW_MS 一致

# ---- 缓启缓停（指令斜坡）------------------------------------------------
# 只作用于"正常行驶指令"：目标 vx/wz 变化时按下面的斜率逐步逼近，
# 避免摇杆一推到底时的阶跃（阶跃会让电机瞬间按最大加速度冲出去）。
#
# 为什么加减速要分开：
#   加速可以快（要跟手），减速必须慢（才有"软停"的手感）。
#   如果共用一个斜率：
#     - 取 1.0：0.3 m/s 松手 0.3 s 就停，人几乎感觉不到；
#     - 取 0.4：起步 0.3 m/s 要 0.75 s，推杆会明显发肉。
#   所以 ACC=1.0 / DEC=0.40。从 0.30 m/s 松手：0.30 / 0.40 = 0.75 s，
#   停止距离 v²/(2a) = 0.30² / 0.80 = 0.1125 m —— 能明显看到"滑一小段"。
#   各档位满速软停时间/距离：
#     0.15 → 0.375 s / 2.8 cm     0.50 → 1.25 s / 31 cm
#     0.30 → 0.75 s  / 11 cm      0.80 → 2.00 s / 80 cm
#     1.20 → 3.00 s  / 1.80 m     1.60 → 4.00 s / 3.20 m
#   高速档软停距离长是物理必然（DEC 固定），所以 UI 对 ≥0.8 m/s 做橙色、
#   ≥1.2 m/s 做红色提示，README 写明"高速档只在空旷场地测试"。
#   真正危险时用急停/超时，那两条路径永远立即归零，不经过本斜坡。
#
#   wz 同理：ACC=4.0（1.00 rad/s → 0.25 s 到位），
#            DEC=1.50（1.00 rad/s → 0.67 s 停下）。
#
# 停止分类（见 step_output）：
#   立即归零：estop / timeout / 前端断连 / 页面隐藏·失焦（hard:true）。
#   软停（DEC）：松手、摇杆回中、键盘松开/空格、主动点停止、/api/stop。
RAMP_DEFAULT_ON = True
RAMP_VX_ACC = 1.0        # m/s²：从 0 到 0.30 约 0.30 s，跟手
RAMP_VX_DEC = 0.40       # m/s²：从 0.30 停下 0.75 s，人能明显感到"软停"
RAMP_WZ_ACC = 4.0        # rad/s²：0→1.0 约 0.25 s
RAMP_WZ_DEC = 1.5        # rad/s²：1.0→0 约 0.67 s
RAMP_MAX_DT = 0.2        # s；串口/线程卡顿后单拍最多积分这么多，防止一次跳变

# 方向约定（与 static/app.js 的 WZ_CMD_SIGN 一一对应）：
#   固件逆解 v_left = vx - wz*TRACK/2, v_right = vx + wz*TRACK/2，
#   所以 wz > 0 = 左轮慢/右轮快 = 车向左转（逆时针）。
#   "右推摇杆 = 右转"就必须发 wz < 0。
# 本文件收到/发出的是**协议域**的 wz：只在 static/app.js 的 WZ_CMD_SIGN
# 处取反一次；这里和遥测解码都不做符号变换（遥测是实际值，不能翻）。
# 所以 build_cmd_frame() 里不要再写一个负号，否则会双重取反。

# 状态帧 flags 位定义（与固件 Status_Send 一一对应）
FLAG_BITS = (
    ("left_online",   "左在线",   True,  0x01),
    ("right_online",  "右在线",   True,  0x02),
    ("left_enabled",  "左使能",   True,  0x04),
    ("right_enabled", "右使能",   True,  0x08),
    ("cmd_timeout",   "命令超时", False, 0x10),
    ("left_fault",    "左故障",   False, 0x20),
    ("right_fault",   "右故障",   False, 0x40),
    ("imu_ok",        "IMU",      True,  0x80),
)

# 达妙反馈帧 ERR 码（0/1 都算正常，其它是故障）
ERR_NAMES = {
    0: "失能", 1: "使能", 5: "读取校准数据错误", 6: "读取电机参数错误",
    7: "传感器读取异常", 8: "超压", 9: "欠压", 0xA: "过电流",
    0xB: "MOS 过温", 0xC: "线圈过温", 0xD: "通讯丢失", 0xE: "过载",
}

IMU_STATUS_NAMES = {
    0: "正常", 1: "SCL 拉不高", 2: "SDA 拉不高", 3: "总线死",
    4: "无应答", 5: "读出错",
}

BOOT_STAGE_NAMES = {
    0: "启动中", 1: "正常", 2: "CAN 初始化失败(已降级)",
    3: "IMU 从未读到(已跳过)", 4: "CAN 与 IMU 都失败",
}


# 颜色分级只在这里定义一次：后端把等级算进 /api/state，前端只负责套 CSS。
# 阈值来自现场经验：MOS/线圈 >70℃ 橙、>85℃ 红（达妙反馈的原始字节就是 ℃）；
# 反馈年龄 >200 ms 橙、>500 ms 红（200 ms 也是固件判轮速"新鲜"的门槛）。
def temp_level(c: int) -> str:
    if c > 85:
        return "bad"
    if c > 70:
        return "warn"
    return "ok"


def age_level(ms: int) -> str:
    if ms == 0xFFFF:                     # 固件用 0xFFFF 表示"从没收到过反馈"
        return "bad"
    if ms > 500:
        return "bad"
    if ms > 200:
        return "warn"
    return "ok"


def boot_level(stage: int) -> str:
    """启动阶段码 → 颜色等级（0 启动中按警告处理，1 正常，2/3 降级，4 双失败）。"""
    if stage == 1:
        return "ok"
    if stage in (2, 3):
        return "warn"
    if stage == 4:
        return "bad"
    return "warn"


def xor_bcc(data: bytes) -> int:
    """STM32 两侧共用的校验：逐字节异或（不含 BCC 自身和帧尾）。"""
    x = 0
    for b in data:
        x ^= b
    return x


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _ms(v: Optional[float]) -> str:
    """/status 用：None → "—"，数值 → 一位小数或不带小数。"""
    if v is None:
        return "—"
    return ("%.1f" % v) if v < 10 else ("%.0f" % v)


def slew(cur: float, target: float, max_step: float) -> float:
    """向 target 靠近最多 max_step（斜坡的单拍积分）。"""
    delta = target - cur
    if abs(delta) <= max_step:
        return target
    return cur + math.copysign(max_step, delta)


def slew_axis(cur: float, target: float, dt: float,
              acc: float, dec: float) -> float:
    """单轴斜坡：加速用 acc，减速/回零/反向用 dec。

    判定规则（每轴独立）：
      * |target| < |cur|：在收油门 / 软停 → dec；
      * 目标与当前反号（含从 0.3 直接给 -0.2）：先按 dec 收回，避免
        过零时还按加速斜率冲；
      * 其余（|target| >= |cur| 且同号）→ acc。
    这样"缓启"快、"缓停"慢，两个方向的手感都符合预期。
    """
    if cur == target:
        return cur
    if abs(target) < abs(cur) or (target != 0.0 and (cur > 0.0) != (target > 0.0)):
        rate = dec
    else:
        rate = acc
    return slew(cur, target, rate * dt)


def parse_onoff(v: Any) -> Optional[bool]:
    """把 HTTP/WS 里常见的开关写法归一成 bool；不认识返回 None。

    显式接受 0/1、"on"/"off"、true/false，避免前端/脚本传字符串时
    bool("false") == True 这种经典坑。
    """
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)) and v in (0, 1):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("1", "on", "true", "yes", "开", "开启"):
            return True
        if s in ("0", "off", "false", "no", "关", "关闭"):
            return False
    return None


def _i16(buf: bytes, off: int) -> int:
    return struct.unpack_from(">h", buf, off)[0]


def _u16(buf: bytes, off: int) -> int:
    return struct.unpack_from(">H", buf, off)[0]


def build_cmd_frame(vx_m_s: float, wz_rad_s: float) -> bytes:
    """组 11 字节命令帧（大端）。

    单位在"收发的边界"上换算一次：m/s → mm/s，rad/s → mrad/s。
    中间层统一用 m/s、rad/s，避免混单位（混单位是最难查的一类 bug）。

    wz 是**协议域**符号：wz<0 = 右转（固件逆解 wz>0 为左转）。
    "网页驾驶意图 → 协议值"的唯一取反点是 static/app.js 的 WZ_CMD_SIGN；
    本函数只搬运，不再翻符号，否则右推会变成左转。
    """
    vx_mm = int(round(clamp(vx_m_s, -HARD_VX_LIMIT, HARD_VX_LIMIT) * 1000.0))
    wz_mrad = int(round(clamp(wz_rad_s, -HARD_WZ_LIMIT, HARD_WZ_LIMIT) * 1000.0))
    vx_mm = int(clamp(vx_mm, -32768, 32767))
    wz_mrad = int(clamp(wz_mrad, -32768, 32767))

    f = bytearray(CMD_SIZE)
    f[0] = CMD_HEAD
    f[1] = 0                      # AutoRecharge，固件忽略
    f[2] = 0                      # SecurityPLY，固件忽略
    f[3] = (vx_mm >> 8) & 0xFF    # 负数用 Python 的算术右移 + &0xFF 即可得到补码高字节
    f[4] = vx_mm & 0xFF
    f[5] = 0                      # vy：差速车没有横向自由度，恒 0
    f[6] = 0
    f[7] = (wz_mrad >> 8) & 0xFF
    f[8] = wz_mrad & 0xFF
    f[9] = xor_bcc(bytes(f[0:9]))
    f[10] = CMD_TAIL
    return bytes(f)


def decode_telemetry(f: bytes) -> dict[str, Any]:
    """解析 24 字节主遥测帧。调用前必须已通过 帧头/帧尾/BCC 校验。"""
    return {
        "stop": bool(f[1]),
        "vx": _i16(f, 2) / 1000.0,
        "vy": _i16(f, 4) / 1000.0,
        "wz": _i16(f, 6) / 1000.0,
        "ax": _i16(f, 8) / ACC_LSB,
        "ay": _i16(f, 10) / ACC_LSB,
        "az": _i16(f, 12) / ACC_LSB,
        "gx": _i16(f, 14) * GYRO_LSB,
        "gy": _i16(f, 16) * GYRO_LSB,
        "gz": _i16(f, 18) * GYRO_LSB,
        "bat_mv": _u16(f, 20),
    }


def decode_status(f: bytes) -> dict[str, Any]:
    """解析 44 字节状态帧（偏移严格照 firmware main.c 的 Status_Send）。"""
    raw_flags = f[1]
    flags = {key: bool(raw_flags & bit) for key, _n, _g, bit in FLAG_BITS}

    def wheel(i: int) -> dict[str, Any]:
        base = 6 + i * 2
        err = f[18 + i]
        omega = _i16(f, base) / 100.0
        target = _i16(f, 10 + i * 2) / 100.0
        mos = f[20 + i]
        coil = f[22 + i]
        age = _u16(f, 32 + i * 2)
        dev = target - omega
        # 跟随度：只在 |目标| ≥0.5 rad/s 时才有意义（目标接近 0 时分母太小，
        # 100% 的抖动会被算成很大的跟随误差）。None = 不显示，不编数。
        follow = None
        if abs(target) >= 0.5:
            follow = round(clamp(100.0 * (1.0 - abs(dev) / abs(target)),
                                 0.0, 100.0), 1)
        return {
            "omega": omega,
            "target": target,
            # 偏差 = 目标 - 实际（rad/s）；正=电机还没跟上，负=冲过头
            "dev": round(dev, 2),
            "follow_pct": follow,
            "torque": _i16(f, 14 + i * 2) / 100.0,
            "err": err,
            "err_text": ERR_NAMES.get(err, "未知故障 0x%X" % err),
            "fault": err > 1,
            "mos_temp": mos,
            "coil_temp": coil,
            "mos_level": temp_level(mos),
            "coil_level": temp_level(coil),
            "pos": _i16(f, 24 + i * 2) / 100.0,
            "fb_age_ms": age,
            "age_level": age_level(age),
            "online": flags["left_online" if i == 0 else "right_online"],
            "enabled": flags["left_enabled" if i == 0 else "right_enabled"],
        }

    boot = f[41]
    return {
        "flags_raw": raw_flags,
        "flags": flags,
        "cmd_vx": _i16(f, 2) / 1000.0,
        "cmd_wz": _i16(f, 4) / 1000.0,
        "left": wheel(0),
        "right": wheel(1),
        "can_fail": f[28],
        "fb_count": f[29],
        "version": f[30],
        "imu_status": f[31],
        "imu_status_text": IMU_STATUS_NAMES.get(f[31], "未知(%d)" % f[31]),
        "tick": _u16(f, 36),
        "imu_addr": f[38],
        "imu_probe": f[39],
        "imu_gyro_ok": bool(f[39] & 0x80),
        "imu_probe_version": bool(f[39] & 0x01),
        "imu_probe_gyro": bool(f[39] & 0x02),
        "imu_probe_mag": bool(f[39] & 0x04),
        "imu_ver": f[40],
        "boot_stage": boot,
        "boot_stage_text": BOOT_STAGE_NAMES.get(boot, "未知(%d)" % boot),
        "boot_level": boot_level(boot),
    }


class FrameParser:
    """串口字节流 → 完整帧。

    帧同步规则（固件里写死的）：**上一字节是帧尾 0x7D，本字节是帧头**。
    固件先发 24 字节 0x7B 帧、再发 44 字节 0x7E 帧，所以两个帧头前面
    都有 0x7D，正好都能用同一条规则识别。

    为什么要"前一字节"而不是"见到 0x7B 就当帧头"：
        帧体里的数据（比如某个 ω 的低字节）随时可能是 0x7B/0x7E，单看
        当前字节会误入帧；有了前导 0x7D 约束，误判概率降一个量级，
        剩下的交给 BCC + 帧尾兜底。

    校验失败时只丢一个字节再重新找帧头，最多 N 个字节就能重新对齐，
    不需要清空缓冲、更不需要复位。
    """

    def __init__(self) -> None:
        self.buf = bytearray()
        self.prev = -1           # 上一个被消费的字节，用于跨 read() 的前导判断
        self.bad = 0             # 校验失败次数
        self.resync = 0          # 丢弃字节数（诊断用）

    def feed(self, data: bytes) -> list[tuple[str, bytes]]:
        self.buf.extend(data)
        out: list[tuple[str, bytes]] = []
        while True:
            start = -1
            for i, b in enumerate(self.buf):
                if b == TEL_HEAD or b == ST_HEAD:
                    prev = self.buf[i - 1] if i > 0 else self.prev
                    if prev == 0x7D:
                        start = i
                        break
            if start < 0:
                # 没有任何候选帧头：只留最后 1 字节（它可能是下个帧头的 0x7D 前导）
                if self.buf:
                    self.prev = self.buf[-1]
                    if len(self.buf) > 1:
                        self.resync += len(self.buf) - 1
                        del self.buf[:-1]
                break

            if start > 0:
                self.prev = self.buf[start - 1]
                self.resync += start
                del self.buf[:start]

            head = self.buf[0]
            size = TEL_SIZE if head == TEL_HEAD else ST_SIZE
            if len(self.buf) < size:
                break                       # 半帧：等下一次串口数据

            frame = bytes(self.buf[:size])
            if frame[-1] == 0x7D and xor_bcc(frame[:-2]) == frame[-2]:
                out.append(("tel" if head == TEL_HEAD else "status", frame))
                self.prev = 0x7D
                del self.buf[:size]
            else:
                # 头对、尾或 BCC 不对：这个"头"多半是数据字节，丢掉它重来
                self.bad += 1
                self.prev = self.buf[0]
                del self.buf[:1]
        return out


# ---------------------------------------------------------------------------
# 2. 共享状态（HTTP/WS 线程只读写这里，不碰串口）
# ---------------------------------------------------------------------------


class CarHub:
    """一个遥控器进程的全部共享状态。

    所有字段都在 self.lock 下访问。原则：**串口 I/O 绝不在持锁时做**，
    否则一个慢写就会把 20 Hz 的状态推送拖死。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.lock = threading.RLock()
        self.args = args
        self.sim = bool(args.sim)
        self.started = time.monotonic()

        # ---- 指令（网页给的目标，尚未夹取）----
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.cmd_owner: Optional[int] = None
        self.cmd_time = 0.0          # 最近一次网页指令的 monotonic 时间（0=从没收到）
        self.cmd_recv_count = 0

        # ---- 指令序号 / 诊断（1.4.0）----
        # 前端每条 cmd 带 session + seq + ts：
        #   session 每次页面加载随机生成，刷新后 seq 从 0 重来；
        #   后端只在该 session 内比较 seq，丢弃"旧序号"（乱序/重发）的包，
        #   避免晚到的旧目标把新目标覆盖回去。
        self.cmd_session: Optional[str] = None
        self.cmd_seq: Optional[int] = None
        self.cmd_last_client_ts: Optional[int] = None
        self.cmd_stale_packets = 0   # 被丢弃的旧序号/重复包
        self.cmd_seq_gaps = 0        # 序号跳号总数（丢包迹象）
        self.cmd_session_resets = 0  # 页面重载/换标签页次数

        # 最近 2 s 的指令到达时刻（含被丢弃的旧包：它们也真的到过）。
        # maxlen 取 1024：即使 120 Hz 发也能覆盖完整 2 s 窗口。
        self.cmd_arrivals: deque[float] = deque(maxlen=1024)
        # 每一拍 step_output 的分类，用于 /status 的 reason 分布。
        self.reason_frames: deque[tuple[float, str]] = deque(maxlen=512)
        self.reason_total = {"web": 0, "stale": 0, "timeout": 0,
                             "hard": 0, "estop": 0}
        self._prev_reason: Optional[str] = None
        # ★ 最关键诊断：因心跳超时而被清零的"次数"（不是帧数）。
        # 只在前一帧还是 web/stale 且当前输出确实非 0 时 +1，
        # 这样手机操作时这个数在涨 → 就是网络/心跳断流。
        self.timeout_zero_count = 0
        self.last_timeout_s: Optional[float] = None

        # ---- 安全 ----
        self.estop = False
        self.estop_time = 0.0
        self.gear = "low"            # 默认最慢档：第一次打开页面不会蹿出去
        # hard_stop=True 表示"这次目标是安全停车，必须立即归零"：
        # 页面隐藏 / 窗口失焦 / pagehide 会带 hard:true 发 0；锁存到下一次
        # 非零行驶指令为止。这样随后的 30 Hz 0 心跳不会把硬停又变回软停。
        self.hard_stop = False

        # ---- 缓启缓停（斜坡）----
        # 默认开启：更平顺。out_* 是**真正写串口**的值（LinkWorker.step_output
        # 推进）；ramp_* 只给状态/诊断用。正常停车走 DEC 软停，只有
        # estop/timeout/hard_stop 立即归零（见 step_output 的注释）。
        self.ramp_on = RAMP_DEFAULT_ON
        self.out_vx = 0.0
        self.out_wz = 0.0
        self._ramp_t = 0.0           # 上一次推进斜坡的 monotonic 时间（0=还没推过）
        self.ramp_step_count = 0

        # ---- 串口 ----
        self.link_open = False
        self.link_error = ""
        # 仿真时不要把真实串口路径报给界面：否则 /status 会显示
        # "/dev/wheeltec_controller OK"，现场很容易误判成串口已经通了。
        self.link_port = "SIM" if args.sim else args.serial
        self.tx_count = 0
        self.tx_vx = 0.0
        self.tx_wz = 0.0
        self.tx_time = 0.0
        self.tel_count = 0
        self.status_count = 0
        self.bad_frames = 0
        self.resync_bytes = 0

        # ---- 最新帧 ----
        self.tel: Optional[dict[str, Any]] = None
        self.tel_time = 0.0
        self.status: Optional[dict[str, Any]] = None
        self.status_time = 0.0

        # ---- 里程（用实际轮速反解的 vx/wz 积分，不是命令值）----
        self.distance = 0.0
        self.yaw = 0.0
        self._odom_last: Optional[tuple[float, float, float]] = None

        # ---- WebSocket ----
        self.ws_clients: dict[int, Any] = {}
        self._next_client_id = 1
        self.ws_connect_count = 0

        # ---- 事件与边沿检测 ----
        self.events: deque[dict[str, Any]] = deque(maxlen=24)
        self._prev_timeout: Optional[bool] = None
        self._prev_fault = False
        self._prev_link: Optional[bool] = None
        self.note("程序启动（%s 模式）" % ("仿真" if self.sim else "真机"))
        self.note("缓启缓停：%s（加速 vx %.2f / 减速 vx %.2f m/s²；"
                  "加速 wz %.2f / 减速 wz %.2f rad/s²）"
                  % ("开" if self.ramp_on else "关",
                     RAMP_VX_ACC, RAMP_VX_DEC, RAMP_WZ_ACC, RAMP_WZ_DEC))
        self.note("停车：急停/超时/断连/页面隐藏立即归零；松手/回中/主动停止走软停")

    # -- 事件 ---------------------------------------------------------------
    def note(self, msg: str) -> None:
        with self.lock:
            self.events.append({"t": time.strftime("%H:%M:%S"), "msg": msg})

    # -- 指令 ---------------------------------------------------------------
    def set_cmd(self, vx: float, wz: float, owner: int,
                hard: bool = False, seq: Optional[int] = None,
                session: Optional[str] = None,
                ts: Optional[int] = None) -> None:
        """记录网页目标值。

        hard=True 只对"目标为 0"的指令有意义：页面隐藏 / 窗口失焦 /
        pagehide 走的是安全停车，必须立即归零；而松手、回中、主动点停止
        发的是普通 0，走斜坡减速（软停）。hard 会锁存到下一次非零指令，
        避免随后的 0 心跳把这次硬停又变回软停。

        **急停锁存期间不接受新目标**：否则复位后会把锁存期间收到的
        最后一帧指令放出来（"复位后必须重新推摇杆"的承诺就破了）。
        急停时只保留 owner/count 用于诊断，目标与时间戳都保持 0。

        1.4.0 新增 seq / session / ts：
          * session 每次页面加载随机生成；换页面后 seq 从 0 重来，
            后端据此重置基线，不会把新页面的指令当"旧包"丢掉。
          * 同一 session 内 seq 单调递增；`seq <= 上次` 视为乱序/重发的旧包，
            直接丢弃（不计入心跳、不覆盖目标），只累加诊断计数。
            TCP/WS 本身有序，这条主要防 HTTP 多连接和浏览器重发。
        """
        vx = float(vx)
        wz = float(wz)
        # JSON 里的 NaN/Infinity 会被 Python 的 json 接受，但 int(round(nan))
        # 会在组帧时抛异常；这里直接判掉，别让一个坏请求把串口线程带崩。
        if not (math.isfinite(vx) and math.isfinite(wz)):
            vx = 0.0
            wz = 0.0
        now = time.monotonic()
        with self.lock:
            self.cmd_owner = owner
            self.cmd_recv_count += 1
            # 到达时刻先记：即使是被丢弃的旧包，它也确实占用了网络链路，
            # 到达间隔统计应该包含它。
            self.cmd_arrivals.append(now)

            if session is not None:
                session = str(session)
                if self.cmd_session is not None and session != self.cmd_session:
                    self.cmd_session_resets += 1
                    self.cmd_seq = None
                self.cmd_session = session

            seq_i: Optional[int] = None
            if seq is not None:
                try:
                    seq_i = int(seq)
                except (TypeError, ValueError, OverflowError):
                    seq_i = None
            if seq_i is not None:
                if self.cmd_seq is not None:
                    if seq_i <= self.cmd_seq:
                        self.cmd_stale_packets += 1
                        return
                    if seq_i > self.cmd_seq + 1:
                        self.cmd_seq_gaps += seq_i - self.cmd_seq - 1
                self.cmd_seq = seq_i

            if ts is not None:
                try:
                    self.cmd_last_client_ts = int(ts)
                except (TypeError, ValueError, OverflowError):
                    pass

            if self.estop:
                self.cmd_vx = 0.0
                self.cmd_wz = 0.0
                self.cmd_time = 0.0
                return
            self.cmd_vx = vx
            self.cmd_wz = wz
            self.cmd_time = now
            if vx != 0.0 or wz != 0.0:
                # 只要重新开始行驶，就清掉硬停锁存（下一次停车按新指令分类）。
                self.hard_stop = False
            elif hard:
                self.hard_stop = True

    def drop_owner(self, owner: int) -> None:
        """某个网页断开：如果它正是最后发指令的人，立刻清零。

        为什么要判断 owner：同时开两个标签页看状态时，关掉"只看不控"的那个
        不应该把正在开的车停掉。反过来，控制端一断就必须停 —— 这是安全底线。
        这里是**立即**归零：断连属于安全停止，不走缓停斜坡。
        """
        with self.lock:
            if self.cmd_owner == owner and not self.estop:
                self.cmd_vx = 0.0
                self.cmd_wz = 0.0
                self.cmd_time = 0.0
                self.hard_stop = True
                self.note("控制端 WebSocket 断开，立即归零")

    def set_gear(self, gear: str) -> bool:
        if gear not in GEARS:
            return False
        with self.lock:
            if gear != self.gear:
                self.gear = gear
                self.note("切换到%s档（vx≤%.2f m/s，wz≤%.2f rad/s）"
                          % (GEARS[gear]["name"], GEARS[gear]["vx"], GEARS[gear]["wz"]))
        return True

    def set_ramp(self, on: bool) -> bool:
        """开/关缓启缓停。返回是否真的发生了变化。

        切换只改标志，不动 out_vx/out_wz：
          - 运行中关闭：下一拍 step_output 直接把目标赋给输出（允许阶跃）；
          - 运行中开启：从当前实际输出继续按斜率逼近，不会先跳一下。
        """
        on = bool(on)
        with self.lock:
            if on == self.ramp_on:
                return False
            self.ramp_on = on
            if on:
                self.note("缓启缓停已开启（加速 vx %.2f / 减速 vx %.2f m/s²，"
                          "加速 wz %.2f / 减速 wz %.2f rad/s²）"
                          % (RAMP_VX_ACC, RAMP_VX_DEC,
                             RAMP_WZ_ACC, RAMP_WZ_DEC))
            else:
                self.note("缓启缓停已关闭（指令直接阶跃到目标，停车也立即归零）")
            return True

    def estop_on(self, why: str = "网页急停") -> None:
        with self.lock:
            self.estop = True
            self.estop_time = time.monotonic()
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
            self.cmd_time = 0.0
            self.hard_stop = True
            self.note("🛑 急停：%s（已锁存，需手动复位）" % why)

    def estop_off(self) -> None:
        with self.lock:
            if not self.estop:
                return
            self.estop = False
            self.estop_time = 0.0
            # 双保险：即使有绕过 set_cmd 的路径在锁存期间改了目标，
            # 复位后也必须是 0，等用户重新推摇杆。
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
            self.cmd_time = 0.0
            self.note("急停复位（目标仍为 0，需重新推摇杆）")

    def reset_odom(self) -> None:
        with self.lock:
            self.distance = 0.0
            self.yaw = 0.0
            self._odom_last = None
            self.note("里程清零")

    # -- 安全核心 -----------------------------------------------------------
    def _degrade_scale_locked(self, age_ms: float) -> tuple[float, bool]:
        """心跳年龄 → 目标缩放。返回 (scale, 是否处于分级衰减)。

        300 ms 起 0.70 → 0.50 → 0.30，500 ms 由 _target_locked 直接归零。
        只用于"链路抖动"的柔性降速，不改变任何立即停的路径。
        """
        for threshold, scale in reversed(CMD_DEGRADE_STEPS):
            if age_ms >= threshold:
                return scale, True
        return 1.0, False

    def _target_locked(self, now: float) -> tuple[float, float, str, bool]:
        """算目标值（已过安全状态机 + 档位/硬上限夹取 + 心跳分级衰减），不推进斜坡。

        返回 (vx, wz, 原因, 是否被夹取)。原因用于界面显示"为什么车不动"。
        调用者必须已持有 self.lock（RLock 重入也行）。
        """
        if self.estop:
            return 0.0, 0.0, "estop", False
        if self.cmd_time <= 0.0:
            return 0.0, 0.0, "timeout", False
        age_ms = (now - self.cmd_time) * 1000.0
        if age_ms > CMD_TIMEOUT_MS:
            # 硬上限：500 ms 到 0，绝不因为"分级衰减"延长到固件 800 ms 附近。
            return 0.0, 0.0, "timeout", False
        scale, degraded = self._degrade_scale_locked(age_ms)
        g = GEARS[self.gear]
        vx_max = min(g["vx"], HARD_VX_LIMIT)
        wz_max = min(g["wz"], HARD_WZ_LIMIT)
        # 先按档位/硬上限夹"网页原始目标"，再乘衰减系数：
        # clamped 只表示"被夹取"，不能把 70% 衰减也算成夹取。
        vx = clamp(self.cmd_vx, -vx_max, vx_max)
        wz = clamp(self.cmd_wz, -wz_max, wz_max)
        clamped = (abs(vx - self.cmd_vx) > 1e-9) or (abs(wz - self.cmd_wz) > 1e-9)
        if degraded:
            vx *= scale
            wz *= scale
        return vx, wz, ("stale" if degraded else "web"), clamped

    def effective(self, now: float) -> tuple[float, float, str, bool]:
        """目标值（不含斜坡）。保留这个名字给状态/诊断用；**不要**在这里积分。

        真正的串口输出由 LinkWorker 调 step_output() 推进，保证只有串口
        线程按固定节拍推进斜坡，20 Hz 的 HTTP/WS 状态读取不会改变控制量。
        """
        with self.lock:
            return self._target_locked(now)

    def step_output(self, now: float) -> tuple[float, float, str, bool]:
        """推进一拍斜坡，返回这一拍真正写串口的 (vx, wz, 原因, 夹取)。

        ★ 停车分两类（这是"松手能感觉到软停、急停绝不拖泥带水"的关键）：

        A. 立即归零（安全停车，不积分、不保留残余）：
           1. 急停锁存（reason=estop）；
           2. 网页心跳超时（reason=timeout，500 ms 没收到指令）；
           3. 前端断连（drop_owner 会把 cmd_time 清 0，等同于 timeout）；
           4. 页面隐藏 / 窗口失焦 / pagehide：前端带 hard:true 发 0，
              由 self.hard_stop 标记。
           前三条是"可能没人看着车了"，必须立刻停；第 4 条是"用户已经
           看不到车/页面失焦"，同样按安全停车处理。

        B. 正常停车走缓停（只受 ramp_on 控制）：
           松手、摇杆回中、键盘松开/空格、主动点"回中/停止"、/api/stop
           都只是把目标置 0，没有 hard 标记；开了斜坡就按 RAMP_*_DEC
           逐拍减速到 0。斜坡关闭时仍保持旧行为：下一拍直接置 0。

        注意：软停期间必须继续有网页心跳（页面在后台仍按 30 Hz 发 0，
        或脚本持续 POST /api/stop）；一旦心跳断了，先按 CMD_DEGRADE_STEPS
        柔性降速，500 ms 后按 A.2 立即归零。
        """
        with self.lock:
            tvx, twz, reason, clamped = self._target_locked(now)
            was_moving = abs(self.out_vx) > 1e-6 or abs(self.out_wz) > 1e-6

            if self._ramp_t <= 0.0:
                dt = 0.0
            else:
                dt = clamp(now - self._ramp_t, 0.0, RAMP_MAX_DT)
            self._ramp_t = now

            hard_stop = (reason in ("estop", "timeout") or self.hard_stop)
            if hard_stop:
                # 立即归零：不积分、不保留残余，下一拍重新从 0 开始。
                self.out_vx = 0.0
                self.out_wz = 0.0
            elif not self.ramp_on:
                # 关闭时保持旧行为：直接把目标值发给串口（含目标 0 → 立即停）。
                self.out_vx = tvx
                self.out_wz = twz
            else:
                # 加速用 ACC，减速/回零/反向用 DEC（见 slew_axis）。
                self.out_vx = slew_axis(self.out_vx, tvx, dt,
                                        RAMP_VX_ACC, RAMP_VX_DEC)
                self.out_wz = slew_axis(self.out_wz, twz, dt,
                                        RAMP_WZ_ACC, RAMP_WZ_DEC)

            # ---- 诊断（1.4.0）：每拍归一个原因，供 /status 看分布 ----
            # 注意分类顺序：estop > timeout > hard（前端 hard:true）> stale > web。
            if reason == "estop":
                cat = "estop"
            elif reason == "timeout":
                cat = "timeout"
            elif self.hard_stop:
                cat = "hard"
            elif reason == "stale":
                cat = "stale"
            else:
                cat = "web"
            self.reason_frames.append((now, cat))
            self.reason_total[cat] = self.reason_total.get(cat, 0) + 1
            # ★ 超时清零"次数"：只在刚跨进 timeout 且此前输出确实非 0 时 +1。
            # 这样停在原地时不会一直涨；手机操作时它涨 = 心跳真的断了。
            if cat == "timeout" and self._prev_reason != "timeout" and was_moving:
                self.timeout_zero_count += 1
                self.last_timeout_s = round(now - self.started, 1)
            self._prev_reason = cat

            self.ramp_step_count += 1
            return self.out_vx, self.out_wz, reason, clamped

    # -- 串口线程回调 -------------------------------------------------------
    def set_link(self, open_: bool, error: str = "") -> None:
        with self.lock:
            changed = (self.link_open != open_) or (error != self.link_error)
            self.link_open = open_
            self.link_error = error
            if open_:
                self.link_error = ""
            if changed and self._prev_link is not None:
                if open_ and self.sim:
                    self.note("仿真链路已就绪（不占用任何串口）")
                elif open_:
                    self.note("串口已打开：%s @ %d" % (self.link_port, self.args.baud))
                else:
                    self.note("串口断开：%s" % (error or "未知原因"))
            self._prev_link = open_

    def on_tx(self, vx: float, wz: float, now: float) -> None:
        with self.lock:
            self.tx_count += 1
            self.tx_vx = vx
            self.tx_wz = wz
            self.tx_time = now

    def on_frames(self, frames: list[tuple[str, bytes]], now: float) -> None:
        with self.lock:
            for kind, f in frames:
                if kind == "tel":
                    self.tel = decode_telemetry(f)
                    self.tel_time = now
                    self.tel_count += 1
                else:
                    self.status = decode_status(f)
                    self.status_time = now
                    self.status_count += 1
            # 里程积分：只在遥测帧到达时推进，dt 取真实时间差并夹到 0.2 s，
            # 防止串口卡顿后一次性把距离跳一大段。
            if self.tel is not None and self.tel_time == now:
                if self._odom_last is not None:
                    last_t, last_d, last_y = self._odom_last
                    dt = clamp(now - last_t, 0.0, 0.2)
                    if dt > 0:
                        self.distance += self.tel["vx"] * dt
                        self.yaw += self.tel["wz"] * dt
                self._odom_last = (now, self.distance, self.yaw)

            # 边沿事件：只在状态变化时记一条，避免 20 Hz 刷屏
            st = self.status
            if st is not None:
                timeout = st["flags"]["cmd_timeout"]
                if timeout != self._prev_timeout:
                    if timeout and self.cmd_time > 0:
                        self.note("STM32 报告命令超时（800 ms 没收到合法帧）")
                    self._prev_timeout = timeout
                fault = st["left"]["fault"] or st["right"]["fault"]
                if fault != self._prev_fault:
                    if fault:
                        self.note("电机故障：左=%s 右=%s"
                                  % (st["left"]["err_text"], st["right"]["err_text"]))
                    else:
                        self.note("电机故障已清除")
                    self._prev_fault = fault

    # -- 状态快照 -----------------------------------------------------------
    def _arrival_stats_locked(self, now: float) -> dict[str, Any]:
        """最近 2 s 内 /cmd（HTTP 或 WS）的到达间隔统计。

        手机 WiFi 抖动最直接的证据：间隔 min/avg/max 和最大抖动。
        只统计"到达"，不区分是否为旧序号包 —— 旧包也的确到了。
        """
        horizon = now - CMD_DIAG_WINDOW_MS / 1000.0
        while self.cmd_arrivals and self.cmd_arrivals[0] < horizon:
            self.cmd_arrivals.popleft()
        arr = list(self.cmd_arrivals)
        out: dict[str, Any] = {
            "n": len(arr), "window_ms": CMD_DIAG_WINDOW_MS,
            "min_ms": None, "avg_ms": None, "max_ms": None, "jitter_ms": None,
        }
        if len(arr) >= 2:
            gaps = [(arr[i] - arr[i - 1]) * 1000.0 for i in range(1, len(arr))]
            avg = sum(gaps) / len(gaps)
            out.update({
                "min_ms": round(min(gaps), 1),
                "avg_ms": round(avg, 1),
                "max_ms": round(max(gaps), 1),
                "jitter_ms": round(max(abs(g - avg) for g in gaps), 1),
            })
        return out

    def _reason_stats_locked(self, now: float) -> dict[str, Any]:
        """最近 2 s 内 step_output 每一拍的原因分布（web/stale/timeout/hard/estop）。"""
        horizon = now - CMD_DIAG_WINDOW_MS / 1000.0
        while self.reason_frames and self.reason_frames[0][0] < horizon:
            self.reason_frames.popleft()
        counts = {"web": 0, "stale": 0, "timeout": 0, "hard": 0, "estop": 0}
        for _t, cat in self.reason_frames:
            if cat in counts:
                counts[cat] += 1
        counts["frames"] = len(self.reason_frames)
        counts["window_ms"] = CMD_DIAG_WINDOW_MS
        return counts

    def _cmd_age_ms(self, now: float) -> Optional[float]:
        if self.cmd_time <= 0.0:
            return None
        return max(0.0, (now - self.cmd_time) * 1000.0)

    def snapshot(self, now: Optional[float] = None) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        with self.lock:
            g = GEARS[self.gear]
            target_vx, target_wz, reason, clamped = self._target_locked(now)
            age_ms = self._cmd_age_ms(now)
            degrade_scale = 1.0
            if reason == "timeout":
                degrade_scale = 0.0        # 已归零：显示 0%，不是 100%
            elif reason == "stale" and age_ms is not None:
                degrade_scale, _ = self._degrade_scale_locked(age_ms)
            # 立即停的语义在显示上也立即生效（即使 LinkWorker 这一拍还没跑到），
            # 但不能把"软停中"也显示成 0，否则界面看不出缓停。只有
            # estop / timeout / hard_stop 这三类才强制显示 0。
            hard_stop = (reason in ("estop", "timeout") or self.hard_stop)
            out_vx = 0.0 if hard_stop else self.out_vx
            out_wz = 0.0 if hard_stop else self.out_wz
            ramp_active = (self.ramp_on and not hard_stop
                           and (abs(out_vx - target_vx) > 1e-6
                                or abs(out_wz - target_wz) > 1e-6))
            # 正在减速 = 斜坡中且目标幅值小于当前输出幅值（含目标 0 的软停）
            decelerating = (ramp_active
                            and (abs(target_vx) < abs(out_vx) - 1e-9
                                 or abs(target_wz) < abs(out_wz) - 1e-9))
            if hard_stop:
                ramp_mode = "hard"
            elif not self.ramp_on:
                ramp_mode = "off"
            elif decelerating:
                ramp_mode = "decel"
            elif ramp_active:
                ramp_mode = "accel"
            else:
                ramp_mode = "idle"
            tel = dict(self.tel) if self.tel else None
            st = dict(self.status) if self.status else None
            if st:
                # 浅拷贝够用：渲染只读，不改嵌套结构
                st = json.loads(json.dumps(st))

            flags = []
            if st:
                for key, label, good_on, _bit in FLAG_BITS:
                    on = bool(st["flags"].get(key))
                    flags.append({
                        "key": key, "label": label, "on": on,
                        # good_on=True 表示"这一位为 1 是好事"，界面据此上色
                        "good": (on == good_on),
                    })

            # IMU 派生量：全部由 24 字节帧的 ax/ay/az/gx/gy/gz 算出，
            # 不是新增传感器字段。静止时 accel_mag 应≈9.81 m/s²，明显偏离
            # 说明有振动/碰撞/模块没装平（或 az 被 ±2g 量程夹住）。
            imu = None
            if tel:
                ax, ay, az = tel["ax"], tel["ay"], tel["az"]
                gx, gy, gz = tel["gx"], tel["gy"], tel["gz"]
                accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
                imu = {
                    "accel_mag": round(accel_mag, 3),
                    "gyro_mag": round(math.sqrt(gx * gx + gy * gy + gz * gz), 4),
                    "tilt_deg": round(math.degrees(math.atan2(
                        math.hypot(ax, ay), abs(az) or 1e-9)), 2),
                }
                if st:
                    imu.update({
                        "status": st["imu_status"],
                        "status_text": st["imu_status_text"],
                        "addr": st["imu_addr"],
                        "version_major": st["imu_ver"],
                        "probe": {
                            "version": st["imu_probe_version"],
                            "gyro": st["imu_probe_gyro"],
                            "mag": st["imu_probe_mag"],
                            "gyro_ever_ok": st["imu_gyro_ok"],
                        },
                    })

            return {
                "ok": True,
                "version": VERSION,
                "ts": round(now - self.started, 3),
                "sim": self.sim,
                "link": {
                    "open": self.link_open,
                    "port": self.link_port,
                    "baud": self.args.baud,
                    "error": self.link_error,
                    "tx_hz": self.args.tx_hz,
                    "tx_count": self.tx_count,
                    "tx_vx": round(self.tx_vx, 3),
                    "tx_wz": round(self.tx_wz, 3),
                    "tx_age_ms": round((now - self.tx_time) * 1000.0, 0) if self.tx_time else None,
                    "telemetry_count": self.tel_count,
                    "status_count": self.status_count,
                    "bad_frames": self.bad_frames,
                    "resync_bytes": self.resync_bytes,
                    "rx_age_ms": round((now - self.tel_time) * 1000.0, 0) if self.tel_time else None,
                    "status_age_ms": round((now - self.status_time) * 1000.0, 0) if self.status_time else None,
                },
                "cmd": {
                    "vx": round(self.cmd_vx, 3),
                    "wz": round(self.cmd_wz, 3),
                    # tx_* = 真正写串口的值（斜坡后的输出），不是网页目标。
                    "tx_vx": round(out_vx, 3),
                    "tx_wz": round(out_wz, 3),
                    "target_vx": round(target_vx, 3),
                    "target_wz": round(target_wz, 3),
                    "reason": reason,
                    "clamped": clamped,
                    "age_ms": round(age_ms or 0.0, 0),
                    "has_client": age_ms is not None and age_ms <= CMD_TIMEOUT_MS,
                    "recv_count": self.cmd_recv_count,
                    # ---- 1.4.0 指令链路诊断 ----
                    "seq": self.cmd_seq,
                    "session": self.cmd_session,
                    "client_ts": self.cmd_last_client_ts,
                    # degraded=true 时 target_* 已经是衰减后的值，scale 是系数
                    "degraded": reason == "stale",
                    "degrade_scale": round(degrade_scale, 2),
                    "stale_packets": self.cmd_stale_packets,
                    "seq_gaps": self.cmd_seq_gaps,
                    "session_resets": self.cmd_session_resets,
                },
                "jitter": {
                    # 最近 2 s 的到达间隔（前端指令 → 本进程）
                    "arrival": self._arrival_stats_locked(now),
                    # 最近 2 s 每一拍输出按原因分类（30 Hz × 2 s ≈ 60 帧）
                    "reason": self._reason_stats_locked(now),
                    "window_ms": CMD_DIAG_WINDOW_MS,
                    # ★ 因心跳超时被清零的次数：手机操作时它涨 = 网络/心跳问题
                    "timeout_zero_count": self.timeout_zero_count,
                    "timeout_zero_last_s": self.last_timeout_s,
                    "stale_frames_total": self.reason_total.get("stale", 0),
                    "reason_total": dict(self.reason_total),
                    "stale_packets": self.cmd_stale_packets,
                    "seq_gaps": self.cmd_seq_gaps,
                    "session_resets": self.cmd_session_resets,
                    "degrade_steps": [
                        {"after_ms": int(ms), "scale": scale}
                        for ms, scale in CMD_DEGRADE_STEPS
                    ],
                    "hard_timeout_ms": CMD_TIMEOUT_MS,
                },
                "ramp": {
                    "on": self.ramp_on,
                    "default_on": RAMP_DEFAULT_ON,
                    "vx_acc": RAMP_VX_ACC,
                    "vx_dec": RAMP_VX_DEC,
                    "wz_acc": RAMP_WZ_ACC,
                    "wz_dec": RAMP_WZ_DEC,
                    "out_vx": round(out_vx, 3),
                    "out_wz": round(out_wz, 3),
                    "target_vx": round(target_vx, 3),
                    "target_wz": round(target_wz, 3),
                    "active": ramp_active,
                    # mode: accel / decel（软停中）/ hard（立即停）/ off / idle
                    "mode": ramp_mode,
                    "stopping": decelerating,
                    "hard_stop": hard_stop,
                    "steps": self.ramp_step_count,
                },
                "safety": {
                    "estop": self.estop,
                    "timeout_ms": CMD_TIMEOUT_MS,
                    "vx_limit": HARD_VX_LIMIT,
                    "wz_limit": HARD_WZ_LIMIT,
                    # 固件单轮 30 rad/s × 0.0855 m = 2.565 m/s，硬上限必须低于它
                    "firmware_vx_max": round(FIRMWARE_VX_MAX, 3),
                },
                "gear": {
                    "key": self.gear,
                    "name": g["name"],
                    "vx": g["vx"],
                    "wz": g["wz"],
                },
                "telemetry": tel,
                "status": st,
                # IMU 派生量（模长/倾角/体检位）；没有遥测帧时为 None
                "imu": imu,
                "flags": flags,
                "odom": {
                    "distance": round(self.distance, 3),
                    "yaw": round(self.yaw, 4),
                },
                "ws_clients": len(self.ws_clients),
                "events": list(self.events)[-8:],
            }

    # -- 客户端登记 ---------------------------------------------------------
    def register_ws(self, client: Any) -> int:
        with self.lock:
            cid = self._next_client_id
            self._next_client_id += 1
            self.ws_clients[cid] = client
            self.ws_connect_count += 1
            self.note("网页已连接（%d 个在线）" % len(self.ws_clients))
            return cid

    def unregister_ws(self, cid: int) -> None:
        with self.lock:
            if cid in self.ws_clients:
                del self.ws_clients[cid]
                self.note("网页断开（剩 %d 个）" % len(self.ws_clients))
        self.drop_owner(cid)


# ---------------------------------------------------------------------------
# 3. 链路：真实串口 / 内部仿真
# ---------------------------------------------------------------------------


class SerialLink:
    """真实串口。独占打开（POSIX 上 TIOCEXCL），避免别的进程同时读。"""

    def __init__(self, port: str, baud: int) -> None:
        import serial  # 延迟导入：--sim 模式下不需要 pyserial

        kwargs = dict(port=port, baudrate=baud, bytesize=8, parity="N",
                      stopbits=1, timeout=0, write_timeout=0.2,
                      xonxoff=False, rtscts=False, dsrdtr=False)
        try:
            # exclusive=True 是硬性要求：/dev/wheeltec_controller 只能有一个读者，
            # 否则两边的 read 会互相抢字节，帧全废。
            self.ser = serial.Serial(exclusive=True, **kwargs)
        except TypeError:
            # 极老的 pyserial 没有 exclusive 参数，退化成普通打开
            self.ser = serial.Serial(**kwargs)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def read_available(self) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


class SimChassis:
    """内部仿真：**产生和固件逐字节同格式的 24/44 字节帧**，并解析命令帧。

    为什么连帧都照真格式造：
        这样 sim 模式跑的是同一条 解析 → 解码 → 状态机 路径，
        帧同步、BCC、缩放系数在 Windows 上就能验证，真机上只剩"线有没有接对"。
    """

    def __init__(self) -> None:
        self.start = time.monotonic()
        self.closed = False
        self.last_cmd_t = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.cmd_buf = bytearray()
        self.cmd_bad = 0

        # 轮子状态（电机原始 ω，方向系数按车型分：DM_DIR_LEFT / DM_DIR_RIGHT，
        # 扫地车左轮 +1、右轮 -1，洗地车两个都 -1；见 docs/vehicle-params.md §4.1）
        self.omega = [0.0, 0.0]
        self.pos = [0.0, 0.0]
        self.torque = [0.0, 0.0]
        self.temp = [31.0, 31.0]
        self.tel_count = 0
        self.st_count = 0
        self._next_frame = self.start
        self._last_step = self.start
        self._yaw = 0.0
        self._rng = 12345

        # 固件里的一份"最近一次收到的指令"
        self._st_cmd_vx_mm = 0
        self._st_cmd_wz_mrad = 0
        self._cmd_timeout = True
        self._eff_vx = 0.0
        self._eff_wz = 0.0

    # -- 随机数：不用 random 模块是为了让 sim 完全可复现 ----------------
    def _noise(self, scale: float) -> float:
        self._rng = (1103515245 * self._rng + 12345) & 0x7FFFFFFF
        return ((self._rng / 0x7FFFFFFF) - 0.5) * 2.0 * scale

    # -- 命令解析：逐字节左移重试，和固件 Cmd_FeedByte 等价 -------------
    def write(self, data: bytes) -> None:
        for b in data:
            if len(self.cmd_buf) >= CMD_SIZE:
                del self.cmd_buf[0]
            self.cmd_buf.append(b)
            if len(self.cmd_buf) == CMD_SIZE:
                f = bytes(self.cmd_buf)
                if f[0] == CMD_HEAD and f[10] == CMD_TAIL and xor_bcc(f[:9]) == f[9]:
                    vx_mm = struct.unpack_from(">h", f, 3)[0]
                    wz_mrad = struct.unpack_from(">h", f, 7)[0]
                    self.last_cmd_t = time.monotonic()
                    self.cmd_vx = vx_mm / 1000.0
                    self.cmd_wz = wz_mrad / 1000.0
                    self._st_cmd_vx_mm = vx_mm
                    self._st_cmd_wz_mrad = wz_mrad
                    self.cmd_buf.clear()
                else:
                    self.cmd_bad += 1      # 丢最老字节，下一个字节进来继续对

    # -- 读：按时间生成帧 ---------------------------------------------------
    def read_available(self) -> bytes:
        now = time.monotonic()
        self._step(now)
        if now < self._next_frame:
            return b""
        self._next_frame = now + 0.05           # 20 Hz，和固件一致
        self.tel_count += 1
        self.st_count += 1
        # 顺序不能反：先 24 字节（0x7B）再 44 字节（0x7E），
        # 这样 0x7B 前面永远是上一个 0x7D，接收端才能按"帧尾+帧头"同步。
        return self._tel_frame(now) + self._status_frame(now)

    def close(self) -> None:
        # 仿真链路没有真实句柄，留一个空实现让 LinkWorker 的关闭路径统一
        self.closed = True

    def _step(self, now: float) -> None:
        dt = clamp(now - self._last_step, 0.0, 0.1)
        self._last_step = now
        if dt <= 0:
            return
        self._cmd_timeout = (self.last_cmd_t <= 0.0
                             or (now - self.last_cmd_t) * 1000.0 > 800.0)
        if self._cmd_timeout:
            self._eff_vx = 0.0
            self._eff_wz = 0.0
        else:
            self._eff_vx = self.cmd_vx
            self._eff_wz = self.cmd_wz

        # 逆解 → 电机目标（仅 --sim 用；方向系数沿洗地车 DM_DIR_LEFT=DM_DIR_RIGHT=-1，
        # 只是让仿真里的 ω 符号看起来一致，真机由 STM32 按车型乘，见 vehicle-params.md §4.1）
        v_left = self._eff_vx - self._eff_wz * TRACK_WIDTH_M / 2.0
        v_right = self._eff_vx + self._eff_wz * TRACK_WIDTH_M / 2.0
        target = [-v_left / WHEEL_R_M, -v_right / WHEEL_R_M]

        for i in range(2):
            # 一阶滞后：真的直流电机也不会瞬间到目标
            err = target[i] - self.omega[i]
            self.omega[i] += err * min(1.0, dt / 0.18)
            self.omega[i] = clamp(self.omega[i], -WHEEL_OMEGA_MAX, WHEEL_OMEGA_MAX)
            self.pos[i] += self.omega[i] * dt
            # 扭矩 ≈ 加速 + 摩擦（只为让读数看起来合理，不是物理模型）
            self.torque[i] = 0.08 * err + 0.015 * self.omega[i] + self._noise(0.02)
            self.temp[i] += (abs(self.omega[i]) * 0.02 - (self.temp[i] - 30.0) * 0.02) * dt

        self._yaw += self._eff_wz * dt

    def _tel_frame(self, now: float) -> bytes:
        v_left = -self.omega[0] * WHEEL_R_M
        v_right = -self.omega[1] * WHEEL_R_M
        vx_mm = int(round((v_left + v_right) * 0.5 * 1000.0))
        wz_mrad = int(round((v_right - v_left) / TRACK_WIDTH_M * 1000.0))

        # 车体加速度：转弯时的向心加速度 + 噪声，az 是重力
        ax = self._eff_vx * self._eff_wz + self._noise(0.06)
        ay = self._noise(0.05)
        az = 9.80665 + self._noise(0.04)
        gx = self._noise(0.006)
        gy = self._noise(0.006)
        gz = self._eff_wz * 0.95 + self._noise(0.004)   # 陀螺主要就是偏航角速度

        f = bytearray(TEL_SIZE)
        f[0] = TEL_HEAD
        f[1] = 1 if (self._eff_vx == 0.0 and self._eff_wz == 0.0) else 0
        struct.pack_into(">h", f, 2, vx_mm)
        struct.pack_into(">h", f, 4, 0)
        struct.pack_into(">h", f, 6, wz_mrad)
        struct.pack_into(">h", f, 8, int(ax * ACC_LSB))
        struct.pack_into(">h", f, 10, int(ay * ACC_LSB))
        struct.pack_into(">h", f, 12, int(az * ACC_LSB))
        struct.pack_into(">h", f, 14, int(gx / GYRO_LSB))
        struct.pack_into(">h", f, 16, int(gy / GYRO_LSB))
        struct.pack_into(">h", f, 18, int(gz / GYRO_LSB))
        struct.pack_into(">H", f, 20, 0)               # 新底盘没有电池采样
        f[22] = xor_bcc(bytes(f[:22]))
        f[23] = TEL_TAIL
        return bytes(f)

    def _status_frame(self, now: float) -> bytes:
        flags = 0
        # 仿真里两个轮子一直在线、使能；IMU 有效
        flags |= 0x01 | 0x02 | 0x04 | 0x08 | 0x80
        if self._cmd_timeout:
            flags |= 0x10
        f = bytearray(ST_SIZE)
        f[0] = ST_HEAD
        f[1] = flags
        struct.pack_into(">h", f, 2, self._st_cmd_vx_mm)
        struct.pack_into(">h", f, 4, self._st_cmd_wz_mrad)
        struct.pack_into(">h", f, 6, int(self.omega[0] * 100))
        struct.pack_into(">h", f, 8, int(self.omega[1] * 100))
        # 目标 ω：固件里是逆解+限幅后的最终值
        v_left = self._eff_vx - self._eff_wz * TRACK_WIDTH_M / 2.0
        v_right = self._eff_vx + self._eff_wz * TRACK_WIDTH_M / 2.0
        struct.pack_into(">h", f, 10, int(clamp(-v_left / WHEEL_R_M * 100, -32768, 32767)))
        struct.pack_into(">h", f, 12, int(clamp(-v_right / WHEEL_R_M * 100, -32768, 32767)))
        struct.pack_into(">h", f, 14, int(self.torque[0] * 100))
        struct.pack_into(">h", f, 16, int(self.torque[1] * 100))
        f[18] = 1                      # ERR=1 使能
        f[19] = 1
        f[20] = int(self.temp[0])
        f[21] = int(self.temp[1])
        f[22] = int(self.temp[0] + 2)
        f[23] = int(self.temp[1] + 2)
        struct.pack_into(">h", f, 24, int(clamp(self.pos[0] * 100, -32768, 32767)))
        struct.pack_into(">h", f, 26, int(clamp(self.pos[1] * 100, -32768, 32767)))
        f[28] = 0                      # canFail
        f[29] = self.st_count & 0xFF
        f[30] = 1                      # 版本号
        f[31] = 0                      # IMU 诊断正常
        struct.pack_into(">H", f, 32, 20)   # 反馈年龄 20 ms
        struct.pack_into(">H", f, 34, 20)
        struct.pack_into(">H", f, 36, int((now - self.start) * 1000) & 0xFFFF)
        f[38] = 0x6A                   # 扫到的 IMU I2C 地址
        f[39] = 0x87                   # bit0/1/2 体检通过 + bit7 陀螺读到过
        f[40] = 1
        f[41] = 1                      # 启动阶段：正常
        f[42] = xor_bcc(bytes(f[:42]))
        f[43] = ST_TAIL
        return bytes(f)


class LinkWorker(threading.Thread):
    """唯一一个碰链路的线程：读帧 + 定时发命令帧。

    为什么不用两个线程（一个读一个写）：
        串口对象不是线程安全的；而且 pyserial 在 close 时会把阻塞中的
        read 变成异常，双线程下很容易出现"写线程拿着已关闭的句柄"。
        一个线程顺序做 read → 组帧 → write，代码短、状态少。
    """

    def __init__(self, hub: CarHub, args: argparse.Namespace) -> None:
        super().__init__(name="link", daemon=True)
        self.hub = hub
        self.args = args
        self.link: Any = None
        self.parser = FrameParser()
        self._stop_evt = threading.Event()
        self._next_open = 0.0

    def stop(self) -> None:
        self._stop_evt.set()

    # -- 打开链路（失败不退出，1 秒后重试）--------------------------------
    def _try_open(self, now: float) -> None:
        if now < self._next_open:
            return
        self._next_open = now + 1.0
        try:
            if self.args.sim:
                self.link = SimChassis()
                self.hub.set_link(True)
                print("[link] 仿真链路已启动（20 Hz 产生 24/44 字节帧）", flush=True)
            else:
                if not os.path.exists(self.args.serial):
                    raise FileNotFoundError(
                        "找不到 %s（确认 udev 符号链接已建立）" % self.args.serial)
                self.link = SerialLink(self.args.serial, self.args.baud)
                self.hub.set_link(True)
                print("[link] 串口已打开：%s @ %d（独占）"
                      % (self.args.serial, self.args.baud), flush=True)
            self.parser = FrameParser()
        except Exception as exc:                      # noqa: BLE001 - 现场要看原因
            self.link = None
            self.hub.set_link(False, str(exc))
            print("[link] 打开失败：%s（1 秒后重试）" % exc, flush=True)

    def _close(self, why: str) -> None:
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        self.link = None
        self.hub.set_link(False, why)
        self._next_open = time.monotonic() + 1.0
        print("[link] 关闭：%s（1 秒后重开）" % why, flush=True)

    def run(self) -> None:
        period = 1.0 / self.args.tx_hz
        next_tx = time.monotonic()
        last_stats = time.monotonic()
        try:
            while not self._stop_evt.is_set():
                now = time.monotonic()
                if self.link is None:
                    self._try_open(now)
                    self._stop_evt.wait(0.2)
                    continue

                # ---- 读（非阻塞）----
                try:
                    data = self.link.read_available()
                    if data:
                        frames = self.parser.feed(data)
                        if frames:
                            self.hub.on_frames(frames, time.monotonic())
                        if self.parser.bad or self.parser.resync:
                            with self.hub.lock:
                                self.hub.bad_frames += self.parser.bad
                                self.hub.resync_bytes += self.parser.resync
                            self.parser.bad = 0
                            self.parser.resync = 0
                except Exception as exc:              # noqa: BLE001
                    self._close("读失败：%s" % exc)
                    continue

                # ---- 定时发命令帧（20~50 Hz）----
                now = time.monotonic()
                if now >= next_tx:
                    # step_output 是唯一推进斜坡的地方：急停/超时/硬停立即归零，
                    # 松手/回中按 DEC 软停。只有串口线程能按固定节拍积分。
                    vx, wz, _reason, _clamped = self.hub.step_output(now)
                    try:
                        self.link.write(build_cmd_frame(vx, wz))
                        self.hub.on_tx(vx, wz, now)
                    except Exception as exc:          # noqa: BLE001
                        self._close("写失败：%s" % exc)
                        continue
                    # 用固定周期推进，而不是 now+period，避免长期漂移；
                    # 如果已经落后超过一个周期就直接对齐到现在。
                    next_tx += period
                    if next_tx < now:
                        next_tx = now + period

                if self.args.verbose and now - last_stats >= 1.0:
                    last_stats = now
                    snap = self.hub.snapshot(now)
                    print("[stat] tx=%d tel=%d st=%d bad=%d tx=(%.3f,%.3f) %s"
                          % (snap["link"]["tx_count"], snap["link"]["telemetry_count"],
                             snap["link"]["status_count"], snap["link"]["bad_frames"],
                             snap["cmd"]["tx_vx"], snap["cmd"]["tx_wz"],
                             snap["cmd"]["reason"]), flush=True)

                self._stop_evt.wait(0.004)
        finally:
            if self.link is not None:
                try:
                    self.link.write(build_cmd_frame(0.0, 0.0))
                except Exception:
                    pass
                self.link.close()


# ---------------------------------------------------------------------------
# 4. 极简 WebSocket（RFC 6455）：只做"收文本 JSON / 推文本 JSON"
# ---------------------------------------------------------------------------

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def ws_accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()


class WsIf:
    """一条 WebSocket 连接：发送带锁（广播线程可能并发），读只在自己的线程。"""

    def __init__(self, cid: int) -> None:
        self.cid = cid
        self.sock: Optional[socket.socket] = None
        self.send_lock = threading.Lock()
        self.closed = False

    def attach(self, sock: socket.socket) -> None:
        self.sock = sock

    def send_text(self, text: str) -> None:
        data = text.encode("utf-8")
        n = len(data)
        if n < 126:
            header = bytes((0x81, n))
        elif n < 65536:
            header = bytes((0x81, 126)) + struct.pack(">H", n)
        else:
            header = bytes((0x81, 127)) + struct.pack(">Q", n)
        with self.send_lock:
            if self.closed or self.sock is None:
                raise ConnectionError("ws closed")
            self.sock.sendall(header + data)

    def send_control(self, opcode: int, payload: bytes = b"") -> None:
        with self.send_lock:
            if self.closed or self.sock is None:
                raise ConnectionError("ws closed")
            self.sock.sendall(bytes((0x80 | opcode, len(payload))) + payload)

    def close(self) -> None:
        with self.send_lock:
            if self.closed:
                return
            self.closed = True
            try:
                if self.sock is not None:
                    self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                if self.sock is not None:
                    self.sock.close()
            except OSError:
                pass


class WsReader:
    """按需从 socket 读一个完整的客户端帧（客户端帧一定带 4 字节掩码）。"""

    MAX_PAYLOAD = 1 << 20

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buf = bytearray()

    def _need(self, n: int) -> bytes:
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("peer closed")
            self.buf.extend(chunk)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def read_frame(self) -> tuple[bool, int, bytes]:
        b1, b2 = self._need(2)
        fin = bool(b1 & 0x80)
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._need(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._need(8))[0]
        if length > self.MAX_PAYLOAD:
            raise ConnectionError("frame too large")
        mask = self._need(4) if masked else b"\x00\x00\x00\x00"
        payload = self._need(length) if length else b""
        if masked:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        return fin, opcode, payload


# ---------------------------------------------------------------------------
# 5. HTTP 服务
# ---------------------------------------------------------------------------

STATIC_DIR = Path(__file__).resolve().parent / "static"

MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


class Handler(BaseHTTPRequestHandler):
    server_version = "CarWeb/" + VERSION
    protocol_version = "HTTP/1.1"

    hub: CarHub = None          # 由 make_server() 注入
    args: argparse.Namespace = None

    # 默认的访问日志会把每个 20 Hz 请求都打到终端，现场看日志反而更难。
    def log_message(self, fmt: str, *a: Any) -> None:
        if getattr(self.args, "verbose", False):
            sys.stderr.write("[http] " + (fmt % a) + "\n")

    # -- 工具 ---------------------------------------------------------------
    def _send(self, code: int, body: bytes, ctype: str,
              extra: Optional[dict[str, str]] = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj: Any, code: int = 200) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False,
                                     separators=(",", ":")).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _body(self) -> dict[str, Any]:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        raw = self.rfile.read(n) if n > 0 else b""
        if not raw:
            return {}
        try:
            obj = json.loads(raw.decode("utf-8", "replace"))
            return obj if isinstance(obj, dict) else {}
        except (ValueError, UnicodeDecodeError):
            return {}

    def _client_ip(self) -> str:
        return self.client_address[0] if self.client_address else "-"

    # -- 路由 ---------------------------------------------------------------
    def do_GET(self) -> None:                       # noqa: N802
        path = urlparse(self.path).path
        if path == "/ws":
            self._handle_ws()
            return
        if path in ("/", "/index.html"):
            self._serve_static("index.html")
            return
        if path.startswith("/static/"):
            self._serve_static(unquote(path[len("/static/"):]))
            return
        if path == "/api/state":
            self._json(self.hub.snapshot())
            return
        if path == "/api/ramp":
            self._json({"ok": True, "ramp": self.hub.snapshot()["ramp"]})
            return
        if path == "/status":
            self._send(200, self._status_text().encode("utf-8"),
                       "text/plain; charset=utf-8")
            return
        if path == "/favicon.ico":
            self._send(204, b"", "image/x-icon")
            return
        self._json({"ok": False, "error": "not found: %s" % path}, 404)

    def do_POST(self) -> None:                      # noqa: N802
        path = urlparse(self.path).path
        body = self._body()
        if path in ("/api/cmd", "/api/stop"):
            try:
                vx = float(body.get("vx", 0.0) or 0.0)
                wz = float(body.get("wz", 0.0) or 0.0)
            except (TypeError, ValueError):
                self._json({"ok": False, "error": "vx/wz 必须是数字"}, 400)
                return
            if path == "/api/stop":
                vx = wz = 0.0
            # hard=true 表示安全停车（页面隐藏/失焦/关页），立即归零；
            # 不打 hard 的 0 是松手/回中/主动停止，走缓停斜坡。
            hard = parse_onoff(body.get("hard")) is True
            # 1.4.0：可选的 seq/session/ts 用于乱序/旧包诊断；不带也能工作
            # （HTTP 脚本客户端向后兼容）。
            self.hub.set_cmd(vx, wz, owner=0, hard=hard,
                             seq=body.get("seq"), session=body.get("session"),
                             ts=body.get("ts"))   # owner=0 表示 HTTP 客户端
            self._json({"ok": True, "cmd": self.hub.snapshot()["cmd"]})
            return
        if path == "/api/ramp":
            # 接受 {"on":true}，也兼容 {"enabled":...} 与 "on"/"off"/0/1。
            raw = body["on"] if "on" in body else body.get("enabled")
            on = parse_onoff(raw)
            if on is None:
                self._json({"ok": False,
                            "error": "on 必须是 true/false（或 1/0、on/off）"}, 400)
                return
            changed = self.hub.set_ramp(on)
            self._json({"ok": True, "changed": changed,
                        "ramp": self.hub.snapshot()["ramp"]})
            return
        if path == "/api/gear":
            gear = str(body.get("gear", ""))
            if not self.hub.set_gear(gear):
                self._json({"ok": False,
                            "error": "gear 必须是 low/mid/high/fast/sprint/ultra"}, 400)
                return
            self._json({"ok": True, "gear": self.hub.snapshot()["gear"]})
            return
        if path == "/api/estop":
            self.hub.estop_on("HTTP /api/estop")
            self._json({"ok": True, "estop": True})
            return
        if path == "/api/estop/reset":
            self.hub.estop_off()
            self._json({"ok": True, "estop": False})
            return
        if path == "/api/odom/reset":
            self.hub.reset_odom()
            self._json({"ok": True})
            return
        self._json({"ok": False, "error": "not found: %s" % path}, 404)

    # -- 静态文件 -----------------------------------------------------------
    def _serve_static(self, name: str) -> None:
        # 只允许 static/ 下的普通文件，防 ../ 穿越
        target = (STATIC_DIR / name).resolve()
        try:
            target.relative_to(STATIC_DIR)
        except ValueError:
            self._json({"ok": False, "error": "forbidden"}, 403)
            return
        if not target.is_file():
            self._json({"ok": False, "error": "not found: %s" % name}, 404)
            return
        data = target.read_bytes()
        ctype = MIME.get(target.suffix.lower(), "application/octet-stream")
        self._send(200, data, ctype, {"Cache-Control": "no-cache"})

    # -- /status 纯文本：curl 一行就能看，不用 jq ---------------------------
    def _status_text(self) -> str:
        s = self.hub.snapshot()
        link = s["link"]
        cmd = s["cmd"]
        st = s["status"]
        tel = s["telemetry"]
        lines = [
            "car_web %s  模式=%s  时间=%.1fs  客户端=%d" %
            (VERSION, "SIM" if s["sim"] else "REAL", s["ts"], s["ws_clients"]),
            "串口: %s @%d %s  rx年龄=%s  tx=%d  tel=%d  st=%d  坏帧=%d" % (
                link["port"], link["baud"], "OK" if link["open"] else "DOWN(%s)" % link["error"],
                "—" if link["rx_age_ms"] is None else "%sms" % link["rx_age_ms"],
                link["tx_count"], link["telemetry_count"],
                link["status_count"], link["bad_frames"]),
            "档位: %s  vx≤%.2f m/s  wz≤%.2f rad/s" %
            (s["gear"]["name"], s["gear"]["vx"], s["gear"]["wz"]),
            "缓启缓停: %s  加速 vx=%.2f / 减速 vx=%.2f m/s²  "
            "加速 wz=%.2f / 减速 wz=%.2f rad/s²  输出=(%.3f, %.3f)  %s" % (
                "开" if s["ramp"]["on"] else "关",
                s["ramp"]["vx_acc"], s["ramp"]["vx_dec"],
                s["ramp"]["wz_acc"], s["ramp"]["wz_dec"],
                s["ramp"]["out_vx"], s["ramp"]["out_wz"],
                {"decel": "缓停中", "accel": "斜坡中",
                 "hard": "立即停", "off": "斜坡关闭", "idle": "—"}[
                     s["ramp"]["mode"]]),
            "指令: 网页=(%.3f, %.3f)  实发=(%.3f, %.3f)  原因=%s  夹取=%s  年龄=%sms"
            "  衰减=%d%%  seq=%s" % (
                cmd["vx"], cmd["wz"], cmd["tx_vx"], cmd["tx_wz"],
                cmd["reason"], "是" if cmd["clamped"] else "否", cmd["age_ms"],
                round(cmd["degrade_scale"] * 100), cmd["seq"]),
            # ---- 1.4.0 链路抖动诊断：手机顿挫先看这三行 ----
            "抖动: 指令到达 %s  min/avg/max=%s/%s/%sms  抖动=%sms（窗口 %dms）" % (
                "%d 帧" % s["jitter"]["arrival"]["n"],
                _ms(s["jitter"]["arrival"]["min_ms"]),
                _ms(s["jitter"]["arrival"]["avg_ms"]),
                _ms(s["jitter"]["arrival"]["max_ms"]),
                _ms(s["jitter"]["arrival"]["jitter_ms"]),
                s["jitter"]["window_ms"]),
            "原因: web=%d stale=%d timeout=%d hard=%d estop=%d（近2s共 %d 帧）" % (
                s["jitter"]["reason"]["web"], s["jitter"]["reason"]["stale"],
                s["jitter"]["reason"]["timeout"], s["jitter"]["reason"]["hard"],
                s["jitter"]["reason"]["estop"], s["jitter"]["reason"]["frames"]),
            "超时清零: %d 次%s  分级衰减帧=%d{300ms:70%%,380ms:50%%,460ms:30%%}→0"
            "  旧包丢弃=%d 缺号=%d 会话重置=%d" % (
                s["jitter"]["timeout_zero_count"],
                "" if s["jitter"]["timeout_zero_last_s"] is None
                else "（最近 %.1fs）" % s["jitter"]["timeout_zero_last_s"],
                s["jitter"]["stale_frames_total"],
                s["jitter"]["stale_packets"], s["jitter"]["seq_gaps"],
                s["jitter"]["session_resets"]),
        ]
        if tel:
            imu = s["imu"] or {}
            lines.append("遥测: vx=%+.3f m/s  wz=%+.3f rad/s  IMU a=(%.2f,%.2f,%.2f) "
                         "|a|=%.2f m/s²  g=(%.3f,%.3f,%.3f)  stop=%s  bat=%dmV" % (
                             tel["vx"], tel["wz"], tel["ax"], tel["ay"], tel["az"],
                             imu.get("accel_mag", 0.0),
                             tel["gx"], tel["gy"], tel["gz"], tel["stop"], tel["bat_mv"]))
        else:
            lines.append("遥测: 还没有收到 0x7B 帧")
        if st:
            lines.append("状态: flags=0x%02X %s" % (
                st["flags_raw"], " ".join(
                    "%s=%d" % (f["label"], 1 if f["on"] else 0) for f in s["flags"])))
            for tag, w in (("左", st["left"]), ("右", st["right"])):
                age = "从未收到" if w["fb_age_ms"] == 0xFFFF else "%dms" % w["fb_age_ms"]
                lines.append(
                    "  %s: ω=%+.2f/目标%+.2f (Δ%+.2f, 跟随%s) rad/s  τ=%+.2f Nm  "
                    "位置=%+.2f rad  ERR=%d(%s)  MOS=%d℃(%s) 线圈=%d℃(%s)  反馈年龄=%s(%s)" % (
                        tag, w["omega"], w["target"], w["dev"],
                        "—" if w["follow_pct"] is None else "%.0f%%" % w["follow_pct"],
                        w["torque"], w["pos"], w["err"], w["err_text"],
                        w["mos_temp"], w["mos_level"], w["coil_temp"], w["coil_level"],
                        age, w["age_level"]))
            lines.append("  底盘回读指令=(%.3f, %.3f)  启动=%s(%s)  IMU=%s  "
                         "CAN失败=%d 反馈帧=%d" % (
                             st["cmd_vx"], st["cmd_wz"], st["boot_stage_text"],
                             st["boot_level"], st["imu_status_text"],
                             st["can_fail"], st["fb_count"]))
        else:
            lines.append("状态: 还没有收到 0x7E 帧")
        lines.append("里程: %.3f m  偏航=%.1f°  急停=%s" %
                     (s["odom"]["distance"], s["odom"]["yaw"] * 57.2958,
                      "是" if s["safety"]["estop"] else "否"))
        return "\n".join(lines) + "\n"

    # -- WebSocket ----------------------------------------------------------
    def _handle_ws(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._json({"ok": False, "error": "缺少 Sec-WebSocket-Key"}, 400)
            return
        accept = ws_accept_key(key)

        iface = WsIf(0)
        iface.attach(self.connection)

        # 先回 101、再登记客户端：如果握手写失败（对端已经断了），
        # 就不会在 hub 里留下一个永远发不出去的广播对象。
        # 直接写裸响应，不走 send_response，避免多出重复头。
        try:
            self.wfile.write((
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                "Sec-WebSocket-Accept: %s\r\n\r\n" % accept
            ).encode())
            self.wfile.flush()
        except OSError:
            return
        self.close_connection = True

        cid = self.hub.register_ws(iface)
        iface.cid = cid

        reader = WsReader(self.connection)
        self.connection.settimeout(1.0)
        stop = threading.Event()

        def sender() -> None:
            """20 Hz 推状态。所有客户端共用同一份 JSON（只序列化一次）。"""
            while not stop.is_set():
                try:
                    text = json.dumps({"t": "state", **self.hub.snapshot()},
                                      ensure_ascii=False, separators=(",", ":"))
                    iface.send_text(text)
                except Exception:                 # noqa: BLE001
                    break
                stop.wait(0.05)
            # 广播失败/退出：主动 shutdown，让下面的 read 立刻返回
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        th = threading.Thread(target=sender, name="ws-send-%d" % cid, daemon=True)
        th.start()

        try:
            while not stop.is_set():
                r, _, _ = select.select([self.connection], [], [], 0.5)
                if not r:
                    continue
                fin, opcode, payload = reader.read_frame()
                if opcode == 0x8:                 # close
                    try:
                        iface.send_control(0x8, payload[:2])
                    except Exception:
                        pass
                    break
                if opcode == 0x9:                 # ping
                    iface.send_control(0xA, payload)
                    continue
                if opcode == 0xA:                 # pong
                    continue
                if opcode not in (0x1, 0x2):
                    continue
                if not fin:
                    # 浏览器发的是小 JSON，不分片；真分片就忽略，
                    # 不要为这个场景维护拼接状态机。
                    continue
                try:
                    msg = json.loads(payload.decode("utf-8", "replace"))
                except ValueError:
                    continue
                if isinstance(msg, dict):
                    self._handle_ws_msg(msg, cid)
        except (ConnectionError, OSError, TimeoutError, ValueError):
            pass
        finally:
            stop.set()
            self.hub.unregister_ws(cid)
            iface.close()

    def _handle_ws_msg(self, msg: dict[str, Any], cid: int) -> None:
        t = msg.get("t")
        if t == "cmd":
            try:
                vx = float(msg.get("vx", 0.0) or 0.0)
                wz = float(msg.get("wz", 0.0) or 0.0)
            except (TypeError, ValueError):
                return
            # hard=true：页面隐藏/失焦/关页的安全停车，后端立即归零。
            hard = parse_onoff(msg.get("hard")) is True
            self.hub.set_cmd(vx, wz, owner=cid, hard=hard,
                             seq=msg.get("seq"), session=msg.get("session"),
                             ts=msg.get("ts"))
        elif t == "gear":
            self.hub.set_gear(str(msg.get("gear", "")))
        elif t == "ramp":
            on = parse_onoff(msg["on"] if "on" in msg else msg.get("enabled"))
            if on is not None:
                self.hub.set_ramp(on)
        elif t == "estop":
            self.hub.estop_on("网页急停（%s）" % self._client_ip())
        elif t == "estop_reset":
            self.hub.estop_off()
        elif t == "odom_reset":
            self.hub.reset_odom()
        elif t == "ping":
            pass


# ---------------------------------------------------------------------------
# 6. 启动
# ---------------------------------------------------------------------------


def local_ip() -> str:
    """取本机对外的 IP（不真的发包，只让内核选路由）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return "127.0.0.1"


def make_server(host: str, port: int, hub: CarHub, args: argparse.Namespace) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"hub": hub, "args": args})
    srv = ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    return srv


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="小车遥控器 Web（RK3576 串口 → STM32 → 达妙电机）")
    ap.add_argument("--host", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    ap.add_argument("--port", type=int, default=8000, help="监听端口（默认 8000）")
    ap.add_argument("--serial", default="/dev/wheeltec_controller",
                    help="串口设备（默认 /dev/wheeltec_controller）")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--sim", action="store_true",
                    help="仿真模式：不打开串口，内部产生遥测/状态帧")
    ap.add_argument("--tx-hz", type=float, default=30.0,
                    help="命令帧发送频率 Hz（20~50，默认 30）")
    ap.add_argument("--verbose", action="store_true", help="打印每秒统计与 HTTP 日志")
    a = ap.parse_args(argv)
    if not (20.0 <= a.tx_hz <= 50.0):
        ap.error("--tx-hz 必须在 20~50 之间（STM32 有 800 ms 看门狗）")
    return a


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    hub = CarHub(args)
    ip = local_ip()

    try:
        srv = make_server(args.host, args.port, hub, args)
    except OSError as exc:
        print("端口 %d 起不来：%s\n换个端口：--port 8001" % (args.port, exc), flush=True)
        return 2

    worker = LinkWorker(hub, args)
    worker.start()

    bar = "=" * 68
    print(bar, flush=True)
    print(" 小车遥控器 Web  %s" % ("【仿真 SIM】" if args.sim else "【真机】"), flush=True)
    print(" 监听:   http://%s:%d" % (args.host, args.port), flush=True)
    print(" 本机:   http://127.0.0.1:%d" % args.port, flush=True)
    print(" 局域网: http://%s:%d" % (ip, args.port), flush=True)
    if args.sim:
        print(" 链路:   内部仿真（不占用任何串口）", flush=True)
    else:
        print(" 链路:   %s @ %d（独占打开）" % (args.serial, args.baud), flush=True)
    print(" 命令帧: %.0f Hz   |vx|<=%.2f m/s  |wz|<=%.2f rad/s  网页心跳 %d ms" %
          (args.tx_hz, HARD_VX_LIMIT, HARD_WZ_LIMIT, CMD_TIMEOUT_MS), flush=True)
    # 控制台在 Windows 上常是 GBK，别在这里用 ² 之类的字符（HTTP/JSON 里可以用）。
    print(" 缓启缓停: 默认%s  加速 vx %.2f / 减速 vx %.2f m/s2  "
          "加速 wz %.2f / 减速 wz %.2f rad/s2" %
          ("开" if RAMP_DEFAULT_ON else "关",
           RAMP_VX_ACC, RAMP_VX_DEC, RAMP_WZ_ACC, RAMP_WZ_DEC), flush=True)
    print(" 停车:   急停/超时/断连/页面隐藏=立即归零；松手/回中/停止按钮=软停",
          flush=True)
    print(" 停止:   Ctrl+C", flush=True)
    print(bar, flush=True)

    srv_thread = threading.Thread(target=srv.serve_forever, name="http", daemon=True)
    srv_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[main] 收到 Ctrl+C，正在停止…", flush=True)
    finally:
        worker.stop()
        worker.join(timeout=2.0)
        srv.shutdown()
        srv.server_close()
        print("[main] 已停止（最后一帧已发 0）", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
