"""TB01 底盘串口协议（主机 ↔ STM32），以本仓库 firmware/stm32-chassis/user/main.c 为准。

物理层：UART 115200 8N1，帧均为**大端**、BCC = 逐字节异或。

1) 速度指令帧（主机 → STM32，11 字节）
   [0] 0x7B
   [1] AutoRecharge（正常 0）
   [2] SecurityPLY（正常 0）
   [3..4]  int16 BE  vx  ×1000 (mm/s)
   [5..6]  int16 BE  vy  ×1000 (mm/s，差速车恒 0)
   [7..8]  int16 BE  wz  ×1000 (mrad/s)
   [9]  BCC = XOR([0..8])
   [10] 0x7D

2) 主遥测帧（STM32 → 主机，24 字节，20 Hz）
   [0] 0x7B
   [1] Flag_Stop: 1 = 当前有效命令为停止（含 800 ms 命令超时）
   [2..3]  int16 BE 车体 vx  mm/s（由**实际轮速**正解，不是目标）
   [4..5]  vy  mm/s（恒 0）
   [6..7]  车体 wz  mrad/s（由实际轮速差速正解）
   [8..13] IMU 加速度三轴原始值（±2g，m/s² = raw / 1671.84）
   [14..19] IMU 陀螺仪三轴原始值（±500dps，rad/s = raw * 0.00026644）
   [20..21] uint16 BE 电池 mV（本板无采样，恒 0）
   [22] BCC = XOR([0..21])
   [23] 0x7D

3) 扩展诊断帧（STM32 → 主机，44 字节，20 Hz，紧跟主遥测）
   详见 firmware Status_Send()：flags/轮速 omega×100/扭矩×100/ERR/温度/
   位置×100/CAN 失败计数/版本号/IMU 状态/反馈年龄/tick/启动阶段。

4) 自动回充遥测（8 字节，0x7C ... 0x7F）、超声波（19 字节，0xFA ... 0xFC）
   本车未使用，解析器仅做帧边界识别（不解析内容）。

5) 主机功能帧（主机 → STM32，11 字节，固件 v2+）
   和速度帧同头同尾同 BCC，靠 [1] 区分：
   [1]=0x00/0x01 普通速度帧；[1]=0x06 要求底盘重新"写速度模式 + 使能"两个
   DM 驱动器（解决"整车断电重启后驱动器失能、主机发速度不动"的问题）。
   底盘只在**车已停**时接受；主机的正确用法是先发一帧零速再发本帧。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------- 帧常量
CMD_SIZE, CMD_HEAD, CMD_TAIL = 11, 0x7B, 0x7D
TEL_SIZE, TEL_HEAD, TEL_TAIL = 24, 0x7B, 0x7D
DIAG_SIZE, DIAG_HEAD, DIAG_TAIL = 44, 0x7E, 0x7D
RECHARGE_SIZE, RECHARGE_HEAD, RECHARGE_TAIL = 8, 0x7C, 0x7F
ULTRASONIC_SIZE, ULTRASONIC_HEAD, ULTRASONIC_TAIL = 19, 0xFA, 0xFC

CMD_SCALE = 1000.0          # m/s -> mm/s, rad/s -> mrad/s
ACCEL_RATIO = 32768.0 / 19.6        # raw -> m/s²（±2g 量程）
GYRO_RATIO = 0.00026644             # raw -> rad/s（±500 dps 量程）

# 诊断帧 [30] 版本号
#   v1: 初版 44 字节诊断帧
#   v2: 增加"主机功能帧"（f[1]=0x06 电机重新使能），见 build_func_frame()
DIAG_VERSION = 2
DIAG_VERSION_MIN = 1            # 向后兼容的最低诊断帧版本
FUNC_FRAME_MIN_VERSION = 2      # 主机功能帧从固件 v2 开始支持

# 主机功能帧码（11 字节帧的 f[1]）
FUNC_MOTOR_REINIT = 0x06        # 重新写速度模式 + 使能（仅停车时底盘才接受）

# 驱动器 ERR（DM 协议语义，来自 DM6540-1EC 反馈帧高 4 位）
# 注意：0/1 是正常状态（失能/使能），>1 才是故障。
FAULTS = {
    0: "失能",
    1: "使能",
    5: "读取校准数据错误",
    6: "读取电机参数错误",
    7: "传感器读取异常",
    8: "超压",
    9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失",
    0xE: "过载",
}
OK_ERR_VALUES = (0, 1)
MODES = {1: "电压", 4: "力矩", 5: "速度", 6: "位置"}
IMU_STATUS = {0: "正常", 1: "SCL拉不高", 2: "SDA拉不高", 3: "总线死", 4: "无应答", 5: "读出错"}
BOOT_STAGE = {0: "启动中", 1: "正常", 2: "CAN 初始化失败", 3: "IMU 从未读到", 4: "CAN+IMU 都失败"}


def bcc(data: bytes) -> int:
    out = 0
    for byte in data:
        out ^= byte
    return out


def clamp_i16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def _i16(frame: bytes, offset: int) -> int:
    return struct.unpack(">h", bytes(frame[offset:offset + 2]))[0]


def _u16(frame: bytes, offset: int) -> int:
    return struct.unpack(">H", bytes(frame[offset:offset + 2]))[0]


# ---------------------------------------------------------------- 速度指令
def build_speed_frame(vx: float, vy: float = 0.0, wz: float = 0.0) -> bytes:
    """构造 11 字节速度指令帧（vx m/s, vy m/s, wz rad/s）。"""
    frame = bytearray(CMD_SIZE)
    frame[0] = CMD_HEAD
    frame[1] = 0            # AutoRecharge，正常走车必须为 0
    frame[2] = 0            # SecurityPLY，正常走车必须为 0
    struct.pack_into(">h", frame, 3, clamp_i16(vx * CMD_SCALE))
    struct.pack_into(">h", frame, 5, clamp_i16(vy * CMD_SCALE))
    struct.pack_into(">h", frame, 7, clamp_i16(wz * CMD_SCALE))
    frame[CMD_SIZE - 2] = bcc(bytes(frame[:CMD_SIZE - 2]))
    frame[CMD_SIZE - 1] = CMD_TAIL
    return bytes(frame)


@dataclass
class SpeedCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def frame(self) -> bytes:
        return build_speed_frame(self.vx, self.vy, self.wz)


# ---------------------------------------------------------------- 主机功能帧
def build_func_frame(func_code: int, value: int = 0) -> bytes:
    """构造 11 字节主机功能帧（需要固件 v2+，见 DIAG_VERSION）。

    - func_code = FUNC_MOTOR_REINIT (0x06)：让底盘重新"写速度模式 + 使能"。
      底盘只在车已停时接受，所以调用前应先发一帧零速。
    - value 写到 f[2]（当前底盘保留，未使用）。
    """
    frame = bytearray(CMD_SIZE)
    frame[0] = CMD_HEAD
    frame[1] = int(func_code) & 0xFF
    frame[2] = int(value) & 0xFF
    frame[CMD_SIZE - 2] = bcc(bytes(frame[:CMD_SIZE - 2]))
    frame[CMD_SIZE - 1] = CMD_TAIL
    return bytes(frame)


# ---------------------------------------------------------------- 主遥测
@dataclass
class Telemetry:
    raw: bytes = field(repr=False, default=b"")
    flag_stop: int = 0
    vx: float = 0.0            # m/s，实际
    vy: float = 0.0            # m/s
    wz: float = 0.0            # rad/s，实际
    accel_raw: tuple[int, int, int] = (0, 0, 0)
    gyro_raw: tuple[int, int, int] = (0, 0, 0)
    battery_mv: int = 0
    timestamp: float = 0.0

    @property
    def battery_v(self) -> float:
        return self.battery_mv / 1000.0

    @property
    def accel_ms2(self) -> tuple[float, float, float]:
        return tuple(v / ACCEL_RATIO for v in self.accel_raw)  # type: ignore[return-value]

    @property
    def gyro_rad_s(self) -> tuple[float, float, float]:
        return tuple(v * GYRO_RATIO for v in self.gyro_raw)  # type: ignore[return-value]


def frame_valid(frame: bytes) -> bool:
    """校验帧尾 + BCC（各帧 BCC 都在倒数第 2 字节）。"""
    if len(frame) < 4:
        return False
    if frame[-1] != frame_tail_for(frame[0]):
        return False
    return frame[-2] == bcc(frame[:-2])


def frame_tail_for(head: int) -> int:
    return {
        CMD_HEAD: CMD_TAIL,
        TEL_HEAD: TEL_TAIL,
        DIAG_HEAD: DIAG_TAIL,
        RECHARGE_HEAD: RECHARGE_TAIL,
        ULTRASONIC_HEAD: ULTRASONIC_TAIL,
    }.get(head, -1)


def frame_size_for(head: int) -> int:
    return {
        CMD_HEAD: CMD_SIZE,
        TEL_HEAD: TEL_SIZE,
        DIAG_HEAD: DIAG_SIZE,
        RECHARGE_HEAD: RECHARGE_SIZE,
        ULTRASONIC_HEAD: ULTRASONIC_SIZE,
    }.get(head, 0)


def parse_telemetry(frame: bytes, timestamp: float = 0.0) -> Optional[Telemetry]:
    if len(frame) != TEL_SIZE or not frame_valid(frame):
        return None
    return Telemetry(
        raw=bytes(frame),
        flag_stop=frame[1],
        vx=_i16(frame, 2) / CMD_SCALE,
        vy=_i16(frame, 4) / CMD_SCALE,
        wz=_i16(frame, 6) / CMD_SCALE,
        accel_raw=(_i16(frame, 8), _i16(frame, 10), _i16(frame, 12)),
        gyro_raw=(_i16(frame, 14), _i16(frame, 16), _i16(frame, 18)),
        battery_mv=_u16(frame, 20),
        timestamp=timestamp,
    )


# ---------------------------------------------------------------- 诊断帧
@dataclass
class ChassisDiag:
    raw: bytes = field(repr=False, default=b"")
    flags: int = 0
    cmd_vx: float = 0.0                 # 底盘最近收到的指令 vx，m/s
    cmd_wz: float = 0.0                 # 底盘最近收到的指令 wz，rad/s
    wheel_vel: tuple[float, float] = (0.0, 0.0)       # 实际 omega，rad/s
    wheel_target: tuple[float, float] = (0.0, 0.0)    # 最终命令 omega，rad/s（含外环修正）
    torque: tuple[float, float] = (0.0, 0.0)          # Nm
    err: tuple[int, int] = (0, 0)
    mos_temp: tuple[int, int] = (0, 0)
    coil_temp: tuple[int, int] = (0, 0)
    position: tuple[float, float] = (0.0, 0.0)        # rad
    can_tx_fail: int = 0
    feedback_count: int = 0
    version: int = 0
    imu_status: int = 0
    fb_age_ms: tuple[int, int] = (0, 0)
    tick_ms: int = 0
    imu_i2c_addr: int = 0
    imu_probe_flags: int = 0
    imu_gyro_ok: bool = False
    imu_version: int = 0
    boot_stage: int = 0
    timestamp: float = 0.0

    @property
    def left_online(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def right_online(self) -> bool:
        return bool(self.flags & 0x02)

    @property
    def left_enabled(self) -> bool:
        return bool(self.flags & 0x04)

    @property
    def right_enabled(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def cmd_timeout(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def left_fault(self) -> bool:
        return bool(self.flags & 0x20)

    @property
    def right_fault(self) -> bool:
        return bool(self.flags & 0x40)

    @property
    def imu_ok(self) -> bool:
        return bool(self.flags & 0x80)

    def err_name(self, index: int) -> str:
        return FAULTS.get(self.err[index], f"未知({self.err[index]})")

    @property
    def is_fault(self) -> bool:
        return any(e not in OK_ERR_VALUES for e in self.err)

    def to_dict(self) -> dict:
        return {
            "flags": self.flags,
            "left_online": self.left_online,
            "right_online": self.right_online,
            "left_enabled": self.left_enabled,
            "right_enabled": self.right_enabled,
            "cmd_timeout": self.cmd_timeout,
            "left_fault": self.left_fault,
            "right_fault": self.right_fault,
            "imu_ok": self.imu_ok,
            "cmd_vx": round(self.cmd_vx, 4),
            "cmd_wz": round(self.cmd_wz, 4),
            "wheel_vel": [round(v, 3) for v in self.wheel_vel],
            "wheel_target": [round(v, 3) for v in self.wheel_target],
            "torque": [round(v, 3) for v in self.torque],
            "err": list(self.err),
            "err_name": [self.err_name(0), self.err_name(1)],
            "mos_temp": list(self.mos_temp),
            "coil_temp": list(self.coil_temp),
            "position": [round(v, 3) for v in self.position],
            "can_tx_fail": self.can_tx_fail,
            "feedback_count": self.feedback_count,
            "version": self.version,
            "imu_status": IMU_STATUS.get(self.imu_status, f"未知({self.imu_status})"),
            "imu_gyro_ok": self.imu_gyro_ok,
            "imu_i2c_addr": self.imu_i2c_addr,
            "imu_version": self.imu_version,
            "fb_age_ms": list(self.fb_age_ms),
            "tick_ms": self.tick_ms,
            "boot_stage": BOOT_STAGE.get(self.boot_stage, f"未知({self.boot_stage})"),
        }


def parse_diag(frame: bytes, timestamp: float = 0.0) -> Optional[ChassisDiag]:
    if len(frame) != DIAG_SIZE or not frame_valid(frame):
        return None
    return ChassisDiag(
        raw=bytes(frame),
        flags=frame[1],
        cmd_vx=_i16(frame, 2) / CMD_SCALE,
        cmd_wz=_i16(frame, 4) / CMD_SCALE,
        wheel_vel=(_i16(frame, 6) / 100.0, _i16(frame, 8) / 100.0),
        wheel_target=(_i16(frame, 10) / 100.0, _i16(frame, 12) / 100.0),
        torque=(_i16(frame, 14) / 100.0, _i16(frame, 16) / 100.0),
        err=(frame[18], frame[19]),
        mos_temp=(frame[20], frame[21]),
        coil_temp=(frame[22], frame[23]),
        position=(_i16(frame, 24) / 100.0, _i16(frame, 26) / 100.0),
        can_tx_fail=frame[28],
        feedback_count=frame[29],
        version=frame[30],
        imu_status=frame[31],
        fb_age_ms=(_u16(frame, 32), _u16(frame, 34)),
        tick_ms=_u16(frame, 36),
        imu_i2c_addr=frame[38],
        imu_probe_flags=frame[39] & 0x7F,
        imu_gyro_ok=bool(frame[39] & 0x80),
        imu_version=frame[40],
        boot_stage=frame[41],
        timestamp=timestamp,
    )


# ---------------------------------------------------------------- 回充 / 超声波
@dataclass
class RechargeTelemetry:
    raw: bytes = field(repr=False, default=b"")
    current_a: float = 0.0
    ir_seen: bool = False
    charging: bool = False
    setting: int = 0


def parse_recharge(frame: bytes) -> Optional[RechargeTelemetry]:
    if len(frame) != RECHARGE_SIZE or not frame_valid(frame):
        return None
    return RechargeTelemetry(
        raw=bytes(frame),
        current_a=_i16(frame, 1) / 1000.0,
        ir_seen=bool(frame[3]),
        charging=bool(frame[4]),
        setting=frame[5],
    )


def is_zero_command(frame: bytes) -> bool:
    """判断一帧速度指令是不是"全零停车"。"""
    try:
        return len(frame) == CMD_SIZE and frame[3:6] == b"\x00\x00\x00" and frame[7:9] == b"\x00\x00"
    except Exception:  # noqa: BLE001
        return False


def parse_cmd_frame(frame: bytes) -> Optional[SpeedCommand]:
    """解析 11 字节速度指令帧（用于自测/抓包核对，节点自身不调用）。"""
    if len(frame) != CMD_SIZE or not frame_valid(frame):
        return None
    return SpeedCommand(
        vx=_i16(frame, 3) / CMD_SCALE,
        vy=_i16(frame, 5) / CMD_SCALE,
        wz=_i16(frame, 7) / CMD_SCALE,
    )
