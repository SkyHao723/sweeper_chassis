"""无 ROS 自测：python3 -m tb01_chassis.selftest

覆盖：
- 速度帧字节与协议文档示例逐字节一致（含 BCC）
- 24 字节主遥测解析（速度/IMU 原始值换算）
- 44 字节扩展诊断帧解析（flags/轮速/扭矩/故障码）
- 里程计积分、IMU 姿态解算
- ChassisCore + 假 STM32 闭环：收发帧、指令超时零速、退出连发零速、垃圾字节容错
"""

from __future__ import annotations

import math
import struct
import sys
import threading
import time

from . import protocol as p
from .core import ChassisCore
from .odometry import ImuOrientation, OdomIntegrator

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


# ---------------------------------------------------------------- 造帧
def _i16(v: int) -> bytes:
    return struct.pack(">h", v)


def _u16(v: int) -> bytes:
    return struct.pack(">H", v)


def build_tel(vx: float, wz: float, accel=(1672, 0, 1672), gyro=(0, 0, 3756), battery_mv=0) -> bytes:
    body = bytearray(p.TEL_SIZE)
    body[0] = p.TEL_HEAD
    body[1] = 1 if (vx == 0 and wz == 0) else 0
    body[2:4] = _i16(int(round(vx * 1000)))
    body[4:6] = _i16(0)
    body[6:8] = _i16(int(round(wz * 1000)))
    for i, value in enumerate(accel):
        body[8 + i * 2:10 + i * 2] = _i16(int(value))
    for i, value in enumerate(gyro):
        body[14 + i * 2:16 + i * 2] = _i16(int(value))
    body[20:22] = _u16(int(battery_mv))
    body[22] = p.bcc(bytes(body[:22]))
    body[23] = p.TEL_TAIL
    return bytes(body)


def build_diag(flags=0x8F, cmd_vx=0.2, cmd_wz=0.5, wheel_vel=(2.0, -1.5),
               target=(2.1, -1.4), torque=(0.5, 0.6), err=(0, 5)) -> bytes:
    body = bytearray(p.DIAG_SIZE)
    body[0] = p.DIAG_HEAD
    body[1] = flags
    body[2:4] = _i16(int(round(cmd_vx * 1000)))
    body[4:6] = _i16(int(round(cmd_wz * 1000)))
    body[6:8] = _i16(int(round(wheel_vel[0] * 100)))
    body[8:10] = _i16(int(round(wheel_vel[1] * 100)))
    body[10:12] = _i16(int(round(target[0] * 100)))
    body[12:14] = _i16(int(round(target[1] * 100)))
    body[14:16] = _i16(int(round(torque[0] * 100)))
    body[16:18] = _i16(int(round(torque[1] * 100)))
    body[18], body[19] = err
    body[20], body[21] = 41, 42
    body[22], body[23] = 19, 20
    body[24:26] = _i16(123)
    body[26:28] = _i16(-45)
    body[28] = 3
    body[29] = 200
    body[30] = p.DIAG_VERSION
    body[31] = 0
    body[32:34] = _u16(50)
    body[34:36] = _u16(60)
    body[36:38] = _u16(12345)
    body[38] = 0x6A
    body[39] = 0x87       # bit7 = gyro_ok
    body[40] = 1
    body[41] = 1
    body[42] = p.bcc(bytes(body[:42]))
    body[43] = p.DIAG_TAIL
    return bytes(body)


# ---------------------------------------------------------------- 假 STM32
class FakeChassis:
    """串口对端：接收 11 字节速度帧，20 Hz 回 24B 遥测 + 44B 诊断帧。"""

    WHEEL_R = 0.1025      # 假轮径（只用于诊断帧里的 omega）

    def __init__(self, add_noise: bool = False):
        self._lock = threading.Lock()
        self._rx = bytearray()          # 主机读到的数据（从真实设备角度叫 tx）
        self._cmd_buf = bytearray()
        self._running = True
        self.add_noise = add_noise

        self.cmds: list[tuple[float, float, float]] = []
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time = 0.0
        self.cmd_timeout = False
        self.enabled = False            # 假底盘：上电默认失能（和 DM 驱动器一致）
        self.reinit_count = 0           # 收到几次"重新使能"功能帧
        self.tel_sent = 0
        self.diag_sent = 0
        self._vx = 0.0
        self._wz = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # 串口接口 ------------------------------------------------------
    def read(self, n: int = 1) -> bytes:
        with self._lock:
            if not self._rx:
                time.sleep(0.002)
                return b""
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def write(self, data: bytes) -> int:
        with self._lock:
            self._cmd_buf.extend(data)
            while len(self._cmd_buf) >= p.CMD_SIZE:
                frame = bytes(self._cmd_buf[:p.CMD_SIZE])
                if p.frame_valid(frame) and frame[0] == p.CMD_HEAD:
                    if frame[1] == p.FUNC_MOTOR_REINIT:
                        # 底盘"重新使能"：模拟 DM 驱动器被使能
                        self.reinit_count += 1
                        self.enabled = True
                        del self._cmd_buf[:p.CMD_SIZE]
                        continue
                    cmd = p.parse_cmd_frame(frame)
                    if cmd is not None:
                        self.last_cmd = (cmd.vx, cmd.vy, cmd.wz)
                        self.last_cmd_time = time.monotonic()
                        self.cmds.append(self.last_cmd)
                    del self._cmd_buf[:p.CMD_SIZE]
                else:
                    del self._cmd_buf[:1]
        return len(data)

    def close(self) -> None:
        self._running = False

    # 模拟底盘 ------------------------------------------------------
    def _run(self) -> None:
        while self._running:
            now = time.monotonic()
            self.cmd_timeout = (now - self.last_cmd_time) > 0.8
            target_vx, target_wz = (0.0, 0.0) if self.cmd_timeout else self.last_cmd[0::2]
            with self._lock:
                self._vx += (target_vx - self._vx) * 0.35
                self._wz += (target_wz - self._wz) * 0.35
                vx, wz = self._vx, self._wz

            tel = build_tel(vx, wz)
            flags = 0x8F if not self.cmd_timeout else 0x9F
            if not self.enabled:
                flags &= ~0x0C           # bit2/bit3 = 左/右使能
            diag = build_diag(
                flags=flags,
                cmd_vx=self.last_cmd[0], cmd_wz=self.last_cmd[2],
                wheel_vel=(vx / self.WHEEL_R - wz * 0.425 / 2 / self.WHEEL_R,
                           vx / self.WHEEL_R + wz * 0.425 / 2 / self.WHEEL_R),
                target=(vx / self.WHEEL_R, vx / self.WHEEL_R),
                err=(1, 1) if self.enabled else (0, 0),
            )
            with self._lock:
                if self.add_noise:
                    self._rx.extend(b"\x13\x37\x55")
                self._rx.extend(tel)
                self._rx.extend(diag)
                self.tel_sent += 1
                self.diag_sent += 1
            time.sleep(0.05)


# ---------------------------------------------------------------- 测试
def test_protocol_vectors() -> None:
    print("[1] 速度帧字节（与协议文档示例核对）")
    cases = [
        ("停车", 0.0, 0.0, "7B 00 00 00 00 00 00 00 00 7B 7D"),
        ("前进 0.2", 0.2, 0.0, "7B 00 00 00 C8 00 00 00 00 B3 7D"),
        ("前进 0.3", 0.3, 0.0, "7B 00 00 01 2C 00 00 00 00 56 7D"),
        ("左转 0.5", 0.0, 0.5, "7B 00 00 00 00 00 00 01 F4 8E 7D"),
        ("后退 -0.2", -0.2, 0.0, "7B 00 00 FF 38 00 00 00 00 BC 7D"),
    ]
    for name, vx, wz, expected in cases:
        frame = p.build_speed_frame(vx, 0.0, wz)
        want = bytes.fromhex(expected)
        check(f"{name} 帧一致", frame == want, f"{frame.hex(' ')} != {want.hex(' ')}")
    check("超限被夹到 int16", p.build_speed_frame(100.0, 0, 0)[3:5] == _i16(32767))
    parsed = p.parse_cmd_frame(p.build_speed_frame(0.25, 0.0, -0.75))
    check("速度帧往返解析", parsed is not None and abs(parsed.vx - 0.25) < 1e-6 and abs(parsed.wz + 0.75) < 1e-6)

    # 主机功能帧（固件 v2+）：f[1]=0x06 = 电机重新使能
    func = p.build_func_frame(p.FUNC_MOTOR_REINIT)
    func_want = bytes.fromhex("7B 06 00 00 00 00 00 00 00 7D 7D")
    check("功能帧字节 (f[1]=0x06)", func == func_want, f"{func.hex(' ')} != {func_want.hex(' ')}")
    check("功能帧 BCC/帧尾合法", p.frame_valid(func))


def test_telemetry_parse() -> None:
    print("[2] 主遥测解析")
    tel = p.parse_telemetry(build_tel(0.2, -0.5, accel=(1672, 0, 1672), gyro=(0, 0, 3756)))
    check("帧校验通过", tel is not None)
    assert tel is not None
    check("vx 换算", abs(tel.vx - 0.2) < 1e-4, f"{tel.vx}")
    check("wz 换算", abs(tel.wz + 0.5) < 1e-4, f"{tel.wz}")
    check("加速度换算", abs(tel.accel_ms2[0] - 1.0) < 0.01, f"{tel.accel_ms2}")
    check("陀螺仪换算", abs(tel.gyro_rad_s[2] - 1.0) < 0.01, f"{tel.gyro_rad_s}")
    check("Flag_Stop=0", tel.flag_stop == 0)
    check("BCC 错误帧被拒", p.parse_telemetry(build_tel(0.2, 0.0)[:-2] + b"\x00\x7D") is None)


def test_diag_parse() -> None:
    print("[3] 扩展诊断帧解析")
    diag = p.parse_diag(build_diag())
    check("帧校验通过", diag is not None)
    assert diag is not None
    check("flags 解码", diag.left_online and diag.right_online and diag.left_enabled
          and diag.right_enabled and diag.imu_ok and not diag.cmd_timeout)
    check("指令回显", abs(diag.cmd_vx - 0.2) < 1e-4 and abs(diag.cmd_wz - 0.5) < 1e-4)
    check("轮速 omega", abs(diag.wheel_vel[0] - 2.0) < 0.02 and abs(diag.wheel_vel[1] + 1.5) < 0.02)
    check("目标 omega", abs(diag.wheel_target[0] - 2.1) < 0.02)
    check("扭矩 Nm", abs(diag.torque[0] - 0.5) < 0.02)
    check("故障码解码", diag.err_name(0) == "失能" and diag.err_name(1) == "读取校准数据错误")
    check("故障判定 (>1 才算故障)", diag.is_fault)
    check("位置 rad", abs(diag.position[0] - 1.23) < 0.02 and abs(diag.position[1] + 0.45) < 0.02)
    check("IMU 状态", diag.imu_ok and diag.imu_gyro_ok and diag.boot_stage == 1)
    check("诊断帧版本 v2", diag.version == p.DIAG_VERSION, f"version={diag.version}")


def test_odom_imu() -> None:
    print("[4] 里程计与 IMU")
    integ = OdomIntegrator()
    integ.update(0.2, 0.0, 0.0, 0.0)
    for i in range(1, 101):
        integ.update(0.2, 0.0, 0.0, i * 0.01)
    check("直线 1s 位移≈0.2m", abs(integ.state.pose.x - 0.2) < 0.01, f"x={integ.state.pose.x:.3f}")

    integ.reset()
    integ.update(0.0, 0.0, 0.5, 0.0)
    for i in range(1, 101):
        integ.update(0.0, 0.0, 0.5, i * 0.01)
    check("原地转 1s 角度≈0.5rad", abs(integ.state.pose.theta - 0.5) < 0.03, f"theta={integ.state.pose.theta:.3f}")

    imu = ImuOrientation()
    ori = imu.update((0.0, 0.0, 9.8), (0.0, 0.0, 0.0), 0.01)
    check("静止时 roll/pitch≈0", abs(ori.roll) < 0.01 and abs(ori.pitch) < 0.01)
    for _ in range(50):
        ori = imu.update((0.0, 0.0, 9.8), (0.0, 0.0, 1.0), 0.01)
    check("陀螺仪 z 积分 yaw≈0.5", abs(ori.yaw - 0.5) < 0.03, f"yaw={ori.yaw:.3f}")
    quat_norm = math.sqrt(sum(q * q for q in ori.quat))
    check("四元数归一", abs(quat_norm - 1.0) < 1e-6)


def test_core_loop() -> None:
    print("[5] ChassisCore + 假 STM32 闭环")
    fake = FakeChassis(add_noise=True)
    telem: list[p.Telemetry] = []
    diags: list[p.ChassisDiag] = []
    log: list[tuple[str, str]] = []

    def on_tel(t: p.Telemetry) -> None:
        telem.append(t)

    def on_diag(d: p.ChassisDiag) -> None:
        diags.append(d)

    core = ChassisCore(
        port="fake",
        cmd_timeout=0.3,
        idle_zero_hz=20.0,
        stop_burst=5,
        logger=lambda level, msg: log.append((level, msg)),
        serial_factory=lambda: fake,
    )
    core.on_telemetry = on_tel
    core.on_diag = on_diag
    try:
        core.start()
        time.sleep(0.6)
        check("串口连接（假设备）", core.connected)
        check("收到主遥测", len(telem) > 5, f"n={len(telem)}")
        check("收到诊断帧", len(diags) > 5, f"n={len(diags)}")
        check("垃圾字节被容错", core.stats["bad_bytes"] > 0 and len(telem) > 0,
              f"bad={core.stats['bad_bytes']}")
        zero_before = sum(1 for c in fake.cmds if c == (0.0, 0.0, 0.0))
        check("空闲时持续发零速（显式停车）", zero_before > 0, f"zero={zero_before}")

        # 持续发前进指令 0.6s
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            core.send_speed(0.2, 0.0, 0.0)
            time.sleep(0.05)
        time.sleep(0.15)
        check("速度帧被底盘收到", any(abs(c[0] - 0.2) < 1e-6 for c in fake.cmds))
        vx_now = telem[-1].vx
        check("回传 vx 跟随指令", abs(vx_now - 0.2) < 0.05, f"vx={vx_now:.3f}")
        check("诊断帧回显指令 vx", abs(diags[-1].cmd_vx - 0.2) < 1e-3, f"{diags[-1].cmd_vx}")

        # 停止发指令 -> 超时后零速保活
        time.sleep(0.6)
        zero_after = sum(1 for c in fake.cmds if c == (0.0, 0.0, 0.0))
        check("指令超时后补发零速", zero_after > zero_before, f"before={zero_before} after={zero_after}")
        check("超时后回传 vx 归零", abs(telem[-1].vx) < 0.05, f"vx={telem[-1].vx:.3f}")
        cmd_before_stop = len(fake.cmds)
    finally:
        core.stop()
    burst = len(fake.cmds) - cmd_before_stop
    check("退出时连发零速", burst >= 5, f"burst={burst}")
    check("统计无异常", core.stats["reconnects"] == 0, str(core.stats))
    fake.close()


def test_reinit() -> None:
    print("[6] 上位机电机重新使能（功能帧 f[1]=0x06）")
    fake = FakeChassis()
    diags: list[p.ChassisDiag] = []
    core = ChassisCore(
        port="fake",
        cmd_timeout=0.3,
        idle_zero_hz=20.0,
        stop_burst=3,
        serial_factory=lambda: fake,
    )
    core.on_diag = lambda d: diags.append(d)
    try:
        core.start()
        time.sleep(0.6)
        check("假底盘初始为失能", bool(diags) and not diags[-1].left_enabled and list(diags[-1].err) == [0, 0],
              f"err={list(diags[-1].err) if diags else None}")
        ok = core.send_func(p.FUNC_MOTOR_REINIT, repeat=3)
        time.sleep(0.5)
        check("功能帧发送并计数", ok and core.stats["func_frames"] == 3, str(core.stats))
        check("假底盘收到重新使能", fake.reinit_count >= 1, f"n={fake.reinit_count}")
        check("底盘回报使能", bool(diags) and diags[-1].left_enabled and diags[-1].right_enabled
              and list(diags[-1].err) == [1, 1], f"err={list(diags[-1].err) if diags else None}")
    finally:
        core.stop()
        fake.close()


def main() -> int:
    print("=== tb01_chassis 自测（不需要 ROS / 硬件） ===")
    test_protocol_vectors()
    test_telemetry_parse()
    test_diag_parse()
    test_odom_imu()
    test_core_loop()
    test_reinit()
    print(f"\n===== 结果：{PASS} 通过，{FAIL} 失败 =====")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
