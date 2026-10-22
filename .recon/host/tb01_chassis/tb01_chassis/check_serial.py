"""串口只读检查工具（不启动 ROS，用来在车/台架上核对协议链路）。

用法：
    # 只读：打印底盘回传（遥测 + 诊断），不会让轮子转
    python3 -m tb01_chassis.check_serial -p /dev/wheeltec_controller

    # 同时以 10 Hz 发零速帧（显式停车，喂 800ms 看门狗），验证写通路
    python3 -m tb01_chassis.check_serial -p /dev/wheeltec_controller --zero

    # 打印原始十六进制帧（前 10 帧）
    python3 -m tb01_chassis.check_serial -p /dev/wheeltec_controller --raw

    # 让底盘重新给两个 DM 驱动器"写速度模式 + 使能"（需要固件 v2+）
    python3 -m tb01_chassis.check_serial -p /dev/wheeltec_controller --reinit

注意：串口独占。运行前先停掉 ROS 底盘节点（否则抢不到串口）。
"""

from __future__ import annotations

import argparse
import sys
import time

from . import protocol as p
from .core import ChassisCore


def main() -> int:
    ap = argparse.ArgumentParser(description="TB01 底盘串口检查（只读 + 可选零速保活）")
    ap.add_argument("-p", "--port", default="/dev/wheeltec_controller")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = 一直运行（Ctrl-C 退出）")
    ap.add_argument("--zero", action="store_true", help="以 10 Hz 持续发零速帧（显式停车）")
    ap.add_argument("--raw", action="store_true", help="打印原始帧十六进制（前 10 帧）")
    ap.add_argument("--reinit", action="store_true",
                    help="发功能帧让底盘重新使能两个 DM 驱动器（需要固件 v2+，仅停车时生效）")
    args = ap.parse_args()

    latest = {"tel": None, "diag": None}
    raw_shown = 0

    def on_tel(tel: p.Telemetry) -> None:
        nonlocal raw_shown
        latest["tel"] = tel
        if args.raw and raw_shown < 10:
            raw_shown += 1
            print(f"TEL  {tel.raw.hex(' ')}")

    def on_diag(diag: p.ChassisDiag) -> None:
        nonlocal raw_shown
        latest["diag"] = diag
        if args.raw and raw_shown < 10:
            raw_shown += 1
            print(f"DIAG {diag.raw.hex(' ')}")

    core = ChassisCore(
        port=args.port,
        baud=args.baud,
        cmd_timeout=0.2,
        idle_zero_hz=10.0 if args.zero else 0.0,
        stop_burst=10,
        logger=lambda level, msg: print(f"[{level}] {msg}"),
    )
    core.on_telemetry = on_tel
    core.on_diag = on_diag
    core.start()

    if args.reinit:
        time.sleep(0.6)
        diag = latest["diag"]
        if diag is not None and diag.version < p.FUNC_FRAME_MIN_VERSION:
            print(f"!! 底盘固件 v{diag.version} 不支持上位机使能（需要 v{p.FUNC_FRAME_MIN_VERSION}+）")
        else:
            print("发送『电机重新使能』功能帧（先零速，再 f[1]=0x06 ×3）…")
            ok = core.send_func(p.FUNC_MOTOR_REINIT, 0, repeat=3, interval=0.05)
            print(f"  发送{'成功' if ok else '失败'}")
            time.sleep(1.2)
            diag = latest["diag"]
            if diag is not None:
                print(f"  -> ERR={list(diag.err)} ({diag.err_name(0)}/{diag.err_name(1)})，"
                      f"使能 L/R={int(diag.left_enabled)}/{int(diag.right_enabled)}")

    start = time.monotonic()
    last_print = 0.0
    try:
        while args.seconds <= 0 or time.monotonic() - start < args.seconds:
            time.sleep(0.05)
            now = time.monotonic()
            if now - last_print < 1.0:
                continue
            last_print = now
            stats = core.status()
            tel = latest["tel"]
            diag = latest["diag"]
            if tel is not None:
                gyro = ",".join(f"{v:+.3f}" for v in tel.gyro_rad_s)
                acc = ",".join(f"{v:+.2f}" for v in tel.accel_ms2)
                print(
                    f"tel : vx={tel.vx:+.3f} m/s wz={tel.wz:+.3f} rad/s stop={tel.flag_stop} | "
                    f"acc=({acc}) m/s² gyro=({gyro}) rad/s | bat={tel.battery_v:.3f}V"
                )
            if diag is not None:
                warnings = []
                if not diag.left_online or not diag.right_online:
                    warnings.append("驱动器掉线")
                if diag.left_fault or diag.right_fault:
                    warnings.append(f"故障 {diag.err_name(0)}/{diag.err_name(1)}")
                if diag.cmd_timeout:
                    warnings.append("命令超时")
                if not diag.imu_ok:
                    warnings.append("IMU 读取失败")
                if diag.boot_stage not in (0, 1):
                    warnings.append(f"启动阶段={diag.to_dict()['boot_stage']}")
                warn = (" !! " + "; ".join(warnings)) if warnings else ""
                print(
                    f"diag: flags=0x{diag.flags:02X} L(on={int(diag.left_online)},en={int(diag.left_enabled)}) "
                    f"R(on={int(diag.right_online)},en={int(diag.right_enabled)}) | "
                    f"轮速 {diag.wheel_vel[0]:+.2f}/{diag.wheel_vel[1]:+.2f} rad/s 目标 "
                    f"{diag.wheel_target[0]:+.2f}/{diag.wheel_target[1]:+.2f} | "
                    f"扭矩 {diag.torque[0]:+.2f}/{diag.torque[1]:+.2f} Nm | "
                    f"命令 vx={diag.cmd_vx:+.3f} wz={diag.cmd_wz:+.3f} | "
                    f"CAN失败={diag.can_tx_fail} 反馈={diag.feedback_count} 版本={diag.version}{warn}"
                )
            if tel is None and diag is None:
                print(f"等待底盘帧… 已收到 {stats['telemetry_frames']} 遥测 / "
                      f"{stats['diag_frames']} 诊断，坏字节 {stats['bad_bytes']}")
    except KeyboardInterrupt:
        pass
    finally:
        core.stop()
        print("已退出（已补发零速停车帧）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
