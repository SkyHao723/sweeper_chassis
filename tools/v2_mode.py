"""给"底盘主控 V2"发工作模式命令（也可顺便发一条速度帧验证模式拦截）。

帧格式（详见 firmware/stm32-chassis-v2/user/proto.h）：
  功能帧 8 B： 7A | func | p0 | p1 | p2 | p3 | BCC | 7D      BCC = XOR(前 6 字节)
               func=0x02 切换模式, p0 = 0 上位机控制 / 1 遥控 / 2 推行
  速度帧 11 B：7B | 0 | 0 | vx_H vx_L | vy_H vy_L | wz_H wz_L | BCC | 7D

用法：
  python tools/v2_mode.py COM15 host              # 切到上位机控制模式
  python tools/v2_mode.py COM15 rc                # 切到遥控模式
  python tools/v2_mode.py COM15 push              # 切到推行模式
  python tools/v2_mode.py COM15 cycle             # host -> rc -> push -> host 各发一次
  python tools/v2_mode.py COM15 rc --speed 200    # 切遥控后再发一条 200mm/s 速度帧
                                                  # （应被拒绝: 屏幕 RX 不涨, RJ 涨）
  python tools/v2_mode.py COM15 host --speed 200  # 上位机模式下发同样的速度帧（应被接受）

"--listen N" 会把串口上收到的调试打印原样转出来 N 秒（默认 2.0 秒），
切完模式就能直接看到 "[mode] RC -> PUSH by uart" 这样的日志。
"""
import argparse
import sys
import time

import serial

MODES = {"host": 0, "rc": 1, "push": 2}
MODE_NAMES = {0: "HOST", 1: "RC", 2: "PUSH"}


def xor_bcc(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def build_set_mode(mode: int) -> bytes:
    """功能帧: 7A 02 mode 00 00 00 BCC 7D"""
    body = bytes([0x7A, 0x02, mode & 0x03, 0x00, 0x00, 0x00])
    return body + bytes([xor_bcc(body), 0x7D])


def build_speed(vx_mm_s: int, wz_mrad_s: int = 0) -> bytes:
    """速度帧: 7B 00 00 vx(2, BE) vy(2, BE) wz(2, BE) BCC 7D"""
    vx = vx_mm_s & 0xFFFF
    vy = 0
    wz = wz_mrad_s & 0xFFFF
    body = bytes([
        0x7B, 0x00, 0x00,
        (vx >> 8) & 0xFF, vx & 0xFF,
        (vy >> 8) & 0xFF, vy & 0xFF,
        (wz >> 8) & 0xFF, wz & 0xFF,
    ])
    return body + bytes([xor_bcc(body), 0x7D])


def hexstr(data: bytes) -> str:
    return " ".join("%02X" % b for b in data)


def listen(port: serial.Serial, seconds: float) -> None:
    deadline = time.time() + seconds
    port.timeout = 0.05
    while time.time() < deadline:
        chunk = port.read(256)
        if chunk:
            sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="底盘主控 V2 模式切换工具")
    parser.add_argument("port", help="串口, 例如 COM15 / /dev/ttyUSB0")
    parser.add_argument("mode", choices=list(MODES.keys()) + ["cycle"],
                        help="目标模式: host / rc / push / cycle")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=None,
                        help="切完模式后再发一条速度帧 (mm/s), 用来验证该模式是否接受")
    parser.add_argument("--listen", type=float, default=2.0,
                        help="切完后监听串口打印的秒数 (默认 2.0, 0 = 不监听)")
    args = parser.parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print("打不开串口 %s: %s" % (args.port, exc))
        return 1

    try:
        # 先清掉串口里的旧数据, 免得把上一轮的打印混进来
        port.reset_input_buffer()

        if args.mode == "cycle":
            targets = [MODES["host"], MODES["rc"], MODES["push"], MODES["host"]]
        else:
            targets = [MODES[args.mode]]

        for mode in targets:
            frame = build_set_mode(mode)
            port.write(frame)
            port.flush()
            print("-> SET_MODE %-4s  %s" % (MODE_NAMES[mode], hexstr(frame)))
            time.sleep(0.5)

        if args.speed is not None:
            frame = build_speed(args.speed)
            port.write(frame)
            port.flush()
            print("-> SPEED    %d mm/s  %s" % (args.speed, hexstr(frame)))

        if args.listen > 0:
            print("--- 串口打印 (%.1f s) ---" % args.listen)
            listen(port, args.listen)
            print("--- 结束 ---")
    finally:
        port.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
