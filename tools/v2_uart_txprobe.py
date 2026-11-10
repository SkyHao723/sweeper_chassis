"""判定 CH343 的 RXD 到底有没有接到 PA9：把 PA9 强行拉低 300ms，看 PC 端能不能收到东西。

原理：
  - 正常工作时 PA9 是 USART1_TX（复用推挽），由串口外设驱动；
  - 这里把 PA9 临时改成 GPIO 推挽输出并拉低（相当于串口线长期保持在低电平 = break），
    同时 PC 在 COM18 上持续读。
  - 如果 PC 收到 0x00 字节（或 framing error），说明 PA9 -> CH343 RXD 这根线是通的；
    如果一个字节都收不到，说明 CH343 的 RXD 根本没连到 PA9（焊错/漏焊/连到别的脚）。

会临时改 GPIOA_CRH，结束后自动恢复并 resume（板子继续跑）。

用法：
  python tools/v2_uart_txprobe.py --port COM18
"""
import argparse
import subprocess
import threading
import time

import serial

OPENOCD = r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe"

GPIOA_CRH = 0x40010804
GPIOA_BRR = 0x40010814
GPIOA_BSRR = 0x40010810

CRH_NORMAL = 0x888448B0   # PA9 = AF_PP 50MHz (0xB)
CRH_PA9_OUT = 0x888448A0  # PA9 = Out_PP 2MHz (0x2), 其它脚不变
BR9 = 0x00000200
BS9 = 0x00000200


def openocd(*cmds, timeout=45):
    args = [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f1x.cfg",
            "-c", "adapter speed 500", "-c", "init", "-c", "halt"]
    for c in cmds:
        args += ["-c", c]
    args += ["-c", "resume", "-c", "shutdown"]
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout)


def drive_low():
    openocd("mww 0x%08X 0x%08X" % (GPIOA_CRH, CRH_PA9_OUT),
            "mww 0x%08X 0x%08X" % (GPIOA_BRR, BR9))


def restore():
    openocd("mww 0x%08X 0x%08X" % (GPIOA_CRH, CRH_NORMAL),
            "mww 0x%08X 0x%08X" % (GPIOA_BSRR, BS9))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM18")
    ap.add_argument("--hold", type=float, default=0.35)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.02)
    ser.reset_input_buffer()

    got = {"data": b"", "n": 0}

    def reader(stop):
        while not stop.is_set():
            try:
                chunk = ser.read(4096)
            except Exception:
                break
            if chunk:
                got["data"] += chunk
                got["n"] += len(chunk)
                if len(got["data"]) > 4096:
                    got["data"] = got["data"][:4096]

    stop = threading.Event()
    th = threading.Thread(target=reader, args=(stop,), daemon=True)
    th.start()

    print("把 PA9 拉低 %.0f ms（模拟 break）..." % (args.hold * 1000))
    drive_low()
    time.sleep(args.hold)
    stop.set()
    th.join(timeout=1.0)
    ser.close()

    print("恢复 PA9 为 USART1_TX")
    restore()

    n = got["n"]
    print("PC 在 COM18 上收到 %d 字节" % n)
    if got["data"]:
        print("样例:", got["data"][:32].hex(" "))
        print("=> PA9 -> CH343 RXD 这根线是通的（收到 %d 字节）" % n)
    else:
        print("=> 一个字节都没收到：CH343 的 RXD 没有连到 PA9（或者中间断/焊错脚）")


if __name__ == "__main__":
    main()
