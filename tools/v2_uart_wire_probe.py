"""判定 CH343P 的 TXD/RXD 到底接在 STM32 的哪个脚上。

做法：把 STM32 的 PA9/PA10 临时改成"浮空输入"，让 PC 在 COM 口上连续发 0x00（几乎一直是低电平），
再用调试器读一次 GPIOA_IDR：哪个脚是低 -> 那个脚接着 CH343 的 TXD。

用法：
    python tools/v2_uart_wire_probe.py --port COM18 --pattern 00
"""
import argparse
import subprocess
import threading
import time

import serial

OPENOCD = r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe"
GPIOA_CRH = 0x40010804
GPIOA_IDR = 0x40010808
CRH_INPUT = 0x88844488      # PA8~PA15 全浮空输入
CRH_RESTORE = 0x888448B0    # PA9=0xB(AF_PP) PA10=0x8(IPU)


def openocd(script, timeout=45):
    args = [
        OPENOCD,
        "-f", "interface/cmsis-dap.cfg",
        "-f", "target/stm32f1x.cfg",
        "-c", "adapter speed 500",
    ]
    for c in script:
        args += ["-c", c]
    r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return (r.stdout or "") + (r.stderr or "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM18")
    ap.add_argument("--pattern", default="00")
    ap.add_argument("--seconds", type=float, default=6.0)
    args = ap.parse_args()

    byte = int(args.pattern, 16) & 0xFF
    stop = threading.Event()
    sent = [0]

    def writer():
        ser = serial.Serial(args.port, 115200, timeout=0.1)
        while not stop.is_set():
            ser.write(bytes([byte]) * 64)
            sent[0] += 64
        ser.close()

    th = threading.Thread(target=writer, daemon=True)
    th.start()
    time.sleep(0.3)
    print("PC 正在 %.0f s 内连续发送 0x%02X ..." % (args.seconds, byte))

    out = openocd([
        "init",
        "halt",
        "mww 0x%08X 0x%08X" % (GPIOA_CRH, CRH_INPUT),
        "resume",
        "sleep 200",
        "halt",
        "mdw 0x%08X 1" % GPIOA_IDR,
        "mww 0x%08X 0x%08X" % (GPIOA_CRH, CRH_RESTORE),
        "resume",
        "shutdown",
    ])
    stop.set()
    th.join(timeout=2.0)

    idr = None
    for line in out.splitlines():
        if line.strip().startswith("0x%08X:" % GPIOA_IDR):
            idr = int(line.split(":")[1].split()[0], 16)
    print("已发送 %d 字节" % sent[0])
    if idr is None:
        print("读 IDR 失败，openocd 输出尾部：")
        print("\n".join(out.strip().splitlines()[-6:]))
        return
    print("GPIOA_IDR = 0x%08X" % idr)
    pa9 = (idr >> 9) & 1
    pa10 = (idr >> 10) & 1
    print("PA9(TX) = %d, PA10(RX) = %d" % (pa9, pa10))
    if byte == 0x00:
        print("发 0x00（低电平为主）时：低=被外部驱动=接在 CH343 的 TXD 上")
    else:
        print("发 0xFF（高电平为主）时：低=说明另有下拉或没接")
    if pa9 == 0 and pa10 == 1:
        print("=> CH343 的 TXD 接在 PA9（和 MCU 的 TX 撞在一起）= TX/RX 接反了")
    elif pa10 == 0 and pa9 == 1:
        print("=> CH343 的 TXD 接在 PA10（方向正确），那 MCU->PC 那根是断的/没接")
    else:
        print("=> 两个都没被拉低，说明 PC 的 TX 没到 MCU（线没接上/接错脚）")


if __name__ == "__main__":
    main()
