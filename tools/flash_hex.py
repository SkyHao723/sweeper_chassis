"""用 DAPLink/ST-Link + openocd 烧录一个 hex, 并做独立回读校验。

为什么要单独写这个脚本:
  1. openocd 的 `program {x.bin}` 不认识 bin 里的地址, 会从 0x00000000 开始写
     (报 "no flash bank found for address 0x00000000"), 所以只用 hex, 让 openocd
     自己解析地址;
  2. 烧完必须自己 dump 回来算 MD5 —— "Verified OK" 是 openocd 自己说的,
     独立回读才算验证;
  3. 板子的 NRST 常常没接, `reset` 走不通, 用写 AIRCR(SYSRESETREQ) 的办法软复位。

用法:
  python tools/flash_hex.py firmware/stm32-chassis-v2/Objects/chassis-v2/chassis-v2.hex
  python tools/flash_hex.py <hex> --no-reset        # 只烧不启动
  python tools/flash_hex.py <hex> --base 0x08000000 --speed 500
"""
import argparse
import hashlib
import os
import subprocess
import sys

from intelhex import IntelHex

OCD = r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe"

# 软件复位: 往 AIRCR 写 VECTKEY | SYSRESETREQ
AIRCR_RESET = "mww 0xE000ED0C 0x05FA0004"


def run_openocd(args, speed):
    cmd = [
        OCD,
        "-f", "interface/cmsis-dap.cfg",
        "-f", "target/stm32f1x.cfg",
        "-c", "adapter speed %d" % speed,
    ] + args
    proc = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    return (proc.stdout or "") + (proc.stderr or "")


def show(text, keywords):
    for line in text.splitlines():
        if any(k in line for k in keywords):
            print("   ", line.strip())


def main():
    parser = argparse.ArgumentParser(description="烧录 hex 并回读校验")
    parser.add_argument("hex", help="hex 文件路径")
    parser.add_argument("--base", type=lambda s: int(s, 0), default=0x08000000,
                        help="flash 起始地址, 默认 0x08000000")
    parser.add_argument("--speed", type=int, default=500,
                        help="SWD 时钟 kHz, 默认 500 (1000 偶发 CMD_INFO failed)")
    parser.add_argument("--no-reset", action="store_true", help="烧完不软复位")
    args = parser.parse_args()

    if not os.path.exists(args.hex):
        sys.exit("hex not found: %s" % args.hex)

    image = IntelHex(args.hex).tobinarray(start=args.base)
    expect = hashlib.md5(bytes(image)).hexdigest().upper()
    print("hex    : %s" % args.hex)
    print("image  : %d B @0x%08X  md5=%s" % (len(image), args.base, expect))

    # 1) 烧录 + openocd 自校验
    print("[1/3] program + verify")
    text = run_openocd(["-c", "program {%s} verify" % args.hex.replace("\\", "/"),
                        "-c", "shutdown"], args.speed)
    show(text, ("device id", "flash size", "Programming", "Verified", "Error", "error"))
    if "Verified OK" not in text:
        print(text)
        sys.exit("烧录失败")

    # 2) 独立回读
    print("[2/3] read back")
    dump = os.path.join(os.environ.get("TEMP", "."), "_flash_readback.bin")
    if os.path.exists(dump):
        os.remove(dump)
    text = run_openocd(["-c", "init", "-c", "halt",
                        "-c", "dump_image {%s} 0x%08X %d" % (dump.replace("\\", "/"), args.base, len(image)),
                        "-c", "shutdown"], args.speed)
    show(text, ("dumped", "Error"))
    if not os.path.exists(dump):
        sys.exit("回读失败")
    got = hashlib.md5(open(dump, "rb").read()).hexdigest().upper()
    os.remove(dump)
    print("       readback md5=%s" % got)
    if got != expect:
        sys.exit("回读校验失败 (MISMATCH)")

    # 3) 软复位启动
    if not args.no_reset:
        print("[3/3] reset (SYSRESETREQ)")
        run_openocd(["-c", "init", "-c", AIRCR_RESET, "-c", "shutdown"], args.speed)

    print("OK: 烧录并回读校验通过 (%d B, md5=%s)" % (len(image), expect))


if __name__ == "__main__":
    main()
