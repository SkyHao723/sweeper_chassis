"""底盘主控 V2 上行遥测帧监视器。

两种用法：

1) 板子 USB 插在电脑上（CH343P 虚拟串口）——实时解读 32 字节遥测帧：
       python tools/v2_monitor.py --port COM15
       python tools/v2_monitor.py --port COM15 --seconds 5

2) 板子没插 USB（只有 DAPLink）——直接从 RAM 里把"最后发出的一帧"读出来。
   前提是固件里 telemetry.c 留了 s_last_frame 快照，且带 .map 文件：
       python tools/v2_monitor.py --ram

帧格式见 firmware/stm32-chassis-v2/user/telemetry.h。
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

FRAME_SIZE = 32
HEAD = 0x7E
TAIL = 0x7D
VERSION = 1

MODE_NAMES = {0: "HOST", 1: "RC", 2: "PUSH"}

MAP_CANDIDATES = [
    r"firmware/stm32-chassis-v2/Listings/chassis-v2/chassis-v2.map",
]

OPENOCD_CANDIDATES = [
    r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe",
]


def i16(frame, offset):
    value = (frame[offset] << 8) | frame[offset + 1]
    return value - 0x10000 if value >= 0x8000 else value


def u16(frame, offset):
    return (frame[offset] << 8) | frame[offset + 1]


def xor_bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def decode(frame):
    """解析一帧，返回 dict；帧不合法返回 None。"""
    if len(frame) != FRAME_SIZE:
        return None
    if frame[0] != HEAD or frame[31] != TAIL:
        return None
    if xor_bcc(frame[:30]) != frame[30]:
        return None

    flags = frame[1]
    acc = [i16(frame, 7), i16(frame, 9), i16(frame, 11)]
    gyro = [i16(frame, 13), i16(frame, 15), i16(frame, 17)]

    return {
        "imu_ok": bool(flags & 0x01),
        "aht_ok": bool(flags & 0x02),
        "mode": frame[2],
        "mode_name": MODE_NAMES.get(frame[2], "?"),
        "vx_mm_s": i16(frame, 3),
        "wz_mrad_s": i16(frame, 5),
        "acc_g": [value / 1000.0 for value in acc],
        "gyro_dps": [value / 10.0 for value in gyro],
        "imu_temp_c": i16(frame, 19) / 100.0,
        "rh_percent": u16(frame, 21) / 100.0,
        "aht_temp_c": i16(frame, 23) / 100.0,
        "tick_ms": u16(frame, 25),
        "rx_count": frame[27],
        "rj_count": frame[28],
        "version": frame[29],
        "raw": bytes(frame).hex(),
    }


def format_line(info):
    return (
        "mode=%-4s imu=%s aht=%s "
        "acc[g]=(%6.3f %6.3f %6.3f) gyro[dps]=(%6.2f %6.2f %6.2f) "
        "T_imu=%5.2fC RH=%5.2f%% T_aht=%5.2fC "
        "v_cmd=(%4d,%5d) tick=%5d rx=%-3d rj=%-3d"
        % (
            info["mode_name"],
            "OK" if info["imu_ok"] else "--",
            "OK" if info["aht_ok"] else "--",
            info["acc_g"][0],
            info["acc_g"][1],
            info["acc_g"][2],
            info["gyro_dps"][0],
            info["gyro_dps"][1],
            info["gyro_dps"][2],
            info["imu_temp_c"],
            info["rh_percent"],
            info["aht_temp_c"],
            info["vx_mm_s"],
            info["wz_mrad_s"],
            info["tick_ms"],
            info["rx_count"],
            info["rj_count"],
        )
    )


# ----------------------------- 遥控上行帧 (0x79, 19 字节) -----------------------------

RC_HEAD = 0x79
RC_SIZE = 19
RC_BUTTONS = [(0x01, "A"), (0x02, "B"), (0x04, "X"), (0x08, "Y"),
              (0x10, "LB"), (0x20, "RB"), (0x40, "Back"), (0x80, "Start")]
RC_HATS = ["-", "U", "UR", "R", "DR", "D", "DL", "L", "UL"]


def decode_rc(frame):
    """19 字节: 79 | flags | 15 字节 HID 原始数据 | BCC | 7D"""
    if len(frame) != RC_SIZE:
        return None
    if frame[0] != RC_HEAD or frame[18] != TAIL:
        return None
    if xor_bcc(frame[:17]) != frame[17]:
        return None
    flags = frame[1]
    hid = frame[2:17]
    buttons = hid[11]
    names = [n for bit, n in RC_BUTTONS if buttons & bit]
    hat = (hid[12] >> 2) & 0x0F
    return {
        "flags": flags,
        "online": bool(flags & 0x01),
        "changed": bool(flags & 0x02),
        "x": hid[1] | (hid[2] << 8),
        "y": hid[3] | (hid[4] << 8),
        "rx": hid[5] | (hid[6] << 8),
        "ry": hid[7] | (hid[8] << 8),
        "z": hid[9] | (hid[10] << 8),
        "buttons": buttons,
        "button_names": names,
        "hat": hat,
        "hat_name": RC_HATS[hat] if hat < len(RC_HATS) else "?",
        "raw": bytes(hid).hex(),
    }


def format_rc(info):
    return ("[rc ] online=%d changed=%d X=%5d Y=%5d Rx=%5d Ry=%5d Z=%5d hat=%s btn=%s"
            % (info["online"], info["changed"], info["x"], info["y"],
               info["rx"], info["ry"], info["z"],
               info["hat_name"], "+".join(info["button_names"]) or "-"))


# ----------------------------- 串口实时 -----------------------------


def run_serial(port, baud, seconds):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("需要 pyserial: pip install pyserial")

    deadline = None if seconds <= 0 else time.time() + seconds
    buffer = bytearray()
    frames = 0
    rc_frames = 0
    bad = 0
    last_rc = None

    with serial.Serial(port, baud, timeout=0.2) as link:
        print("串口 %s @%d, 按 Ctrl-C 结束" % (port, baud))
        while True:
            if deadline is not None and time.time() > deadline:
                break
            chunk = link.read(64)
            if chunk:
                buffer.extend(chunk)

            # 两种帧都要认: 0x7E 遥测(32B) / 0x79 遥控(19B)
            while True:
                if not buffer:
                    break
                head = buffer[0]
                if head == HEAD:
                    need = FRAME_SIZE
                    decoder = decode
                elif head == RC_HEAD:
                    need = RC_SIZE
                    decoder = decode_rc
                else:
                    bad += 1
                    del buffer[:1]
                    continue
                if len(buffer) < need:
                    break
                candidate = buffer[:need]
                info = decoder(candidate)
                if info is None:
                    bad += 1
                    del buffer[:1]
                    continue
                del buffer[:need]
                if head == HEAD:
                    frames += 1
                    print("[%6.2fs] %s" % (time.time() % 1000, format_line(info)))
                else:
                    rc_frames += 1
                    line = format_rc(info)
                    if line != last_rc:
                        last_rc = line
                        print("[%6.2fs] %s" % (time.time() % 1000, line))

    print("")
    print("共 %d 帧遥测, %d 帧遥控, 丢弃 %d 字节组" % (frames, rc_frames, bad))
    return 0


# ----------------------------- RAM 快照 -----------------------------


def find_map():
    for path in MAP_CANDIDATES:
        if os.path.exists(path):
            return path
    hits = glob.glob("**/chassis-v2.map", recursive=True)
    return hits[0] if hits else None


def symbol_address(map_path, symbol):
    pattern = re.compile(r"^\s+%s\s+0x([0-9a-fA-F]+)" % re.escape(symbol))
    with open(map_path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            match = pattern.match(line)
            if match:
                return int(match.group(1), 16)
    return None


def find_openocd():
    for path in OPENOCD_CANDIDATES:
        if os.path.exists(path):
            return path
    return "openocd"


def read_ram(address, size):
    dump = os.path.join(os.environ.get("TEMP", "."), "v2_monitor_ram.bin")
    if os.path.exists(dump):
        os.remove(dump)
    command = [
        find_openocd(),
        "-f",
        "interface/cmsis-dap.cfg",
        "-f",
        "target/stm32f1x.cfg",
        "-c",
        "adapter speed 500",
        "-c",
        "init",
        "-c",
        "halt",
        "-c",
        "dump_image {%s} 0x%08X %d" % (dump, address, size),
        "-c",
        "shutdown",
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if not os.path.exists(dump):
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        sys.exit("dump_image 失败")
    with open(dump, "rb") as fh:
        return fh.read()


def run_ram():
    map_path = find_map()
    if not map_path:
        sys.exit("找不到 chassis-v2.map，先编译一次工程")
    address = symbol_address(map_path, "s_last_frame")
    if address is None:
        sys.exit("map 里没有 s_last_frame（固件太旧？）")

    print("map   : %s" % map_path)
    print("frame : 0x%08X (32 B)" % address)
    data = read_ram(address, FRAME_SIZE)
    print("raw   : %s" % data.hex())

    info = decode(data)
    if info is None:
        sys.exit("这一帧校验不过（可能还没发过帧，或固件版本不匹配）")
    print(format_line(info))
    print("version=%d" % info["version"])
    return 0


def main():
    parser = argparse.ArgumentParser(description="底盘 V2 遥测监视器")
    parser.add_argument("--port", help="串口，如 COM15")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0, help="0 = 一直跑")
    parser.add_argument("--ram", action="store_true", help="从 RAM 读最后发出的一帧")
    args = parser.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    if args.ram:
        return run_ram()
    if not args.port:
        parser.error("要么给 --port，要么用 --ram")
    return run_serial(args.port, args.baud, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
