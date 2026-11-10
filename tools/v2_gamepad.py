"""手柄接收器 / 遥控链路工具（底盘主控 V2）。

两个用途：
  1) 没接 CH554G 和手柄接收器时，从调试串口把 15 字节手柄原始帧"注入"给 STM32，
     走的是和真实链路完全相同的解析 + 映射代码（只用调试帧包装一下）。
  2) 接上真实链路后，解析 STM32 上行的 0x79 遥控帧（15 字节原始数据原样带回），
     看看到底收到的是什么。

协议（见 firmware/stm32-chassis-v2/user/rc.h 与 proto.h）：

  手柄原始帧 15 字节（Xbox 360 兼容接收器, VID 045E PID 028E, 500Hz）
    [0]      0x00 固定
    [1..2]   X   int16 LE   左摇杆 X, 中位 32768
    [3..4]   Y   int16 LE   左摇杆 Y, 中位 32767（向上变大）
    [5..6]   Rx  int16 LE   右摇杆 X, 中位 32768
    [7..8]   Ry  int16 LE   右摇杆 Y, 中位 32767
    [9..10]  Z   int16 LE   = 32768 + (LT-RT)*128
    [11]     按钮低 8 位    A=01 B=02 X=04 Y=08 LB=10 RB=20 Back=40 Start=80
    [12]     bit0=LS bit1=RS bit2..5=十字键(0..8)
    [13][14] 0x00 固定

  调试注入帧 18 字节（只给本工具用，走 USART1）
    78 | 15 字节手柄原始帧 | BCC=XOR(前 16 字节) | 7D

  上行遥控帧 19 字节（STM32 -> 上位机, 20Hz）
    79 | flags(bit0=在线 bit1=这一帧变了) | 15 字节原始帧 | BCC=XOR(前 17) | 7D

用法：
  # 只发一帧中位（验证解析链路通不通；屏幕第 3 行应该变成 RC X0 Y0 h0）
  python tools/v2_gamepad.py COM15 center

  # 发一帧"左摇杆推到右上角 + 按 A + 十字键上"
  python tools/v2_gamepad.py COM15 send --lx 1000 --ly 1000 --a --hat 1

  # 切到遥控模式，摇杆扫一圈（每 20ms 一帧，共 6 秒），同时解析上行遥控帧
  python tools/v2_gamepad.py COM15 play --mode rc --seconds 6 --listen

  # 原样发一帧 15 字节（比如从 sample_capture.csv 里复制一行 raw_hex）
  python tools/v2_gamepad.py COM15 raw "00 00 80 ff 7f 00 80 ff 7f 00 80 01 00 00 00"

摇杆参数用千分制：-1000..1000，0=中位；右/上为正（和上位机约定一致），
固件里再做死区（10%）和限幅（vx 最大 600 mm/s，wz 最大 800 mrad/s）。
"""
import argparse
import sys
import time

import serial

# ---------------------------------------------------------------- 帧构造
INJECT_HEAD = 0x78
INJECT_SIZE = 18
RC_HEAD = 0x79
RC_FRAME_SIZE = 19
TELEMETRY_HEAD = 0x7E
TELEMETRY_SIZE = 32
TAIL = 0x7D

CENTER_X = 32768
CENTER_Y = 32767
CENTER_Z = 32768

BTN_BITS = {
    "a": (0x01, 0), "b": (0x02, 0), "x": (0x04, 0), "y": (0x08, 0),
    "lb": (0x10, 0), "rb": (0x20, 0), "back": (0x40, 0), "start": (0x80, 0),
    "ls": (0x01, 1), "rs": (0x02, 1),
}

# 和固件 rc.h 保持一致
RC_VX_MAX_MM_S = 600
RC_WZ_MAX_MRAD_S = 800
RC_DEADZONE_PERMILLE = 100


def xor_bcc(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def hexstr(data: bytes) -> str:
    return " ".join("%02X" % b for b in data)


def permille_to_raw(value: int, center: int) -> int:
    """千分制 -1000..1000 -> 接收器原始值 (0..65535)。"""
    value = max(-1000, min(1000, int(value)))
    raw = center + int(round(value * 32768.0 / 1000.0))
    return max(0, min(0xFFFF, raw))


def parse_buttons(text) -> int:
    """把 "a+b+start" / "a,b" / "a b" 解析成 (低字节, 高位字节) 两个掩码。

    单个字母可以连写: "abxy" 也可以。
    """
    if not text:
        return 0, 0
    if isinstance(text, (bytes, bytearray)):
        text = text.decode("ascii", errors="replace")
    tokens = [tok for tok in __import__("re").split(r"[,\s+]+", text) if tok]
    # 全是单字母且每个都是合法键名 -> 拆成单字符
    if len(tokens) == 1 and len(tokens[0]) > 1 and all(ch in BTN_BITS for ch in tokens[0]):
        tokens = list(tokens[0])
    low = 0
    high = 0
    for token in tokens:
        name = token.lower()
        if name not in BTN_BITS:
            raise ValueError("未知按键 %r，可用: %s" % (token, " ".join(sorted(BTN_BITS))))
        bit, which = BTN_BITS[name]
        if which == 0:
            low |= bit
        else:
            high |= bit
    return low, high


def build_hid_frame(lx=0, ly=0, rx=0, ry=0, lt=0, rt=0,
                    buttons="", hat: int = 0) -> bytes:
    """按接收器 15 字节布局拼一帧。

    lx/ly/rx/ry 用千分制（右/上为正），lt/rt 用 0..255，
    buttons 用键名串（例如 "a+b" / "abxy" / "start"），hat 0..8。
    """
    x = permille_to_raw(lx, CENTER_X)
    y = permille_to_raw(ly, CENTER_Y)
    rxs = permille_to_raw(rx, CENTER_X)
    rys = permille_to_raw(ry, CENTER_Y)
    z = CENTER_Z + (int(lt) - int(rt)) * 128
    z = max(0, min(0xFFFF, z))

    btn_low, btn_high = parse_buttons(buttons)
    hat = max(0, min(8, int(hat)))

    frame = bytearray(15)
    frame[0] = 0x00
    frame[1] = x & 0xFF
    frame[2] = (x >> 8) & 0xFF
    frame[3] = y & 0xFF
    frame[4] = (y >> 8) & 0xFF
    frame[5] = rxs & 0xFF
    frame[6] = (rxs >> 8) & 0xFF
    frame[7] = rys & 0xFF
    frame[8] = (rys >> 8) & 0xFF
    frame[9] = z & 0xFF
    frame[10] = (z >> 8) & 0xFF
    frame[11] = btn_low & 0xFF
    frame[12] = (btn_high & 0x03) | ((hat & 0x0F) << 2)
    frame[13] = 0x00
    frame[14] = 0x00
    return bytes(frame)


def build_inject(hid: bytes) -> bytes:
    """调试注入帧: 78 | 15B | BCC | 7D"""
    if len(hid) != 15:
        raise ValueError("hid frame must be 15 bytes, got %d" % len(hid))
    body = bytes([INJECT_HEAD]) + hid
    return body + bytes([xor_bcc(body), TAIL])


def build_set_mode(mode: int) -> bytes:
    body = bytes([0x7A, 0x02, mode & 0x03, 0x00, 0x00, 0x00])
    return body + bytes([xor_bcc(body), TAIL])


# ---------------------------------------------------------------- 解码
def decode_hid(frame: bytes) -> dict:
    x = frame[1] | (frame[2] << 8)
    y = frame[3] | (frame[4] << 8)
    rx = frame[5] | (frame[6] << 8)
    ry = frame[7] | (frame[8] << 8)
    z = frame[9] | (frame[10] << 8)
    buttons = frame[11]
    buttons2 = frame[12] & 0x03
    hat = (frame[12] >> 2) & 0x0F
    return {
        "x": x, "y": y, "rx": rx, "ry": ry, "z": z,
        "buttons": buttons, "buttons2": buttons2, "hat": hat,
    }


def deadzone_permille(raw: int, center: int) -> int:
    """和固件 rc.c 的 Rc_StickPermille() 一致的换算（验证映射用）。"""
    diff = raw - center
    permille = int(diff * 1000.0 / 32768.0)
    if -RC_DEADZONE_PERMILLE < permille < RC_DEADZONE_PERMILLE:
        return 0
    if permille < 0:
        magnitude = (-permille - RC_DEADZONE_PERMILLE) * 1000 // (1000 - RC_DEADZONE_PERMILLE)
        return -min(1000, magnitude)
    magnitude = (permille - RC_DEADZONE_PERMILLE) * 1000 // (1000 - RC_DEADZONE_PERMILLE)
    return min(1000, magnitude)


def cmd_from_hid(info: dict) -> tuple:
    """和固件一致: Y 向上=前进(vx+), X 向右=右转(wz-)"""
    sx = deadzone_permille(info["x"], 32768)
    sy = deadzone_permille(info["y"], 32767)
    vx = int(sy * RC_VX_MAX_MM_S / 1000.0)
    wz = int(-sx * RC_WZ_MAX_MRAD_S / 1000.0)
    return sx, sy, vx, wz


BUTTON_NAMES = [(0x01, "A"), (0x02, "B"), (0x04, "X"), (0x08, "Y"),
                (0x10, "LB"), (0x20, "RB"), (0x40, "Back"), (0x80, "Start")]


def buttons_text(info: dict) -> str:
    names = [name for bit, name in BUTTON_NAMES if info["buttons"] & bit]
    if info["buttons2"] & 0x01:
        names.append("LS")
    if info["buttons2"] & 0x02:
        names.append("RS")
    return "+".join(names) if names else "-"


def format_rc_line(info: dict, flags: int = 0) -> str:
    sx, sy, vx, wz = cmd_from_hid(info)
    return ("rc flags=0x%02X  X=%5d Y=%5d Rx=%5d Ry=%5d Z=%5d  hat=%d btn=%-10s"
            "  -> stick=(%4d,%4d) vx=%4d mm/s wz=%5d mrad/s"
            % (flags, info["x"], info["y"], info["rx"], info["ry"], info["z"],
               info["hat"], buttons_text(info), sx, sy, vx, wz))


# ---------------------------------------------------------------- 解析上行
class UplinkParser:
    """从串口字节流里挑出 0x79 遥控帧 和 0x7E 遥测帧。"""

    def __init__(self):
        self.buf = bytearray()
        self.rc_frames = 0
        self.telemetry_frames = 0
        self.bad = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while self.buf:
            head = self.buf[0]
            if head == RC_HEAD:
                need = RC_FRAME_SIZE
            elif head == TELEMETRY_HEAD:
                need = TELEMETRY_SIZE
            else:
                del self.buf[0]
                self.bad += 1
                continue
            if len(self.buf) < need:
                break
            frame = bytes(self.buf[:need])
            if (frame[need - 1] != TAIL) or (xor_bcc(frame[:need - 2]) != frame[need - 2]):
                del self.buf[0]
                self.bad += 1
                continue
            del self.buf[:need]
            if head == RC_HEAD:
                self.rc_frames += 1
                out.append(("rc", frame))
            else:
                self.telemetry_frames += 1
                out.append(("tel", frame))
        return out


def format_telemetry(frame: bytes) -> str:
    def i16(offset):
        value = (frame[offset] << 8) | frame[offset + 1]
        return value - 0x10000 if value & 0x8000 else value

    mode = frame[2] & 0x03
    mode_name = {0: "HOST", 1: "RC", 2: "PUSH"}.get(mode, "?")
    return ("tel fw%d mode=%s vx=%4d wz=%5d acc=(%5d,%5d,%5d)mg gyro=(%d,%d,%d)"
            % (frame[29], mode_name, i16(3), i16(5), i16(7), i16(9), i16(11),
               i16(13), i16(15), i16(17)))


def listen(port: serial.Serial, seconds: float) -> None:
    parser = UplinkParser()
    deadline = time.time() + seconds
    port.timeout = 0.05
    last_rc_print = 0.0
    while time.time() < deadline:
        chunk = port.read(256)
        if not chunk:
            continue
        for kind, frame in parser.feed(chunk):
            if kind == "rc":
                info = decode_hid(frame[2:17])
                # 20Hz 太快, 每 100ms 打一行就够看
                now = time.time()
                if now - last_rc_print > 0.1:
                    last_rc_print = now
                    print(format_rc_line(info, frame[1]))
            else:
                print(format_telemetry(frame))
    print("--- 统计: rc=%d telemetry=%d bad=%d ---"
          % (parser.rc_frames, parser.telemetry_frames, parser.bad))


# ---------------------------------------------------------------- 动作
def send_inject(port: serial.Serial, hid: bytes, count: int = 1, period: float = 0.02):
    frame = build_inject(hid)
    for _ in range(count):
        port.write(frame)
        port.flush()
        if count > 1:
            time.sleep(period)


def action_center(port, args):
    hid = build_hid_frame()
    print("-> 中位帧   hid: %s" % hexstr(hid))
    print("-> 注入帧   %s" % hexstr(build_inject(hid)))
    send_inject(port, hid, args.count, args.period)
    return 0


def action_send(port, args):
    hid = build_hid_frame(lx=args.lx, ly=args.ly, rx=args.rx, ry=args.ry,
                          lt=args.lt, rt=args.rt,
                          buttons=args.buttons.encode(), hat=args.hat)
    info = decode_hid(hid)
    print("-> 手柄帧   %s" % hexstr(hid))
    print("-> 解析     %s" % format_rc_line(info))
    send_inject(port, hid, args.count, args.period)
    return 0


def action_raw(port, args):
    parts = args.hex.replace(",", " ").split()
    if len(parts) != 15:
        print("需要 15 个字节, 收到 %d 个" % len(parts))
        return 2
    hid = bytes(int(p, 16) for p in parts)
    info = decode_hid(hid)
    print("-> 手柄帧   %s" % hexstr(hid))
    print("-> 解析     %s" % format_rc_line(info))
    send_inject(port, hid, args.count, args.period)
    return 0


def action_play(port, args):
    """摇杆扫动: 先回中, 左摇杆在四个方向各推满 1.5s, 最后回中。"""
    if args.mode is not None:
        frame = build_set_mode(args.mode)
        port.write(frame)
        port.flush()
        print("-> SET_MODE %s  %s"
              % ({0: "HOST", 1: "RC", 2: "PUSH"}[args.mode], hexstr(frame)))
        time.sleep(0.3)

    steps = [
        ("中位", 0, 0),
        ("前推", 0, 1000),
        ("回中", 0, 0),
        ("后拉", 0, -1000),
        ("回中", 0, 0),
        ("右推", 1000, 0),
        ("回中", 0, 0),
        ("左推", -1000, 0),
        ("中位", 0, 0),
    ]
    each = max(0.2, args.seconds / len(steps))
    period = args.period
    for name, lx, ly in steps:
        hid = build_hid_frame(lx=lx, ly=ly)
        info = decode_hid(hid)
        print("[%s] %s" % (name, format_rc_line(info)))
        deadline = time.time() + each
        while time.time() < deadline:
            port.write(build_inject(hid))
            port.flush()
            time.sleep(period)
    return 0


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(
        description="底盘主控 V2 手柄/遥控链路工具（注入 15B 手柄帧 / 解析 0x79 上行帧）")
    parser.add_argument("port", help="串口, 例如 COM15")
    parser.add_argument("action", choices=["center", "send", "raw", "play"],
                        help="center=中位一帧 send=按参数造一帧 raw=原样发15B play=摇杆扫动")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=1, help="重复发几帧 (默认 1)")
    parser.add_argument("--period", type=float, default=0.02, help="重复发送间隔秒 (默认 0.02)")
    parser.add_argument("--lx", type=int, default=0, help="左摇杆 X, 千分制 -1000..1000")
    parser.add_argument("--ly", type=int, default=0, help="左摇杆 Y, 千分制, 向上为正")
    parser.add_argument("--rx", type=int, default=0)
    parser.add_argument("--ry", type=int, default=0)
    parser.add_argument("--lt", type=int, default=0, help="左扳机 0..255")
    parser.add_argument("--rt", type=int, default=0, help="右扳机 0..255")
    parser.add_argument("--hat", type=int, default=0, help="十字键 0..8")
    parser.add_argument("--buttons", default="", help="按键串, 例如 abxy / start / ls")
    parser.add_argument("--a", action="store_true", help="等价于 --buttons a")
    parser.add_argument("--hex", default="", help="raw 动作的 15 字节, 空格或逗号分隔")
    parser.add_argument("--mode", type=int, default=None,
                        help="play 前先切模式: 0=HOST 1=RC 2=PUSH")
    parser.add_argument("--seconds", type=float, default=6.0, help="play 总时长秒")
    parser.add_argument("--listen", action="store_true", help="结束后解析上行帧")
    parser.add_argument("--listen-seconds", type=float, default=3.0)
    args = parser.parse_args()

    if args.a:
        args.buttons = args.buttons or "a"

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print("打不开串口 %s: %s" % (args.port, exc))
        return 1

    try:
        port.reset_input_buffer()
        if args.action == "center":
            rc = action_center(port, args)
        elif args.action == "send":
            rc = action_send(port, args)
        elif args.action == "raw":
            rc = action_raw(port, args)
        else:
            rc = action_play(port, args)

        if args.listen > 0 or args.listen:
            print("--- 解析上行帧 (%.1f s) ---" % args.listen_seconds)
            listen(port, args.listen_seconds)
    finally:
        port.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
