"""通过调试器把一帧 15 字节手柄数据塞进 rc.c 的环形缓冲, 验证解析 + 映射.

不需要 CH554G / 串口: 直接写 s_ring + s_ring_head, 主循环 Rc_Poll() 会照常取出来解码。
关键: 在线判定 300ms 超时, 所以"注入 -> 等待 -> 读回"必须在同一个 openocd 会话里做完
(两次调用之间光启动就要 1 秒, 早就超时了)。
"""
import re
import subprocess
import sys

OC = r"D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe"

G_MODE = 0x20000014
RING_HEAD = 0x2000002C
RING_TAIL = 0x2000002E
RING = 0x200000AE
HAVE_FRAME = 0x2000003C
FRAMES = 0x2000003E
VX = 0x20000046
WZ = 0x20000048
STATE = 0x200001DC

TELEMETRY = 0x2000008E

READ_CMDS = ["mdw 0x20000014 1", "mdw 0x2000002c 2", "mdw 0x2000003c 4",
             "mdw 0x200001dc 4", "mdw 0x20000044 2", "mdw 0x2000008c 9"]


def run(cmds):
    args = [OC, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32f1x.cfg",
            "-c", "adapter speed 500", "-c", "init", "-c", "halt"]
    for cmd in cmds:
        args += ["-c", cmd]
    args += ["-c", "resume", "-c", "shutdown"]
    out = subprocess.run(args, capture_output=True, text=True, errors="replace")
    return out.stdout + out.stderr


def parse(text):
    words = {}
    for line in text.splitlines():
        m = re.match(r"0x([0-9a-f]{8}):\s+((?:[0-9a-f]{8}\s*)+)", line.strip())
        if not m:
            continue
        base = int(m.group(1), 16)
        for index, chunk in enumerate(m.group(2).split()):
            words[base + index * 4] = int(chunk, 16)
    return words


def byte(words, address):
    return (words.get(address & ~3, 0) >> ((address & 3) * 8)) & 0xFF


def u16(words, address):
    return byte(words, address) | (byte(words, address + 1) << 8)


def i16(words, address):
    value = u16(words, address)
    return value - 0x10000 if value & 0x8000 else value


def report(tag, text):
    words = parse(text)
    state = [byte(words, STATE + i) for i in range(14)]
    telemetry = bytes(byte(words, TELEMETRY + i) for i in range(32))
    print("%s: mode=%d head=%d tail=%d have=%d frames=%d vx=%d wz=%d" % (
        tag, byte(words, G_MODE), u16(words, RING_HEAD), u16(words, RING_TAIL),
        byte(words, HAVE_FRAME), u16(words, FRAMES), i16(words, VX), i16(words, WZ)))
    print("      state x=%d y=%d rx=%d ry=%d z=%d btn=0x%02X btn2=0x%02X hat=%d valid=%d" % (
        state[0] | (state[1] << 8), state[2] | (state[3] << 8),
        state[4] | (state[5] << 8), state[6] | (state[7] << 8),
        state[8] | (state[9] << 8), state[10], state[11], state[12], state[13]))
    if telemetry[0] == 0x7E and telemetry[31] == 0x7D:
        vx = int.from_bytes(telemetry[3:5], "big", signed=True)
        wz = int.from_bytes(telemetry[5:7], "big", signed=True)
        print("      telemetry mode=%d v_cmd=(%d, %d) flags=0x%02X"
              % (telemetry[2], vx, wz, telemetry[1]))
    else:
        print("      telemetry: no valid frame (%s)" % telemetry[:4].hex())


def main():
    mode = 1
    if len(sys.argv) > 1:
        mode = int(sys.argv[1])

    # 左摇杆推满向前: X 中位, Y 最大; 右下角再测一次 wz
    frames = {
        "forward": bytes([0x00, 0x00, 0x80, 0xFF, 0xFF, 0x00, 0x80, 0xFF, 0x7F,
                          0x00, 0x80, 0x00, 0x00, 0x00, 0x00]),
        "right":   bytes([0x00, 0xFF, 0xFF, 0x00, 0x80, 0x00, 0x80, 0xFF, 0x7F,
                          0x00, 0x80, 0x00, 0x00, 0x00, 0x00]),
        "center":  bytes([0x00, 0x00, 0x80, 0xFF, 0x7F, 0x00, 0x80, 0xFF, 0x7F,
                          0x00, 0x80, 0x00, 0x00, 0x00, 0x00]),
        # 按 A + 十字键"右"(hat=3 → byte12 bit2..5=3 → 0x0C)
        "a+hat_r": bytes([0x00, 0x00, 0x80, 0xFF, 0x7F, 0x00, 0x80, 0xFF, 0x7F,
                          0x00, 0x80, 0x01, 0x0C, 0x00, 0x00]),
        # 扳机 LT 全按(RT=0): z = 32768 + 255*128 = 65408
        "lt_full": bytes([0x00, 0x00, 0x80, 0xFF, 0x7F, 0x00, 0x80, 0xFF, 0x7F,
                          0x80, 0xFF, 0x00, 0x00, 0x00, 0x00]),
    }

    for name in ("forward", "right", "center", "a+hat_r", "lt_full"):
        hid = frames[name]
        cmds = ["mwb 0x%08X 0x%02X" % (G_MODE, mode),
                "mwh 0x%08X 0x0000" % RING_HEAD,
                "mwh 0x%08X 0x0000" % RING_TAIL]
        for index, value in enumerate(hid):
            cmds.append("mwb 0x%08X 0x%02X" % (RING + index, value))
        cmds.append("mwh 0x%08X 0x%04X" % (RING_HEAD, len(hid)))
        cmds.append("resume")
        cmds.append("sleep 150")
        cmds.append("halt")
        cmds.extend(READ_CMDS)
        text = run(cmds)
        print("--- %s: %s" % (name, " ".join("%02X" % b for b in hid)))
        report("   ", text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
