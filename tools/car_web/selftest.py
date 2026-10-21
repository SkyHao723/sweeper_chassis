#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_web.py --sim 离线自测。

覆盖断言（共 45 项）：
  * 原有 22 项：页面/静态资源、24/44 字节帧解析、BCC、档位夹取、轮速跟随、
    里程、急停锁存/复位、500 ms 心跳超时、非法档位、NaN、WebSocket 断开清零、
    真实浏览器无 JS 异常 / 无横向溢出 / canvas 有绘制。
  * 方向/斜坡基础（23~33）：
    23 右推指令 → 真正发给串口的 wz < 0
    24 11 字节命令帧字段为 -500 mrad
    25 真实浏览器：已连接、无横向溢出、canvas 有绘制、无 JS 异常、6 档按钮
    26 真实浏览器右推摇杆 → wz<0；松手 → 目标 0，软停后回 0
    27 UI 开关能设置/查询缓启缓停（默认开→关→开）
    28 缓启缓停开：0→0.3 m/s 约 100 ms 时实发在 (0, 0.3)
    29 斜坡最终到 0.3 m/s
    30 关闭斜坡：指令立即阶跃到目标
    31 斜坡开时急停仍立即归零
    32 斜坡开时心跳超时仍立即归零
    33 斜坡开时 /api/stop {"hard":true}（页面隐藏/失焦）立即归零
  * 新增软停/高速档（34~39）：
    34 软停：0.3 m/s 松手约 100 ms 时实发仍 >0 且 <0.3（证明在减速）
    35 软停：最终到 0，用时符合 RAMP_VX_DEC（0.3 / 0.40 ≈ 0.75 s）
    36 斜坡开、0.8 m/s 行驶时急停仍立即归零（不按 2 s 的软停斜率）
    37 斜坡开、0.8 m/s 行驶时心跳超时仍立即归零
    38 最高档 ultra：实发 vx = 1.60 m/s，且不超过硬夹 2.0 / 固件 2.565
    39 最高档 ultra：实发 wz = 1.50 rad/s，且轮速不超固件 30 rad/s
  * 新增信息显示（40~45）：
    40 版本号 1.3.0；IMU 加速度模长 |a|≈9.81（由 24B 帧 ax/ay/az 计算）
    41 44B 新字段齐全且合理：位置/偏差/跟随度/温度分级/反馈年龄/CAN/启动阶段
    42 ERR 中文映射完整（0/1 正常，5~E 按达妙错误码翻译）
    43 颜色分级阈值：温度 >70 橙 / >85 红；反馈年龄 >200 橙 / >500 红
    44 390px 移动端无横向溢出，左右轮详情 + IMU 模长 + 反馈年龄都在
    45 页面新增值真实渲染（年龄 ms、温度、ERR 使能、|a|≈9.81、版本 1.3.0）

用法（Windows / Linux 均可，只需要 Python 标准库）：
    cd tools/car_web
    python selftest.py                 # 自动找端口、--sim、跑完自动退出
    python selftest.py --no-browser    # 没有浏览器的机器上跳过 25/26/27

退出码 0 = 全部 PASS（或有显式 SKIP），1 = 有 FAIL。
"""

from __future__ import annotations

import base64
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
CAR_WEB = HERE / "car_web.py"

# 为了直接引用 RAMP_VX_DEC / HARD_VX_LIMIT 等常量做断言（导入无副作用）。
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import car_web  # noqa: E402


# ---------------------------------------------------------------------------
# 断言记录
# ---------------------------------------------------------------------------
class Checker:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.failures: list[str] = []

    def check(self, num: int, name: str, cond: bool, detail: str = "") -> bool:
        ok = bool(cond)
        if ok:
            self.passed += 1
            print("[PASS] %02d %s" % (num, name))
        else:
            self.failed += 1
            self.failures.append("%02d %s %s" % (num, name, detail))
            print("[FAIL] %02d %s  %s" % (num, name, detail))
        return ok

    def skip(self, num: int, name: str, why: str) -> None:
        self.skipped += 1
        print("[SKIP] %02d %s  (%s)" % (num, name, why))


# ---------------------------------------------------------------------------
# HTTP 小工具
# ---------------------------------------------------------------------------
def _request(base: str, path: str, data: bytes | None = None,
             ctype: str = "application/json", timeout: float = 5.0):
    headers = {"Content-Type": ctype} if data is not None else {}
    req = urllib.request.Request(base + path, data=data, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.headers.get("Content-Type", ""), r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get("Content-Type", ""), e.read()


def get_text(base: str, path: str) -> tuple[int, str, str]:
    code, ctype, body = _request(base, path)
    return code, ctype, body.decode("utf-8", "replace")


def get_json(base: str, path: str) -> dict:
    code, _ctype, body = _request(base, path)
    if code != 200:
        raise RuntimeError("GET %s -> HTTP %d: %s" % (path, code, body[:200]))
    return json.loads(body.decode("utf-8"))


def post_json(base: str, path: str, obj: dict) -> tuple[int, dict]:
    data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    code, _ctype, body = _request(base, path, data)
    try:
        parsed = json.loads(body.decode("utf-8")) if body else {}
    except ValueError:
        parsed = {"_raw": body.decode("utf-8", "replace")}
    return code, parsed


def post_raw(base: str, path: str, raw: str) -> tuple[int, dict]:
    code, _ctype, body = _request(base, path, raw.encode("utf-8"))
    try:
        parsed = json.loads(body.decode("utf-8")) if body else {}
    except ValueError:
        parsed = {"_raw": body.decode("utf-8", "replace")}
    return code, parsed


def wait_state(base: str, pred, timeout: float = 5.0,
               interval: float = 0.02) -> dict:
    deadline = time.monotonic() + timeout
    last: dict = {}
    while time.monotonic() < deadline:
        try:
            last = get_json(base, "/api/state")
        except Exception:
            time.sleep(interval)
            continue
        if pred(last):
            return last
        time.sleep(interval)
    return last


def hold_cmd(base: str, vx: float, wz: float, seconds: float,
             interval: float = 0.07) -> None:
    """持续发同一个指令（保住 500 ms 心跳），持续 seconds 秒。"""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        post_json(base, "/api/cmd", {"vx": vx, "wz": wz})
        time.sleep(interval)


# ---------------------------------------------------------------------------
# 极简 WebSocket 客户端（浏览器 CDP 和 /ws 都用它）
# ---------------------------------------------------------------------------
class WSClient:
    def __init__(self, host: str, port: int, path: str, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (
            "GET %s HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
            % (path, host, port, key)
        )
        self.sock.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("ws handshake closed")
            buf += chunk
        head, rest = buf.split(b"\r\n\r\n", 1)
        status = head.split(b"\r\n", 1)[0]
        if b" 101 " not in status:
            raise ConnectionError("ws handshake failed: %r" % status)
        self.buf = bytearray(rest)

    def _readn(self, n: int) -> bytes:
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("ws peer closed")
            self.buf.extend(chunk)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def _send_frame(self, opcode: int, payload: bytes = b"") -> None:
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            header = bytes((0x80 | opcode, 0x80 | n))
        elif n < 65536:
            header = bytes((0x80 | opcode, 0x80 | 126)) + struct.pack(">H", n)
        else:
            header = bytes((0x80 | opcode, 0x80 | 127)) + struct.pack(">Q", n)
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(header + mask + masked)

    def send_text(self, text: str) -> None:
        self._send_frame(0x1, text.encode("utf-8"))

    def send_json(self, obj: dict) -> None:
        self.send_text(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))

    def recv_frame(self, timeout: float = 5.0) -> tuple[int, bytes]:
        self.sock.settimeout(timeout)
        b1, b2 = self._readn(2)
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._readn(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._readn(8))[0]
        mask = self._readn(4) if masked else None
        payload = self._readn(length) if length else b""
        if mask:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        return opcode, payload

    def recv_json(self, timeout: float = 5.0) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                opcode, payload = self.recv_frame(
                    max(0.05, deadline - time.monotonic()))
            except (socket.timeout, TimeoutError):
                continue
            if opcode == 0x1:
                return json.loads(payload.decode("utf-8"))
            if opcode == 0x8:
                raise ConnectionError("ws close")
            if opcode == 0x9:
                self._send_frame(0xA, payload)
        raise TimeoutError("ws recv timeout")

    def close(self) -> None:
        try:
            self._send_frame(0x8, b"")
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Chrome DevTools Protocol
# ---------------------------------------------------------------------------
class CDP:
    def __init__(self, ws_url: str):
        u = urllib.parse.urlparse(ws_url)
        if not u.hostname or not u.port:
            raise RuntimeError("bad CDP URL: %s" % ws_url)
        self.ws = WSClient(u.hostname, u.port, u.path or "/", timeout=5.0)
        self.next_id = 1
        self.events: list[dict] = []

    def call(self, method: str, params: dict | None = None,
             timeout: float = 10.0) -> dict:
        mid = self.next_id
        self.next_id += 1
        self.ws.send_json({"id": mid, "method": method, "params": params or {}})
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = self.ws.recv_json(max(0.05, deadline - time.monotonic()))
            except (TimeoutError, socket.timeout):
                continue
            if msg.get("id") == mid:
                if "error" in msg:
                    raise RuntimeError("CDP %s: %s" % (method, msg["error"]))
                return msg.get("result", {})
            self.events.append(msg)
        raise TimeoutError("CDP %s timeout" % method)

    def wait_event(self, method: str, timeout: float = 10.0) -> dict:
        for i, ev in enumerate(self.events):
            if ev.get("method") == method:
                return self.events.pop(i)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = self.ws.recv_json(max(0.05, deadline - time.monotonic()))
            except (TimeoutError, socket.timeout):
                continue
            if msg.get("method") == method:
                return msg
            self.events.append(msg)
        raise TimeoutError("CDP event %s timeout" % method)

    def eval(self, expr: str, timeout: float = 5.0):
        res = self.call("Runtime.evaluate", {
            "expression": expr,
            "returnByValue": True,
            "awaitPromise": True,
        }, timeout=timeout)
        if "exceptionDetails" in res:
            raise RuntimeError("JS exception: %s" % res["exceptionDetails"])
        return res.get("result", {}).get("value")

    def close(self) -> None:
        try:
            self.ws.close()
        except Exception:
            pass


def find_browser() -> str | None:
    cands = []
    for env in ("ProgramFiles(x86)", "ProgramFiles", "LOCALAPPDATA"):
        root = os.environ.get(env)
        if not root:
            continue
        cands += [
            os.path.join(root, "Microsoft", "Edge", "Application", "msedge.exe"),
            os.path.join(root, "Google", "Chrome", "Application", "chrome.exe"),
        ]
    cands += ["/usr/bin/google-chrome", "/usr/bin/chromium",
              "/usr/bin/chromium-browser", "/usr/bin/microsoft-edge"]
    for p in cands:
        if os.path.isfile(p):
            return p
    for name in ("msedge", "chrome", "chromium", "google-chrome"):
        p = shutil.which(name)
        if p:
            return p
    return None


class BrowserSession:
    """启动一个 headless 浏览器，连到本机 car_web 页面。

    width/height/mobile 用来做移动端断言（390px 不允许横向溢出）。
    """

    def __init__(self, base_url: str, width: int = 1280, height: int = 900,
                 mobile: bool = False):
        self.base = base_url
        self.width = width
        self.height = height
        self.mobile = mobile
        self.proc: subprocess.Popen | None = None
        self.cdp: CDP | None = None
        self.profile = ""
        self.port = 0

    def start(self, timeout: float = 15.0) -> None:
        exe = find_browser()
        if not exe:
            raise RuntimeError("找不到 Edge/Chrome")
        self.port = free_port()
        self.profile = tempfile.mkdtemp(prefix="carweb-browser-")
        args = [
            exe, "--headless=new", "--disable-gpu", "--no-first-run",
            "--no-default-browser-check", "--disable-extensions",
            "--hide-scrollbars", "--mute-audio",
            "--remote-debugging-port=%d" % self.port,
            "--user-data-dir=%s" % self.profile,
            "about:blank",
        ]
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + timeout
        targets = None
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("浏览器启动后退出了（exit=%s）" % self.proc.returncode)
            try:
                code, _ctype, body = _request(
                    "http://127.0.0.1:%d" % self.port, "/json/list",
                    timeout=1.0)
                if code == 200:
                    targets = json.loads(body.decode("utf-8"))
                    if targets:
                        break
            except Exception:
                pass
            time.sleep(0.1)
        if not targets:
            raise TimeoutError("浏览器 DevTools 端口没起来")
        page = next((t for t in targets if t.get("type") == "page"), None)
        if page is None:
            raise RuntimeError("/json/list 里没有 page target")
        self.cdp = CDP(page["webSocketDebuggerUrl"])
        self.cdp.call("Runtime.enable")
        self.cdp.call("Log.enable")
        self.cdp.call("Page.enable")
        self.cdp.call("Emulation.setDeviceMetricsOverride", {
            "width": self.width, "height": self.height, "deviceScaleFactor": 1,
            "mobile": self.mobile,
        })
        self.cdp.call("Page.navigate", {"url": self.base + "/"})
        self.cdp.wait_event("Page.loadEventFired", timeout=15.0)

    def close(self) -> None:
        if self.cdp is not None:
            self.cdp.close()
            self.cdp = None
        if self.proc is not None:
            try:
                if os.name == "nt":
                    subprocess.run(
                        ["taskkill", "/F", "/T", "/PID", str(self.proc.pid)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=10)
                else:
                    self.proc.terminate()
                    self.proc.wait(timeout=5)
            except Exception:
                pass
            self.proc = None
        if self.profile:
            shutil.rmtree(self.profile, ignore_errors=True)
            self.profile = ""


# ---------------------------------------------------------------------------
# 端口 / 进程
# ---------------------------------------------------------------------------
def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
    finally:
        s.close()


def start_server() -> tuple[subprocess.Popen, str, str, Path]:
    port = free_port()
    base = "http://127.0.0.1:%d" % port
    log_path = Path(tempfile.gettempdir()) / ("carweb-selftest-%d.log" % port)
    log = open(log_path, "wb")
    proc = subprocess.Popen(
        [sys.executable, str(CAR_WEB), "--sim", "--host", "127.0.0.1",
         "--port", str(port)],
        cwd=str(HERE), stdout=log, stderr=subprocess.STDOUT)
    log.close()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError("car_web.py 启动失败（exit=%s）\n%s"
                               % (proc.returncode, read_log(log_path)))
        try:
            st = get_json(base, "/api/state")
            if st.get("ok") and st.get("link", {}).get("open"):
                return proc, base, "127.0.0.1", log_path
        except Exception:
            pass
        time.sleep(0.1)
    raise TimeoutError("car_web.py 15 s 内没起来\n%s" % read_log(log_path))


def read_log(path: Path) -> str:
    try:
        return path.read_text("utf-8", "replace")[-4000:]
    except Exception:
        return "(日志读不到)"


def stop_server(proc: subprocess.Popen) -> None:
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 主测试
# ---------------------------------------------------------------------------
def run(checker: Checker, base: str, args) -> None:
    # ---------------- 原有 1~8：页面 / 帧 / BCC / IMU ----------------
    code, ctype, html = get_text(base, "/")
    checker.check(1, "首页 200、UTF-8 HTML、含标题",
                  code == 200 and "text/html" in ctype
                  and "小车遥控器" in html,
                  "code=%s ctype=%s" % (code, ctype))

    code, ctype, css = get_text(base, "/static/style.css")
    checker.check(2, "style.css 200、text/css、含深色变量",
                  code == 200 and "text/css" in ctype and "--accent" in css,
                  "code=%s ctype=%s" % (code, ctype))

    code, ctype, js = get_text(base, "/static/app.js")
    checker.check(3, "app.js 200、JS、含唯一符号常量 WZ_CMD_SIGN=-1",
                  code == 200 and "javascript" in ctype
                  and "WZ_CMD_SIGN = -1" in js
                  and "function steerWz" in js,
                  "code=%s ctype=%s" % (code, ctype))

    st = get_json(base, "/api/state")
    checker.check(4, " /api/state ok、SIM、链路已打开",
                  st.get("ok") is True and st.get("sim") is True
                  and st.get("link", {}).get("open") is True,
                  json.dumps(st.get("link", {}), ensure_ascii=False)[:200])

    st = wait_state(base, lambda s: s["link"]["telemetry_count"] >= 5, 5.0)
    tel = st.get("telemetry") or {}
    checker.check(5, "24 字节主遥测帧解析（vx/wz/IMU 六轴）",
                  st["link"]["telemetry_count"] >= 5
                  and all(k in tel for k in
                          ("vx", "vy", "wz", "ax", "ay", "az",
                           "gx", "gy", "gz", "bat_mv")),
                  "tel=%s" % (sorted(tel.keys())[:8],))

    st = wait_state(base, lambda s: s["link"]["status_count"] >= 5, 5.0)
    status = st.get("status") or {}
    checker.check(6, "44 字节状态帧解析（左右轮/flags/启动阶段）",
                  st["link"]["status_count"] >= 5
                  and "left" in status and "right" in status
                  and "flags" in status and "boot_stage_text" in status,
                  "status keys=%s" % (sorted(status.keys())[:8],))

    # 启动后的第一帧前面没有 0x7D 前导（协议要求"帧尾+帧头"同步），
    # 解析器会丢掉这 24 字节，这是正常的一次性重同步，不是坏帧。
    checker.check(7, "BCC/帧同步无坏帧（启动期最多丢首帧 24 B）",
                  st["link"]["bad_frames"] == 0
                  and st["link"]["resync_bytes"] <= 24,
                  "bad=%s resync=%s" % (st["link"]["bad_frames"],
                                        st["link"]["resync_bytes"]))

    # 静止时 az≈9.8、vx≈0
    checker.check(8, "IMU 静止 az≈9.8、vx≈0",
                  tel and 8.5 <= tel["az"] <= 11.0 and abs(tel["vx"]) < 0.05,
                  "az=%s vx=%s" % (tel.get("az"), tel.get("vx")))

    # ---------------- 9~11：档位 / 轮速 / 里程 ----------------
    post_json(base, "/api/gear", {"gear": "low"})
    post_json(base, "/api/cmd", {"vx": 0.5, "wz": 1.0})
    st = wait_state(base, lambda s: s["cmd"]["has_client"], 2.0)
    cmd = st["cmd"]
    checker.check(9, "低速档把 vx=0.5/wz=1.0 夹到 0.15/0.50",
                  abs(cmd["target_vx"] - 0.15) < 1e-6
                  and abs(cmd["target_wz"] - 0.50) < 1e-6,
                  "target=(%s,%s)" % (cmd["target_vx"], cmd["target_wz"]))
    post_json(base, "/api/stop", {})
    time.sleep(0.65)

    # 中速直行：等斜坡到 0.3，再看轮速跟随
    post_json(base, "/api/gear", {"gear": "mid"})
    post_json(base, "/api/ramp", {"on": True})
    odom_before = get_json(base, "/api/state")["odom"]["distance"]
    hold_cmd(base, 0.3, 0.0, 1.0)
    st = wait_state(base, lambda s: abs(s["cmd"]["tx_vx"] - 0.3) < 0.01, 1.0)
    status = st["status"]
    checker.check(10, "左右轮实际 ω 跟随目标（前进为负、直行两轮接近）",
                  status["left"]["omega"] < -0.5
                  and status["right"]["omega"] < -0.5
                  and abs(status["left"]["omega"] - status["right"]["omega"]) < 1.0
                  and abs(status["left"]["target"] - status["left"]["omega"]) < 2.0,
                  "L=%s/%s R=%s/%s" % (
                      status["left"]["omega"], status["left"]["target"],
                      status["right"]["omega"], status["right"]["target"]))
    odom_after = get_json(base, "/api/state")["odom"]["distance"]
    checker.check(11, "里程用实际轮速积分并增长",
                  odom_after - odom_before > 0.02,
                  "before=%s after=%s" % (odom_before, odom_after))
    post_json(base, "/api/stop", {})
    time.sleep(0.65)

    # ---------------- 12~16：急停锁存 / 复位 ----------------
    post_json(base, "/api/estop", {})
    st = get_json(base, "/api/state")
    checker.check(12, "急停锁存 safety.estop=true",
                  st["safety"]["estop"] is True, str(st["safety"]))
    post_json(base, "/api/cmd", {"vx": 0.3, "wz": 0.0})
    st = wait_state(base, lambda s: s["cmd"]["reason"] == "estop", 1.0)
    checker.check(13, "急停期间再发指令仍然只发 0",
                  st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0
                  and st["cmd"]["reason"] == "estop",
                  str(st["cmd"]))
    st = wait_state(base, lambda s: s["telemetry"]
                    and abs(s["telemetry"]["vx"]) < 0.05, 2.0)
    checker.check(14, "急停后 500 ms 内车实际停住",
                  st["telemetry"] and abs(st["telemetry"]["vx"]) < 0.05,
                  "vx=%s" % (st["telemetry"] and st["telemetry"]["vx"]))
    post_json(base, "/api/estop/reset", {})
    st = wait_state(base, lambda s: s["safety"]["estop"] is False, 1.0)
    checker.check(15, "急停复位后目标仍为 0（timeout / 实发 0）",
                  st["safety"]["estop"] is False
                  and st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0,
                  str(st["cmd"]))
    post_json(base, "/api/cmd", {"vx": 0.1, "wz": 0.0})
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.05, 2.0)
    checker.check(16, "复位后必须重新给指令才动",
                  st["cmd"]["tx_vx"] > 0.05, str(st["cmd"]))
    post_json(base, "/api/stop", {})
    time.sleep(0.65)

    # ---------------- 17~18：心跳 / 超时 ----------------
    post_json(base, "/api/cmd", {"vx": 0.1, "wz": 0.0})
    st = wait_state(base, lambda s: s["cmd"]["has_client"], 1.0)
    time.sleep(0.2)
    st = get_json(base, "/api/state")
    checker.check(17, "200 ms 内有指令：has_client=true、reason=web",
                  st["cmd"]["has_client"] is True
                  and st["cmd"]["reason"] == "web"
                  and 100 <= st["cmd"]["age_ms"] <= 450,
                  str(st["cmd"]))
    time.sleep(0.7)
    st = get_json(base, "/api/state")
    checker.check(18, "700 ms 后心跳超时：reason=timeout、实发 0",
                  st["cmd"]["reason"] == "timeout"
                  and st["cmd"]["tx_vx"] == 0
                  and st["cmd"]["tx_wz"] == 0
                  and st["cmd"]["has_client"] is False,
                  str(st["cmd"]))

    # ---------------- 19~20：非法档位 / NaN ----------------
    code, _ = post_json(base, "/api/gear", {"gear": "turbo"})
    checker.check(19, "非法档位返回 400", code == 400, "code=%s" % code)

    code, _ = post_raw(base, "/api/cmd", '{"vx": NaN, "wz": Infinity}')
    st = wait_state(base, lambda s: s["cmd"]["recv_count"] > 0, 1.0)
    time.sleep(0.1)
    st = get_json(base, "/api/state")
    checker.check(20, "NaN/Infinity 不污染控制（目标归零、实发有限）",
                  code == 200 and st["cmd"]["vx"] == 0.0
                  and st["cmd"]["wz"] == 0.0
                  and st["cmd"]["tx_vx"] == 0.0
                  and math.isfinite(st["cmd"]["tx_vx"]),
                  "code=%s cmd=%s" % (code, st["cmd"]))
    post_json(base, "/api/stop", {})
    time.sleep(0.65)

    # ---------------- 21~22：WebSocket 握手 / 推送 / 断开清零 ----------------
    ws = WSClient("127.0.0.1", int(base.rsplit(":", 1)[1]), "/ws")
    try:
        msg = None
        for _ in range(50):
            msg = ws.recv_json(timeout=2.0)
            if msg.get("t") == "state":
                break
        checker.check(21, "WebSocket 握手/20 Hz state 推送正常",
                      bool(msg and msg.get("t") == "state"
                           and msg.get("sim") is True),
                      str(msg and msg.get("t")))
        # 让这个客户端成为 owner，然后直接断开
        ws.send_json({"t": "cmd", "vx": 0.1, "wz": 0.0})
        time.sleep(0.15)
    finally:
        ws.close()
    time.sleep(0.3)
    st = get_json(base, "/api/state")
    checker.check(22, "WebSocket 断开后控制端目标清零",
                  st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0
                  and st["cmd"]["reason"] == "timeout",
                  str(st["cmd"]))

    # ---------------- 新增 23~24：方向（协议层）----------------
    post_json(base, "/api/cmd", {"vx": 0.0, "wz": -0.5})
    # 同时等 STM32 回读（44 字节状态帧 20 Hz，比 30 Hz 命令帧慢半拍）
    st = wait_state(base, lambda s: s["cmd"]["tx_wz"] < 0
                    and s["status"] and s["status"]["cmd_wz"] < 0, 1.0)
    checker.check(23, "右推指令（wz=-0.5）真正发到串口仍为负",
                  st["cmd"]["tx_wz"] < 0
                  and st["status"]["cmd_wz"] < 0,
                  "tx=%s echo=%s" % (st["cmd"]["tx_wz"],
                                     st["status"]["cmd_wz"]))
    post_json(base, "/api/stop", {})
    # 11 字节帧字段再独立验证一次（不经过 HTTP 层）
    import car_web  # noqa: PLC0415
    frame = car_web.build_cmd_frame(0.0, -0.5)
    wz_mrad = struct.unpack(">h", frame[7:9])[0]
    checker.check(24, "11 字节命令帧的 wz 字段为 -500 mrad",
                  frame[0] == 0x7B and frame[10] == 0x7D
                  and wz_mrad == -500,
                  "wz_mrad=%s frame=%s" % (wz_mrad, frame.hex()))

    # ---------------- 新增 25~27（浏览器端到端）----------------
    # 25 无 JS 异常/无横向溢出/canvas 有绘制
    # 26 真实摇杆右推 → wz<0，松手 → 0
    # 27 UI 开关 → /api/ramp 默认开→关→开
    if args.no_browser:
        checker.skip(25, "真实浏览器无 JS 异常/横向溢出/canvas 绘制", "--no-browser")
        checker.skip(26, "真实浏览器右推摇杆/松手回 0", "--no-browser")
        checker.skip(27, "UI 开关能设置/查询缓启缓停", "--no-browser")
    else:
        browser = BrowserSession(base)
        try:
            browser.start()
            assert browser.cdp is not None
            # 等前端 WebSocket 连上、拿到第一帧状态
            ok = False
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                ok = bool(browser.cdp.eval(
                    "!!document.getElementById('wsBadge')"
                    " && document.getElementById('wsBadge').classList.contains('ok')"))
                if ok:
                    break
                time.sleep(0.1)
            meta = browser.cdp.eval("""(() => {
              const c = document.getElementById('wave');
              let painted = 0;
              try {
                const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
                for (let i = 3; i < d.length; i += 4) if (d[i]) painted++;
              } catch (e) { painted = -1; }
              return {
                connected: !!document.getElementById('wsBadge')
                  && document.getElementById('wsBadge').classList.contains('ok'),
                overflow: document.documentElement.scrollWidth
                  - document.documentElement.clientWidth,
                canvasW: c.width, canvasH: c.height, painted: painted,
                rampChecked: document.getElementById('rampToggle').checked,
                rampState: document.getElementById('rampState').textContent,
                gearCount: document.querySelectorAll('.gear-btn').length,
                gearActive: (document.querySelector('.gear-btn.active') || {dataset: {}}).dataset.gear || '',
                gearWarnHidden: document.getElementById('gearWarn').hidden,
                cmdVxClass: document.getElementById('cmdVx').className,
                jsErrors: 0
              };
            })()""")
            errors = [e for e in browser.cdp.events
                      if e.get("method") == "Runtime.exceptionThrown"]
            checker.check(25, "真实浏览器：已连接、无溢出、canvas 有绘制、6 档按钮、无 JS 异常",
                          ok and meta and meta["overflow"] <= 1
                          and meta["canvasW"] > 0 and meta["canvasH"] > 0
                          and meta["painted"] > 0 and len(errors) == 0
                          and meta["gearCount"] == 6
                          and meta["gearActive"] == "low"
                          and meta["gearWarnHidden"] is True,
                          "meta=%s errors=%s" % (meta,
                                                 [e.get("params", {}).get("exceptionDetails", {}).get("text")
                                                  for e in errors[:3]]))

            # 25b：真实浏览器往右推摇杆 → 串口回读 wz<0
            browser.cdp.eval("""(() => {
              const pad = document.getElementById('pad');
              const r = pad.getBoundingClientRect();
              pad.dispatchEvent(new PointerEvent('pointerdown', {
                pointerId: 7, pointerType: 'mouse', button: 0, bubbles: true,
                clientX: r.left + r.width * 0.93,
                clientY: r.top + r.height / 2
              }));
              return true;
            })()""")
            time.sleep(0.25)
            st = get_json(base, "/api/state")
            right_ok = (st["cmd"]["wz"] < 0 and st["cmd"]["tx_wz"] < 0
                        and st["status"]["cmd_wz"] < 0)
            # 松手：前端发普通 0（不是 hard），后端目标立即 0，实发按 DEC 软停
            browser.cdp.eval("""(() => {
              const pad = document.getElementById('pad');
              pad.dispatchEvent(new PointerEvent('pointerup', {
                pointerId: 7, pointerType: 'mouse', button: 0, bubbles: true
              }));
              return true;
            })()""")
            st2 = wait_state(base, lambda s: s["cmd"]["wz"] == 0, 1.0)
            # wz 从 -0.50 按 1.5 rad/s² 收回约需 0.33 s；网页 20 Hz 心跳还在，
            # 不会被 500 ms 超时截断。
            st3 = wait_state(base, lambda s: s["cmd"]["tx_wz"] == 0
                             and s["cmd"]["tx_vx"] == 0, 1.5)
            checker.check(26, "真实浏览器右推摇杆 → wz<0；松手 → 目标 0，软停后实发 0",
                          right_ok and st2["cmd"]["wz"] == 0
                          and st3["cmd"]["tx_wz"] == 0
                          and st3["cmd"]["tx_vx"] == 0,
                          "right=%s release=%s" % (st["cmd"], st3["cmd"]))

            # 浏览器里点开关：服务端 /api/ramp 跟着变，且默认是开
            ramp0 = get_json(base, "/api/ramp")
            browser.cdp.eval("document.getElementById('rampToggle').click(); true")
            time.sleep(0.3)
            ramp1 = get_json(base, "/api/ramp")
            browser.cdp.eval("document.getElementById('rampToggle').click(); true")
            time.sleep(0.3)
            ramp2 = get_json(base, "/api/ramp")
            checker.check(27, "UI 开关能设置/查询缓启缓停（默认开→关→开）",
                          ramp0["ramp"]["on"] is True
                          and ramp1["ramp"]["on"] is False
                          and ramp2["ramp"]["on"] is True,
                          "on=%s off=%s on2=%s" % (
                              ramp0["ramp"]["on"], ramp1["ramp"]["on"],
                              ramp2["ramp"]["on"]))
        finally:
            browser.close()
            time.sleep(0.3)

    # ---------------- 新增：斜坡 / 停止语义 ----------------
    post_json(base, "/api/estop/reset", {})
    post_json(base, "/api/gear", {"gear": "mid"})
    post_json(base, "/api/ramp", {"on": True})
    post_json(base, "/api/stop", {})
    time.sleep(0.7)

    # 28：0 → 0.3，100 ms 时应在中间
    post_json(base, "/api/cmd", {"vx": 0.3, "wz": 0.0})
    t0 = time.monotonic()
    first_t = None
    sample = None
    while time.monotonic() - t0 < 1.2:
        st = get_json(base, "/api/state")
        if first_t is None and st["cmd"]["tx_vx"] > 0:
            first_t = time.monotonic()
        if first_t is not None and time.monotonic() - first_t >= 0.10:
            sample = st["cmd"]["tx_vx"]
            break
        # 保住心跳，避免 500 ms 超时
        post_json(base, "/api/cmd", {"vx": 0.3, "wz": 0.0})
        time.sleep(0.02)
    checker.check(28, "缓启缓停开：0→0.3 m/s 约 100 ms 时实发在 (0, 0.3)",
                  sample is not None and 0.0 < sample < 0.3,
                  "sample=%s first_t=%s" % (sample, first_t and round((first_t - t0) * 1000, 1)))

    # 继续到 0.3
    hold_cmd(base, 0.3, 0.0, 0.8)
    st = get_json(base, "/api/state")
    checker.check(29, "斜坡最终到达目标 0.3 m/s",
                  abs(st["cmd"]["tx_vx"] - 0.3) < 0.01
                  and st["ramp"]["active"] is False,
                  str(st["cmd"]))

    # 关闭斜坡：立即阶跃
    post_json(base, "/api/ramp", {"on": False})
    time.sleep(0.12)
    st = get_json(base, "/api/state")
    checker.check(30, "关闭缓启缓停：指令立即阶跃到 0.3",
                  abs(st["cmd"]["tx_vx"] - 0.3) < 1e-6
                  and st["ramp"]["on"] is False,
                  str(st["cmd"]))
    post_json(base, "/api/ramp", {"on": True})

    # 31：斜坡开时急停立即归零
    hold_cmd(base, 0.3, 0.0, 0.6)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.25, 1.0)
    t_estop = time.monotonic()
    post_json(base, "/api/estop", {})
    # 等 STM32 回读变成 0（状态帧 20 Hz，最多 ~50 ms）；若走软停，
    # 从 0.3 降到 0 需要 0.75 s，不可能在 250 ms 内精确回读 0。
    st = wait_state(base, lambda s: s["status"] and s["status"]["cmd_vx"] == 0
                    and s["cmd"]["reason"] == "estop", 0.35)
    elapsed = (time.monotonic() - t_estop) * 1000.0
    checker.check(31, "斜坡开时急停仍立即归零（不按斜率下降）",
                  st["cmd"]["reason"] == "estop"
                  and st["cmd"]["tx_vx"] == 0
                  and st["status"]["cmd_vx"] == 0
                  and elapsed < 250.0,
                  "cmd=%s echo=%s %.0fms" % (st["cmd"],
                                             st["status"]["cmd_vx"], elapsed))
    post_json(base, "/api/estop/reset", {})
    time.sleep(0.1)

    # 32：斜坡开时心跳超时立即归零
    hold_cmd(base, 0.3, 0.0, 0.6)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.25, 1.0)
    time.sleep(0.56)                       # 500 ms 超时 + 2 拍
    st = get_json(base, "/api/state")
    checker.check(32, "斜坡开时心跳超时立即归零（不按斜率下降）",
                  st["cmd"]["reason"] == "timeout"
                  and st["cmd"]["tx_vx"] == 0
                  and st["status"]["cmd_vx"] == 0,
                  "cmd=%s echo=%s" % (st["cmd"], st["status"]["cmd_vx"]))

    # 33：hard:true（页面隐藏/失焦）仍然立即归零 —— 和松手/回中的软停区分开
    hold_cmd(base, 0.3, 0.0, 0.7)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.25, 1.0)
    t_hard = time.monotonic()
    post_json(base, "/api/stop", {"hard": True})
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] == 0
                    and s["status"] and s["status"]["cmd_vx"] == 0, 0.35)
    elapsed = (time.monotonic() - t_hard) * 1000.0
    checker.check(33, "斜坡开时 hard:true（页面隐藏/失焦）立即归零",
                  st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0
                  and st["status"]["cmd_vx"] == 0
                  and st["ramp"]["mode"] == "hard"
                  and elapsed < 250.0,
                  "cmd=%s mode=%s echo=%s %.0fms" % (
                      st["cmd"], st["ramp"]["mode"],
                      st["status"]["cmd_vx"], elapsed))

    # ---------------- 34~35：主动停车 → 减速斜坡（软停）----------------
    # 先把硬停锁存清掉：重新给非零指令即可。目标 0.3 加速 0.3 s 到位。
    hold_cmd(base, 0.3, 0.0, 0.7)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.29, 1.0)
    dec_expected = 0.30 / car_web.RAMP_VX_DEC     # 0.3 / 0.40 = 0.75 s

    # 关键：软停期间必须持续发 0 保住 500 ms 心跳（真实网页松手后就是 20 Hz 发 0）。
    t0 = time.monotonic()
    sample = None
    sample_state = None
    zero_at = None
    st = {}
    while time.monotonic() - t0 < 1.6:
        post_json(base, "/api/stop", {})
        st = get_json(base, "/api/state")
        el = time.monotonic() - t0
        if sample is None and el >= 0.10:
            sample = st["cmd"]["tx_vx"]
            sample_state = st
        if st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0:
            zero_at = el
            break
        time.sleep(0.02)

    checker.check(34, "软停：0.3 m/s 松手约 100 ms 时实发仍 >0 且 <0.3（正在减速）",
                  sample is not None and 0.05 < sample < 0.29
                  and sample_state is not None
                  and sample_state["ramp"]["mode"] == "decel"
                  and sample_state["ramp"]["stopping"] is True,
                  "sample=%s mode=%s" % (
                      sample, sample_state and sample_state["ramp"]["mode"]))

    checker.check(35, "软停最终到 0，用时符合 DEC 斜率（0.3 / 0.40 ≈ 0.75 s）",
                  zero_at is not None
                  and st["cmd"]["tx_vx"] == 0 and st["cmd"]["tx_wz"] == 0
                  and abs(zero_at - dec_expected) <= 0.20
                  and zero_at > 0.50,
                  "zero_at=%s expected=%.2fs" % (
                      zero_at is not None and round(zero_at, 3), dec_expected))

    # ---------------- 36：高速行驶时急停仍立即停 ----------------
    post_json(base, "/api/gear", {"gear": "fast"})       # 0.80 m/s
    hold_cmd(base, 0.8, 0.0, 1.2)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.75, 1.5)
    t_estop = time.monotonic()
    post_json(base, "/api/estop", {})
    st = wait_state(base, lambda s: s["cmd"]["reason"] == "estop"
                    and s["cmd"]["tx_vx"] == 0
                    and s["status"] and s["status"]["cmd_vx"] == 0, 0.35)
    elapsed = (time.monotonic() - t_estop) * 1000.0
    checker.check(36, "0.8 m/s 行驶中急停仍立即归零（软停本需 2.0 s）",
                  st["cmd"]["reason"] == "estop"
                  and st["cmd"]["tx_vx"] == 0
                  and st["status"]["cmd_vx"] == 0
                  and elapsed < 250.0,
                  "cmd=%s echo=%s %.0fms" % (st["cmd"],
                                             st["status"]["cmd_vx"], elapsed))
    post_json(base, "/api/estop/reset", {})
    time.sleep(0.1)

    # ---------------- 37：高速行驶时心跳超时仍立即停 ----------------
    hold_cmd(base, 0.8, 0.0, 1.2)
    st = wait_state(base, lambda s: s["cmd"]["tx_vx"] > 0.75, 1.5)
    time.sleep(0.56)                       # 500 ms 超时 + 2 拍
    st = get_json(base, "/api/state")
    checker.check(37, "0.8 m/s 行驶中心跳超时仍立即归零（软停本需 2.0 s）",
                  st["cmd"]["reason"] == "timeout"
                  and st["cmd"]["tx_vx"] == 0
                  and st["status"]["cmd_vx"] == 0
                  and st["ramp"]["mode"] == "hard",
                  "cmd=%s mode=%s echo=%s" % (
                      st["cmd"], st["ramp"]["mode"], st["status"]["cmd_vx"]))

    # ---------------- 38~39：最高档位与硬上限 ----------------
    post_json(base, "/api/estop/reset", {})
    post_json(base, "/api/gear", {"gear": "ultra"})
    post_json(base, "/api/ramp", {"on": True})
    # 先给非零指令清掉 hard_stop 锁存；目标会被夹到 ultra 的 1.60。
    hold_cmd(base, 5.0, 0.0, 2.0)
    st = wait_state(base, lambda s: s["cmd"]["target_vx"] == 1.6
                    and s["cmd"]["tx_vx"] >= 1.59, 1.5)
    checker.check(38, "最高档 ultra：目标/实发 vx=1.60，且不超硬夹 2.0 与固件 2.565",
                  st["gear"]["key"] == "ultra"
                  and st["gear"]["vx"] == 1.6
                  and st["cmd"]["target_vx"] == 1.6
                  and abs(st["cmd"]["tx_vx"] - 1.6) <= 0.01
                  and car_web.HARD_VX_LIMIT == 2.0
                  and car_web.HARD_VX_LIMIT <= car_web.FIRMWARE_VX_MAX
                  and 1.6 <= car_web.FIRMWARE_VX_MAX,
                  "gear=%s cmd=%s hard=%s fw=%.3f" % (
                      st["gear"], st["cmd"],
                      car_web.HARD_VX_LIMIT, car_web.FIRMWARE_VX_MAX))
    post_json(base, "/api/stop", {"hard": True})
    time.sleep(0.15)

    # wz 上限 1.5 rad/s：按 456 mm 轮距，固件单轮需要 |wz|*TRACK/2/r = 4.00 rad/s，远小于 30。
    hold_cmd(base, 0.0, 9.0, 1.0)
    st = wait_state(base, lambda s: s["cmd"]["target_wz"] == 1.5
                    and abs(s["cmd"]["tx_wz"] - 1.5) <= 0.01, 1.5)
    wheel_need = car_web.HARD_WZ_LIMIT * car_web.TRACK_WIDTH_M / 2.0 / car_web.WHEEL_R_M
    checker.check(39, "最高档 ultra：wz=1.50，轮速需求 %.2f < 固件 30 rad/s" % wheel_need,
                  st["cmd"]["target_wz"] == 1.5
                  and abs(st["cmd"]["tx_wz"] - 1.5) <= 0.01
                  and car_web.HARD_WZ_LIMIT == 1.5
                  and wheel_need < car_web.WHEEL_OMEGA_MAX
                  and st["status"] is not None
                  and abs(st["status"]["left"]["target"]) < car_web.WHEEL_OMEGA_MAX
                  and abs(st["status"]["right"]["target"]) < car_web.WHEEL_OMEGA_MAX,
                  "cmd=%s wheel_need=%.2f echo L/R=%s/%s" % (
                      st["cmd"], wheel_need,
                      st["status"] and st["status"]["left"]["target"],
                      st["status"] and st["status"]["right"]["target"]))
    post_json(base, "/api/stop", {"hard": True})
    post_json(base, "/api/gear", {"gear": "low"})

    # ---------------- 40~43：新增信息显示的数据来源 / 分级 ----------------
    # 先等车停稳，加速度模长才是"静止 ≈9.81"（行驶中 ax 含向心加速度）。
    time.sleep(0.6)
    st = wait_state(base, lambda s: s.get("imu") is not None
                    and s["imu"].get("accel_mag") is not None
                    and s["cmd"]["tx_vx"] == 0, 2.0)
    imu = st.get("imu") or {}
    checker.check(
        40, "版本 1.3.0；IMU 加速度模长 |a|≈9.81（由 24B 帧 ax/ay/az 计算）",
        st.get("version") == "1.3.0"
        and 8.5 <= imu.get("accel_mag", 0.0) <= 11.0
        and abs(imu.get("tilt_deg", 99.0)) < 5.0
        and imu.get("gyro_mag", 99.0) < 0.5
        and (imu.get("probe") or {}).get("version") is True,
        "version=%s imu=%s" % (st.get("version"), imu))

    status = st.get("status") or {}
    lw = status.get("left") or {}
    rw = status.get("right") or {}
    wheel_fields = ("omega", "target", "dev", "follow_pct", "torque", "pos",
                    "err", "err_text", "fault", "mos_temp", "coil_temp",
                    "mos_level", "coil_level", "fb_age_ms", "age_level")
    checker.check(
        41, "44B 新字段齐全且数值合理（位置/偏差/跟随/温度分级/反馈年龄/CAN/启动）",
        all(k in lw and k in rw for k in wheel_fields)
        and lw.get("fb_age_ms") == 20 and rw.get("fb_age_ms") == 20
        and 20 <= lw.get("mos_temp", 0) <= 85
        and 20 <= lw.get("coil_temp", 0) <= 85
        and lw.get("coil_temp") == lw.get("mos_temp") + 2
        and abs(lw.get("dev", 99.0)) <= 1.0
        and abs(rw.get("dev", 99.0)) <= 1.0
        and lw.get("age_level") == "ok"
        and lw.get("mos_level") == "ok"
        and lw.get("follow_pct") is None
        and math.isfinite(lw.get("pos", float("nan")))
        and math.isfinite(lw.get("torque", float("nan")))
        and status.get("can_fail") == 0 and status.get("fb_count", 0) > 0
        and status.get("boot_stage") == 1
        and status.get("boot_stage_text") == "正常"
        and status.get("boot_level") == "ok",
        "L=%s R=%s can=%s/%s boot=%s" % (
            {k: lw.get(k) for k in wheel_fields},
            {k: rw.get(k) for k in wheel_fields},
            status.get("can_fail"), status.get("fb_count"),
            status.get("boot_stage_text")))

    expected_err = {
        0: "失能", 1: "使能", 5: "读取校准数据错误", 6: "读取电机参数错误",
        7: "传感器读取异常", 8: "超压", 9: "欠压", 0xA: "过电流",
        0xB: "MOS 过温", 0xC: "线圈过温", 0xD: "通讯丢失", 0xE: "过载",
    }
    checker.check(
        42, "ERR 中文映射完整（0/1 正常，其余故障码按 DM 表翻译）",
        car_web.ERR_NAMES == expected_err
        and lw.get("err") == 1 and lw.get("err_text") == "使能"
        and lw.get("fault") is False
        and car_web.ERR_NAMES[0xE] == "过载"
        and rw.get("err_text") == "使能",
        "names=%d L=%s/%s R=%s/%s" % (
            len(car_web.ERR_NAMES), lw.get("err"), lw.get("err_text"),
            rw.get("err"), rw.get("err_text")))

    temps = [car_web.temp_level(x) for x in (69, 70, 71, 85, 86, 120)]
    ages = [car_web.age_level(x) for x in (0, 200, 201, 500, 501, 0xFFFF)]
    checker.check(
        43, "分级阈值：温度 >70 橙 / >85 红；反馈年龄 >200 橙 / >500 红",
        temps == ["ok", "ok", "warn", "warn", "bad", "bad"]
        and ages == ["ok", "ok", "warn", "warn", "bad", "bad"],
        "temp=%s age=%s" % (temps, ages))

    # ---------------- 44~45：390px 移动端 + 新增显示确实渲染出来 ----------------
    if args.no_browser:
        checker.skip(44, "390px 移动端无横向溢出、新卡片都在", "--no-browser")
        checker.skip(45, "新增字段在页面渲染（年龄/温度/ERR/|a|/版本）", "--no-browser")
    else:
        browser = BrowserSession(base, width=390, height=844, mobile=True)
        try:
            browser.start()
            assert browser.cdp is not None
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                if browser.cdp.eval(
                        "!!document.getElementById('wsBadge')"
                        " && document.getElementById('wsBadge').classList.contains('ok')"):
                    break
                time.sleep(0.1)
            time.sleep(0.4)       # 等第一帧 /api/state 渲染完
            meta = browser.cdp.eval("""(() => {
              const de = document.documentElement;
              const boxes = ['.layout', '.topbar', '.wheel-grid', '.wheel-facts',
                             '.diag-grid', '.imu-body', '.card-note'];
              const inner = [];
              for (const sel of boxes) {
                document.querySelectorAll(sel).forEach(el => {
                  if (el.scrollWidth > el.clientWidth + 1) inner.push(sel);
                });
              }
              return {
                width: window.innerWidth,
                overflow: de.scrollWidth - de.clientWidth,
                bodyOverflow: document.body.scrollWidth - de.clientWidth,
                inner: inner,
                wheels: document.querySelectorAll('.wheel').length,
                hasImuMag: !!document.getElementById('imuMag'),
                hasAge: !!document.getElementById('w0age'),
              };
            })()""")
            checker.check(
                44, "390px 移动端：无横向溢出、左右轮详情 + IMU 模长 + 反馈年龄都在",
                meta and abs(meta["width"] - 390) <= 1
                and meta["overflow"] <= 1 and meta["bodyOverflow"] <= 1
                and not meta["inner"]
                and meta["wheels"] == 2
                and meta["hasImuMag"] and meta["hasAge"],
                "meta=%s" % (meta,))

            vals = browser.cdp.eval("""(() => {
              const t = id => (document.getElementById(id) || {}).textContent || '';
              const c = id => (document.getElementById(id) || {}).className || '';
              const css = document.querySelector('link[rel=stylesheet]');
              const js = document.querySelector('script[src]');
              return {
                version: t('footerVer'),
                cssHref: css ? css.href : '',
                jsSrc: js ? js.src : '',
                age: t('w0age'), ageCls: c('w0age'),
                mos: t('w0mos'), coil: t('w0coil'),
                pos: t('w0pos'), dev: t('w0dev'), follow: t('w0follow'),
                torque: t('w0torque'), err: t('w0err'), errCls: c('w0err'),
                imuMag: t('imuMag'), imuMagCls: c('imuMag'),
                boot: t('dBoot'), can: t('dCan'), fbAge: t('dFbAge'),
                note: (document.querySelector('.card-note') || {}).textContent || '',
              };
            })()""")
            note = (vals or {}).get("note", "")
            checker.check(
                45, "页面新增值：反馈年龄 ms / 温度 / ERR 使能 / |a|≈9.81 / 版本 1.3.0",
                vals
                and "1.3.0" in vals["version"]
                and "1.3.0" in vals["cssHref"] and "1.3.0" in vals["jsSrc"]
                and vals["age"].endswith(" ms") and "ok" in vals["ageCls"]
                and vals["mos"].isdigit() and vals["coil"].isdigit()
                and math.isfinite(float(vals["pos"]))
                and math.isfinite(float(vals["torque"]))
                and math.isfinite(float(vals["dev"]))
                and "使能" in vals["err"] and "ok" in vals["errCls"]
                and 8.5 <= float(vals["imuMag"]) <= 11.0
                and "ok" in vals["imuMagCls"]
                and vals["boot"] == "正常" and vals["can"].startswith("0 /")
                and "20 ms" in vals["fbAge"]
                and "扭矩" in note and "KT" in note and "不是相电流" in note,
                "vals=%s" % (vals,))
        finally:
            browser.close()
            time.sleep(0.3)


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="car_web --sim 自测")
    ap.add_argument("--no-browser", action="store_true",
                    help="跳过真实浏览器断言（没有 Edge/Chrome 时）")
    args = ap.parse_args()

    checker = Checker()
    proc = None
    log_path = None
    try:
        proc, base, _host, log_path = start_server()
        print("car_web --sim 已启动：%s\n" % base)
        run(checker, base, args)
    except Exception as exc:
        print("\n[ERROR] 自测中断：%s" % exc)
        if log_path:
            print("\n--- car_web 日志（末尾）---")
            print(read_log(log_path))
        return 1
    finally:
        if proc is not None:
            stop_server(proc)

    print("\n" + "=" * 60)
    print("PASS=%d  FAIL=%d  SKIP=%d" %
          (checker.passed, checker.failed, checker.skipped))
    if checker.failures:
        print("\n失败项：")
        for f in checker.failures:
            print("  - " + f)
    print("=" * 60)
    return 0 if checker.failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
