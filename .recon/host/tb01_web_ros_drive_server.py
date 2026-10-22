#!/usr/bin/env python3
"""TB01 底盘网页驾驶（ROS cmd_vel 模式）。

与旧 web_driver（直连 USB-CAN）的区别：本入口**不碰串口**，只做
    浏览器 --WebSocket 20Hz--> 本程序 --ROS /cmd_vel--> scrubber_damiao_chassis --UART--> STM32
    浏览器 <--20Hz 状态-- 本程序 <--/odom、/chassis_diag-- ROS

只用标准库 + ROS 自带 rclpy（不需要 FastAPI/uvicorn/额外 pip 包）。

用法（车上，已 source ROS 环境）：
    export ROS_DOMAIN_ID=7
    python3 ros_drive_server.py --port 8080
    # 浏览器打开 http://<车IP>:8080

无 ROS 的本地 UI 调试：
    python3 ros_drive_server.py --sim --port 8080

安全（三层）：
1. 浏览器：松手/失焦/切标签页/空格/急停 → 立刻发停车并撤销使能；
2. 服务端：0.5 s 没收到浏览器指令 → 自动发零速；WebSocket 全断 → 撤销使能 + 补发零速；
3. 底盘：STM32 800 ms 看门狗刹车（本程序退出后也会自动刹）。
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

PUBLISH_HZ = 20.0
CMD_TIMEOUT = 0.5          # s，服务端多久没收到指令就归零
STOP_BURST = 8             # 撤销使能时补发几帧零速


# ---------------------------------------------------------------------------
# 桥：ROS 真机 / 仿真
# ---------------------------------------------------------------------------
class BridgeBase:
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def set_cmd(self, v: float, w: float) -> None: ...
    def set_enabled(self, enabled: bool) -> None: ...
    def get_state(self) -> dict[str, Any]: ...
    def request_reinit(self) -> bool:
        """请求底盘重新给驱动器"写速度模式+使能"（固件 v2+）。"""
        return False


class SimBridge(BridgeBase):
    """--sim：无 ROS、无硬件，用一阶模型模拟 odom，用来调 UI。"""

    def __init__(self, max_vx: float, max_wz: float):
        self.max_vx = max_vx
        self.max_wz = max_wz
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._enabled = False
        self._cmd = (0.0, 0.0)
        self._last_cmd = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._last_t = time.monotonic()

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def set_cmd(self, v: float, w: float) -> None:
        with self._lock:
            self._cmd = (max(-self.max_vx, min(self.max_vx, v)), max(-self.max_wz, min(self.max_wz, w)))
            self._last_cmd = time.monotonic()

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled
            if not enabled:
                self._cmd = (0.0, 0.0)

    def _loop(self) -> None:
        while self._running:
            now = time.monotonic()
            dt = now - self._last_t
            self._last_t = now
            with self._lock:
                target = self._cmd if (self._enabled and now - self._last_cmd <= CMD_TIMEOUT) else (0.0, 0.0)
                self._vx += (target[0] - self._vx) * min(1.0, 6.0 * dt)
                self._wz += (target[1] - self._wz) * min(1.0, 6.0 * dt)
                self._th += self._wz * dt
                self._x += self._vx * math.cos(self._th) * dt
                self._y += self._vx * math.sin(self._th) * dt
            time.sleep(0.01)

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            fresh = time.monotonic() - self._last_cmd <= CMD_TIMEOUT
            return {
                "sim": True,
                "ros_ok": True,
                "enabled": self._enabled,
                "publishing": self._enabled and fresh,
                "cmd": {"v": round(self._cmd[0], 3), "w": round(self._cmd[1], 3)},
                "odom": {"x": self._x, "y": self._y, "theta": self._th,
                         "vx": self._vx, "wz": self._wz},
                "diag": None,
                "firmware_version": 2,
                "reinit_supported": True,
                "limits": {"max_vx": self.max_vx, "max_wz": self.max_wz},
            }

    def request_reinit(self) -> bool:
        return True     # 仿真模式：假装底盘收到重新使能命令


class RosBridge(BridgeBase):
    """真机：发布 /cmd_vel，订阅 /odom、/chassis_diag。"""

    def __init__(self, max_vx: float, max_wz: float):
        self.max_vx = max_vx
        self.max_wz = max_wz
        self._lock = threading.Lock()
        self._running = False
        self._enabled = False
        self._cmd = (0.0, 0.0)
        self._last_cmd = 0.0
        self._publishing = False
        self._odom: dict[str, float] = {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0.0, "wz": 0.0}
        self._odom_stamp = 0.0
        self._diag: Optional[dict] = None
        self._diag_stamp = 0.0
        self._node = None
        self._empty_type = None
        self._reinit_pub = None
        self._thread: Optional[threading.Thread] = None
        self._spin_thread: Optional[threading.Thread] = None

    # -- ROS -----------------------------------------------------------
    def start(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty, String

        rclpy.init(args=None)
        self._rclpy = rclpy
        node = rclpy.create_node("web_cmdvel_bridge")
        self._node = node
        self._pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self._msg_type = Twist
        self._empty_type = Empty
        self._reinit_pub = node.create_publisher(Empty, "/chassis_motor_reinit", 10)
        node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        node.create_subscription(String, "/chassis_diag", self._on_diag, 10)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        self._running = True
        self._thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._thread.start()
        threading.Thread(target=self._health_loop, daemon=True).start()

    def _spin_loop(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception as exc:  # noqa: BLE001
            print(f"[ros_drive_server] ROS spin 退出: {exc!r}", file=sys.stderr, flush=True)
            os._exit(3)

    def _health_loop(self) -> None:
        """rclpy 上下文被外部 shutdown（SIGINT/SIGTERM 等）后进程会半死：
        HTTP 还活着但发布/订阅全失效。这里检测到就退出，交给 systemd 重启。"""
        while self._running:
            try:
                ok = self._rclpy.ok()
            except Exception:  # noqa: BLE001
                ok = False
            if not ok:
                print("[ros_drive_server] ROS 上下文已关闭（被外部 SIGTERM/SIGINT？），退出进程等待重启",
                      file=sys.stderr, flush=True)
                os._exit(4)
            time.sleep(0.5)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass

    # -- 回调 ----------------------------------------------------------
    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._odom = {
                "x": p.x, "y": p.y, "theta": theta,
                "vx": msg.twist.twist.linear.x,
                "wz": msg.twist.twist.angular.z,
            }
            self._odom_stamp = time.monotonic()

    def _on_diag(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._diag = data
            self._diag_stamp = time.monotonic()

    # -- 指令 ----------------------------------------------------------
    def set_cmd(self, v: float, w: float) -> None:
        with self._lock:
            self._cmd = (max(-self.max_vx, min(self.max_vx, v)), max(-self.max_wz, min(self.max_wz, w)))
            self._last_cmd = time.monotonic()

    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._enabled = enabled
            if not enabled:
                self._cmd = (0.0, 0.0)

    def _tx_loop(self) -> None:
        period = 1.0 / PUBLISH_HZ
        was_publishing = False
        zero_burst_left = 0
        while self._running:
            now = time.monotonic()
            with self._lock:
                enabled = self._enabled
                fresh = now - self._last_cmd <= CMD_TIMEOUT
                v, w = self._cmd if (enabled and fresh) else (0.0, 0.0)
                publishing = enabled and fresh
                if publishing and not was_publishing:
                    self._publishing = True
                if not publishing and was_publishing:
                    zero_burst_left = STOP_BURST
            if publishing:
                self._publish(v, w)
            elif zero_burst_left > 0:
                self._publish(0.0, 0.0)
                zero_burst_left -= 1
            with self._lock:
                self._publishing = publishing
            was_publishing = publishing
            time.sleep(period)

    def _publish(self, v: float, w: float) -> None:
        try:
            msg = self._msg_type()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self._pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass

    # -- 状态 ----------------------------------------------------------
    def get_state(self) -> dict[str, Any]:
        now = time.monotonic()
        try:
            rclpy_ok = self._rclpy.ok()
        except Exception:  # noqa: BLE001
            rclpy_ok = False
        with self._lock:
            return {
                "sim": False,
                "ros_ok": self._node is not None and rclpy_ok,
                "enabled": self._enabled,
                "publishing": self._publishing,
                "cmd": {"v": round(self._cmd[0], 3), "w": round(self._cmd[1], 3)},
                "odom": {k: round(v, 4) for k, v in self._odom.items()},
                "odom_age": round(now - self._odom_stamp, 2) if self._odom_stamp else None,
                "diag": self._diag,
                "diag_age": round(now - self._diag_stamp, 2) if self._diag_stamp else None,
                "firmware_version": (self._diag or {}).get("version"),
                "reinit_supported": bool(self._diag and int(self._diag.get("version") or 0) >= 2),
                "limits": {"max_vx": self.max_vx, "max_wz": self.max_wz},
            }

    def request_reinit(self) -> bool:
        """发布 /chassis_motor_reinit（std_msgs/Empty），让底盘重新"写模式+使能"。"""
        if self._reinit_pub is None or self._empty_type is None:
            return False
        try:
            self._reinit_pub.publish(self._empty_type())
            return True
        except Exception:  # noqa: BLE001
            return False


# ---------------------------------------------------------------------------
# 极简 WebSocket（文本 JSON）——避免车端装 websockets 库
# ---------------------------------------------------------------------------
class WsClient:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self._send_lock = threading.Lock()

    def send_text(self, text: str) -> None:
        data = text.encode("utf-8")
        header = bytearray([0x81])
        length = len(data)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack(">H", length))
        else:
            header.append(127)
            header.extend(struct.pack(">Q", length))
        with self._send_lock:
            self.sock.sendall(bytes(header) + data)

    def recv_text(self) -> Optional[str]:
        """返回文本消息；连接关闭返回 None。"""
        header = self._recv_exact(2)
        if header is None:
            return None
        b0, b1 = header[0], header[1]
        opcode = b0 & 0x0F
        masked = (b1 & 0x80) != 0
        length = b1 & 0x7F
        if length == 126:
            ext = self._recv_exact(2)
            if ext is None:
                return None
            length = struct.unpack(">H", ext)[0]
        elif length == 127:
            ext = self._recv_exact(8)
            if ext is None:
                return None
            length = struct.unpack(">Q", ext)[0]
        mask = self._recv_exact(4) if masked else None
        if masked and mask is None:
            return None
        payload = self._recv_exact(length) if length else b""
        if payload is None:
            return None
        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        if opcode == 0x8:
            return None
        if opcode in (0x9, 0xA):
            return ""
        try:
            return payload.decode("utf-8")
        except UnicodeDecodeError:
            return ""

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:  # noqa: BLE001
            pass

    def _recv_exact(self, n: int) -> Optional[bytes]:
        chunks = bytearray()
        while len(chunks) < n:
            try:
                chunk = self.sock.recv(n - len(chunks))
            except Exception:  # noqa: BLE001
                return None
            if not chunk:
                return None
            chunks.extend(chunk)
        return bytes(chunks)


class DriveServer:
    def __init__(self, host: str, port: int, bridge: BridgeBase):
        self.host = host
        self.port = port
        self.bridge = bridge
        self.clients: list[WsClient] = []
        self.clients_lock = threading.Lock()
        self.last_client_cmd = 0.0
        self.last_bye = 0.0
        self._running = True
        self._watchdog_flag = False

    # -- 广播 ----------------------------------------------------------
    def _broadcast_loop(self) -> None:
        period = 1.0 / 20.0
        while self._running:
            state = self.bridge.get_state()
            state.update({
                "type": "state",
                "ts": time.time(),
                "clients": len(self.clients),
                "cmd_age": round(time.monotonic() - self.last_client_cmd, 2) if self.last_client_cmd else None,
                "watchdog": self._watchdog_flag,
            })
            payload = json.dumps(state, ensure_ascii=False)
            with self.clients_lock:
                dead = []
                for client in self.clients:
                    try:
                        client.send_text(payload)
                    except Exception:  # noqa: BLE001
                        dead.append(client)
                for client in dead:
                    self.clients.remove(client)
            if not self.clients and self.bridge.get_state().get("enabled"):
                self.bridge.set_enabled(False)
            time.sleep(period)

    def _watchdog_loop(self) -> None:
        """服务端兜底：0.5s 没收到浏览器指令就归零（每次失效只补发一次）。"""
        while self._running:
            now = time.monotonic()
            enabled = self.bridge.get_state().get("enabled")
            if enabled and self.last_client_cmd and now - self.last_client_cmd > CMD_TIMEOUT:
                if not self._watchdog_flag:
                    self._watchdog_flag = True
                    self.bridge.set_cmd(0.0, 0.0)
            elif not enabled:
                self._watchdog_flag = False
            time.sleep(0.05)

    # -- 指令处理 ------------------------------------------------------
    def handle_ws_message(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        kind = msg.get("type")
        if kind == "enable":
            enabled = bool(msg.get("on"))
            self.bridge.set_enabled(enabled)
            if enabled:
                self.last_client_cmd = time.monotonic()
                self._watchdog_flag = False
            else:
                self.last_client_cmd = 0.0
        elif kind == "cmd":
            if not self.bridge.get_state().get("enabled"):
                return
            try:
                v = float(msg.get("v", 0.0))
                w = float(msg.get("w", 0.0))
            except (TypeError, ValueError):
                return
            self.last_client_cmd = time.monotonic()
            self._watchdog_flag = False
            self.bridge.set_cmd(v, w)
        elif kind == "stop":
            self.bridge.set_enabled(False)
            self.last_client_cmd = 0.0
            self._watchdog_flag = False

    # -- 启动 ----------------------------------------------------------
    def start(self) -> None:
        threading.Thread(target=self._broadcast_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def shutdown(self) -> None:
        self._running = False
        self.bridge.set_enabled(False)
        with self.clients_lock:
            for client in self.clients:
                client.close()
            self.clients.clear()


class Handler(BaseHTTPRequestHandler):
    server_version = "Tb01Drive/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: Any) -> None:  # 安静一点
        pass

    # --------------------------------------------------------------
    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        # WebSocket 升级请求
        if self.headers.get("Upgrade", "").lower() == "websocket":
            self._handle_websocket()
            return
        path = self.path.split("?", 1)[0]
        if path == "/api/state":
            state = self.server.drive.bridge.get_state()  # type: ignore[attr-defined]
            state["clients"] = len(self.server.drive.clients)  # type: ignore[attr-defined]
            self._send(200, json.dumps(state, ensure_ascii=False).encode(), "application/json; charset=utf-8")
            return
        if path in ("/", "/index.html", "/drive.html"):
            self._send_file(STATIC_DIR / "drive.html", "text/html; charset=utf-8")
            return
        if path.startswith("/static/"):
            name = path[len("/static/"):]
            file_path = (STATIC_DIR / name).resolve()
            if STATIC_DIR not in file_path.parents and file_path != STATIC_DIR:
                self._send(403, b"forbidden", "text/plain")
                return
            content_types = {
                ".html": "text/html; charset=utf-8",
                ".js": "text/javascript; charset=utf-8",
                ".css": "text/css; charset=utf-8",
                ".png": "image/png",
                ".svg": "image/svg+xml",
            }
            suffix = file_path.suffix.lower()
            self._send_file(file_path, content_types.get(suffix, "application/octet-stream"))
            return
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def _send_file(self, file_path: Path, content_type: str) -> None:
        try:
            body = file_path.read_bytes()
        except OSError:
            self._send(404, b"not found", "text/plain; charset=utf-8")
            return
        self._send(200, body, content_type)

    # --------------------------------------------------------------
    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path not in ("/api/cmd", "/api/enable", "/api/stop", "/api/reinit"):
            self._send(404, b"not found", "text/plain")
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except Exception:  # noqa: BLE001
            body = {}
        drive = self.server.drive  # type: ignore[attr-defined]
        if path == "/api/enable":
            drive.bridge.set_enabled(bool(body.get("on")))
        elif path == "/api/cmd":
            drive.bridge.set_cmd(float(body.get("v", 0.0)), float(body.get("w", 0.0)))
            drive.last_client_cmd = time.monotonic()
        elif path == "/api/reinit":
            ok = drive.bridge.request_reinit()
            state = drive.bridge.get_state()
            state["reinit_sent"] = ok
            self._send(200, json.dumps(state, ensure_ascii=False).encode(), "application/json")
            return
        else:
            drive.bridge.set_enabled(False)
        self._send(200, json.dumps(drive.bridge.get_state(), ensure_ascii=False).encode(), "application/json")

    # --------------------------------------------------------------
    def do_HEAD(self) -> None:  # noqa: N802
        self._send(200, b"", "text/plain")

    def _handle_websocket(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key", "")
        accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        ).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        client = WsClient(self.connection)
        drive = self.server.drive  # type: ignore[attr-defined]
        with drive.clients_lock:
            drive.clients.append(client)
        try:
            while True:
                raw = client.recv_text()
                if raw is None:
                    break
                if raw:
                    drive.handle_ws_message(raw)
        except Exception:  # noqa: BLE001
            pass
        finally:
            self.close_connection = True
            with drive.clients_lock:
                if client in drive.clients:
                    drive.clients.remove(client)
            client.close()
            # 控制端全断 → 撤销使能（桥会补发零速）
            with drive.clients_lock:
                empty = not drive.clients
            if empty:
                drive.bridge.set_enabled(False)


def main() -> int:
    ap = argparse.ArgumentParser(description="TB01 底盘网页驾驶（ROS cmd_vel 模式）")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--sim", action="store_true", help="不用 ROS/硬件，模拟 odom 调试 UI")
    ap.add_argument("--max-vx", type=float, default=0.3, help="最大线速度 m/s（服务端夹紧）")
    ap.add_argument("--max-wz", type=float, default=0.8, help="最大角速度 rad/s（服务端夹紧）")
    args = ap.parse_args()

    bridge: BridgeBase = SimBridge(args.max_vx, args.max_wz) if args.sim else RosBridge(args.max_vx, args.max_wz)
    bridge.start()

    # rclpy 自己会接管 SIGINT/SIGTERM 只 shutdown 上下文而不退出进程（导致半死）。
    # 这里覆盖成"优雅退出"，配合 systemd Restart=always 自动恢复。
    def _signal_handler(signum, frame):  # noqa: ARG001
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _signal_handler)
        except (ValueError, OSError):
            pass

    drive = DriveServer(args.host, args.port, bridge)
    drive.start()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.daemon_threads = True
    httpd.drive = drive  # type: ignore[attr-defined]
    ip = "127.0.0.1"
    try:
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
    except Exception:  # noqa: BLE001
        pass
    mode = "仿真" if args.sim else "ROS /cmd_vel"
    print("\n  TB01 底盘网页驾驶（%s）" % mode)
    print("  本机:   http://127.0.0.1:%d" % args.port)
    print("  局域网: http://%s:%d" % (ip, args.port))
    print("  限速:   %.2f m/s, %.2f rad/s（服务端夹紧）" % (args.max_vx, args.max_wz))
    print("  Ctrl-C 退出（会撤销使能并补发零速）\n", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        drive.shutdown()
        bridge.stop()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
