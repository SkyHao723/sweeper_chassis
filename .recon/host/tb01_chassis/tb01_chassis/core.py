"""底盘串口链路核心逻辑（ROS 无关，可单测）。

职责：
- 打开串口（115200 8N1），后台线程按字节拼帧（24B 遥测 / 44B 诊断 / 8B 回充 / 19B 超声波）
- 发送 11 字节速度指令帧（带锁，线程安全）
- 指令超时后持续发零速"显式停车"帧（喂 STM32 看门狗 + 刹车）
- 退出时连发一串零速
- 串口异常自动重连
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from . import protocol as p

LogFn = Callable[[str, str], None]     # (level, message)，level: info/warn/error
SerialFactory = Callable[[], object]   # 返回串口对象，需实现 read(n)/write(bytes)/close()


class ChassisCore:
    def __init__(
        self,
        port: str = "/dev/wheeltec_controller",
        baud: int = 115200,
        cmd_timeout: float = 0.5,
        idle_zero_hz: float = 10.0,
        stop_burst: int = 10,
        logger: Optional[LogFn] = None,
        serial_factory: Optional[SerialFactory] = None,
        reconnect_period: float = 1.0,
    ):
        self.port = port
        self.baud = int(baud)
        self.cmd_timeout = float(cmd_timeout)
        self.idle_zero_hz = max(0.0, float(idle_zero_hz))
        self.stop_burst = max(1, int(stop_burst))
        self.reconnect_period = float(reconnect_period)
        self._log = logger or (lambda level, msg: None)
        self._serial_factory = serial_factory
        self._serial = None
        self._serial_lock = threading.Lock()
        self._buf = bytearray()

        self._running = False
        self._reader_thread: Optional[threading.Thread] = None
        self._idle_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._last_cmd_time = 0.0
        self._last_zero_time = 0.0
        self._last_telemetry_time = 0.0
        self._last_rx_time = 0.0
        self._connected = False
        self._released_stop = False

        # 回调（节点启动后设置）
        self.on_telemetry: Optional[Callable[[p.Telemetry], None]] = None
        self.on_diag: Optional[Callable[[p.ChassisDiag], None]] = None
        self.on_recharge: Optional[Callable[[p.RechargeTelemetry], None]] = None
        self.on_link_state: Optional[Callable[[bool], None]] = None

        # 统计
        self.stats = {
            "telemetry_frames": 0,
            "diag_frames": 0,
            "recharge_frames": 0,
            "bad_bytes": 0,
            "rx_bytes": 0,
            "tx_frames": 0,
            "zero_frames": 0,
            "func_frames": 0,
            "reconnects": 0,
        }

    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_cmd_age(self) -> float:
        if self._last_cmd_time <= 0:
            return float("inf")
        return time.monotonic() - self._last_cmd_time

    @property
    def last_rx_age(self) -> float:
        if self._last_rx_time <= 0:
            return float("inf")
        return time.monotonic() - self._last_rx_time

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._try_open()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="tb01_serial_rx", daemon=True)
        self._idle_thread = threading.Thread(target=self._idle_loop, name="tb01_idle_tx", daemon=True)
        self._reader_thread.start()
        self._idle_thread.start()

    def stop(self) -> None:
        self._running = False
        self._stop_event.set()
        # 先发一串零速把车按住（显式停车）
        self._send_stop_burst()
        for thread in (self._reader_thread, self._idle_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        self._close_serial()

    # ------------------------------------------------------------------
    def _try_open(self) -> bool:
        try:
            if self._serial_factory is not None:
                serial_obj = self._serial_factory()
            else:
                import serial  # 延迟导入，便于无 pyserial 环境跑部分单测

                serial_obj = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.02,
                    write_timeout=0.2,
                )
                serial_obj.reset_input_buffer()   # 丢掉开机残留字节（对应厂商 flushInput）
        except Exception as exc:  # noqa: BLE001
            self._set_connected(False)
            self._log("error", f"打开串口失败 {self.port}@{self.baud}: {exc}")
            return False
        with self._serial_lock:
            self._serial = serial_obj
        self._buf.clear()
        self._set_connected(True)
        self._log("info", f"串口已打开: {self.port} @ {self.baud} 8N1")
        return True

    def _close_serial(self) -> None:
        with self._serial_lock:
            serial_obj = self._serial
            self._serial = None
        if serial_obj is not None:
            try:
                serial_obj.close()
            except Exception:  # noqa: BLE001
                pass

    def _set_connected(self, connected: bool) -> None:
        if connected == self._connected:
            return
        self._connected = connected
        if self.on_link_state is not None:
            try:
                self.on_link_state(connected)
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------
    def send_speed(self, vx: float, vy: float = 0.0, wz: float = 0.0) -> bool:
        """发送一条速度指令（m/s, m/s, rad/s）。成功返回 True。"""
        frame = p.build_speed_frame(vx, vy, wz)
        ok = self._write(frame)
        if ok:
            self._last_cmd_time = time.monotonic()
            self._released_stop = False
        return ok

    def send_zero(self) -> bool:
        ok = self._write(p.build_speed_frame(0.0, 0.0, 0.0))
        if ok:
            self._last_zero_time = time.monotonic()
            self.stats["zero_frames"] += 1
        return ok

    def send_func(self, func_code: int, value: int = 0, repeat: int = 3, interval: float = 0.05) -> bool:
        """发送主机功能帧（固件 v2+），重复 repeat 次、间隔 interval 秒。

        当前唯一用途：FUNC_MOTOR_REINIT —— 让底盘重新"写速度模式 + 使能"。
        底盘只在车已停时接受，所以这里**先补一发零速帧**再发功能帧。
        返回 True 表示至少发出了一帧。
        """
        self.send_zero()
        sent = False
        for index in range(max(1, int(repeat))):
            if not self._write(p.build_func_frame(func_code, value)):
                return sent
            self.stats["func_frames"] += 1
            sent = True
            if index + 1 < int(repeat):
                time.sleep(max(0.0, float(interval)))
        return sent

    def _write(self, frame: bytes) -> bool:
        with self._serial_lock:
            serial_obj = self._serial
            if serial_obj is None:
                return False
            try:
                serial_obj.write(frame)
            except Exception as exc:  # noqa: BLE001
                self._log("warn", f"串口写失败: {exc}")
                return False
        self.stats["tx_frames"] += 1
        return True

    def _send_stop_burst(self) -> None:
        for _ in range(self.stop_burst):
            if not self.send_zero():
                break
            time.sleep(0.02)

    # ------------------------------------------------------------------
    def _idle_loop(self) -> None:
        period = 1.0 / self.idle_zero_hz if self.idle_zero_hz > 0 else 0.02
        while self._running and not self._stop_event.is_set():
            try:
                if self._connected:
                    age = self.last_cmd_age
                    if age > self.cmd_timeout:
                        if self.idle_zero_hz > 0.0:
                            self.send_zero()
                        elif not self._released_stop:
                            # idle_zero_hz=0：超时只补发一次零速
                            self.send_zero()
                            self._released_stop = True
                            self._log("warn", f"超过 {self.cmd_timeout:.2f}s 没有 cmd_vel，已补发零速停车")
                self._stop_event.wait(period if self.idle_zero_hz > 0 else 0.05)
            except Exception as exc:  # noqa: BLE001
                self._log("error", f"idle 线程异常: {exc}")
                time.sleep(0.1)

    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        while self._running and not self._stop_event.is_set():
            if not self._connected:
                time.sleep(self.reconnect_period)
                if not self._running:
                    break
                self.stats["reconnects"] += 1
                self._try_open()
                continue
            with self._serial_lock:
                serial_obj = self._serial
            if serial_obj is None:
                time.sleep(0.05)
                continue
            try:
                data = serial_obj.read(1)
                if not data:
                    continue
                self.stats["rx_bytes"] += len(data)
                self._last_rx_time = time.monotonic()
                self._feed(data)
            except Exception as exc:  # noqa: BLE001
                self._log("error", f"串口读异常: {exc}")
                self._set_connected(False)
                self._close_serial()
                time.sleep(0.1)

    # ------------------------------------------------------------------
    def _feed(self, data: bytes) -> None:
        """逐字节拼帧：宽容地找帧头，校验失败就丢 1 字节继续。"""
        self._buf.extend(data)
        buf = self._buf
        while True:
            # 找最近的候选帧头
            head_index = -1
            for i, byte in enumerate(buf):
                if byte in (p.TEL_HEAD, p.DIAG_HEAD, p.RECHARGE_HEAD, p.ULTRASONIC_HEAD):
                    head_index = i
                    break
            if head_index < 0:
                self.stats["bad_bytes"] += len(buf)
                buf.clear()
                return
            if head_index > 0:
                self.stats["bad_bytes"] += head_index
                del buf[:head_index]
            head = buf[0]
            size = p.frame_size_for(head)
            if size <= 0:
                self.stats["bad_bytes"] += 1
                del buf[:1]
                continue
            if len(buf) < size:
                return                     # 等更多字节
            frame = bytes(buf[:size])
            if not p.frame_valid(frame):
                self.stats["bad_bytes"] += 1
                del buf[:1]
                continue
            del buf[:size]
            self._dispatch(frame)

    def _dispatch(self, frame: bytes) -> None:
        now = time.monotonic()
        head = frame[0]
        if head == p.TEL_HEAD:
            telemetry = p.parse_telemetry(frame, now)
            if telemetry is not None:
                self.stats["telemetry_frames"] += 1
                self._last_telemetry_time = now
                if self.on_telemetry is not None:
                    self.on_telemetry(telemetry)
        elif head == p.DIAG_HEAD:
            diag = p.parse_diag(frame, now)
            if diag is not None:
                self.stats["diag_frames"] += 1
                if self.on_diag is not None:
                    self.on_diag(diag)
        elif head == p.RECHARGE_HEAD:
            recharge = p.parse_recharge(frame)
            if recharge is not None:
                self.stats["recharge_frames"] += 1
                if self.on_recharge is not None:
                    self.on_recharge(recharge)

    # ------------------------------------------------------------------
    def status(self) -> dict:
        return {
            "connected": self._connected,
            "port": self.port,
            "baud": self.baud,
            "telemetry_frames": self.stats["telemetry_frames"],
            "diag_frames": self.stats["diag_frames"],
            "bad_bytes": self.stats["bad_bytes"],
            "last_rx_age": round(self.last_rx_age, 3) if self._last_rx_time else None,
            "last_cmd_age": round(self.last_cmd_age, 3) if self._last_cmd_time else None,
            "cmd_timeout": self.cmd_timeout,
        }
