"""ROS 2 节点：TB01 底盘串口驱动（替代厂商 wheeltec_robot_node 的最小实现）。

- 订阅 /cmd_vel（geometry_msgs/Twist）→ 11 字节速度帧写串口
- 读 24 字节主遥测 → /odom、/imu/data_raw、/PowerVoltage
- 读 44 字节扩展诊断帧 → /chassis_diag（JSON）、/chassis_diag_raw、/diagnostics
- 指令超时后持续发零速显式停车（喂 STM32 800 ms 看门狗并刹车）
- 退出时连发零速

TF 默认不发（与厂商栈一致：odom_combined -> base_footprint 由 robot_localization 的 EKF 发）。
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty, Float32, String, UInt8MultiArray
from std_srvs.srv import Trigger

from . import protocol as p
from .core import ChassisCore
from .odometry import ImuOrientation, OdomIntegrator

try:
    from tf2_ros import TransformBroadcaster
except Exception:  # pragma: no cover
    TransformBroadcaster = None  # type: ignore


class Tb01ChassisNode(Node):
    def __init__(self) -> None:
        super().__init__("scrubber_damiao_chassis")
        self._declare_params()
        cfg = self._params

        self.odom = OdomIntegrator()
        self.imu = ImuOrientation()
        self._last_telemetry_time = 0.0
        self._power_counter = 0
        self._last_log: dict[str, float] = {}
        self._last_nonzero_cmd = 0.0
        self._firmware_version = 0
        self._last_diag = None
        self._last_reinit_time = 0.0
        self._reinit_deadline = 0.0
        self._reinit_reason = ""
        self._reinit_attempts = 0
        self._suspend_tx_until = 0.0

        self.core = ChassisCore(
            port=cfg["port"],
            baud=cfg["baud"],
            cmd_timeout=cfg["cmd_timeout"],
            idle_zero_hz=cfg["idle_zero_hz"],
            stop_burst=cfg["stop_burst"],
            logger=self._log,
        )
        self.core.on_telemetry = self._on_telemetry
        self.core.on_diag = self._on_diag
        self.core.on_recharge = self._on_recharge
        self.core.on_link_state = self._on_link_state

        # 发布者
        self.odom_pub = self.create_publisher(Odometry, cfg["odom_topic"], 10)
        self.imu_pub = self.create_publisher(Imu, cfg["imu_topic"], 10)
        self.power_pub = self.create_publisher(Float32, cfg["power_topic"], 10)
        self.diag_pub = self.create_publisher(String, cfg["diag_topic"], 10)
        self.diag_raw_pub = self.create_publisher(UInt8MultiArray, cfg["diag_raw_topic"], 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.joint_pub = self.create_publisher(JointState, cfg["joint_states_topic"], 10)
        self.charging_current_pub = self.create_publisher(Float32, "/robot_charging_current", 10)
        self.charging_flag_pub = self.create_publisher(Bool, "/robot_charging_flag", 10)
        self.red_flag_pub = self.create_publisher(Bool, "/robot_red_flag", 10)

        self.odom_frame = cfg["odom_frame"]
        self.base_frame = cfg["base_frame"]
        self.imu_frame = cfg["imu_frame"]
        self.max_vx = float(cfg["max_vx"])
        self.max_wz = float(cfg["max_wz"])
        self.max_vy = float(cfg["max_vy"])
        self.publish_tf = bool(cfg["publish_tf"])
        self.publish_joint_states = bool(cfg["publish_joint_states"])
        self.publish_diagnostics = bool(cfg["publish_diagnostics"])
        self.imu_publish_orientation = bool(cfg["imu_publish_orientation"])
        self.tf_broadcaster = TransformBroadcaster(self) if (TransformBroadcaster and cfg["publish_tf"]) else None

        self.create_subscription(Twist, cfg["cmd_vel_topic"], self._on_cmd_vel, 2)
        self.create_subscription(Empty, "/chassis_motor_reinit", self._on_reinit_msg, 10)
        self.create_service(Trigger, "/chassis_reinit_motors", self._on_reinit_srv)
        self.create_timer(0.2, self._on_status_timer)

        self.core.start()
        self.get_logger().info(
            f"TB01 底盘驱动已启动: {cfg['port']} @ {cfg['baud']}, "
            f"cmd_vel={cfg['cmd_vel_topic']}, 超时 {cfg['cmd_timeout']}s 后发零速"
        )

    # ------------------------------------------------------------------
    def _declare_params(self) -> None:
        defaults = {
            "port": "/dev/wheeltec_controller",
            "baud": 115200,
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
            "imu_topic": "/imu/data_raw",
            "power_topic": "/PowerVoltage",
            "diag_topic": "/chassis_diag",
            "diag_raw_topic": "/chassis_diag_raw",
            "joint_states_topic": "/joint_states",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "imu_frame": "gyro_link",
            "cmd_timeout": 0.5,          # s，超过没有 cmd_vel 就持续发零速
            "idle_zero_hz": 10.0,        # 零速保活频率；0 = 超时只补发一次
            "stop_burst": 10,            # 退出时连发零速帧数
            "max_vx": 0.6,               # m/s（Nav2 配置上限 0.6）
            "max_wz": 1.0,               # rad/s
            "max_vy": 0.0,               # 差速车恒 0
            "publish_tf": False,         # 与厂商栈一致：TF 由 EKF 发
            "publish_joint_states": False,
            "publish_diagnostics": True,
            "imu_publish_orientation": True,
            "power_publish_every_n": 10, # 遥测每 N 帧发一次 /PowerVoltage
            "auto_reinit": True,         # 双轮失能且正在下发指令时，自动发"重新使能"
            "reinit_retry_sec": 3.0,     # 自动重新使能的最小间隔（s）
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._params = {name: self.get_parameter(name).value for name in defaults}

    def _log(self, level: str, msg: str) -> None:
        """注意：rclpy 要求同一调用点的 severity 不能变化，所以每个级别单独一行。"""
        logger = self.get_logger()
        try:
            if level == "error":
                logger.error(msg)
            elif level == "warn":
                logger.warn(msg)
            else:
                logger.info(msg)
        except Exception:  # noqa: BLE001 - 日志异常绝不能杀死串口线程
            pass

    def _throttled_log(self, key: str, level: str, msg: str, period: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) >= period:
            self._last_log[key] = now
            try:
                self._log(level, msg)
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = max(-self.max_vx, min(self.max_vx, float(msg.linear.x)))
        wz = max(-self.max_wz, min(self.max_wz, float(msg.angular.z)))
        vy = max(-self.max_vy, min(self.max_vy, float(msg.linear.y)))
        if abs(msg.linear.x) > self.max_vx or abs(msg.angular.z) > self.max_wz:
            self._throttled_log(
                "clamp", "warn",
                f"cmd_vel 超限被夹: vx={msg.linear.x:.2f}->{vx:.2f}, wz={msg.angular.z:.2f}->{wz:.2f}",
            )
        if abs(vx) > 1e-3 or abs(wz) > 1e-3:
            self._last_nonzero_cmd = time.monotonic()
        # "重新使能"期间暂停转发速度帧：底盘只在车已停(目标=0)时才接受功能帧，
        # 若此时继续把用户的速度帧写下去，功能帧会被底盘拒绝。
        if time.monotonic() < self._suspend_tx_until:
            return
        if not self.core.send_speed(vx, vy, wz):
            self._throttled_log("tx_fail", "warn", "串口未就绪，cmd_vel 未发送")

    # ------------------------------------------------------------------
    # 上位机控制电机使能（固件 v2+）：见 firmware main.c FUNC_MOTOR_REINIT
    # ------------------------------------------------------------------
    def _request_reinit(self, source: str) -> bool:
        """让底盘重新给两个 DM 驱动器"写速度模式 + 使能"。

        为什么需要：DM 驱动器上电默认失能，只能由 STM32 发使能帧；而旧固件
        只在 STM32 自己启动时发一次，整车断电重启时很容易丢（之后车一直不动，
        诊断 err=0）。固件 v2 新增 f[1]=0x06 功能帧，主机在停车状态下可随时补。
        """
        now = time.monotonic()
        if self._firmware_version < p.FUNC_FRAME_MIN_VERSION:
            self._throttled_log(
                "reinit_old_fw", "warn",
                f"上位机使能命令需要底盘固件 v{p.FUNC_FRAME_MIN_VERSION}+，"
                f"当前 v{self._firmware_version or '?'}（需升级固件；临时办法：复位 STM32）",
                10.0,
            )
            return False
        if now - self._last_reinit_time < 1.0:
            self._throttled_log("reinit_fast", "warn", "重新使能请求太频繁（1s 内只发一次）", 5.0)
            return False
        self._last_reinit_time = now
        self._reinit_deadline = now + 2.5
        self._reinit_reason = source
        self._suspend_tx_until = now + 0.4      # 0.4s 内不转发 cmd_vel，保证底盘目标为 0
        self._log("info", f"电机重新使能（{source}）：先发零速，再发功能帧 {p.FUNC_MOTOR_REINIT:#x}")
        self.core.send_func(p.FUNC_MOTOR_REINIT, 0, repeat=3, interval=0.05)
        return True

    def _on_reinit_msg(self, msg: Empty) -> None:  # noqa: ARG002
        if self._request_reinit("话题 /chassis_motor_reinit"):
            self._reinit_attempts += 1

    def _on_reinit_srv(self, request, response):  # noqa: ARG002
        ok = self._request_reinit("服务 /chassis_reinit_motors")
        response.success = bool(ok)
        if ok:
            response.message = "已发送电机重新使能命令；1~2 秒后看 /chassis_diag 的 err 是否变成 [1, 1]"
        else:
            response.message = (
                f"未发送：当前底盘固件 v{self._firmware_version} 不支持上位机使能"
                f"（需 v{p.FUNC_FRAME_MIN_VERSION}+），或请求过于频繁"
            )
        return response

    def _check_reinit_result(self, diag: p.ChassisDiag) -> None:
        if not self._reinit_deadline:
            return
        if diag.left_enabled and diag.right_enabled:
            self._log("info", f"电机重新使能成功（{self._reinit_reason}）：ERR=使能")
            self._reinit_deadline = 0.0
            self._reinit_attempts = 0
        elif time.monotonic() > self._reinit_deadline:
            self._throttled_log(
                "reinit_fail", "error",
                f"电机重新使能失败（{self._reinit_reason}）：ERR 仍为 {list(diag.err)}。"
                "检查驱动器功率电/CAN 线；固件 v2 也会在上电后 10s 内自己重试使能",
                10.0,
            )
            self._reinit_deadline = 0.0

    def _auto_reinit(self, diag: p.ChassisDiag, now: float) -> None:
        """正在下发指令、但两个驱动器都失能 -> 自动补一次使能（固件 v2+）。"""
        if not self._params["auto_reinit"] or self._reinit_deadline:
            return
        if self._firmware_version < p.FUNC_FRAME_MIN_VERSION:
            return
        if diag.left_enabled and diag.right_enabled:
            return
        if self.core.last_cmd_age > 0.6:        # 只有正在被主机驱动时才自动救
            return
        retry = max(1.0, float(self._params["reinit_retry_sec"]))
        if now - self._last_reinit_time < retry:
            return
        if self._request_reinit("自动：正在下发指令但双轮失能"):
            self._reinit_attempts += 1

    # ------------------------------------------------------------------
    def _on_telemetry(self, tel: p.Telemetry) -> None:
        now = time.monotonic()
        dt = now - self._last_telemetry_time if self._last_telemetry_time else 0.0
        self._last_telemetry_time = now

        stamp = self.get_clock().now().to_msg()
        state = self.odom.update(tel.vx, tel.vy, tel.wz, now)
        pose = state.pose
        half = pose.theta / 2.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = tel.vx
        odom.twist.twist.linear.y = tel.vy
        odom.twist.twist.angular.z = tel.wz
        odom.pose.covariance = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = pose.x
            tf.transform.translation.y = pose.y
            tf.transform.rotation.z = math.sin(half)
            tf.transform.rotation.w = math.cos(half)
            self.tf_broadcaster.sendTransform(tf)

        # IMU
        accel = tel.accel_ms2
        gyro = tel.gyro_rad_s
        orientation = self.imu.update(accel, gyro, dt)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        if self.imu_publish_orientation:
            qx, qy, qz, qw = orientation.quat
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
            imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.5]
        else:
            imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = accel
        imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self.imu_pub.publish(imu)

        # 电池电压（本板固件恒 0，仍然按参数节流发布，保持厂商节点行为）
        self._power_counter += 1
        if self._power_counter >= max(1, int(self._params["power_publish_every_n"])):
            self._power_counter = 0
            power = Float32()
            power.data = float(tel.battery_v)
            self.power_pub.publish(power)

        # 只有"刚发过非零指令"时 Flag_Stop=1 才值得报警；空闲时底盘本来就该是停止状态
        if tel.flag_stop and time.monotonic() - self._last_nonzero_cmd < 1.0:
            self._throttled_log("flag_stop", "warn", "底盘 Flag_Stop=1（已下发非零指令但轮速正解为 0）")

    # ------------------------------------------------------------------
    def _on_diag(self, diag: p.ChassisDiag) -> None:
        now = time.monotonic()
        raw = UInt8MultiArray()
        raw.data = list(diag.raw)
        self.diag_raw_pub.publish(raw)

        payload = String()
        payload.data = json.dumps(diag.to_dict(), ensure_ascii=False)
        self.diag_pub.publish(payload)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ["left_wheel", "right_wheel"]
            js.position = list(diag.position)
            js.velocity = list(diag.wheel_vel)
            js.effort = list(diag.torque)
            self.joint_pub.publish(js)

        self._firmware_version = diag.version
        self._last_diag = diag
        if diag.version < p.DIAG_VERSION_MIN or diag.version > p.DIAG_VERSION:
            self._throttled_log(
                "diag_version", "warn",
                f"诊断帧版本={diag.version}（本驱动支持 v{p.DIAG_VERSION_MIN}~v{p.DIAG_VERSION}），字段可能不准确",
                30.0,
            )
        elif diag.version >= p.FUNC_FRAME_MIN_VERSION:
            self._throttled_log(
                "fw_v2", "info",
                f"底盘固件 v{diag.version}: 支持上位机电机使能(/chassis_reinit_motors 或 /chassis_motor_reinit)",
                60.0,
            )

        # 主机请求的"重新使能"结果校验 + 自动补救
        self._check_reinit_result(diag)
        self._auto_reinit(diag, now)

        if self.publish_diagnostics:
            self.diagnostics_pub.publish(self._build_diagnostics(diag))

        if diag.left_fault:
            self._throttled_log("l_fault", "error", f"左轮驱动器故障: {diag.err_name(0)} (码 {diag.err[0]})")
        if diag.right_fault:
            self._throttled_log("r_fault", "error", f"右轮驱动器故障: {diag.err_name(1)} (码 {diag.err[1]})")
        if not diag.left_online or not diag.right_online:
            self._throttled_log(
                "offline", "warn",
                f"驱动器未在线: 左={diag.left_online} 右={diag.right_online}",
            )
        if diag.boot_stage not in (0, 1):
            self._throttled_log("boot", "warn", f"STM32 启动阶段异常: {diag.to_dict()['boot_stage']}")
        if not diag.left_enabled or not diag.right_enabled:
            hint = (
                "可调用 /chassis_reinit_motors 或网页『重新使能电机』"
                if diag.version >= p.FUNC_FRAME_MIN_VERSION
                else "固件 v1 不支持上位机使能，需复位 STM32 底盘"
            )
            self._throttled_log(
                "drv_disabled", "warn",
                f"驱动器失能: err={list(diag.err)}（{diag.err_name(0)}/{diag.err_name(1)}）；{hint}",
                10.0,
            )

    def _build_diagnostics(self, diag: p.ChassisDiag) -> DiagnosticArray:
        status = DiagnosticStatus()
        status.name = "tb01_chassis"
        status.hardware_id = self._params["port"]
        level = DiagnosticStatus.OK
        messages = []
        values = [
            KeyValue(key="left_online", value=str(diag.left_online)),
            KeyValue(key="right_online", value=str(diag.right_online)),
            KeyValue(key="left_enabled", value=str(diag.left_enabled)),
            KeyValue(key="right_enabled", value=str(diag.right_enabled)),
            KeyValue(key="cmd_timeout", value=str(diag.cmd_timeout)),
            KeyValue(key="left_fault", value=str(diag.err_name(0))),
            KeyValue(key="right_fault", value=str(diag.err_name(1))),
            KeyValue(key="left_target_omega", value=f"{diag.wheel_target[0]:.2f}"),
            KeyValue(key="right_target_omega", value=f"{diag.wheel_target[1]:.2f}"),
            KeyValue(key="left_actual_omega", value=f"{diag.wheel_vel[0]:.2f}"),
            KeyValue(key="right_actual_omega", value=f"{diag.wheel_vel[1]:.2f}"),
            KeyValue(key="left_current_a", value=f"{diag.torque[0]:.2f}"),
            KeyValue(key="right_current_a", value=f"{diag.torque[1]:.2f}"),
            KeyValue(key="can_tx_fail", value=str(diag.can_tx_fail)),
            KeyValue(key="imu_ok", value=str(diag.imu_ok)),
            KeyValue(key="boot_stage", value=str(diag.boot_stage)),
            KeyValue(key="firmware_version", value=str(diag.version)),
        ]
        if diag.left_fault or diag.right_fault:
            level = DiagnosticStatus.ERROR
            messages.append(f"驱动器故障: {diag.err_name(0)} / {diag.err_name(1)}")
        if not diag.left_online or not diag.right_online:
            level = max(level, DiagnosticStatus.ERROR)
            messages.append("驱动器掉线")
        if diag.cmd_timeout:
            level = max(level, DiagnosticStatus.WARN)
            messages.append("指令超时（底盘按 0 速度刹车）")
        if not diag.imu_ok:
            level = max(level, DiagnosticStatus.WARN)
            messages.append("IMU 读取失败")
        if not diag.left_enabled or not diag.right_enabled:
            level = max(level, DiagnosticStatus.WARN)
            if diag.version >= p.FUNC_FRAME_MIN_VERSION:
                messages.append("驱动器失能（可用 /chassis_reinit_motors 重新使能）")
            else:
                messages.append("驱动器失能（固件 v1 不支持上位机使能，需复位 STM32）")
        status.level = level
        status.message = "; ".join(messages) if messages else "正常"
        status.values = values
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        return array

    def _on_recharge(self, recharge: p.RechargeTelemetry) -> None:
        current = Float32()
        current.data = float(recharge.current_a)
        self.charging_current_pub.publish(current)
        flag = Bool()
        flag.data = bool(recharge.charging)
        self.charging_flag_pub.publish(flag)
        red = Bool()
        red.data = bool(recharge.ir_seen)
        self.red_flag_pub.publish(red)

    def _on_link_state(self, connected: bool) -> None:
        if connected:
            self._log("info", "底盘串口已连接")
        else:
            self._log("error", "底盘串口断开（自动重连中）")

    def _on_status_timer(self) -> None:
        age = self.core.last_rx_age
        if self.core.connected and age > 1.0:
            self._throttled_log("no_tel", "warn", f"底盘串口 {age:.1f}s 没收到任何帧", 5.0)
        # 诊断帧没来时也做一次"重新使能"超时兜底
        if self._reinit_deadline and time.monotonic() > self._reinit_deadline and self._last_diag is not None:
            self._check_reinit_result(self._last_diag)

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        try:
            self.core.stop()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = Tb01ChassisNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"[tb01_chassis] 启动/运行异常: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
