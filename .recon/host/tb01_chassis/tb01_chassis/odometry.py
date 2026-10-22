"""里程计积分与 IMU 姿态（纯 Python，无 ROS 依赖）。"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class OdomState:
    pose: Pose2D = None  # type: ignore[assignment]
    linear: float = 0.0
    angular: float = 0.0
    distance: float = 0.0

    def __post_init__(self) -> None:
        if self.pose is None:
            self.pose = Pose2D()


class OdomIntegrator:
    """用底盘回传的车体速度 (vx, vy, wz) 积分位姿。

    与厂商节点一致：
        x += (vx*cosψ - vy*sinψ) * dt
        y += (vx*sinψ + vy*cosψ) * dt
        ψ += wz * dt
    dt 用相邻两帧遥测的到达时间差（不是标称 50 ms）。
    """

    def __init__(self) -> None:
        self.state = OdomState(pose=Pose2D())
        self._last_ts: float | None = None

    def reset(self) -> None:
        self.state = OdomState(pose=Pose2D())
        self._last_ts = None

    def update(self, vx: float, vy: float, wz: float, timestamp: float) -> OdomState:
        dt = 0.0
        if self._last_ts is not None:
            dt = timestamp - self._last_ts
            if dt < 0.0 or dt > 0.5:      # 掉线/时间跳变：本帧只更新状态，不积分
                dt = 0.0
        self._last_ts = timestamp

        pose = self.state.pose
        if dt > 0.0:
            mid = pose.theta + wz * dt / 2.0
            pose.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
            pose.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
            pose.theta = wrap_angle(pose.theta + wz * dt)
            self.state.distance += math.hypot(vx, vy) * dt
        self.state.linear = vx
        self.state.angular = wz
        return self.state


@dataclass
class Orientation:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)   # x, y, z, w


class ImuOrientation:
    """轻量姿态解算：加速度计定 roll/pitch，陀螺仪 z 积分 yaw（与厂商 Quaternion_Solution 同类）。

    没有磁力计，yaw 会漂移；Nav2 侧由 EKF 用轮速 + 陀螺仪融合。
    """

    def __init__(self, accel_lpf: float = 0.2) -> None:
        self.accel_lpf = accel_lpf
        self.accel = [0.0, 0.0, 0.0]
        self.orientation = Orientation()

    def reset(self) -> None:
        self.accel = [0.0, 0.0, 0.0]
        self.orientation = Orientation()

    def update(self, accel_ms2: tuple[float, float, float], gyro_rad_s: tuple[float, float, float], dt: float) -> Orientation:
        a = self.accel_lpf
        for i in range(3):
            self.accel[i] = a * accel_ms2[i] + (1.0 - a) * self.accel[i]
        ax, ay, az = self.accel
        roll = math.atan2(ay, az) if (az or ay) else 0.0
        pitch = math.atan2(-ax, math.hypot(ay, az)) if ax or ay or az else 0.0

        yaw = self.orientation.yaw
        if dt > 0.0 and dt < 0.5:
            yaw = wrap_angle(yaw + gyro_rad_s[2] * dt)

        qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)
        self.orientation = Orientation(roll=roll, pitch=pitch, yaw=yaw, quat=(qx, qy, qz, qw))
        return self.orientation


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw
