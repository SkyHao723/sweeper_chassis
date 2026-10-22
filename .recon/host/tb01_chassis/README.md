# tb01_chassis —— TB01 底盘 ROS 2 主机驱动

把 **RK3588 / RK3576 上的 ROS 2** 接到 **STM32 底盘控制器（UART 115200）**，替代厂商的
`wheeltec_robot_node`（`turn_on_wheeltec_robot` 包里的串口节点）的最小实现。

协议依据（本机 `D:\github\TB01-CAN-USART` 仓库）：

- `docs/chassis-serial-protocol.md` —— 主机 ↔ STM32 的 Wheeltec 11B/24B 串口协议
- `firmware/stm32-chassis/user/main.c` —— **以代码为准**：当前固件是 44B 诊断帧 v1，
  且 `Cmd_Apply()` 会忽略速度帧 `f[1]/f[2]` 标志（老文档里"标志非 0 丢帧"的坑对本固件不适用）

```
/cmd_vel ──► UART 11B 速度帧 (vx,vy,wz ×1000, 大端, BCC) ──► STM32 ──CAN──► FOC 驱动器
             ◄── 24B 主遥测 (车体速度 + IMU + 电压) ── 20Hz
             ◄── 44B 诊断帧 (轮速/目标/扭矩/故障码/温度/CAN 错误) ── 20Hz
```

节点发布 `/odom`、`/imu/data_raw`、`/PowerVoltage`、`/chassis_diag`、`/chassis_diag_raw`、
`/diagnostics`，并支持 `/robot_charging_*`（回充帧，若固件发送）。

---

## 1. 和厂商节点的关系（drop-in 替换）

| | 厂商 `wheeltec_robot_node` | 本包 `tb01_chassis` |
|---|---|---|
| `/cmd_vel` → 串口 11B 速度帧 | ✅ | ✅（同样的字段/单位/BCC） |
| `/odom`（`odom` → `base_footprint`） | ✅ | ✅（同样的积分公式） |
| `/imu/data_raw` | ✅（含厂家姿态解算） | ✅（加速度计定 roll/pitch + 陀螺仪积分 yaw） |
| `/PowerVoltage` | ✅ | ✅（本板固件该字段恒 0） |
| TF `odom_combined → base_footprint` | ❌（由 EKF 发） | ❌（保持一致，交给 EKF） |
| 44B 扩展诊断帧 | ❌ 整帧跳过 | ✅ 解析并发布 `/chassis_diag` + `/diagnostics` |
| 滚刷/水泵继电器转发 | 需打仓库里的 `vendor-patches/chassis_relay_vendor.patch` | ❌ 未实现（用继电器就继续用厂商节点，或把该补丁逻辑搬过来） |
| 指令超时停车 | 依赖 STM32 800 ms 看门狗 | **主动保活：0.5 s 没 cmd_vel 就持续发零速**（显式刹车） |

> 两者**不能同时运行**：串口是独占的。切换前先停厂商栈：
> `pkill -f wheeltec_robot_node`，或 `./chassis_bringup.sh stop`。

## 2. 在 RK3588 / RK3576 上部署

### 2.1 依赖与串口

```bash
# 依赖（Ubuntu + ROS2 Humble/Jazzy）
sudo apt install -y python3-serial

# udev：普通 CH340 (1a86:7523) 厂商规则不认，需自己加（车上已加过可跳过）
sudo tee /etc/udev/rules.d/wheeltec_controller_ch340.rules >/dev/null <<'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

ls -l /dev/wheeltec_controller   # 应指向 ttyUSB*
```

> 两个 CH340 同时插在一台机器上时，`/dev/wheeltec_controller` 会指向最后枚举的那个
> （CH340 没有唯一序列号）。这种场景要按 USB 物理口钉死（`KERNELS=="1-1"`），详见仓库 README。

### 2.2 编译运行

```bash
mkdir -p ~/tb01_ws/src
cp -r tb01_chassis ~/tb01_ws/src/
cd ~/tb01_ws
colcon build --packages-select tb01_chassis
source install/setup.bash

# 不接 ROS 先验串口（会先停掉占用串口的进程！）
ros2 run tb01_chassis tb01_check_serial -p /dev/wheeltec_controller --zero

# 起节点
ros2 launch tb01_chassis tb01_chassis.launch.py
```

另开终端验证：

```bash
ros2 topic echo /odom --once
ros2 topic echo /chassis_diag --once
ros2 topic hz /odom             # 应约 20 Hz
ros2 topic echo /imu/data_raw --once
```

手动发速度（**必须持续发 ≥2 Hz**，否则 STM32 800 ms 看门狗会刹车）：

```bash
# 前进 0.2 m/s，持续 20 Hz 发布
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# 停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

> ⚠️ `teleop_twist_keyboard` 在 Linux 下每按一次键只发**一条** Twist，车只会走 0.8 s；
> 要连续行驶请用 `ros2 topic pub -r 20` 或仓库里的网页遥操（20 Hz 发布）。

## 3. 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 只用 `linear.x`（m/s）、`angular.z`（rad/s）；`linear.y` 默认忽略 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 20 Hz，`frame_id=odom`，`child_frame_id=base_footprint`；位姿积分，twist 用本帧实测 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 发布 | 20 Hz，accel/gyro 按协议换算，姿态为轻量解算（yaw 会漂） |
| `/PowerVoltage` | `std_msgs/Float32` | 发布 | 每 10 帧一次；当前固件该字段恒 0（本板无采样） |
| `/chassis_diag` | `std_msgs/String` | 发布 | 44B 诊断帧解析后的 JSON（轮速/目标/扭矩/故障码/温度/IMU 状态…） |
| `/chassis_diag_raw` | `std_msgs/UInt8MultiArray` | 发布 | 诊断帧原始 44 字节 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 发布 | 驱动器掉线/故障/命令超时/IMU 异常 |
| `/robot_charging_current` | `std_msgs/Float32` | 发布 | 收到 0x7C 回充帧时 |
| `/robot_red_flag`、`/robot_charging_flag` | `std_msgs/Bool` | 发布 | 同上 |

## 4. 参数（`config/tb01_chassis.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` / `baud` | `/dev/wheeltec_controller` / 115200 | 串口 |
| `cmd_timeout` | 0.5 | 超过多久没 `/cmd_vel` 就持续发零速 |
| `idle_zero_hz` | 10.0 | 零速保活频率；0 = 超时只补发一次 |
| `stop_burst` | 10 | 退出时连发零速帧数（约 200 ms） |
| `max_vx` / `max_wz` / `max_vy` | 0.6 / 1.0 / 0.0 | 发送前夹紧 |
| `publish_tf` | false | 与厂商栈一致：TF 由 EKF 发；没有 EKF 时可打开 |
| `imu_publish_orientation` | true | false 时把 orientation_covariance[0] 置 -1（表示不提供姿态） |
| `publish_diagnostics` | true | 是否发 `/diagnostics` |
| `publish_joint_states` | false | 需要轮子关节可视化时打开 |

轮径/轮距**不在主机侧配置**：逆运动学在 STM32 固件里（`WHEEL_DIAMETER_MM` / `TRACK_WIDTH_MM`），
主机只发车体速度 `(vx, vy, wz)`。

## 5. 安全要点（读一遍）

1. **STM32 看门狗 800 ms**：它只认"速度帧"；发一条就停 → 车走 0.8 s 后停。
   本节点在空闲/超时后以 10 Hz 发**全零速度帧**——这是合法的"显式停车"，
   会喂看门狗并让驱动器刹车（不是沉默）。
2. **零速帧 ≠ 沉默**：显式零速还会清掉上次看门狗停车的锁存。
3. **退出会连发 `stop_burst` 帧零速**再关串口。
4. **上电自检点动**：STM32 上电 3 秒后会自己往前动一下（约 7~16 cm，固件 `SELFTEST_*`；
   torque-mode 分支默认已关）。上车前人不要站在车前面。
5. 速度包线（旧固件实测，参考）：**0.25~0.35 m/s 最稳**（103%/-3.2°/m），>0.4 会超速、
   0.15 又慢又偏。torque-mode 分支换了力矩环后需要重测。
6. 不要和厂商节点同时开（串口独占）；`ROS_DOMAIN_ID` 在 SSH/脚本里也要一致。

## 6. 自测（不需要 ROS、不需要硬件）

```bash
cd ~/tb01_ws/src/tb01_chassis
python3 -m tb01_chassis.selftest
```

40 项检查：速度帧与协议文档示例**逐字节一致**、遥测/诊断帧解析、里程计/IMU 解算、
以及 ChassisCore 与"假 STM32"（20 Hz 回帧 + 800 ms 看门狗模型）的完整闭环——
包括垃圾字节容错、指令超时补零速、退出连发零速。

## 7. 文件结构

```
tb01_chassis/
├── tb01_chassis/
│   ├── protocol.py     # 11B/24B/44B/8B/19B 帧构造与解析、BCC、故障码表
│   ├── core.py         # 串口线程、拼帧、发送、超时零速、退出零速、重连
│   ├── odometry.py     # 位姿积分 + IMU 姿态解算
│   ├── node.py         # ROS 2 节点
│   ├── check_serial.py # 串口只读检查（可选 --zero）
│   └── selftest.py     # 无 ROS 自测
├── config/tb01_chassis.yaml
├── launch/tb01_chassis.launch.py
├── package.xml
└── setup.py
```

## 8. 故障排查

| 现象 | 处理 |
|---|---|
| 打不开串口 / `Permission denied` | udev 规则 + `dialout` 组；或先用 `sudo chmod 666 /dev/ttyUSB0` 临时验证 |
| 能开串口但收不到帧 | 波特率必须 115200；TX/RX 交叉；底盘是否上电（PB9 灯 2Hz/5Hz 闪） |
| `/odom` 不动、`/chassis_diag` 里 cmd_timeout=1 | 没人发 `/cmd_vel`，或发得太稀（<2 Hz） |
| 车不动但 `/cmd_vel` 正常 | `ros2 topic echo /chassis_diag --once`：看 `cmd_vx` 是否到了底盘、`left_online/right_online`、故障码 |
| 转向方向错 | STM32 固件里的 `MOTOR1_IS_LEFT` / `MOTOR*_INVERT`（主机侧不调） |
| 诊断帧字段乱 | 固件版本不同：看日志 "诊断帧版本=x"；本驱动按 v1（44 字节）解析 |
| `/PowerVoltage` 恒 0 | 本板固件没有电压采样，属正常 |
