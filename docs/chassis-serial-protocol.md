# 底盘控制说明（嵌入式视角）

本文说明主机是怎么让轮子转的：物理接口、串口帧格式、校验、下位机回传、以及 ROS 侧怎么接到这条链路上。

**结论先说：** 主机不直接控电机、不发 PWM、不发四个轮的转速。主机只通过 UART 给 STM32 发**车体目标速度** `(vx, vy, wz)`；STM32 做逆运动学和电机闭环。回传同样是车体速度 + IMU + 电压，不是各轮编码器。

协议实现在厂商包，不在本仓库：

- 节点：`wheeltec_robot_node`（包 `turn_on_wheeltec_robot`）
- 源码：`/home/smart/smart_ws/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`
- 头文件：`.../include/turn_on_wheeltec_robot/wheeltec_robot.h`
- 启动：`ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py`

本车车型配置：`car_mode: senior_4wd_bs`（四驱差速）。有效控制量基本是 `vx + wz`，`vy` 发 0。

---

## 1. 分层：谁干什么

```
┌─────────────────────────────────────────────────────────────┐
│ 主机（RK3588 / ROS 2）                                      │
│   Nav2 / 手柄 / 停车脚本                                    │
│     → geometry_msgs/Twist  /cmd_vel                         │
│   wheeltec_robot_node                                       │
│     → 打包 11 字节速度帧，写串口                            │
│     ← 读 24 字节遥测帧，积分里程计，发 /odom /imu/data_raw  │
└──────────────────────────┬──────────────────────────────────┘
                           │ UART 115200 8N1
                           │ /dev/wheeltec_controller
┌──────────────────────────▼──────────────────────────────────┐
│ STM32 底盘板                                                │
│   解析 (vx, vy, wz) → 按车型逆解成各轮目标转速              │
│   电机驱动 + 编码器 PID → 轮子转                            │
│   周期上报：车体速度、IMU 原始值、电池电压                  │
└─────────────────────────────────────────────────────────────┘
```

| 层级 | 做什么 | 不做什么 |
|------|--------|----------|
| Nav2 / 上层 | 算该走多快，发 `/cmd_vel` | 不碰串口 |
| `wheeltec_robot_node` | Twist ↔ 二进制帧，积分 `/odom` | 不控电机 |
| STM32 | 逆解、电机闭环、采 IMU/电压 | 不回位姿、不 ACK 指令 |

通信模型：

- **没有握手、没有 ACK、没有指令序号。**
- 主机：**事件驱动**发速度帧（有 `/cmd_vel` 就发）。
- STM32：**自己周期**往上推遥测。
- 同一根 UART 全双工，发指令和收回传互不等待。
- **下一帧覆盖上一帧。停发 ≠ 停车**，要停必须发全零。进程退出时析构函数会再发一帧零速度。

---

## 2. 物理层

| 项目 | 值 | 来源 |
|------|----|------|
| 设备节点 | `/dev/wheeltec_controller` | `base_serial.launch.py` → `usart_port_name` |
| 波特率 | 115200 | `serial_baud_rate` |
| 数据位/校验/停止 | 8N1 | `serial` 库默认 |
| 读超时 | 2000 ms | `serial::Timeout::simpleTimeout(2000)` |
| 打开后 | `flushInput()` | 丢掉开机残留字节 |

本机没有 STM32 固件源码。下面协议全部从主机驱动反推。

---

## 3. 协议公共约定

三种业务帧共用这些规则（机械臂帧除外，见第 6 节）。

| 约定 | 说明 |
|------|------|
| 粘包 | 帧头 + 定长 + 帧尾 |
| 字节序 | **大端**：高字节在前 |
| 有符号整数 | `int16`，C 的 `short` |
| 速度缩放 | 主机↔STM32 都是 **×1000**：m/s ↔ mm/s，rad/s ↔ mrad/s |
| 校验 | **BCC**：指定范围内逐字节异或 |
| 接收拼帧 | 主机循环一次 `read()` 1 字节，用静态计数器凑齐再校验 |

BCC 算法（`Check_Sum()`）：

```
check = 0
for i in 0 .. Count_Number-1:
    check ^= buf[i]
```

- 发送：对 `tx[0]..tx[8]` 共 9 字节异或，结果放 `tx[9]`，`tx[10]` 是帧尾。
- 接收主遥测：对 `rx[0]..rx[21]` 共 22 字节异或，应等于 `rx[22]`，`rx[23]` 是帧尾。

帧头/帧尾：

| 帧类型 | 头 | 尾 | 长度 |
|--------|----|----|------|
| 速度 / 灯带 / 回充开关 / 安全开关（主机→STM32） | `0x7B` | `0x7D` | 11 |
| 主遥测（STM32→主机） | `0x7B` | `0x7D` | 24 |
| 自动回充遥测 | `0x7C` | `0x7F` | 8 |
| 超声波 | `0xFA` | `0xFC` | 19 |
| 机械臂 | `0xAA` | `0xBB` | 10 |

主遥测和速度指令头尾相同、长度不同，靠定长区分。

---

## 4. 主机 → STM32：怎么让轮子转

### 4.1 入口

`wheeltec_robot_node` 订阅 `/cmd_vel`（`geometry_msgs/msg/Twist`，队列深度 2）。回调 `Cmd_Vel_Callback()` 里立刻组帧、`Stm32_Serial.write()`。

Twist 三个有效字段：

| Twist 字段 | 物理含义 | 本车 |
|------------|----------|------|
| `linear.x` | 前进，m/s，前为正 | 用，Nav2 上限 0.6 |
| `linear.y` | 左移，m/s | 差速车发 0 |
| `angular.z` | 绕竖直轴，rad/s，逆时针为正 | 用，Nav2 上限 0.75 |

打包前可走 `avoid_obstacle()`。本车 `ranger_avoid_flag: false`、`ultrasonic_avoid: false`，一般原样转发。

### 4.2 速度指令帧（11 字节）

```
偏移  长度  类型     内容
0     1     uint8    帧头 0x7B
1     1     uint8    AutoRecharge（正常走车 = 0）
2     1     uint8    SecurityPLY（正常走车 = 0）
3–4   2     int16 BE vx * 1000（mm/s）
5–6   2     int16 BE vy * 1000（mm/s）
7–8   2     int16 BE wz * 1000（mrad/s）
9     1     uint8    BCC = XOR(tx[0]..tx[8])
10    1     uint8    帧尾 0x7D
```

C++ 侧写法（有符号右移取高字节）：

```cpp
short transition = twist.linear.x * 1000;
tx[4] = transition;        // 低 8 位
tx[3] = transition >> 8;   // 高 8 位
```

STM32 收到后：

1. 核对帧头、帧尾、BCC。
2. 把 `(vx, vy, wz)` 按 `senior_4wd_bs` 逆解成各轮目标转速。
3. 电机 PID 跟踪。主机看不到这一步。

### 4.3 组帧示例

**停车**

```
7B 00 00 00 00 00 00 00 00 7B 7D
BCC = 0x7B
```

**前进 0.2 m/s**（`0.2 * 1000 = 200 = 0x00C8`）

```
7B 00 00 00 C8 00 00 00 00 B3 7D
              ^^^^ vx
BCC = 0x7B ^ 0xC8 = 0xB3
```

**前进 0.3 m/s**（`300 = 0x012C`）

```
7B 00 00 01 2C 00 00 00 00 56 7D
BCC = 0x7B ^ 0x01 ^ 0x2C = 0x56
```

**原地左转 0.5 rad/s**（`500 = 0x01F4`）

```
7B 00 00 00 00 00 00 01 F4 8E 7D
                       ^^^^ wz
BCC = 0x7B ^ 0x01 ^ 0xF4 = 0x8E
```

**后退 0.2 m/s**（`-200 = 0xFF38`，int16 补码）

```
7B 00 00 FF 38 00 00 00 00 BC 7D
BCC = 0x7B ^ 0xFF ^ 0x38 = 0xBC
```

用串口助手发上面任意一帧，轮子就应该动。随后应持续看到 24 字节 `7B ... 7D` 回传。

### 4.4 谁在往 `/cmd_vel` 写

| 来源 | 频率 / 时机 | 内容 |
|------|-------------|------|
| Nav2 `controller_server`（RotationShim + MPPI，DiffDrive） | 约 12 Hz | 跟路径的速度 |
| `behavior_server` | 恢复动作时 | 原地转 / 后退等 |
| `keepout_escape_manager` | 禁行区脱困开始、节点退出 | 全零停车 |
| `/ly_nav/stop_all` | 一键停机 | 全零，再杀进程 |
| 键盘 / 手柄（若启动） | 按键时 | 与导航抢同一话题 |

本仓库**故意不启** `velocity_smoother`，`controller_server` 的 `cmd_vel` **不**重映射成 `cmd_vel_nav`。底盘看到的就是控制器的原始 Twist。

Nav2 速度上限见 `src/navigation/mid360_nav_bridge/config/param_nav.yaml`：`vx_max: 0.6`，`wz_max: 0.75`，`vy_max: 0.0`。

---

## 5. STM32 → 主机：回传内容

STM32 **主动循环上报**，不是“你发一帧我回一帧”。主机 `Control()` 循环里每次读 1 字节，三种帧并行拼。

拼帧同步：上一字节是某帧帧尾、本字节是另一帧帧头时开始计数。主遥测在上一字节为 `0x7D` 或回充帧尾 `0x7F`、本字节为 `0x7B` 时入帧。

### 5.1 主遥测帧（走车真正用的）——24 字节

```
偏移    类型       含义                         主机换算
0       uint8      帧头 0x7B
1       uint8      Flag_Stop（预留）
2–3     int16 BE   车体 vx（mm/s）              / 1000 → m/s
4–5     int16 BE   车体 vy（mm/s）              / 1000 → m/s
6–7     int16 BE   车体 wz（mrad/s）            / 1000 → rad/s
8–9     int16 BE   IMU ax 原始值                / 1671.84 → m/s²
10–11   int16 BE   IMU ay                       同上
12–13   int16 BE   IMU az                       同上
14–15   int16 BE   IMU gx 原始值                × 0.00026644 → rad/s
16–17   int16 BE   IMU gy                       同上
18–19   int16 BE   IMU gz                       同上
20–21   uint16 BE  电池电压（mV）               / 1000 → V
22      uint8      BCC = XOR(rx[0]..rx[21])
23      uint8      帧尾 0x7D
```

校验顺序：长度 24 → 帧尾 `0x7D` → BCC 等于 `rx[22]`。全过才解析。

单位依据（头文件宏，与 STM32 上 IMU 初始化量程绑定）：

| 量 | 量程 | 换算 |
|----|------|------|
| 加速度 | ±2 g = ±19.6 m/s²，原始 ±32768 | `ACCEl_RATIO = 32768/19.6 = 1671.84` |
| 陀螺仪 | ±500 °/s，原始 ±32768 | `GYROSCOPE_RATIO = 1/65.5/57.30 ≈ 0.00026644` |

速度换算 `Odom_Trans()`：把两字节合成 `short`，再 `(v / 1000) + (v % 1000) * 0.001`。对负数同样成立（C++ 向零取整）。

**回的是融合后的车体速度，不是四个轮各自的 RPM。** 也没有位姿、没有“指令执行成功”。

### 5.2 主机拿到遥测之后

1. 用 `odom_*_scale` 校正速度（当前全是 1.0）。
2. 积分位移（STM32 不报位置）：

```
x += (vx * cosψ − vy * sinψ) * dt
y += (vx * sinψ + vy * cosψ) * dt
ψ += wz * dt
```

`dt` 是相邻两帧成功解析的时间差。

3. 用加速度 + 角速度做姿态解算（`Quaternion_Solution`），得到 IMU 四元数。
4. 发布：

| 话题 | 类型 | 内容 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | `frame_id=odom`，`child=base_footprint`，位姿来自积分，twist 来自本帧速度 |
| `/imu/data_raw` | `sensor_msgs/Imu` | 三轴角速度、加速度、解算出的姿态 |
| `/PowerVoltage` | `std_msgs/Float32` | 电压，约每 10 帧成功解析发一次 |

节点**自己不发 TF**。`odom_combined → base_footprint` 由 `robot_localization` EKF 发：融合 `/odom` 的 `vx + wz` 和 `/imu/data_raw` 的 `gz`，20 Hz。

### 5.3 自动回充遥测（可选）——8 字节

```
0       帧头 0x7C
1–2     int16 BE 充电电流（mA → A）
3       红外是否看到充电桩
4       是否在充电
5       回充设置状态
6       BCC（前 6 字节异或）
7       帧尾 0x7F
```

发布：`/robot_charging_current`、`/robot_red_flag`、`/robot_charging_flag`。

### 5.4 超声波（可选）——19 字节

```
0       帧头 0xFA
1–12    6 路距离，每路 int16 BE，单位 mm → m（A–F）
13–16   预留 / 自检
17      BCC（前 17 字节异或）
18      帧尾 0xFC
```

本车 `ultrasonic_avoid: false`，走车不用。

---

## 6. 其它下行帧（不转轮子，但走同一串口）

11 字节外壳不变，用 `tx[1]` / `tx[2]` 当功能码。

### 6.1 灯带

`tx[1] = 0x04`。

```
7B 04 EN R G B 00 00 00 BCC 7D
```

`EN=1` 开灯。话题：`/set_rgb_color`（`UInt8MultiArray`，4 个元素：使能, R, G, B）。

### 6.2 自动回充开关

新协议：`tx[1]=1`，`tx[2]=0xA1` 开 / `0xA0` 关，速度字节全 0。话题：`/robot_recharge_flag`。

正常速度帧的 `tx[1]` 也会带当前 `AutoRecharge`，下位机用来区分“导航速度”还是“回充速度”。

### 6.3 底盘安全防护

新协议：`tx[1]=0`，`tx[2]=0xB1` 开 / `0xB0` 关。话题：`/chassis_security`。

速度帧的 `tx[2]` 带当前 `SecurityPLY`。

### 6.4 机械臂（另一套头尾）

```
AA  [j1×1000 BE] [j2×1000 BE] [j3×1000 BE]  gripper  BCC  BB
```

10 字节，头 `0xAA`、尾 `0xBB`。话题：`/arm_cmd`。跟走车无关。

### 6.5 退出时的停车

析构函数发两帧：

1. 标准速度全零（`0x7B ... 0x7D`）。
2. 机械臂复位姿态（`0xAA ... 0xBB`）。

然后关串口。

---

## 7. 时序

```
Nav2 ~12 Hz                    STM32 周期上报
Twist ──────────────────┐      24B 遥测 ──────────────┐
                        ▼                             ▼
              Cmd_Vel_Callback                  Get_Sensor_Data_New
              打包 11B write                    逐字节拼帧、BCC
                        │                             │
                        └──── UART TX/RX ─────────────┘
                                      │
                                      ▼
                               积分 /odom
                               EKF → odom_combined
                               Nav2 用里程计跟路径
```

要点：

- 控制开环（相对主机）：发目标速度，不等人回 ACK。
- 电机闭环在 STM32：轮子跟没跟上，看回传 `Robot_Vel` 是否接近指令。
- 导航闭环在 ROS：Nav2 看 `/odom_combined` 和 TF，不看串口原始帧。
- `/cmd_vel` 停更不等于车停。掉节点、拔串口、STM32 看门狗如何处理，主机驱动里没有写死；能依赖的只有“主动发零”和析构发零。

---

## 8. ROS 对照

```
/cmd_vel  (Twist)
    └─ wheeltec_robot_node ──UART──► STM32 ──► 电机

STM32 ──UART──► wheeltec_robot_node
                    ├─ /odom              原始轮速积分
                    ├─ /imu/data_raw      板载 IMU
                    └─ /PowerVoltage

/odom + /imu/data_raw
    └─ ekf_filter_node
            ├─ /odom_combined
            └─ TF: odom_combined → base_footprint

Open3D ICP（本仓库）
    └─ TF: map → odom_combined     只修全局，不控底盘
```

启动底盘（不在本仓库）：

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

它会拉起：串口驱动、`joint_state_publisher`、EKF、URDF / 静态 TF。

本仓库导航只约定：底盘订 `/cmd_vel`、提供 `base_footprint`、发 `odom_combined → base_footprint`。

---

## 9. 手工验证

前提：底盘 bringup 已起，串口被 `wheeltec_robot_node` 占用时不要再用串口助手抢同一设备。

**走 ROS：**

```bash
# 前进 0.2 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 停车
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

**看回传是否进 ROS：**

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic echo /PowerVoltage --once
```

`twist.twist.linear.x` 应接近指令；`pose` 应随时间积分增加。

**直接 UART（节点必须先停，否则抢口）：**

```
前进 0.2 m/s:  7B 00 00 00 C8 00 00 00 00 B3 7D
停车:          7B 00 00 00 00 00 00 00 00 7B 7D
```

115200 8N1，发十六进制原始字节。正常时应持续收到 24 字节 `7B ... 7D`，其中偏移 2–3 在前进时应接近 `00 C8`。

---

## 10. 源码索引

| 内容 | 位置 |
|------|------|
| 组速度帧、写串口 | `wheeltec_robot.cpp` → `Cmd_Vel_Callback()` |
| 读串口、拆三种回传帧 | `Get_Sensor_Data_New()` |
| BCC | `Check_Sum()` / `Calculate_BCC()` |
| 速度 / IMU 换算 | `Odom_Trans()` / `IMU_Trans()`，宏在 `wheeltec_robot.h` |
| 积分里程计、发 `/odom` | `Control()` / `Publish_Odom()` |
| 开串口参数 | `launch/base_serial.launch.py` |
| 车型、超声波开关 | `config/wheeltec_param.yaml` |
| EKF | `launch/wheeltec_ekf.launch.py`，`config/ekf.yaml` |
| 本仓库 `/cmd_vel` 来源 | `src/navigation/mid360_nav_bridge/launch/navigation_launch.py` |
| Nav2 速度限制 | `src/navigation/mid360_nav_bridge/config/param_nav.yaml` |

---

## 11. 一句话总结

主机通过 **UART 115200** 向 STM32 发 **11 字节速度帧**（`0x7B` + `vx,vy,wz`×1000 大端 + BCC + `0x7D`），STM32 负责把车体速度变成轮子转；STM32 主动回 **24 字节遥测**（车体速度 + IMU + 电压），主机用来积分 `/odom`。没有 ACK，没有轮级指令，停车必须显式发零。
