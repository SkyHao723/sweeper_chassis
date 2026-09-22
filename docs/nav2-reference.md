# 底盘接口与精度参考（给做 Nav2 定位/导航的同事）

> 对象：**在 RK3588 上开发 ROS 2 Humble + Nav2 的定位/导航**。
> 内容：这台清扫车底盘**能给你什么、不能给你什么、哪些数字可以直接引用**。
> 全部数字都是实车实测的，测量方法和条件写在旁边 —— **别脱离条件引用**。
>
> 底盘控制是自研 STM32 固件（`firmware/stm32-chassis/`），厂商只提供了
> `wheeltec_robot_node` 这个串口桥。凡是"厂商默认值没实测过"的地方，都标了 ⚠。

---

## 0. 一页速查

| 你想干的事 | 用什么 |
|---|---|
| 发速度 | `/cmd_vel`（`geometry_msgs/Twist`），**20 Hz 持续发** |
| 拿里程计（**Nav2 用这个**） | `/odom_combined`（EKF 输出），TF 父帧 `odom_combined` |
| 拿纯轮速里程计（对照/调试用） | `/odom`，**它没有 TF**，`odom` 是孤儿帧 |
| 拿 IMU | `/imu/data_raw`（20 Hz，含三轴角速度/加速度/解算姿态） |
| 电池电压 | `/PowerVoltage` |
| 开关扫地滚刷 / 水泵（本车扩展） | `/chassis_relay`（`std_msgs/UInt8`：bit0 滚刷 bit1 水泵） |
| **本次导航速度上限** | `vx ≤ 0.30~0.35 m/s`，`|wz| ≥ 0.25 rad/s`（理由见 §3） |

**三条最容易踩的**：

1. **`odom` 帧没有 TF**，只有 `odom_combined` 有。Nav2 的 `odom_topic` 用
   `/odom_combined`，`odom_frame` 写 `odom_combined`。
2. **阶跃命令会过冲到约 138%**：你发 0.30，车会先到约 0.41 m/s 再落回。
   Nav2 的限速、以及任何"按时间估算应走多远"的检查都要容忍这个。
3. **停车后会滑约 12 cm**（低速时刹车力弱）。`goal` 的容差要留够。

---

## 1. 话题与坐标系

### 1.1 话题

| 话题 | 类型 | frame_id → child | 频率（实测） | 谁发 |
|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `odom` → `base_footprint` | **20.0~20.2 Hz** | `wheeltec_robot_node`（纯轮速积分） |
| `/odom_combined` | `nav_msgs/Odometry` | `odom_combined` → `base_footprint` | ~20 Hz | `ekf_filter_node` |
| `/imu/data_raw` | `sensor_msgs/Imu` | `gyro_link` | 20 Hz | `wheeltec_robot_node` |
| `/PowerVoltage` | `std_msgs/Float32` | — | 低 | 同上 |
| `/cmd_vel` | `geometry_msgs/Twist` | — | **订阅，≥20 Hz** | 你发 |
| `/odometry/filtered` | 同 `/odom_combined` | — | — | EKF（remap 别名） |
| `/chassis_relay` | `std_msgs/UInt8` | — | 事件 | 你发（本车扩展，需厂商节点转发） |

### 1.2 TF 树（★ 先看这张图再看代码）

```
map ──(SLAM/AMCL 发布, 目前没有)──► odom_combined ──(EKF 发布)──► base_footprint
                                                                    │
                                          (robot_state_publisher, URDF) │
                                                                       ▼
                                        base_link / laser / gyro_link / radar …
```

- **只有 EKF 发 `odom_combined → base_footprint` 这一条动态 TF**（`ekf.yaml`
  里 `publish_tf: true`）。厂商节点**不发 TF** —— 它只在 `/odom` 消息里填了
  `frame_id`/`child_frame_id`，**没有 `sendTransform`**。所以 `odom` 这个帧
  **没有任何 TF 连到树上**，是个孤儿帧。
- `base_footprint → 各传感器`由 `robot_state_publisher`（URDF）和四个
  `static_transform_publisher`（`base_to_link` / `base_to_laser` /
  `base_to_gyro` / `base_to_radar`）发布。
- **`base_footprint` 的定义是"两个驱动轮触地点的中点在地面的投影"** ——
  所有传感器外参都要从这一点量（见 §6.3）。

### 1.3 `/cmd_vel` 的语义

| 字段 | 单位 | 正方向 |
|---|---|---|
| `linear.x` | m/s | **前进** |
| `angular.z` | rad/s | **左转**（从上往下看逆时针）—— 用 IMU 验过 ✓ |

`linear.y` 忽略（差速车）。固件内的限幅是 `|vx| ≤ 2.15 m/s`、`|wz| ≤ 3.0 rad/s`
—— **很高，不能当安全网**，请自己在 Nav2 侧限。

**为什么必须持续发**：STM32 有 **800 ms 运动看门狗** —— 收不到速度帧就自动把
目标冻结。停止发布 = 停车，这是设计的安全网之一。`ros2 topic pub --once` 只会让
车动不到 0.8 秒。

---

## 2. 实测精度（可以直接引用）

> 除非另外注明，都是**最近一次固件**（外环带"线性带"那版）在**轮子着地**下测的。
> 工具：`tools/straight_quality.py`、`tools/turn_truth.py`、`tools/chassis_check.py`。

### 2.1 直线（命令 0.30 m/s，持续 10 秒，实测 3.0 m）

| 指标 | 实测 | 说明 |
|---|---|---|
| **平均速度** | **92%**（另一趟 **99%**） | 用 `/odom` twist 平均 |
| 路径长度 | 92%（另一趟 99%） | 用 `/odom` 位姿积分 |
| **真实偏航**（IMU 积分） | **−0.36° / 3 m = 0.12°/m** | 命令 `wz=0`，所以这就是"车实际歪了多少" |
| **里程计偏航 vs IMU 真值** | **−0.08°**（0.03°/m） | 轮速里程计的诚实度 |
| **一次过冲** | 峰值到 **138%**，约 **3 秒**落回 | 驱动器速度环的阶跃响应 |
| IMU 陀螺零偏 | **−0.0007 rad/s**（0.04°/s） | 直行时 `wz` 均值 |
| IMU 陀螺噪声 | **σ = 0.069 rad/s**（行驶中） | 见 §5.2 |

**结论：直线精度够用。** 主要误差是那一次过冲（一次性瞬态，不影响平均）。

### 2.2 原地转（`wz = 0.5 / 0.7 / 1.0`，持续 4 秒）

| 命令 wz | 单轮目标 | 4 秒转过总角度（对 IMU 真值） |
|---|---|---|
| 0.50 | 21.6 RPM | **101%** |
| 0.70 | 30.3 RPM | **100%** |
| 1.00 | 43.3 RPM | **102%** |
| 0.30 | 13.0 RPM | **49%** ⚠ 别用这个档 |

- 稳态转速 95~99%。**末段不再塌陷**（早期版本会退到 14~54%）。
- ⚠ 以上是**外环 v2** 测的；之后给外环加了"线性带"（只影响积分、不动起步助推），
  **原地转没有重测**，但预期相近。`0.30` 这一档从 v1 就一直很差，属于低速死区。

### 2.3 距离刻度

- 用 `chassis_check.py --drive-to 1.0`（把里程计钉在 1.0 m 就停）测到车实际停在
  **约 1.12 m**。**但其中约 12 cm 是停车后的滑行**，所以**不能用它当刻度结论**。
- 交叉验证：原地转时 `/odom` 的偏航相对 IMU 只低 **2.6~3.8%**，而轮速里程计的
  偏航 = `轮速 × 周长 / 轮距`，**对周长的灵敏度和直行一样** —— 若周长真差 11%，
  偏航也该差 11%。所以**刻度误差远小于 11%，大概率在 3% 上下**。
- ⚠ **权威做法**：沿地面参照线跑 4 m 以上，量真实距离，再用
  `base_serial.launch.py` 里的 `odom_x_scale` 修（**只影响里程计积分，不影响
  下发指令**，所以随便调，不用重烧固件）。

---

## 3. ★ 速度包线（配 Nav2 参数看这一节）

| 区间 | 表现 | 建议 |
|---|---|---|
| `vx < 0.22 m/s` | **驱动器速度环不稳**（单轮 <20 RPM 进入死区/极限环） | **不要在 Nav2 里用** |
| **`vx = 0.25~0.35 m/s`** | 均值 92~103%，最稳 | **推荐工作区** |
| `vx > 0.5 m/s` | 超速（早期实测 134%） | 别用 |
| `\|wz\| < 0.25 rad/s` | 同上，进死区 | 设成 Nav2 的最小角速度 |

**不要用单一系数去补速度误差。** 早期实测（外环之前）：

```
0.15 m/s → 64%      0.30 m/s → 103%      0.50 m/s → 134%
```

**低速偏低、高速偏高**，是明显的非线性（≈`0.28 × 命令^1.51`）。用一个
全局 scale 会让某些档更糟（0.15 档要 ×1.56，0.50 档要 ×0.75）。

**原地转的起步要 1.5~2 秒**（`wz=1.0` 时第 0 秒也只有 36%）。所以
`RotationShim` / 原地转的超时、以及恢复行为的时间预算都要按这个留。

**停车滑行约 12 cm**（`CTRL_BRAKE` 低速制动力弱，是这台驱动器的固有特性）。
`xy_goal_tolerance` 要大于它。

---

## 4. EKF 现状

配置文件：`~/smart_ws/src/turn_on_wheeltec_robot/config/ekf.yaml`

| 项 | 值 |
|---|---|
| `frequency` | 20.0 |
| `two_d_mode` | true |
| `odom_frame` / `world_frame` | `odom_combined` |
| `base_link_frame` | `base_footprint` |
| `odom0` 融合 | 只融合 `x_vel` + `yaw_vel`（不融合位姿） |
| `imu0` 融合 | **只融合 `yaw_vel`** |
| `transform_time_offset` | **0.0**（刻意。非零会让 AMCL 拿扫描去配 150 ms 前的位姿，转弯时变成恒定的角度滞后，随转动累积） |
| `publish_tf` | true（**整个 TF 树的唯一动态边**） |

### 4.1 ★ 偏航几乎全靠 IMU，而 IMU 没有绝对参照

厂商给的协方差：`/odom` 的 yaw_vel 方差 **5e-2**（运动）/ **1e-2**（静止），
IMU 的 `angular_velocity_covariance[8]` = **2.5e-3** —— **IMU 权重是轮速的
4~20 倍**。所以 `/odom_combined` 的偏航 ≈ IMU 陀螺积分。

**这意味着**：

- 短期准（实测偏航 0.12°/m，比纯轮速还好）
- **但没有任何绝对参照 → 长期必然漂，且漂了回不来**
- 唯一的治本是**装上雷达跑 SLAM**（或者融合磁力计）

### 4.2 IMU 噪声比厂商假设的略大

实测行驶中陀螺 `wz` 标准差 **0.069 rad/s**，厂商假设 **0.05**（1.4 倍）。
而且噪声是**重尾**的（偶发 ±0.5 rad/s 尖峰，来自真实振动 —— 加速度计峰峰值
也有 2.5 m/s²）。IMU 的拒绝阈值是 100σ = 328°/s，**拦不住这些尖峰**。

**影响**：偏航的**均值**不受影响（尖峰正负对称，实测零偏只有 −0.0007 rad/s），
但**抖动**偏大。按实测把协方差改成 `0.069² ≈ 4.8e-3` 是个 1.9 倍的小修正，
收益有限，**不是当务之急**。

### 4.3 没做的（明确列出来，免得以为配好了）

- 没有融合磁力计、没有融合 `/odom` 的绝对位姿
- 没有 `use_control`（不用 `/cmd_vel` 预测）
- 没有融合 IMU 线加速度
- 传感器外参目前是厂商默认值，**不是实测值**（见 §6.3）

---

## 5. 已知故障模式与症状（省得重踩）

| 症状 | 真正原因 | 怎么办 |
|---|---|---|
| `/odom` **时有时无**、或突然消失 | CH340 在 USB 层掉线（内核日志有 `USB disconnect` + 重新枚举） | 软件侧有 `chassis-watchdog`（停发就重启栈）。**硬件侧要查 USB 线/接头** |
| `/odom` **一条都没有**，但话题存在 | ①STM32 没在发（供电/卡死）②**两个进程抢同一个串口** | `fuser -v /dev/wheeltec_controller`；用 `tools/decode_diag.py` 直接读原始串口 —— **它若也读不到，问题在 STM32 侧，不在 ROS** |
| 车**忽快忽慢**、完全不听 | `/cmd_vel` 上有**第二个发布者**在抢（`chassis-web.service` 开机自启；**只有网页里点了"使能"它才真发**） | `ros2 topic list` + `ros2 topic info /cmd_vel --verbose` 看发布者节点名 |
| 车**偶尔不动 / 偶尔歪一下** | 驱动器没都回码时单轮出力 = 原地打转 | 固件已有"就绪联锁"：两个驱动器没都回码就**只发刹车**。诊断帧 `flags` bit5 会显示 |
| 直行**跑偏** | 先排除假象：①IMU 死了（EKF 偏航会冻住，`flags` bit2/bit3）②`/odom` 不可信 | 用 `tools/straight_quality.py` —— **它拿 IMU 当真值**，并会自检 IMU 死活 |
| 低速**又慢又抖** | 驱动器速度环死区/极限环 | 别用 `<0.22 m/s`（§3）。这是驱动器的固有问题，**改 Nav2 参数救不了** |

**电磁/供电类的坑**（都会表现成"随机、不可复现"）：继电器模块必须和 STM32
**共电源共地**，否则光耦半开、线圈半通电、持续嗡嗡响；IMU 掉电时会表现为
`imu=无应答`，而它一死整个航向就没了。

---

## 6. 配置检查清单

### 6.1 ROS 环境

- **`ROS_DOMAIN_ID=5` 必须处处一致**。它写在 `.bashrc` 里，而 `.bashrc` 对
  **非交互 shell 会提前 return** —— 所以 `ssh host 'cmd'`、脚本、systemd 都会
  落到 domain 0，症状是"节点活着但收不到数据"，极难查。已经写进
  `/etc/environment` 兜住，但**你自己写的脚本里仍要显式 export**。
- 启动：`ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py`
  或 `bash ~/chassis_tools/chassis_bringup.sh start|stop|restart|status`

### 6.2 Nav2 侧要改的

```yaml
# 里程计
odom_topic: /odom_combined        # ★ 不是 /odom (那个没有 TF)
robot_base_frame: base_footprint
odom_frame: odom_combined

# 速度限幅（★ 见 §3）
max_vel_x: 0.30                   # 别超过 0.35
min_vel_x: 0.25                   # ★ 低于 0.22 驱动器会不稳
max_vel_theta: 1.0
min_speed_theta: 0.25             # ★ 同上
# 停车滑行约 12 cm
xy_goal_tolerance: 0.15
```

### 6.3 ⚠ 装雷达后**必须实测**的外参

`config/robot_model.yaml` 里的偏移量**目前是厂商默认值，不是本车实测值**。
参数格式是 `[x, y, z, roll, pitch, yaw]`（米 / 弧度）：

```yaml
base_to_laser: [...]     # ★ 雷达位置错了, 建图和定位全错
base_to_link: [...]      # 底盘本体参考面离地高度
```

**量测基准**：`base_footprint` = **两个驱动轮触地点的中点在地面的投影**
（不是车体中心，也不是轮轴上方）。

其它已知量：**轮距（左右轮触地点间距）930 mm**，**轮周长 0.644 m**
（实测 2 m ÷ 3.1 圈 = 0.645 m，吻合）。
⚠ **URDF 里的 `senior_diff` 尺寸是错的**（轮距 2×0.1815 = 0.363 m，而实际
930 mm）。它**只影响 RViz 显示**，不影响里程计（固件里用的是 930 mm）——
但你在 RViz 里看到的机器人形状和真实大小对不上，别被它误导。

### 6.4 本车扩展：滚刷 / 水泵

`/chassis_relay`（`std_msgs/UInt8`，bit0 滚刷 bit1 水泵）—— 固件里滚刷和水泵都
**只能显式开关**，不会自动起。底层是自定义的 `f[1]=0x05` 帧，靠厂商节点转发
（补丁见 `vendor-patches/chassis_relay_vendor.patch`）。

---

## 7. 还没测 / 不确定的（诚实清单）

| 项 | 状态 |
|---|---|
| **端到端延迟**（`/cmd_vel` → 车动、以及 `/odom` 的滞后） | **没测**。已知源侧时序：STM32 CAN 周期 20 ms、遥测 50 ms（所以 `/odom` 源数据本身最多旧 50 ms） |
| **原地转精度**（外环加了"线性带"之后） | **没重测**（v2 的数据见 §2.2） |
| **距离刻度的权威值** | 只有 1 m 的测量（且被滑行污染），**需要 4 m 以上重测** |
| `vx=0.30` 的均值 | 两趟分别 92% / 99%，**离散度没搞清**；怀疑与电池电压有关（实测 `bat` 从 26.9V 掉到 22.8V 时表现变差） |
| **电池** | **实测掉到 22.8V**（驱动峰值电流 7~11A）。电压低时驱动器表现会变差 → **建议充满再标定/评估** |
| **雷达** | **未安装** → **SLAM / Nav2 目前无法工作**（它们唯一的环境输入就是激光） |
| 传感器外参 | 厂商默认值，**未实测**（§6.3） |
| 固件实验分支 | `pos-mode`（位置模式）在另一条分支上，**未验证**，不要用在 main 上 |

---

## 8. 可用工具（都在 `tools/`，部署在车的 `~/chassis_tools/`）

| 工具 | 干什么 |
|---|---|
| `straight_quality.py VX DUR` | **直线质量**：用 IMU 当偏航真值，量化"忽快忽慢"和跑偏；含 IMU 死活自检和 `/cmd_vel` 污染自检 |
| `turn_truth.py WZ DUR` | **原地转真值**：同时记录 `/odom`、`/odom_combined`、`/imu/data_raw`，逐秒角速率 |
| `turn_sweep.sh` | 原地转对 `wz` 扫一遍（4 个点），看低速区有多坏 |
| `chassis_check.py` | 验收：`--once VX`、`--drive-to 米`、`--turn-only`；自带就绪等待和污染自检 |
| `decode_diag.py` | **直读串口**看 STM32 内部状态（驱动器电流/故障码/模式/目标 vs 实际）。**串口是独占的，要先停 ROS 底盘节点** |
| `chassis_bringup.sh` | `start|stop|restart|status` |
| `chassis_watchdog.sh` | `/odom` 停发就重启栈（systemd: `chassis-watchdog.service`） |
| `chassis_web.py` | 网页监视 + 手动控制：`http://<车IP>:8080`。⚠ 点"使能"后它会接管 `/cmd_vel` |

**两个使用纪律**（都是踩过坑总结的）：

1. **测量前先确认 `/cmd_vel` 上没有别人在发**。工具会自检，但手工测试时自己也要看一眼。
2. **`decode_diag` 需要独占串口**，跑之前先 `touch /tmp/chassis_watchdog.pause`
   暂停看门狗、停掉 ROS 栈，跑完 `rm` 再起栈。

---

## 9. 更深的细节在哪

| 想了解 | 看 |
|---|---|
| 底盘串口协议（11 字节命令帧 / 24 字节遥测帧） | `docs/chassis-serial-protocol.md` |
| 接线、引脚、IMU 为什么用软件 I2C | `docs/wiring.md` |
| ROS 接入的完整过程、踩过的坑、工作包线 | `docs/ros-integration.md` |
| 固件设计、继电器、外环修正、诊断帧 | `README.md` |
| 位置模式实验（另一分支） | `docs/pos-mode.md` |
