# 底盘接入 ROS 2（RK3588）

本文记录**这台车**接进 ROS 2 的完整状态：怎么起、验证过什么、还缺什么。
厂商那一套包本身有文档，这里只写**本车特有的部分和踩过的坑**。

---

## 1. 一句话启动

```bash
# .bashrc 已经配好了 ROS 环境(见第 5 节), 新开终端直接:
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

或者用 `tools/chassis_bringup.sh`（推荐，它会等到确认 `/odom` 真的有数据）：

```bash
./chassis_bringup.sh start     # 启动并等到 /odom 出数据 (最多 40 秒)
./chassis_bringup.sh stop      # 停掉全部相关进程
./chassis_bringup.sh restart
./chassis_bringup.sh status    # 节点 / 串口占用 / /odom 有没有数据
```

**为什么值得用脚本**：这套栈有几个"看起来像挂了其实没挂、看着没挂其实挂了"的坑
（见第 6 节），脚本把这些都兜住了。

这一条 launch 会把四部分全带起来：

| 组成 | 作用 |
|---|---|
| `base_serial.launch.py` | `wheeltec_robot_node` —— 读写串口, 出 `/odom` `/imu/data_raw` `/PowerVoltage`，收 `/cmd_vel` |
| `wheeltec_ekf.launch.py` | `robot_localization` 的 EKF，出 `/odom_combined` 和 TF |
| `robot_mode_description.launch.py` | `robot_state_publisher` + 静态 TF（雷达/IMU/底盘）|
| `joint_state_publisher` | 轮子关节（可视化用）|

**别只起 `base_serial.launch.py`** —— 那样没有 TF 树，Nav2/SLAM 什么都干不了。
（我一开始就是这么起的，白折腾了一轮。）

停车：`pkill -f "ros2 launch"` 加上各个节点名（见第 6 节的清理命令）。

---

## 2. 验证过的状态

从**冷启动**（整机断电重启）跑一遍，全部通过：

| 检查 | 结果 |
|---|---|
| 节点数 | 8 个，无重复 |
| `/odom` | 20 Hz（与 STM32 的 50ms 上报周期吻合）|
| `/odom_combined` | 20 Hz（EKF 滤波输出）|
| `/imu/data_raw` | 20 Hz |
| `/PowerVoltage` | 有数据 |
| 坐标系 | `/odom`: `frame_id=odom` → `child_frame_id=base_footprint` |
| TF 树 | 完整，见下 |
| 启动日志 | 无 ERROR / Traceback |

TF 树：

```
odom_combined → base_footprint                       (EKF 发的)
base_footprint → base_link / laser / laser_link / gyro_link / radar
base_link → left_wheel_link, right_wheel_link        (两驱动轮)
          → quanxiang_wheel_link                     (万向轮)
          → camera_link / controller_link
```

**`/cmd_vel` 是订阅话题**，没人往里发时 `ros2 topic hz /cmd_vel` 会报
"does not appear to be published yet" —— 那是正常的，不是故障。
同理 `ros2 topic hz` 偶尔会对正在发布的话题误报，`ros2 topic echo --once`
更可靠。

---

## 3. 本车改动过的地方

### 3.1 车型：`senior_diff`（改过，原来是四驱）

`config/wheeltec_param.yaml`：

```yaml
car_mode:  senior_diff      # 原为 senior_4wd_bs
imu_mode:  stm32
```

原来是 `senior_4wd_bs`，URDF 里是 `lf/rf/lb/rb_wheel_link` **四个轮子**，
而本车是两驱差速。改成 `senior_diff` 后 URDF 变成
`left_wheel_link / right_wheel_link / quanxiang_wheel_link`（两驱动 + 一万向），
和实车一致 ✓

> 原文件备份在同目录 `wheeltec_param.yaml.orig`。

### 3.2 传感器安装位置在 `robot_model.yaml`，不在 URDF 里

**这是个容易找错地方的点**：雷达和 IMU 的安装偏移不在 URDF，而在
`config/robot_model.yaml` 的 `base_to_laser` / `base_to_gyro` /
`base_to_link` / `base_to_radar`，由 `robot_mode_description.launch.py`
读出来发成静态 TF。

所以**按实车校正传感器位置只需要改这个 yaml**，不用动 URDF。格式是
`[x, y, z, roll, pitch, yaw]`，单位米/弧度：

```yaml
senior_diff:
  base_to_link:  [0.0, 0.0, 0.0366, 0.0, 0.0, 0.0]
  base_to_laser: [0.05911, 0.00016, 0.13416, 0.0, 0.0, 0.0]
```

含义（相对 `base_footprint`，也就是车体中心在地面的投影）：

| 字段 | 含义 |
|---|---|
| `base_to_laser` x | 雷达在**两驱动轮轴线**前方多少米 |
| `base_to_laser` z | 雷达离**地面**多高 |
| `base_to_laser` y | 偏离车身中线多少（偏左为正）|
| `base_to_link` z | 底盘本体参考面离地高度 |

⚠️ **本车雷达还没装**，现在的值是从 `senior_diff` 抄来的默认值。
**装雷达后必须实测改成实车尺寸** —— 雷达位置错了，SLAM 建图和 Nav2 定位
会系统性偏移，而且很难从现象上看出是这里的问题。

### 3.3 串口：CH340 的 udev 规则

底盘用 **CH340 (`1a86:7523`)** 接 RK3588。厂家的规则只认 CP2102 和 CH343，
**不认普通 CH340**，所以自己加了一条（已经装在车上了）：

```udev
# /etc/udev/rules.d/wheeltec_controller_ch340.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"
```

普通 CH340 **没有唯一序列号**（`ID_SERIAL=1a86_USB_Serial`），只能按 VID:PID
匹配。以后加第二个 CH340 就得改用 `KERNELS=="3-1.2"` 指定 USB 物理口。

---

## 4. 底盘验收

改完固件或参数后跑一遍，看有没有退步：

```bash
# 串口被底盘节点占着, 而这个脚本走 ROS, 所以两者可以同时跑
python3 tools/chassis_check.py --dur 2
```

它把"起步慢"和"稳态不准"分开报 —— 这两件事的修法完全不同。

**看底盘原始数据**（驱动器电流/故障码/模式）要停掉底盘节点，让它让出串口：

```bash
./chassis_bringup.sh stop
python3 tools/decode_diag.py -p /dev/wheeltec_controller --seconds 3
./chassis_bringup.sh start
```

---

## 4.5 实测工作包线（配 Nav2 速度参数看这个）

固件 `ACCEL_STEP_MS=0` 那一版实测（轮子着地, 每次 2 秒）：

| 命令速度 | 稳态实测 | 直线漂移 | 每米漂移 | 评价 |
|---|---|---|---|---|
| 0.15 m/s | **64%** | −8.9° | **−46°/m** | ❌ 又慢又偏，别用 |
| **0.30 m/s** | **103%** | −1.2° | **−3.2°/m** | ✅ **最佳工作点** |
| 0.50 m/s | **134%** | −0.2° | **−0.25°/m** | ⚠️ 很直但超速 34% |
| 后退 −0.20 m/s | 83% | +3.2° | +15°/m | ⚠️ 偏慢 |
| 原地左转 0.5 rad/s | **98%** | — | — | ✅ |
| 原地右转 0.5 rad/s | **85%** | — | — | ✅ 左右差 13%（两轮振荡相位差）|

### 结论：把导航速度限在 0.25~0.35 m/s

**这个区间速度 103%、漂移 −3.2°/m，基本是完美的。** 低速（<0.2 m/s）既慢又偏，
高速（>0.4 m/s）超速。

### 为什么会这样：不是刻度问题，别用单一系数去补

```
0.15 m/s → 64%   (低 36%)
0.30 m/s → 103%  (准)
0.50 m/s → 134%  (高 34%)
```

**低速偏低、高速偏高**，拟合大约 `实际 ≈ 0.28 × 命令^1.51` —— 是明显的非线性。
**用单一系数去补会让某些速度更糟**（0.15 档要 ×1.56、0.50 档要 ×0.75）。

根因是 **FOC 驱动器自己的速度环**：空载直接读驱动器回码能看到，目标 18 RPM 时
它的实际转速在 **0~22 之间来回冲**（低频振荡/极限环）。这一层协议**不暴露 PID
参数**，所以要么找厂家调，要么在 STM32 上用扭矩模式自己做速度环。

> 别指望用"命令值 vs /odom"来验证轮径 —— 两者用的是**同一个轮径常数**，
> 比值恒等于 1，测不出轮径对不对。轮径要单独量（胎上贴标记，推 2.00 米数圈数）。
> 本车实测 2m ÷ 3.1 圈 = 0.645m，和固件里的 205mm 吻合 ✓

---

## 5. ROS 环境是怎么配的

`.bashrc` 里有个 `_ros_source()` 函数，先 source `/opt/ros/humble`，
再按顺序叠加 `$HOME` 下的工作区：

```bash
_ros_source() {
  source /opt/ros/humble/setup.bash
  for ws in "$@"; do
    f="$HOME/$ws/install/local_setup.bash"
    if [[ -f $f ]]; then source "$f"; else echo "[.bashrc] ROS 工作区缺失: $f" >&2; fi
  done
}
```

默认叠加 `smart_ws`、`robot_lidar_stack/ros2_ws`、`LY_inspect_ws`、`teb_ws`。

### ★ ROS_DOMAIN_ID 必须处处一致（踩过，而且坑了很久）

`ROS_DOMAIN_ID=5` **写在 `.bashrc` 里 —— 这是不够的**。

`.bashrc` 顶部有：

```bash
case $- in
    *i*) ;;
      *) return;;
esac
```

**非交互 shell 会直接 return**，所以通过 `ssh host '命令'`、脚本、
systemd 服务跑的时候，`.bashrc` 里的 `export ROS_DOMAIN_ID=5` **根本不会执行**，
用的是默认域 **0**。

后果是 DDS 域不一致，**两边互相看不见对方的话题**：

| 现象 | 真相 |
|---|---|
| 脚本报"收不到 /odom"，但节点明明活着 | 栈在域 5、检查在域 0 |
| "节点卡死、进程活着但不发数据" | 同上，节点其实是好的 |
| 用脚本 start 起来就正常，用户手动起就不行 | start 的启动和检查在同一个域 |
| **节点崩溃循环、`serial port opened` 后立刻抛 SerialException** | 两份 launch 一个域 5 一个域 0 —— DDS 里不冲突，**但串口是独占的，两个进程在死磕** |

**已修**：把 `ROS_DOMAIN_ID=5` 写进 `/etc/environment`（系统级，PAM 会给所有
登录会话设上），`chassis_bringup.sh` 里也兜了一层。

> 排查时的教训：`ros2 node list` 空、`ros2 topic echo` 报
> "Could not determine the type for the passed topic"，**先别怀疑节点** ——
> 先确认 `echo $ROS_DOMAIN_ID` 在**当前这个 shell** 里是多少。
> （另外 daemon 缓存过期也会有类似症状，`ros2 daemon stop; ros2 daemon start`。）

### 踩过的坑：`setup.bash` 里有失效的父工作区链

**在脚本里 source `~/smart_ws/install/setup.bash` 会报：**

```
not found: "/home/smart/nav2_3d/install/local_setup.bash"
```

原因：`setup.bash` 里硬编码了一串"当初建这个工作区时 source 过的父工作区"，
其中 `nav2_3d` 已经删掉了，而它用的是不带存在性检查的
`_colcon_prefix_chain_bash_source_script`。

`.bashrc` 走的是 `local_setup.bash`（没有这个链），所以只在脚本里报。

**已修**：把那两行从 `smart_ws/install/setup.bash` 删掉了（备份 `.orig`）。
注意这是**生成的文件**，以后重新 `colcon build` 且当时 source 着别的工作区，
它可能又被写回来。

---

## 6. 常用命令

```bash
# 启动 / 停止
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
pkill -f "ros2 launch"; pkill -f wheeltec_robot_node; pkill -f ekf_node
pkill -f robot_state_publisher; pkill -f joint_state_publisher; pkill -f base_to_

# 看底盘在不在线
ros2 topic echo /PowerVoltage --once
ros2 topic echo /odom --field pose.pose.position --once

# 手动发速度 (测试用, 小心车会动)
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# 串口被谁占着
fuser -v /dev/wheeltec_controller
```

---

## 7. 还缺什么

| 项 | 状态 | 影响 |
|---|---|---|
| **雷达** | 未安装 | **SLAM / Nav2 完全无法工作** —— 它们唯一的环境输入就是激光。装上后要改 `robot_model.yaml` 的 `base_to_laser` |
| **转速控制精度** | 低速 64%、常用档 103%、高速 134% | **已按包线规避**：导航速度限在 0.25~0.35 m/s（见 4.5 节）。根治要么找厂家调驱动器速度环，要么在 STM32 上用扭矩模式自己做环 |
| **CH340 USB 掉线** | 硬件问题 | 内核日志实测会 `USB disconnect` + 重新枚举。软件侧已加 `chassis-watchdog`（`/odom` 停发自动重启栈，能处理崩溃和卡死）。**硬件侧要查 USB 线/接头** |
| IMU | 正常 | 加速度/陀螺仪/磁力计都验证过。陀螺仪**静止时输出 0 是模块的正常行为**，不是故障 |
| 里程计刻度 | 正常 | 轮周长实测验证（2m ÷ 3.1 圈 = 0.645m）；换算公式读码确认 |
| 电机继电器 | 已解决 | STM32 收命令自动合闸 |
| 位置锁位 | 已删除 | 曾经导致实车失控 |

### 这一路上误判过、后来纠正的结论（留个记录）

- **"陀螺仪坏了"** → 错。是模块在检测到静止时把角速度输出归零，我们基于
  "真实陀螺仪静止也有噪声"这个**没有依据的假设**造了个假故障码，白追了好几轮。
  判断陀螺仪死活只能靠"动一下看数值变不变"。
- **"节点卡死 / 收不到 /odom"** → 大部分是 **ROS_DOMAIN_ID 不一致**（见 5 节），
  节点一直是好的。
- **"手推可以标定里程计"** → 错。被反拖时只有一路轮速上报，测出来的不是刻度。
- **"轮径标定正确"**（早期结论）→ 方法无效。命令和里程计用同一个常数，比值恒为 1。
- **"节点崩溃就靠 respawn"** → 不够。卡死的进程不退出，respawn 永远不触发，必须外部看门狗。

