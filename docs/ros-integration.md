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

### 3.4 厂商节点改动：滚刷/水泵转发通路（补丁）

**为什么必须改厂商节点**：STM32 只认串口上的 11 字节帧，而这个串口被
`wheeltec_robot_node` **独占** —— 别的进程（包括我们自己的脚本）根本写不进去。
所以滚刷/水泵这类新增外设的控制，只能由厂商节点代为转发。

改动很小，**纯新增、0 删除**，全部集中在两个文件：

| 文件 | 加了什么 |
|---|---|
| `src/wheeltec_robot.cpp` | `ChassisRelay_Callback` / `Send_Relay_Frame` / `Relay_Hold_Callback`；构造函数里注册订阅 + 200ms 定时器 |
| `include/.../wheeltec_robot.h` | 订阅者 / 掩码 / 定时器成员，三个方法声明，`#include <chrono>` |

**补丁存在本仓库**：`vendor-patches/chassis_relay_vendor.patch`（138 行）。

```bash
# 应用 (路径相对 ~/smart_ws, 必须在 ~/smart_ws 下用 -p0)
cd ~/smart_ws && patch -p0 < <本仓库>/vendor-patches/chassis_relay_vendor.patch
colcon build --packages-select turn_on_wheeltec_robot

# 撤销 (smart_ws 自己就是 git 仓库, 这两个文件被跟踪)
cd ~/smart_ws && git checkout -- src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp \
    src/turn_on_wheeltec_robot/include/turn_on_wheeltec_robot/wheeltec_robot.h
```

用法与**那个 200ms 定时器为什么不能省**，见 `README.md` 的继电器一节。

> ⚠ `Send_Data` 是节点里的共享发送缓冲，定时器回调和 `/cmd_vel` 回调都会写它。
> 安全的前提是**单线程** —— 本节点用的是 `rclcpp::spin_some()`，回调天然串行。
> 如果以后改成 `MultiThreadedExecutor`，这两处必须加锁。
>
> ⚠ `smart_ws` 这个 git 仓库里有若干**未提交**的改动（`ekf.yaml`、
> `wheeltec_param.yaml`、`base_serial.launch.py`、以及本补丁）。它们只存在于车上的
> 工作区，本仓库的文档记录了内容但**不是可执行备份** —— 换机器时别只拷本仓库。

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
| 原地左转 0.5 rad/s | **81%** | — | — | ⚠️ 起步要 1.5~2 秒，见 4.6 |
| 原地右转 0.5 rad/s | **71%** | — | — | ⚠️ 右转起步更差，见 4.6 |

### 结论：把导航速度限在 0.25~0.35 m/s

**这个区间速度 103%、漂移 −3.2°/m，基本是完美的。** 低速（<0.2 m/s）既慢又偏，
高速（>0.4 m/s）超速。

> ⚠️ **这张表是加轮速外环之前测的。** 低速档（0.15 m/s → 64%）和高速档
> （0.50 m/s → 134%）都是**稳态偏差**，正是外环要修的东西。烧录外环之后
> **必须重测这条包线**，别拿旧数字去配 Nav2。（原地转起步那部分另说，见 4.6。）

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

## 4.6 原地转向为什么会偏：拿 IMU 当"真值"查过一遍

起因是两套工具测出来的数差得很远（同一组 0.5 rad/s 原地转，一个说 43%，
一个说 82%），必须查清是谁在说谎。**光比"指令 vs 轮速里程计"是查不出来的**
—— 因为那两者本来就是同一个数：

```
STM32 轮速 -> 24B 遥测 [6-7]=wz -> 厂商节点直接取用 -> /odom.twist.angular.z
```

`/odom` 的角速度就是 STM32 按轮速算出来的那个值，中间没有任何独立加工。
比值恒等于 1，什么都验证不了。必须找一个**跟轮子完全无关**的参照：**IMU 陀螺仪**。

工具 `tools/turn_truth.py`：同时记录 `/odom`、`/odom_combined`、`/imu/data_raw`，
按时间做梯形积分，并打印逐秒的平均角速率。

实测（命令 wz = ±0.5 rad/s，持续 4 秒，期望转过 ±114.6°）：

| 来源 | 左转 | 右转 |
|---|---|---|
| `/odom` 纯轮速 | +92.7°（81%） | −81.3°（71%） |
| `/odom_combined` EKF | +83.8°（73%） | −81.2°（71%） |
| `/imu/data_raw` 陀螺仪 | **+84.8°（74%）** | **−83.6°（73%）** |

**三者一致，差不到 10%。里程计没有说谎 —— 车是真的没转够。**

逐秒平均角速率（rad/s）把真相摆得很清楚：

```
左转  第0秒 +0.221(44%)  第1秒 +0.350(70%)  第2秒 +0.609(122%)  第3秒 +0.441(88%)
右转  第0秒 -0.067(13%)  第1秒 -0.245(49%)  第2秒 -0.622(124%)  第3秒 -0.488(98%)
```

**起步要 1.5~2 秒、第 2 秒还过冲 22%、最后稳在 88~98%。** 原地转只需要 ±21 RPM，
正好落在驱动器低速死区里（见 4.5：目标 18 RPM 时实际在 0~22 之间冲）。
右转第一秒只有 13%，比左转的 44% 差得多 —— 左右不对称就是这么来的。

### 结论

1. **EKF 融合出来的数据本身是真实的** —— `/odom_combined` 和 IMU 几乎重合
   （73% vs 74%）。它没骗人，它忠实反映了"车真的只转了这么多"。
2. **"跑起来有偏移"的主因是底盘没执行到位**，不是融合或上报的误差。
3. **STM32 在这一环是完全开环的**：`Drive_Apply()` 算完目标轮速就直接发，
   从不回读驱动器上报的实际转速做修正，所以它对"没转够"一无所知。
4. 陀螺仪 Z 轴符号**验证正确**：左转为正、右转为负，`IMU_GYRO_Z_SIGN 1` 不用改。
5. 停车后 IMU 残余速率 0.0000 rad/s，而 `/odom` 残余 −0.059 rad/s（约 3°/s）：
   IMU 说明车没在转，是某个轮子有轻微蠕动或回码残值。

### 已经动手改的：STM32 轮速外环修正

原来 STM32 是**纯开环** —— `Drive_Apply()` 算完目标轮速直接发，从不回读实际转速，
所以它对"没做到位"一无所知。现在加了一层 `Trim_Apply()`：**慢积分 + 定时助推**，
按驱动器回传的实际转速去修目标值。

- **积分**只修稳态偏差（`TRIM_KI=0.6`），不介入快速动态。
- **助推**（`KICK_RPM=12`，400 ms）专门对付起步的静摩擦 —— 那需要"立刻给力"，
  而积分天生要等误差攒起来；把积分调到能立刻给力，它就会在轮子追上来之后迟迟
  不回，反而造成 175% 的过冲和几秒的振荡（第一版实测就是这样）。
- **限幅按名义值比例**（`+50% / -50%`），所以指令永远落在 `[0.5, 1.5]` 倍名义值
  之内、**方向永远不变**。第一版用绝对 ±30 RPM，在名义值只有 13 RPM 的原地转上
  把指令顶成了 `13-30 = -17`，**轮子反转**。

**目标为 0 时恒定输出 0**，所以它不可能在停车指令下把车带着走。实现细节、第一版
的完整事后分析、调参方法见 `README.md` 的"轮速外环修正"。

**外环改的是下发值，里程计不受影响** —— `Wheel_Update()` 用的仍是驱动器回传的
实际转速，所以上报给上位机的轮速依然诚实，EKF 拿到的还是真数据。

**已被推翻的判断**：一度以为"轮速低于约 20 RPM 时驱动器才开始不稳，绕开低速即可"。
wz 扫描否掉了它 —— **即使目标高达 43 RPM，第 0 秒也只有 26%**，而直线运动 0.07 秒
就到 80%。同一个轮速区间差这么多，所以问题不在低速死区，而在**原地转本身要克服
轮胎刮擦/静摩擦**：轮子是真的没转（不是空转打滑），把指令顶上去它就会转。

**测量前必须先清场**：诊断脚本僵住不退出会一直占着串口，那样两个进程读同一个
tty、字节被随机分走，测出来的东西全废而且极难察觉（实测白测了一整轮）。现在三个
工具都会自检 `/cmd_vel` 的发布者数并拒绝在有污染时测量；`decode_diag --drive`
还需要先 `touch /tmp/chassis_watchdog.pause` 把看门狗暂停（否则它会反复把 ROS
节点拉回来抢串口）。完整教训见 `README.md` 的"已知的坑"。

### 还没解决的

- **EKF 没有任何绝对偏航参照**（没融合磁力计、也没雷达），航向长期会漂。
  这是"EKF 数据真实性"目前唯一的真实缺口，要等雷达装上、跑起 SLAM 才能治本。
- **第二版外环还没上车验证**，`KICK_RPM` / `TRIM_KI` 都只是首猜值。
- `chassis_check.py` 的转向用例窗口偏短（`SETTLE_S=0.6`），而原地转起步本身就要
  1.5~2 秒，所以它报出来的转向百分比天生偏低；要看转向真值请用 `turn_truth.py`。
- `can_err` 报的是**开机以来的累计值**（`uint8`，会饱和在 255）。所以看到
  `can_err=255` 无法区分"曾经错过 255 次"和"现在还在错"。想判断当前有没有问题，
  只能看它有没有在涨。

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

仓库 `tools/` 下的诊断脚本（都部署在 RK3588 的 `~/chassis_tools/`）：

```bash
# 部署后总检查: 看门狗状态 / 栈唯一性(重复启动会抢串口) / 串口占用 / odom 频率
bash ~/chassis_tools/post_deploy_check.sh

# 转向"真值"对照: 里程计 vs IMU 陀螺仪, 带逐秒角速率 (★ 车会真的原地转)
python3 ~/chassis_tools/turn_truth.py 0.5 4      # 左转 4 秒
python3 ~/chassis_tools/turn_truth.py -0.5 4     # 右转 4 秒

# /odom 是谁发的、字段内容对不对 (查"里程计是不是在说真话"用这个)
bash ~/chassis_tools/inspect_odom.sh

# 改完 ekf.yaml 校验还能不能解析、关键参数有没有被动过
bash ~/chassis_tools/check_ekf_yaml.sh

# 同一工况连跑几次看可复现性 —— 驱动器低速环本身在抖, 单次结果不可信
for i in 1 2 3; do python3 ~/chassis_tools/chassis_check.py --turn-only --dur 3; done
```

---

## 7. 还缺什么

| 项 | 状态 | 影响 |
|---|---|---|
| **雷达** | 未安装 | **SLAM / Nav2 完全无法工作** —— 它们唯一的环境输入就是激光。装上后要改 `robot_model.yaml` 的 `base_to_laser` |
| **转速控制精度** | 原地转 95~102%（v2.2 实测）；直线受驱动器速度环欠阻尼影响（过冲约 186%）| 原地转的**外环修正**已上车验证有效（4.6 节 / README）。直线的锅在**驱动器内部的速度环**，外环够不着 —— 详见 README"直线精度的锅不在外环" |
| **CH340 USB 掉线** | 硬件问题 | 内核日志实测会 `USB disconnect` + 重新枚举。软件侧已加 `chassis-watchdog`（`/odom` 停发自动重启栈，能处理崩溃和卡死）。**硬件侧要查 USB 线/接头** |
| IMU | 正常 | 加速度/陀螺仪/磁力计都验证过。陀螺仪**静止时输出 0 是模块的正常行为**，不是故障 |
| 里程计刻度 | 正常 | 轮周长实测验证（2m ÷ 3.1 圈 = 0.645m）；换算公式读码确认 |
| 滚刷/水泵继电器 | 已解决 | PB0 滚刷、PB1 水泵 ——**和轮毂电机无关**（驱动器常电）。两条通路：台架 `decode_diag.py --relay`，ROS 里 `ros2 topic pub /chassis_relay std_msgs/msg/UInt8 "{data: 1}"`。ROS 侧靠在厂商节点里加转发通路实现，补丁见 `vendor-patches/chassis_relay_vendor.patch` |
| 看门狗 | 已拆成两条 | **运动看门狗**（800ms 无速度帧→停车）和**链路看门狗**（800ms 无任何合法帧→断继电器）分开；混在一起会让"只发继电器帧"把车也停住、或者让显式开的继电器被自己关掉 |
| 驱动器就绪联锁 | 已加 | 两个驱动器没都回码时只发刹车，防"单轮出力 = 原地打转"。**根因（三次三种结果）未查明** |
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
- **"加了看门狗只有好处"** → 错。第一版看门狗每 10 秒新建一个节点、只转 4 秒就判
  `/odom` 死活，而**新建节点的 DDS 发现本身可能就要好几秒** —— 于是它在**健康的栈上
  反复误判重启**，日志里实打实重启了 3 次。每次重启 `/odom` 位姿归零，
  正在进行的测量被截断，表现成"车明明动了里程计只涨一点点"，而且完全不可复现，
  为此白追了好几轮。现在改成：**进程没了才立即重启；数据探测要连续失败 3 次（30 秒）
  才动手**。教训是"监控工具自己也会成为故障源"。
- **"原地转向的角度偏差是里程计/融合算错了"** → 错。`/odom` 的角速度就是 STM32 按
  轮速算出来的值，厂商节点直接取用，**跟指令同源，比值恒等于 1，用它验证它自己毫无意义**。
  换成 IMU 陀螺仪（独立物理量）一对才发现：`/odom` 81%、`/odom_combined` 73%、
  IMU 74%，**三者一致 —— 里程计是诚实的，是车真没转够**。详见 4.6 节。
- **"分母改成窗口时长就修好了"** → 只改了一半。`chassis_check.py` 的"全程比例"
  原来用 `dur` 当分母而计时段只跑了 `dur - SETTLE_S`，偏低约 30%；后来把分母改成
  `win = dur - SETTLE_S`，**但计时段还在跑 `dur` 秒** —— 于是 bug 从偏低 30% 变成
  **偏高 25%**，一样是错的，而且更难发现（数字看着挺合理）。现在两边都用 `win`。
  教训：**改比例公式时分子和分母必须一起看，只改一半等于换个方向错。**
- **"继电器给轮毂驱动器供电，断链断电所以驱动器在重新上电初始化"** → 错，而且
  是一个**名字引发的连锁误判**。PB0 一直叫 `RELAY_MOTOR`/"电机继电器"，文档还写着
  "不合闸电机就没电"，于是"电机"被读成了**轮毂驱动电机**。实际上**继电器管的是
  **滚刷**和水泵**，和轮毂驱动器毫无关系（驱动器独立常电）。
  后果：基于这个假前提写了一整轮推理（"三次不同结果是驱动器在上电初始化"），
  并把错误理由写进了代码注释和 README。**等待条件本身是对的**（单轮出力 = 原地
  打转），所以联锁保留、理由重写；名字全部改成 `RELAY_BRUSH_PIN`/"滚刷继电器"。
  教训：**命名错了会把错误前提送进推理链，而且看起来完全合理。**

