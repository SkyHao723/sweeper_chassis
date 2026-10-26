# TB01 底盘控制固件 — 达妙 DM-H65 桥

STM32F103 两驱差速底盘控制器。分支 **`damiao-bridge`**。

固件只做两件事，**故意不做第三件**：

1. **忠实执行上位机命令** —— 车体速度 `(vx, wz)` → 差速逆解 → 达妙速度模式 CAN 帧；
2. **忠实上报电机状态** —— 轮速/扭矩/位置/温度/故障码/反馈年龄 + IMU 六轴。

> **不做**：轮速闭环、助推、偏航捕获、自检点动、里程计、EKF。
> 航向保持与定位都在上位机（RK3576 / ROS 2）。责任边界见
> [`docs/division-of-labor.md`](docs/division-of-labor.md)（该文档写于 FOC 时代，
> 但"底盘只做忠实执行器"这条结论对现在依然成立）。

```
┌──────────────┐  USB 115200  ┌────────────────┐  CAN 1 Mbps  ┌─────────────────┐
│   RK3576     │◄────────────►│     STM32      │◄───────────►│ 达妙 DM-H65 ×2   │
│ ROS 2 humble │   CH340      │  F103 底盘板   │  PA11/PA12   │ 左#2 / 右#1      │
└──────────────┘  PA9/PA10    └────────────────┘  标准帧       └─────────────────┘
                                     │
                    PB9  ────────────┴── 心跳灯（500 ms 翻转，只表示主循环活着）
                    PB10/PB11 ─────────── 亚博 YbImu（软件 I2C，地址 0x23）
```

---

## 1. 车型（两辆车共用这一份固件）

两辆车**只有轮距不同**，电机、CAN、协议、IMU 全部一样。轮距由 **Keil target 的编译宏**
`TRACK_WIDTH_MM` 传入，`main.c` 里**没有默认值** —— 不传就直接 `#error` 编不过。

| | 扫地车 `sweeper` | 洗地车 `scrubber` |
|---|---|---|
| 电机 | 达妙 DM-H65 (DM6540-1EC) | 同款 |
| 轮径 | 171 mm | 171 mm |
| **轮距** | **716 mm** | **456 mm** |
| 编译宏 | `-DTRACK_WIDTH_MM=716` | `-DTRACK_WIDTH_MM=456` |
| 产出 | `Objects/sweeper/sweeper.hex` | `Objects/scrubber/scrubber.hex` |

**几何、换算系数、加新车型的步骤、以及"怎么在车上验证烧的是哪个车型"，
全部在 [`docs/vehicle-params.md`](docs/vehicle-params.md)。**

> ⚠️ **轮距写错是安全级错误**：转向增益会差 `716/456 = 1.570` 倍，
> 而**直行完全不受影响** —— 低速直线测试会"看起来正常"，问题只在转弯和导航时暴露。

---

## 2. 关键参数（`firmware/stm32-chassis/user/main.c`）

| 宏 | 值 | 说明 |
|---|---|---|
| `WHEEL_R_M` | `0.0855f` | 轮半径，171 mm 轮径的一半（两车相同） |
| `TRACK_WIDTH_M` | `TRACK_WIDTH_MM/1000.0f` | **由 target 传入**，无默认值 |
| `MAX_OMEGA_RAD_S` | `30.0f` | 单轮角速度上限 ≈ 2.565 m/s |
| `DM_DIR_SIGN` | `-1.0f` | 电机方向；**命令侧和反馈侧都乘** |
| `CTRL_PERIOD_MS` | `20` | 控制/下发周期 50 Hz |
| `STATUS_PERIOD_MS` | `50` | 状态上报 20 Hz |
| `CMD_TIMEOUT_MS` | `800` | 上位机命令超时 → **只清零，不失能** |
| `FB_TIMEOUT_MS` | `2000` | 电机反馈超时 → 标 offline + 目标清零 |
| `STATUS_VERSION` | `2` | 诊断帧 `[30]`，v2 = 支持主机功能帧 `0x06` 重新使能 |
| CAN | 1 Mbps，**标准帧** | 达妙协议，不是旧 FOC 的 500k 扩展帧 |

**为什么不失能而是清零**：达妙驱动器**失能后不制动**（转子 3.15 kg 会自由滑行）。
要停车必须主动发 `v_des = 0`，让驱动器自己去减速。`CMD_TIMEOUT` 只把目标清零。

---

## 3. 串口协议（UART 115200，三帧）

帧同步靠"上一字节是帧尾 `0x7D`、本字节是帧头"。发送顺序固定：
先 24 B 遥测（`0x7B`），再 44 B 诊断（`0x7E`），这样诊断帧尾正好给下一个遥测帧当头。

### 3.1 速度指令帧（主机 → STM32，11 字节）

| 偏移 | 长度 | 类型 | 内容 |
|---|---|---|---|
| 0 | 1 | uint8 | 帧头 `0x7B` |
| 1 | 1 | uint8 | AutoRecharge（**忽略**） |
| 2 | 1 | uint8 | SecurityPLY（**忽略**） |
| 3–4 | 2 | int16 **BE** | `vx * 1000`（mm/s） |
| 5–6 | 2 | int16 BE | `vy * 1000`（差速车**忽略**） |
| 7–8 | 2 | int16 BE | `wz * 1000`（mrad/s） |
| 9 | 1 | uint8 | `BCC = XOR(f[0]..f[8])` |
| 10 | 1 | uint8 | 帧尾 `0x7D` |

特殊：**`f[1] == 0x06`（`FUNC_MOTOR_REINIT`）为功能帧**，不是速度帧 ——
要求 STM32 重新"写速度模式 + 使能"（上电时使能帧可能丢，这是补救手段）。

### 3.2 主遥测帧（STM32 → 主机，24 字节）

兼容厂商 `wheeltec_robot_node`（它**只**解析这一帧），字段与它逐一对齐：

| 偏移 | 内容 |
|---|---|
| 0 | 帧头 `0x7B` |
| 1 | `Flag_Stop` |
| 2–3 | `vx`（mm/s，int16 BE） |
| 4–5 | `vy` |
| 6–7 | `wz`（mrad/s） |
| 8–13 | accel 三轴 int16 BE（m/s² = raw / 1671.84） |
| 14–19 | gyro 三轴 int16 BE（rad/s = raw × 0.00026644） |
| 20–21 | 电池 mV（**本板无采样，恒 0**） |
| 22 | `BCC = XOR([0..21])` |
| 23 | 帧尾 `0x7D` |

### 3.3 扩展诊断帧（STM32 → 主机，44 字节，`0x7E`）

厂商节点**不解析**这一帧（安全，随便改），用来查"轮子到底有没有劲"。
布局和字段含义见 [`docs/chassis-serial-protocol.md`](docs/chassis-serial-protocol.md)；
在线解码工具：

```powershell
python tools/decode_diag.py --port COM3
```

关键字段：目标/实际 ω、扭矩 Nm、位置、ERR 码、MOS/线圈温度、
**左右反馈年龄**、CAN 失败计数、启动阶段、`[30]` 版本号。

> 达妙反馈的是**扭矩 Nm**，不是电流 A。固件没读 KT 寄存器，**所以不显示电流、
> 也不做换算**（换算出来是编的）。

---

## 4. 编译与烧录

工程：`firmware/stm32-chassis/stm32-chassis.uvprojx`

**★ 工程里有两个 target，选 target 就是选车型 —— 不用改源码：**

| Keil target | 车型 | 轮距 | 产出 hex |
|---|---|---|---|
| `sweeper` | 新扫地车 | **716 mm** | `Objects/sweeper/sweeper.hex` |
| `scrubber` | 洗地车 | **456 mm** | `Objects/scrubber/scrubber.hex` |

两个 target 的唯一差别就是编译宏（`TRACK_WIDTH_MM`），源码是同一份。
**每个 target 必须有自己的独立输出目录**，否则 `.o` 互相覆盖 —— 见
[`docs/archive/README-foc-era.md`](docs/archive/README-foc-era.md) 里
"2026-10-12 烧错固件"那节（真事，导航跑飞过）。

命令行编译（UV4 是 GUI，必须等它退）：

```powershell
Start-Process -FilePath "D:\Keil\UV4\UV4.exe" `
  -ArgumentList '-b','-r','stm32-chassis.uvprojx','-t','sweeper','-o','build.log' -Wait
```

验收门槛：日志里 **`0 Error(s), 0 Warning(s)`**，当前 Code ≈ 11392 B。

**烧录前必读**：车轮架空 **或** 驱动器断电。历史上在地面带载烧录时
发生过底盘原地暴冲（目标被夹到上限、单轮 −40 A），这条不能破。

**烧完怎么确认车型对不对**（别信标签，看书据）：

```powershell
# ① 两个 hex 的 MD5 必须不同
(Get-FileHash Objects\sweeper\sweeper.hex  -Algorithm MD5).Hash
(Get-FileHash Objects\scrubber\scrubber.hex -Algorithm MD5).Hash
```

② 车上验证：车轮架空，发 `vx=0, wz=+0.30`，读诊断帧 `[10..11]`/`[12..13]` 目标 ω：
**sweeper `+126/−126`，scrubber `+80/−80`**。详见 `docs/vehicle-params.md` §4。

---

## 5. 目录结构

```
TB01-CAN-USART/
├── README.md                        本文件
├── docs/
│   ├── ★ vehicle-params.md          **车型几何 / target 选择 / 车型验证**（现役）
│   ├── ★ rk3576-chassis-inventory-2026-10-22.md  上位机侧现状盘点（现役）
│   ├── chassis-serial-protocol.md   串口协议（**帧格式有效**，FOC 相关段落已过时）
│   ├── wiring.md                    接线总表（引脚有效，继电器章节已废弃）
│   ├── division-of-labor.md         责任边界（写于 FOC 时代，结论仍成立）
│   ├── ros-integration.md / nav2-reference.md   上位机接入（**数字是 FOC 时代实测，未复核**）
│   ├── foc-can-protocol.md          旧 FOC 驱动器 CAN 协议（**已归档，仅历史参考**）
│   ├── torque-mode.md               旧 FOC 力矩模式实验（**已归档**）
│   ├── foc-driver-protocol-v1.2.pdf 旧 FOC 驱动器厂家 PDF（**已归档**）
│   └── archive/README-foc-era.md    旧 README 全文（FOC 时代，含大量有用教训）
├── tools/
│   ├── car_web/                     网页遥控器（:8000，直连串口，供前期调试）
│   ├── rsh.py                       远程执行主机命令（paramiko）
│   └── decode_diag.py               解码 44 B 诊断帧
└── firmware/
    └── stm32-chassis/               Keil MDK 工程（唯一的固件）
        ├── stm32-chassis.uvprojx    两个 target：sweeper / scrubber
        ├── user/                    main.c（全部固件逻辑）
        ├── hardware/                YbImu 等外设驱动
        ├── sys/  lib/  start/  RTE/  ST 标准外设库 / CMSIS
        └── Objects/<车型>/          各 target 独立输出目录
```

---

## 6. 上位机（RK3576）

主机上目前有 **4 套底盘实现**（厂商 C++ 栈 / 新写 Python 栈 / 两套网页遥控），
串口是**独占**的，只能起一套。现状、冲突、启动命令见
[`docs/rk3576-chassis-inventory-2026-10-22.md`](docs/rk3576-chassis-inventory-2026-10-22.md)。

**厂商 ROS 2 栈日常启动（最小可用，只要 `/cmd_vel` 和 `/odom`）：**

```bash
source /opt/ros/humble/setup.bash
source /home/smart/wheeltec_ws/install/setup.bash
export ROS_DOMAIN_ID=7          # ★ 必须显式设置，三套服务各用不同的 domain
ros2 launch turn_on_wheeltec_robot base_serial.launch.py
```

- 起之前先停掉别的占串口进程：`pkill -f tb01` / `fuser -v /dev/ttyUSB0`
- `/odom` 的 frame_id 是 **`odom_combined`**（不是 `odom`）
- 它**不校验固件版本**、只解析 24 B 帧 → 与本固件完全兼容，44 B 诊断帧可无视

---

## 7. 排查顺序（车不动时）

1. **串口有没有帧**：`python tools/decode_diag.py --port COM3` 能收到 20 Hz 帧吗？
   - 收不到 → 先看 `lsusb` 有没有 `1a86:7523`（CH340）。**没有就是物理层**
     （线没插 / 线是充电线 / STM32 没上电），此时改代码无用。
2. **电机使能了吗**：诊断帧 `[18]`/`[19]` 的 ERR 应是 **1（使能）** 或 0（失能）。
   - 一直是 0 → 上电时使能帧丢了。发功能帧 `0x06`（`FUNC_MOTOR_REINIT`）重新使能，
     或复位 STM32（**必须在驱动器已上电稳定之后**复位）。
3. **目标 ω 有没有**：`[10..11]`/`[12..13]` 非零吗？
   - 零 → 上位机没在发命令，或命令看门狗超时（`flags` bit4）。
4. **实际 ω 跟不跟**：目标 vs 实际差距大 → 看扭矩、温度、ERR 码。
   - 架空实测跟随度 **99.4 %**（目标 −3.50 / 实际 −3.48 rad/s），跟不住就是真问题。

---

## 8. 相关仓库/资料

- 达妙电机原始资料与网页驱动：`D:\github\damiao`（协议、寄存器表、`dm_protocol.py`）
- 旧的 FOC 底盘固件：`main` 分支（本分支已把 FOC 部分全部删除）
