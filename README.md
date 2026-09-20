# sweeper_chassis — 两驱差速清扫车底盘

STM32 底盘控制器 + FOC 驱动器 CAN 总线 + ESP32 网页遥控（调试用）。
后期上位机换成 RK3588，走串口接同一条协议。

```
手机浏览器
    │ WiFi
    ▼
┌──────────────┐  UART 115200  ┌────────────────┐  CAN 500k  ┌──────────────┐
│     ESP32    │◄─────────────►│      STM32     │◄──────────►│ FOC 驱动器 ×2 │
│ (调试用上位机) │  PA2/PA3      │   (底盘控制器)   │  PA11/PA12 │  1号 / 2号    │
└──────────────┘               └───┬────────┬───┘            └──────────────┘
                                   │        │
                              PB0 ┌▼┐   PB1 ┌▼┐
                                  电机继电器  水泵继电器

后期:  RK3588 ──USB── CH340 ──UART──► STM32 PA9/PA10（主口，优先级更高）
```

---

## 目录结构

```
sweeper_chassis/
├── README.md                             本文件
├── docs/
│   ├── wiring.md                         完整接线总表（先看这个）
│   ├── chassis-serial-protocol.md        底盘串口协议说明（Wheeltec 那套）
│   └── foc-driver-protocol-v1.2.pdf      FOC 驱动器通讯协议原始文档
└── firmware/
    ├── stm32-chassis/                    STM32 底盘控制器（Keil MDK）
    │   ├── stm32-chassis-rk3588.uvprojx  ← 生产版：只接 RK3588
    │   ├── stm32-chassis.uvprojx         ← 调试版：双串口 + ESP32
    │   ├── user/                         main.c / 中断 / 库配置
    │   ├── hardware/                     外设驱动（YbImu / OLED）
    │   ├── sys/                          Delay
    │   ├── lib/  start/  RTE/            ST 标准外设库 / CMSIS
    │   └── DebugConfig/
    └── esp32-web-host/                   ESP32 网页遥控（Arduino）
        ├── esp32-web-host.ino
        └── README.md
```

---

## 两个 STM32 版本（**源码只有一份**）

两个 `.uvprojx` 打开的是**同一套源码**，区别只有一个预处理器宏
`CHASSIS_RK3588_ONLY`（写在各工程的 `C/C++ → Define` 里）：

| | `stm32-chassis-rk3588.uvprojx` | `stm32-chassis.uvprojx` |
|---|---|---|
| 用途 | **生产**：以后就用这个 | **调试**：现在用 ESP32 网页遥控 |
| 上位机 | 只有 RK3588（USART1 / PA9-PA10）| RK3588（主）+ ESP32（从，USART2 / PA2-PA3）|
| USART2 | **完全不初始化**，PA2/PA3 空着 | 初始化，跑命令和遥测 |
| 控制权仲裁 | 无（只有一个上位机）| 有（主口优先，500ms 超时释放）|
| 本地 EKF | **编译掉** | 跑（网页要画轨迹）|
| `0x7E` 位姿帧 | **不发** | 发给 ESP32 |
| 遥测流量 | 24 字节 / 50ms | 84 字节 / 50ms |
| Code 大小 | **9536 B** | 14376 B |

**为什么不做成两份源码**：修 bug 只修一处，不会出现"改了一份忘了另一份"。
从使用角度看和两份没区别 —— Keil 里打开哪个就烧哪个，产出两个独立的 `.axf`。

**改动差异的地方**都在 `main.c` 里用 `#if CHASSIS_RK3588_ONLY` / `#if EKF_ON_STM32`
圈出来了，搜这两个宏就能看到全貌。

> ⚠️ 两个版本**都还用同一套协议**（`docs/chassis-serial-protocol.md`），
> 也都有看门狗（800ms 收不到命令自动刹车 + 断继电器）。生产版并不"更不安全"。

---

## 分工

| 谁 | 干什么 |
|---|---|
| **STM32** | 解析车体速度 `(vx, vy, wz)` → 差速逆解 → CAN 下发；轮速正解上报；看门狗；继电器；可选本地 EKF |
| **ESP32** | 只负责连 WiFi + 跑网页遥控器 + 串口转发。**不做任何运动学计算** |
| **RK3588**（后期）| 真正的上位机，跑 ROS 2 + EKF。接 STM32 的主串口 |

**计算放哪**：默认 `EKF_ON_STM32 = 1`，STM32 自己算位姿好让网页画轨迹。
接上 RK3588 之后改成 `0`，EKF 交回上位机，STM32 只按协议报车体速度。

---

## 快速开始

### 接线

**先读 [`docs/wiring.md`](docs/wiring.md)**，里面有全部引脚、电源要求和排错顺序。

最容易踩的三个坑：

1. **所有板子必须共地**（继电器不共地会持续嗡嗡响）
2. **串口 TX/RX 要交叉接**（TX 接对方的 RX）
3. **CAN 收发器选 3.3V 的**（SN65HVD230），TJA1050 是 5V 的，直连 PA11 有风险

### 编译烧录 STM32

Keil MDK 里**打开哪个 `.uvprojx` 就是哪个版本**：

```
firmware/stm32-chassis/stm32-chassis-rk3588.uvprojx    ← 生产版（只接 RK3588）
firmware/stm32-chassis/stm32-chassis.uvprojx           ← 调试版（接 ESP32）
```

用 **ST-Link (SWD)** 烧录。SWD 占用 PA13/PA14，和串口不冲突。

两个工程输出到不同的目录（`Objects-rk3588/` 和 `Objects/`），**互不干扰**，
可以随时来回烧，不会出现增量编译串味。

### 编译烧录 ESP32

Arduino IDE 打开 `firmware/esp32-web-host/esp32-web-host.ino`，开发板选 `ESP32 Dev Module`。

WiFi 凭据写在源码顶部：

```cpp
static const char *WIFI_SSID     = "...";
static const char *WIFI_PASSWORD = "...";
```

上电后 OLED 会显示分到的 IP，手机连同一个路由器打开这个 IP 就是遥控器。

---

## 底盘参数

分散在 `firmware/stm32-chassis/user/main.c` 顶部，**换车必须重新标定**：

```c
#define WHEEL_DIAMETER_MM   205.0f   /* 车轮直径 */
#define TRACK_WIDTH_MM      930.0f   /* 左右轮触地间距 */
#define MAX_RPM             200      /* 单轮转速上限 */

#define MOTOR1_IS_LEFT      1        /* 1号电机在左侧 */
#define MOTOR1_INVERT       0        /* 某个轮子转向相反时改成 1 */
#define MOTOR2_INVERT       1        /* 实测 2 号电机正方向是反的 */
```

方向不对照着 `docs/wiring.md` 的校准表改，别硬调混控逻辑。

---

## 双串口与控制权（仅调试版）

调试版里 STM32 有两个串口跑**同一套协议**：

| 口 | 引脚 | 接谁 | 角色 |
|---|---|---|---|
| USART1 | PA9 / PA10 | CH340 → RK3588 | **主口**，可随时抢占 |
| USART2 | PA2 / PA3 | ESP32 | 从口，主口空闲时才能接管 |

仲裁规则：非零命令才抢控制权 → 主口优先 → 500ms 不刷新就释放 →
**看门狗只认控制方的帧**（否则从口心跳会把看门狗一直喂着，主口掉线车也不停）。

网页上的「控制方」会显示当前谁在开车。

生产版只有一个上位机，这套仲裁整个编译掉了。

---

## RK3588 侧的准备

底盘通过 **CH340 (`1a86:7523`)** 接 RK3588 的 USB。厂家的 udev 规则只认
CP2102 (`10c4:ea60`) 和 CH343 (`1a86:55d4`)，**不认普通 CH340**，
所以要自己加一条（已经加在车上那台 RK3588 上了）：

```udev
# /etc/udev/rules.d/wheeltec_controller_ch340.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"
```

普通 CH340 **没有唯一序列号**（`ID_SERIAL=1a86_USB_Serial`），只能按 VID:PID 匹配。
以后要是加第二个 CH340，改成用 `KERNELS=="3-1.2"` 指定 USB 物理口。

生效后 `/dev/wheeltec_controller -> ttyUSB*`，厂商的 launch 文件不用改。

---

## 安全机制

| 机制 | 行为 |
|---|---|
| 网页松手 | 立即发零速度 |
| ESP32 空闲心跳 | 400ms 没收到遥控请求就补发零速度（手机锁屏、关页面都算） |
| STM32 看门狗 | 上位机 800ms 不发命令 → 刹车 + 断开继电器（**两个版本都有**）|
| 上电 | 先发零速度、继电器全断，车不会自己跑 |
| 急停按钮 | 刹车 + 断开两个继电器 |

---

## 已知的坑（都踩过）

- **继电器模块必须和 STM32 共电源共地**，否则光耦半开、线圈半通电、持续嗡嗡响
- **STM32F1 的 USART 没有 RX FIFO**，收包必须用中断 + 环形缓冲，轮询一定会丢字节
- **CAN 收发器和 STM32 之间选对电平**，PA11/PA12 不是 5V 容忍
- **Blue Pill 的 PA11/PA12 同时连着 USB 座**，用 CAN 时别插那种会短接 D+/D- 的充电器
- **不要用「位置模式锁位」实现待机时锁死轮子**（重要，已踩）。做法是先刹停、
  再切 `MODE_POSITION`、然后每个周期死守驱动器回码里的位置。它在台架上表现很好
  （推一下会弹回原位），但**实车上直接失控**：驱动器被切出速度模式后，靠一路
  陈旧的绝对位置目标维持，多出「模式切换」和「位置目标」两个变量。
  现已删除，停车路径是**无状态**的——每个控制周期重发同一条帧，不记忆、不切换。
  想调待机手感只改 `main.c` 里的 `STOP_CTRL` 一个宏（`CTRL_BRAKE` 动态制动 /
  `CTRL_DISABLE` 自由滑行 / `CTRL_ENABLE`+0）。实测 `CTRL_BRAKE` 在低速时制动力
  很弱，慢慢推仍能推动——这是这台驱动器的固有特性，用锁位去换它的代价不值得。
- **原地转向时两个轮子反向转是正常的**，不是故障。左轮 `vx - wz·b/2`、右轮
  `vx + wz·b/2`，原地转（`vx=0`）时两轮等速反向。看到「内轮倒转」别急着断电。
