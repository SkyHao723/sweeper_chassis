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
    │   ├── stm32-chassis.uvprojx
    │   ├── user/                         main.c / 中断 / 库配置
    │   ├── hardware/                     外设驱动
    │   ├── sys/                          Delay
    │   ├── lib/  start/  RTE/            ST 标准外设库 / CMSIS
    │   └── DebugConfig/
    └── esp32-web-host/                   ESP32 网页遥控（Arduino）
        ├── esp32-web-host.ino
        └── README.md
```

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

Keil MDK 打开 `firmware/stm32-chassis/stm32-chassis.uvprojx`，用 **ST-Link (SWD)** 烧录。
SWD 占用 PA13/PA14，和串口不冲突。

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

## 双串口与控制权

STM32 有两个串口跑**同一套协议**：

| 口 | 引脚 | 接谁 | 角色 |
|---|---|---|---|
| USART1 | PA9 / PA10 | CH340 → RK3588 | **主口**，可随时抢占 |
| USART2 | PA2 / PA3 | ESP32 | 从口，主口空闲时才能接管 |

仲裁规则：非零命令才抢控制权 → 主口优先 → 500ms 不刷新就释放 →
**看门狗只认控制方的帧**（否则从口心跳会把看门狗一直喂着，主口掉线车也不停）。

网页上的「控制方」会显示当前谁在开车。

---

## 安全机制

| 机制 | 行为 |
|---|---|
| 网页松手 | 立即发零速度 |
| ESP32 空闲心跳 | 400ms 没收到遥控请求就补发零速度（手机锁屏、关页面都算） |
| STM32 看门狗 | 控制方 800ms 不发命令 → 刹车 + 断开继电器 |
| 上电 | 先发零速度、继电器全断，车不会自己跑 |
| 急停按钮 | 刹车 + 断开两个继电器 |

---

## 已知的坑（都踩过）

- **继电器模块必须和 STM32 共电源共地**，否则光耦半开、线圈半通电、持续嗡嗡响
- **STM32F1 的 USART 没有 RX FIFO**，收包必须用中断 + 环形缓冲，轮询一定会丢字节
- **CAN 收发器和 STM32 之间选对电平**，PA11/PA12 不是 5V 容忍
- **Blue Pill 的 PA11/PA12 同时连着 USB 座**，用 CAN 时别插那种会短接 D+/D- 的充电器
