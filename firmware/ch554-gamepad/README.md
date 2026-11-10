# CH554G 手柄接收器 → UART 转发固件

CH554G 当 **USB Host**，接一个 Xbox 360 / XInput 兼容的**无线手柄接收器**
（`VID=045E`、`PID=028E`，HID 接口 `usage_page=1 / usage=5`，输入报告 **15 字节**，约 500 Hz），
把收到的 15 字节输入报告 **原样**从 UART0 发给 STM32（115200 8N1）。

STM32 侧（`firmware/stm32-chassis-v2/user/rc.c`）按"裸 15 字节"解析，
不加帧头、不加校验、不加长度——所以这里一个字都不能多、不能少。

```
[无线手柄] ~~2.4G~~> [Xbox 接收器] --USB--> [CH554G] --UART0--> [STM32 USART3]
                                             P3.1 TXD0            PB11 RX
```

---

## 1. 15 字节报告格式（CH554 只是搬运，不解析）

| 字节 | 含义 |
|------|------|
| `[0]` | `0x00` 固定 |
| `[1..2]` | X（左侧摇杆水平），`int16` 小端，中位 **32768** |
| `[3..4]` | Y（左侧摇杆垂直），中位 **32767**，向上变大 |
| `[5..6]` | Rx（右侧摇杆水平），中位 32768 |
| `[7..8]` | Ry（右侧摇杆垂直），中位 32767 |
| `[9..10]` | Z = `32768 + (LT差 - RT差)*128` |
| `[11]` | 按钮低 8 位：A=`0x01` B=`0x02` X=`0x04` Y=`0x08` LB=`0x10` RB=`0x20` Back=`0x40` Start=`0x80` |
| `[12]` | bit0=LS、bit1=RS、bit2..5=十字键（0 中 1 上 2 右上 3 右 4 右下 5 下 6 左下 7 左 8 左上） |
| `[13]` | `0x00` 固定 |
| `[14]` | `0x00` 固定 |

## 2. 硬件连接

本板实际接线：CH554 的 **P3.0 / P3.1** 已焊到 STM32（也就是 UART0 那组脚）。

| CH554G 侧 | STM32 侧 | 方向 |
|-----------|----------|------|
| `P3.1`（TXD0） | `PB11`（USART3_RX） | 数据往 STM32 走 |
| `P3.0`（RXD0） | `PB10`（USART3_TX） | 本固件不发，留着以后收命令 |
| `GND` | `GND` | 必须共地 |
| `V33` | 3.3V | 供电 |

> 注意 1：板上丝印的 `SCS / MISO / SCK` 是 **P1.4 / P1.6 / P1.7**（SPI0 从机脚，也复用 UART1），
> 本板没接到 STM32，本固件也不用它们。
>
> 注意 2：`P3.0 / P3.1` 同时是 CH554 的 **ISP 下载口（UART0）**，
> 所以烧录时要么走 USB-A 口（方法 A），要么让 STM32 让开这两根线（方法 B）。

## 3. 编译

需要 **Keil C51**（uVision 4/5）。工程文件 `CH554Gamepad.uvproj`。

命令行编译（和 STM32 工程一样的用法）：

```powershell
$p = Start-Process -FilePath "D:\Keil\UV4\UV4.exe" `
     -ArgumentList '-b','CH554Gamepad.uvproj','-o','build.log' `
     -Wait -PassThru
"exit=$($p.ExitCode)"
Get-Content build.log | Select-Object -Last 6
```

当前版本的产物：

- **0 Error(s), 0 Warning(s)**
- `Program Size: data=16.3 xdata=794 code=8206`
- `CH554Gamepad.hex` 23965 B，MD5 `C2283A8DF7CD88BBFBDCB41C65EC2C42`
- 镜像 8206 B（0x0000~0x200D），MD5 `3526A79605E005E66F2994D411FD3031`

工程里只有 3 个源文件：`Public\Debug.C`（时钟/延时）、`USB\USB_LIB\USBHOST.C`（WCH USB Host 协议栈）、
`App\GamepadHost.C`（本应用）。`Public\` 与 `USB\USB_LIB\` 是 WCH 官方 CH554 EVT 例程原文件，
只作编译依赖，不要改。

## 4. 烧录

CH55x 的 ISP **只有两种下载方式：USB 或 UART0**，没有 SPI 下载
（所以预留的 `SCS/MISO/SCK` 不是烧录口）。烧录软件用 **WCHISPStudio / WCHISPTool**。

### 方法 A：USB 下载（推荐）

因为这片 CH554 的 USB 口就是**宿主口**（接手柄接收器的那个 USB-A），
所以要烧它，得用一个 **USB 公对公（A-A）线** 把这个口接到电脑：

1. 断电；把 USB-A 口上的手柄接收器**拔掉**
2. 用 **A-A 公对公线** 把板子的 USB-A 口接到电脑
3. 打开 WCHISPStudio → **芯片类型选"8位" → 型号 CH554 → 下载方式 USB**
4. **选择用户程序文件** `CH554Gamepad.hex`
5. 先让芯片进入 BOOT 再点下载：
   - 板上有 **DL / BOOT 焊盘或按键**的：先短接/按住，**再上电**（CH55x 是"上电瞬间检测"）
   - 没有的话：在 WCHISPStudio 里点"下载"后，**给板子重新上电**
6. 下载记录里看到"成功" → 断开 DL 短接 → 断电重上电

### 方法 B：串口下载（本板 P3.0/P3.1 已接到 STM32，需要"让开"）

WCHISPTool 的"串口下载"走的就是 **UART0：P3.0(RXD0) / P3.1(TXD0) / GND**。
本板这两根线已经焊到 STM32 的 PB10/PB11 上了，所以要先把 STM32 变成高阻：

1. **让 STM32 松手**（二选一）：
   - 断电最省事：只给 CH554 供 3.3V，STM32 不上电；
   - 或者用 DAPLink 把 STM32 按住复位：`openocd ... -c "init" -c "reset halt" -c "shutdown"`
     （复位状态下 GPIO 是高阻，不会和 USB-TTL 抢线）
2. USB-TTL（就是桌面上那根 `CH343`，`COM18`）的 **TXD → P3.0**、**RXD → P3.1**、**GND → GND**
   （可以夹在 STM32 的 PB10/PB11 那一侧，电气上就是同一根网络）
3. WCHISPStudio → 芯片 CH554 → **下载方式"串口"** → 选这个 COM 口
4. 先短接 DL / 再上电（或点下载后重新给 CH554 上电）→ 点下载
5. 烧完把线撤掉，恢复 STM32 供电

> 如果这两组脚都没引出，只能用方法 A。
> **A-A 线只接到了 CH554 的 USB 口（P3.6/P3.7）才会有 USB 下载**；
> 如果这块板把 USB-A 口的 D+/D- 从别的地方取了，就用方法 B。

## 5. 验证

1. 烧完断电，**插上手柄接收器**，再上电（接收器要在上电前/上电后插都可以，固件支持热插拔）
2. 手柄按 Xbox 键开机；接收器指示灯常亮表示已对码
3. 用示波器/逻辑分析仪看 `P3.1`：空闲高电平，有数据时是 115200 的脉冲（每 8 ms 一帧 15 字节）
4. 接 STM32 后看现象：
   - 新板 OLED 第 3 行：`RC X<n> Y<n> h<n>`（离线时是 `RC -- (no pad)`）
   - 上位机：`python tools/v2_monitor.py --port COMx --seconds 5` 看 `rc` 行
   - 把模式切到 RC（按键或 `python tools/v2_mode.py COMx rc`），推左摇杆，看遥测 `v_cmd`

## 6. 可调参数（`App/GamepadHost.C` 顶部）

| 宏 | 默认 | 说明 |
|----|------|------|
| `GAMEPAD_REPORT_SIZE` | `15` | 输入报告长度，必须与 STM32 侧一致 |
| `GAMEPAD_SEND_MS` | `8` | 转发节奏（8 ms = 125 Hz）。STM32 侧 300 ms 判离线，别改大到 >300 |
| `GAMEPAD_UART0_ALTER` | `0` | `0`=TXD0/RXD0 在 **P3.1/P3.0**；`1`=改到 **P1.3/P1.2**（`bUART0_PIN_X`） |
| `GAMEPAD_DEBUG_UART1` | `0` | 改 `1` 用 UART1（P1.7/P1.6，115200）打调试信息，和数据口分开 |

## 7. 为什么这样设计（容易踩的两点）

1. **不能"只在数据变化时发"**。WCH 官方键鼠例程把 `RxBuffer` 和 `LastBuffer` 逐字节比，
   只有变了才上报——但 STM32 侧靠帧率判断遥控是否在线（300 ms 超时），
   而且手柄不动时坐标不变，那样 STM32 会一直判离线。
   所以本固件：**缓存最近一帧，按 8 ms 固定节奏重发**。
2. **不能加帧头**。DFRobot 那篇例程转发的是 CH9350 风格 `57 AB 88 len 10 ... seq sum`，
   那是给它的上位机用的；我们的 STM32 要的是裸 15 字节。

## 8. 排错

| 现象 | 排查 |
|------|------|
| 手柄没反应、STM32 一直 `RC -- (no pad)` | 接收器有没有对码（手柄 Xbox 键 + 接收器对码键）；换一个 USB 口试 |
| CH554 完全不发数据 | 量 `P3.1` 有没有波形；没波形说明 USB 没枚举到 HID 设备 → 把 `GAMEPAD_DEBUG_UART1` 改 1，从 UART1(P1.7) 看枚举日志 |
| 发数据但 STM32 不认 | `P3.1` 是否接到 STM32 `PB11`；是否共地；STM32 侧 USART3 是否 115200 |
| 只有 15 字节以外的长度被忽略 | 代码只转发 `len == 15` 的报告；其它 HID 设备（键鼠）会被 WCH 库归成 `DEV_TYPE_KEYBOARD/MOUSE`，本固件根本不去取 |
| 手柄拔掉后 STM32 还显示在线 | 已处理：`SearchTypeDevice` 找不到设备时会清 `g_have_report`，STM32 300 ms 后判离线 |
| 波特率不准 | `FREQ_SYS=24 MHz`（内部 RC）来自 `Public/Debug.H`；UART0 用 Timer1 做波特率（`TH1=0xF3`，且 `T2MOD` 里 `bT1_CLK` 让 T1 按 Fsys 计数）→ 115384，误差 0.16%，可接受 |

## 9. 目录结构

```
firmware/ch554-gamepad/
├── App/GamepadHost.C        ← 应用代码（我们写的，改这里）
├── Public/CH554.H           ← WCH SDK：寄存器定义
├── Public/CH554.INC         ← WCH SDK：汇编用（本工程未使用，留档）
├── Public/Debug.C/.H        ← WCH SDK：CfgFsys / mDelay / UART0
├── USB/USB_LIB/USBHOST.C/.H ← WCH SDK：USB Host 协议栈（枚举、传输）
├── CH554Gamepad.uvproj      ← Keil C51 工程
├── CH554Gamepad.hex         ← 编译产物（可直接烧）
└── README.md
```

SDK 文件来自 WCH 官方 CH554 EVT 示例（USB Host 键鼠例程），仅作编译依赖。
