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

> 注意 1：板上丝印的 `SCS / MISO / SCK` 是 **P1.4 / P1.6 / P1.7** —— 既是 SPI0 从机脚，
> 也是 **UART1（RXD1/TXD1）**；本板这组脚预留成了**烧录口**（MISO/SCK/GND 接 USB-TTL），
> 没有接到 STM32，本固件也不用它们。
>
> 注意 2：`P3.0 / P3.1` 是 **UART0（数据口）**，**不承担 ISP 下载**；
> CH554 的串口 ISP 走 **UART1 = P1.6(RXD1) / P1.7(TXD1)**，也就是板子上预留的
> `MISO / SCK` 那两个脚（见 §4）。数据口和下载口分开，互不影响。

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

CH55x 的 ISP 有 **两条路：USB 下载 或 串口下载**，没有 SPI 下载。
但注意 **CH554 的串口 ISP 走的是 UART1（P1.6 = RXD1 / P1.7 = TXD1）**，不是 UART0：

> WCH《CH554 评估版说明》§3.1.3 原文："串口下载使用芯片的 **P16（RXD）和 P17（TXD）** 引脚，
> 将图1.1 ⑬选择**串口1**"；跳线表里也把"串口1(P16,P17)"标成 **ISP 下载串口**，
> 而"串口0(P30,P31)"只是**程序默认调试串口**（57600bps）。
>
> 所以本板预留的 **`MISO`(=P1.6/RXD1) / `SCK`(=P1.7/TXD1) / `GND`** 就是串口下载口，
> `SCS`(P1.4) 不参与。**数据口（P3.0/P3.1）和下载口（P1.6/P1.7）是分开的，互不影响**。

烧录软件用 **WCHISPStudio / WCHISPTool**（本机还没装，去
<https://www.wch.cn/downloads/WCHISPTool_Setup.html> 下）。

### 方法 A：串口下载（推荐，用预留的 MISO/SCK/GND 三根）

1. 用一个 **USB-TTL**（比如桌面那根 CH343，设备名 `COM18`）：
   - USB-TTL 的 **TXD → CH554 `P1.6`（板上丝印 `MISO`）**
   - USB-TTL 的 **RXD → CH554 `P1.7`（板上丝印 `SCK`）**
   - **GND 必须共地**
2. WCHISPStudio → 芯片类型"8位" → 型号 `CH554` → **下载方式选"串口"** → 选中这个 COM 口
3. **选择用户程序文件** = `CH554Gamepad.hex`
4. 让芯片进入 BOOT：**先点"下载"，再给 CH554 断电重上电**（CH55x 是上电瞬间检测；
   空片（还没烧过程序）上电天然就在 BOOT，直接点下载即可；以后有程序了就要"短接 DL 焊盘/按住键再上电"）
5. 下载记录出现"成功"→ 断电重上电，把线撤掉

> 不需要动 STM32、不需要断电：P1.6/P1.7 跟 STM32 没有连接。
> 下载时 CH554 的串口是 57600 还是自适应由 BOOT 决定，WCHISPStudio 会自动协商。

### 方法 B：USB 下载（需要一个 USB A-A 公对公线）

这片 CH554 的 USB 口就是**宿主口**（接手柄接收器的那个 USB-A），所以要烧它得用 A-A 线把这个口接到电脑：

1. 断电；把 USB-A 口上的手柄接收器**拔掉**
2. 用 **A-A 公对公线** 把板子的 USB-A 口接到电脑
3. WCHISPStudio → **8位 → CH554 → 下载方式 USB** → 选 `CH554Gamepad.hex`
4. 点下载，再给板子上电（有 DL 焊盘就先短接再上电）
5. 下载成功 → 断开短接 → 断电重上电

> 前提是这块板的 USB-A 口 D+/D- 直接接到 CH554 的 P3.6/P3.7（设计上是这样）。
> 如果 A-A 线接上去 WCHISPStudio 认不到设备，就用方法 A。

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
