# CH554G 手柄接收器 → UART 转发固件

CH554G 当 **USB Host**，接一个 Xbox 360 / XInput 兼容的**无线手柄接收器**
（`VID=045E`、`PID=028E`，HID 接口 `usage_page=1 / usage=5`，输入报告 **15 字节**，约 500 Hz），
把收到的 15 字节输入报告 **原样**从 UART1 发给 STM32（115200 8N1）。

STM32 侧（`firmware/stm32-chassis-v2/user/rc.c`）按"裸 15 字节"解析，
不加帧头、不加校验、不加长度——所以这里一个字都不能多、不能少。

```
[无线手柄] ~~2.4G~~> [Xbox 接收器] --USB--> [CH554G] --UART1--> [STM32 USART3]
                                             P1.7 TXD1            PB11 RX
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

| CH554G 侧 | 板上丝印 | STM32 侧 |
|-----------|----------|----------|
| `P1.7`（TXD1） | `SCK` | `PB11`（USART3_RX） ← 数据往这个方向 |
| `P1.6`（RXD1） | `MISO` | `PB10`（USART3_TX） 本固件不发，留着以后收命令 |
| `P1.4` | `SCS` | 不用 |
| `GND` | `GND` | `GND`（必须共地） |
| `V33` | `3.3V` | 3.3V |

> 注意：CH554 的 `P1.6 / P1.7` 同时是 **UART1**（RXD1/TXD1）和 **SPI0**（MISO/SCK）的默认引脚。
> 板子上写的是 SPI 的名字，但这里按 **UART1 使用**，两个功能不会同时生效。

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
- `Program Size: data=16.3 xdata=799 code=8206`
- `CH554Gamepad.hex` 23965 B，MD5 `1FE9EB6500A271385CF24A8CA1C4FA1F`
- 镜像 8206 B，MD5 `B697143D6C616E5625ED8FBE5AD488D4`

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

### 方法 B：串口下载（备用）

如果 CH554 的 UART0 引脚（**P3.1 = TXD0、P3.0 = RXD0、GND**）引到了排针/焊盘上，
就不用 A-A 线：

1. USB-TTL 接 `P3.0`、`P3.1`、`GND`（TTL 电平，不要接 RS232）
2. WCHISPStudio → 芯片 CH554 → **下载方式"串口"** → 选对应 COM 口
3. 同样先短接 DL / 再上电，然后点下载

> 如果这两组脚都没引出，只能用方法 A。

## 5. 验证

1. 烧完断电，**插上手柄接收器**，再上电（接收器要在上电前/上电后插都可以，固件支持热插拔）
2. 手柄按 Xbox 键开机；接收器指示灯常亮表示已对码
3. 用示波器/逻辑分析仪看 `P1.7`：空闲高电平，有数据时是 115200 的脉冲（每 8 ms 一帧 15 字节）
4. 接 STM32 后看现象：
   - 新板 OLED 第 3 行：`RC X<n> Y<n> h<n>`（离线时是 `RC -- (no pad)`）
   - 上位机：`python tools/v2_monitor.py --port COMx --seconds 5` 看 `rc` 行
   - 把模式切到 RC（按键或 `python tools/v2_mode.py COMx rc`），推左摇杆，看遥测 `v_cmd`

## 6. 可调参数（`App/GamepadHost.C` 顶部）

| 宏 | 默认 | 说明 |
|----|------|------|
| `GAMEPAD_REPORT_SIZE` | `15` | 输入报告长度，必须与 STM32 侧一致 |
| `GAMEPAD_SEND_MS` | `8` | 转发节奏（8 ms = 125 Hz）。STM32 侧 300 ms 判离线，别改大到 >300 |
| `GAMEPAD_UART1_ALTER` | `0` | `0`=TXD1 在 **P1.7**；`1`=改到 **P3.2**（`bUART1_PIN_X`） |
| `GAMEPAD_DEBUG_UART0` | `0` | 改 `1` 用 UART0（P3.1，115200）打调试信息（需要电脑能看串口） |

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
| CH554 完全不发数据 | 量 `P1.7` 有没有波形；没波形说明 USB 没枚举到 HID 设备 → 打开 `GAMEPAD_DEBUG_UART0` 从 UART0 看枚举日志 |
| 发数据但 STM32 不认 | `P1.7` 是否接到 STM32 `PB11`；是否共地；STM32 侧 USART3 是否 115200 |
| 只有 15 字节以外的长度被忽略 | 代码只转发 `len == 15` 的报告；其它 HID 设备（键鼠）会被 WCH 库归成 `DEV_TYPE_KEYBOARD/MOUSE`，本固件根本不去取 |
| 手柄拔掉后 STM32 还显示在线 | 已处理：`SearchTypeDevice` 找不到设备时会清 `g_have_report`，STM32 300 ms 后判离线 |
| 波特率不准 | `FREQ_SYS=24 MHz`（内部 RC）来自 `Public/Debug.H`；`SBAUD1 = 0 - 24e6/16/115200 = 0xF3` → 实际 115384，误差 0.16%，可接受 |

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
