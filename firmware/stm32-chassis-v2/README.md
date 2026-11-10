# 底盘主控 V2（新板，STM32F103C8T6）

新底盘主控板固件：点灯、读按键、跑**三种工作模式**，**读 IMU（ICM-42670-P）与温湿度（AHT20）**，
并且**接遥控手柄（CH554G 转发的 15 字节 HID 报告）**，把数据打进 32 字节上行遥测帧发给上位机。

> 旧底盘（洗地车 / 扫地车）的固件仍在 [`../stm32-chassis/`](../stm32-chassis/)，稳定版归档在
> [`../releases/fw-v8.0-20261110/`](../releases/fw-v8.0-20261110/MANIFEST.md)，**两者互不影响**。

---

## 1. 这一版做了什么

| 外设 | 引脚 | 实现 | 现象 |
|------|------|------|------|
| OLED（SSD1306 128×64） | PB8=SCL, PB9=SDA | **软件 I2C**（开漏，见 §3.1） | 屏上 8 行：板子信息 + 模式 + 加速度/角速度 + 温湿度 + 计数 |
| RGB 灯（XL-5050） | PB5 | WS2812 单线时序，DWT 周期计数器打拍，**软件调光 30 %** | **颜色 = 当前模式**：蓝=上位机控制 / 绿=遥控 / 橙=推行 |
| 板载按键 SW1 | PA15 | 上拉输入（需先关 JTAG，见 §3.2） | **每按一下切模式**：HOST → RC → PUSH → HOST |
| 调试串口 | PA9=TX, PA10=RX | USART1 115200 8N1 → CH343P → USB | 上电横幅 + 模式日志 + **20 Hz 32 字节遥测帧**（§1.2） |
| IMU（ICM-42670-P） | PB6=SCL, PB7=SDA | **硬件 I2C1** 400 kHz，±4g / ±1000dps / 200 Hz | 屏幕第 4/5 行显示加速度(g)与角速度(°/s) |
| 温湿度（AHT20） | PB8/PB9（与 OLED 共总线） | 软件 I2C，**非阻塞两步采样**（触发 → 80 ms 后取） | 屏幕第 6 行显示温度(°C)与湿度(%RH) |
| 遥控手柄 | PB11=RX（USART3），PB10=TX 不用 | 115200 8N1，收 CH554G 转发的 **15 字节 HID 原始报告**，20 Hz 回发 19 字节 `0x79` 帧（§1.3） | 屏幕第 3 行 `RC X/Y/h`；`MODE_RC` 下左摇杆直接控速 |

上电后屏幕应该是（第 1 行随模式变，第 3 行是遥控状态，第 4/5/6 行随传感器数据变）：

```
TB01 CHASSIS V2
CLK 72 MODE HOST
IMU 0x68 AHT OK
RC -- (no pad)
a 0.025 0.021 1.024
g 0.3 -0.5 0.1
T 19.7C H 30.2%
RX 0 RJ 0 RUN 000012
```

串口（115200）同时打印：

```
======================================
 TB01 chassis controller V2
 modes: HOST / RC / PUSH
======================================
SYSCLK = 72 MHz
OLED   = OK (SSD1306 @0x3C, soft I2C PB8/PB9)
LED    = WS2812 on PB5, brightness 30 %
KEY    = PA15: press to switch mode
UART   = mode cmd: 7A 02 <mode> 00 00 00 <bcc> 7D
UPLINK = telemetry 32B @20Hz (see telemetry.h)
IMU    = ICM-42670-P on hardware I2C1 (PB6/PB7)
AHT20  = on soft I2C (PB8/PB9), 500ms sample

[imu] addr=0x68 whoami=0x67 OK
[aht20] OK
[mode] power on = HOST
```

### 1.1 三种工作模式

| 模式 | 编号 | 灯色 | 上位机速度帧（`0x7B`） | 屏幕第 5 行 |
|------|------|------|------------------------|-------------|
| **HOST** 上位机控制 | 0 | 蓝 | **接受**（`RX` 计数 +1） | `UART CMD OK` |
| **RC** 遥控 | 1 | 绿 | 忽略，记到 `RJ` | `REMOTE ONLY` |
| **PUSH** 推行 | 2 | 橙 | 忽略，记到 `RJ`（电机释放，人工推车） | `MOTOR FREE` |

两种切换方式，效果一样（都会刷灯 + 刷屏 + 打一行 `[mode] A -> B by key/uart`）：

1. **按板载按键 PA15**：每按一下切下一个（30 ms 消抖）。
2. **上位机发功能帧**（8 字节，`BCC = XOR(前 6 字节)`，尾 `0x7D`）：

```
上位机控制  7A 02 00 00 00 00 78 7D
遥控        7A 02 01 00 00 00 79 7D
推行        7A 02 02 00 00 00 7A 7D
```

用电脑试（串口走 CH343P，先装上 CH343 驱动，设备管理器看是哪个 COM）：

```powershell
python tools/v2_mode.py COM15 host            # 切上位机控制
python tools/v2_mode.py COM15 rc --listen 3   # 切遥控, 并监听 3 秒串口打印
python tools/v2_mode.py COM15 cycle           # HOST -> RC -> PUSH -> HOST 各切一次
python tools/v2_mode.py COM15 rc --speed 200  # 切遥控后再发一条 200mm/s 速度帧
                                              # （应被拒绝: 屏幕 RX 不变, RJ +1）
```

**亮度**：`hardware/ws2812.h` 里的 `WS2812_BRIGHTNESS_PERCENT`（默认 **30**，0~100），
直接等比缩小 RGB 数值。觉得还亮就改 20、觉得暗就改 50，改完重新编译烧录即可。

**看这两行就知道板子基本是好的**：

- `CLK 72` —— 8 MHz 晶振起振了（显示 `CLK 8` 说明 HSE 没起来、退回内部 HSI，
  查 X1、18 pF、C7、BOOT0 下拉电阻）。
- `IMU 0x68` / `AHT OK` —— 两条 I2C 总线上都有应答（`IMU` 显示 `0x00` 或 `AHT ERR` 见 §4）。

### 1.2 上行遥测帧（32 字节，20 Hz）

每 50 ms 从 USART1 发一帧。**大端 int16，和旧底盘的速度帧一致**，上位机可以复用解析代码。

```
[0]     0x7E  帧头
[1]     flags bit0=IMU 有效, bit1=AHT20 有有效样本, bit2..3=模式
[2]     mode  0=HOST 1=RC 2=PUSH
[3..4]  vx 指令 mm/s        (int16 BE)
[5..6]  wz 指令 mrad/s      (int16 BE)
[7..8]  acc X mg            (int16 BE)   ← 0.001 g
[9..10] acc Y mg
[11..12]acc Z mg
[13..14]gyro X 0.1°/s       (int16 BE)
[15..16]gyro Y 0.1°/s
[17..18]gyro Z 0.1°/s
[19..20]IMU 内部温度 0.01°C (int16 BE)
[21..22]湿度 0.01 %RH       (uint16 BE)
[23..24]AHT20 温度 0.01°C   (int16 BE)
[25..26]tick ms 低 16 位    (uint16 BE)
[27]    接受的 host 速度帧数（饱和 255）
[28]    因模式不对被丢的帧数（饱和 255）
[29]    协议版本 1
[30]    BCC = XOR([0..29])
[31]    0x7D 帧尾
```

两种读法：

```powershell
# 1) 板子 USB 插在电脑上: 实时解读 (每秒 20 行)
python tools/v2_monitor.py --port COM15
python tools/v2_monitor.py --port COM15 --seconds 5

# 2) 没插 USB(只有 DAPLink): 直接用调试器把 RAM 里"最后发出的一帧"读出来
python tools/v2_monitor.py --ram
```

`--ram` 读的是 `telemetry.c` 里的 `s_last_frame` 快照（地址从 `.map` 里查），适合板上没引 USB 时排除问题：

```
mode=HOST imu=OK aht=OK acc[g]=( 0.025  0.021  1.024) gyro[dps]=(  0.30  -0.50   0.10)
T_imu=19.63C RH=30.27% T_aht=19.75C v_cmd=(   0,    0) tick= 5800 rx=0   rj=0
```

**怎么判断数据是真的**：板子静止放在桌上 → `acc` 的 Z 轴 ≈ 1.000 g（重力）、X/Y ≈ 0，
`gyro` 三轴 ≈ 0；把板子翻转 Z 变 -1；用手捂一下 AHT20 湿度和温度会往上走。

### 1.3 遥控手柄链路（CH554G 转发 15 字节）

**分工**：Xbox 360 兼容接收器（VID `045E` / PID `028E`）插在板上那颗 **CH554G** 的 USB-A 口上，
CH554G 当 USB 主机，把 HID 输入报告**原样**从串口吐出来；STM32 用 **USART3（PB11 = RX）** 收。

CH554G 侧只要做到这三条（本固件按这个约定写）：

| 项 | 值 |
|----|----|
| 接口 | UART，**115200 8N1**，无流控 |
| 发送频率 | 接收器来一包转一包（约 500 Hz，7500 B/s，115200 够用） |
| 内容 | HID 输入报告 15 字节，**原样**，不加帧头帧尾/校验 |

**15 字节原始帧布局**（从接收器抓包逐字节对出来的）：

| 偏移 | 含义 |
|------|------|
| `[0]` | `0x00` 固定 |
| `[1..2]` | X  int16 小端，左摇杆 X，中位 **32768** |
| `[3..4]` | Y  int16 小端，左摇杆 Y，中位 **32767**，**向上变大** |
| `[5..6]` | Rx int16 小端，右摇杆 X，中位 32768 |
| `[7..8]` | Ry int16 小端，右摇杆 Y，中位 32767 |
| `[9..10]` | Z  int16 小端，`32768 + (LT-RT)*128` |
| `[11]` | 按钮低 8 位：A=0x01 B=0x02 X=0x04 Y=0x08 LB=0x10 RB=0x20 Back=0x40 Start=0x80 |
| `[12]` | bit0=LS、bit1=RS、**bit2..5=十字键**（0 中 1 上 2 右上 3 右 4 右下 5 下 6 左下 7 左 8 左上） |
| `[13..14]` | `0x00` 固定 |

**映射到运动**（`rc.c`，只在 `MODE_RC` 模式下生效）：左摇杆 Y → 线速度 vx（上限 **600 mm/s**），
左摇杆 X → 角速度 wz（上限 **800 mrad/s**，右推 = 右转 = wz 为负），死区 **10%**，30 ms 内没有新帧 → 清零。

**上行遥控帧 19 字节（20 Hz，紧跟在遥测帧后面发）**：把收到的原始 15 字节原样回给上位机，方便在上位机侧
看到"底盘到底收到什么"，也方便没有接收器时比对。

| 偏移 | 含义 |
|------|------|
| `[0]` | `0x79` 帧头 |
| `[1]` | flags：bit0=手柄在线、bit1=这一帧内容和上一帧不同 |
| `[2..16]` | 手柄原始 15 字节 |
| `[17]` | BCC = XOR([0..16]) |
| `[18]` | `0x7D` 帧尾 |

**调试注入帧 18 字节（下行，只给工具用）**：板上还没接 CH554G/接收器时，可以从调试串口把 15 字节
塞进**同一条**解析 + 映射路径（`Rc_InjectFrame()`），验证屏幕、遥测里的 vx/wz 是否按预期变化。

| 偏移 | 含义 |
|------|------|
| `[0]` | `0x78` 帧头 |
| `[1..15]` | 手柄原始 15 字节 |
| `[16]` | BCC = XOR([0..15]) |
| `[17]` | `0x7D` 帧尾 |

用 `tools/v2_gamepad.py` 注入和解读：

```powershell
# 中位一帧（屏幕上第 3 行会变成 RC X0 Y0 h0）
python tools/v2_gamepad.py COM18 center

# 左摇杆推到右上角 + 按 A + 十字键上
python tools/v2_gamepad.py COM18 send --lx 1000 --ly 1000 --a --hat 1

# 切到遥控模式, 摇杆扫一圈(每 20ms 一帧), 同时解析上行 0x79/0x7E 帧
python tools/v2_gamepad.py COM18 play --mode rc --seconds 6 --listen
```

屏幕第 3 行就是遥控状态：`RC X<n> Y<n> h<n>`（左摇杆千分制 + 十字键），最右边 `b0xNN` 是按钮字节；
离线时显示 `RC -- (no pad)`。


---

## 2. 编译与烧录

```powershell
# 编译（Keil MDK；UV4 是 GUI，必须 Start-Process -Wait 等它结束）
$uv   = 'D:\Keil\UV4\UV4.exe'
$proj = 'D:\github\TB01-CAN-USART\firmware\stm32-chassis-v2\stm32-chassis-v2.uvprojx'
$log  = 'D:\github\TB01-CAN-USART\firmware\stm32-chassis-v2\build.log'
$p = Start-Process -FilePath $uv -ArgumentList @('-b','-r',$proj,'-t','chassis-v2','-o',$log) -Wait -PassThru
"exit=$($p.ExitCode)"; Get-Content $log -Tail 5
# 要求: exit=0 且 "0 Error(s), 0 Warning(s)"
```

产物：`Objects/chassis-v2/chassis-v2.hex`
（hex 36583 B MD5 `AE6639F1EA40FAF102D09AF2B5E24F43`；镜像 12988 B MD5 `8C3CC824576770B482ED6C30A19CF478`；Code 12308 B / RO 556 B / RW 124 B / ZI 2004 B）。

**推荐用脚本烧（会自己回读校验，并处理 NRST 没接的问题）**：

```powershell
python tools/flash_hex.py firmware/stm32-chassis-v2/Objects/chassis-v2/chassis-v2.hex
```

它会做三件事：`program ... verify` → 独立 `dump_image` 回读比 MD5 → 写 AIRCR 软复位启动。
注意 openocd 的 `program {x.bin}` **不认 bin 里的地址**（会从 0x00000000 开始写，报
`no flash bank found for address 0x00000000`），所以只能喂 hex 让它自己解析地址。

手动烧（等价命令）：

```powershell
D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe `
  -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg `
  -c "adapter speed 500" `
  -c "program {D:/github/TB01-CAN-USART/firmware/stm32-chassis-v2/Objects/chassis-v2/chassis-v2.hex} verify" `
  -c "init" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown"
```

- `adapter speed 1000` 偶尔报 `Error: CMSIS-DAP command CMD_INFO failed` → 用 **500**。
- 板子的 NRST 没接调试器时，`program ... verify reset exit` 里的 reset 会失败 →
  拆成 `program ... verify` + 写 AIRCR（上面那条 `mww`）走软件复位。
- 用 ST-Link + STM32CubeProgrammer 且 NRST 没接时，`-c port=SWD mode=normal` 会报
  `DEV_TARGET_RESET_ERR`，改用 `mode=hotplug`。

---

## 3. 设计上必须知道的几个点

### 3.1 为什么显示用软件 I2C

STM32F103 的 **I2C2 只在 PB10/PB11**，而这块板把 PB10/PB11 给了 CH554G（USB-A 主机口协处理）。
原理图上标的 `PB8/PB9 = I2C2` 其实不成立 —— PB8/PB9 是 **I2C1 的重映射脚**。

所以分工是：

- **IMU（ICM-42670-P）** 走 PB6/PB7 的**硬件 I2C1**（400 kHz，`I2C_Init` + StdPeriph 轮询）
- **OLED + AHT20** 走 PB8/PB9 的**软件 I2C**（`hardware/softi2c.c`，两条设备共总线）

总线地址：OLED `0x3C`（写 `0x78`）、AHT20 `0x38`（写 `0x70`），不冲突；IMU `0x68`
（`AD0` 接高则是 `0x69`，固件两个地址都会试）。

### 3.1.1 软件 I2C 读数据必须把地址最低位置 1

`SoftI2c_ReadBytes()` 里发的是 `address | 0x01`（R/W=1）。**这个坑真踩过**：一开始写成 `address`，
从机照常回 ACK（所以"通信正常"），但它以为主机在写，一直不驱动 SDA —— 读回来**全是 `0xFF`**。
AHT20 的 BUSY 位正好是 bit7，于是一直"忙"、温度湿度恒为 0，看起来像传感器坏了/没焊。
判断特征：**读出来的字节全是 0xFF**。

### 3.2 PA15 是 JTDI，用之前必须先关 JTAG

`PA15`（按键 SW1）、`PB3`、`PB4` 复位后默认被 JTAG 占用。用 SWD 调试时初始化里要调：

```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  /* 只关 JTAG，保留 SWD */
```

`Board_SwdOnly()` 就是做这件事，`main()` 里必须在 `Board_KeyInit()` **之前**调用。

### 3.3 RGB 灯是 5V 供电、3.3V 数据

WS2812 的 VIH 约 `0.7 × VDD`。若灯珠由 5V（USB VBUS）供电，VIH ≈ 3.5 V，而 STM32 输出高只有
3.3 V —— **临界**，可能点不亮或颜色乱。三条对策（硬件侧，任选）：

1. **灯珠改由 3.3 V 供电** → 3.3 V 数据就够（最简单）；
2. 数据线加一级 74HCT 缓冲（HCT 输入阈值 2.0 V，可吃 3.3 V）；
3. 数据线串 330 Ω 只能抑制振铃，**不能解决电平不足**。

软件侧补偿不了，所以"灯不亮"先查这个，再查时序/焊点。

### 3.4 5V 来自 USB VBUS

板上只画了 +3.3V 网络。TJA1050（CAN 收发器，4.5~5.5 V）、WS2812、CH554G 都需要 5V，
而 5V 来自 USB VBUS ⇒ **不插 USB 时 CAN 和灯都不工作**。整机要么保证一直有 USB，
要么另加 5V DCDC，别指望 3.3V 供这三样。

### 3.5 PB2 = BOOT1

PB2 引到了 CN6 排针。复位瞬间 PB2 被拉高会从**内置 SRAM** 启动（表现为"程序不跑"）。
外接模块时别用 PB2 做"上电即高"的输出，或在 PB2 上加 10k 下拉。

---

## 4. 排错（屏 / 灯 / 按键不工作）

按"物理 → 时钟 → 总线 → 代码"的顺序查，别一上来改代码：

| 现象 | 先查什么 |
|------|----------|
| 串口**完全没输出** | USB 线是不是只能充电；CH343P 驱动；SWD 能否读到芯片（`Device ID 0x410`）；BOOT0 是否被拉高 |
| `CLK 8 MHZ` | 8 MHz 晶振没起振：X1 焊接、18 pF、BOOT0 下拉、`SYSCLK_FREQ_72MHz` 是否生效 |
| `OLED NO ACK` | SDA/SCL 接反；模块有没有上拉（多数 0.96" 模块自带 4.7k）；屏是否 3.3 V；地址是 0x3C 还是 0x3D；PB8/PB9 被别的代码占用 |
| 屏亮但全黑/花屏 | 初始化序列（对比度 `0x81 0x7F`、复用率 `0xA8 0x3F`、最后必须 `0xAF` 开显示） |
| 灯不亮 | §3.3 电平问题（最常见）；PB5 是否接到灯珠 DIN；灯珠方向（DIN/DOUT 反了不亮） |
| 灯亮但颜色不对 | WS2812 是 **G→R→B** 顺序，不是 RGB；`Ws2812_SetPixel(i, r, g, b)` 内部已换成 G,R,B。**正常情况灯色 = 模式**（蓝/绿/橙），出现红/白/灭说明跑的是旧固件 |
| 灯**太亮/太暗** | 调 `hardware/ws2812.h` 的 `WS2812_BRIGHTNESS_PERCENT`（默认 30，0~100），这是软件调光、等比缩小 RGB 数值 |
| 按键没反应 / 模式不切 | 是否调了 `Board_SwdOnly()`；PA15 是否被调试器占用；按下是否真把脚拉到 GND（万用表量）；消抖是 30 ms（3×10 ms 采样），按太短会漏 |
| 上位机发命令没反应 | COM 口选错（要选 CH343P 那个）；波特率必须 115200；帧必须 8 字节且 BCC 对；最快判据是屏幕第 7 行 `RX` / `RJ` 有没有变化 |
| `RJ` 一直猛涨 | `RJ` 记的是"因模式不对被丢掉的速度帧"，若在 HOST 模式下还涨说明波特率不对或帧被截断（正常应为 0） |
| 屏幕第 2 行 `IMU 0x00` | IMU 没应答：PB6/PB7 是否接对；模块有没有上拉（多数自带 4.7k）；地址是 `0x68` 还是 `0x69`（固件都会试）；`WHO_AM_I` 必须是 `0x67`（不对说明是别的型号） |
| 屏幕第 2 行 `AHT ERR` | AHT20 没应答：确认挂在 PB8/PB9 这条总线上；地址 `0x38`；上电要等 40 ms 才能通信（固件已等 45 ms） |
| 温湿度一直是 0 / 一直 `AHT ERR` | **先看是不是读回全 `0xFF`**（§3.1.1）——那是 R/W 位没置 1 的典型症状，不是传感器坏 |
| IMU 数值不动 / 全是 0 | 是不是没等陀螺稳定（固件初始化后等 60 ms）；`PWR_MGMT0` 是否写成 `0x0F`；`I2C1` 是否被重复初始化 |
| `acc Z` 不是 ±1g | 模块贴装方向不同（Z 轴朝桌面还是朝上），不是故障；拿重力当场标定 |

用 DAPLink 直接读寄存器也很好使（GPIOB 基址 `0x40010C00`）：

```
GPIOB_CRL  = 0x44484422   # PB0..PB7 的模式, 每 4 bit 一个脚 (0x2=推挽输出, 0x8=上拉输入)
GPIOB_ODR  = 0x00000c10   # 输出电平
GPIOB_IDR  = 0x0000fc10   # 实际电平
```

---

## 5. 代码结构

```
stm32-chassis-v2/
├── stm32-chassis-v2.uvprojx   单 target: chassis-v2（宏: USE_STDPERIPH_DRIVER）
├── user/
│   ├── main.c                 bring-up 主程序（屏幕 8 行 + 模式 + 传感器 + 按键 + 串口）
│   ├── board.c / board.h      板级：DWT 延时、关 JTAG、未用脚驻留、USART1(收发/发二进制)、按键
│   ├── mode.c / mode.h        工作模式状态机（HOST / RC / PUSH）
│   ├── proto.c / proto.h      上位机串口协议（0x7B 速度帧 / 0x7A 功能帧，切模式）
│   ├── telemetry.c / .h       32 字节上行遥测帧（模式/指令/IMU/温湿度/计数）
│   ├── rc.c / rc.h            遥控手柄：USART3 收 CH554G 转发的 15 字节 HID 帧，映射到 vx/wz，20Hz 回发 0x79 帧
│   ├── stm32f10x_conf.h       标准外设库裁剪
│   └── stm32f10x_it.c/h       中断向量（USART3_IRQHandler 在 rc.c 里）
├── hardware/
│   ├── ws2812.c / .h          WS2812 单线驱动（DWT 打拍，发送期间关中断）
│   ├── ssd1306.c / .h         SSD1306 驱动（软件 I2C + 5x7 字库 + 页寻址）
│   ├── softi2c.c / .h         PB8/PB9 软件 I2C 主机（OLED + AHT20 共用）
│   ├── icm42670.c / .h        ICM-42670-P 六轴 IMU（硬件 I2C1，±4g/±1000dps/200Hz）
│   └── aht20.c / .h           AHT20 温湿度（软件 I2C，非阻塞 触发→80ms 后取）
├── sys/Delay.c / .h           SysTick 微秒/毫秒延时（沿用旧工程）
├── start/                     CMSIS + 启动文件（startup_stm32f10x_md.s）
└── lib/                       ST 标准外设库 3.5.0
```

工程文件由脚本从旧工程生成（`tools/gen_v2_project.py`）：只改 target 名、输出目录、宏定义
和 user/hardware 两个文件组，编译选项与调试器配置沿用旧工程。**新增 .c 文件后要在这个脚本里登记并重跑。**

当前体积：**Code 12308 B / RO 556 B / RW 124 B / ZI 2004 B**（64 KB Flash，余量充足）。

---

## 6. 进度与下一步（按顺序）

已完成：点亮 OLED / RGB / 按键 → 亮度可调 → **三种工作模式（HOST / RC / PUSH，按键 + 串口命令切换）**
→ **IMU（ICM-42670-P，硬件 I2C1）读到真实加速度/角速度** → **AHT20 读到真实温湿度** → **20 Hz 32 字节上行遥测帧**
→ **CH554G 手柄链路（USART3 收 15 字节 HID，RC 模式摇杆控速，20 Hz 回发 0x79 帧，另带 0x78 调试注入帧）**。

1. **CH554G 侧固件**：USB 主机读接收器 15 字节报告 → 串口 115200 原样转发（见 §1.3 的三条约定）
2. **CAN**：PA11/PA12 + TJA1050，1 Mbps 标准帧，从旧固件搬达妙电机协议（速度帧 + 反馈解析）
3. **运动控制**：速度指令已经在收（`Proto_GetHostCmd()` 拿 vx/wz），接上电机后按模式分发：
   HOST 用上位机命令、RC 用遥控输入、PUSH 电机释放（达妙失能不制动，语义天然吻合）
4. **IMU 闭环/姿态**：现在只给原始数据，后面要做倾角/角速度零偏标定再谈闭环
5. **回退到旧协议**：旧固件的 24 B 遥测帧 / 44 B 诊断帧可以并行发，car_web 不用改

旧固件可参考：`firmware/stm32-chassis/user/main.c` 的 CAN 收发与达妙协议；
`firmware/stm32-chassis/hardware/YbImu.c` 是老的 YbImu（不是 ICM-42670-P），只能参考写法。
