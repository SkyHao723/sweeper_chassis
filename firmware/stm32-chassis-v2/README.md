# 底盘主控 V2（新板，STM32F103C8T6）

新底盘主控板的第一版固件，只做**点灯验证**：把 OLED 屏点亮、把 RGB 灯点亮、把板载按键读出来，
同时从调试串口打同样的信息。用来确认新板焊接、供电、晶振是否正常。

> 旧底盘（洗地车 / 扫地车）的固件仍在 [`../stm32-chassis/`](../stm32-chassis/)，稳定版归档在
> [`../releases/fw-v8.0-20261110/`](../releases/fw-v8.0-20261110/MANIFEST.md)，**两者互不影响**。

---

## 1. 这一版做了什么

| 外设 | 引脚 | 实现 | 现象 |
|------|------|------|------|
| OLED（SSD1306 128×64） | PB8=SCL, PB9=SDA | **软件 I2C**（开漏，见 §3.1） | 屏上 8 行字 + 每 100 ms 自增的运行计数 |
| RGB 灯（XL-5050） | PB5 | WS2812 单线时序，DWT 周期计数器打拍，**软件调光 50 %** | 上电红色，**按一下按键换下一个颜色**：红→绿→蓝→白→灭→红… |
| 板载按键 SW1 | PA15 | 上拉输入（需先关 JTAG，见 §3.2） | **切换 RGB 颜色**；屏上 `KEY DOWN`/`KEY UP`，串口打印边沿 |
| 调试串口 | PA9=TX, PA10=RX | USART1 115200 8N1 → CH343P → USB | 上电横幅 + 按键/换色日志 |

上电后屏幕应该是：

```
TB01 CHASSIS V2
MCU F103C8T6
CLK 72 MHZ
OLED OK
LED PB5 WS2812
KEY UP
COLOR RED
RUN 000123
```

串口（115200）同时打印：

```
======================================
 TB01 chassis controller V2 bring-up
======================================
SYSCLK = 72 MHz
OLED   = OK (SSD1306 @0x3C, soft I2C PB8/PB9)
LED    = WS2812 on PB5, brightness 50 %
KEY    = PA15 (pull-up, active low), press to change color
```

按一下 PA15，屏幕第 6 行会变成 `KEY DOWN`、第 7 行 `COLOR GREEN`（再红→蓝→白→灭），
松开变回 `KEY UP`；串口对应打印 `[key] SW1 pressed` / `[key] SW1 released` / `[led] color = GREEN`。

**亮度**：`hardware/ws2812.h` 里的 `WS2812_BRIGHTNESS_PERCENT`（默认 **50**，0~100），
直接等比缩小 RGB 数值。觉得还亮就改 30、觉得暗就改 70，改完重新编译烧录即可。

**看这两行就知道板子基本是好的**：

- `CLK 72 MHZ` —— 8 MHz 晶振起振了（显示 `CLK 8 MHZ` 说明 HSE 没起来、退回内部 HSI，
  查 X1、18 pF、C7、BOOT0 下拉电阻）。
- `OLED OK` —— 显示这一路 I2C 有应答（`OLED NO ACK` 见 §4 排查）。

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
（hex 14517 B MD5 `0FC8FACD1821F1CFC26DD6D5C2D1C57B`；镜像 5140 B MD5 `DDB4B3A6816F0882D25D798A4B6E7467`；Code 4572 B / RO 524 B / RW 44 B / ZI 1636 B）。

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

- **IMU（ICM-42670-P）** 走 PB6/PB7 的**硬件 I2C1**（以后做）
- **OLED + AHT20** 走 PB8/PB9 的**软件 I2C**（本版已实现，刷屏够用）

总线地址：OLED `0x3C`（写 `0x78`）、AHT20 `0x38`，不冲突。

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
| 灯亮但颜色错 | WS2812 是 **G→R→B** 顺序，不是 RGB；`Ws2812_SetPixel(i, r, g, b)` 内部已换成 G,R,B |
| 灯**太亮/太暗** | 调 `hardware/ws2812.h` 的 `WS2812_BRIGHTNESS_PERCENT`（默认 50，0~100），这是软件调光、等比缩小 RGB 数值 |
| 按键没反应 / 不换色 | 是否调了 `Board_SwdOnly()`；PA15 是否被调试器占用；按下是否真把脚拉到 GND（万用表量）；消抖是 30 ms（3×10 ms 采样），按太短会漏 |

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
│   ├── main.c                 bring-up 主程序（屏幕 8 行 + 按键换色 + 串口）
│   ├── board.c / board.h      板级：DWT 延时、关 JTAG、未用脚驻留、USART1、按键
│   ├── stm32f10x_conf.h       标准外设库裁剪
│   └── stm32f10x_it.c/h       中断向量
├── hardware/
│   ├── ws2812.c / .h          WS2812 单线驱动（DWT 打拍，发送期间关中断）
│   └── ssd1306.c / .h         SSD1306 驱动（软件 I2C + 5x7 字库 + 页寻址）
├── sys/Delay.c / .h           SysTick 微秒/毫秒延时（沿用旧工程）
├── start/                     CMSIS + 启动文件（startup_stm32f10x_md.s）
└── lib/                       ST 标准外设库 3.5.0
```

工程文件由脚本从旧工程生成（`tools/gen_v2_project.py`）：只改 target 名、输出目录、宏定义
和 user/hardware 两个文件组，编译选项与调试器配置沿用旧工程。

当前体积：**Code 4352 B / RO 524 B / RW 44 B / ZI 1636 B**（64 KB Flash，余量充足）。

---

## 6. 下一步（按顺序）

1. **IMU**：ICM-42670-P 上硬件 I2C1（PB6/PB7），先读 WHO_AM_I（`0x75` 应为 `0x67`），再出角速度
2. **AHT20**：挂在现有软件 I2C 上（地址 `0x38`）
3. **CH554G**：USART3（PB10/PB11）通信协议待定（对方 MCU 负责 USB-A 主机口）
4. **CAN**：PA11/PA12 + TJA1050，1 Mbps 标准帧，从旧固件搬达妙电机协议（速度帧 + 反馈解析）
5. **串口协议**：把旧固件的 24 B 遥测帧 / 44 B 诊断帧 / 0x7A 功能帧搬过来（上位机不用改）

旧固件可参考：`firmware/stm32-chassis/user/main.c` 的 CAN 收发与达妙协议；
`firmware/stm32-chassis/hardware/YbImu.c` 是老的 YbImu（不是 ICM-42670-P），只能参考写法。
