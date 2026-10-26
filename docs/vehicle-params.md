# 车型几何参数与 Keil target 选择

> 适用分支：`damiao-bridge`（达妙 DM-H65 桥接固件）
> 相关文件：`firmware/stm32-chassis/user/main.c`（参数定义）、
> `firmware/stm32-chassis/stm32-chassis.uvprojx`（两个 target 的编译宏）

## 1. 两辆车**只有轮距不同**

两辆车用的是**同一款轮毂电机**（达妙 DM-H65 / DM6540-1EC，轮径 171 mm），
控制链路、CAN ID、IMU、协议帧**完全一样**，唯一的差异是**轮距**。

| | 扫地车 `sweeper` | 洗地车 `scrubber` |
|---|---|---|
| 电机 | DM-H65 (DM6540-1EC) | DM-H65 (DM6540-1EC) |
| 轮径 | **171 mm**（半径 0.0855 m） | **171 mm**（半径 0.0855 m） |
| **轮距** | **716 mm** | **456 mm** |
| 轮周长 | 0.537212 m | 0.537212 m |
| `vx`(m/s) → 单轮 ω | `× 11.6959` rad/s | `× 11.6959` rad/s |
| `wz`(rad/s) → 单轮 Δω | `× 4.18713` rad/s | `× 2.66667` rad/s |
| 单轮 ω 上限 | 30 rad/s | 30 rad/s |
| 最大线速度 | 2.565 m/s | 2.565 m/s |
| Keil target | `sweeper` | `scrubber` |
| 编译宏 | `-DTRACK_WIDTH_MM=716` | `-DTRACK_WIDTH_MM=456` |
| 产出 hex | `Objects/sweeper/sweeper.hex` | `Objects/scrubber/scrubber.hex` |

推导：

```
ω_wheel = vx / 0.0855                          （vx 直行）
Δω      = wz * (TRACK/2) / 0.0855              （差速转向）
        = wz * 0.358 / 0.0855 = wz * 4.18713   （716）
        = wz * 0.228 / 0.0855 = wz * 2.66667   （456）
ω_max   = 30 rad/s  →  v_max = 30 * 0.0855 = 2.565 m/s
```

## 2. 参数是怎么传进固件的

`main.c` **不再硬编码轮距**，由 Keil target 的编译宏传入：

```c
#ifndef TRACK_WIDTH_MM
#error "未指定车型轮距: 请在 Keil target 的 C/C++ Define 里加 TRACK_WIDTH_MM (sweeper=716, scrubber=456)"
#endif
#if (TRACK_WIDTH_MM < 200) || (TRACK_WIDTH_MM > 2000)
#error "TRACK_WIDTH_MM 超出合理范围 (200~2000 mm), 检查是否写错单位(要毫米, 不是米)"
#endif
#define TRACK_WIDTH_M       ((float)TRACK_WIDTH_MM / 1000.0f)
```

**为什么宁可"编不过"也不给默认值**：轮距是**安全级**参数，见下节。

**加新车型的做法**（不需要改 `main.c`）：
1. Keil 里 `Project → Manage → Project Items` 复制一个 target；
2. 改 `TargetName`、`OutputDirectory`（必须**独立目录**）、`OutputName`；
3. 在 `C/C++ → Define` 里加 `TRACK_WIDTH_MM=<新轮距毫米数>`；
4. 编一次，确认 hex 与其它车型**不同**。

## 3. ★ 轮距写错是安全级错误

差速逆解是：

```c
v_left  = vx - wz * (TRACK/2);
v_right = vx + wz * (TRACK/2);
```

轮距写错 → **转向增益错**，而且**上报的 `wz` 是用同一个错轮距反算的**
（`wz_mrad = (v_right - v_left) / TRACK`），**自洽**，看数据完全发现不了，
只有车跑起来才知道。

把 `716` 的固件烧到 `456` 的车上（或反之）：

| 现象 | 倍数 |
|---|---|
| 实际转向比期望更猛/更软 | `716/456 = 1.570` 倍（57% 偏差） |
| 上报 `wz` 与真实转角偏差 | `456/716 = 0.637` 倍（低估 36%） |
| 直行 | **不受影响**（轮距只进差速项） |

> 直行不受影响这点很坑：低速直线测试会"看起来正常"，问题只在转弯和导航时暴露。

## 4. ★ 怎么验证烧的是哪个车型（不用信标签，看书据）

**前提：车轮架空**（下面会让轮子转）。

1. 让上位机发一条**纯转向**命令：`vx = 0`，`wz = +0.30 rad/s`；
2. 读 44 字节状态帧的 **目标轮速**（`[10..11]` 左目标、`[12..13]` 右目标，×100 rad/s）：

| 车型 | 左目标 `[10..11]` | 右目标 `[12..13]` |
|---|---|---|
| **`sweeper`（716）** | **−126** | **−126**（两轮**同号**） |
| **`scrubber`（456）** | **+80** | **−80**（两轮**异号**） |

（**两轮同号 vs 异号本身就是车型指纹**：扫地车左轮是镜像安装，方向系数
`DM_DIR_LEFT=+1 / DM_DIR_RIGHT=−1`，所以"左转"命令下两个电机在电机符号空间里
朝同一方向给；洗地车两台电机同向安装，`DM_DIR_LEFT=DM_DIR_RIGHT=−1`，两轮异号。
详见 `main.c` 的方向宏注释。）

## 4.1 ★ 左右方向系数（`DM_DIR_LEFT` / `DM_DIR_RIGHT`）

轮毂电机装在左右两侧是**镜像安装**的，同一个"电机反馈为正"在左轮上对应前进、
在右轮上就对应后退，所以方向系数必须**每个轮子一个**，并且**按车型分**：

| 车型 | `DM_DIR_LEFT` | `DM_DIR_RIGHT` |
|---|---|---|
| `sweeper`（716，扫地车） | **+1.0f** | **−1.0f** |
| `scrubber`（456，洗地车） | **−1.0f** | **−1.0f** |

宏定义由 `VEHICLE_SEL` 选择（`#if VEHICLE_SEL==1 / #elif ==2 / #else #error`），
命令侧和反馈侧乘**同一组**宏（否则"命令往前、上报往后"）。

**症状 → 判定（现场 2 分钟定位）**：车轮架空，发纯前进 `vx=+0.10, wz=0`：

| 现象 | 结论 |
|---|---|
| 两轮同向朝前 | 正确 |
| 两轮对转（车原地打转） | **某一侧取反了**；朝后转的那个轮子就是要取反的那一侧 |
| 按"左转"反而前进 | 同上（两侧系数符号必须相反，实测成了相同） |

> 历史病案（2026-10-26）：扫地车换成 171 轮毂电机后，沿用洗地车的单一全局
> `DM_DIR_SIGN=-1`，结果**按左转 → 车往前走**、**按前进 → 原地打转**。
> 根因不是混控公式（`main.c` 逆解与网页键位都核对过是对的），而是左轮方向系数
> 少了一次取反。**遇到"动作错位"先怀疑方向系数，别去改逆解公式。**

**另一个更省事的判据：比较 hex**

```powershell
# 两个 target 各编一次后，MD5 必须不同
(Get-FileHash firmware\stm32-chassis\Objects\sweeper\sweeper.hex  -Algorithm MD5).Hash
(Get-FileHash firmware\stm32-chassis\Objects\scrubber\scrubber.hex -Algorithm MD5).Hash
```

> 历史教训：改造达妙固件后 `VEHICLE_SEL` 一度不再被代码引用，
> 两个 target 编出来的 hex **MD5 完全相同**（都是 `0F1D1D8E…`），
> 也就是说"选 target 就是选车型"曾经是**假的**。现在由 `TRACK_WIDTH_MM`
> 驱动，已恢复真实差异，并且可以用上面的浮点常量指纹核对。

## 5. 相关但**不属于**车型差异的东西

以下参数两辆车**相同**，改车型时不用动：

| 参数 | 值 | 位置 |
|---|---|---|
| 轮半径 `WHEEL_R_M` | 0.0855 m | `main.c` |
| 单轮 ω 上限 `MAX_OMEGA_RAD_S` | 30 rad/s | `main.c` |
| 电机方向 `DM_DIR_LEFT` / `DM_DIR_RIGHT` | **按车型不同**，见 §4.1 | `main.c` |
| 左轮 CAN ID / MST | 2 / 0x12 | `main.c` |
| 右轮 CAN ID / MST | 1 / 0x11 | `main.c` |
| CAN 速率 / 帧格式 | 1 Mbps / 标准帧 | `main.c` |
| 命令超时 / 反馈超时 | 800 ms / 2000 ms | `main.c` |
| PB9 心跳 | 500 ms 翻转（**不再编码车型**） | `main.c` |

> ⚠️ PB9 心跳灯现在只是"主循环还活着"的指示，**不能**用来区分车型
> （旧 FOC 固件曾用 2 Hz / 5 Hz 编码车型，那套已删除）。

## 6. 上位机侧

- **底盘帧协议与车型无关**：24 字节遥测 + 44 字节诊断帧布局两辆车完全一致，
  换车型**不需要改上位机**。
- `tools/car_web/car_web.py` 里有个 `TRACK_WIDTH_M = 0.456`，
  **只用于 `--sim` 仿真和界面换算**，不参与真机控制（真机的差速在 STM32 里算）。
  用网页遥控扫地车时，仿真里的转弯半径会偏小，真机不受影响。
- 旧厂商 ROS 节点 `turn_on_wheeltec_robot` 只解析 24 字节帧，
  不认识诊断帧里的目标轮速 —— 要查车型请用上面的 §4 方法（读 44 字节帧）。
