# 固件稳定版 fw-v8.0（2026-11-10 归档）

> ★ **这是第一版"可以交付给同事"的稳定固件** —— 2026-11-10 用户确认：
> "好，这是一版稳定的固件，保存一下"。
>
> 对应的 git 提交：**`f808ff1`**（分支 `damiao-bridge`，固件源码提交），标签 **`fw-v8.0`**、
> 分支 **`stable-v8`**（两者都指向"归档提交"，比 `f808ff1` 只多出本目录的归档文件，
> 源码与 hex 完全一致）。上位机 `car_web` 配对版本 **1.6.2**。

## 一、这两个 hex 是什么

| 文件 | 车型 | 大小 | hex MD5 | 镜像 MD5（`0x08000000` 起） |
|---|---|---|---|---|
| `scrubber.hex` | **洗地车**（`VEHICLE_SEL=2`，轮距 456 mm，双轮 `DM_DIR=-1`，**有** PB0/PB1 辅助按钮） | 35900 B | `7BEC84E036E49B7306E68FC22B2B10EB` | `A95A8FD12FDD5E17DE8D1E36C10D6733`（12744 B） |
| `sweeper.hex` | **扫地车**（`VEHICLE_SEL=1`，轮距 716 mm，左 `+1` / 右 `−1`，辅助按钮空实现） | 34767 B | `039FCA5F46FF3CD19DB7771FA850B3F3` | `BFD29D3C3B03C26AA56DBD0A02BB540C`（12340 B） |

- 镜像 MD5 是**烧进去的字节**的 MD5（`IntelHex(...).tobinarray(start=0x08000000)`），
  烧录后用 `dump_image` 回读比对就是这个值。
- 归档 hex **逐字节原样保存**：`.gitattributes` 里专门钉了
  `firmware/releases/**/*.hex binary`。★ 这里踩过坑 —— gitattributes **后面的规则覆盖前面的**，
  原来 `* text=auto` 写在 `*.hex binary` 后面，导致 hex 被当文本、提交时 CRLF→LF，
  归档出来的 hex 比原文件少 800 字节、MD5 全变（2026-11-10 已修）。
- 编译：Keil MDK ARMCC V5.06u1，两 target 均 **0 Error / 0 Warning**；
  Code+RO：洗地车 12620 B / 扫地车 12228 B（ROM 12744 / 12340 B）。
- **洗地车这一份已经实烧验证**（`** Verified OK **` + 回读 MD5 一致 + 辅助按钮实车按过）；
  **扫地车这一份是同一提交编出来的、未上过车**（它没有辅助按钮那级硬件，
  差异只在 `VEHICLE_SEL` 分支，但首次上车请按第五节的现象表验一遍）。

## 二、这一版有什么（相对上一版稳定点 `aeb9efd`）

| 功能 | 说明 |
|---|---|
| **命令锁存**（v4） | `CMD_TIMEOUT_ENABLE = 0`：没收到新命令就一直按最后一条跑，**不会自己停**；唯一瞬停入口 = 电机反馈掉线 2 s。风险与回退见 `README.md` §3.1.1 |
| **缓起缓停**（斜坡，固件内） | `RAMP_VX_ACC 1.0` / `RAMP_VX_DEC 0.80`（m/s²）、`RAMP_WZ_ACC 4.0` / `RAMP_WZ_DEC 1.5`（rad/s²）。实测 0→0.20 m/s 用 200 ms、0.20→0 用 250 ms；1.6 m/s 起步 1.6 s / 停 2.0 s |
| **洗地车辅助按钮**（v5~v8） | PB0 = 开关刷盘、PB1 = 开关风机；**独立功能帧 `0x7A`**（8 字节），一次按键默认 200 ms；极性已修正（驱动低边 MOS 栅极，**空闲输出低=松开**）|
| **诊断帧 `[30]` = 8** | 判断板子上是不是这一版。`[41]` bit6 = 1 表示命令锁存模式 |
| 心跳机制 | **已移除**（`f[1]=0x08` 不再有意义）；厂商节点不要发心跳帧 |

详细协议见 `README.md` §3；辅助按钮协议见 §3.1.2。

## 三、怎么烧（Windows + DAPLink）

DAPLink 插在电脑上（`pyocd list` 能看到 `Horco CMSIS-DAP`），用 openocd：

```powershell
D:\program\xpack-openocd-0.12.0-7\bin\openocd.exe `
  -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg `
  -c "adapter speed 500" `
  -c "program {D:/github/TB01-CAN-USART/firmware/releases/fw-v8.0-20261110/scrubber.hex} verify" `
  -c "init" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown"
```

三个坑（都踩过）：

1. **`adapter speed` 必须是 500**，1000 会偶发 `Error: CMSIS-DAP command CMD_INFO failed`。
2. hex 路径**必须用正斜杠**，反斜杠会被 openocd 吃掉。
3. 板子 **NRST 没接**，所以不能用 `program ... verify reset exit`（会报 `Unable to reset target`）；
   先 `program ... verify`，再用 `mww 0xE000ED0C 0x05FA0004`（写 AIRCR 的 SYSRESETREQ）软复位。

**烧完必须独立回读校验**（不要相信 `Verified OK` 一个标志）：

```powershell
openocd ... -c "adapter speed 500" -c "init" -c "halt" `
  -c "dump_image C:/Temp/readback.bin 0x08000000 12744" -c "shutdown"
# 洗地车回读 12744 B、扫地车 12340 B，MD5 应等于上表的"镜像 MD5"
```

## 四、怎么确认板子上跑的是这一版

看 44 字节诊断帧（`0x7E` 开头）的第 `[30]` 字节：

- `[30] = 8` → 这一版；
- `[41]` bit6 = 1 → 命令锁存模式（不会有"松手自动停"）；
- `[39]` bit3/4/5 → 辅助按钮正在按 / 最近按过刷盘 / 最近按过风机。

用网页（`car_web`）最省事：`/api/state` 里 `status.version` 就是 `[30]`，
`status.flags_raw` 的第 6 位就是锁存位。

## 五、上车 30 秒自检清单

1. 车轮**架空**（第一次验证务必架空）。
2. 上电，看 LED 心跳；网页/串口能收到 44 B 诊断帧、20 Hz。
3. `[30]=8`、两轮反馈 `err` = 1（使能）或 0（失能），无故障码。
4. 发一个很小的速度（如 0.05 m/s），确认两轮**同向**、实际 ω 跟得上
   （洗地车两轮都应往一个方向转，扫地车左右轮是反的机械安装，看车体方向）。
5. 发 0 → 车**软停**（不是瞬停），此时再发 0.05 → 又能走。
6. 洗地车额外：按一下 PB0/PB1，确认对方刷盘/风机真的切换（用万用表量对方按钮脚：
   空闲 3.3 V、按下瞬间 0 V 约 200 ms）。
