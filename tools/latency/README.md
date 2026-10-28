# 链路延迟测量：`/cmd_vel` 发布 → 轮子实际转动

测的是**上位机（RK3576）发布 `/cmd_vel`** 到 **轮子真的开始转** 之间的延迟，走真实链路：

```
probe_latency.py ──/cmd_vel──> wheeltec_robot_node ──串口 11 B 帧──> STM32F103
                                                                      │ CAN 1 Mbps
                                                                      ▼
                                                          达妙 DM-H65 速度环
                                                                      │
         /odom <── wheeltec_robot_node <──串口 24 B 遥测 @20 Hz <──────┘
```

只做软件侧测量，**不改固件**（用户要求）。结论（2026-10-28 实测）：

| 分段 | 数值 | 说明 |
|---|---|---|
| **T1 发布 → 串口写出** | **中位 21.6 ms**（均值 19.9，0~43 ms，100% 落在 0~50 ms） | µs 级精确。被厂商节点的 **20 Hz 大循环**量化，见下 |
| **T2 串口写出 → 轮子开始转** | **6 ~ 46 ms**，取 ~26 ms | 只能给区间（原因见"关键结构发现"），物理预算 12~32 ms |
| **T_total 发布 → 轮子开始转** | **≈ 47 ms**（保守 28 ~ 68 ms） | T1 + T2 |
| **T_stop 发 0 → 停住** | ≈ 190 ms | 减速是斜坡，不是立即停 |

## 文件

| 文件 | 作用 |
|---|---|
| `probe_latency.py` | **主机上跑**：发阶跃 + 收 `/odom`，记录所有时刻 → `/tmp/latency.json` |
| `run_latency.sh` | **主机上跑**：一键 停 car_web → 起厂商节点（挂 `writets.so`）→ 跑 probe → 收尾 |
| `writets.c` | LD_PRELOAD 钩子：抓节点每次 `write(11 B 命令帧)` 的 µs 时刻 + **帧内容** |
| `analyze_latency.py` | **本机跑**：配对发布↔串口写，出 T1 直方图 / T2 区间 / 逐 trial 表 |
| `analyze_profile.py` | **本机跑**：合并重建响应曲线 v(τ)，出 T2 区间、过冲、减速曲线 |
| `selftest_analyze.py` | 用**已知真值**的假数据 + 假 LED 帧验证分析脚本（`PASSED` 才可信） |
| `teleop_key_probe.py` | **主机上跑**：pty 里起 `teleop_twist_keyboard`，脚本"按键"并记录按键时刻 + `/cmd_vel`/`/odom` → `/tmp/key_probe.json` |
| `analyze_teleop.py` | **本机跑**：把键盘全链路切成 T0(按键→cmd_vel)/T1(cmd_vel→串口写)/T2(写→动)/T_total |
| `check_topics.py` | **主机上跑**：快速体检 `/odom`、`/cmd_vel` 帧率与 `header.stamp` 偏差 |
| `results/` | 每次实测的原始数据与分析输出（保留，便于复查） |

### 键盘 teleop 全链路（换一种测法）

不直接发 `/cmd_vel`，而是真的按键：`teleop_twist_keyboard` → `/cmd_vel` → 厂商节点 → 串口。
两点前提：humble 版 teleop **每按一次键只发一条 Twist**；`getKey()` 用
`tty.setraw()`+`termios.tcgetattr()`，**stdin 必须是 pty**（管道会抛异常），所以探针用 `pty.openpty()`。

```bash
# 主机上：先把底盘 launch 起来（钩子挂在 launch 上，节点会继承 LD_PRELOAD）
export LD_PRELOAD=/tmp/writets.so WRITETS_LOG=/tmp/writets_key.log
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
# 主机上：跑探针（它会自己起 teleop，不用手敲）
export ROS_DOMAIN_ID=15
python3 /tmp/teleop_key_probe.py --trials 20 --speed 0.20 --out /tmp/key_probe.json
# 本机：拉回 + 分析
python tools/rsh.py 192.168.5.7 12345678 --get /tmp/key_probe.json    tools/latency/results/key_probe.json
python tools/rsh.py 192.168.5.7 12345678 --get /tmp/writets_key.log   tools/latency/results/writets_key.log
python tools/latency/analyze_teleop.py --json tools/latency/results/key_probe.json --hook tools/latency/results/writets_key.log
```

注意 `--speed` 是传给 teleop 的参数（`-p speed:=`），`i`/`,` 是前进/后退键。
「写串口时车还没停稳」的 trial 会被剔除（逐 trial 表里标 `*`）。

## 怎么跑

```bash
# 1) 上传（本机）
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/probe_latency.py /tmp/probe_latency.py
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/run_latency.sh    /tmp/run_latency.sh
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/writets.c         /tmp/writets.c
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/analyze_latency.py /tmp/analyze_latency.py

# 2) 方向自检（车只动约 5 cm）—— 先确认方向没反
python tools/rsh.py 192.168.5.11 123456 'CHECK=1 bash /tmp/run_latency.sh'

# 3) 正式测量（20 个 trial，车原地小幅前后摆，净位移≈0，全程约 45 s）
python tools/rsh.py 192.168.5.11 123456 'TRIALS=20 SPEED=0.20 bash /tmp/run_latency.sh'

# 4) 拉回数据（本机）
python tools/rsh.py 192.168.5.11 123456 --get /tmp/latency.json tools/latency/results/latency_<日期>.json
python tools/rsh.py 192.168.5.11 123456 --get /tmp/writets.log  tools/latency/results/writets_<日期>.log

# 5) 分析（本机，两个脚本都要跑）
python tools/latency/analyze_latency.py tools/latency/results/latency_<日期>.json tools/latency/results/writets_<日期>.log
python tools/latency/analyze_profile.py tools/latency/results/latency_<日期>.json tools/latency/results/writets_<日期>.log
```

前提：车**落地**、前后留 1.5 m 净空、没有人站在车前后。
`car_web` 会占串口，脚本会自动 `pkill -f car_web.py`；跑完**不自动恢复** `car_web`。

主机上没有 `strace`（也不想装），所以用 `LD_PRELOAD` 钩 `write()`：
比 ptrace 开销小、无需 root、时间戳由进程自己打，还顺便记下帧内容。

## 关键结构发现（决定 T2 只能给区间）

厂商节点 `Control()` 是一个**阻塞大循环**，每轮：

```
Get_Sensor_Data_New()  ← 阻塞等一帧 24 B 遥测（20 Hz）
Publish_Odom()         ← 发 /odom
Publish_ImuSensor() / Publish_Voltage()
spin_some()            ← 这里才处理 /cmd_vel 回调 → 写 11 B 到串口
```

由此产生**两个直接后果**，都已被实测证实：

1. **T1 被量化到 0~50 ms**：回调每轮最多被服务一次，均值多等 ~25 ms。
   实测 T1 直方图 100% 落在 0~50 ms（64/64），跨 trial 还呈**锯齿**（每 trial 相位漂移
   ~4.5 ms 后复位）—— 这是"回调只能在 20 Hz 网格上被服务"的硬证据。
   **这是纯软件延迟，改多线程 executor / 单独线程收 `cmd_vel` 就能省掉均值 ~20 ms。**
2. **写串口永远紧跟在一次 `/odom` 发布之后**（实测 τ ≈ 0~6 ms 就有一帧，然后下一帧在
   τ ≈ 41~61 ms）。也就是说**观测网格和命令网格是锁在一起的**，τ ∈ (6, 40) ms
   这个窗口**根本没有样本** → T2 的 50 ms 量化**无法靠多点统计消除**。
   （早先以为能靠"上升沿两帧外推"压到 ±3 ms，实测数据证明那是错的：真实响应是
   一条持续几百 ms 的斜坡，两帧外推会给出 −41 ms 这种荒谬值。自测里能对，
   是因为假数据设的上升时间只有 120 ms，不具代表性。）

所以 **T2 只报区间**：下界 = 最后一帧"明确没动"（τ≈6 ms），上界 = 第一帧"明确在动"（τ≈46 ms）。
物理预算（USB ~2 ms + STM32 20 ms 周期 0~20 ms + 驱动/机械 ~10 ms ≈ 12~32 ms）落在区间内。

## 三段量怎么读

- **T1**（`analyze_latency.py`）：配对"发布时刻 ↔ 11 B write 时刻"。配对**按帧内容**
  （`[3..4]`=vx×1000 大端、`[7..8]`=wz×1000）而不是"窗口内最早一帧"——因为厂商源码里
  `Stm32_Serial.write(Send_Data.tx, 11)` 有 **9 处调用点**，其中 L98-109 的 LED 帧
  同样以 `0x7B` 开头、同样 11 字节。不按内容筛就会配错（实测多出 1 帧）。
- **T2**（`analyze_profile.py`）：把 20 个 trial 的 (τ, v) 合并分箱重建 v(τ)，
  给出"最后一帧没动 / 第一帧动了"的硬区间，并顺带看过冲与减速。
- **T_total** = T1 中位 + T2 区间中点。

## 自测

```bash
python tools/latency/selftest_analyze.py
```

造一份已知真值的假数据：`T1 ~ U(0, 50 ms)`、`T2 = 30 ms`、odom 20 Hz 且网格相位独立，
并**故意在每条真命令帧前 2 ms 插一条同长度同帧头的 LED 假帧**（解出来是 vx=5.0 m/s）。

硬判据（任一不满足即 `FAILED`）：
- 配对统计必须是"按内容 64, 按时间兜底 0, 没配上 0"—— 内容配对真的生效
- T1 必须 100% 落在 0~50 ms —— 假帧没有顶掉真帧
- T2 与真值 30 ms 偏差 < 6 ms

## 已知限制

- `/odom` 20 Hz 是硬约束；T2 只能给 50 ms 宽的区间，除非改固件提高上报率（用户不允许）。
- 主机无 `strace`，用 `LD_PRELOAD` 钩子；钩子本身开销约几 µs，可忽略。
- 若固件方向系数还没烧进板子，前进命令可能变成原地打转；`probe` 用
  `vwheel = |vx| + |wz|*TRACK/2` 兼容这种情况，先用 `CHECK=1` 看自检判读
  （`forward` / `spin` / `backward`）。
- 落地测量会让车真动（单程约 14 cm/trial，方向交替净位移≈0），必须人在旁边。
