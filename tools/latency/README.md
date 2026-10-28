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

只做软件侧测量，**不改固件**。因为所有上报帧都是 20 Hz，单次分辨率 50 ms；
靠多次试验 + 上升沿外推把分辨率压到 ~±3 ms。

## 文件

| 文件 | 作用 |
|---|---|
| `probe_latency.py` | 主机上跑：发阶跃 + 收 `/odom`，记录所有时刻 → `/tmp/latency.json` |
| `run_latency.sh` | 主机上一键：停 car_web → 起厂商节点（带 strace）→ 跑 probe → 收尾 |
| `analyze_latency.py` | 本机跑：把 json + strace 日志分析成分段延迟 |
| `selftest_analyze.py` | 用已知真值（T1~U(0,50ms)、T2=30ms）的假数据验证分析脚本 |

## 怎么跑

```bash
# 1) 上传（本机）
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/probe_latency.py /tmp/probe_latency.py
python tools/rsh.py 192.168.5.11 123456 --put tools/latency/run_latency.sh    /tmp/run_latency.sh

# 2) 方向自检（车只动约 5 cm）—— 先确认方向没反
python tools/rsh.py 192.168.5.11 123456 'CHECK=1 bash /tmp/run_latency.sh'

# 3) 正式测量（20 个 trial，车原地小幅前后摆，净位移≈0）
python tools/rsh.py 192.168.5.11 123456 'bash /tmp/run_latency.sh'

# 4) 拉回数据 + 分析（本机）
python tools/rsh.py 192.168.5.11 123456 --get /tmp/latency.json       ./latency.json
python tools/rsh.py 192.168.5.11 123456 --get /tmp/strace_node.log    ./strace_node.log
python tools/latency/analyze_latency.py ./latency.json ./strace_node.log
```

前提：车**落地**、前后留 1.5 m 净空、没有人站在车前后。
`car_web` 会占串口，脚本会自动 `pkill -f car_web.py`；跑完不自动恢复 `car_web`。

## 延迟预算（预期值）

| 段 | 量级 | 说明 |
|---|---|---|
| DDS 投递 | < 1 ms | 同一主机内 |
| **ROS 回调调度** | **0~50 ms（均值 25）** | 厂商节点 `Control()` 是阻塞大循环：`Get_Sensor_Data_New()` 阻塞等 24 B 遥测帧（20 Hz）→ `spin_some()` 每轮只跑一次，`Cmd_Vel_Callback` 只能等下一轮 |
| 串口写 11 B@115200 | ≈ 0.96 ms | 加 CH340 USB 缓冲 |
| STM32 控制周期 | 0~20 ms（均值 10） | `CTRL_PERIOD_MS = 20` |
| CAN 4 B@1 Mbps | ≈ 0.11 ms | |
| 达妙速度环 + 机械 | **未知** | 本次要测的另一半 |

**关键结论（源码已确认，待实测复核）**：厂商节点的大循环结构让 `/cmd_vel` 平均多等 25 ms、
最坏 50 ms。这是纯软件问题，改多线程 executor 或单独线程收 `cmd_vel` 就能省掉。

## 三段量怎么读

- **T1 = 发布 → 节点把 11 B 帧 write 到串口**：strace 抓的墙钟，µs 级精确。
  若呈 **0~50 ms 均匀分布** → 直接证实上面的 `spin_some` 20 Hz 假设。
- **T2 = 串口写出 → `/odom` 首次报轮子在转**：固件 20 ms 周期 + CAN + 驱动器 + 机械，
  外加 odom 的 50 ms 量化。
  - `回码前`/`回码后` 是量化区间的下界/上界，含"轮子要转过检测门限才报动"的偏置，只能当区间证据。
  - **`上升沿外推` 才是主估计**：假设 50 ms 内速度线性上升，由首两帧的值求过零点。
    自测（真值 30 ms）得到 28.8 ms，偏差 ~1 ms。
- **T_total = T1 中位 + T2 估计**。

## 自测

```bash
python tools/latency/selftest_analyze.py
```

真值 `T1 ~ U(0,50 ms)`、`T2 = 30 ms`、odom 20 Hz（网格相位独立于发布），
期望输出：T1 mean≈25 ms 且 100% 落在 0~50 ms；T2 估计 ≈28~31 ms；平台速度检查 OK。
（2026-10-28 实测自测：T1 mean 25.9 ms / T2 28.8 ms / 平台 0.201 vs 0.200 ✅）

## 已知限制

- 20 Hz 上报决定了单次分辨率 50 ms，`T2` 靠多点统计压到 ±3 ms，不是单次精度。
- `strace` 会带来每 syscall 十几 µs 的开销，对本测量可忽略。
- 若固件方向系数还没烧进板子（`aeb9fd` 之后的方向修复），前进命令可能变成原地打转；
  `probe` 用 `vwheel = |vx| + |wz|*TRACK/2` 兼容这种情况，先用 `CHECK=1` 看自检判读。
