# 在 ROS 侧给底盘加"心跳"（**已废弃，仅存档**）

> ## ⚠️ 这份文档描述的机制已被删除
>
> 2026-11-07 用户要求 **"把心跳删了吧"**：固件里的 `f[1]=0x08` 心跳帧、
> 300 ms 链路看门狗、**速度目标锁存** 全部移除；`tools/car_web` 不再发心跳帧
> （1.5.1），厂商节点补丁也已回退。
>
> **原因**：心跳模式会"锁存最后一条命令"，而 `teleop_twist_keyboard`
> **松手时不发 0**（只在收到字符时 publish，Ctrl-C 直接 break），
> 于是键盘一松手车就一直按上一帧走 —— 与"按住才走、松手慢慢停"的预期冲突。
>
> **现在的语义**（见根 `README.md` §3.1.1）：只有一条 800 ms 命令看门狗，
> 超时即按 `RAMP_TIMEOUT_DEC = 0.70 m/s²` 软停；唯一瞬停入口是电机反馈掉线。
>
> 本文档保留，只是为了让以后想再引入"链路存活"语义的人知道踩过什么坑
> （尤其是 §7「七个坑」里的"锁存必须和'我还活着'绑定"那条）。
> 补丁文件 `vendor-patches/vendor-node-heartbeat.patch` 已删除，需要时可从
> git 历史 `ac2309c` 取回；下文提到的 `tools/latch_test.py` 也已改名为
> `tools/cmd_timeout_test.py`，判据从"锁存"反转为"800 ms 后自动软停"。

---

> 适用固件：**v3 及以上**（`STATUS_VERSION = 3`，诊断帧 `[30] = 3`）。
> 固件老于 v3 的也能发，不会报错，只是会被当成普通命令帧忽略，行为退回"800 ms 命令看门狗"。

## 1. 为什么要有心跳

ST 老固件的看门狗只有一个判据：**"多久没收到命令帧"**。它分不清下面两件事：

- 上位机还想动，只是没有新命令（例如键盘一次按键只发一帧、网页松手后上层不再发）；
- 上位机已经死了（进程崩了 / 串口线掉了 / 主机重启了）。

前者是"没有话要说"，后者是"人不在了"，但两者在串口上长得一模一样。把两者当同一个处理，就只能选一个折中：短超时（安全，但"停一下又走"很烦）或长超时（跟手，但真掉线要等很久）。

心跳把这两件事拆开：

| 信号 | 含义 | 超时后固件做什么 |
|---|---|---|
| **速度命令帧**（`f[1] = 0x00`） | "我要往哪走、多快" | 命令超时（非心跳模式 800 ms）→ 软停 |
| **心跳帧**（`f[1] = 0x08`） | "我这个进程还活着" | 心跳超时（心跳模式 300 ms）→ **立即急停** |

## 2. 固件侧行为（v3 定稿）

| | 非心跳模式（老行为） | 心跳模式（发过心跳后） |
|---|---|---|
| 怎么进入 | 默认 | 收到**第一条**心跳帧后自动进入 |
| 怎么退出 | — | 心跳源消失 **600 ms**（`HB_STALE_MS`）后自动退回 |
| 看门狗 | 800 ms 没命令帧 → 软停（`RAMP_TIMEOUT_DEC` 0.70 m/s²） | 300 ms 没心跳帧 → **瞬停**（清斜坡） |
| 速度目标 | 命令帧到期即清零 | **锁存**：一直执行最后一条命令 |
| 心跳断时 | — | 立即急停 **且丢弃锁存命令**（恢复后必须重新下发，不会自己又冲出去） |
| 电机反馈掉线 | 立即急停 | 立即急停 |

三条重要推论：

1. **心跳模式是"按最后指令一直走"的语义**。发心跳的一方必须保证"我还在跑"和"我还想让车按这条命令跑"是同一件事。适合"按住走"的遥控、手机网页；不适合"发完就走"的脚本。
2. **心跳是唯一的瞬停入口**。正常发 0 是缓停（走斜坡）；只有心跳断和电机掉线会瞬停。
3. **不认识 0x08 的老固件不会崩**：帧头/帧尾/BCC 都合法，老固件只是忽略 `f[1]` 的这一位，行为完全不变。

## 3. 心跳帧格式

11 字节，和速度命令帧**完全同构**，只是 `f[1]` 用 `0x08`，载荷全 0：

| 字节 | 值 | 说明 |
|---|---|---|
| 0 | `0x7B` | 帧头 |
| 1 | `0x08` | `FUNC_HEARTBEAT` |
| 2–8 | `0x00` | 载荷全 0（心跳不带任何信息） |
| 9 | `0x73` | BCC = `XOR(f[0..8])` = `0x7B ^ 0x08` |
| 10 | `0x7D` | 帧尾 |

即整帧 `7B 08 00 00 00 00 00 00 00 73 7D`。

- 校验算法与其它帧一致：`BCC = XOR(tx[0..8])`（厂商代码里是 `Check_Sum(9, SEND_DATA_CHECK)`）。
- 带宽：11 B × 20 Hz = 220 B/s，115200 下占 **≈2%**，可以和 30 Hz 控制帧同时跑。
- **频率建议 5–20 Hz**：不用更快（300 ms 的超时给 5 次容错已经足够），也别更慢（<3 Hz 就接近超时边界了）。

## 4. 在厂商 `wheeltec_robot_node` 上加（实测可用）

### 4.1 位置：只能加在 `Control()` 的循环体里

厂商节点的架构很特殊（见 `docs/ros2-to-motion-chain.md`）：

```cpp
void turn_on_robot::Control()
{
  while (rclcpp::ok())
  {
    _Now = rclcpp::Time(...);
    if (Get_Sensor_Data_New())            // 阻塞读串口，直到凑齐一帧 24 B 遥测
    {
      ... 里程计积分 / 姿态解算 ...
      Publish_Odom(); Publish_ImuSensor(); Publish_Voltage();
    }
    rclcpp::spin_some(this->get_node_base_interface());   // ← 心跳插在这行之前
  }
}
```

- **不能只靠 `rclcpp::create_wall_timer()`**：这个节点没有 `spin()` 在转，`spin_some()` 只在"收完一帧遥测之后"被调用一次，timer 的实际触发时刻完全被串口节奏绑住，还可能在 `Get_Sensor_Data_New()` 阻塞时整体停摆。直接在循环里发最可预测。
- 但**必须限速**：这个 `Control()` 循环不是 20 Hz，而是"每收到一个串口字节就转一圈"，每秒上千圈。不限速会把 115200 串口塞满，还会挤掉真正的控制帧。

### 4.2 代码

`include` 区加：

```cpp
#include <chrono>
```

`Control()` 里、`rclcpp::spin_some(...)` **之前**插入：

```cpp
    //=========================================================================
    // 心跳帧 (20 Hz)：让 STM32 进入"心跳模式"
    //   帧: [0x7B][0x08][0×7][BCC][0x7D]，只续命，不改目标
    //=========================================================================
    {
      static std::chrono::steady_clock::time_point _Last_Heartbeat =
          std::chrono::steady_clock::now();
      static bool _Heartbeat_Logged = false;
      auto _now_hb = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(
              _now_hb - _Last_Heartbeat).count() >= 50)   // 20 Hz
      {
        _Last_Heartbeat = _now_hb;
        if (!_Heartbeat_Logged)
        {
          _Heartbeat_Logged = true;
          RCLCPP_INFO(this->get_logger(),
                      "heartbeat 开始发送: 20 Hz, f[1]=0x08 -> STM32 进入心跳模式");
        }
        Send_Data.tx[0]  = FRAME_HEADER;                  // 0x7B
        Send_Data.tx[1]  = 0x08;                          // FUNC_HEARTBEAT
        Send_Data.tx[2]  = 0; Send_Data.tx[3] = 0;
        Send_Data.tx[4]  = 0; Send_Data.tx[5] = 0;
        Send_Data.tx[6]  = 0; Send_Data.tx[7] = 0;
        Send_Data.tx[8]  = 0;
        Send_Data.tx[9]  = Check_Sum(9, SEND_DATA_CHECK); // BCC
        Send_Data.tx[10] = FRAME_TAIL;                    // 0x7D
        try
        {
          Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
        }
        catch (serial::IOException& e)
        {
          RCLCPP_ERROR(this->get_logger(), "heartbeat send failed");
        }
      }
    }
```

现成的补丁：`vendor-patches/vendor-node-heartbeat.patch`（基于 `src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`）。

### 4.3 编译与部署

```bash
cd ~/wheeltec
cp src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp ~/wheeltec_robot.cpp.before_hb   # 备份（这个 workspace 不是 git 仓库！）
# 覆盖源码或打补丁后：
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_wheeltec_robot
source install/setup.bash
export ROS_DOMAIN_ID=15
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

启动日志里应出现一次：

```
[wheeltec_robot]: heartbeat 开始发送: 20 Hz, f[1]=0x08 -> STM32 进入心跳模式
```

## 5. 自己写的 rclpy 节点怎么加

有正常 `spin()` 的节点用 timer 即可（比厂商节点干净得多），示例见 `tools/car_web/car_web.py` 的 `build_heartbeat_frame()`（串口线程里 5 Hz 发，`HB_PERIOD_S = 0.2`）：

```python
HB_PERIOD_S = 0.2   # 5 Hz

def build_heartbeat_frame() -> bytes:
    f = bytearray(11)
    f[0] = 0x7B          # 帧头
    f[1] = 0x08          # FUNC_HEARTBEAT
    # f[2..8] 全 0
    f[9] = xor_bcc(bytes(f[0:9]))
    f[10] = 0x7D         # 帧尾
    return bytes(f)

# 在 rclpy 节点里
self.create_timer(HB_PERIOD_S, lambda: self.serial.write(build_heartbeat_frame()))
```

**串口独占**提醒：同一个 `/dev/ttyUSBx` 同一时刻只能被一个进程打开，所以心跳只能由"那个正在写串口的进程"发，不能另起一个节点去发。

## 6. 怎么验证心跳真的生效

三个层次，从便宜到彻底：

1. **节点日志**：出现 `heartbeat 开始发送` 即代码路径跑通（只证明发了，不证明对方收到）。
2. **看诊断帧 `[41]` 的 bit7**：心跳模式下固件把 `[41] |= 0x80`（低 3 位仍是启动阶段）。用 `tools/ramp_test.py` / `tools/imu_probe.py` 直连串口读 44 B 诊断帧即可看到 `f41=129`（`0x81`）之类的值。
3. **行为验证**：`tools/latch_test.py`（`python3 latch_test.py 0.06`）—— 只往 `/cmd_vel` 发**一条** 0.06 m/s，然后观察 `/odom`。心跳模式下车会一直走到你发 0 为止；非心跳模式下 800 ms 就归零。

## 7. 常见坑

1. **不限速直接塞进 `Control()`**：控制环每秒上千圈 → 串口被心跳淹没，`/odom` 掉帧、命令延迟飙升。必须用 `steady_clock` 限到 5–20 Hz。
2. **在 `Get_Sensor_Data_New()` 阻塞期间心跳停了**：`Control()` 结构决定心跳最坏会有"读一帧遥测的时间"抖动（~50 ms 量级），所以 `HB_TIMEOUT_MS` 不能设得比这还小（300 ms 有 6 倍余量，安全）。
3. **发了心跳却还在用非心跳的短超时期望**：一旦进心跳模式，命令就被**锁存**——"发一帧就走"的脚本会让车一直走。要么别发心跳，要么发完命令后必须显式发 0。
4. **心跳和速度命令用了不同进程**：串口独占，第二个进程打不开端口。
5. **忘了 `#include <chrono>`**：`std::chrono::steady_clock` 编译不过。
6. **`Send_Data.tx` 和命令帧共用缓冲区**：心跳写完 `tx` 就被下次命令覆盖是正常的（每次发送前都重新组帧），但如果你把心跳放在"组命令帧之后、发送之前"，会破坏命令帧。放在 `spin_some()` 之前是安全的（此时命令帧早就发出去了）。
7. **改完没备份**：`~/wheeltec` 的 workspace **不是 git 仓库**，覆盖前先 `cp` 到 `~/wheeltec_robot.cpp.before_hb`。

## 8. 回退

- 源码回退：`cp ~/wheeltec_robot.cpp.before_hb ~/wheeltec/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp` 后重新 `colcon build`。
- 反向打补丁：`patch -R -p1 < vendor-patches/vendor-node-heartbeat.patch`。
- 只停心跳、不改固件：把上面代码块里的 `>= 50` 改成 `>= 100000`（等效关闭），或直接注释掉整块。
- 固件侧也可以单方面退回：`main.c` 里 `#define HB_ENABLE 0`（编译后心跳帧会被当作普通帧忽略，行为等同 v2）。
