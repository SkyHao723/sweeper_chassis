#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "Delay.h"
#include "YbImu.h"
#include <math.h>

/*==========================================================================
 * 两驱差速清扫车 —— STM32 底盘板
 *
 *   USART1  PA9(TX) / PA10(RX)  <->  上位机 CH340 / RK3588   115200 8N-1
 *   CAN1    PA12(TX)/ PA11(RX)  -->  两台 FOC 驱动器          500 kbps 扩展帧
 *   PB10/PB11                   <->  亚博 YbImu (软件 I2C)
 *   PB0 / PB1                   -->  滚刷继电器 / 水泵继电器
 *   PB9                         -->  状态灯 (上电后 2Hz 闪烁, 心跳/主循环活着)
 *
 * 只有一个上位机(RK3588)。**定位不在本板做**: 卡尔曼滤波在上位机跑,
 * 本板只出运动学、CAN 下发、看门狗、继电器和状态上报。PA2/PA3 空着。
 *
 * STM32 负责:
 *   1. 解析上位机发来的车体目标速度 (vx, vy, wz)
 *   2. 差速逆解 -> 左右轮目标转速 -> CAN 下发
 *   3. 轮速正解 + 读 IMU -> 按协议上报
 *   4. 看门狗: 上位机静默 800ms 就自动停车, 并断开继电器
 *
 * ---------------------------------------------------------------------------
 * 协议沿用 Wheeltec 底盘那套 (见 chassis_serial_control.md)
 *
 * 【命令帧 11 字节  上位机 -> STM32】
 *   [0]    0x7B          帧头
 *   [1]    AutoRecharge  正常走车 = 0
 *   [2]    SecurityPLY   正常走车 = 0
 *   [3-4]  int16 BE      vx * 1000   mm/s
 *   [5-6]  int16 BE      vy * 1000   mm/s    (差速车忽略)
 *   [7-8]  int16 BE      wz * 1000   mrad/s
 *   [9]    BCC = XOR([0..8])
 *   [10]   0x7D          帧尾
 *
 * 【主遥测帧 24 字节  STM32 -> 上位机】  (完全按厂商协议标准)
 *   [0]    0x7B
 *   [1]    Flag_Stop
 *   [2-3]  int16 BE 车体 vx   mm/s
 *   [4-5]  int16 BE 车体 vy   mm/s
 *   [6-7]  int16 BE 车体 wz   mrad/s
 *   [8-9]  int16 BE IMU ax 原始值
 *   [10-11]        ay
 *   [12-13]        az
 *   [14-15]        gx
 *   [16-17]        gy
 *   [18-19]        gz
 *   [20-21] uint16 BE 电池电压 mV
 *   [22]   BCC = XOR([0..21])
 *   [23]   0x7D
 *
 * 【扩展诊断帧 36 字节  STM32 -> 上位机】  (本工程自定义, 头 0x7E)
 *   厂商 ROS 节点不认识这一帧。它靠"上一字节是帧尾 0x7D、本字节是 0x7B"
 *   才入帧, 而本帧永远夹在两个完整 24 字节帧之间发出, 所以它会被完整跳过、
 *   并在下一个 0x7B 处正常重新同步 —— 不需要厂商那边做任何改动。
 *   解析脚本: tools/decode_diag.py
 *   [0]     0x7E
 *   [1]     flags  bit0 曾收到命令  bit1 看门狗已停车  bit2 IMU 加速度有效
 *                  bit3 IMU 陀螺仪自开机以来读到过非零
 *                  bit4 上次复位是 IWDG 引起的(说明主循环卡死过, 要查!)
 *                  bit5 驱动器未就绪(有驱动器没回码, 此时只发刹车)
 *                  bit6 起步助推正在加力(轮子还没转起来)
 *                  bit7 助推加到上限还不转(轮子被顶住?), 锁存到目标归零
 *   [2-3]   int16 BE 车体 vx   mm/s     (轮速正解)
 *   [4-5]   int16 BE 车体 wz   mrad/s
 *   [6-7]   int16 BE 左轮 实际转速 RPM
 *   [8-9]   int16 BE 右轮 实际转速 RPM
 *   [10-11] int16 BE 左轮 目标转速 RPM
 *   [12-13] int16 BE 右轮 目标转速 RPM
 *   [14-15] int16 BE 1号驱动器 输出扭矩电流  A×10
 *   [16-17] int16 BE 2号驱动器 输出扭矩电流  A×10
 *   [18]    1号驱动器 故障码   (见 FOC_FAULT_*)
 *   [19]    2号驱动器 故障码
 *   [20]    1号驱动器 当前运行模式  (0x05 速度 / 0x06 位置 ...)
 *   [21]    2号驱动器 当前运行模式
 *   [22]    CAN 发送错误计数
 *   [23]    UART 坏帧计数
 *   [24]    UART 有效命令计数
 *   [25]    UART 溢出计数
 *   [26]    继电器状态  bit0 滚刷 bit1 水泵
 *   [27]    IMU 诊断码  0=正常 1=SCL拉不高 2=SDA拉不高 3=总线死 4=无应答 5=读出错
 *                      6=加速度正常但角速度恒为 0
 *   [28]    IMU 扫描到的 I2C 地址 (0 = 没扫到)
 *   [29]    帧序号(自增), 上位机据此发现丢帧
 *   [30]    IMU 寄存器体检标志位: bit0 版本号 bit1 陀螺仪 bit2 磁力计
 *                               bit3 四元数 bit4 欧拉角  (1 = 该块读到非零数据)
 *           ★ bit5-6 = 车型号 (1=扫地机 2=洗地机), 见"车型选择"
 *           ★ bit7   = 上电自检"两轮都没转" (见 SELFTEST_* 说明)
 *   [31]    IMU 版本号(主版本)
 *   [32-33] IMU 内部融合的偏航角, int16, 单位 0.01 rad
 *   [34]    BCC = XOR([0..33])
 *   [35]    0x7D
 *
 *   [30..33] 是陀螺仪恒 0 时用来定位问题的: 只要 [30] 里除了 bit1 以外还有
 *   别的位置 1, 就说明模块其它功能块是活的, 问题只在陀螺仪这一块;
 *   同时 [32-33] 的偏航角还能当角速度的替代来源(对时间求导)。
 *
 *   为什么把驱动器的电流/故障码单独报出来: 目标转速和实际转速对不上时,
 *   看电流就能分清是"驱动器根本没给力"(电流≈0, 多半是故障或没使能)还是
 *   "给了力但被堵住/拖住"(电流很大) —— 这是判断轮子没劲唯一的客观依据。
 *========================================================================*/

/*========================= 车型选择 ================================
 * 扫地机和洗地机的底盘控制**完全一样**, 只有几何(和可能的接线方向)不同,
 * 所以两辆车共用这一份固件, 靠这一个宏区分。
 *
 * ★★ 选错车型的后果是**安全级**的, 不是"精度差一点":
 *     轮距假设 930 而实车 425 时, 转向增益会错
 *         wz_实际 / wz_命令 = 930 / 425 = **2.19 倍**
 *     (固件按 track 算轮速: ω = wz × (track/2)/周长 × 60。track 假设偏大
 *      -> 同样的 wz 命令给出的轮速差偏大 -> 实车转过头)
 *     反过来假设偏小时车会转不够。
 *     里程计也一样错: 上报的 body_wz = (v_r-v_l)/track, 会变成真值的
 *     425/930 = 0.457 倍 —— EKF 会严重低估转角, 导航直接跑飞。
 *
 * ★ 所以这里不留"忘了改就用默认值"的余地: 选错时下面的 #error 会直接编译失败。
 *   另外烧完上车前**看一眼 PB9 的灯**, 频率就是车型(见 LED_BLINK_MS)。
 *-----------------------------------------------------------------*/
#define VEHICLE_SWEEPER     1       /* 扫地机: 轮径 205mm / 轮距 930mm */
#define VEHICLE_SCRUBBER    2       /* 洗地机: 轮径 150mm / 轮距 425mm */

/* ★ 优先级: **工程 target 里的 -D 定义 > 下面这个默认值**。
   工程里已经建好两个 Keil target(sweeper / scrubber), 各自带
   `-DVEHICLE_SEL=1` / `-DVEHICLE_SEL=2` —— 所以在 Keil 里**选 target 就够了,
   不用改源码**, 而且两个 target 产出的 hex 文件名也不同(sweeper.hex /
   scrubber.hex), 不会编完分不清哪个是哪个。
   下面这个默认值只在"没有 -D"时生效(命令行直接编、或新加 target 忘了带 -D)。 */
#ifndef VEHICLE_SEL
  #define VEHICLE_SEL       VEHICLE_SCRUBBER
#endif

#if VEHICLE_SEL == VEHICLE_SWEEPER
  #define VEHICLE_NAME        "sweeper(扫地机)"
  #define WHEEL_DIAMETER_MM   205.0f
  #define TRACK_WIDTH_MM      930.0f
  #define MOTOR1_IS_LEFT      1     /* 1=1号电机在左侧 */
  #define MOTOR1_INVERT       0
  #define MOTOR2_INVERT       1     /* 实测2号电机正方向是反的 */
  #define LED_BLINK_MS        250   /* 2Hz = 扫地机 */
#elif VEHICLE_SEL == VEHICLE_SCRUBBER
  #define VEHICLE_NAME        "scrubber(洗地机)"
  #define WHEEL_DIAMETER_MM   150.0f
  #define TRACK_WIDTH_MM      425.0f
  /* ⚠ 下面这三个是**接线属性**, 不是几何。现在照抄扫地机的值, 洗地机第一次
     上电必须验证: 发 +vx 两个轮子都往前 = 对; 某个轮子反了就把对应的
     INVERT 改成 1。左右搞反了改 MOTOR1_IS_LEFT。
     (验证方法: tools/decode_diag.py --drive 0.2 0, 看两个"实际"是不是都为正) */
  #define MOTOR1_IS_LEFT      1
  #define MOTOR1_INVERT       0
  #define MOTOR2_INVERT       1
  #define LED_BLINK_MS        100   /* 5Hz = 洗地机(比扫地机快一倍, 一眼能分) */
#else
  #error "VEHICLE_SEL 没选或选错: 必须显式选 VEHICLE_SWEEPER 或 VEHICLE_SCRUBBER"
#endif

/* ★ 怎么确认"这块板烧的是哪辆车的固件" —— 三条, 按方便程度:
 *   1. **看 PB9 的灯**(最方便): 2Hz = 扫地机, 5Hz = 洗地机。
 *   2. **看诊断帧**: `f[30]` 的 bit5-6 就是车型号(1=扫地机 2=洗地机),
 *      `tools/decode_diag.py` 会直接印出来。
 *   3. 看源码里 VEHICLE_SEL 这一行(编译前)。
 *   ✗ 不要用 `#pragma message` 打在编译日志里 —— ARMCC V5.06 不认它, 会报
 *     #161-D warning, 而本工程的要求是两条构建路径都 0 Error 0 Warning。 */

#define MAX_RPM             200     /* 单轮转速上限 */
#define MAX_YAW_RATE        3.0f    /* 车体角速度限幅 rad/s */

/*---------------------- 从上面推出来的量 (别手改) --------------------
 *   轮周长 = π × 轮径
 *   线速度 = 轮速RPM × 周长 / 60
 *   偏航   = (右轮线速 - 左轮线速) / 轮距
 *
 *   两车的对照(洗地机轮子小、轮距窄):
 *                        扫地机        洗地机
 *     轮周长              0.64403 m     0.47124 m
 *     最大线速度(200RPM)   2.147 m/s     1.571 m/s
 *     vx -> 单轮RPM 系数   93.03         127.32   (rpm = vx × 系数)
 *     wz -> 单轮RPM 系数   43.26         27.06    (rpm = wz × 系数)
 *     ★ 起步静摩擦门槛 ~15 RPM 是**驱动器/电机**属性, 换算成车体量就随几何变:
 *         最低直线速度  0.22 m/s  ->  0.12 m/s
 *         最低原地转角  0.35 rad/s -> **0.55 rad/s**(轮距窄了, 同样的轮速
 *                                         只能换来更小的角速度)
 *       所以 Nav2 的 min_speed_theta 要跟着改(见 docs/nav2-reference.md §3)。
 *       但 15 RPM 这个数本身是在扫地机上实测的, **洗地机必须重测**。
 *-----------------------------------------------------------------*/

/*========================= 想改的地方 =========================*/
/* (几何和接线方向已经移到上面的"车型选择"里, 这里不再重复定义) */

/*---------------------- 待机时的电机状态 ----------------------------
 *   没有命令(或收到零速度)时、以及断链看门狗触发时, 一律发 STOP_CTRL。
 *   想换待机手感只改这一个宏:
 *     CTRL_BRAKE (03)   动态制动, 相线短接。转速越低制动力越小 -> 慢慢推很轻
 *     CTRL_DISABLE(02)  完全松开, 轮子自由滑行
 *     CTRL_ENABLE(01)+0 速度环有静区: 不推时误差0、输出0 -> 慢慢推也不出力
 *
 *   历史教训: 这里曾经做过"位置模式锁位"(先刹停 -> 切 MODE_POSITION ->
 *   死守驱动器回码里的 m1_pos/m2_pos)。它会把驱动器切出速度模式, 再靠一路
 *   陈旧的绝对位置目标维持, 平白多出"模式切换"和"位置目标"两个变量 ——
 *   实测在车上出现了失控。已删除。
 *   现在的停车路径是**无状态**的: 每个控制周期重发同一条帧, 不记忆、不切换。
 *-----------------------------------------------------------------*/
#define STOP_CTRL           CTRL_BRAKE

/*====================== 上电自检点动 (PB9 之外唯一的"自己会动") ==========
 * 上电 **延迟 3 秒** 后, 自己往前点动一下再停。用途是**自检**:
 * 证明 驱动器/CAN/电机/接线方向 这一整条链是好的 —— 动一下就说明都通,
 * 不动就说明有问题(配合诊断帧看 flags bit5 和驱动器故障码能定位到具体哪一环)。
 *
 * ★★ 这是全固件**唯一**一处"不等上位机命令就自己动"。所以边界写死:
 *
 *   1. **延迟 3 秒再走**(SELFTEST_DELAY_MS)。开机瞬间旁边可能有人、车可能贴着墙,
 *      给一个明确的"要先躲开"的窗口。板子一上电 PB9 就开始闪, 所以这 3 秒不是
 *      静默的 —— 看到灯在闪就知道板子活了、接下来会动一下。
 *   2. **等两个驱动器都回过码**(Drive_Ready)。没就绪就只刹车、不动 ——
 *      否则差速车单轮出力 = 原地打转。
 *   3. **上位机一旦开始发指令, 立刻让位**(ever_linked)。绝不自检和主人抢车。
 *      副作用: 如果 RK3588 比这 3 秒先起来并发帧, 自检就直接跳过 —— 这是对的。
 *   4. **看门狗复位过就跳过**(reset_by_iwdg)。IWDG 复位意味着"刚才在跑的时候
 *      主循环卡死过", 车可能正贴着墙/在人旁边; 这种复位绝不能再往前窜一下。
 *   5. 有独立超时(SELFTEST_ABORT_MS): 到点不管走到哪都收手。
 *
 * ★ "10cm"只是**名义值**, 不是承诺: 命令 0.25 m/s × 300ms = 7.5cm, 但停车后
 *   还会滑(本车没有保持力, 见 docs/nav2-reference.md §5.1.1), 所以实际距离
 *   大约是 洗地机 ~12cm / 扫地机 ~16cm。**要更接近 10cm 就把 SELFTEST_MS 调小**
 *   (经验: 每减 100ms 少走约 2.5cm)。真要精确 10cm 得在上位机用 EKF 里程计闭环,
 *   这条开环点动做不到, 也没打算做。
 *
 * ★ 自检"没转"会被记下来(诊断帧 f[30] bit7), 因为"动了一下"是操作员肉眼判断的,
 *   而"到底哪个轮子没转"得看数据。
 *------------------------------------------------------------------------*/
/* ★★ 自检的开关**不在这里** —— 它按驱动模式决定, 必须放在 DRIVE_MODE_SEL
 *   定义之后(见下面"驱动模式"那一段后面的 SELFTEST_ENABLE)。
 *   曾经放在这里 -> 用在了定义之前 -> #if 恒真 -> 自检被静默关掉。 */
#define SELFTEST_DELAY_MS   3000    /* ★ 上电后先等这么久再动 */
#define SELFTEST_VX         0.25f   /* 点动速度 m/s。必须明显高于起步门槛
                                       (洗地机 0.12 / 扫地机 0.22 m/s), 否则
                                       静摩擦推不动, 自检会误报"没转" */
#define SELFTEST_MS         300     /* 点动时长 ms (名义 0.25 × 0.30 = 7.5cm) */
#define SELFTEST_READY_MS   2000    /* 延迟到了但驱动器还没就绪, 再等这么久 */
#define SELFTEST_ABORT_MS   3000    /* 开动之后的硬上限, 到点必停 */

/*---------------------- 串口 / 看门狗 ----------------------------
 *   只有一个上位机: USART1 (PA9/PA10) -> CH340 / RK3588。
 *   LINK_TIMEOUT_MS: 上位机这么久不发命令就自动停车(看门狗)。
 *-----------------------------------------------------------------*/
#define MAIN_BAUD           115200  /* 对齐厂商驱动, 别改 */
#define LINK_TIMEOUT_MS     800

/*---------------------- GPIO 继电器 (外设开关) ----------------------
 *   PB0 -> 滚刷继电器 IN     PB1 -> 水泵继电器 IN
 *
 *   ★ 这两个继电器是**外设开关**: PB0 管扫地滚刷电机, PB1 管水泵。
 *     它们和轮毂电机**没有任何关系** —— 轮毂电机的 FOC 驱动器是独立供电、
 *     常电的, 继电器断开不会给它们断电, CAN 通信和使能状态都不受影响。
 *
 *     ★ 命名教训: PB0 以前叫 RELAY_MOTOR / "电机继电器", 文档里还写成
 *       "继电器给驱动器供电"。于是所有基于"继电器合闸 = 驱动器上电初始化"
 *       的推理全是错的, 白追了一轮"为什么第一条命令没反应"。**叫滚刷**。
 *
 *   ★ 接线铁律: 模块的 VCC/GND 一定要和 STM32 共用同一个电源参考，
 *     最省事的做法就是直接从本板引 5V 和 GND。
 *     曾经用独立 5V 供电(没共地)时，光耦输入回路没有稳定的返回路径，
 *     驱动电流时有时无 -> 光耦半开 -> 线圈半通电 -> 继电器持续嗡嗡响。
 *     改成从本板取 5V/GND 后立刻正常。线也要短、要粗。
 *
 *   触发电平(本车实测): 模块是低电平触发，所以下面用 1。
 *     IN 给低 -> 继电器吸合(负载通电)      IN 给高 -> 断开
 *   换模块后如果"开/关"反了，把这个值改成 0 即可。
 *   模块上如果有 H/L 跳线，拨到 L。
 *
 *   RELAY_OFF_ON_LINK_LOSS: 链路断了(看门狗)自动断开两个继电器。
 *-----------------------------------------------------------------*/
#define RELAY_BRUSH_PIN     GPIO_Pin_0      /* PB0 滚刷电机 */
#define RELAY_PUMP_PIN      GPIO_Pin_1      /* PB1 水泵 */
#define RELAY_PORT          GPIOB
#define RELAY_ACTIVE_LOW    1               /* 1=低电平触发(实测) 0=高电平触发 */
#define RELAY_OFF_ON_LINK_LOSS  1
/*=============================================================*/

/*-------------------------- 状态灯 (PB9) ---------------------------
 * 上电后就一直在 PB9 上"亮-灭-亮-灭"地闪, 当心跳灯用。
 *
 * ★ PB9 的选择: 全片查过没有冲突 —— CAN1 走的是 PA11/PA12(没重映射, 所以
 *   PB8/PB9 空着), USART1 走 PA9/PA10, IMU 软件 I2C 走 PB10/PB11, 继电器是
 *   PB0/PB1。PB9 也不是 JTAG 脚(JTAG 是 PA13/14/15 + PB3/PB4), 所以不需要
 *   任何重映射就能直接当普通 IO 用。
 *
 * ★ 这个灯是**翻转式**的, 不是延时式: 在主循环里按 g_tick_ms 到点翻转, 没有
 *   任何阻塞。所以它顺带还是一个"主循环还活着"的指示 —— 主循环一旦卡死,
 *   灯就定住不动了(而 IWDG 会在 1 秒后复位, 复位后又开始闪)。
 *
 * ★ LED_ON_LEVEL: 点亮电平。**点亮是指"给高"还是"给低"完全取决于你的模块**,
 *   我这边看不到实物, 所以做成一个宏:
 *     1 = 高电平点亮(引脚->电阻->LED->GND, 最常见)
 *     0 = 低电平点亮(3.3V->LED->电阻->引脚, 或模块标了 L/低电平触发)
 *   接反了不会烧, 只是"亮灭颠倒", 把 0/1 对调即可。
 *-----------------------------------------------------------------*/
#define LED_ENABLE          1
#define LED_PORT            GPIOB
#define LED_PIN             GPIO_Pin_9
#define LED_ON_LEVEL        1               /* ★ 亮灭反了就改这个 */
/* ★ LED_BLINK_MS 在"车型选择"里按车型定义, 因为**闪的频率就是车型标识**:
     扫地机 250ms(2Hz) / 洗地机 100ms(5Hz)。
     上车前扫一眼灯就知道这块板烧的是哪辆车的固件 —— 几何选错的后果是
     转向增益差 2.19 倍, 值得一个能一眼看出来的标识。 */
/*=============================================================*/

/*-------------------------- 物理换算 ------------------------------*/
#define WHEEL_CIRC_M        (3.14159265f * WHEEL_DIAMETER_MM / 1000.0f)
#define TRACK_WIDTH_M       (TRACK_WIDTH_MM / 1000.0f)
#define MAX_LIN_SPEED       (MAX_RPM * WHEEL_CIRC_M / 60.0f)

/*-------------------------- CAN --------------------------------*/
#define MOTOR1_CAN_ID      0x1101E600UL
#define MOTOR2_CAN_ID      0x1201E600UL
#define MOTOR1_REPLY_ID    (MOTOR1_CAN_ID + 1UL)   /* 控制帧回码 */
#define MOTOR2_REPLY_ID    (MOTOR2_CAN_ID + 1UL)
#define MOTOR1_REPORT_ID   (MOTOR1_CAN_ID + 3UL)   /* 定时上报帧 */

#define MODE_SPEED         0x05
#define MODE_POSITION      0x06
#define MODE_TORQUE        0x04    /* ★ 力矩模式。协议 PDF 第 4/8 页: 单位 A,
                                      放大 10 倍保留 1 位小数 -> 写 -100~100
                                      对应 -10.0A~+10.0A, 正负 = 扭矩方向。
                                      (我们曾经误判"0x04 不存在", 见
                                       docs/foc-can-protocol.md §2.1/§7) */
#define CTRL_ENABLE        0x01
#define CTRL_DISABLE       0x02
#define CTRL_BRAKE         0x03
#define CTRL_RESET_FAULT   0x0F    /* ★ 复位/清故障。驱动器故障锁存后不用再断电。
                                      目前**没有**被代码用到, 先记在这里 —— 用之前
                                      想清楚"什么时候该清", 别把真故障清成静默。 */

/*====================== 驱动模式: 驱动器自己闭环 or 我们闭环 ==========
 * 背景: 实测驱动器**内部速度环**有极限环 —— 目标 27.9 RPM, 实际在 0↔71 RPM
 * 之间暴力来回冲、电流 7~11A。外环(我们的 Trim)只贡献几个 RPM, 够不着这一层。
 * 而协议**不暴露任何参数读写**(见 docs/foc-can-protocol.md §6), 我们调不了它。
 *
 * 所以只剩两条路: ①找厂家调; ②**切扭矩模式, 速度环由 STM32 自己做**。
 * 这个开关就是第 ② 条。两条路都保留, 靠编译宏 A/B, 默认还是速度模式。
 *
 *   DRIVE_MODE_SPEED  0 : 驱动器内部速度环(现状, 默认)。Trim/Boost 在它外面打补丁。
 *   DRIVE_MODE_TORQUE 1 : 我们发扭矩, 自己用 PI 做速度环。Trim/Boost **不再使用**
 *                         (它们本来就是为"修补驱动器那个环"而生的; 现在那个环不在了,
 *                          两个积分环串在一起必然打架 —— 只留一个积分器)。
 *
 * ★★ 安全模型完全不同, 这是这个开关最需要注意的地方 ★★
 *   速度模式: 指令卡住 = 匀速跑(危险但收敛)。
 *   扭矩模式: 指令卡住 = **持续加速**(不收敛)。所以要额外三道保护,
 *             见下面 Torque_Guard() —— 没有它们, 扭矩模式不能上车。
 *===================================================================*/
#define DRIVE_MODE_SPEED    0
#define DRIVE_MODE_TORQUE   1
#ifndef DRIVE_MODE_SEL
/* ★★ 这个默认值在 torque-mode 分支上是 **TORQUE** ★★
 *   也就是在这个分支上, Keil 里照常选 sweeper/scrubber, 编出来的就是**力矩模式**
 *   固件 —— 不需要额外选 target, 更不需要手改源码(手改源码正是"选 sweeper 却烧成
 *   洗地机"那次事故的成因, 绝不再引入第二种"我记得我改过")。
 *   想在本分支编速度模式做 A/B: 命令行加 -DDRIVE_MODE_SEL=0(工程里的 <Define>
 *   优先于这里的默认值), 或者把这里改回 DRIVE_MODE_SPEED。
 *   ★ main 分支上这个默认值是 DRIVE_MODE_SPEED, 不受影响。 */
#define DRIVE_MODE_SEL      DRIVE_MODE_TORQUE
#endif

/* ★★ 上电自检点动: 力矩模式下默认关掉 ★★
 *   理由: 自检会在上电 3 秒后自己往前窜一下。速度模式下那一窜是驱动器速度环
 *   执行的、幅度可预期; 而力矩模式的增益(TQ_KP/TQ_KI/TQ_STICTION)目前**全是
 *   没标定的估计值**, 同一条命令可能窜出去很远。第一次上电不该带一个
 *   "参数还没标定的自动前窜"。
 *   台架标定完、并且确认过点动幅度之后, 再把下面这个改回 1。
 *
 *   ★★★ 位置很重要: 这一段**必须放在 DRIVE_MODE_SEL 定义之后** ★★★
 *   它一开始被放在自检那段注释下面(文件靠前的位置), 也就是**在 DRIVE_MODE_SEL
 *   定义之前** —— 而 C 预处理器对未定义标识符按 0 处理, 于是
 *       #if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
 *   变成 `0 == 0` **恒真**, 结果自检在**两种模式下都被关掉了**。
 *   症状很隐蔽: 编译 0 Error 0 Warning, 只是速度模式的自检悄悄没了
 *   (2026-10-12 用"开关切换前后 hex 是否相同"抓到的: 两份 hex 逐字节一样)。
 *   教训: **凡是 #if 引用某个宏, 先确认那个宏在这行之前已经定义。** */
#ifndef SELFTEST_ENABLE
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
#define SELFTEST_ENABLE     0
#else
#define SELFTEST_ENABLE     1
#endif
#endif

/* 故障码 (FOC_CH4-V1.2 第 11 页附表), 回码 DATA1 / 上报帧 DATA2 */
#define FOC_FAULT_NONE       0
#define FOC_FAULT_DRIVER     1
#define FOC_FAULT_OVERCURR   5
#define FOC_FAULT_OVERVOLT   6
#define FOC_FAULT_UNDERVOLT  7    /* 24V 电池带 36V 额定电机, 重点怀疑对象 */
#define FOC_FAULT_OVERTEMP   8
#define FOC_FAULT_HALL_M2    23
#define FOC_FAULT_HALL_M1    24
#define FOC_FAULT_STALL_M2   25
#define FOC_FAULT_STALL_M1   26
#define FOC_FAULT_UART       27
#define FOC_FAULT_RS485      28
#define FOC_FAULT_CAN        29

/*------------------ 命令帧的加减速斜坡 (DATA4 / DATA5) ------------
 *  驱动器文档原文: "当前转速为 0rpm, 目标 100rpm, 若设置间隔为 1ms,
 *  则目标转速值每 1ms 加 1, 直至 100rpm"。
 *  即 DATA4 的单位是 **ms / 每 1 RPM**, 范围 0~100, 0 = 阶跃(响应最快)。
 *
 *  ★ 现在改回 0 (阶跃)。原因:
 *    当初从 0 改成 3 是为了让起步柔和、少顶过流保护, 但**一直没做 A/B**。
 *    后来实测发现底盘直线会漂 (用户目测 2 米偏 40°, 约 20°/m), 而在更早
 *    (还是 ESP32 当上位机的年代) 的记录里是 7.4°/m —— 也就是变差了 2.7 倍。
 *    命令帧格式两代完全一致(厂商 Cmd_Vel_Callback 和自己写的 ESP32 发的
 *    是同一套 11 字节), CAN 重发周期也都是 20ms, **两代之间唯一的运动相关
 *    差异就是这个斜坡参数**。所以先把它改回 0, 把变量消掉再看。
 *
 *    如果改回 0 之后漂移没变化, 说明斜坡无关, 那就是驱动器速度环本身的
 *    问题(它确实在振荡: 目标 18RPM 时实际在 0~22 之间来回冲)。
 *    那时再把斜坡加回来也不迟 —— 它本来是为了柔顺, 不是为了准确。
 *
 *  减速保持 0: 要停就立刻停, 别拖泥带水(停车安全优先)。 */
#define ACCEL_STEP_MS      0
#define DECEL_STEP_MS      0

/*-------------------------- 协议 --------------------------------*/
#define CMD_FRAME_SIZE     11
#define CMD_HEAD           0x7B
#define TEL_FRAME_SIZE     24
#define TEL_HEAD           0x7B
#define TEL_TAIL           0x7D
#define DIAG_FRAME_SIZE    36
#define DIAG_HEAD          0x7E
#define DIAG_TAIL          0x7D

/* 命令帧里的功能码(f[1]), 沿用协议里"非速度帧靠功能码区分"的做法。
 * 协议已占用: 0x04 灯带 / 0x01 回充开关 / 0x00 安全防护, 0x05 是空的。 */
#define FUNC_RELAY         0x05     /* f[2] = 掩码: bit0 滚刷继电器 bit1 水泵继电器 */

/*-------------------------- 周期 --------------------------------*/
#define CAN_PERIOD_MS      20       /* 向电机重复发送的周期 */
#define ODOM_PERIOD_MS     20       /* 轮速采样/(可选)卡尔曼周期 (50Hz) */
#define TELEMETRY_PERIOD_MS 50      /* 上报周期 (20Hz) */
#define RPM_STALE_MS       200      /* 驱动器这么久没回码就认为转速无效 */

/*------------------- 轮速闭环修正 (外环 trim) ----------------------
 * 为什么需要:
 *   驱动器自己的速度环在小误差下给出的电流不足以快速突破静摩擦。实测原地
 *   转: 第 0 秒只有目标的 1%~26% —— **即使目标高达 43 RPM 也一样**; 而直线
 *   运动 0.07 秒就到 80%。同一个轮速区间表现差这么多, 所以这不是驱动器低速
 *   死区, 是原地转要克服轮胎刮擦/静摩擦。轮子是真的没转(不是空转打滑),
 *   所以把指令顶上去它就会转。
 *   STM32 这边原来是**纯开环**: 算完目标轮速直接发, 从不回读实际转速。
 *
 * ★ 第一版的教训 (已实测, 别重犯)
 *   第一版是"纯积分 + 绝对限幅 ±30 RPM + KI=1.5"。原地转的四秒总角度确实从
 *   41%~82% 提到了 85%~93%, 稳态也从 115% 回到了 101% —— 但代价是:
 *
 *   1. **绝对限幅在小名义值下会把指令顶成反向。** 原地转 wz=0.30 的名义轮速
 *      只有 13 RPM, 而修正量能到 -30, 于是 指令 = 13-30 = **-17 (轮子反转)**,
 *      车直接停住。直线 0.15 m/s (13.9 RPM) 更明显: 车中途往回走, 于是
 *      chassis_check 报出"后 40% 有 92% 但全程只有 26%"这种自相矛盾的数据。
 *      一个"修正"把自己修成了故障。
 *   2. **KI=1.5 太快, 和驱动器那个环组成慢速极限环。** 逐秒角速率变成
 *      过冲→塌陷→再爬: wz=1.00 是 0.43/1.73/0.90/0.54 (第 1 秒 173%)。
 *      积分增益和"被控对象滞后"的乘积大于 1 就会这样。
 *   3. 原注释里"输出永远落在普通全速命令能到的范围内"是**错的** ——
 *      MAX_RPM 是 200, 而 ±30 的绝对限幅在名义值 13 时已经把指令推到 2~3 倍。
 *
 * 第二版(现在)的三条改动, 逐条对症:
 *   a. **限幅改成按名义值的比例**(TRIM_UP_K / TRIM_DOWN_K), 不用绝对值。
 *      这样每种速度下行为一致, 而且反向最多只能减到名义值的一半 ——
 *      **修正量在任何情况下都不可能把指令的方向改掉**, 反转从机制上被消除。
 *   b. **KI 降到 0.6**, 让它只修稳态偏差, 不去和驱动器那个环抢动态。
 *   c. **起步助推 (kick) 单独做**, 见 KICK_*。突破静摩擦需要的是"立刻给力",
 *      而积分天生要"等误差攒起来"。把积分调快到能立刻给力, 它就会在轮子追上
 *      来之后迟迟不回, 造成上面那种过冲和振荡。两件事分开: 积分慢而稳只管
 *      稳态, 助推开环/定时/有界只管起步这一下 —— 开环所以不会构成第二个
 *      反馈回路, 也就不会振。
 *
 * 安全性(按"绝不引入新的失控模式"设计):
 *   1. 修正量按比例限幅(正向最多 +TRIM_UP_K 倍名义值, 反向最多 -TRIM_DOWN_K 倍),
 *      再受 TRIM_MAX_RPM 约束; 输出另受 MAX_RPM 约束。**净效果: 指令永远落在
 *      [1-TRIM_DOWN_K, 1+TRIM_UP_K] 倍名义值之内, 方向不变。**
 *   2. 目标为 0(停车/刹车/单轮不动)时修正量和助推一起清零, 不留历史。
 *   3. 驱动器回码不新鲜时不积分(只保持) —— 拿旧数据积分会把修正量喂到限幅。
 *   4. 修正量可从诊断帧推出(上报的"目标"是修正后的最终命令, 减去主机下发的
 *      理论目标就是修正量), 它有没有在乱冲一看便知。
 *=================================================================*/
/* ★ 第二版的实测结论(逐帧抓到了): 积分在**起步暂态**里把自己顶到限幅, 然后
 *   和驱动器的 PI 组成一个 0.25Hz 的极限环 —— `目标-理论` 那一列直接看得到:
 *       +7 -> +13(贴住限幅) -> +12 -> +9 -> +5 -> 0, 来回摆 ±14 RPM,
 *       周期 4~5 秒, 和速度振荡测出来的 0.25Hz 完全吻合。
 *   机制: 起步时轮子没转 -> 误差巨大 -> 积分顶到 +50% -> 指令 1.5 倍 ->
 *         驱动器本来就过冲 -> 实际冲到 52 RPM(名义 186%) -> 误差大幅反向 ->
 *         积分慢慢退回甚至摆到 -50% -> 指令 0.5 倍 -> 车慢下来 -> 积分又上去。
 *   而同一趟的**均值是 99%** —— 说明直行的稳态本来就不需要修, 积分没干好事。
 *
 * 所以第三版改三处, 都针对"别去积分它控制不了的暂态":
 *   a. **加线性带**(TRIM_LIN_BAND_RPM): 误差大于这个带就不积分。
 *      起步/被刮擦卡住时误差是 20~28 RPM, 落在带外 -> 完全不积分 ->
 *      从根上消掉那个 windup。轮子接近目标后才慢慢修稳态残差。
 *   b. **KI 0.6 -> 0.3**: 和外层拉开增益。驱动器里面已经有一个 PI 在管同一个
 *      误差, 两个积分器串在同一个误差上**必须拉开增益**, 否则必然振。
 *   c. **权限 ±50% -> ±20%**: 就算还有残余振荡, 幅度也被限在 ±20% 而不是
 *      ±50%(实测振幅就是限幅值本身)。稳态残差通常只有几个百分点, ±20% 够用。
 *
 * 助推(KICK)当时没动 —— 它只在起步那一小段起作用, 不在这个 0.25Hz 环里。
 * (助推后来在第四版改掉了, 见下面 KICK_* 那一段。) */
#define TRIM_ENABLE        1
#define TRIM_KI            0.3f     /* 每秒、每 1 RPM 偏差攒多少 RPM 修正 */
#define TRIM_UP_K          0.2f     /* 正向修正最多 +20% 名义值 */
#define TRIM_DOWN_K        0.2f     /* 反向修正最多 -20% 名义值 -> 永不反转 */
#define TRIM_MAX_RPM       20.0f    /* 绝对上限, 防高名义值下修正量过大 */
#define TRIM_LIN_BAND_RPM  5.0f     /* ★ 误差超过它就不积分(见上面第 a 条) */
#define TRIM_DEADBAND_RPM  0.5f     /* 目标小于这个就当"不该动", 全部清零 */

/* ★ 第三版(2026-09)【判据已被第四版取代, 这里只留下"为什么要动它"的来龙去脉】:
 *   助推从"一次性定时"改成"**卡住就给**"(带迟滞)。
 *
 * 为什么必须改 —— 实车报的"发小一点的转向命令, 车会愣在原地不动"就是这个:
 *   小原地转的单轮目标可能只有 8~13 RPM(对应 wz 0.2~0.3), 本来就在驱动器死区
 *   (~14~18 RPM)之下。旧逻辑靠助推把它顶到 20+ 破静摩擦, **但助推只有 400ms**,
 *   一到就撤; 撤掉后轮子掉回死区又卡住, 而助推**不会重新武装**(目标从没回到 0)
 *   -> 车就永远愣在原地。助推本来是为了解决"起步没劲", 结果在小命令上反而
 *   造了一个新故障。
 *
 * 第三版的判据(**已被第四版换掉, 因为它按名义值比例判定, 见下**):
 *     实际轮速 < 名义值 × 0.2   -> 给
 *     实际轮速 > 名义值 × 0.6   -> 撤
 *     中间                      -> 保持上一状态
 * 好处: 一直卡着就一直有助推(自愈, 不需要"重新武装"); 一旦真转起来立刻撤,
 * 过冲也被压小 —— 旧逻辑固定给满 400ms, 命令越大过冲越大。
 *
 * ★ 迟滞不能省: 单阈值会让助推在阈值附近按 20Hz 来回开关 -> 轮子嗡嗡抖。
 *
 * ★★ 助推必须是**两轮共享的一次判决**, 不能每轮各判一次(Boost_Decide)。
 *    助推是**加性**大增量, 在小命令下相当于把指令翻好几倍。若按各自转速独立
 *    判决, 转弯/起步时总有一个轮子先转起来、先撤掉助推, 另一个还在加 ——
 *    两轮指令瞬间差十几 RPM, 车就会"歪一下"。旧版是固定 400ms 定时, 两边
 *    同时给同时撤, 天然对称; 改成"卡住就给"之后必须显式做成共享, 否则等于
 *    凭空引入一个偏航源(而且它只在起步那一下出现, 最难查)。
 *    共享判据: **任一有效轮没转起来 -> 给; 两个有效轮都转起来了 -> 撤。**
 *
 * ==================================================================
 * 第四版(2026-09, 现在): 第三版那套"名义值比例"判据还是不对, 改成
 * **绝对转速判据 + 按缺口给量 + 给不动就加力**。原因是实车又报了两个症状,
 * 而它们是**同一条曲线的两端**:
 *     小命令(|名义| < ~15 RPM, 即 wz < 0.35)  -> 推不动, 一直愣在原地;
 *     稍大一点                                 -> 一下子窜出去, "转向太敏感"。
 * 这正是**静摩擦(起步门槛)**的特征, 而且 ★关键: 门槛是**起步**门槛, 不是
 * 速度死区 —— 实测转起来之后 6 RPM 也能维持(wz=0.30 的原地转测到 49%,
 * 也就是 6.4 RPM 真的在转)。所以"给一下就撤"是对的, "一直给"是错的。
 *
 * 第二版那套的两个毛病正好一人对应一个症状:
 *   a. 判据按**名义值比例**: 名义值只有 10 RPM 时, "转起来"的门槛是 6 RPM,
 *      而驱动器其实要 15 才动 —— 助推被判定为"已经转起来了"而提前撤掉,
 *      轮子掉回静止, 然后又"没跟上"再给, 于是在静止附近反复抖, 平均还是不动。
 *      (症状: 愣在原地)
 *   b. 助推量**固定 +12 RPM**, 而且**按各自名义方向加**。这一条对原地转的
 *      影响被严重低估了 —— 加性助推让右轮 +12、左轮 -12, 换算成偏航角速率:
 *          每轮名义 RPM = wz × K,  K = (轮距/2)/轮周长 × 60
 *                                  = 43.26 (扫地机) / 27.06 (洗地机)
 *          所以助推 b 等效于把 wz 抬高 b / K rad/s
 *          b=12 -> 扫地机 **+0.28 rad/s**, 洗地机 **+0.44 rad/s**(更严重,
 *                 因为轮距窄: 同样的轮速差换来的角速度更大)
 *      也就是原地转**起步那段时间真实 wz 比命令值高出一大截**:
 *      扫地机 wz=0.30 的命令实际是 0.58(192%), wz=0.50 的实际是 0.78(155%)。
 *      (症状: 转向太敏感 —— 而且这也解释了为什么**稳态**测出来是 100~102%
 *       却仍然手感"贼": 助推是暂态, 稳态数据里看不到它, 人却每次起步都能感到。
 *       旧版是固定 400ms 定时, 所以这个"窜"每次转向都准时出现 400ms。)
 *
 * 第四版三条:
 *   1. **绝对转速判"动没动"**(KICK_STILL_RPM / KICK_MOVING_RPM), 不看比例。
 *      要破的是静摩擦, "轮子转没转"这件事跟命令多大无关。
 *      副产物: 判据与命令大小无关 -> 大命令上不会过度助推, 也不会抖。
 *   2. **按缺口给量**(KICK_BASE_RPM):
 *          助推 = clamp(KICK_BASE_RPM - 最小|名义值|, 0, KICK_MAX_RPM)
 *      名义值本身就超过起步门槛 -> 助推 = 0 -> **正常行驶完全没有助推**:
 *        · 直行 0.30 m/s 名义 27.9 RPM -> 助推 0, 起步那一下的过冲消失;
 *        · 原地转 wz >= 15/K -> 扫地机 0.35 / 洗地机 **0.55** -> 助推 0;
 *        · 扫地机 wz=0.30(名义 13 RPM) -> 助推 2 RPM   -> 额外 wz +0.05 (15%);
 *        · 扫地机 wz=0.25(名义 10.8)   -> 助推 4.2 RPM -> 额外 wz +0.10 (39%)。
 *      ★ 洗地机轮距窄, 同样的轮速差换来的 wz 更大, 所以**助针对偏航的影响
 *        在洗地机上更明显**, 而且"助推=0"的门槛从 0.35 抬到 0.55 rad/s。
 *        洗地机第一次上车要重新确认这两条(门槛和超调)。
 *      小命令上的超调仍然存在, 但它是**破静摩擦的物理代价**, 而且**只在
 *      |实际| < 6 RPM 的这段时间里存在**(一到 6 RPM 立刻撤, 见 KICK_MOVING_RPM)。
 *   3. **给不动就逐级加力**(KICK_STEP_*): 补到门槛还不动, 每 KICK_STEP_MS
 *      再加 KICK_STEP_RPM, 直到动起来或到 KICK_MAX_RPM。这样"到底要多大劲
 *      才动得起来"是**实测量出来的**, 不靠我猜的 KICK_BASE_RPM —— 它只是起点。
 *
 *   4. **加到上限还不转就"给一会儿歇一会儿"**(KICK_MAX_MS / KICK_COOL_MS):
 *      轮子被东西顶住时不能无限加力(会烤电机和驱动器), 所以加满上限还不动
 *      就先歇 KICK_COOL_MS, 再接着试, 并把 flags bit7 置位让上位机看得见。
 *      ★★ 这里**绝对不能做成"永久放弃"**。第一版的毛病就是"超时之后再也不给",
 *         于是小命令永远愣在原地 —— 如果这台车破静摩擦真的需要比 KICK_MAX_RPM
 *         更大的劲, 一次放弃就等于把那个故障原样造回来。占空比
 *         (3s 给 / 2s 歇) 已经能把堵转发热砍掉一大半, 而车最终还能脱困。
 *         目标归零时这个"歇"也一并清掉, 等于上层显式重下命令就重新给足机会。
 *
 * ★ 撤回判据用绝对值还有一个原因: 助推必须留到轮子进入"能自持"的转速
 *   (≈6 RPM) 才能撤。若按名义值的比例撤, 名义值 4 RPM 时会要求"转到 2.4
 *   才算转起来", 那时命令一撤回 4 RPM 轮子立刻又停 —— 助推变成振铃。 */
#define KICK_BASE_RPM      15.0f    /* 起步门槛估计: 助推把命令补到这个轮速 */
#define KICK_MAX_RPM       20.0f    /* 助推上限(含逐级加力) */
#define KICK_STEP_RPM      4.0f     /* 给不动 -> 每 KICK_STEP_MS 再加这么多 */
#define KICK_STEP_MS       200      /* 加力间隔 */
#define KICK_STILL_RPM     2.0f     /* |实际| 低于它 = 还是静止 */
#define KICK_MOVING_RPM    6.0f     /* |实际| 高于它 = 转起来了, 撤助推 */
#define KICK_MAX_MS        3000     /* 加到上限还不动, 先给这么久就歇一下(防卡死) */
#define KICK_COOL_MS       2000     /* ★ 歇多久再试。**不能永久放弃**, 见下面说明 */

/*-------------------------- IMU --------------------------------*/
/* IMU 读一次大概 1~3ms(位翻转 I2C), 和轮速一起按 ODOM_PERIOD_MS 采样。
 *
 * ★ 协议要求上报的 IMU 原始值必须是 ±2g / ±500dps 量程下的值, 因为上位机
 *   是用固定系数换算的(MD 5.1 节):
 *       加速度: 原始值 / 1671.84    = m/s²
 *       陀螺仪: 原始值 * 0.00026644 = rad/s
 *   而 YbImu 的原始量程是 ±16g / ±2000dps, 所以这里必须换算 + 限幅。
 *   换错量程不会报错, 只会让上位机的数据全错。
 */
#define IMU_ACCEL_G_TO_RAW  (9.80665f * 1671.84f)   /* g      -> ±2g   原始值 */
#define IMU_GYRO_TO_RAW     (1.0f / 0.00026644f)    /* rad/s  -> ±500dps 原始值 */
#define IMU_GYRO_Z_SIGN     1       /* 陀螺 Z 轴方向; 见下面 IMU_Tick 里的用法 */

/*=========================== 全局状态 ==============================*/
volatile uint32_t g_tick_ms;

/*---------------------- 串口口 (就一个) ----------------------------*/
#define PORT_MAIN   0
#define PORT_COUNT  1

typedef struct
{
    uint8_t  id;
    USART_TypeDef *usart;
    uint8_t  ring[64];                  /* 接收环形缓冲 */
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t overflow;
    uint8_t  frame[CMD_FRAME_SIZE];     /* 组帧滑动窗口 */
    uint8_t  len;
    uint8_t  ok_count;
    uint8_t  bad_count;
} uart_port_t;

static uart_port_t ports[PORT_COUNT];

/* 目标 / 状态 */
static float    target_vx;          /* m/s */
static float    target_wz;          /* rad/s */
static uint8_t  ever_linked;
static uint8_t  failsafe_latched;

/* 两个驱动器是否都回过码 (bit0 = 1号, bit1 = 2号, 自开机起累计)。
 *
 * ★ 为什么需要: 差速车**只有一个轮子出力就等于原地打转**。实测出现过同一条
 *   直行命令连着发三次, 结果分别是"没反应"、"向左转"、"正常前进" ——
 *   直行变左转, 就是那一次只有一个驱动器响应了。
 *   所以策略: 没确认两个驱动器都活着之前, 只发刹车, 不发速度。见 Drive_Ready()。
 *
 * ★ 这里以前写的是"继电器合闸后等驱动器上电初始化" —— 那是错的:
 *   继电器管的是**滚刷和水泵**, 和轮毂驱动器没有关系(驱动器是常电的)。
 *   等待条件本身是对的, 错的是理由; 现在按真实理由重写。 */
static uint8_t drv_seen_mask;

/* 物理轮速目标 —— 上报给上位机, 用来和实际轮速对比(诊断"没劲"的关键) */
static int16_t  target_left_rpm;
static int16_t  target_right_rpm;

static uint32_t last_cmd_ms;        /* 最后一条**速度帧** (决定运动超时) */
static uint32_t last_frame_ms;      /* 最后一条**任何合法帧** (决定链路超时, 见看门狗) */
static uint32_t last_can_ms;
static uint32_t last_odom_ms;
static uint32_t last_tel_ms;

/* 诊断计数 */
static uint8_t  can_error_count;

/* 驱动器回码 (控制帧 ...E601 的应答, 每个字段来自同一帧, 时间戳共用 m*_rpm_ms) */
static volatile uint8_t m1_fault, m2_fault;     /* DATA1 故障码, 见 FOC_FAULT_* */
static volatile uint8_t m1_mode,  m2_mode;      /* DATA0 当前运行模式 0x05/0x06... */
static volatile int16_t m1_rpm,   m2_rpm;       /* DATA2/3 实际转速 RPM */
static volatile int16_t m1_cur,   m2_cur;       /* DATA4/5 输出扭矩电流, A x10 */
static uint32_t m1_rpm_ms, m2_rpm_ms;
static volatile uint16_t battery_mv;

/* 轮速计 */
static int16_t  wheel_left_rpm, wheel_right_rpm;   /* 已按接线校准的物理轮速 */
static uint8_t  wheel_left_fresh, wheel_right_fresh; /* 该轮回码是否新鲜 */
static float    body_vx, body_wz;                  /* 轮速正解出的车体速度 */

/* ★ 下面的 Trim(轮速外环) 和 Boost(起步助推)**只服务于速度模式** —— 它们的
   存在意义是给驱动器内部那个(有极限环的)速度环打补丁。切到力矩模式之后我们自己
   就是那个环(见 Torque_Apply), 两个积分环串联必然打架, 所以整段不编译。
   用 #if 隔开而不是留着不用: ARMCC 会对"定义了但没人引用"报警告, 而本工程
   坚持两个模式都 0 Warning —— 有警告就意味着"这段代码的生死没人管"。 */
#if DRIVE_MODE_SEL == DRIVE_MODE_SPEED

/* 外环修正状态 (物理轮速域, 左右各一份) —— 见 TRIM_* 的说明 */
typedef struct
{
    float    trim;          /* 积分修正量, RPM */
} wheel_trim_t;

static wheel_trim_t trim_w[2];      /* [0] = 左轮, [1] = 右轮 */

/* 助推的状态 —— ★ 这几个是**两轮共享**的, 不是每轮一份。
 *
 * 为什么必须共享: 助推是**加性**大增量(起步时相当于把命令翻好几倍)。如果按
 * 各自的实际转速独立决定, 一个轮子先转起来就会先撤掉助推 -> 两轮指令差出十几
 * RPM -> **起步时车会歪一下**。旧版是固定 400ms 定时, 两边同时给同时撤, 天然
 * 对称; 改成"没转起来就给"之后必须显式做成共享, 否则等于凭空引入一个偏航源。
 *
 * 判据(共享): 任一有效轮还没转起来 -> 给; 两个有效轮都转起来了 -> 撤。 */
static uint8_t  boost_on;           /* 助推这一轮是否开着 (带迟滞, 见 KICK_* 的说明) */
static float    boost_amt;          /* 当前助推量 RPM (会逐级加力, 见 KICK_STEP_*) */
static uint16_t boost_ms;           /* 本轮已连续助推了多久, 用于防卡死保护 */
static uint16_t boost_step_ms;      /* 距上次加力多久 */
static uint16_t boost_cool_ms;      /* 防卡死"歇一会儿"的剩余时间, 见 KICK_COOL_MS */

#endif  /* DRIVE_MODE_SEL == DRIVE_MODE_SPEED */

/* ★ 左右轮的下标两种模式都用(trim_w 和 tq_w 同序), 所以放在 #if 外面 */
#define TRIM_IDX_LEFT   0
#define TRIM_IDX_RIGHT  1

/* ★ boost_stuck 也放在 #if 外面: 诊断帧的 flags bit7 要读它, 而且两种模式下
   "有没有卡住过"都是有意义的信息(力矩模式下由 Torque_Guard 的堵转保护对应)。 */
static uint8_t  boost_stuck;        /* 加到上限还不转 -> 锁存。目标归零才清 */

/* IMU (YbImu, 位翻转 I2C on PB10/PB11) */
static float    imu_accel_g[3];      /* 单位 g */
static float    imu_gyro[3];         /* 单位 rad/s */

/* IMU 诊断状态 —— 通过诊断帧上报, 方便查 I2C 接线/器件问题 */
static uint8_t  imu_ok;              /* 1 = 加速度有效 */
static uint8_t  imu_gyro_ok;         /* 1 = 陀螺仪有效(不是恒 0) */
static uint8_t  imu_status;          /* YBIMU_ST_* 失败原因 */
static uint8_t  imu_found_addr;      /* 扫描到的器件地址, 0 = 没扫到 */
static uint32_t imu_diag_ms;         /* 上次诊断的时刻 */

/* IMU 寄存器体检结果 (1Hz 刷一次)。
   陀螺仪恒 0 时靠它区分"只有陀螺仪坏"还是"整个模块只剩加速度可用"。 */
static uint8_t  imu_probe_flags;     /* bit0 版本 bit1 陀螺 bit2 磁力 bit3 四元数 bit4 欧拉 */
static uint8_t  imu_ver_major;
static int16_t  imu_euler_yaw_crad;  /* 模块自己融合的偏航角, 单位 0.01rad */
static uint32_t imu_probe_ms;        /* 上次寄存器体检的时刻 */

/* 上次复位是不是看门狗引起的 —— 是就说明主循环卡死过, 必须知道 */
static uint8_t  reset_by_iwdg;

/* 继电器 */
static uint8_t  relay_state;                       /* bit0 滚刷 bit1 水泵 */

/*=========================== 1ms 时基 ==============================*/
static void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    tim.TIM_Prescaler = 7200 - 1;          /* 72MHz / 7200 = 10kHz */
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 10 - 1;               /* 10kHz / 10  = 1kHz -> 1ms */
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        g_tick_ms++;
    }
}

/*======================= 独立看门狗 (IWDG) =========================
 * ★ 为什么必须有它: 原来只有"软件看门狗" —— 在主循环里检查上位机多久没发
 *   命令。**但它本身也在主循环里**: 一旦主循环卡住(比如某处死等), 软件看门狗
 *   也跟着停, STM32 就再也不发 CAN 命令了。
 *
 *   而 FOC 驱动器**没有命令超时** —— 它会一直执行最后收到的那条命令。
 *   于是最坏情况是: 主循环卡死 -> STM32 静默 -> 驱动器保持最后的速度
 *   -> **车自己开走**。
 *
 *   IWDG 用的是芯片内部独立的 40kHz LSI(不受主时钟影响), 主循环卡住超过
 *   超时时间就强制复位 MCU。复位后 main() 会在 CAN 初始化完的第一时间发
 *   刹车帧, 车就停下了。
 *
 *   超时 = 预分频 / LSI * 重装值 = 64 / 40000 * 625 = 1.0 秒。
 *   (LSI 实际在 30~60kHz 之间飘, 所以真实超时约 0.67~1.33 秒, 够用。)
 *
 *   IWDG 一旦使能就**关不掉**, 只能靠复位。所以初始化放在这里、喂狗放主循环。
 *=================================================================*/
static void IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);   /* 40kHz / 64 = 625Hz */
    IWDG_SetReload(625);                    /* 625 / 625Hz = 1 秒 */
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/*============================= CAN1 ================================*/
static void CAN1_Init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12;               /* CAN1_TX */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;               /* CAN1_RX */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = ENABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_8tq;
    can.CAN_BS2 = CAN_BS2_3tq;
    /* PCLK1 = 36MHz: 36 / 6 / (1 + 8 + 3) = 500 kbit/s */
    can.CAN_Prescaler = 6;
    CAN_Init(CAN1, &can);

    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0;
    filter.CAN_FilterIdLow = 0;
    filter.CAN_FilterMaskIdHigh = 0;
    filter.CAN_FilterMaskIdLow = 0;
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);
}

static void CAN1_SendRaw(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint8_t status;
    uint8_t index;
    uint32_t timeout;

    tx.StdId = 0;
    tx.ExtId = id;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = dlc;
    for (index = 0; index < 8; index++)
    {
        tx.Data[index] = (index < dlc) ? data[index] : 0;
    }

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        if (can_error_count != 0xFF) can_error_count++;
        return;
    }

    timeout = 20000;
    do
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
    } while ((status == CAN_TxStatus_Pending) && (--timeout != 0));

    if (status != CAN_TxStatus_Ok)
    {
        if (can_error_count != 0xFF) can_error_count++;
    }
}

/* 给一台电机发指令: 模式 + 控制字 + 设定值(转速 RPM 或位置 度) */
static void CAN1_SendMotorCmd(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t value)
{
    uint8_t data[8];
    uint16_t raw = (uint16_t)value;

    data[0] = mode;
    data[1] = ctrl;
    data[2] = (uint8_t)(raw >> 8);
    data[3] = (uint8_t)raw;
    data[4] = ACCEL_STEP_MS;    /* 加速间隔 ms/1RPM, 0=阶跃 */
    data[5] = DECEL_STEP_MS;    /* 减速间隔 ms/1RPM, 0=阶跃 */
    data[6] = 0;                /* 输入电源低压保护阈值(V), 0=不设(用驱动器默认) */
    data[7] = 0;                /* 预留 */
    CAN1_SendRaw(id, 8, data);
}

#define CAN1_SendSpeed(id, rpm)  CAN1_SendMotorCmd((id), MODE_SPEED, CTRL_ENABLE, (rpm))

/* ★ 停车帧按驱动模式选模式号, 但**控制字和值永远一样**(刹车 + 0)。
 *   这样做的好处是: 全固件所有停车路径(上电、看门狗、驱动器没就绪、上电自检收尾、
 *   目标为 0)都自动跟着模式走, 一处都不用改 —— 而且扭矩模式下**模式号恒定不变**,
 *   不会半个字出现"跑着跑着切模式"(协议要求"模式切换需在电机停止空闲状态下生效",
 *   见 docs/foc-can-protocol.md §2.3)。 */
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
#define CAN1_SendStop(id)        CAN1_SendMotorCmd((id), MODE_TORQUE, CTRL_BRAKE, 0)
#define CAN1_SendTorque(id, i)   CAN1_SendMotorCmd((id), MODE_TORQUE, CTRL_ENABLE, (i))
#else
#define CAN1_SendStop(id)        CAN1_SendMotorCmd((id), MODE_SPEED, STOP_CTRL,   0)
#endif

static void CAN1_Poll(void)
{
    CanRxMsg rx;
    uint32_t id;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        id = (rx.IDE == CAN_Id_Extended) ? rx.ExtId : rx.StdId;

        /* 控制帧回码 ...E601:
             DATA0=当前运行模式  DATA1=故障码  DATA2/3=实际转速
             DATA4/5=当前输出扭矩电流(A x10)  DATA6/7=当前位置(度)
           DATA4/5 是判断"这个轮子到底出没出力"唯一的客观指标:
           目标转速不等于实际转速时, 看电流就知道是驱动器没给力(电流≈0)
           还是给了力但被堵住/拖住了(电流很大)。 */
        if ((id == MOTOR1_REPLY_ID) && (rx.DLC >= 4))
        {
            m1_mode  = rx.Data[0];
            m1_fault = rx.Data[1];
            m1_rpm = (int16_t)(((uint16_t)rx.Data[2] << 8) | rx.Data[3]);
            m1_rpm_ms = g_tick_ms;
            drv_seen_mask |= 0x01;          /* 1号回过码了 */
            if (rx.DLC >= 6)
            {
                m1_cur = (int16_t)(((uint16_t)rx.Data[4] << 8) | rx.Data[5]);
            }
        }
        else if ((id == MOTOR2_REPLY_ID) && (rx.DLC >= 4))
        {
            m2_mode  = rx.Data[0];
            m2_fault = rx.Data[1];
            m2_rpm = (int16_t)(((uint16_t)rx.Data[2] << 8) | rx.Data[3]);
            m2_rpm_ms = g_tick_ms;
            drv_seen_mask |= 0x02;          /* 2号回过码了 */
            if (rx.DLC >= 6)
            {
                m2_cur = (int16_t)(((uint16_t)rx.Data[4] << 8) | rx.Data[5]);
            }
        }
        /* 定时上报帧 ...E603: DATA4/5=电源输入电压(放大10倍) -> mV */
        else if ((id == MOTOR1_REPORT_ID) && (rx.DLC >= 6))
        {
            int16_t v10 = (int16_t)(((uint16_t)rx.Data[4] << 8) | rx.Data[5]);
            if (v10 > 0)
            {
                battery_mv = (uint16_t)((int32_t)v10 * 100);   /* 12.0V -> 12000mV */
            }
        }
    }
}

/*========================= 串口接收 ===============================
 * 每收到一个字节就进中断塞进环形缓冲, 主循环再慢慢组帧。
 * 用中断而不是轮询是必须的: STM32F1 的 USART 没有 RX FIFO, 只有一个字节的
 * 数据寄存器, 主循环一卡(比如发遥测)就会 ORE 丢字节。
 *=================================================================*/
static void Port_Isr(uart_port_t *p)
{
    if ((p->usart->SR & (USART_FLAG_RXNE | USART_FLAG_ORE)) != 0)
    {
        uint8_t value = (uint8_t)p->usart->DR;   /* 读 DR 同时清 RXNE 和 ORE */
        uint8_t next = (uint8_t)((p->head + 1) & 0x3F);

        if (next != p->tail)
        {
            p->ring[p->head] = value;
            p->head = next;
        }
        else if (p->overflow != 0xFF)
        {
            p->overflow++;
        }
    }
}

void USART1_IRQHandler(void)
{
    Port_Isr(&ports[PORT_MAIN]);
}

#define USART_TX_TIMEOUT    20000   /* 等 TXE 的最多循环次数, 见 Port_SendByte */

static void Port_SendByte(uart_port_t *p, uint8_t value)
{
    uint32_t timeout = USART_TX_TIMEOUT;

    /* ★ 必须有超时。原来是 `while (TXE == RESET) {}` 死等 —— 万一 USART 出
       状态问题(时钟被关、寄存器异常), 这里会**永久卡住**。而软件看门狗就在
       主循环里, 会跟着一起死: STM32 不再发 CAN 命令, 但**驱动器没有命令超时,
       会一直执行最后一条命令 -> 车自己开走**。
       加了 IWDG 之后最坏也就是被复位, 但明确超时更干净:
       丢一帧遥测无所谓, 卡死是要命的。
       115200 下发一个字节只要约 87us, 20000 次循环 > 5ms, 余量足够。 */
    while ((USART_GetFlagStatus(p->usart, USART_FLAG_TXE) == RESET) &&
           (--timeout != 0))
    {
    }
    if (timeout == 0) return;           /* 发不出去就放弃这一字节 */
    USART_SendData(p->usart, value);
}

static void Usart_Config(USART_TypeDef *usart, uint32_t baud)
{
    USART_InitTypeDef uart;

    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(usart, &uart);

    USART_ITConfig(usart, USART_IT_RXNE, ENABLE);
    USART_Cmd(usart, ENABLE);
}

static void Usart_InitPins(uint16_t tx_pin, uint16_t rx_pin)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = tx_pin;                    /* 复用推挽输出 */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = rx_pin;                    /* 上拉输入 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
}

/* USART1: PA9(TX) / PA10(RX)  ->  主上位机 CH340 / RK3588 */
static void USART1_Init(void)
{
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    Usart_InitPins(GPIO_Pin_9, GPIO_Pin_10);
    Usart_Config(USART1, MAIN_BAUD);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    ports[PORT_MAIN].id = PORT_MAIN;
    ports[PORT_MAIN].usart = USART1;
}

static void PutI16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)v;
}

static void PutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

/*========================= GPIO 继电器 ============================
 * 两路推挽输出。RELAY_ACTIVE_LOW 决定"开"到底是高电平还是低电平。
 *=================================================================*/
static void Relay_Apply(void)
{
    uint8_t brush_on = (relay_state & 0x01) != 0;
    uint8_t pump_on  = (relay_state & 0x02) != 0;

#if RELAY_ACTIVE_LOW
    brush_on = (uint8_t)(!brush_on);
    pump_on  = (uint8_t)(!pump_on);
#endif

    if (brush_on) GPIO_SetBits(RELAY_PORT, RELAY_BRUSH_PIN);
    else          GPIO_ResetBits(RELAY_PORT, RELAY_BRUSH_PIN);

    if (pump_on) GPIO_SetBits(RELAY_PORT, RELAY_PUMP_PIN);
    else         GPIO_ResetBits(RELAY_PORT, RELAY_PUMP_PIN);
}

static void Relay_Set(uint8_t mask)
{
    relay_state = (uint8_t)(mask & 0x03);
    Relay_Apply();
}

static void Relay_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = RELAY_BRUSH_PIN | RELAY_PUMP_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RELAY_PORT, &gpio);

    Relay_Set(0);       /* 上电先全部断开, 不要一通电就抽水 */
}

/*========================== 状态灯 (PB9) ==========================
 * 见上面 LED_* 那一大段的说明。这里只有两件事:
 *   LED_Init: 配成推挽输出, 并**明确写一次初始电平** —— 不写的话上电瞬间
 *             引脚是浮空的, 灯会先闪一下或者微亮, 看着像故障。
 *   LED_Tick: 到点翻转, 不阻塞。
 *
 * ★ 灯相关的状态和 LED_Apply 都包在 #if LED_ENABLE 里 —— 这个文件里凡是
 *   "平时不编译的那条路"都踩过"改了这边忘了那边"的坑(见 TRIM_ENABLE 的注释),
 *   而 ARMCC 对没被引用的 file-scope 变量/函数会报 177-D。两条路都必须是
 *   0 Error 0 Warning, 否则真警告会被淹掉。
 *=================================================================*/
#if LED_ENABLE
static uint32_t led_last_ms;
static uint8_t  led_on;

static void LED_Apply(uint8_t on)
{
#if LED_ON_LEVEL
    if (on) GPIO_SetBits(LED_PORT, LED_PIN);
    else    GPIO_ResetBits(LED_PORT, LED_PIN);
#else
    if (on) GPIO_ResetBits(LED_PORT, LED_PIN);
    else    GPIO_SetBits(LED_PORT, LED_PIN);
#endif
}
#endif  /* LED_ENABLE */

static void LED_Init(void)
{
#if LED_ENABLE
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &gpio);

    led_on = 0;
    led_last_ms = g_tick_ms;
    LED_Apply(0);       /* 先明确灭掉 */
#endif
}

/* 在 main 循环里每圈调一次。找不到到点条件就直接返回, 代价可以忽略。 */
static void LED_Tick(void)
{
#if LED_ENABLE
    if ((g_tick_ms - led_last_ms) >= LED_BLINK_MS)
    {
        led_last_ms = g_tick_ms;
        led_on = (uint8_t)(led_on ? 0 : 1);
        LED_Apply(led_on);
    }
#endif
}

/*======================== 命令帧 (11字节) ==========================
 * 帧头 + 定长 + 帧尾 + BCC。校验失败时整体左移一字节重新同步,
 * 这样即使中间丢了字节也能自己找回来。
 *=================================================================*/
static float ClampF(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t Cmd_Verify(const uint8_t *f)
{
    uint8_t i;
    uint8_t bcc = 0;

    if (f[0] != CMD_HEAD) return 0;
    if (f[CMD_FRAME_SIZE - 1] != TEL_TAIL) return 0;
    for (i = 0; i < CMD_FRAME_SIZE - 2; i++)
    {
        bcc ^= f[i];
    }
    return (bcc == f[CMD_FRAME_SIZE - 2]);
}

/*========================= 命令帧应用 =============================
 * 只有一个上位机, 没有控制权仲裁 —— 收到合法帧就认。
 * 继电器是"锁存式开关", 任何时候都可以操作。
 *
 * ★ 进来就记 last_frame_ms: 能走到这里的帧都过了长度+帧尾+BCC 校验, 足以证明
 *   上位机还活着。但**不**动 last_cmd_ms —— 那个只认速度帧, 否则发一堆灯带帧
 *   就能把运动看门狗喂饱, 车会一直跑下去。见主循环里的看门狗。
 *=================================================================*/
static void Cmd_Apply(uint8_t port_id, const uint8_t *f)
{
    int16_t vx_mm;
    int16_t wz_mrad;

    last_frame_ms = g_tick_ms;

    /* ---- 功能帧: f[1] 是功能码, 不是速度帧 ---- */
    if (f[1] == FUNC_RELAY)
    {
        Relay_Set(f[2]);            /* f[2] = 掩码 bit0 滚刷 bit1 水泵 */
        if (ports[port_id].ok_count != 0xFF) ports[port_id].ok_count++;
        return;
    }

    /* ---- 协议里的其它功能帧: 本工程没实现, 必须忽略 ----
     * 这些帧和速度帧同为 11 字节、同头同尾, 靠 f[1]/f[2] 区分
     * (docs/chassis-serial-protocol.md 第 6 节):
     *     f[1]=0x04             灯带       f[2]=EN, f[3..5]=RGB
     *     f[1]=1, f[2]=0xA0/0xA1 回充开关
     *     f[1]=0, f[2]=0xB0/0xB1 底盘安全防护
     * ★ 绝对不能当速度帧解析 —— 那样 f[3..8] 会被误读成 vx/wz, 车会乱窜。
     *   正常速度帧的 f[1] 只可能是 AutoRecharge(0/1), 所以 f[1]>1 一律不是速度帧。
     */
    if ((f[1] > 1) ||
        ((f[1] == 1) && ((f[2] == 0xA0) || (f[2] == 0xA1))) ||
        ((f[1] == 0) && ((f[2] == 0xB0) || (f[2] == 0xB1))))
    {
        if (ports[port_id].ok_count != 0xFF) ports[port_id].ok_count++;
        return;
    }

    /* ---- 速度帧 ---- */
    /* f[1]=AutoRecharge  f[2]=SecurityPLY  f[5..6]=vy (差速车忽略) */
    vx_mm = (int16_t)(((uint16_t)f[3] << 8) | f[4]);
    wz_mrad = (int16_t)(((uint16_t)f[7] << 8) | f[8]);
    if (ports[port_id].ok_count != 0xFF) ports[port_id].ok_count++;

    if (port_id != PORT_MAIN)
    {
        return;                     /* 只认 PA9/PA10, 别的口一律不理 */
    }

    target_vx = ClampF((float)vx_mm / 1000.0f, -MAX_LIN_SPEED, MAX_LIN_SPEED);
    target_wz = ClampF((float)wz_mrad / 1000.0f, -MAX_YAW_RATE, MAX_YAW_RATE);

    /* ★ 滚刷和水泵都**不**自动合闸。
       两个继电器都是"外设开关", 只能由上位机显式发 f[1]=0x05 帧
       (f[2] = 掩码: bit0 滚刷 bit1 水泵)。

       ★ 历史上这里会自动合上 PB0, 理由是"不合闸电机就没电" —— 但那个"电机"
         指的是**轮毂驱动电机**, 而继电器根本不管它(驱动器独立常电)。理由本身
         就是错的, 于是实际效果变成了"一给运动命令扫地滚刷就转", 一个没人明确
         决定过的副作用。**已改成显式控制。**

       ⚠ 后果要说清楚: 厂商 ROS 节点不认识这条自定义帧, 而且它独占串口 ——
         所以在 ROS 栈跑着的时候**没法开关滚刷**, 需要另加一条控制通路。
         台架上可以先用 tools/decode_diag.py --relay 直接开关。
         详见 README 的继电器一节。 */

    last_cmd_ms = g_tick_ms;
    ever_linked = 1;
    failsafe_latched = 0;
}

static void Port_Feed(uart_port_t *p, uint8_t value)
{
    uint8_t i;

    if (p->len >= CMD_FRAME_SIZE)
    {
        /* 上一帧没通过校验, 左移一字节用滑动窗口重新找帧头 */
        for (i = 1; i < CMD_FRAME_SIZE; i++)
        {
            p->frame[i - 1] = p->frame[i];
        }
        p->len = CMD_FRAME_SIZE - 1;
    }

    p->frame[p->len++] = value;

    if (p->len == CMD_FRAME_SIZE)
    {
        if (Cmd_Verify(p->frame))
        {
            Cmd_Apply(p->id, p->frame);
            p->len = 0;
        }
        else if (p->bad_count != 0xFF)
        {
            p->bad_count++;
        }
        /* 校验失败时保留数据, 下次进来左移重试 */
    }
}

static void Port_ProcessCommands(uart_port_t *p)
{
    while (p->tail != p->head)
    {
        uint8_t value = p->ring[p->tail];
        p->tail = (uint8_t)((p->tail + 1) & 0x3F);
        Port_Feed(p, value);
    }
}

/*========================= 轮速计 ==================================
 * 把两个电机的实际转速按接线校准成"左轮/右轮物理转速",
 * 再正解出车体速度 (vx, wz) 供上报。
 *=================================================================*/
static void Wheel_Update(void)
{
    int16_t a = m1_rpm;
    int16_t b = m2_rpm;
    uint8_t f1, f2;
    float v_left;
    float v_right;

    /* 驱动器掉线时实际转速会一直停在最后一个值, 必须当成 0, 否则会飘。
       同时把"新鲜度"单独记下来 —— 外环积分必须知道这个数是真值还是残值,
       拿残值积分会把修正量一路喂到限幅。 */
    f1 = ((g_tick_ms - m1_rpm_ms) > RPM_STALE_MS) ? 0 : 1;
    f2 = ((g_tick_ms - m2_rpm_ms) > RPM_STALE_MS) ? 0 : 1;
    if (!f1) a = 0;
    if (!f2) b = 0;

    /* 驱动器报的是"命令方向"上的转速, 按安装校准换算成物理方向 */
    if (MOTOR1_INVERT) a = (int16_t)(-a);
    if (MOTOR2_INVERT) b = (int16_t)(-b);

    if (MOTOR1_IS_LEFT)
    {
        wheel_left_rpm = a;
        wheel_right_rpm = b;
        wheel_left_fresh  = f1;
        wheel_right_fresh = f2;
    }
    else
    {
        wheel_left_rpm = b;
        wheel_right_rpm = a;
        wheel_left_fresh  = f2;
        wheel_right_fresh = f1;
    }

    /* 正向运动学: 左右轮线速度 -> 车体速度 (协议里上报的就是这个) */
    v_left  = (float)wheel_left_rpm  * WHEEL_CIRC_M / 60.0f;
    v_right = (float)wheel_right_rpm * WHEEL_CIRC_M / 60.0f;
    body_vx = (v_left + v_right) * 0.5f;
    body_wz = (v_right - v_left) / TRACK_WIDTH_M;
}

/* ★★ 从这里到 Trim_Apply 结束, 整段是**速度模式专用**(给驱动器内部速度环打补丁)。
   力矩模式下这段不编译 —— 见上面 statics 处的说明。 */
#if DRIVE_MODE_SEL == DRIVE_MODE_SPEED

/*========================= 起步助推判决 ============================
 * 两轮**共享**的一次判决, 返回这一周期要叠加的助推量 RPM。
 * 为什么必须共享、不能每轮各判, 见上面 KICK_* 说明里那一段。
 *
 * 判据(绝对转速 + 迟滞):
 *   任一"有效轮" |实际| < KICK_STILL_RPM    -> 给
 *   两个"有效轮" |实际| > KICK_MOVING_RPM   -> 撤
 *   其余(在迟滞带里 / 反馈不新鲜)           -> 保持上一状态
 * 所谓"有效轮" = |名义值| >= TRIM_DEADBAND_RPM 的那一轮。原地转时两轮名义值
 * 大小一样; 混合了 vx 的转弯里两轮可能一大一小, 只要**小的那个**卡住就得给。
 *
 * ★ 反馈不新鲜时**倾向于不撤**(保持当前状态): 没有观测就不敢做结论。
 *   注意 Drive_Ready() 已经保证走到这里时两路回码都是新鲜的, 这条只是兜底。
 *
 * ★ 目标全归零 -> 状态全清, 包括"卡住过"的锁存标志。这样"停一下再走"能
 *   重新获得完整的加力过程, 而停在地上的车不会带着历史。
 *
 * ★ 助推量怎么定(见 KICK_BASE_RPM / KICK_STEP_*):
 *     起始量 = clamp(KICK_BASE_RPM - 最小有效|名义值|, 0, KICK_MAX_RPM)
 *     还不动 -> 每 KICK_STEP_MS 再加 KICK_STEP_RPM, 到 KICK_MAX_RPM 封顶
 *   用**最小**的那个名义值算, 因为要救的是"命令最小、最容易卡住"的那个轮子。
 *
 * ★ 助推对 (vx, wz) 的影响 —— 这一点很容易想错, 必须写清楚:
 *   助推是**按各自名义方向**加的, 所以影响取决于两轮名义值**同号还是异号**:
 *     · 原地转(wz≠0, vx=0): 两轮名义值**异号** -> 助推把两轮往相反方向推 ->
 *       差分 D = ω_r - ω_l 增加 2×助推 -> **偏航被抬高**,
 *       Δwz = 助推/K (K = 43.26 扫地机 / 27.06 洗地机, 见"车型选择")。
 *       这就是"转向太敏感"的来源, 也是第四版要在 |wz|≥15/K 上把助推压到 0
 *       的原因(扫地机 0.35 / 洗地机 0.55 rad/s)。
 *     · 边走边转(vx 足够大, 两轮名义值**同号**): 助推给两轮加的是**同一个
 *       方向** -> 差分**完全不变** -> **偏航一点没变**, 只是车临时多往前冲了
 *       助推那么多。此时 wz 是诚实的, 不需要担心"转向变敏感"。
 *       (比如扫地机 vx=0.30 + wz=0.50: 左轮 6.3 RPM、右轮 49.5 RPM, 助推 8.7
 *        会让两轮都 +8.7, wz 不变, 只有 vx 短暂偏高 —— 而 vx 偏高正是我们要的
 *        "破静摩擦", 外环积分随后会把两轮一起收回来。)
 *=================================================================*/
static float Boost_Decide(float nom_l, float nom_r,
                          float act_l, float act_r,
                          uint8_t fresh_l, uint8_t fresh_r, float dt)
{
#if TRIM_ENABLE
    float mag_l = fabsf(nom_l);
    float mag_r = fabsf(nom_r);
    float smallest = 0.0f;      /* 最小的有效名义值 —— 决定起始助推量 */
    uint8_t any_valid = 0;
    uint8_t give = 0;           /* 有轮子还静止 */
    uint8_t all_moving = 1;     /* 所有有效轮都确认转起来了 */
    uint8_t still_all = 1;      /* 所有有效轮都还静止(决定要不要继续加力) */
    uint16_t dms;

    /*---- 全零: 状态清干净, 不留历史 ----*/
    if ((mag_l < TRIM_DEADBAND_RPM) && (mag_r < TRIM_DEADBAND_RPM))
    {
        boost_on      = 0;
        boost_amt     = 0.0f;
        boost_ms      = 0;
        boost_step_ms = 0;
        boost_cool_ms = 0;      /* 上层显式重下命令 -> "歇"也清掉, 重新给足机会 */
        boost_stuck   = 0;      /* 目标归零 -> 清掉"卡住过"的锁存标志 */
        return 0.0f;
    }

    if (mag_l >= TRIM_DEADBAND_RPM)
    {
        any_valid = 1;
        smallest  = mag_l;
        if (fresh_l)
        {
            float a = fabsf(act_l);
            if (a < KICK_STILL_RPM)   give = 1;
            if (a <= KICK_MOVING_RPM) all_moving = 0;
            if (a >= KICK_STILL_RPM)  still_all = 0;
        }
        else
        {
            all_moving = 0;     /* 没观测 -> 不撤 */
            still_all  = 0;     /* 没观测 -> 也不加力 */
        }
    }
    if (mag_r >= TRIM_DEADBAND_RPM)
    {
        any_valid = 1;
        if ((mag_r < smallest) || (smallest <= 0.0f)) smallest = mag_r;
        if (fresh_r)
        {
            float a = fabsf(act_r);
            if (a < KICK_STILL_RPM)   give = 1;
            if (a <= KICK_MOVING_RPM) all_moving = 0;
            if (a >= KICK_STILL_RPM)  still_all = 0;
        }
        else
        {
            all_moving = 0;
            still_all  = 0;
        }
    }

    if (!any_valid)
    {
        boost_on      = 0;      /* 理论上到不了这里, 兜底 */
        boost_amt     = 0.0f;
        boost_ms      = 0;
        boost_step_ms = 0;
        boost_cool_ms = 0;
        return 0.0f;
    }

    /*---- 状态机: 给 / 撤 / 保持 ----*/
    if (give)
    {
        if (!boost_on)
        {
            /* 刚开: 按"补到起步门槛"算起始量; 名义值已经够大就是 0 ——
               正常行驶(直行 0.30 m/s 名义 27.9 RPM)从这里开始就完全没有助推,
               旧版那个固定 +12 造成的起步过冲随之消失。 */
            boost_on      = 1;
            boost_amt     = KICK_BASE_RPM - smallest;
            if (boost_amt < 0.0f)          boost_amt = 0.0f;
            if (boost_amt > KICK_MAX_RPM)  boost_amt = KICK_MAX_RPM;
            boost_ms      = 0;
            boost_step_ms = 0;
        }
    }
    else if (all_moving)
    {
        boost_on      = 0;      /* 转起来了 -> 立刻撤 */
        boost_amt     = 0.0f;
        boost_ms      = 0;
        boost_step_ms = 0;
        boost_cool_ms = 0;
    }
    /* 其余情况保持上一状态(迟滞带防抖) */

    if (!boost_on)
    {
        return 0.0f;
    }

    /* 计时。dt 极小(卡顿后立刻恢复)时至少算 1ms, 否则计时永远不动,
       防卡死保护就失效了。 */
    dms = (uint16_t)(dt * 1000.0f + 0.5f);
    if (dms == 0) dms = 1;
    boost_ms = (uint16_t)(boost_ms + dms);

    /*---- 防卡死"歇一会儿"中: 这一周期不给力 ----*/
    if (boost_cool_ms > 0)
    {
        boost_cool_ms = (boost_cool_ms > dms) ? (uint16_t)(boost_cool_ms - dms) : 0;
        return 0.0f;
    }

    /*---- 逐级加力: 只在"都还静止"时加 ----
       已经在迟滞带里(有一个轮子转起来一点了)就保持当前量, 不加也不撤。 */
    if (still_all)
    {
        if ((boost_amt >= KICK_MAX_RPM) && (boost_ms >= KICK_MAX_MS))
        {
            /* 防卡死保护: 已经顶到上限 push 满 KICK_MAX_MS 还一直不动, 说明
               轮子被东西顶住了 —— 再给下去要烤电机和驱动器。歇 KICK_COOL_MS
               再接着试(★ 不是永久放弃, 理由见 KICK_COOL_MS 上面那段),
               并**锁存** flags bit7 让上位机看得见。 */
            boost_stuck   = 1;
            boost_ms      = 0;
            boost_step_ms = 0;
            boost_cool_ms = KICK_COOL_MS;
            return 0.0f;
        }

        boost_step_ms = (uint16_t)(boost_step_ms + dms);
        if ((boost_step_ms >= KICK_STEP_MS) && (boost_amt < KICK_MAX_RPM))
        {
            boost_step_ms = 0;
            boost_amt += KICK_STEP_RPM;
            if (boost_amt > KICK_MAX_RPM) boost_amt = KICK_MAX_RPM;
        }
    }
    else
    {
        boost_step_ms = 0;      /* 不是全静止就不加力 */
    }

    return boost_amt;
#else
    (void)nom_l; (void)nom_r; (void)act_l; (void)act_r;
    (void)fresh_l; (void)fresh_r; (void)dt;
    boost_stuck = 0;        /* 诊断帧读它, 所以这条路径也必须赋值 */
    /* ★ A/B 路径里助推整段不编译, 这几个状态没有任何读者。显式吃掉,
       否则 TRIM_ENABLE=0 的构建会多几条 550-D 警告(不是错误, 但会遮住
       别的真警告)。 */
    (void)boost_on;
    (void)boost_amt;
    (void)boost_ms;
    (void)boost_step_ms;
    (void)boost_cool_ms;
    return 0.0f;
#endif
}

/*========================= 轮速外环修正 ============================
 * 按实际轮速修目标轮速, 返回真正要发给驱动器的转速。
 * 两部分: 慢积分修稳态偏差 + 助推破静摩擦。
 * 为什么这么分、第一版错在哪, 见上面 TRIM_* / KICK_* 那一大段。
 *
 * nominal / actual 都在物理轮速域(已按接线校准), 可以直接相减。
 * boost 由 Boost_Decide() 在 Drive_Apply 里**算一次给两轮共享**(理由见那里),
 * 本函数只负责"按名义方向把它加上去"。
 *
 *★ 输出保证: 落在 [1-TRIM_DOWN_K, 1+TRIM_UP_K] 倍名义值之内(助推另加),
 *  且**方向永远和名义值一致** —— 修正不可能把车修成倒着走。
 *=================================================================*/
static float Trim_Apply(wheel_trim_t *w, float nominal, float actual,
                        uint8_t fresh, float dt, float boost)
{
    float out;

#if TRIM_ENABLE
    float mag;
    float up;
    float dn;

    mag = fabsf(nominal);

    /*---- 不该动: 修正清零, 不留任何历史 ----
       (助推/计时的清零在 Boost_Decide 里, 因为它是共享的) */
    if (mag < TRIM_DEADBAND_RPM)
    {
        w->trim = 0.0f;
        return 0.0f;
    }

    /*---- 慢积分: 只修稳态偏差 ----
       ★ 只在**线性带内**积分。实测: 起步时轮子没转, 误差 20~28 RPM,
         积分会一路顶到限幅, 然后和驱动器的 PI 组成 0.25Hz 极限环
         (`目标-理论` 在 ±14 RPM 之间来回摆, 和速度振荡频率完全吻合)。
         误差超出这个带说明轮子"还没跟上", 那是助推和驱动器该管的事,
         外环不该在控制不了的暂态里攒积分。 */
    if (fresh)
    {
        float e = nominal - actual;
        if ((e < TRIM_LIN_BAND_RPM) && (e > -TRIM_LIN_BAND_RPM))
        {
            w->trim += TRIM_KI * e * dt;
        }
    }

    /*---- 限幅: 按名义值的比例, 不用绝对值 ----
       反向最多 -TRIM_DOWN_K 倍名义值, 所以指令的正负号永远不变。
       第一版用的是绝对 ±30 RPM, 在名义值只有 13 RPM 的原地转上把指令
       顶成了 13-30 = -17, 轮子反转、车停住 —— 比例限幅从机制上消除它。 */
    up = mag * TRIM_UP_K;
    dn = mag * TRIM_DOWN_K;
    if (up > TRIM_MAX_RPM) up = TRIM_MAX_RPM;
    if (dn > TRIM_MAX_RPM) dn = TRIM_MAX_RPM;
    if (w->trim >  up) w->trim =  up;
    if (w->trim < -dn) w->trim = -dn;

    out = nominal + w->trim;

    /* 助推按名义方向叠加 —— boost 是 Boost_Decide 给的两轮共享值, 这里无脑加,
       不再各自判决(每轮各判会让两轮指令差 12 RPM, 起步车会歪) */
    if (boost > 0.0f)
    {
        out += (nominal > 0.0f) ? boost : -boost;
    }
#else
    (void)actual;
    (void)fresh;
    (void)dt;
    (void)boost;            /* A/B 路径里助推整段不编译, 这里显式吃掉免警告 */
    w->trim = 0.0f;
    out = nominal;
#endif

    out = ClampF(out, -(float)MAX_RPM, (float)MAX_RPM);

    /* ★ 最后一道保险: 修正 + 助推绝不允许把指令的方向改掉。
       名义为正就保证输出 >= 0, 名义为负就保证输出 <= 0。
       即使上面的限幅哪天被人改错了, 这一条也能兜住。 */
    if ((nominal > 0.0f) && (out < 0.0f)) out = 0.0f;
    if ((nominal < 0.0f) && (out > 0.0f)) out = 0.0f;

    return out;
}

#endif  /* DRIVE_MODE_SEL == DRIVE_MODE_SPEED  (助推 + 轮速外环修正) */

/*===================== 驱动器就绪判断 ==============================
 * 差速车只有一个轮子出力就等于原地打转, 所以**没确认两个驱动器都活着之前,
 * 只发刹车、绝不发速度** —— 直行命令变成左转就是这么来的(见 drv_seen_mask)。
 *
 * 判据是两条, 都要满足:
 *   1. 两个驱动器**各自回过码**(证明 CAN 通、驱动器活着);
 *   2. 两路回码**现在都还新鲜** —— 用一次就够不算数, 半路掉线的驱动器必须
 *      立刻让车停下来, 否则又是一次"单轮驱动 = 原地打转"。
 *
 * 为什么不会死锁: 等待期间 Drive_Apply 每 20ms 发的是**刹车帧**, 而驱动器对
 * 刹车帧同样会回码, 所以回码一定会来。万一 CAN 真断了就一直刹车 ——
 * 这正是应该的结果(驱动器都没通, 本来就不该动)。
 *=================================================================*/
static uint8_t Drive_Ready(void)
{
    if ((drv_seen_mask & 0x03) != 0x03) return 0;             /* 有一路从没回过 */
    if ((g_tick_ms - m1_rpm_ms) > RPM_STALE_MS) return 0;     /* 1号现在不回了 */
    if ((g_tick_ms - m2_rpm_ms) > RPM_STALE_MS) return 0;     /* 2号现在不回了 */
    return 1;
}

/*========================= 上电自检点动 ============================
 * 见上面 SELFTEST_* 那一大段(边界、为什么不能做成精确 10cm)。
 *
 * 状态机故意做成**非阻塞**的、挂在主循环上, 原因有三个:
 *   1. 主循环照常喂 IWDG、照常发遥测 —— 操作员能在 /odom 上看到这次点动,
 *      比"阻塞式 Delay 里偷偷动一下"可验证得多;
 *   2. 上位机一插话就能立刻让位(每圈都查 ever_linked);
 *   3. 复用 Drive_Apply 的整套安全(就绪联锁、目标归零就发刹车)。
 *
 * 点动期间本函数**自己当指挥官**: 保持 target 并把 last_cmd_ms 刷新, 否则
 * 800ms 运动看门狗会把这次点动中途掐掉(SELFTEST_MS 目前 300ms 掐不到, 但
 * 一旦有人把它调到 800 以上就会莫名其妙走一半 —— 所以这里显式喂狗)。
 *=================================================================*/
#if SELFTEST_ENABLE
static uint8_t  st_state;           /* 见下面的 ST_* */
static uint32_t st_t0_ms;           /* 进入当前状态的时刻 */
static int16_t  st_peak_l;          /* 点动期间左轮见过的最大 |实际转速| */
static int16_t  st_peak_r;
#endif
/* ★ 这个放在 #if 外面: 诊断帧要读它(bit7)。整段关掉时它恒为 0 = "没查出问题",
   语义上正好是"没做自检所以没发现问题", 不会误导。 */
static uint8_t  selftest_no_move;   /* 点动期间两轮都没转 -> 自检没过 */

#define ST_DELAY    0       /* 上电先等 SELFTEST_DELAY_MS */
#define ST_READY    1       /* 等两个驱动器回码(最多 SELFTEST_READY_MS) */
#define ST_RUN      2       /* 正在点动 */
#define ST_DONE     3       /* 结束(或放弃), 再也不动 */

static void SelfTest_Init(void)
{
#if SELFTEST_ENABLE
    st_state  = ST_DELAY;
    st_t0_ms  = g_tick_ms;
    st_peak_l = 0;
    st_peak_r = 0;

    /* ★ 看门狗复位过就**跳过自检**。
       IWDG 复位意味着"刚才主循环卡死过", 而刚才车是在跑的 —— 可能正贴着墙、
       正停在人旁边。这种复位之后再自己往前窜 7~16cm 是纯粹的风险, 没有任何
       自检价值(自检是给"刚上电"用的)。注意本函数是在 main 里读完 RCC_CSR
       之后才调的, reset_by_iwdg 已经是最终值。 */
    if (reset_by_iwdg)
    {
        st_state = ST_DONE;
    }
#endif
}

/* 挂在主循环里每圈调一次。只在自检还没结束时干活, 结束后开销为 0。 */
static void SelfTest_Tick(void)
{
#if SELFTEST_ENABLE
    uint32_t elapsed;
    int16_t  a;

    if (st_state == ST_DONE) return;

    /* ★ 上位机开始指挥了 -> 立刻让位, 把状态收干净走人。
       绝不自检和主人抢车。注意 Cmd_Apply 已经把 target 写成上位机的值了,
       这里只负责不再覆盖它。 */
    if (ever_linked)
    {
        st_state = ST_DONE;
        return;
    }

    elapsed = g_tick_ms - st_t0_ms;

    switch (st_state)
    {
    case ST_DELAY:
        /* ★ 上电后先等这 3 秒: 让旁边的人有时间躲开, 也给驱动器上电时间。
           这期间**什么都不做** -> target 保持 0 -> Drive_Apply 一直发刹车帧。 */
        if (elapsed >= SELFTEST_DELAY_MS)
        {
            st_state = ST_READY;
            st_t0_ms = g_tick_ms;
        }
        break;

    case ST_READY:
        if (Drive_Ready())
        {
            st_state = ST_RUN;
            st_t0_ms = g_tick_ms;
            st_peak_l = 0;
            st_peak_r = 0;
        }
        else if (elapsed >= SELFTEST_READY_MS)
        {
            /* 驱动器一直没回码 -> 不做自检。车保持刹车, 什么都不动。
               这时候"为什么不动"由诊断帧回答(flags bit5 + 驱动器故障码)。 */
            st_state = ST_DONE;
        }
        break;

    case ST_RUN:
        /* 记录两轮见过的峰值, 用来判断"到底转没转" —— 只看结束那一拍的
           瞬时值是碰运气(可能正好落在 0 附近)。 */
        a = wheel_left_rpm;  if (a < 0) a = (int16_t)(-a);
        if (a > st_peak_l) st_peak_l = a;
        a = wheel_right_rpm; if (a < 0) a = (int16_t)(-a);
        if (a > st_peak_r) st_peak_r = a;

        /* 硬上限: 不管因为什么, 到点必停(正常 300ms 就结束了) */
        if (elapsed >= SELFTEST_ABORT_MS)
        {
            target_vx = 0.0f;
            target_wz = 0.0f;
            st_state = ST_DONE;
            break;
        }

        if (elapsed >= SELFTEST_MS)
        {
            /* 点动结束: 目标交回 0, 下一拍 Drive_Apply 就发刹车帧。
               起点和终点都归零是刻意的 —— 自检不该给系统留下任何状态。 */
            target_vx = 0.0f;
            target_wz = 0.0f;

            if ((st_peak_l == 0) && (st_peak_r == 0))
            {
                selftest_no_move = 1;   /* 一点没转: 驱动/CAN/电机这条链有问题 */
            }
            st_state = ST_DONE;
            break;
        }

        /* 点动中 */
        target_vx = SELFTEST_VX;
        target_wz = 0.0f;
        last_cmd_ms = g_tick_ms;        /* 喂运动看门狗, 见函数头说明 */
        break;

    default:
        st_state = ST_DONE;
        break;
    }
#else
    /* A/B 路径: 整段不编译。函数体留空, 但被 main 引用, 所以不会报未引用。 */
    (void)0;
#endif
}

/*========================= 力矩模式: 我们自己的速度环 ==================
 * 只在 DRIVE_MODE_SEL == DRIVE_MODE_TORQUE 时编译。
 *
 * 输入: 目标轮速(RPM) + 实际轮速(RPM), 输出: 扭矩电流指令(A x10)。
 * 也就是把驱动器内部那个(有极限环的)速度环整个换掉。
 *
 *★★ 参数全是**起始猜测**, 必须在台架上标定 ★★
 *   我没有这台电机+这台车的转矩常数、静摩擦、转动惯量, 所以下面每个数都是
 *   "量级上说得通"的估计, **不是实测值**。第一次上车必须架空、从很小的数开始
 *   往上试, 并把最终值记回 docs/nav2-reference.md。
 *   已知的锚点只有一个: 满量程 ±10.0A(写 ±100), 以及速度模式测出的"起步门槛
 *   约 15 RPM"(那是驱动器电压域的量, 换算不成电流, 只能当个提醒: 静摩擦不小)。
 *
 * 控制律:  误差 = 目标 - 实际
 *          P  = TQ_KP * 误差
 *          I  += TQ_KI * 误差 * dt        (带抗饱和)
 *          静摩擦补偿: 轮子还没转起来时, 输出至少给到 TQ_STICTION_A10
 *          硬限流 ±TQ_MAX_A10
 *          变化率限幅 TQ_SLEW_A10_PER_S (扭矩不能阶跃)
 *
 * 为什么必须单独做"静摩擦补偿": 速度模式下静摩擦是**驱动器**帮忙顶过去的
 * (它的环会一直加电压直到轮子动)。扭矩模式下没有这个东西了 —— 纯 PI 要等
 * 积分自己爬到起步力矩, 起步会很慢, 而且在极低速会"给一下就停、停一下再给"
 * 地抖。所以显式给一个起步力矩下限。
 *===================================================================*/
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE

#define TQ_KP               1.2f    /* A x10 / RPM   —— 10 RPM 误差 -> 1.2A */
#define TQ_KI               0.6f    /* A x10 / (RPM·s) */
#define TQ_DEADBAND_RPM     2.0f    /* 误差小于这个就不管(免得在原地抖) */
#define TQ_MAX_A10          80.0f   /* ★ 硬限流 = 8.0A。
                                       ★★ 2026-10-12 首次台架实测后从 3.0A 提到 8.0A ★★
                                       原值 30(3.0A) 的依据只是"先保守", 结果洗地机上
                                       **3A 完全推不动**: 实测电流确实爬到了 2.5A、驱动器
                                       回码模式=力矩、无故障, 但两个轮子一动不动、IMU 加速度
                                       峰峰值只有 0.03 m/s2(独立裁判: 车没窜)。
                                       改这个值的依据不是猜 —— **扫地机在速度模式下正常行驶时
                                       驱动器回码的电流是 7~11A**, 也就是说推这台车本来就需要
                                       这个量级的电流, 3A 只有它的 30%。8.0A 仍在驱动器满量程
                                       10.0A 之内。
                                       ⚠ 若实测发现起步只需 3~5A, 请把这里调回去: 限流越低
                                         越安全, 8A 只代表"别低到推不动"。 */
#define TQ_STICTION_A10     25.0f   /* 起步力矩下限的**起始值** = 2.5A。
                                       ★ 原来 0.8A 只是"不知道要多少"的保守值, 但实测证明
                                         它离起步门槛太远(爬到 3A 都没动)。起始值放到 2.5A,
                                         让爬升起点离有用区间近一点。
                                       ★ 它仍然会自己往上爬, 见 TQ_FLOOR_* */
#define TQ_STALL_RPM        1.5f    /* 认为"轮子没转"的阈值 */
#define TQ_SLEW_A10_PER_S   120.0f  /* 指令变化率限幅 -> 0.8A 大约 7ms 加到位 */
#define TQ_STALL_TGT_RPM    3.0f    /* 目标大于这个才做堵转判断 */
#define TQ_STALL_MS         3000    /* **下限爬满之后**还不转, 再顶这么久就收手。
                                       原 2000ms; 抬到 3000ms 是为了给"起步力矩够不够"
                                       这种实测留出观察窗口 —— 下限起始值 2.5A、每步 1A、
                                       每 200ms 一步, 大约 1.1s 才爬到 8A, 加上这 3s,
                                       一次测试最长约 4s 就能给出结论。 */
#define TQ_RUNAWAY_MARGIN_RPM  25.0f/* 实际超过目标这么多 = 可疑 */
#define TQ_RUNAWAY_MS       800     /* 可疑状态持续这么久 = 失控, 锁存 */
#define TQ_ABS_MAX_RPM      60.0f   /* 绝对上限, 超了立刻锁存(不看目标) */

/* 起步力矩下限的爬升 —— ★ 这是从"助推"那里学来的教训, 别重复踩:
 * 我们**不知道**这台车起步需要多少扭矩(静摩擦、载重、坡道都影响它)。如果下限
 * 固定成一个小值, 那么"扭矩不够 -> 轮子不动 -> 堵转保护锁存"会被误读成
 * "力矩模式不工作"; 如果固定成一个大值, 起步又会窜。
 * 做法: 从 TQ_STICTION_A10 起, 只要"有目标、却没转起来"就每 TQ_FLOOR_STEP_MS
 * 往上加 TQ_FLOOR_STEP_A10, 一直爬到 TQ_MAX_A10 为止; 一旦转起来就立刻收回起始值。
 * 这样它**自己找到**起步需要的力矩, 而且爬升过程能从驱动器回码的电流栏直接看到。
 * ★ 两轮**共享**一个下限(不是每轮各一份): 和助推同样的理由 —— 各自爬升会让两轮
 *   指令差出十几, 起步时车会歪一下。 */
#define TQ_FLOOR_STEP_A10   10.0f   /* 每 TQ_FLOOR_STEP_MS 往上加 1.0A
                                       (原 0.4A/250ms —— 实测嫌太慢: 一步太小时,
                                        在"有用区间"附近要磨蹭很久, 台架上一轮测试里
                                        看不到车轮起步的那一刻) */
#define TQ_FLOOR_STEP_MS    200

typedef struct
{
    float i10;      /* 上一次输出, 用于变化率限幅 */
    float integ;    /* 积分项 (A x10) */
} wheel_torque_t;

static wheel_torque_t tq_w[2];      /* [0]=左 [1]=右, 和 trim_w 同序 */
static float    tq_floor;           /* 当前起步力矩下限 (A x10), 两轮共享 */
static uint16_t tq_floor_ms;        /* 距上次抬下限多久 */
static uint16_t tq_guard_ms;        /* 超速可疑已持续多久 */
static uint16_t tq_stall_ms;        /* 下限爬满之后仍不转已持续多久 */
static uint8_t  torque_fault;       /* 锁存: 1 = 触发过保护, 只刹车 */
static uint16_t torque_stop_ms;     /* 目标为 0 已持续多久(用于解除锁存) */
#define TQ_REARM_STOP_MS    500     /* 停稳这么久才允许重新起步(解除锁存) */

/* 起步力矩下限的爬升(两轮共享一次判决)。必须在 Torque_Apply 之前调。 */
static void Torque_Floor_Update(float tgt_l, float act_l, uint8_t fresh_l,
                                float tgt_r, float act_r, uint8_t fresh_r, float dt)
{
    uint8_t need;
    float   amax;

    if (!fresh_l || !fresh_r) return;   /* 数据不可信就不动下限 */

    need = (uint8_t)((fabsf(tgt_l) > TQ_STALL_TGT_RPM) ||
                     (fabsf(tgt_r) > TQ_STALL_TGT_RPM));

    amax = fabsf(act_l);
    if (fabsf(act_r) > amax) amax = fabsf(act_r);

    if (need && (amax < TQ_STALL_RPM))
    {
        tq_floor_ms += (uint16_t)(dt * 1000.0f);
        while (tq_floor_ms >= TQ_FLOOR_STEP_MS)
        {
            tq_floor_ms -= TQ_FLOOR_STEP_MS;
            tq_floor += TQ_FLOOR_STEP_A10;
            if (tq_floor > TQ_MAX_A10) tq_floor = TQ_MAX_A10;
        }
    }
    else
    {
        /* 转起来了(或没有目标): 立刻收回起始值, 下次起步从头爬 */
        tq_floor_ms = 0;
        tq_floor    = TQ_STICTION_A10;
    }
}

/* 每轮一份的扭矩 PI + 静摩擦补偿。tgt == 0 时返回 0(调用方会发刹车帧)。 */
static float Torque_Apply(wheel_torque_t *w, float tgt, float act, float dt)
{
    float err;
    float out;
    float adir;

    if (tgt == 0.0f)
    {
        w->integ = 0.0f;
        w->i10   = 0.0f;
        return 0.0f;
    }

    err = tgt - act;
    if ((err > -TQ_DEADBAND_RPM) && (err < TQ_DEADBAND_RPM)) err = 0.0f;

    if (err != 0.0f)
    {
        w->integ += TQ_KI * err * dt;
        /* 抗饱和: 积分自己不许超过硬限流 */
        if (w->integ >  TQ_MAX_A10) w->integ =  TQ_MAX_A10;
        if (w->integ < -TQ_MAX_A10) w->integ = -TQ_MAX_A10;
    }

    out = TQ_KP * err + w->integ;

    /* 静摩擦补偿: 只在"还没转起来"时兜底, 用的是**会往上爬的**下限 tq_floor。
       转起来之后交给 PI —— 否则轮子一动就撤、一停又给, 会在极低速来回磕。 */
    if ((act > -TQ_STALL_RPM) && (act < TQ_STALL_RPM) && (err != 0.0f))
    {
        adir = (err > 0.0f) ? 1.0f : -1.0f;
        if ((out * adir) < tq_floor) out = adir * tq_floor;
    }

    out = ClampF(out, -TQ_MAX_A10, TQ_MAX_A10);

    /* 变化率限幅: 扭矩指令不能阶跃(机械冲击 + 电流冲击) */
    {
        float step = TQ_SLEW_A10_PER_S * dt;
        if (out > w->i10 + step) out = w->i10 + step;
        if (out < w->i10 - step) out = w->i10 - step;
    }

    w->i10 = out;
    return out;
}

/* 三道保护。**两轮一起判、一起停** —— 和助推同样的理由: 单轮自己收手会让
 * 差速车原地打转, 而"打转"比"停下"危险得多。
 *
 *   (1) 绝对超速    |实际| > TQ_ABS_MAX_RPM            -> 立刻锁存
 *   (2) 超目标速度  |实际| > |目标| + 余量, 持续 TQ_RUNAWAY_MS -> 锁存
 *                   ★ 这一条同时覆盖"接线反了" —— 接线反了闭环就变成正反馈,
 *                     表现为实际转速朝反方向越来越大, 一定会撞上这条。
 *   (3) 堵转        有目标、|实际| 却一直 ≈ 0, 持续 TQ_STALL_MS -> 锁存
 *                   ★ 这一条挡的是"驱动器根本不认 mode 4"(那就是白给电流发热)、
 *                     轮子被卡住、以及 CAN 没通到电机。
 *
 * 锁存之后只发刹车帧, 直到**目标为 0 且持续 TQ_REARM_STOP_MS** 才重新允许起步。
 * 为什么不一直锁死到断电: 机器人不能因为一次保护就得人工重启。为什么不立刻
 * 自动重试: 立刻重试就成了"抖一下停一下"的循环, 比停着更难看也更危险。
 */
static void Torque_Guard(float tgt_l, float act_l, uint8_t fresh_l,
                         float tgt_r, float act_r, uint8_t fresh_r, float dt)
{
    float tmax = fabsf(tgt_l);
    float amax = fabsf(act_l);
    uint16_t ms = (uint16_t)(dt * 1000.0f);

    /* ★ 反馈不新鲜就不判决 —— 拿陈旧数据判"失控"是错的(会把正常减速误判成失控)。
       正常情况下轮不到这里: Drive_Ready() 已经在更外层拦住这种局面并刹车了
       (RPM_STALE_MS), 这里只是**每轮各自**再兜一道 —— 因为 Drive_Ready 是"两轮
       一起新鲜"的与逻辑, 而 Torque_Guard 的判据本质上是"这一轮的数据可信吗"。 */
    if (!fresh_l || !fresh_r)
    {
        tq_guard_ms = 0;
        tq_stall_ms = 0;
        return;
    }

    if (fabsf(tgt_r) > tmax) tmax = fabsf(tgt_r);
    if (fabsf(act_r) > amax) amax = fabsf(act_r);

    /* 目标为 0: 复位计时并逐步解除锁存(车已经停了, 允许下一次起步) */
    if (tmax <= 0.0f)
    {
        tq_guard_ms = 0;
        tq_stall_ms = 0;
        if (torque_stop_ms < TQ_REARM_STOP_MS) torque_stop_ms += ms;
        if (torque_stop_ms >= TQ_REARM_STOP_MS) torque_fault = 0;
        return;
    }
    torque_stop_ms = 0;

    if (torque_fault) return;       /* 已锁存, 不重复判 */

    if (amax > TQ_ABS_MAX_RPM)      /* (1) */
    {
        torque_fault = 1;
        return;
    }

    if (amax > (tmax + TQ_RUNAWAY_MARGIN_RPM))  /* (2) */
    {
        tq_guard_ms += ms;
        if (tq_guard_ms >= TQ_RUNAWAY_MS) { torque_fault = 1; return; }
    }
    else tq_guard_ms = 0;

    /* (3) 堵转: 有目标、轮子却一直不转, **而且下限已经爬满了**(说明该给的都给了)
            —— 这才是真堵住/驱动器不认 mode 4/轮子被卡住。
           ★ 判据里带 tq_floor >= TQ_MAX_A10 是关键: 只看"两秒没转"会把
             "起步电流不够"误判成堵转, 而那是调参问题不是故障 —— 见 TQ_FLOOR_*。 */
    if ((tmax > TQ_STALL_TGT_RPM) && (amax < TQ_STALL_RPM) && (tq_floor >= TQ_MAX_A10))
    {
        tq_stall_ms += ms;
        if (tq_stall_ms >= TQ_STALL_MS) torque_fault = 1;
    }
    else tq_stall_ms = 0;
}

/* 把扭矩环状态清干净(锁存/未就绪/停车时调) */
static void Torque_Reset(void)
{
    tq_w[TRIM_IDX_LEFT].i10    = 0.0f;
    tq_w[TRIM_IDX_LEFT].integ  = 0.0f;
    tq_w[TRIM_IDX_RIGHT].i10   = 0.0f;
    tq_w[TRIM_IDX_RIGHT].integ = 0.0f;
    tq_floor    = TQ_STICTION_A10;      /* 下限回到起始值, 下次起步重新爬 */
    tq_floor_ms = 0;
}

#endif  /* DRIVE_MODE_SEL == DRIVE_MODE_TORQUE */

/* 诊断帧 bit6 的判据, 两种模式各有一个对应物:
 *   速度模式 = "正在给起步助推"(叠加了 boost_amt RPM)
 *   力矩模式 = "扭矩环正在出力"(|扭矩指令| > 0.5 -> 0.05A)
 * ★ 台架上这一位很好用: 命令了却一直不亮 = 环没在给(或者保护锁存了);
 *   亮了却不动 = 在给但推不动(电流不够/被卡住)。 */
static uint8_t Drive_ActiveFlag(void)
{
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
    return (uint8_t)((fabsf(tq_w[TRIM_IDX_LEFT].i10)  > 0.5f) ||
                     (fabsf(tq_w[TRIM_IDX_RIGHT].i10) > 0.5f));
#else
    return (uint8_t)(boost_on && (boost_amt > 0.5f) && (boost_cool_ms == 0));
#endif
}

/*========================= 差速逆解 ================================
 * 车体 (vx, wz) -> 左右轮目标转速。差速车用不到 vy。
 *   v_left  = vx - wz * b/2
 *   v_right = vx + wz * b/2
 *=================================================================*/
static void Drive_Apply(void)
{
    /* 外环积分用的真实周期。主循环里还夹着 IMU 读取(1~3ms)等开销, 实际周期
       会比 CAN_PERIOD_MS 略大; 用固定值会让 TRIM_KI 的实际含义随负载漂移,
       参数就没法标定了。异常值(第一次调用/计时器回绕/严重卡顿)兜底成标称值。 */
    static uint32_t last_trim_ms = 0;
    float dt = (float)(g_tick_ms - last_trim_ms) * 0.001f;

    float v_left = target_vx - target_wz * (TRACK_WIDTH_M * 0.5f);
    float v_right = target_vx + target_wz * (TRACK_WIDTH_M * 0.5f);
    float rpm_l = v_left / WHEEL_CIRC_M * 60.0f;
    float rpm_r = v_right / WHEEL_CIRC_M * 60.0f;
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
    float   tq_l, tq_r;                 /* 左右轮的扭矩指令 (A x10) */
    int16_t i1, i2;                     /* 转成电机域之后的扭矩指令 */
#else
    float cmd_l;
    float cmd_r;
    float boost;
    int16_t m1;
    int16_t m2;
#endif

    if ((dt <= 0.0f) || (dt > 0.2f)) dt = (float)CAN_PERIOD_MS * 0.001f;
    last_trim_ms = g_tick_ms;      /* 就算下面提前 return 也要更新, 免得恢复时 dt 爆掉 */

    /* ★ 驱动器没都就绪: 只发刹车, 绝不发速度。
       差速车单轮出力 = 原地打转, 而实测真的出现过"直行命令变成向左转"。
       顺便把外环状态清零, 免得带着上一次攒的修正冲出去。 */
    if (!Drive_Ready())
    {
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
        Torque_Reset();     /* 力矩模式: 把我们的积分/输出清零(见 Torque_Reset) */
#else
        trim_w[TRIM_IDX_LEFT].trim  = 0.0f;
        trim_w[TRIM_IDX_RIGHT].trim = 0.0f;
        boost_on      = 0;  /* 停过之后助推状态也清干净 */
        boost_amt     = 0.0f;
        boost_ms      = 0;
        boost_step_ms = 0;
        boost_cool_ms = 0;
#endif
        boost_stuck   = 0;
        target_left_rpm  = 0;
        target_right_rpm = 0;
        CAN1_SendStop(MOTOR1_CAN_ID);
        CAN1_SendStop(MOTOR2_CAN_ID);
        return;
    }

    rpm_l = ClampF(rpm_l, -(float)MAX_RPM, (float)MAX_RPM);
    rpm_r = ClampF(rpm_r, -(float)MAX_RPM, (float)MAX_RPM);

#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
    /*=================== 力矩模式: 我们自己的速度环 ===================
     * 和速度模式最大的区别在**这一层没有第二个人**了: 速度模式下我们发的 RPM
     * 是"请求", 驱动器内部那个环负责去实现; 这里我们直接给电流, 驱动器只当
     * 一个电流源 —— 所以"实现转速"这件事整个落在下面这几行上。
     * 相应地 Trim/Boost **全部不用**: 它们的存在意义是修补驱动器那个环,
     * 现在那个环不在了, 留着就是两个积分环串联, 必然打架。
     *================================================================*/
    Torque_Floor_Update(rpm_l, (float)wheel_left_rpm,  wheel_left_fresh,
                        rpm_r, (float)wheel_right_rpm, wheel_right_fresh, dt);
    Torque_Guard(rpm_l, (float)wheel_left_rpm,  wheel_left_fresh,
                 rpm_r, (float)wheel_right_rpm, wheel_right_fresh, dt);

    /* 保护锁存 或 目标为 0: 一律刹车, 并把自己的状态清干净。
       ★ 用 CAN1_SendStop 而不是"发 0 扭矩" —— 0 扭矩 = 自由滑行, 那不是停车。 */
    if (torque_fault || ((rpm_l == 0.0f) && (rpm_r == 0.0f)))
    {
        Torque_Reset();
        target_left_rpm  = 0;
        target_right_rpm = 0;
        CAN1_SendStop(MOTOR1_CAN_ID);
        CAN1_SendStop(MOTOR2_CAN_ID);
        return;
    }

    tq_l = Torque_Apply(&tq_w[TRIM_IDX_LEFT],  rpm_l, (float)wheel_left_rpm,  dt);
    tq_r = Torque_Apply(&tq_w[TRIM_IDX_RIGHT], rpm_r, (float)wheel_right_rpm, dt);

    /* 物理轮速域 -> 电机域: 和速度模式用**同一套**接线校准, 这样两个模式
       对"哪个电机是左边""哪个要取反"的理解不可能分叉。 */
    if (MOTOR1_IS_LEFT) { i1 = (int16_t)tq_l; i2 = (int16_t)tq_r; }
    else                { i1 = (int16_t)tq_r; i2 = (int16_t)tq_l; }
    if (MOTOR1_INVERT) i1 = (int16_t)(-i1);
    if (MOTOR2_INVERT) i2 = (int16_t)(-i2);

    /* 上报的"目标轮速"是**名义目标**(不含我们的修正) —— 力矩模式下没有
       "目标 RPM 的修正量"这个概念了, 主机拿它和自己下发的比就是 0。 */
    target_left_rpm  = (int16_t)rpm_l;
    target_right_rpm = (int16_t)rpm_r;

    /* 每轮各自决定: 有目标 -> 给扭矩(使能); 目标为 0 -> 给刹车。
       ★ 目标为 0 的轮子发**刹车**而不是"0 扭矩": 原地转时内轮会自由滑行被拖着走,
         发刹车(动态制动)更像速度模式下"内轮被驱动器按在 0"的手感。
         刹车是被动的, 不注入能量, 所以这一步是安全的。 */
    if (rpm_l != 0.0f) CAN1_SendTorque(MOTOR1_IS_LEFT ? MOTOR1_CAN_ID : MOTOR2_CAN_ID,
                                       MOTOR1_IS_LEFT ? i1 : i2);
    else               CAN1_SendStop(MOTOR1_IS_LEFT ? MOTOR1_CAN_ID : MOTOR2_CAN_ID);
    if (rpm_r != 0.0f) CAN1_SendTorque(MOTOR1_IS_LEFT ? MOTOR2_CAN_ID : MOTOR1_CAN_ID,
                                       MOTOR1_IS_LEFT ? i2 : i1);
    else               CAN1_SendStop(MOTOR1_IS_LEFT ? MOTOR2_CAN_ID : MOTOR1_CAN_ID);

#else   /* ================= 速度模式(现状) ================= */

    /* 起步助推: **一次判决, 两轮共用**(见 KICK_* 最后一段)。必须放在两次
       Trim_Apply 之前, 而且只调一次 —— 每轮各判一次会让两轮指令差 12 RPM。 */
    boost = Boost_Decide(rpm_l, rpm_r,
                         (float)wheel_left_rpm, (float)wheel_right_rpm,
                         wheel_left_fresh, wheel_right_fresh, dt);

    /* 外环修正: 用实际轮速去修目标轮速。原地转起步时轮子是真的没转, 这里会把
       指令顶上去直到它真的转; 稳态偏高时又会把指令压回来。
       ★ 目标为 0 时 Trim_Apply 恒定返回 0, 而且它保证输出方向和名义值一致,
         所以外环**不可能**在主机下令停车的时候把车带着走, 也不可能把某个轮子
         修成倒转 —— 这是这套做法敢上车的底线。 */
    cmd_l = Trim_Apply(&trim_w[TRIM_IDX_LEFT], rpm_l,
                       (float)wheel_left_rpm, wheel_left_fresh, dt, boost);
    cmd_r = Trim_Apply(&trim_w[TRIM_IDX_RIGHT], rpm_r,
                       (float)wheel_right_rpm, wheel_right_fresh, dt, boost);

    /* 物理轮速 -> CAN 命令 (反向应用接线校准) */
    if (MOTOR1_IS_LEFT) { m1 = (int16_t)cmd_l; m2 = (int16_t)cmd_r; }
    else                { m1 = (int16_t)cmd_r; m2 = (int16_t)cmd_l; }
    if (MOTOR1_INVERT) m1 = (int16_t)(-m1);
    if (MOTOR2_INVERT) m2 = (int16_t)(-m2);

    /* 上报的是**最终真正发给驱动器**的物理轮速(含外环修正), 不是电机命令,
       免得左右概念混淆。主机拿自己下发的理论目标去减, 就是外环修正量。 */
    target_left_rpm  = (int16_t)cmd_l;
    target_right_rpm = (int16_t)cmd_r;

    /*===================== 走 / 停 两态 =====================
     * 停车就是一条 STOP_CTRL 帧, 每个控制周期重发。
     * 不切模式、不记忆状态、不引用位置 —— 出问题只可能出在电机或接线,
     * 不可能出在这几行逻辑上。这是上一版"位置锁位"失控后刻意保留的简单。
     *=======================================================*/
    if ((m1 != 0) || (m2 != 0))
    {
        CAN1_SendSpeed(MOTOR1_CAN_ID, m1);
        CAN1_SendSpeed(MOTOR2_CAN_ID, m2);
    }
    else
    {
        CAN1_SendStop(MOTOR1_CAN_ID);
        CAN1_SendStop(MOTOR2_CAN_ID);
    }
#endif
}

/*======================= IMU (YbImu) ==============================
 * 按 ODOM_PERIOD_MS 采一次。读失败不清零 —— 保留上一次的值。
 *
 * 还会记录一串诊断状态, 读失败时最多每秒做一次总线检查 + 地址扫描,
 * 结果通过诊断帧上报 —— 否则"读取失败"这四个字什么信息都没有。
 *=================================================================*/
#define IMU_DIAG_PERIOD_MS  1000

static void IMU_Tick(void)
{
    float accel[3];
    float gyro[3];
    uint8_t err;

    err = YbImu_ReadMotion(accel, gyro);

    if (err == YBIMU_ST_OK)
    {
        imu_accel_g[0] = accel[0];
        imu_accel_g[1] = accel[1];
        imu_accel_g[2] = accel[2];
        imu_gyro[0] = gyro[0];
        imu_gyro[1] = gyro[1];
        /* ★ Z 轴方向在这里生效。IMU_GYRO_Z_SIGN 以前是个死宏(只定义没使用),
           现在接回来了 —— 否则万一陀螺仪 Z 装反了, 上位机 EKF 会拿它去
           "纠正"轮速, 越纠越偏, 而且没有任何可调的地方。
           验证方法: 把车从上方看**逆时针**转(左转),
           `ros2 topic echo /imu/data_raw --field angular_velocity`
           的 z 应该是**正的**。反了就把它改成 -1。 */
        imu_gyro[2] = gyro[2] * (float)IMU_GYRO_Z_SIGN;

        /* 静止时模块会把角速度输出归零, 那是正常行为, 不是故障。
           所以"读数是否为零"判断不了陀螺仪死活 —— 这里改成记录
           "自开机以来有没有读到过非零角速度": 车动过之后它还是 0,
           才说明陀螺仪真有问题。诊断帧的 bit3 报的就是这个。 */
        if ((gyro[0] != 0.0f) || (gyro[1] != 0.0f) || (gyro[2] != 0.0f))
        {
            imu_gyro_ok = 1;
        }

        imu_ok = 1;
        imu_status = YBIMU_ST_OK;
        imu_found_addr = YBIMU_I2C_ADDR;
    }
    else
    {
        imu_ok = 0;
        imu_gyro_ok = 0;
        imu_status = err;

        /* 失败了才诊断, 而且不要每次都跑。
         *
         * ★ 而且**只在车停着的时候诊断**。原因: 地址扫描要遍历 0x08~0x77
         *   共 112 个地址, 位翻转 I2C 全程要 100~150ms —— 这段时间主循环被
         *   堵住, CAN 命令发不出去, 车会一顿一顿的。诊断是给"停车排查"用的,
         *   行驶中不需要, 更不该干扰控制。
         *   (现在 IMU 正常所以看不出来; 但一旦 IMU 坏了, 诊断本身就会变成
         *    一个新的故障源 —— 这种"为了查问题而制造问题"的坑踩过。) */
        if ((g_tick_ms - imu_diag_ms) >= IMU_DIAG_PERIOD_MS)
        {
            if ((target_vx == 0.0f) && (target_wz == 0.0f))
            {
                uint8_t bus = YbImu_BusCheck();
                imu_diag_ms = g_tick_ms;

                if (bus != YBIMU_ST_OK)
                {
                    imu_status = bus;               /* 总线本身就不对 */
                    imu_found_addr = 0;
                }
                else
                {
                    imu_found_addr = YbImu_Scan();  /* 总线是好的, 那就是地址/器件问题 */
                    imu_status = (imu_found_addr == 0) ? YBIMU_ST_NO_ACK : YBIMU_ST_READ_ERR;
                }
            }
            else
            {
                /* 在动, 只记录"读失败", 等停下来再做重诊断 */
                imu_diag_ms = g_tick_ms - IMU_DIAG_PERIOD_MS + 100;
            }
        }
    }

    /* 寄存器体检 (1Hz), 放最后 —— 它读得最长(0x16 是 16 字节), 万一出问题
       也不该影响上面那次关键的运动数据读取。 */
    if ((g_tick_ms - imu_probe_ms) >= IMU_DIAG_PERIOD_MS)
    {
        YbImu_Probe_t pr;

        imu_probe_ms = g_tick_ms;
        YbImu_Probe(&pr);
        imu_probe_flags = (uint8_t)((pr.ver_ok   ? 0x01 : 0) |
                                    (pr.gyro_ok  ? 0x02 : 0) |
                                    (pr.mag_ok   ? 0x04 : 0) |
                                    (pr.quat_ok  ? 0x08 : 0) |
                                    (pr.euler_ok ? 0x10 : 0));
        imu_ver_major = pr.ver[0];
        imu_euler_yaw_crad = (int16_t)(pr.euler_yaw * 100.0f);   /* rad -> 0.01rad */
    }
}

/* 物理量 -> 协议规定的 ±2g / ±500dps 原始值, 带限幅(超量程夹住而不是回绕) */
static int16_t IMU_AccelToRaw(float accel_g)
{
    float v = accel_g * IMU_ACCEL_G_TO_RAW;

    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

static int16_t IMU_GyroToRaw(float gyro_rad_s)
{
    float v = gyro_rad_s * IMU_GYRO_TO_RAW;

    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

/*======================= 轮速采样 + IMU =========================
 * 只做采样和上报。**定位不在这里**: 卡尔曼滤波在上位机 RK3588 上跑。
 *=================================================================*/
static void Odom_Tick(void)
{
    Wheel_Update();
    IMU_Tick();     /* 位翻转 I2C 读 IMU, 大约 1~3ms */
}

/*=========================== 状态回传 ==============================
 * 主遥测帧严格按协议 24 字节; 扩展诊断帧头是 0x7E, 见文件开头的说明。
 *=================================================================*/
static void Send_MainTelemetry(uart_port_t *p)
{
    uint8_t f[TEL_FRAME_SIZE];
    uint8_t i;
    uint8_t bcc = 0;
    int16_t vx_mm = (int16_t)(body_vx * 1000.0f);       /* 轮速正解出的车体速度 */
    int16_t vy_mm = 0;                                  /* 差速车没有横向速度 */
    int16_t wz_mrad = (int16_t)(body_wz * 1000.0f);

    f[0] = TEL_HEAD;
    f[1] = (target_vx == 0.0f && target_wz == 0.0f) ? 1 : 0;   /* Flag_Stop */
    PutI16(&f[2], vx_mm);
    PutI16(&f[4], vy_mm);
    PutI16(&f[6], wz_mrad);
    /* IMU 六轴原始值。
       ★ 必须是 ±2g / ±500dps 量程下的值 —— 上位机是按固定系数换算的
         (MD 5.1 节): 加速度 /1671.84 = m/s², 角速度 ×0.00026644 = rad/s。
         YbImu 本身是 ±16g / ±2000dps, 已经在 IMU_AccelToRaw/GyroToRaw 里换算了。
       读失败时保留上一次的值(不清零), 上位机看 flags 的 bit3 判断是否可信。 */
    PutI16(&f[8],  IMU_AccelToRaw(imu_accel_g[0]));   /* ax */
    PutI16(&f[10], IMU_AccelToRaw(imu_accel_g[1]));   /* ay */
    PutI16(&f[12], IMU_AccelToRaw(imu_accel_g[2]));   /* az */
    PutI16(&f[14], IMU_GyroToRaw(imu_gyro[0]));       /* gx */
    PutI16(&f[16], IMU_GyroToRaw(imu_gyro[1]));       /* gy */
    PutI16(&f[18], IMU_GyroToRaw(imu_gyro[2]));       /* gz */
    PutU16(&f[20], battery_mv);
    for (i = 0; i < TEL_FRAME_SIZE - 2; i++) bcc ^= f[i];
    f[TEL_FRAME_SIZE - 2] = bcc;
    f[TEL_FRAME_SIZE - 1] = TEL_TAIL;

    for (i = 0; i < TEL_FRAME_SIZE; i++) Port_SendByte(p, f[i]);
}

/* 扩展诊断帧 (0x7E, 32 字节) —— 本工程自定义, 厂商驱动不认识。
 *
 * 完整布局见文件开头。这里只说为什么需要它:
 * 主遥测帧(24B)是厂商协议定死的, 没有地方放驱动器的输出电流和故障码。
 * 而"某个轮子到底有没有劲"这件事, 不看这两个量就只能靠猜。
 *
 * 帧同步上的安全性 —— 厂商 ROS 节点的入帧条件是"上一字节是帧尾 0x7D(或 0x7F)、
 * 本字节是帧头 0x7B"。本帧永远夹在两个完整的 24 字节帧之间发出, 自己帧头是
 * 0x7E、帧尾是 0x7D, 所以厂商那边会把它整帧跳过, 紧接着的那个 0x7B 恰好满足
 * 入帧条件 —— 24 字节帧一帧都不会丢, 厂商侧不需要任何改动。
 *=================================================================*/
static void Send_DiagFrame(uart_port_t *p)
{
    static uint8_t seq;
    uint8_t f[DIAG_FRAME_SIZE];
    uint8_t i;
    uint8_t bcc = 0;
    uint8_t flags = 0;

    if (ever_linked)      flags |= 0x01;
    if (failsafe_latched) flags |= 0x02;
    if (imu_ok)           flags |= 0x04;    /* 加速度最近一次读取成功 */
    if (imu_gyro_ok)      flags |= 0x08;    /* 陀螺仪自开机以来读到过非零 */
    if (reset_by_iwdg)    flags |= 0x10;    /* 上次复位是看门狗引起的(主循环卡死过) */
    if (!Drive_Ready())   flags |= 0x20;    /* 有驱动器没回码/掉线: 此时只发刹车 */
    if (Drive_ActiveFlag()) flags |= 0x40;   /* 速度模式=正在助推 / 力矩模式=正在给扭矩 */
    if (boost_stuck)      flags |= 0x80;    /* 加到上限还不转, 已进入"给一会歇一会" */

    f[0] = DIAG_HEAD;
    f[1] = flags;
    PutI16(&f[2],  (int16_t)(body_vx * 1000.0f));   /* 轮速正解出的车体速度 */
    PutI16(&f[4],  (int16_t)(body_wz * 1000.0f));
    PutI16(&f[6],  wheel_left_rpm);      /* 实际 */
    PutI16(&f[8],  wheel_right_rpm);
    PutI16(&f[10], target_left_rpm);     /* 目标 */
    PutI16(&f[12], target_right_rpm);
    PutI16(&f[14], m1_cur);              /* 1号驱动器 输出扭矩电流 A×10 */
    PutI16(&f[16], m2_cur);              /* 2号驱动器 输出扭矩电流 A×10 */
    f[18] = m1_fault;                    /* 1号故障码, 见 FOC_FAULT_* */
    f[19] = m2_fault;
    f[20] = m1_mode;                     /* 1号当前模式 0x05 速度 / 0x06 位置 */
    f[21] = m2_mode;
    f[22] = can_error_count;
    f[23] = ports[PORT_MAIN].bad_count;
    f[24] = ports[PORT_MAIN].ok_count;
    f[25] = ports[PORT_MAIN].overflow;
    /* bit0 滚刷 bit1 水泵
       ★ bit2 = 本固件是**力矩模式**(编译期常量, 上车前一眼确认)
         bit3 = **扭矩保护已锁存**(超速/堵转, 见 Torque_Guard) -> 那时候只刹车
       速度模式下 bit2/3 恒为 0, 所以这两位对现有上位机/脚本是透明的。 */
#if DRIVE_MODE_SEL == DRIVE_MODE_TORQUE
    f[26] = (uint8_t)(relay_state | 0x04 | (torque_fault ? 0x08 : 0x00));
#else
    f[26] = relay_state;
#endif
    f[27] = imu_status;                  /* YBIMU_ST_* */
    f[28] = imu_found_addr;              /* 0 = 没扫到 */
    f[29] = seq++;                       /* 帧序号, 上位机据此发现丢帧 */
    /* bit0-4 = IMU 各功能块活没活; bit5-6 = **车型号**(1 扫地机 / 2 洗地机);
       bit7 = **上电自检没转**(点动期间两个轮子实际转速一直是 0)。
       车型塞在这里是因为 ARMCC5 不支持 #pragma message, 编译日志里留不下记号,
       而"这块板烧的是哪辆车的几何"必须能被查出来 —— 选错的后果是转向增益
       差 2.19 倍。bit7 是自检的结论, 因为"动了一下"是操作员肉眼看的, 而
       "到底转没转"得看数据。decode_diag.py 会把车型和这一位都印出来。
       ★ bit7 = 0 有歧义: 可能是"自检通过", 也可能是"自检没跑"(驱动器没就绪
         时放弃、上位机已接管、看门狗复位后跳过)。要区分得加位, 而这里只剩
         1 个空位; 目前靠"上电 3 秒内有没有看到车动一下"来判。 */
    f[30] = (uint8_t)(imu_probe_flags | (uint8_t)(VEHICLE_SEL << 5) |
                      (uint8_t)(selftest_no_move ? 0x80 : 0x00));
    f[31] = imu_ver_major;               /* 模块版本号 */
    PutI16(&f[32], imu_euler_yaw_crad);  /* 模块自己融合的偏航角 0.01rad */
    /* f[34] = BCC, f[35] = 0x7D 下面填 */

    for (i = 0; i < DIAG_FRAME_SIZE - 2; i++) bcc ^= f[i];
    f[DIAG_FRAME_SIZE - 2] = bcc;
    f[DIAG_FRAME_SIZE - 1] = DIAG_TAIL;

    for (i = 0; i < DIAG_FRAME_SIZE; i++) Port_SendByte(p, f[i]);
}

/*============================== main ===============================*/
int main(void)
{
    Tick_Init();
    CAN1_Init();

    /* ★ 读复位原因。IWDG 一旦触发就是"主循环卡死过", 这是必须知道的事 ——
       否则只会看到"车莫名其妙停了一下"却查不出原因。
       RCC_CSR 里的标志是掉电才清, 所以读完要主动清掉, 免得下次复位还报旧账。 */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        reset_by_iwdg = 1;
    }
    RCC_ClearFlag();

    /* ★ CAN 一通就立刻刹车, 一毫秒都别等。
       看门狗复位时驱动器还在执行上一条命令, 每多等一毫秒车就多冲一点。
       以前这里是先 Delay_ms(200) 再刹车 —— 那个延时大概是等驱动器上电就绪
       用的, 但对"MCU 软件复位"这种情况毫无必要: 驱动器本来就是活的,
       帧发过去立刻生效。 */
    CAN1_SendStop(MOTOR1_CAN_ID);
    CAN1_SendStop(MOTOR2_CAN_ID);

    USART1_Init();          /* PA9/PA10 -> CH340 / RK3588, 唯一的上位机口 */
    Relay_Init();
    LED_Init();             /* PB9 状态灯, 上电后一直闪 */

    /* 独立看门狗。放在初始化后面开 —— 前面这些初始化是确定性的、不会卡,
       真正的风险在主循环里(见 IWDG_Init 的说明)。 */
    IWDG_Init();

    /* 只给驱动器/IMU 留点上电就绪的时间。
       200ms 远小于 1 秒的看门狗超时, 所以这里不用喂狗。 */
    Delay_ms(200);

    target_vx = 0.0f;
    target_wz = 0.0f;
    /* 再刹一次, 覆盖上面那 200ms */
    CAN1_SendStop(MOTOR1_CAN_ID);
    CAN1_SendStop(MOTOR2_CAN_ID);

    last_can_ms = g_tick_ms;
    last_odom_ms = g_tick_ms;
    last_tel_ms = g_tick_ms;
    last_cmd_ms = g_tick_ms;

    SelfTest_Init();        /* ★★ 上电自检: 延迟 3 秒后往前点动一下。
                               必须在读过 RCC_CSR 之后调 —— 它要看 reset_by_iwdg
                               决定跳不跳(看门狗复位过就绝不再自己动)。
                               整段由 SELFTEST_ENABLE 控制, 详见 SELFTEST_* 说明。 */

    while (1)
    {
        IWDG_ReloadCounter();       /* 喂狗: 主循环还在转就说明没卡死 */

        LED_Tick();                 /* PB9 心跳灯, 非阻塞。放在循环最前面:
                                       下面的任何一段一旦卡住, 灯就停住不动 ——
                                       这本身就是"主循环卡在哪"的现场指示 */

        Port_ProcessCommands(&ports[PORT_MAIN]);
        CAN1_Poll();

        /* 上电自检点动。放在读命令**之后**: 这样"上位机这一拍发了帧"能立刻
           被看到(ever_linked), 自检当拍就让位, 不会和上位机抢一拍。
           放在看门狗**之前**: 点动期间它自己刷 last_cmd_ms, 免得被死手掐掉。 */
        SelfTest_Tick();

        /*--------------------- 看门狗 (两件事, 必须分开) ---------------------
         * 以前只有一条 last_cmd_ms, 把两件事混在一起了, 结果**继电器帧不算喂狗**,
         * 于是"显式开滚刷"会在 800ms 后自己被断链保护关掉 —— 那样显式控制根本
         * 没法用。两件事本来就不是一回事:
         *
         *   1. **运动超时**: 多久没收到"速度帧" -> 停车(target=0)。
         *      只认速度帧 —— 收到一堆灯带帧/继电器帧并不代表有人在管车速。
         *   2. **链路超时**: 多久没收到"任何合法帧" -> 认定上位机没了, 断继电器。
         *      任何通过校验的帧都算 (证明上位机还活着)。
         *
         * 注意第 1 条是**锁存**的(failsafe_latched), 免得每个周期重复清 target;
         * 第 2 条不锁存 —— Relay_Set(0) 本身幂等, 而且链路恢复后必须能自动合回来。
         *-------------------------------------------------------------------*/
        if ((failsafe_latched == 0) &&
            (ever_linked) &&
            ((g_tick_ms - last_cmd_ms) > LINK_TIMEOUT_MS))
        {
            target_vx = 0.0f;
            target_wz = 0.0f;
            failsafe_latched = 1;
        }

#if RELAY_OFF_ON_LINK_LOSS
        if ((ever_linked) &&
            ((g_tick_ms - last_frame_ms) > LINK_TIMEOUT_MS))
        {
            Relay_Set(0);       /* 上位机彻底不说话了, 滚刷/水泵也断开 */
        }
#endif

        /* 轮速采样 + IMU */
        if ((g_tick_ms - last_odom_ms) >= ODOM_PERIOD_MS)
        {
            last_odom_ms = g_tick_ms;
            Odom_Tick();
        }

        /* 逆解 + CAN 下发 */
        if ((g_tick_ms - last_can_ms) >= CAN_PERIOD_MS)
        {
            last_can_ms = g_tick_ms;
            Drive_Apply();
        }

        /* 上报: 先标准 24 字节帧, 紧跟一帧扩展诊断帧。
           顺序不能反 —— 诊断帧必须夹在两个 24 字节帧之间, 厂商节点才能
           干净地跳过它并正确重新同步。 */
        if ((g_tick_ms - last_tel_ms) >= TELEMETRY_PERIOD_MS)
        {
            last_tel_ms = g_tick_ms;

            Send_MainTelemetry(&ports[PORT_MAIN]);
            Send_DiagFrame(&ports[PORT_MAIN]);
        }

        CAN1_Poll();
        Delay_ms(1);
    }
}
