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
 *   [2-3]   int16 BE 车体 vx   mm/s     (轮速正解)
 *   [4-5]   int16 BE 车体 wz   mrad/s
 *   [6-7]   int16 BE 左轮 实际转速 RPM
 *   [8-9]   int16 BE 右轮 实际转速 RPM
 *   [10-11] int16 BE 左轮 目标转速 RPM
 *   [12-13] int16 BE 右轮 目标转速 RPM
 *   [14-15] int16 BE 1号驱动器 输出扭矩电流  A×100
 *   [16-17] int16 BE 2号驱动器 输出扭矩电流  A×100
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

/*========================= 想改的地方 =========================*/
#define WHEEL_DIAMETER_MM   205.0f  /* 车轮直径 mm */
#define TRACK_WIDTH_MM      930.0f  /* 左右轮触地间距 mm */

#define MAX_RPM             200     /* 单轮转速上限 */
#define MAX_YAW_RATE        3.0f    /* 车体角速度限幅 rad/s */

#define MOTOR1_IS_LEFT         1    /* 1=1号电机在左侧; 若左右反了改成 0 */
#define MOTOR1_INVERT          0    /* 某个轮子转向相反时改成 1 */
#define MOTOR2_INVERT          1    /* 实测2号电机正方向是反的 */

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

/*================= 位置模式 (pos-mode 分支的实验特性) =================
 * 为什么要试它: 驱动器内部那个速度环欠阻尼 —— 阶跃下去过冲到 138%(实测),
 * 约 3 秒才落回, 而且协议不暴露它的 PID。位置环是另一套: 位置误差小到 0.1°
 * 也会出力, 没有速度环那种"误差小就没劲"的死区, 所以**低速和定位理论上会更准**。
 *
 * 语义假设: `data[2..3] = 目标位置(度)`, 与回码 DATA6/7 **同一个参考系**
 * (即驱动器自己的多圈计数)。这类 FOC 驱动器通常都是绝对值语义。
 *
 * ★ 进入位置模式的第一帧, 目标位置**用驱动器实测位置播种**(pos = m_pos),
 *   不用我们自己的累加器。上次"位置锁位"失控的机制就是"死守一路陈旧的绝对
 *   位置目标" —— 一进位置模式就命令一个离当前实际位置很远的绝对角, 驱动器
 *   会以最大电流冲过去。播种从根上消掉这一条, 而且只花一行。
 *
 * 跟随误差限幅: 目标位置领先实测位置超过 POS_ERR_MAX_DEG 就不再往前走。
 *   轮子被堵住时目标不能无限跑掉, 否则障碍一松开车会窜出去。这是任何位置
 *   伺服都要有的, 不是额外谨慎。
 *
 * ★ 位置模式下**停车 = 保持当前位置**(驱动器位置环自己锁住), 这是位置模式自然
 *   的待机行为, 也正好是当初想要的"没命令时轮子锁死"。代价是**不会再切回速度
 *   模式** —— 一进一出正是上次多出来的那个变量。
 *=====================================================================*/
#define DRIVE_MODE_POSITION   1         /* 1 = 位置模式; 0 = 原来的速度模式 */
#define POS_ERR_MAX_DEG       90.0f     /* 跟随误差上限(度) */
#define POS_DEG_PER_RPM       6.0f      /* 1 RPM = 360/60 = 6 度/秒 */

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

/*-------------------------- 物理换算 ------------------------------*/
#define WHEEL_CIRC_M        (3.14159265f * WHEEL_DIAMETER_MM / 1000.0f)  /* 0.64403 m */
#define TRACK_WIDTH_M       (TRACK_WIDTH_MM / 1000.0f)                   /* 0.930 m */
#define MAX_LIN_SPEED       (MAX_RPM * WHEEL_CIRC_M / 60.0f)             /* 2.147 m/s */

/*-------------------------- CAN --------------------------------*/
#define MOTOR1_CAN_ID      0x1101E600UL
#define MOTOR2_CAN_ID      0x1201E600UL
#define MOTOR1_REPLY_ID    (MOTOR1_CAN_ID + 1UL)   /* 控制帧回码 */
#define MOTOR2_REPLY_ID    (MOTOR2_CAN_ID + 1UL)
#define MOTOR1_REPORT_ID   (MOTOR1_CAN_ID + 3UL)   /* 定时上报帧 */

#define MODE_SPEED         0x05
#define MODE_POSITION      0x06
#define CTRL_ENABLE        0x01
#define CTRL_DISABLE       0x02
#define CTRL_BRAKE         0x03

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
#define DIAG_FRAME_SIZE    40       /* pos-mode 分支: 36 -> 40, 加了左右位置(度) */
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
 * 助推(KICK)不动 —— 它只在起步 400ms 内起作用, 不在这个 0.25Hz 环里。 */
#define TRIM_ENABLE        1
#define TRIM_KI            0.3f     /* 每秒、每 1 RPM 偏差攒多少 RPM 修正 */
#define TRIM_UP_K          0.2f     /* 正向修正最多 +20% 名义值 */
#define TRIM_DOWN_K        0.2f     /* 反向修正最多 -20% 名义值 -> 永不反转 */
#define TRIM_MAX_RPM       20.0f    /* 绝对上限, 防高名义值下修正量过大 */
#define TRIM_LIN_BAND_RPM  5.0f     /* ★ 误差超过它就不积分(见上面第 a 条) */
#define TRIM_DEADBAND_RPM  0.5f     /* 目标小于这个就当"不该动", 全部清零 */

/* 起步助推: 目标从"不动"变成"要动"的瞬间, 额外给一段速度。
 * 专门用来突破静摩擦 —— 这是积分做不好也不该做的事(见上面第 c 条)。
 * 幅度取固定值而不是按名义值缩放: 要突破的是**摩擦力矩**, 它和你要跑多快
 * 没关系, 所以固定的一脚在低速档反而相对更有力, 正是需要的。
 *
 * ★ 助推必须"轮子一转起来就撤", 不能只靠定时。实测踩到: 助推只做了定时
 *   (固定 400ms), 结果原地转(真需要助推)没问题, 但**直线运动被助推害了** ——
 *   直线没有静摩擦问题, 轮子 0.06 秒就起来了, 此时还在助推就是纯超速:
 *   `decode --drive 0.30 0` 对应的稳态从 103% 变成 144%, 而 144% ≈ 名义值+助推,
 *   对得上。助推是给"卡住"用的, 所以判据应该是"轮子还没转起来", 不是"时间没到"。
 * 想 A/B 就把 KICK_RPM 改成 0。 */
#define KICK_RPM           12.0f    /* 助推幅度 (RPM) */
#define KICK_MS            400      /* 助推的硬上限时长(轮子一直没起来才用满) */
#define KICK_RELEASE_K     0.6f     /* 实际轮速达到名义值的这个比例就撤掉助推 */
#define KICK_REARM_MS      300      /* 目标归零后要静止这么久才允许再次助推 */

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
static volatile int16_t m1_pos,   m2_pos;       /* DATA6/7 当前位置(度), 需 DLC>=8 */
static uint8_t  pos_seen_mask;                  /* bit0/1 = 收到过该路位置字段 */
static uint32_t m1_rpm_ms, m2_rpm_ms;
static volatile uint16_t battery_mv;

/* 轮速计 */
static int16_t  wheel_left_rpm, wheel_right_rpm;   /* 已按接线校准的物理轮速 */
static uint8_t  wheel_left_fresh, wheel_right_fresh; /* 该轮回码是否新鲜 */
static float    body_vx, body_wz;                  /* 轮速正解出的车体速度 */

/* 外环修正状态 (物理轮速域, 左右各一份) —— 见 TRIM_* 的说明 */
typedef struct
{
    float    trim;          /* 积分修正量, RPM */
    uint32_t kick_until;    /* 起步助推的截止时刻 */
    uint32_t zero_since;    /* 目标从何时起为 0 (0 = 还没开始记) */
    uint8_t  armed;         /* 助推是否已武装 (1 = 下次目标非零时助推) */
} wheel_trim_t;

static wheel_trim_t trim_w[2];      /* [0] = 左轮, [1] = 右轮 */
#define TRIM_IDX_LEFT   0
#define TRIM_IDX_RIGHT  1

/* 位置模式的状态。★ 注意这两个是**驱动器参考系**的绝对角度(度), 不是物理轮角 ——
 * 它们由驱动器实测位置(tx DATA6/7)播种, 之后按"实际发给驱动器的转速"积分推进。
 * 用驱动器自己的参考系, 就不用去猜它的零点在哪、也不用管 MOTOR*_INVERT 怎么映射。 */
static float   pos_t_m1, pos_t_m2;  /* 目标位置(度), 驱动器参考系 */
static uint8_t pos_seeded;          /* 0 = 还没用实测位置播种 */

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
#define CAN1_SendStop(id)        CAN1_SendMotorCmd((id), MODE_SPEED, STOP_CTRL,   0)
/* 位置模式: 设定值是**目标位置(度)**, 和回码 DATA6/7 同一个参考系 */
#define CAN1_SendPosition(id, deg) CAN1_SendMotorCmd((id), MODE_POSITION, CTRL_ENABLE, (deg))

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
           还是给了力但被堵住/拖住了(电流很大)。

           ★ DATA6/7(位置) 需要 DLC>=8, 以前从来没解析过。做位置模式必须先看到它。
             上次"位置锁位"失控的机制是"死守一路陈旧的绝对位置目标", 而当时我们
             对这个位置计数器的语义(参考系/零点/回绕)并没有实测确认。 */
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
            if (rx.DLC >= 8)
            {
                m1_pos = (int16_t)(((uint16_t)rx.Data[6] << 8) | rx.Data[7]);
                pos_seen_mask |= 0x01;      /* 确实收到过位置字段 */
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
            if (rx.DLC >= 8)
            {
                m2_pos = (int16_t)(((uint16_t)rx.Data[6] << 8) | rx.Data[7]);
                pos_seen_mask |= 0x02;
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

/*========================= 轮速外环修正 ============================
 * 按实际轮速修目标轮速, 返回真正要发给驱动器的转速。
 * 两部分: 慢积分修稳态偏差 + 定时助推破静摩擦。
 * 为什么这么分、第一版错在哪, 见上面 TRIM_* / KICK_* 那一大段。
 *
 * nominal / actual 都在物理轮速域(已按接线校准), 可以直接相减。
 *★ 输出保证: 落在 [1-TRIM_DOWN_K, 1+TRIM_UP_K] 倍名义值之内(助推另加),
 *  且**方向永远和名义值一致** —— 修正不可能把车修成倒着走。
 *=================================================================*/
static float Trim_Apply(wheel_trim_t *w, float nominal, float actual,
                        uint8_t fresh, float dt)
{
    float out;
    float kick = 0.0f;

#if TRIM_ENABLE
    float mag;
    float up;
    float dn;

    mag = fabsf(nominal);

    /*---- 不该动: 修正、助推、计时全部清零, 不留任何历史 ----*/
    if (mag < TRIM_DEADBAND_RPM)
    {
        w->trim = 0.0f;
        w->kick_until = 0;

        /* 静止够久才重新武装助推。没这一条的话, 目标在 0 附近抖动
           (比如主机反复发 0.001 m/s) 会不停触发助推, 车一窜一窜的。 */
        if (w->armed == 0)
        {
            if (w->zero_since == 0)
            {
                w->zero_since = g_tick_ms;
            }
            else if ((g_tick_ms - w->zero_since) >= KICK_REARM_MS)
            {
                w->armed = 1;
                w->zero_since = 0;
            }
        }
        return 0.0f;
    }

    /*---- 目标从"不动"变成"要动": 起一次助推 ----*/
    if (w->armed)
    {
        w->armed = 0;
        w->zero_since = 0;
        w->kick_until = g_tick_ms + KICK_MS;
    }

    /*---- 助推的撤销: 轮子一旦真的转起来就立刻撤, 时间到只是兜底 ----
       原地转的轮子会被刮擦卡住(实际转速远低于名义值), 助推就一直给到超时;
       直线运动的轮子 0.06 秒就起来了, 助推马上就被撤掉, 不会造成超速。
       撤掉之后由积分接手, 不需要助推再参与。 */
    if (w->kick_until != 0)
    {
        if (fresh && (fabsf(actual) >= (mag * KICK_RELEASE_K)))
        {
            w->kick_until = 0;                      /* 已经转起来了 */
        }
        else if ((int32_t)(g_tick_ms - w->kick_until) >= 0)
        {
            w->kick_until = 0;                      /* 超时兜底 */
        }
        else
        {
            kick = KICK_RPM;
        }
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

    /* 助推按名义方向叠加 */
    if (kick > 0.0f)
    {
        out += (nominal > 0.0f) ? kick : -kick;
    }
#else
    (void)actual;
    (void)fresh;
    (void)dt;
    w->trim = 0.0f;
    w->kick_until = 0;
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

/*=============== 位置模式的角回绕工具 ==============================
 * 位置指令和回码都是 int16 **度**, 也就是 ±32767°(约 ±91 圈)之后回绕。
 * 0.3m/s 时轮子 167°/s, 约 196 秒就会走到边界 —— 跑久一点必然遇到。
 *
 * 回绕本身不可怕(双方一起回绕就没事), 可怕的是**跨越回绕点的那一帧**: 直接
 * 相减会得到 ±65534° 的假误差, 跟随误差限幅会把它当成"严重落后"从而乱放行
 * 或乱刹车。所以差值必须折回 ±32768° 再比较。
 *=================================================================*/
static float WrapDeg(float d)
{
    while (d >  32767.0f) d -= 65536.0f;
    while (d < -32768.0f) d += 65536.0f;
    return d;
}

static float WrapDiff(float a, float b)     /* a - b, 折回 ±32768° */
{
    return WrapDeg(a - b);
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
    float cmd_l;
    float cmd_r;
    int16_t m1;
    int16_t m2;

    if ((dt <= 0.0f) || (dt > 0.2f)) dt = (float)CAN_PERIOD_MS * 0.001f;
    last_trim_ms = g_tick_ms;      /* 就算下面提前 return 也要更新, 免得恢复时 dt 爆掉 */

    /* ★ 驱动器没都就绪: 只发刹车, 绝不发速度。
       差速车单轮出力 = 原地打转, 而实测真的出现过"直行命令变成向左转"。
       顺便把外环状态清零, 免得带着上一次攒的修正冲出去。 */
    if (!Drive_Ready())
    {
        trim_w[TRIM_IDX_LEFT].trim  = 0.0f;
        trim_w[TRIM_IDX_RIGHT].trim = 0.0f;
        trim_w[TRIM_IDX_LEFT].kick_until  = 0;
        trim_w[TRIM_IDX_RIGHT].kick_until = 0;
        pos_seeded = 0;         /* 位置模式: 停过之后要重新用实测位置播种 */
        target_left_rpm  = 0;
        target_right_rpm = 0;
        CAN1_SendStop(MOTOR1_CAN_ID);
        CAN1_SendStop(MOTOR2_CAN_ID);
        return;
    }

    rpm_l = ClampF(rpm_l, -(float)MAX_RPM, (float)MAX_RPM);
    rpm_r = ClampF(rpm_r, -(float)MAX_RPM, (float)MAX_RPM);

    /* 外环修正: 用实际轮速去修目标轮速。原地转起步时轮子是真的没转, 这里会把
       指令顶上去直到它真的转; 稳态偏高时又会把指令压回来。
       ★ 目标为 0 时 Trim_Apply 恒定返回 0, 而且它保证输出方向和名义值一致,
         所以外环**不可能**在主机下令停车的时候把车带着走, 也不可能把某个轮子
         修成倒转 —— 这是这套做法敢上车的底线。 */
    cmd_l = Trim_Apply(&trim_w[TRIM_IDX_LEFT], rpm_l,
                       (float)wheel_left_rpm, wheel_left_fresh, dt);
    cmd_r = Trim_Apply(&trim_w[TRIM_IDX_RIGHT], rpm_r,
                       (float)wheel_right_rpm, wheel_right_fresh, dt);

    /* 物理轮速 -> CAN 命令 (反向应用接线校准) */
    if (MOTOR1_IS_LEFT) { m1 = (int16_t)cmd_l; m2 = (int16_t)cmd_r; }
    else                { m1 = (int16_t)cmd_r; m2 = (int16_t)cmd_l; }
    if (MOTOR1_INVERT) m1 = (int16_t)(-m1);
    if (MOTOR2_INVERT) m2 = (int16_t)(-m2);

    /* 上报的是**最终真正发给驱动器**的物理轮速(含外环修正), 不是电机命令,
       免得左右概念混淆。主机拿自己下发的理论目标去减, 就是外环修正量。 */
    target_left_rpm  = (int16_t)cmd_l;
    target_right_rpm = (int16_t)cmd_r;

#if DRIVE_MODE_POSITION
    /*===================== 位置模式 =====================
     * 把速度指令积分成位置目标, 下发**位置**, 让驱动器的位置环去追。
     *
     * 两个要点(理由见 DRIVE_MODE_POSITION 上面那段):
     *   - 第一帧用驱动器**实测位置**播种, 绝不命令一个远处的绝对角;
     *   - 跟随误差限幅: 目标领先实测超过 POS_ERR_MAX_DEG 就不再放行。
     * 停车 = 保持当前位置(位置环自己锁住), 不切模式、不发刹车 ——
     * 一进一出正是上次多出来的那个变量。
     *===================================================*/
    if ((pos_seeded == 0) && ((pos_seen_mask & 0x03) == 0x03))
    {
        pos_t_m1 = (float)m1_pos;       /* ★ 播种: 从驱动器实测位置开始 */
        pos_t_m2 = (float)m2_pos;
        pos_seeded = 1;
    }

    if (pos_seeded == 0)
    {
        /* 还没拿到两路位置回码: 什么都别发, 保持刹车 */
        CAN1_SendStop(MOTOR1_CAN_ID);
        CAN1_SendStop(MOTOR2_CAN_ID);
        return;
    }

    /* 目标按"实际发给驱动器的转速"推进 (1 RPM = 6 度/秒) */
    pos_t_m1 = WrapDeg(pos_t_m1 + (float)m1 * POS_DEG_PER_RPM * dt);
    pos_t_m2 = WrapDeg(pos_t_m2 + (float)m2 * POS_DEG_PER_RPM * dt);

    /* 跟随误差限幅(用回绕感知的差值, 否则跨 ±32767° 时会误判成巨大误差) */
    pos_t_m1 = (float)m1_pos + ClampF(WrapDiff(pos_t_m1, (float)m1_pos),
                                      -POS_ERR_MAX_DEG, POS_ERR_MAX_DEG);
    pos_t_m2 = (float)m2_pos + ClampF(WrapDiff(pos_t_m2, (float)m2_pos),
                                      -POS_ERR_MAX_DEG, POS_ERR_MAX_DEG);

    CAN1_SendPosition(MOTOR1_CAN_ID, (int16_t)pos_t_m1);
    CAN1_SendPosition(MOTOR2_CAN_ID, (int16_t)pos_t_m2);
#else
    /*===================== 走 / 停 两态 (速度模式) =====================
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
    if ((pos_seen_mask & 0x03) == 0x03) flags |= 0x40;  /* 两路位置回码都收到过 */

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
    f[26] = relay_state;                 /* bit0 滚刷 bit1 水泵 */
    f[27] = imu_status;                  /* YBIMU_ST_* */
    f[28] = imu_found_addr;              /* 0 = 没扫到 */
    f[29] = seq++;                       /* 帧序号, 上位机据此发现丢帧 */
    f[30] = imu_probe_flags;             /* 各功能块活没活 */
    f[31] = imu_ver_major;               /* 模块版本号 */
    PutI16(&f[32], imu_euler_yaw_crad);  /* 模块自己融合的偏航角 0.01rad */
    PutI16(&f[34], m1_pos);              /* 1号驱动器实测位置 (度), DATA6/7 */
    PutI16(&f[36], m2_pos);              /* 2号驱动器实测位置 (度) */
    /* f[38] = BCC, f[39] = 0x7D 下面填 */

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

    while (1)
    {
        IWDG_ReloadCounter();       /* 喂狗: 主循环还在转就说明没卡死 */

        Port_ProcessCommands(&ports[PORT_MAIN]);
        CAN1_Poll();

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
