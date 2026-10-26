#include "stm32f10x.h"
#include "YbImu.h"

/* ★ 本文件不再 include Delay.h。
 *   Delay_ms() 走 SysTick 且把"1 us = 72 个周期"写死: HSE 起不来(实际跑
 *   8 MHz HSI) 时同一个 Delay_ms(30) 会变成 270 ms。启动里 6 次 30 ms 就是
 *   1.6 s, 超过 1 s 的 IWDG —— 表现成"上电就复位、永远进不了主循环"。
 *   启动/主循环的等待统一改用下面的 Wait_Ms()(TIM2 时基 + 硬上限)。
 *   Delay.c/Delay.h 仍留在工程里, 只是不再被调用。 */

/*==========================================================================
 * 两驱差速底盘 —— 达妙 DM-H65 轮毂电机 (DM6540-1EC) 最小"忠实桥"固件
 *
 *   本文件做三件事:
 *     1. 忠实执行上位机命令 —— 收 11 字节速度帧, 差速逆解后原样下发给
 *        两台达妙电机 (CAN 标准帧, 1 Mbps, 速度模式)。
 *     2. 20 Hz 发 44 字节诊断帧 (0x7E), 内容全部来自 DM 的反馈帧, 不做
 *        滤波、不做外环、不做任何"智能"修正。
 *     3. 20 Hz 发 24 字节主遥测帧 (0x7B), **IMU 六轴原始值就在这一帧**:
 *        [8..19] = accel xyz / gyro xyz。字段偏移和缩放与旧 main 分支
 *        逐字节一致 (见 Send_MainTelemetry), 上位机/工具不用改。
 *
 *   旧固件里的 Boost 助推、Trim 轮速外环、力矩模式、偏航捕获、上电自检、
 *   里程计、继电器、死锁保护、FOC 协议、扩展帧、500 kbps 全部删除;
 *   IMU (YbImu, PB10/PB11 位翻转 I2C) 保留并恢复上报 —— 它是上位机 EKF
 *   的独立角速度来源, 和轮速无关, 删掉就没有姿态反馈了。
 *   原因很简单: 底盘电机换成了带自己速度环的达妙伺服, 上位机也换成了
 *   RK3588。STM32 再叠一层控制律, 只会和电机内部的速度环打架; 而"桥"
 *   越简单, 出问题时越容易定位是"命令没到"还是"电机没执行"。
 *
 *   硬件连接:
 *     USART1  PA9(TX) / PA10(RX) <-> RK3588        115200 8N1
 *     CAN1    PA12(TX) / PA11(RX) -> 左 DM (ID 1) / 右 DM (ID 2), 1 Mbps
 *     PB10/PB11                   <-> 亚博 YbImu (软件 I2C, 地址 0x23)
 *     PB9     LED 心跳, 500 ms 翻转
 *     IWDG    主循环卡死 -> 复位; 复位后重新写模式并使能
 *
 *   本文件不实现: 里程计、继电器、OLED、参数读取、浮点 printf。
 *   两帧都是定长二进制, 上位机自己按大端解析, 不需要 printf。
 *   24 字节帧的电池电压字段保留但填 0 —— 本板没有电池采样, 达妙协议
 *   也没有对应量, 不猜、不编。
 *========================================================================*/

/*============================== 配置 ====================================
 * 所有长度单位统一成 m、rad、s。协议里的 mm/s、mrad/s 只在收发的边界上
 * 做一次换算, 中间控制量不混用单位 —— 混单位的 bug 最难查。
 *=======================================================================*/
/* ★ 2026-10-21 实测更正: 原来以为 CAN_ID=1 是左轮, 实际 **CAN_ID=1 是右轮**。
   所以这里把两个节点的 CAN_ID / MST_ID 对调 —— 不是"改硬件", 是改正软件里
   左右的定义。改完之后: 左轮状态帧字段/左轮目标 → 真正接到左轮那个驱动器。
   ★ 只改这一处即可: 全文件所有发送/接收都走这 4 个宏。 */
#define MOTOR1_CAN_ID       2           /* 左轮 DM 节点号 (控制帧 0x202) */
#define MOTOR1_MST_ID       0x12        /* 左轮反馈帧 ID */
#define MOTOR2_CAN_ID       1           /* 右轮 DM 节点号 (控制帧 0x201) */
#define MOTOR2_MST_ID       0x11        /* 右轮反馈帧 ID */

#define WHEEL_R_M           0.0855f     /* 轮半径 m (171 mm 轮径 / 2) */
/*====================== 电机安装方向 (左右独立, 按车型分) =================
 * 轮毂电机装在车的左侧和右侧是**镜像安装**的: 同一个"电机反馈为正", 在左轮上
 * 对应车前进, 在右轮上就对应车后退。所以方向系数必须**每个轮子一个**。
 *
 * 旧固件只有一个全局 DM_DIR_SIGN, 那是洗地车两台电机恰好同向安装的巧合。
 * 换车/重新装配后就会出现整机动作错位, 典型症状:
 *     按"前进" → 原地打转;   按"左转" → 车往前走
 * 根因是某一侧方向系数反了, **不是混控公式错** —— 别去改逆解。
 *
 * 语义:  target_omega = DM_DIR_x * (v_x / WHEEL_R_M)
 *        +1.0f = 该轮"发正转速 → 车前进"
 *        -1.0f = 该轮取反
 *        命令侧和反馈侧必须乘**同一个**宏, 否则"命令往前、上报往后"。
 *
 * ★ 现场判定 (车轮必须架空):
 *     1. 发纯前进 vx=+0.10, wz=0;
 *     2. 两个轮子应当**同向朝前**;
 *     3. 若两轮对转 → 某一侧反了, **朝后转的那个轮子**就是要取反的那个。
 *   判定完只改对应车型的那一行, 重新编译。
 *
 * 为什么按车型分而不是写死一份: 方向属性和轮距一样, 是**车身装配**属性,
 * 洗地车两台电机同向装、扫地车左轮镜像装 —— 两个 target 必须各带各的。
 *=======================================================================*/
#if (VEHICLE_SEL == 1)                  /* 扫地车 (716 轮距) */
#define DM_DIR_LEFT         (+1.0f)     /* 左轮镜像安装: 正转速 = 车前进 */
#define DM_DIR_RIGHT        (-1.0f)     /* 右轮: 负转速 = 车前进 */
#elif (VEHICLE_SEL == 2)                /* 洗地车 (456 轮距) */
#define DM_DIR_LEFT         (-1.0f)     /* 两轮同向安装: 负转速 = 车前进 */
#define DM_DIR_RIGHT        (-1.0f)
#else
#error "VEHICLE_SEL 未定义或非法: 必须是 1(扫地车) 或 2(洗地车), 在 Keil target Define 里指定"
#endif
/*----------------------------- 车型参数 ---------------------------------
 * 轮距由 **Keil target 的编译宏**传进来, 不再写死在代码里:
 *      -D TRACK_WIDTH_MM=716   → sweeper  (新扫地车, 171 轮 / 716 轮距)
 *      -D TRACK_WIDTH_MM=456   → scrubber (洗地车,   171 轮 / 456 轮距)
 *
 * 为什么必须由 target 传、且没传就 #error:
 *   轮距错是**安全级**错误 —— 差速逆解 v_left/v_right = vx ∓ wz*TRACK/2,
 *   轮距写错等于转向增益写错, 车会转过头或转不够, 而上报的 wz 却是按错的
 *   轮距反算的(自洽), 看数据根本发现不了, 只有车跑起来才知道。
 *   所以宁可**编不过**, 也不要静默用错车型的参数。
 *
 * 加新车型: 在 Keil 里复制一个 target, 改 OutputDirectory/OutputName 和
 *   这一个 -D 数字即可, **不需要改本文件**。
 *-----------------------------------------------------------------------*/
#ifndef TRACK_WIDTH_MM
#error "未指定车型轮距: 请在 Keil target 的 C/C++ Define 里加 TRACK_WIDTH_MM (sweeper=716, scrubber=456)"
#endif
#if (TRACK_WIDTH_MM < 200) || (TRACK_WIDTH_MM > 2000)
#error "TRACK_WIDTH_MM 超出合理范围 (200~2000 mm), 检查是否写错单位(要毫米, 不是米)"
#endif
#define TRACK_WIDTH_M       ((float)TRACK_WIDTH_MM / 1000.0f)
#define MAX_OMEGA_RAD_S     30.0f       /* 单轮角速度上限 rad/s (~2.57 m/s) */

/*----------------------------- 时间参数 ---------------------------------
 * 20 ms 控制周期 = 50 Hz。达妙速度模式内部跑自己的电流/速度环, 50 Hz
 * 下发足够; 再快只会增加总线负载, 不会让车响应更好。
 * 50 ms 上报 = 20 Hz。44 字节在 115200 下约 3.8 ms, 占用可控。
 *-----------------------------------------------------------------------*/
#define CMD_TIMEOUT_MS      800         /* 上位机命令超时 */
#define FB_TIMEOUT_MS       2000        /* 电机反馈超时 */
#define WHEEL_STALE_MS      200         /* 遥测帧轮速超过这么久没回码就按 0 报 */
#define CTRL_PERIOD_MS      20          /* 控制/下发周期 */
#define STATUS_PERIOD_MS    50          /* 状态上报周期 */

#define LED_BLINK_MS        500         /* PB9 心跳翻转周期 */
#define STATUS_VERSION      2           /* 状态帧 [30] 版本号: v2 = 支持主机功能帧(电机重新使能) */
#define STATUS_FRAME_SIZE   44
#define STATUS_HEAD         0x7E
#define STATUS_TAIL         0x7D

/*--------------------- 24 字节主遥测帧 (旧协议) -------------------------
 * 旧固件 Send_MainTelemetry 的格式, 原样恢复。帧头同样是 0x7B, 和 11 字节
 * 命令帧"同头不同向/不同长": 命令帧是上位机 -> STM32, 本帧是 STM32 ->
 * 上位机, 靠帧长和收发方向区分。
 *
 * 帧同步依赖"上一字节是帧尾 0x7D、本字节是帧头 0x7B"。所以发送顺序必须
 * 是 **先 24 字节帧, 再 44 字节诊断帧**:
 *   ... 0x7D (tel 尾) -> 0x7E (diag 头) -> ... -> 0x7D (diag 尾)
 *       -> 0x7B (下一帧 tel 头)                    <- 接收端在这里入帧
 * 诊断帧被完整跳过, 它的帧尾又给下一个 0x7B 当好"前导帧尾"。
 * 反过来的话, 诊断帧头 0x7E 前面是 0x7B 而不是帧尾, 两端都可能失去同步。
 *-----------------------------------------------------------------------*/
#define TEL_FRAME_SIZE      24
#define TEL_HEAD            0x7B
#define TEL_TAIL            0x7D

#define CMD_FRAME_SIZE      11
#define CMD_HEAD            0x7B
#define CMD_TAIL            0x7D

/*--------------------------- 达妙 DM 协议 --------------------------------
 * 控制帧 ID = 0x200 + CAN_ID, 速度模式下数据是 float32 小端 rad/s。
 * 反馈帧 ID = MST_ID, 8 字节定长。
 * 参数读写统一走 0x7FF:
 *   写: [CANID_L, CANID_H, 0x55, RID, d0, d1, d2, d3] (d 为 uint32 小端)
 *   读: [CANID_L, CANID_H, 0x33, RID, 0, 0, 0, 0]
 * 本固件只写 CTRL_MODE, 不读参数 —— 见下面 DM_PMAX/VMAX/TMAX 的风险说明。
 *-----------------------------------------------------------------------*/
#define DM_CMD_ENABLE       0xFC        /* FF FF FF FF FF FF FF FC */
#define DM_CMD_DISABLE      0xFD        /* 本固件运行时不发, 见命令超时说明 */
#define DM_CMD_CLEAR_ERR    0xFB        /* 本固件不自动清错, 让上位机看见故障 */

#define DM_RID_CTRL_MODE    0x0A        /* 控制模式寄存器 */
#define DM_MODE_VELOCITY    3           /* 3 = 速度模式 (1=MIT 2=位置) */

/* ★ 默认量程假设 (DM6540-1EC 出厂默认) —— 不读 0x15/0x16/0x17。
 *
 *   为什么先不读: 读参数要额外等 0x7FF 回帧, 一旦驱动器还没启动完就会
 *   超时, 又多一条启动失败路径; 而这三个默认值对"把车先动起来"足够。
 *   风险: 如果实车电机被改过量程, 反馈的 vel/torque/pos 会按比例失真,
 *   但速度指令本身 (rad/s) 不受影响。上车第一件事应该是核对这三个值。
 */
#define DM_PMAX             12.566f     /* 位置量程 ±PMAX rad (4π) */
#define DM_VMAX             100.0f      /* 速度量程 ±VMAX rad/s */
#define DM_TMAX             40.0f       /* 扭矩量程 ±TMAX Nm */

#define DM_PARAM_ID         0x7FF       /* 参数读写广播 ID */

/*------------------------------ IMU ------------------------------------
 * YbImu 的原始量程是 ±16g / ±2000dps, 而 24 字节遥测帧里的 IMU 字段
 * 必须是上位机约定的 ±2g / ±500dps 原始值 —— 上位机按固定系数换算:
 *     m/s²  = 原始值 / 1671.84
 *     rad/s = 原始值 * 0.00026644
 * 所以这里必须先换算, 再限幅 (超量程夹住, 不能回绕)。
 * 这两个常量与旧 main 分支完全一致, 改错不会报错, 只会让上位机数据全错。
 *-----------------------------------------------------------------------*/
#define IMU_ACCEL_G_TO_RAW  (9.80665f * 1671.84f)   /* g      -> ±2g   原始值 */
#define IMU_GYRO_TO_RAW     (1.0f / 0.00026644f)    /* rad/s  -> ±500dps 原始值 */
#define IMU_GYRO_Z_SIGN     1       /* 陀螺 Z 轴方向; 装反了就改成 -1 */
#define IMU_DIAG_PERIOD_MS  1000    /* 读失败时, 总线检查/地址扫描最多 1 Hz */

/*=========================== 全局状态 =================================*/
volatile uint32_t g_tick_ms;            /* TIM2 1 ms 时基, 中断里自增 */

/*-------------------------- 启动阶段 / 错误码 --------------------------
 * 通过 44 字节状态帧最后一个保留字节 [41] 上报。以前这里恒 0, 上电没数据时
 * 只能猜"是没进主循环, 还是外设坏了"; 现在串口上直接能分开:
 *   0 = 启动中        (本固件所有帧都在主循环里发, 所以串口上不会看到 0)
 *   1 = 正常          (CAN 初始化成功, 且 IMU 至少成功读过一次)
 *   2 = CAN 初始化失败 (已降级继续跑, canFail 会一直涨)
 *   3 = IMU 从没读到  (已跳过, imu_ok=0, 遥测帧照发, 六轴填 0/上次值)
 *   4 = CAN 和 IMU 都失败
 * 这三个状态都**不影响**帧继续发 —— 外设坏只降级, 不阻塞。
 *--------------------------------------------------------------------*/
#define BOOT_STAGE_RUNNING      1
#define BOOT_STAGE_CAN_FAIL     2
#define BOOT_STAGE_IMU_FAIL     3
#define BOOT_STAGE_BOTH_FAIL    4

static uint8_t g_boot_stage;            /* 1 或 2, 由 CAN 初始化结果决定 */
static uint8_t g_can_init_ok;           /* 1 = CAN_Init 返回成功 */

/*----------------------------- 电机反馈 --------------------------------*/
typedef struct
{
    float    pos_rad;                   /* 位置 rad */
    float    vel_rad_s;                 /* 实际速度 rad/s */
    float    tq_nm;                     /* 实际扭矩 Nm */
    uint8_t  err;                       /* DM 4 bit 错误码: 0=失能 1=使能 */
    uint8_t  mos_temp;                  /* MOS 温度 ℃ (原始字节) */
    uint8_t  coil_temp;                 /* 线圈温度 ℃ (原始字节) */
} dm_motor_t;

static dm_motor_t g_motor[2];           /* [0]=左 [1]=右 */
static uint8_t    g_fb_seen[2];         /* 是否至少收到过一帧反馈 */
static uint32_t   g_last_fb_ms[2];      /* 最后一帧反馈的时刻 */
static uint8_t    g_online[2];          /* 反馈在 FB_TIMEOUT_MS 内为 1 */
static uint8_t    g_fb_count;           /* 收到的反馈帧总数, 饱和 8 位 */

/*----------------------------- 命令与目标 ------------------------------*/
static int16_t  g_cmd_vx_mm;            /* 最近一次收到的 vx, mm/s */
static int16_t  g_cmd_wz_mrad;          /* 最近一次收到的 wz, mrad/s */
static int16_t  g_eff_vx_mm;            /* 本拍实际执行的 vx (超时则 0) */
static int16_t  g_eff_wz_mrad;          /* 本拍实际执行的 wz (超时则 0) */
static uint32_t g_last_cmd_ms;          /* 最近一次合法速度帧的时刻 */
static uint32_t g_boot_reinit_ms;       /* 上电后补使能的节流时刻 (见主循环) */
static uint8_t  g_cmd_timeout;          /* 1 = 超过 800 ms 没收到命令 */
static float    g_target_omega[2];      /* 下发给电机的目标 rad/s */
static uint8_t  g_can_tx_fail;          /* CAN 发送失败计数, 饱和 8 位 */

/*------------------------------ IMU -----------------------------------*/
static float    imu_accel_g[3];         /* 加速度, 单位 g (YbImu 原始量程) */
static float    imu_gyro[3];            /* 角速度, 单位 rad/s */

/* IMU 诊断状态 —— 从旧 main 分支原样搬来, 通过 44 字节诊断帧上报。
   读失败时不清零 imu_accel_g/imu_gyro, 保留上一次的值 (旧行为)。 */
static uint8_t  imu_ok;                 /* 1 = 最近一次读取成功(加速度有效) */
static uint8_t  imu_gyro_ok;            /* 1 = 最近一次成功读取时角速度非零 */
static uint8_t  imu_ever_ok;            /* 1 = 上电后至少成功读过一次(给 [41] 用) */
static uint8_t  imu_status;             /* YBIMU_ST_* 失败原因, 0 = 正常 */
static uint8_t  imu_found_addr;         /* 扫描到的 I2C 地址, 0 = 没扫到 */
static uint32_t imu_diag_ms;            /* 上次总线检查/地址扫描的时刻 */

/* IMU 寄存器体检 (1 Hz)。陀螺仪恒 0 时靠它区分"只有陀螺仪坏"还是
   "整个模块只剩加速度可用"。YbImu.c 里已不再读 0x16/0x26 长块。 */
static uint8_t  imu_probe_flags;        /* bit0 版本 bit1 陀螺 bit2 磁力 */
static uint8_t  imu_ver_major;          /* 模块版本号主字节 */
static uint32_t imu_probe_ms;           /* 上次寄存器体检的时刻 */

/*=========================== 1 ms 时基 ================================*/
static void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    tim.TIM_Prescaler = 7200 - 1;       /* 72 MHz / 7200 = 10 kHz */
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 10 - 1;            /* 10 kHz / 10 = 1 kHz -> 1 ms */
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

/*==================== 不依赖 SysTick 的毫秒等待 ======================
 * 为什么不用 Delay_ms:
 *   Delay_us() 是 `SysTick->LOAD = 72 * xus`, 把系统时钟写死成 72 MHz。
 *   如果 HSE 晶振没起振, SystemInit 会退回 8 MHz HSI, 此时 Delay_us(1000)
 *   实际是 9 ms, Delay_ms(30) 实际 270 ms —— 启动里 6 次就 1.6 s, 超过
 *   1 s 的 IWDG, 板子进不了主循环, 而且现场只能看到"上电没数据"。
 *
 * 这里用 TIM2 的 1 ms 时基等, 并且带 guard:
 *   时基如果停摆(只可能发生在 Tick_Init 之前, 而它是最先调的), guard 到 0
 *   就返回 —— 宁可少等一会儿, 也绝不在启动路径里永久卡住。
 *===================================================================*/
static void Wait_Ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    uint32_t guard = ms * 20000u + 20000u;

    while ((g_tick_ms - start) < ms)
    {
        if (--guard == 0)
        {
            return;                     /* 时基异常: 不在这里把整机拖死 */
        }
    }
}

/*=========================== 独立看门狗 ===============================
 * 为什么必须有 IWDG:
 *   软件看门狗 (命令超时) 本身就在主循环里。主循环一旦卡死, 它也跟着停,
 *   STM32 就不再发速度帧; 而达妙驱动器不会因为"没收到命令"自己停, 它会
 *   一直执行最后一条速度命令 —— 车自己开走。
 *   IWDG 用独立的 LSI, 主循环超过 ~1 s 不喂狗就强制复位; 复位后本固件
 *   重新写模式、使能, 并从 0 速度开始。
 *
 * 超时 = 64 / 40 kHz * 625 = 1.0 s (LSI 实际 30~60 kHz, 约 0.67~1.33 s)。
 *=====================================================================*/
static void IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(625);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/*=============================== CAN1 ================================*/
/* 1 Mbps 位时序 (PCLK1 = 36 MHz):
 *   36 MHz / 4 = 9 MHz;  9 / (1 + BS1(6) + BS2(2)) = 1 Mbps。
 *   采样点 (1+6)/9 = 77.8%, 对 1 Mbps 是常见的安全取值。
 * 只收标准帧: 达妙 DM 的控制帧/反馈帧都是 11 bit 标准帧, 旧固件的
 * 扩展帧 (IDE=1) 是 FOC 驱动器的协议, 已随旧驱动器一起删除。
 */
/* 返回 1 = 初始化成功, 0 = 失败(超时/进不了初始化模式)。
 * ★ 以前这里忽略 CAN_Init() 的返回值, 而且外面没有超时 —— 照标准库的写法,
 *   只要调用方套一层 `while (CAN_InitStatus_Failed == CAN_Init(...))` 就是
 *   永久卡死。本函数**不重试**: 失败就返回 0, 由调用方标记 boot_stage=2
 *   后照常进主循环, 发送失败的帧由 canFail 计数反映。
 *   标准库 CAN_Init 自己等 INAK 的上限是 INAK_TIMEOUT(0xFFFF 次), 有界。 */
static uint8_t CAN1_Init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12;                /* CAN1_TX */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;                /* CAN1_RX */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;                      /* 总线关闭后自动恢复 */
    can.CAN_AWUM = ENABLE;
    /* ★ NART = ENABLE (不自动重传) 是这里的关键决定:
       如果某台电机掉线/没上电, 发往它的帧永远等不到 ACK。自动重传会
       把 3 个发送邮箱全部占满, 结果**另一台还在线的电机也收不到命令**。
       关掉自动重传, 一帧发一次, 失败就计数, 控制循环不会被拖住。 */
    can.CAN_NART = ENABLE;
    can.CAN_RFLM = DISABLE;                     /* FIFO 覆盖旧帧, 永远读最新 */
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_6tq;
    can.CAN_BS2 = CAN_BS2_2tq;
    can.CAN_Prescaler = 4;
    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        return 0;                   /* 有界失败: 不重试, 不让启动卡在这里 */
    }

    /* 接收全部通过, 软件里只认 0x11/0x12。总线上只有两台电机, 没必要
       为两个 ID 配硬件过滤; 过滤全开还能顺带看到参数写回帧, 便于排查。 */
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

    return 1;
}

static void Can_TxFail(void)
{
    if (g_can_tx_fail != 0xFF)
    {
        g_can_tx_fail++;
    }
}

/* 发一帧标准帧。发送是阻塞的, 但有上限:
 * 1 Mbps 下一帧约 110 us, 即使对端不在线 (NART) 也会很快失败。
 * 8000 次轮询是"远大于正常耗时、又不会让主循环卡死"的折中。
 */
static void CAN1_SendFrame(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    CanTxMsg tx;
    uint8_t mailbox;
    uint8_t status;
    uint16_t timeout;
    uint8_t i;

    tx.StdId = id;
    tx.ExtId = 0;
    tx.IDE = CAN_Id_Standard;           /* ★ 标准帧, 不是扩展帧 */
    tx.RTR = CAN_RTR_Data;
    tx.DLC = dlc;
    for (i = 0; i < 8; i++)
    {
        tx.Data[i] = (i < dlc) ? data[i] : 0;
    }

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        Can_TxFail();
        return;
    }

    timeout = 8000;
    do
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
    } while ((status == CAN_TxStatus_Pending) && (--timeout != 0));

    if (status != CAN_TxStatus_Ok)
    {
        Can_TxFail();
    }
}

/*============================ 达妙协议 ================================
 * 使能/失能/清错都是"特殊帧": 发到控制帧 ID, 8 字节 FFx7 + 命令字。
 * 本固件只在启动时发使能; 运行时绝不失能 —— 理由见命令超时那段。
 *=====================================================================*/
static void DM_SendSpecial(uint8_t can_id, uint8_t cmd)
{
    uint8_t d[8];

    d[0] = 0xFF;
    d[1] = 0xFF;
    d[2] = 0xFF;
    d[3] = 0xFF;
    d[4] = 0xFF;
    d[5] = 0xFF;
    d[6] = 0xFF;
    d[7] = cmd;
    CAN1_SendFrame((uint16_t)(0x200 + can_id), d, 8);
}

static void DM_SendEnable(uint8_t can_id)
{
    DM_SendSpecial(can_id, DM_CMD_ENABLE);
}

/* 速度模式控制帧: 0x200 + CAN_ID, float32 小端, 单位 rad/s, DLC=4。 */
static void DM_SendSpeed(uint8_t can_id, float rad_s)
{
    union
    {
        float    f;
        uint8_t  b[4];
    } u;
    uint8_t d[4];

    u.f = rad_s;
    d[0] = u.b[0];
    d[1] = u.b[1];
    d[2] = u.b[2];
    d[3] = u.b[3];
    CAN1_SendFrame((uint16_t)(0x200 + can_id), d, 4);
}

/* 参数写: [CANID_L, CANID_H, 0x55, RID, uint32 小端 x4]。 */
static void DM_ParamWrite(uint16_t can_id, uint8_t rid, uint32_t value)
{
    uint8_t d[8];

    d[0] = (uint8_t)(can_id & 0xFF);
    d[1] = (uint8_t)(can_id >> 8);
    d[2] = 0x55;
    d[3] = rid;
    d[4] = (uint8_t)(value & 0xFF);
    d[5] = (uint8_t)((value >> 8) & 0xFF);
    d[6] = (uint8_t)((value >> 16) & 0xFF);
    d[7] = (uint8_t)((value >> 24) & 0xFF);
    CAN1_SendFrame(DM_PARAM_ID, d, 8);
}

/*========================= 反馈帧解码 ================================
 * 8 字节布局 (DM 说明书):
 *   D0     = can_id | (ERR << 4)
 *   D1..D2 = POS 16 bit 大端
 *   D3..D4高4位 = VEL 12 bit
 *   D4低4位..D5 = T 12 bit
 *   D6 = MOS 温度, D7 = 线圈温度
 *
 * 12/16 bit 原始值按说明书公式线性映射到物理量。这里用 float 只做
 * 标定换算, 不参与任何控制 —— 控制量本来就是电机自己的速度环。
 *=====================================================================*/
static float uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x * span / (float)((1UL << bits) - 1UL)) + x_min;
}

static void DM_DecodeFeedback(uint8_t idx, const CanRxMsg *rx)
{
    uint32_t pos_i;
    uint32_t vel_i;
    uint32_t tq_i;

    if (rx->DLC < 8)
    {
        return;                         /* 长度不对, 宁可丢帧也不猜 */
    }

    g_motor[idx].err = (uint8_t)((rx->Data[0] >> 4) & 0x0F);

    pos_i = ((uint32_t)rx->Data[1] << 8) | (uint32_t)rx->Data[2];
    vel_i = ((uint32_t)rx->Data[3] << 4) | ((uint32_t)rx->Data[4] >> 4);
    tq_i = (((uint32_t)rx->Data[4] & 0x0F) << 8) | (uint32_t)rx->Data[5];

    g_motor[idx].pos_rad = uint_to_float(pos_i, -DM_PMAX, DM_PMAX, 16);
    g_motor[idx].vel_rad_s = uint_to_float(vel_i, -DM_VMAX, DM_VMAX, 12);
    g_motor[idx].tq_nm = uint_to_float(tq_i, -DM_TMAX, DM_TMAX, 12);

    g_motor[idx].mos_temp = rx->Data[6];
    g_motor[idx].coil_temp = rx->Data[7];

    g_last_fb_ms[idx] = g_tick_ms;
    g_fb_seen[idx] = 1;
    if (g_fb_count != 0xFF)
    {
        g_fb_count++;
    }
}

/* FIFO 里可能不止一帧, 必须一次读干净:
 * 50 Hz x 2 台 = 100 帧/s, 主循环 1 ms 一圈, 正常不会积压;
 * 但如果某圈被状态帧阻塞了 4 ms, 这里就要能补读。 */
static void CAN1_Poll(void)
{
    CanRxMsg rx;
    uint32_t id;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        id = (rx.IDE == CAN_Id_Extended) ? rx.ExtId : rx.StdId;

        if ((id == MOTOR1_MST_ID) && (rx.IDE == CAN_Id_Standard))
        {
            DM_DecodeFeedback(0, &rx);
        }
        else if ((id == MOTOR2_MST_ID) && (rx.IDE == CAN_Id_Standard))
        {
            DM_DecodeFeedback(1, &rx);
        }
        /* 其它 ID (例如 0x7FF 参数回帧) 直接忽略 */
    }
}

/*============================= 串口 ==================================
 * USART1 中断只负责把字节塞进环形缓冲, 主循环再组帧。
 * STM32F1 的 USART 没有 RX FIFO, 主循环发 44 字节状态帧时可能阻塞
 * 几毫秒, 不靠中断就会 ORE 丢命令字节。
 *=====================================================================*/
#define RX_RING_SIZE        64
#define USART_TX_TIMEOUT_MS 2           /* 单字节等 TXE 的硬上限, 正常只要 87 us */

static volatile uint8_t g_rx_ring[RX_RING_SIZE];
static volatile uint8_t g_rx_head;
static volatile uint8_t g_rx_tail;

void USART1_IRQHandler(void)
{
    /* ORE 和 RXNE 都要读 DR 才能清: 只处理 RXNE 会让 ORE 一直挂着。 */
    if ((USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) ||
        (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET))
    {
        uint8_t value = (uint8_t)USART_ReceiveData(USART1);
        uint8_t next = (uint8_t)((g_rx_head + 1) & (RX_RING_SIZE - 1));

        if (next != g_rx_tail)
        {
            g_rx_ring[g_rx_head] = value;
            g_rx_head = next;
        }
        /* 环形缓冲满: 丢最新字节。命令帧有 BCC, 丢字节后会自动重新同步。 */
    }
}

/* 发一个字节。返回 1 = 已写进 DR, 0 = 超时丢弃。
 *
 * ★ 这里是"发了 0x7B 就停"最值得怀疑的地方, 所以上限必须说清楚:
 *   旧写法是 `while (TXE == RESET && --timeout)` 配 20000 次循环。它确实
 *   有界, 但 -O0 下每次循环都要调用一次 USART_GetFlagStatus(), 20000 次
 *   约 2~4 ms/字节; 一帧 68 字节最坏能吃掉 150~270 ms, 主循环看起来就像
 *   "发到第 4 个字节卡住了"。现在改成 **2 ms 硬期限**(正常一字节只要
 *   87 us, 差 20 倍), 超时就返回 0, 调用方立刻放弃本帧 —— 下一帧重新
 *   同步, 绝不会停在半帧上。
 *
 *   注意: 真正"卡在第 4 个字节"还需要 MCU 本身被复位/挂死, 光靠本函数
 *   是做不到的; 现场那次 4 字节的实测原因是主机侧抢串口(见交付说明)。
 */
static uint8_t Port_SendByte(uint8_t value)
{
    uint32_t start = g_tick_ms;

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        if ((g_tick_ms - start) > USART_TX_TIMEOUT_MS)
        {
            return 0;
        }
    }
    USART_SendData(USART1, value);
    return 1;
}

static void USART1_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;                 /* TX */
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;                /* RX */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/*============================ 命令帧 ==================================
 * 11 字节, 大端:
 *   [0]=0x7B  [1]=AutoRecharge(忽略)  [2]=SecurityPLY(忽略)
 *   [3..4]=vx int16 mm/s  [5..6]=vy(忽略)  [7..8]=wz int16 mrad/s
 *   [9]=XOR(f[0..8])  [10]=0x7D
 *
 * 校验失败/头尾不对时**逐字节左移**重新找帧头。这样总线上丢一个字节
 * 之后, 最多 11 个字节就能重新对上, 不需要上位机复位。
 *=====================================================================*/
static uint8_t g_cmd_frame[CMD_FRAME_SIZE];
static uint8_t g_cmd_len;

/*===================== 主机功能帧 (v2 新增) ============================
 * 11 字节帧和速度帧同头同尾同校验, 靠 f[1] 区分:
 *   f[1] = 0x00/0x01: 普通速度帧 (0x01 是旧协议的"回充"标志, 本固件忽略)
 *   f[1] = 0x06:      FUNC_MOTOR_REINIT —— 主机要求重新"写速度模式 + 使能"
 *
 * 为什么需要: DM 驱动器上电后默认失能, 只能由 STM32 发 FFx7+0xFC 使能。
 * 而 STM32 只在**自己启动时**发一次(3 次重试, 总预算 400 ms)。整车断电
 * 重启时 STM32 往往比驱动器先起来, 使能帧就静默丢了 —— 之后整车一直失能,
 * 主机发什么速度都不动 (诊断帧 [18]/[19] ERR=0, flags 的使能位=0)。
 * 有了这条功能帧, 主机在"停车"状态下随时可以让 STM32 重新补一次使能,
 * 不再需要拆机按复位/重新上电。
 *
 * ★★ 硬安全约束: **只在车已停时接受**(目标为 0 且两轮实际转速很小)。
 *    跑着的时候重写模式会让输出瞬间消失 = 失控。
 *    主机的正确用法: 先发一帧全零速度帧, 再发这条功能帧。
 *=====================================================================*/
#define FUNC_MOTOR_REINIT          0x06
#define REINIT_MAX_WHEEL_RAD_S     2.0f    /* 轮速超过这个值认为车在动, 拒绝 */
static uint8_t  g_reinit_ok;               /* 成功执行次数 (饱和) */
static uint8_t  g_reinit_reject;           /* 因车没停被拒次数 (饱和) */

static void Motors_Startup(void);          /* 定义在文件后半, 这里先声明 */

static void Cmd_Apply(const uint8_t *f)
{
    /* 能进到这里说明帧头、帧尾、BCC 全对, 足以证明上位机活着。 */
    g_last_cmd_ms = g_tick_ms;

    /* ---- 主机功能帧: f[1]=FUNC_MOTOR_REINIT, 重新写模式 + 使能 ---- */
    if (f[1] == FUNC_MOTOR_REINIT)
    {
        /* 只在车已停时接受: 目标为 0(上一帧零速) 且两轮实际转速很小 */
        if ((g_cmd_vx_mm == 0) && (g_cmd_wz_mrad == 0) &&
            (g_eff_vx_mm == 0) && (g_eff_wz_mrad == 0) &&
            (g_motor[0].vel_rad_s > -REINIT_MAX_WHEEL_RAD_S) &&
            (g_motor[0].vel_rad_s < REINIT_MAX_WHEEL_RAD_S) &&
            (g_motor[1].vel_rad_s > -REINIT_MAX_WHEEL_RAD_S) &&
            (g_motor[1].vel_rad_s < REINIT_MAX_WHEEL_RAD_S))
        {
            Motors_Startup();
            if (g_reinit_ok != 0xFF)
            {
                g_reinit_ok++;
            }
        }
        else
        {
            if (g_reinit_reject != 0xFF)
            {
                g_reinit_reject++;
            }
        }
        return;                     /* 功能帧不当速度帧用, 目标保持原值 */
    }

    g_cmd_vx_mm = (int16_t)(((uint16_t)f[3] << 8) | f[4]);
    g_cmd_wz_mrad = (int16_t)(((uint16_t)f[7] << 8) | f[8]);

    /* f[1] AutoRecharge / f[2] SecurityPLY / f[5..6] vy 全部忽略:
       差速车没有横向自由度, 回充和安全防护由 RK3588 自己管。 */
}

static void Cmd_FeedByte(uint8_t value)
{
    uint8_t i;
    uint8_t bcc;

    if (g_cmd_len >= CMD_FRAME_SIZE)
    {
        /* 上一帧校验没过: 丢掉最老的一个字节, 窗口左移一位 */
        for (i = 1; i < CMD_FRAME_SIZE; i++)
        {
            g_cmd_frame[i - 1] = g_cmd_frame[i];
        }
        g_cmd_len = CMD_FRAME_SIZE - 1;
    }

    g_cmd_frame[g_cmd_len++] = value;

    if (g_cmd_len == CMD_FRAME_SIZE)
    {
        bcc = 0;
        for (i = 0; i < CMD_FRAME_SIZE - 2; i++)
        {
            bcc ^= g_cmd_frame[i];
        }

        if ((g_cmd_frame[0] == CMD_HEAD) &&
            (g_cmd_frame[CMD_FRAME_SIZE - 1] == CMD_TAIL) &&
            (bcc == g_cmd_frame[CMD_FRAME_SIZE - 2]))
        {
            Cmd_Apply(g_cmd_frame);
            g_cmd_len = 0;
        }
        /* 校验失败: 保留 11 字节, 下个字节进来时再左移重试 */
    }
}

static void Port_ProcessCommands(void)
{
    while (g_rx_tail != g_rx_head)
    {
        uint8_t value = g_rx_ring[g_rx_tail];
        g_rx_tail = (uint8_t)((g_rx_tail + 1) & (RX_RING_SIZE - 1));
        Cmd_FeedByte(value);
    }
}

/*=============================== LED =================================
 * PB9 心跳: 500 ms 翻转一次。灯停 = 主循环卡死, 这是现场最快的第一判据。
 * 上电时明确写一次灭灯, 否则引脚浮空会让灯先微亮一下, 像故障。
 *=====================================================================*/
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_Pin_9
#define LED_ON_LEVEL    1

/* 旧板上的滚刷/水泵继电器引脚。新固件**不驱动**它们 (继电器逻辑已删),
   但宏名保留: 旧接线图、旧工具和现场排线记录都按这两个名字引用, 删掉
   会让"这块板到底哪根线是继电器"失去唯一的代码依据。 */
#define RELAY_BRUSH_PIN GPIO_Pin_0
#define RELAY_PUMP_PIN  GPIO_Pin_1
#define RELAY_PORT      GPIOB

static uint32_t g_led_last_ms;
static uint8_t  g_led_on;

static void LED_Apply(uint8_t on)
{
#if LED_ON_LEVEL
    if (on)
    {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
    else
    {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    }
#else
    if (on)
    {
        GPIO_ResetBits(LED_PORT, LED_PIN);
    }
    else
    {
        GPIO_SetBits(LED_PORT, LED_PIN);
    }
#endif
}

static void LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &gpio);

    g_led_on = 0;
    g_led_last_ms = g_tick_ms;
    LED_Apply(0);
}

static void LED_Tick(void)
{
    if ((g_tick_ms - g_led_last_ms) >= LED_BLINK_MS)
    {
        g_led_last_ms = g_tick_ms;
        g_led_on = (uint8_t)(g_led_on ? 0 : 1);
        LED_Apply(g_led_on);
    }
}

/*=========================== 控制/状态更新 ============================*/
static float ClampF(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/* 浮点 -> int16, 带饱和。状态帧所有物理量都是 x100 定点的:
   omega x100 rad/s, torque x100 Nm, pos x100 rad。
   显式饱和是为了防止异常反馈 (比如温度/参数被改) 溢出后回绕成
   一个看起来合理的负数, 上位机会以为轮子在反转。 */
static int16_t ClampToI16(float v)
{
    if (v > 32767.0f)
    {
        v = 32767.0f;
    }
    else if (v < -32768.0f)
    {
        v = -32768.0f;
    }
    return (int16_t)v;
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

/*=========================== IMU (YbImu) =============================
 * 从旧 main 分支原样搬来的采样/有效性判定。按 20 ms 控制周期采一次,
 * 一次 YbImu_ReadMotion 大约 1~3 ms (PB10/PB11 位翻转 I2C)。
 *
 * 读失败**不清零** imu_accel_g/imu_gyro —— 保留上一次的有效值, 上位机
 * 通过 flags bit7 (以及诊断帧 [31] 的 YBIMU_ST_* 状态码) 判断可信度。
 * 上电后从未读成功时, 数组是 0, 遥测帧就报 0: 这是"没有数据", 不是
 * 编造出来的姿态。
 *
 * ★ IWDG 安全: YbImu.c 里每个等待都有上限 (SCL_WaitHigh 最多 2000 次,
 *   读写函数的每条错误分支都会补 STOP), 没有无超时忙等; 唯一耗时长的
 *   地址扫描 (0x08~0x77) 只在"车停着"且最多 1 Hz 时做。因此即使 IMU
 *   拔掉/总线短路, 主循环也能在几十毫秒内回到顶部继续喂狗 ——
 *   不会出现"I2C 读失败 -> 主循环卡死 -> 狗咬"。
 *
 * ★ 启动路径里**一次 I2C 都不碰**: 见下面的 IMU_BootInit()。第一次真正的
 *   读取发生在主循环 20 ms 任务里; 读失败只是 imu_ok=0, 帧照发。
 *=====================================================================*/

/* IMU 的"初始化"= 什么都不做, 只把两个诊断时间戳拨到当前。
 *
 * 为什么不在这里读一次/扫一次:
 *   - 位翻转 I2C 的单次等待都有界, 但"扫 0x08~0x77"最坏要 100 ms 量级;
 *     把它放在启动路径里, 就等于让一个可能拔掉的 IMU 决定整机上电时间。
 *   - 上电时 IMU 本身也可能还没启动完, 这时候读失败不代表坏了。
 * 所以启动阶段只记时间戳: 第一次总线检查/地址扫描被推迟到进主循环 1 s
 * 之后(而且仍然只在 g_eff_vx/wz == 0 的停车状态才扫), 寄存器体检同理。
 */
static void IMU_BootInit(void)
{
    imu_diag_ms = g_tick_ms;
    imu_probe_ms = g_tick_ms;
    imu_status = YBIMU_ST_OK;       /* 还没读过: 不冒充失败码, 真伪看 flags bit7 */
    imu_found_addr = 0;
}

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
        /* Z 轴方向在这里生效: 从上往下看逆时针(左转)时 z 应为正。
           装反了就改 IMU_GYRO_Z_SIGN = -1, 否则上位机 EKF 越纠越偏。 */
        imu_gyro[2] = gyro[2] * (float)IMU_GYRO_Z_SIGN;

        /* 静止时模块本来就把角速度归零, 所以"这一拍为 0"不是故障;
           只记录"这次成功读取时读到过非零"。 */
        if ((gyro[0] != 0.0f) || (gyro[1] != 0.0f) || (gyro[2] != 0.0f))
        {
            imu_gyro_ok = 1;
        }

        imu_ok = 1;
        imu_ever_ok = 1;
        imu_status = YBIMU_ST_OK;
        imu_found_addr = YBIMU_I2C_ADDR;
    }
    else
    {
        imu_ok = 0;
        imu_gyro_ok = 0;
        imu_status = err;

        /* 失败了才诊断, 而且不要每次都跑; 更**只在车停着的时候**扫描。
           地址扫描要遍历 112 个地址, 位翻转 I2C 全程 100~150 ms, 行驶中
           做会把 CAN 下发拖顿。诊断是给停车排查用的。 */
        if ((g_tick_ms - imu_diag_ms) >= IMU_DIAG_PERIOD_MS)
        {
            if ((g_eff_vx_mm == 0) && (g_eff_wz_mrad == 0))
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
                    imu_found_addr = YbImu_Scan();  /* 总线好 -> 查地址/器件 */
                    imu_status = (imu_found_addr == 0) ? YBIMU_ST_NO_ACK
                                                       : YBIMU_ST_READ_ERR;
                }
            }
            else
            {
                /* 在动, 先只记"读失败", 等停下来再重诊断 */
                imu_diag_ms = g_tick_ms - IMU_DIAG_PERIOD_MS + 100;
            }
        }
    }

    /* 寄存器体检 (1 Hz)。放最后: 它读得最长, 不能影响上面那次关键读取。 */
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
    }
}

/* 物理量 -> 协议规定的 ±2g / ±500dps 原始值, 带限幅(夹住, 不回绕)。 */
static int16_t IMU_AccelToRaw(float accel_g)
{
    float v = accel_g * IMU_ACCEL_G_TO_RAW;

    if (v > 32767.0f)
    {
        v = 32767.0f;
    }
    if (v < -32768.0f)
    {
        v = -32768.0f;
    }
    return (int16_t)v;
}

static int16_t IMU_GyroToRaw(float gyro_rad_s)
{
    float v = gyro_rad_s * IMU_GYRO_TO_RAW;

    if (v > 32767.0f)
    {
        v = 32767.0f;
    }
    if (v < -32768.0f)
    {
        v = -32768.0f;
    }
    return (int16_t)v;
}

/*======================= 24 字节主遥测帧 =============================
 * 旧协议, 20 Hz, 大端。**字段偏移/缩放必须和旧 main 分支完全一致**,
 * 以 tools/decode_diag.py 的 tel_battery_mv / tel_gyro_raw /
 * tel_accel_ms2 为准:
 *   [0]     0x7B
 *   [1]     Flag_Stop: 1 = 当前命令为停止 (超时也按停止算)
 *   [2..3]  int16 BE 车体 vx  mm/s   (左右轮实际转速正解, 不是目标)
 *   [4..5]  int16 BE 车体 vy  mm/s   (差速车恒 0)
 *   [6..7]  int16 BE 车体 wz  mrad/s (左右轮差速正解)
 *   [8..9]  int16 BE IMU ax 原始值   (±2g 量程; m/s² = 值 / 1671.84)
 *   [10..11]        ay
 *   [12..13]        az
 *   [14..15]        gx               (±500dps 量程; rad/s = 值 * 0.00026644)
 *   [16..17]        gy
 *   [18..19]        gz
 *   [20..21] uint16 BE 电池电压 mV   (本板无采样, 恒 0; 不编数)
 *   [22]    BCC = XOR([0..21])
 *   [23]    0x7D
 *
 * 为什么 [2..7] 报轮速正解而不是命令: 旧固件就是这么定义的, 上位机拿它
 * 和"目标"对比才能判断底盘有没有做到; 目标看 44 字节诊断帧 [10..13]。
 *===================================================================*/
static void Send_MainTelemetry(void)
{
    uint8_t f[TEL_FRAME_SIZE];
    uint8_t i;
    uint8_t bcc = 0;
    uint8_t left_fresh;
    uint8_t right_fresh;
    float v_left;
    float v_right;
    int16_t vx_mm;
    int16_t wz_mrad;

    /* 轮速用 200 ms 新鲜度(旧固件 RPM_STALE_MS), 不用控制环的 2 s 在线
       判定: 掉线的轮子按 0 算, 否则上报的是掉线前最后一帧的残值, 看起来像在动。 */
    left_fresh = (uint8_t)(g_fb_seen[0] &&
                           ((g_tick_ms - g_last_fb_ms[0]) <= WHEEL_STALE_MS));
    right_fresh = (uint8_t)(g_fb_seen[1] &&
                            ((g_tick_ms - g_last_fb_ms[1]) <= WHEEL_STALE_MS));

    /* ★ 上报的 vx/wz 由**实际反馈**反解, 所以反馈也要乘同一组方向系数 ——
       否则命令往前走、上报却说在往后。左右各乘各的, 镜像安装才不会算错。 */
    v_left = left_fresh ? (DM_DIR_LEFT * g_motor[0].vel_rad_s * WHEEL_R_M) : 0.0f;
    v_right = right_fresh ? (DM_DIR_RIGHT * g_motor[1].vel_rad_s * WHEEL_R_M) : 0.0f;

    vx_mm = ClampToI16((v_left + v_right) * 0.5f * 1000.0f);
    wz_mrad = ClampToI16((v_right - v_left) / TRACK_WIDTH_M * 1000.0f);

    f[0] = TEL_HEAD;
    f[1] = (uint8_t)(((g_eff_vx_mm == 0) && (g_eff_wz_mrad == 0)) ? 1 : 0);
    PutI16(&f[2], vx_mm);
    PutI16(&f[4], 0);                       /* vy: 差速车没有横向速度 */
    PutI16(&f[6], wz_mrad);

    /* IMU 六轴: 读失败时保留上一次有效值 (见 IMU_Tick), 上电未读到时为 0。 */
    PutI16(&f[8],  IMU_AccelToRaw(imu_accel_g[0]));
    PutI16(&f[10], IMU_AccelToRaw(imu_accel_g[1]));
    PutI16(&f[12], IMU_AccelToRaw(imu_accel_g[2]));
    PutI16(&f[14], IMU_GyroToRaw(imu_gyro[0]));
    PutI16(&f[16], IMU_GyroToRaw(imu_gyro[1]));
    PutI16(&f[18], IMU_GyroToRaw(imu_gyro[2]));

    /* 电池电压: 新板没有采样, 达妙反馈里也没有对应量。保留字段并填 0,
       上位机 decode_diag.py 对 0 的处理就是"不显示 bat="。 */
    PutU16(&f[20], 0);

    for (i = 0; i < TEL_FRAME_SIZE - 2; i++)
    {
        bcc ^= f[i];
    }
    f[TEL_FRAME_SIZE - 2] = bcc;
    f[TEL_FRAME_SIZE - 1] = TEL_TAIL;

    for (i = 0; i < TEL_FRAME_SIZE; i++)
    {
        if (!Port_SendByte(f[i]))
        {
            /* USART 状态异常: 放弃本帧, 下一帧重新发。绝不在这里拖住主循环 */
            return;
        }
    }
}

/* 1 ms 调一次, 把"命令超时"和"反馈在线"两个状态算好。
 *
 * ★ 命令超时只把速度清 0, **绝不发失能**:
 *   达妙失能 = 输出级关断 = 轮子自由滑行, 在坡上/被推时会自己溜车;
 *   而速度模式下的 0 目标是让驱动器主动把轮子刹住 (保持零速)。
 *   另外, 失能后再使能要走一次握手, 上位机恢复发送时会有明显延迟;
 *   保持使能、只改目标, 恢复是下一帧就生效。
 *
 * ★ 从没收到过反馈的电机也按 offline 处理:
 *   在确认电机状态之前不发速度, 宁可先停着。收到第一帧反馈后自动恢复。
 *=====================================================================*/
static void Bridge_Update(uint32_t now)
{
    g_cmd_timeout = ((now - g_last_cmd_ms) > CMD_TIMEOUT_MS) ? 1 : 0;

    if (g_cmd_timeout)
    {
        g_eff_vx_mm = 0;
        g_eff_wz_mrad = 0;
    }
    else
    {
        g_eff_vx_mm = g_cmd_vx_mm;
        g_eff_wz_mrad = g_cmd_wz_mrad;
    }

    g_online[0] = (uint8_t)((g_fb_seen[0] &&
                             ((now - g_last_fb_ms[0]) <= FB_TIMEOUT_MS)) ? 1 : 0);
    g_online[1] = (uint8_t)((g_fb_seen[1] &&
                             ((now - g_last_fb_ms[1]) <= FB_TIMEOUT_MS)) ? 1 : 0);
}

/* 20 ms 一拍: 差速逆解 + 限幅 + 下发。
 *
 *   v_left  = vx - wz * track / 2
 *   v_right = vx + wz * track / 2
 *   omega   = v / wheel_r
 *
 * 只限幅 omega, 不再对 vx/wz 另设软限幅 —— 单轮 30 rad/s 已经等价于
 * 2.57 m/s, 上层给再大的 vx 也会在这里被轮速上限吃掉。
 *=====================================================================*/
static void Control_Tick(void)
{
    float vx;
    float wz;
    float v_left;
    float v_right;

    vx = (float)g_eff_vx_mm / 1000.0f;      /* mm/s -> m/s */
    wz = (float)g_eff_wz_mrad / 1000.0f;    /* mrad/s -> rad/s */

    v_left = vx - wz * (TRACK_WIDTH_M * 0.5f);
    v_right = vx + wz * (TRACK_WIDTH_M * 0.5f);

    g_target_omega[0] = DM_DIR_LEFT * ClampF(v_left / WHEEL_R_M,
                               -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);
    g_target_omega[1] = DM_DIR_RIGHT * ClampF(v_right / WHEEL_R_M,
                               -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);

    /* 反馈超时的轮子目标清零: 驱动器可能已经掉线, 继续发旧目标没有意义,
       而且一旦它重新上线, 会立刻执行一个过期目标, 很危险。 */
    if (!g_online[0])
    {
        g_target_omega[0] = 0.0f;
    }
    if (!g_online[1])
    {
        g_target_omega[1] = 0.0f;
    }

    DM_SendSpeed(MOTOR1_CAN_ID, g_target_omega[0]);
    DM_SendSpeed(MOTOR2_CAN_ID, g_target_omega[1]);
}

/*=========================== 状态上报 ================================
 * 44 字节, 20 Hz, 大端。所有值都是"原始反馈", 不做滤波:
 *   [0] 0x7E
 *   [1] flags: bit0 左在线 bit1 右在线 bit2 左使能 bit3 右使能
 *              bit4 命令超时 bit5 左故障 bit6 右故障
 *              bit7 IMU 最近一次读取成功 (1 = 加速度有效)
 *        ★ 旧协议把 IMU 有效位放在 flags bit2/bit3; 本固件里 bit2/bit3
 *          已经分配给了"左/右使能", 不能复用(同一位置 1 会被上位机误读成
 *          使能)。所以 IMU 状态改放 bit7, 其余位含义一律不动。
 *   [2..3]  最近一次收到的 vx (mm/s) [4..5] 最近一次收到的 wz (mrad/s)
 *   [6..7]  左实际 omega x100        [8..9] 右实际 omega x100
 *   [10..11] 左目标 omega x100       [12..13] 右目标 omega x100
 *   [14..15] 左扭矩 x100 Nm          [16..17] 右扭矩 x100 Nm
 *   [18] 左 ERR  [19] 右 ERR
 *   [20] 左 MOS 温度  [21] 右 MOS 温度
 *   [22] 左线圈温度   [23] 右线圈温度
 *   [24..25] 左位置 x100 rad         [26..27] 右位置 x100 rad
 *   [28] CAN 发送失败计数 (饱和)     [29] 反馈帧总数 (饱和)
 *   [30] 版本号 = 2 (v2 起支持主机功能帧 f[1]=0x06 电机重新使能)
 *   [31] IMU 诊断码 YBIMU_ST_*: 0 正常 1 SCL拉不高 2 SDA拉不高
 *        3 总线死 4 无应答 5 读出错
 *        ★ 旧协议把它放 [27]/[28], 但本固件 [27] 是右位置高字节、[28] 是
 *          CAN 失败计数, 都已被占用; 所以改用原"保留 0"的 [31]。
 *   [32..33] 左反馈年龄 ms           [34..35] 右反馈年龄 ms
 *   [36..37] tick 低 16 位
 *   [38] IMU 扫描到的 I2C 地址 (0 = 没扫到)
 *   [39] IMU 体检标志: bit0 版本 bit1 陀螺 bit2 磁力
 *        bit7 = imu_gyro_ok (这次上电读到过非零角速度; 读失败会清)
 *   [40] IMU 版本号主字节
 *   [41] 启动阶段/错误码 (以前恒 0):
 *        0=启动中(串口上不会出现, 帧都在主循环里发) 1=正常
 *        2=CAN 初始化失败已降级  3=IMU 从没读到已跳过  4=两者都失败
 *        ★ 这一字节只做诊断, 不改任何已有字段的含义。
 *   [42] BCC = XOR(f[0..41])         [43] 0x7D
 *
 * 为什么把"目标"也报上去: 上位机只有同时看到"目标"和"实际", 才能区分
 * "电机没执行"和"我正在被轮速上限夹"。目标已经是逆解+限幅后的最终值。
 *=====================================================================*/
/* 启动阶段码, 只在诊断帧 [41] 里用:
 *   CAN 初始化结果(g_boot_stage) 与 "IMU 上电后有没有成功读过一次" 组合。
 *   imu_ever_ok 而不是 imu_ok: 偶发一次读失败不该把阶段码来回抖成 3。
 */
static uint8_t BootStage(void)
{
    if (!imu_ever_ok)
    {
        return (g_boot_stage == BOOT_STAGE_RUNNING) ? BOOT_STAGE_IMU_FAIL
                                                    : BOOT_STAGE_BOTH_FAIL;
    }
    return g_boot_stage;
}

static uint16_t FbAge(uint8_t idx, uint32_t now)
{
    uint32_t age;

    if (!g_fb_seen[idx])
    {
        return 0xFFFF;                  /* 从没收到过: 年龄报满, 一眼看出 */
    }

    age = now - g_last_fb_ms[idx];
    if (age > 0xFFFF)
    {
        return 0xFFFF;                  /* 饱和, 不回绕 */
    }
    return (uint16_t)age;
}

static void Status_Send(uint32_t now)
{
    uint8_t f[STATUS_FRAME_SIZE];
    uint8_t flags = 0;
    uint8_t bcc = 0;
    uint8_t i;

    if (g_online[0])
    {
        flags |= 0x01;
    }
    if (g_online[1])
    {
        flags |= 0x02;
    }
    if (g_online[0] && (g_motor[0].err == 1))
    {
        flags |= 0x04;
    }
    if (g_online[1] && (g_motor[1].err == 1))
    {
        flags |= 0x08;
    }
    if (g_cmd_timeout)
    {
        flags |= 0x10;
    }
    /* 故障位不要求"当前在线": 最后一次看到的故障仍然是最有价值的信息。
       电机掉线后若把故障位清掉, 上位机只会看到"不在线", 反而丢了故障线索。
       故障位保持到下一次收到 err<=1 的反馈才清。 */
    if (g_motor[0].err > 1)
    {
        flags |= 0x20;
    }
    if (g_motor[1].err > 1)
    {
        flags |= 0x40;
    }
    /* bit7 = IMU 最近一次读取成功。旧协议的 bit2/bit3 已被使能位占用,
       所以 IMU 的"活/不活"只能借用 bit7, 详见上面的帧格式说明。 */
    if (imu_ok)
    {
        flags |= 0x80;
    }

    f[0] = STATUS_HEAD;
    f[1] = flags;
    /* 这里报"最近一次收到的指令", 不是超时后的 0:
       上位机靠它确认自己发的命令到底有没有到; 命令超时由 bit4 单独表达,
       真正在执行的目标看 [10..13]。 */
    PutI16(&f[2], g_cmd_vx_mm);
    PutI16(&f[4], g_cmd_wz_mrad);
    PutI16(&f[6], ClampToI16(g_motor[0].vel_rad_s * 100.0f));
    PutI16(&f[8], ClampToI16(g_motor[1].vel_rad_s * 100.0f));
    PutI16(&f[10], ClampToI16(g_target_omega[0] * 100.0f));
    PutI16(&f[12], ClampToI16(g_target_omega[1] * 100.0f));
    PutI16(&f[14], ClampToI16(g_motor[0].tq_nm * 100.0f));
    PutI16(&f[16], ClampToI16(g_motor[1].tq_nm * 100.0f));
    f[18] = g_motor[0].err;
    f[19] = g_motor[1].err;
    f[20] = g_motor[0].mos_temp;
    f[21] = g_motor[1].mos_temp;
    f[22] = g_motor[0].coil_temp;
    f[23] = g_motor[1].coil_temp;
    PutI16(&f[24], ClampToI16(g_motor[0].pos_rad * 100.0f));
    PutI16(&f[26], ClampToI16(g_motor[1].pos_rad * 100.0f));
    f[28] = g_can_tx_fail;
    f[29] = g_fb_count;
    f[30] = STATUS_VERSION;
    f[31] = imu_status;
    PutU16(&f[32], FbAge(0, now));
    PutU16(&f[34], FbAge(1, now));
    PutU16(&f[36], (uint16_t)g_tick_ms);
    f[38] = imu_found_addr;
    f[39] = (uint8_t)(imu_probe_flags | (uint8_t)(imu_gyro_ok ? 0x80 : 0x00));
    f[40] = imu_ver_major;
    f[41] = BootStage();            /* 启动阶段/错误码, 见帧格式说明 */

    for (i = 0; i < STATUS_FRAME_SIZE - 2; i++)
    {
        bcc ^= f[i];
    }
    f[STATUS_FRAME_SIZE - 2] = bcc;
    f[STATUS_FRAME_SIZE - 1] = STATUS_TAIL;

    /* 阻塞发送, 但每个字节都有 2 ms 硬上限(见 Port_SendByte):
       115200 下 44 字节约 3.8 ms; 控制周期 20 ms, 留有余量。
       不引入 TX 中断/双缓冲, 是为了让这份"桥"尽量少的状态机。 */
    for (i = 0; i < STATUS_FRAME_SIZE; i++)
    {
        if (!Port_SendByte(f[i]))
        {
            return;                     /* 同上: 丢帧, 不拖住主循环 */
        }
    }
}

/*============================= 启动流程 ==============================
 * 达妙电机上电后按 Flash 里保存的模式工作, 出厂可能还是 MIT 模式。
 * 只发速度帧是不够的 —— MIT 模式会把 4 字节 float 当成别的字段解析,
 * 电机根本不会按速度转。所以每次上电必须:
 *   1. 写 CTRL_MODE (0x0A) = 3 (速度模式)
 *   2. 发使能帧
 *
 * 为什么每个都重复 3 次: CAN 只在链路层有 ACK, 应用层没有"参数已写入"
 * 的确认 (除非再读回)。驱动器上电/看门狗复位时可能还没准备好接收
 * 0x7FF 写帧, 单发一次会静默丢失。重复 3 次 + 30 ms 间隔, 覆盖这个窗口。
 *
 * ★ 这里**只发不等**: 不发读回、不等 0x7FF 应答, 所以不会出现"驱动器不理
 *   我 -> 启动卡住"。等待只来自 CAN1_SendFrame 的发送完成轮询(8000 次上限)
 *   和 30 ms 间隔。
 * ★ 总预算 400 ms (tick 计): 就算 HSE 没起来导致时基慢 9 倍、或者 CAN 每次
 *   都发不出去, 两轮重试也会在预算用完时提前收手, 保证启动一定走完。
 *   每轮之间都喂狗, 30 ms 的等待不会变成 IWDG 复位。
 *=====================================================================*/
static void Motors_Startup(void)
{
    uint8_t i;
    uint32_t start = g_tick_ms;

    for (i = 0; i < 3; i++)
    {
        DM_ParamWrite(MOTOR1_CAN_ID, DM_RID_CTRL_MODE, DM_MODE_VELOCITY);
        DM_ParamWrite(MOTOR2_CAN_ID, DM_RID_CTRL_MODE, DM_MODE_VELOCITY);
        IWDG_ReloadCounter();           /* 先喂狗再等, 缩短喂狗间隔 */
        Wait_Ms(30);
        if ((g_tick_ms - start) > 400u)
        {
            break;                      /* 启动预算用完: 剩下的重试不再做 */
        }
    }

    for (i = 0; i < 3; i++)
    {
        DM_SendEnable(MOTOR1_CAN_ID);
        DM_SendEnable(MOTOR2_CAN_ID);
        IWDG_ReloadCounter();
        Wait_Ms(30);
        if ((g_tick_ms - start) > 400u)
        {
            break;
        }
    }

    IWDG_ReloadCounter();
}

/*============================== main =================================
 * 启动顺序的原则: 每一步都有硬上限, 外设失败只降级、不阻塞。
 *
 *   Tick_Init        无等待
 *   CAN1_Init        标准库等 INAK 上限 0xFFFF 次; 失败返回 0, **不重试**
 *   0 速度帧        只等发送完成(8000 次上限), CAN 坏就计数
 *   USART1_Init     无等待
 *   LED_Init        无等待
 *   IWDG_Init       无等待
 *   Motors_Startup  3+3 次重试, 每轮喂狗; 总预算 400 ms, 只发不等应答
 *   IMU_BootInit    **不做任何 I2C 访问**, 只记时间戳(扫描推迟到主循环)
 *
 * 也就是说: 从复位到进主循环的最坏路径 ≈ CAN 初始化几 ms + 启动 400 ms,
 * 而且中间反复喂狗; 无论 IMU 插没插、CAN 通不通, 都一定能进主循环开始
 * 发 0x7B/0x7E 帧。进不去只可能是时钟/供电这种更底层的问题 —— 那种情况下
 * 44 字节帧的 [41] 也帮不上忙, 但至少不会"看起来像外设卡死"。
 *===================================================================*/
int main(void)
{
    uint32_t now;
    uint32_t last_ctrl_ms;
    uint32_t last_status_ms;

    Tick_Init();            /* 先有时基, 后面所有调度/超时都靠它 */

    /* CAN 初始化失败不拦启动: 标记 boot_stage=2, 帧照发, canFail 会反映。 */
    g_can_init_ok = CAN1_Init();            /* 1 Mbps, 标准帧 */
    g_boot_stage = g_can_init_ok ? BOOT_STAGE_RUNNING : BOOT_STAGE_CAN_FAIL;

    /* ★ CAN 一通先发 0 速度, 不等 USART/LED/看门狗初始化:
       看门狗复位时驱动器还在执行复位前最后一条速度命令, 每多等 1 ms
       车就多冲一点。DM 在速度模式下收到 0 会主动刹住; 若它还在 MIT
       模式, 这个 4 字节帧会被忽略, 不会造成动作。 */
    if (g_can_init_ok)
    {
        DM_SendSpeed(MOTOR1_CAN_ID, 0.0f);
        DM_SendSpeed(MOTOR2_CAN_ID, 0.0f);
    }

    USART1_Init();          /* PA9/PA10, 115200, 中断收 */
    LED_Init();             /* PB9 心跳 */
    IWDG_Init();            /* 开狗; 后面初始化步骤都会喂 */
    IWDG_ReloadCounter();

    /* 写模式 + 使能。最长 400 ms, 每轮喂狗, 里面不等任何应答。 */
    Motors_Startup();

    /* IMU: 启动阶段**不读、不扫**, 只把诊断时间戳拨到现在。
       第一次真正读取在主循环 20 ms 任务里; 失败只是 imu_ok=0。 */
    IMU_BootInit();
    IWDG_ReloadCounter();

    /* 时基起点。启动期间上位机可能已经发了命令 (在环形缓冲里), 主循环
       第一圈就会解析并刷新 g_last_cmd_ms。 */
    g_last_cmd_ms = g_tick_ms;
    last_ctrl_ms = g_tick_ms;
    last_status_ms = g_tick_ms;

    while (1)
    {
        IWDG_ReloadCounter();       /* 主循环活着 = 喂狗 */

        LED_Tick();                 /* 灯停 = 主循环卡死, 现场第一判据 */

        Port_ProcessCommands();     /* 收命令 -> 组帧 -> 更新 g_cmd_* */
        CAN1_Poll();                /* 收反馈 -> 更新 g_motor[] */

        now = g_tick_ms;
        Bridge_Update(now);         /* 命令超时 / 在线判断, 每 1 ms 刷新 */

        /* ★ 上电后前 10 秒补使能: 驱动器可能比 STM32 晚就绪, 启动时那 3 次
           "写模式+使能"会静默丢掉(使能帧没有应答, 丢了不会重发)。这里在
           "车停 + 仍失能 + 至少有一个驱动器在线"时每秒补一次, 最多补到上电
           10 秒。这解决了"整车断电重启后车一直不动 (ERR=0 失能)"的竞争,
           不需要再拆机按复位/重新上电。 */
        if ((now < 10000u) && (g_boot_stage == BOOT_STAGE_RUNNING) &&
            ((g_motor[0].err == 0) || (g_motor[1].err == 0)) &&
            (g_online[0] || g_online[1]) &&
            (g_eff_vx_mm == 0) && (g_eff_wz_mrad == 0) &&
            ((now - g_boot_reinit_ms) >= 1000u))
        {
            g_boot_reinit_ms = now;
            Motors_Startup();
        }

        if ((now - last_ctrl_ms) >= CTRL_PERIOD_MS)
        {
            last_ctrl_ms = now;
            IMU_Tick();             /* 20 ms 采一次 IMU (旧固件 ODOM_PERIOD_MS) */
            Control_Tick();         /* 逆解 + 限幅 + 两帧速度命令 */
        }

        /* 上报顺序不能反: 先 24 字节主遥测帧(0x7B), 再 44 字节诊断帧(0x7E)。
           这样诊断帧永远夹在两个完整遥测帧之间, 接收端按"上一字节是帧尾
           0x7D、本字节是帧头"同步时能把它整帧跳过, 下一个 0x7B 仍能入帧。 */
        if ((now - last_status_ms) >= STATUS_PERIOD_MS)
        {
            last_status_ms = now;
            Send_MainTelemetry();   /* 24 字节, 含 IMU 六轴 */
            Status_Send(now);       /* 44 字节诊断帧 */
        }

        /* 1 ms 让出 CPU。用 Wait_Ms 而不是 Delay_ms:
           后者写死 72 MHz, HSE 没起来时会慢 9 倍(1 ms -> 9 ms), 整个调度
           节拍跟着歪; Wait_Ms 走 TIM2 时基, 而且有 guard 上限。 */
        Wait_Ms(1);
    }
}
