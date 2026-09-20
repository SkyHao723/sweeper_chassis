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
 *   PB0 / PB1                   -->  电机继电器 / 水泵继电器
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
 *   [1]     flags  bit0 曾收到命令  bit1 看门狗已停车  bit2 IMU 有效
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
 *   [26]    继电器状态  bit0 电机 bit1 水泵
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

/*---------------------- 串口 / 看门狗 ----------------------------
 *   只有一个上位机: USART1 (PA9/PA10) -> CH340 / RK3588。
 *   LINK_TIMEOUT_MS: 上位机这么久不发命令就自动停车(看门狗)。
 *-----------------------------------------------------------------*/
#define MAIN_BAUD           115200  /* 对齐厂商驱动, 别改 */
#define LINK_TIMEOUT_MS     800

/*---------------------- GPIO 继电器 (外设开关) ----------------------
 *   PB0 -> 电机继电器 IN     PB1 -> 水泵继电器 IN
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
#define RELAY_MOTOR_PIN     GPIO_Pin_0      /* PB0 */
#define RELAY_PUMP_PIN      GPIO_Pin_1      /* PB1 */
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
 *  以前固定发 0, 目标转速是瞬间跳变的, 电流冲击最大 —— 起步那一下最容易
 *  顶到过流保护, 车也窜。给一点加速斜坡既柔和又更不容易触发保护:
 *      ACCEL_STEP_MS = 3  ->  0 到 100RPM 约 300ms, 0 到 35RPM 约 105ms
 *  减速保持 0: 要停就立刻停, 别拖泥带水(停车安全优先)。
 *
 *  想恢复原行为(纯阶跃)把 ACCEL_STEP_MS 改回 0 即可。 */
#define ACCEL_STEP_MS      3
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
#define FUNC_RELAY         0x05     /* f[2] = 掩码: bit0 电机继电器 bit1 水泵继电器 */

/*-------------------------- 周期 --------------------------------*/
#define CAN_PERIOD_MS      20       /* 向电机重复发送的周期 */
#define ODOM_PERIOD_MS     20       /* 轮速采样/(可选)卡尔曼周期 (50Hz) */
#define TELEMETRY_PERIOD_MS 50      /* 上报周期 (20Hz) */
#define RPM_STALE_MS       200      /* 驱动器这么久没回码就认为转速无效 */

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
#define IMU_GYRO_Z_SIGN     1       /* 陀螺 Z 方向和车体逆时针不一致时改成 -1 */

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

/* 物理轮速目标 —— 上报给上位机, 用来和实际轮速对比(诊断"没劲"的关键) */
static int16_t  target_left_rpm;
static int16_t  target_right_rpm;

static uint32_t last_cmd_ms;
static uint32_t last_can_ms;
static uint32_t last_odom_ms;
static uint32_t last_tel_ms;

/* 诊断计数 */
static uint8_t  can_error_count;

/* 驱动器回码 (控制帧 ...E601 的应答, 每个字段来自同一帧, 时间戳共用 m*_rpm_ms) */
static volatile uint8_t m1_fault, m2_fault;     /* DATA1 故障码, 见 FOC_FAULT_* */
static volatile uint8_t m1_mode,  m2_mode;      /* DATA0 当前运行模式 0x05/0x06... */
static volatile int16_t m1_rpm,   m2_rpm;       /* DATA2/3 实际转速 RPM */
static volatile int16_t m1_cur,   m2_cur;       /* DATA4/5 输出扭矩电流, A x100 */
static uint32_t m1_rpm_ms, m2_rpm_ms;
static volatile uint16_t battery_mv;

/* 轮速计 */
static int16_t  wheel_left_rpm, wheel_right_rpm;   /* 已按接线校准的物理轮速 */
static float    body_vx, body_wz;                  /* 轮速正解出的车体速度 */

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

/* 继电器 */
static uint8_t  relay_state;                       /* bit0 电机 bit1 水泵 */

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
             DATA4/5=当前输出扭矩电流(A x100)  DATA6/7=当前位置(度)
           DATA4/5 是判断"这个轮子到底出没出力"唯一的客观指标:
           目标转速不等于实际转速时, 看电流就知道是驱动器没给力(电流≈0)
           还是给了力但被堵住/拖住了(电流很大)。 */
        if ((id == MOTOR1_REPLY_ID) && (rx.DLC >= 4))
        {
            m1_mode  = rx.Data[0];
            m1_fault = rx.Data[1];
            m1_rpm = (int16_t)(((uint16_t)rx.Data[2] << 8) | rx.Data[3]);
            m1_rpm_ms = g_tick_ms;
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

static void Port_SendByte(uart_port_t *p, uint8_t value)
{
    while (USART_GetFlagStatus(p->usart, USART_FLAG_TXE) == RESET)
    {
    }
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
    uint8_t motor_on = (relay_state & 0x01) != 0;
    uint8_t pump_on  = (relay_state & 0x02) != 0;

#if RELAY_ACTIVE_LOW
    motor_on = (uint8_t)(!motor_on);
    pump_on  = (uint8_t)(!pump_on);
#endif

    if (motor_on) GPIO_SetBits(RELAY_PORT, RELAY_MOTOR_PIN);
    else          GPIO_ResetBits(RELAY_PORT, RELAY_MOTOR_PIN);

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

    gpio.GPIO_Pin = RELAY_MOTOR_PIN | RELAY_PUMP_PIN;
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
 *=================================================================*/
static void Cmd_Apply(uint8_t port_id, const uint8_t *f)
{
    int16_t vx_mm;
    int16_t wz_mrad;

    /* ---- 功能帧: f[1] 是功能码, 不是速度帧 ---- */
    if (f[1] == FUNC_RELAY)
    {
        Relay_Set(f[2]);            /* f[2] = 掩码 bit0 电机 bit1 水泵 */
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

    /* ★ 电机继电器必须有人自动管。
       继电器帧(f[1]=0x05)是本工程自定义的扩展, 厂商 ROS 节点不认识、永远
       不会发 —— 如果只靠上位机显式合闸, 换成 RK3588 当上位机之后电机永远
       没有电, 而 STM32 照样会发 CAN 转速命令, 现象是"目标转速有、实际为 0、
       电流≈0", 和"驱动器没给力"在诊断上完全分不出来。
       所以策略改成: 只要收到有效命令就自动合上电机继电器, 断链时由看门狗
       自动断开(见 main 里的 Relay_Set(0)) —— 不需要上位机配合, 车也不会在
       断链后自己复活。

       ★ 水泵**不**自动合闸: 误抽水的代价太大, 只能由上位机显式发 0x05 帧。 */
    if ((relay_state & 0x01) == 0)
    {
        Relay_Set((uint8_t)(relay_state | 0x01));
    }

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
    float v_left;
    float v_right;

    /* 驱动器掉线时实际转速会一直停在最后一个值, 必须当成 0, 否则会飘 */
    if ((g_tick_ms - m1_rpm_ms) > RPM_STALE_MS) a = 0;
    if ((g_tick_ms - m2_rpm_ms) > RPM_STALE_MS) b = 0;

    /* 驱动器报的是"命令方向"上的转速, 按安装校准换算成物理方向 */
    if (MOTOR1_INVERT) a = (int16_t)(-a);
    if (MOTOR2_INVERT) b = (int16_t)(-b);

    if (MOTOR1_IS_LEFT)
    {
        wheel_left_rpm = a;
        wheel_right_rpm = b;
    }
    else
    {
        wheel_left_rpm = b;
        wheel_right_rpm = a;
    }

    /* 正向运动学: 左右轮线速度 -> 车体速度 (协议里上报的就是这个) */
    v_left  = (float)wheel_left_rpm  * WHEEL_CIRC_M / 60.0f;
    v_right = (float)wheel_right_rpm * WHEEL_CIRC_M / 60.0f;
    body_vx = (v_left + v_right) * 0.5f;
    body_wz = (v_right - v_left) / TRACK_WIDTH_M;
}

/*========================= 差速逆解 ================================
 * 车体 (vx, wz) -> 左右轮目标转速。差速车用不到 vy。
 *   v_left  = vx - wz * b/2
 *   v_right = vx + wz * b/2
 *=================================================================*/
static void Drive_Apply(void)
{
    float v_left = target_vx - target_wz * (TRACK_WIDTH_M * 0.5f);
    float v_right = target_vx + target_wz * (TRACK_WIDTH_M * 0.5f);
    float rpm_l = v_left / WHEEL_CIRC_M * 60.0f;
    float rpm_r = v_right / WHEEL_CIRC_M * 60.0f;
    int16_t m1;
    int16_t m2;

    rpm_l = ClampF(rpm_l, -(float)MAX_RPM, (float)MAX_RPM);
    rpm_r = ClampF(rpm_r, -(float)MAX_RPM, (float)MAX_RPM);

    /* 物理轮速 -> CAN 命令 (反向应用接线校准) */
    if (MOTOR1_IS_LEFT) { m1 = (int16_t)rpm_l; m2 = (int16_t)rpm_r; }
    else                { m1 = (int16_t)rpm_r; m2 = (int16_t)rpm_l; }
    if (MOTOR1_INVERT) m1 = (int16_t)(-m1);
    if (MOTOR2_INVERT) m2 = (int16_t)(-m2);

    target_left_rpm = (int16_t)rpm_l;      /* 上报给上位机的是物理轮速目标, */
    target_right_rpm = (int16_t)rpm_r;     /* 不是电机命令, 免得左右概念混淆 */

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
        imu_gyro[2] = gyro[2];

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

        /* 失败了才诊断, 而且不要每次都跑(扫描很慢) */
        if ((g_tick_ms - imu_diag_ms) >= IMU_DIAG_PERIOD_MS)
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
    if (imu_gyro_ok)      flags |= 0x08;    /* 陀螺仪不是恒 0(不是又没接上) */

    f[0] = DIAG_HEAD;
    f[1] = flags;
    PutI16(&f[2],  (int16_t)(body_vx * 1000.0f));   /* 轮速正解出的车体速度 */
    PutI16(&f[4],  (int16_t)(body_wz * 1000.0f));
    PutI16(&f[6],  wheel_left_rpm);      /* 实际 */
    PutI16(&f[8],  wheel_right_rpm);
    PutI16(&f[10], target_left_rpm);     /* 目标 */
    PutI16(&f[12], target_right_rpm);
    PutI16(&f[14], m1_cur);              /* 1号驱动器 输出扭矩电流 A×100 */
    PutI16(&f[16], m2_cur);              /* 2号驱动器 */
    f[18] = m1_fault;                    /* 1号故障码, 见 FOC_FAULT_* */
    f[19] = m2_fault;
    f[20] = m1_mode;                     /* 1号当前模式 0x05 速度 / 0x06 位置 */
    f[21] = m2_mode;
    f[22] = can_error_count;
    f[23] = ports[PORT_MAIN].bad_count;
    f[24] = ports[PORT_MAIN].ok_count;
    f[25] = ports[PORT_MAIN].overflow;
    f[26] = relay_state;                 /* bit0 电机 bit1 水泵 */
    f[27] = imu_status;                  /* YBIMU_ST_* */
    f[28] = imu_found_addr;              /* 0 = 没扫到 */
    f[29] = seq++;                       /* 帧序号, 上位机据此发现丢帧 */
    f[30] = imu_probe_flags;             /* 各功能块活没活 */
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
    USART1_Init();          /* PA9/PA10 -> CH340 / RK3588, 唯一的上位机口 */
    Relay_Init();
    Delay_ms(200);

    target_vx = 0.0f;
    target_wz = 0.0f;
    /* 上电先刹住, 等上位机发命令 */
    CAN1_SendStop(MOTOR1_CAN_ID);
    CAN1_SendStop(MOTOR2_CAN_ID);

    last_can_ms = g_tick_ms;
    last_odom_ms = g_tick_ms;
    last_tel_ms = g_tick_ms;
    last_cmd_ms = g_tick_ms;

    while (1)
    {
        Port_ProcessCommands(&ports[PORT_MAIN]);
        CAN1_Poll();

        /* 看门狗: 上位机 800ms 不发命令就自动停车, 并断开继电器。
           只有一个上位机, 它的帧直接喂狗, 不需要仲裁。 */
        if ((failsafe_latched == 0) &&
            (ever_linked) &&
            ((g_tick_ms - last_cmd_ms) > LINK_TIMEOUT_MS))
        {
            target_vx = 0.0f;
            target_wz = 0.0f;
            failsafe_latched = 1;
#if RELAY_OFF_ON_LINK_LOSS
            Relay_Set(0);       /* 断线了顺便把电机/水泵继电器也断开 */
#endif
        }

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
