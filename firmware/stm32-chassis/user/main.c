#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "Delay.h"
#include "YbImu.h"
#include <math.h>

/*==========================================================================
 * 两驱差速小车 —— STM32 底盘板
 *
 *   USART1  PA9(TX) / PA10(RX)  <->  主上位机 CH340 / RK3588   115200 8N-1
 *   USART2  PA2(TX) / PA3(RX)   <->  调试用 ESP32              115200 8N-1
 *   CAN1    PA12(TX)/ PA11(RX)  -->  两台 FOC 驱动器            500 kbps 扩展帧
 *
 * 两个串口跑**同一套协议**，谁最近发了非零命令谁就拿到控制权，
 * 主口(USART1)优先级更高，可以随时抢占。详见下面"控制权仲裁"。
 *
 * STM32 负责:
 *   1. 解析上位机发来的车体目标速度 (vx, vy, wz)
 *   2. 差速逆解 -> 左右轮目标转速 -> CAN 下发
 *   3. 轮速正解 -> 车体速度, 按协议上报
 *   4. 看门狗: 当前控制方静默 800ms 就自动停车
 *   5. (可选) 本地扩展卡尔曼滤波, 见 EKF_ON_STM32
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
 * 【主遥测帧 24 字节  STM32 -> 上位机】  (完全按协议标准, 两个口都发)
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
 * 【里程计帧 34 字节  STM32 -> 上位机】  (本工程扩展, 头 0x7E)
 *   厂商驱动不认识这一帧, 默认只发给调试口(ESP32), 见 ODOM_FRAME_TO_MAIN。
 *   [0]    0x7E
 *   [1]    flags  bit0 里程计已收敛  bit1 看门狗停车  bit2 曾收到命令
 *   [2-5]  int32 BE x   mm
 *   [6-9]  int32 BE y   mm
 *   [10-11] int16 BE 航向角  单位 0.01 度
 *   [12-13] int16 BE 滤波后 vx   mm/s
 *   [14-15] int16 BE 滤波后 wz   mrad/s
 *   [16-17] int16 BE 左轮物理转速 RPM
 *   [18-19] int16 BE 右轮物理转速 RPM
 *   [20-21] int16 BE 左轮目标 RPM
 *   [22-23] int16 BE 右轮目标 RPM
 *   [24]   1号驱动器故障码
 *   [25]   2号驱动器故障码
 *   [26]   CAN 发送错误计数
 *   [27]   UART 坏帧计数
 *   [28]   UART 有效命令计数
 *   [29]   UART 溢出计数
 *   [30]   继电器状态  bit0 电机 bit1 水泵
 *   [31]   当前控制方  0=无 1=主口(CH340/RK3588) 2=从口(ESP32)
 *   [32]   IMU 诊断码  0=正常 1=SCL拉不高 2=SDA拉不高 3=总线死 4=无应答 5=读出错
 *   [33]   IMU 扫描到的 I2C 地址 (0 = 没扫到)
 *   [34]   BCC = XOR([0..33])
 *   [35]   0x7D
 *========================================================================*/

/*========================= 想改的地方 =========================*/
#define WHEEL_DIAMETER_MM   205.0f  /* 车轮直径 mm */
#define TRACK_WIDTH_MM      930.0f  /* 左右轮触地间距 mm */

#define MAX_RPM             200     /* 单轮转速上限 */
#define MAX_YAW_RATE        3.0f    /* 车体角速度限幅 rad/s */

#define MOTOR1_IS_LEFT         1    /* 1=1号电机在左侧; 若左右反了改成 0 */
#define MOTOR1_INVERT          0    /* 某个轮子转向相反时改成 1 */
#define MOTOR2_INVERT          1    /* 实测2号电机正方向是反的 */

#define STOP_CTRL   CTRL_BRAKE      /* 停车用刹车; 想滑行改成 CTRL_DISABLE */

/*---------------------- 双串口 / 控制权 ----------------------------
 *   PORT_MAIN (USART1, PA9/PA10) -> CH340 / RK3588   主口, 可以抢占
 *   PORT_AUX  (USART2, PA2/PA3)  -> 调试用 ESP32     从口, 主口空闲时才能拿
 *
 *   OWNER_TIMEOUT_MS: 控制方多久不发命令就释放控制权(让另一个口能接管)。
 *                     必须比 LINK_TIMEOUT_MS 短, 否则交接时车会先被看门狗刹停。
 *   LINK_TIMEOUT_MS : 当前控制方多久不发命令就自动停车(看门狗)。
 *-----------------------------------------------------------------*/
#define MAIN_BAUD           115200  /* 对齐厂商驱动, 别改 */
#define AUX_BAUD            115200  /* ESP32 那个口, 想提速可以改 */
#define OWNER_TIMEOUT_MS    500
#define LINK_TIMEOUT_MS     800

/*---------------------- 本地卡尔曼(可选) ---------------------------
 *   EKF_ON_STM32 = 1 : STM32 自己积分位姿 + 卡尔曼滤波, 发 0x7E 位姿帧。
 *                      调试阶段用 ESP32 网页看轨迹时需要这个。
 *   EKF_ON_STM32 = 0 : EKF 在上位机(RK3588)做, STM32 只按协议报车体速度。
 *                      接上 RK3588 之后改成 0, 省 CPU 也省串口带宽。
 *-----------------------------------------------------------------*/
#define EKF_ON_STM32        1
#define ODOM_FRAME_TO_MAIN  0   /* 位姿帧是本工程扩展, 厂商驱动不认识, 默认只发调试口。ESP32 还没挪到 PA2/PA3 时改成 1 */

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
 *   换模块后如果网页上"开/关"反了，把这个值改成 0 即可。
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
#define CTRL_ENABLE        0x01
#define CTRL_DISABLE       0x02
#define CTRL_BRAKE         0x03

/*-------------------------- 协议 --------------------------------*/
#define CMD_FRAME_SIZE     11
#define CMD_HEAD           0x7B
#define TEL_FRAME_SIZE     24
#define TEL_HEAD           0x7B
#define TEL_TAIL           0x7D
#define ODOM_FRAME_SIZE    36
#define ODOM_HEAD          0x7E
#define ODOM_TAIL          0x7D

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

/*=================== 卡尔曼滤波参数 (调试时可改) ===================*/
#define EKF_SIGMA_V        0.05f    /* 轮速测量噪声 m/s */
#define EKF_SIGMA_W        0.10f    /* 轮速角速度测量噪声 rad/s */
#define EKF_SIGMA_GZ       0.01f    /* 陀螺仪测量噪声 rad/s */
#define EKF_SLIP_DIST      0.03f    /* 每米行驶距离的位置不确定度 */
#define EKF_SLIP_ANG       0.05f    /* 每弧度转过的航向不确定度 */
#define EKF_ACCEL_NOISE    0.30f    /* 线加速度随机游走 (m/s^2) */
#define EKF_ANGACC_NOISE   1.00f    /* 角加速度随机游走 (rad/s^2) */
#define EKF_BIAS_WALK      0.001f   /* 陀螺零偏随机游走 (rad/s^2) */
/*=================================================================*/

/*=========================== 全局状态 ==============================*/
volatile uint32_t g_tick_ms;

/*---------------------- 串口口 (两个口跑同一套协议) -----------------*/
#define PORT_MAIN   0
#define PORT_AUX    1
#define PORT_COUNT  2

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

/* 控制权: 谁最近发了"非零"命令谁说了算; 主口可以抢占 */
static uint8_t  ctrl_owner;         /* 0xFF = 无 */
static uint32_t ctrl_owner_ms;
#define OWNER_NONE  0xFF

/* 目标 / 状态 */
static float    target_vx;          /* m/s */
static float    target_wz;          /* rad/s */
static uint8_t  ever_linked;
static uint8_t  failsafe_latched;

static int16_t  target_left_rpm;
static int16_t  target_right_rpm;

static uint32_t last_cmd_ms;
static uint32_t last_can_ms;
static uint32_t last_odom_ms;
static uint32_t last_tel_ms;

/* 诊断计数 */
static uint8_t  can_error_count;

/* 驱动器回码 */
static volatile uint8_t m1_fault, m2_fault;
static volatile int16_t m1_rpm, m2_rpm;
static uint32_t m1_rpm_ms, m2_rpm_ms;
static volatile uint16_t battery_mv;

/* 轮速计 */
static int16_t  wheel_left_rpm, wheel_right_rpm;   /* 已按接线校准的物理轮速 */
static float    body_vx, body_wz;                  /* 轮速正解出的车体速度 */

/* IMU (YbImu, 位翻转 I2C on PB10/PB11) */
static float    imu_accel_g[3];      /* 单位 g */
static float    imu_gyro[3];         /* 单位 rad/s */
static uint8_t  imu_ok;              /* 1 = 加速度有效 */
static uint8_t  imu_gyro_ok;         /* 1 = 陀螺仪有效(不是恒 0) */
static uint8_t  imu_status;          /* YBIMU_ST_* 失败原因 */
static uint8_t  imu_found_addr;      /* 扫描到的器件地址, 0 = 没扫到 */
static uint32_t imu_diag_ms;         /* 上次诊断的时刻 */

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

/* 给一台电机发"速度模式 + 控制字 + 目标转速" */
static void CAN1_SendMotor(uint32_t id, uint8_t ctrl, int16_t rpm)
{
    uint8_t data[8];
    uint16_t raw = (uint16_t)rpm;

    data[0] = MODE_SPEED;
    data[1] = ctrl;
    data[2] = (uint8_t)(raw >> 8);
    data[3] = (uint8_t)raw;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    CAN1_SendRaw(id, 8, data);
}

static void CAN1_Poll(void)
{
    CanRxMsg rx;
    uint32_t id;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        id = (rx.IDE == CAN_Id_Extended) ? rx.ExtId : rx.StdId;

        /* 控制帧回码 ...E601: DATA1=故障码, DATA2/3=实际转速 */
        if ((id == MOTOR1_REPLY_ID) && (rx.DLC >= 4))
        {
            m1_fault = rx.Data[1];
            m1_rpm = (int16_t)(((uint16_t)rx.Data[2] << 8) | rx.Data[3]);
            m1_rpm_ms = g_tick_ms;
        }
        else if ((id == MOTOR2_REPLY_ID) && (rx.DLC >= 4))
        {
            m2_fault = rx.Data[1];
            m2_rpm = (int16_t)(((uint16_t)rx.Data[2] << 8) | rx.Data[3]);
            m2_rpm_ms = g_tick_ms;
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

/*======================== 双串口 (两个口同一套协议) =================
 * 每收到一个字节就进中断塞进各自的环形缓冲, 主循环再慢慢组帧。
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

void USART2_IRQHandler(void)
{
    Port_Isr(&ports[PORT_AUX]);
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

/* USART2: PA2(TX) / PA3(RX)  ->  调试用 ESP32 */
static void USART2_Init(void)
{
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    Usart_InitPins(GPIO_Pin_2, GPIO_Pin_3);
    Usart_Config(USART2, AUX_BAUD);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    ports[PORT_AUX].id = PORT_AUX;
    ports[PORT_AUX].usart = USART2;
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

static void PutI32(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u >> 24);
    p[1] = (uint8_t)(u >> 16);
    p[2] = (uint8_t)(u >> 8);
    p[3] = (uint8_t)u;
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

/*==================== 控制权仲裁 (两个串口共用一个底盘) =============
 * 问题: 主口(RK3588)和从口(ESP32)都能发速度帧, 谁说了算?
 *   - 主口是长期上位机, 优先级高
 *   - 从口是调试用的, 主口不在的时候才能接管
 *
 * 规则:
 *   1. 只有"非零"速度帧才claim控制权(零速度是停车/心跳, 不抢)
 *   2. 主口随时可以抢; 从口只能在"主口不是控制方"或"主口控制权已超时"时拿
 *   3. 控制权 500ms 不刷新就释放(OWNER_TIMEOUT_MS)
 *   4. 不是控制方的速度帧直接丢弃
 *   5. 看门狗只认控制方的帧 —— 否则从口的心跳会把看门狗一直喂着,
 *      主口掉线了车也不会停, 这是会出事的
 *   6. 继电器是"锁存式开关", 两个口都随时可操作, 不参与仲裁
 *=================================================================*/
static void Cmd_Apply(uint8_t port_id, const uint8_t *f)
{
    int16_t vx_mm;
    int16_t wz_mrad;
    uint8_t nonzero;

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
    nonzero = (uint8_t)((vx_mm != 0) || (wz_mrad != 0));
    if (ports[port_id].ok_count != 0xFF) ports[port_id].ok_count++;

    /* ---- 抢占 / 续期 ---- */
    if (nonzero)
    {
        if (port_id == PORT_MAIN)
        {
            ctrl_owner = PORT_MAIN;                 /* 主口随时可抢 */
            ctrl_owner_ms = g_tick_ms;
        }
        else if ((ctrl_owner != PORT_MAIN) ||
                 ((g_tick_ms - ctrl_owner_ms) > OWNER_TIMEOUT_MS))
        {
            ctrl_owner = PORT_AUX;                  /* 从口只能捡主口不要的 */
            ctrl_owner_ms = g_tick_ms;
        }
    }
    else if (ctrl_owner == port_id)
    {
        ctrl_owner_ms = g_tick_ms;                  /* 控制方发零速, 续期 */
    }

    /* ---- 不是控制方就丢弃 ---- */
    if (ctrl_owner != port_id)
    {
        return;
    }

    target_vx = ClampF((float)vx_mm / 1000.0f, -MAX_LIN_SPEED, MAX_LIN_SPEED);
    target_wz = ClampF((float)wz_mrad / 1000.0f, -MAX_YAW_RATE, MAX_YAW_RATE);

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

    if ((m1 == 0) && (m2 == 0))
    {
        CAN1_SendMotor(MOTOR1_CAN_ID, STOP_CTRL, 0);
        CAN1_SendMotor(MOTOR2_CAN_ID, STOP_CTRL, 0);
    }
    else
    {
        CAN1_SendMotor(MOTOR1_CAN_ID, CTRL_ENABLE, m1);
        CAN1_SendMotor(MOTOR2_CAN_ID, CTRL_ENABLE, m2);
    }
}

/*==================================================================
 *  扩展卡尔曼滤波
 *
 *  状态 x = [ X, Y, theta, v, wz, bgz ]
 *      X,Y    位置 m
 *      theta  航向 rad, 逆时针为正, 0 指向 +X
 *      v      车体线速度 m/s
 *      wz     车体角速度 rad/s
 *      bgz    陀螺仪 Z 轴零偏 rad/s   (现在没用上, IMU 来了就生效)
 *
 *  预测:  用当前 v, wz 推位姿
 *  观测1: 轮速计算出的 (v, wz)          —— 一直有
 *  观测2: 陀螺仪 Z 轴角速度 gz = wz + bgz —— 接了 IMU 才有
 *==================================================================*/
#define EKF_N 6

static float ekf_x[EKF_N];
static float ekf_P[EKF_N][EKF_N];
static uint8_t ekf_ready;

static float WrapPi(float a)
{
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

static void Ekf_Symmetrize(void)
{
    uint8_t i, j;
    for (i = 0; i < EKF_N; i++)
    {
        for (j = (uint8_t)(i + 1); j < EKF_N; j++)
        {
            float s = 0.5f * (ekf_P[i][j] + ekf_P[j][i]);
            ekf_P[i][j] = s;
            ekf_P[j][i] = s;
        }
        if (ekf_P[i][i] < 1e-9f) ekf_P[i][i] = 1e-9f;
    }
}

static void Ekf_Init(void)
{
    uint8_t i, j;

    for (i = 0; i < EKF_N; i++)
    {
        ekf_x[i] = 0.0f;
        for (j = 0; j < EKF_N; j++)
        {
            ekf_P[i][j] = (i == j) ? 0.01f : 0.0f;
        }
    }
    ekf_ready = 1;
}

static void Ekf_Predict(float dt)
{
    float F[EKF_N][EKF_N];
    float FP[EKF_N][EKF_N];
    float FPFt[EKF_N][EKF_N];
    float v = ekf_x[3];
    float w = ekf_x[4];
    float theta_mid = ekf_x[2] + w * dt * 0.5f;
    float c = cosf(theta_mid);
    float s = sinf(theta_mid);
    float ds = fabsf(v) * dt;
    float dth = fabsf(w) * dt;
    float q_pos, q_th, q_v, q_w, q_b;
    float sum;
    uint8_t i, j, k;

    /* --- 状态传播 --- */
    ekf_x[0] += v * c * dt;
    ekf_x[1] += v * s * dt;
    ekf_x[2] = WrapPi(ekf_x[2] + w * dt);

    /* --- 状态转移雅可比 F = I + 偏置 --- */
    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    F[0][2] = -v * s * dt;
    F[0][3] =  c * dt;
    F[0][4] = -v * s * dt * dt * 0.5f;
    F[1][2] =  v * c * dt;
    F[1][3] =  s * dt;
    F[1][4] =  v * c * dt * dt * 0.5f;
    F[2][4] =  dt;

    /* --- P = F P F' + Q --- */
    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            sum = 0.0f;
            for (k = 0; k < EKF_N; k++) sum += F[i][k] * ekf_P[k][j];
            FP[i][j] = sum;
        }
    }
    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            sum = 0.0f;
            for (k = 0; k < EKF_N; k++) sum += FP[i][k] * F[j][k];   /* F[j][k] = F'[k][j] */
            FPFt[i][j] = sum;
        }
    }

    /* 过程噪声: 走得越远/转得越多, 位置和航向越不确定 */
    q_pos = EKF_SLIP_DIST * ds + 0.0005f;  q_pos *= q_pos;
    q_th  = EKF_SLIP_ANG  * dth + 0.001f;  q_th  *= q_th;
    q_v   = EKF_ACCEL_NOISE * dt;          q_v   *= q_v;
    q_w   = EKF_ANGACC_NOISE * dt;         q_w   *= q_w;
    q_b   = EKF_BIAS_WALK * dt;            q_b   *= q_b;

    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            ekf_P[i][j] = FPFt[i][j];
        }
    }
    ekf_P[0][0] += q_pos;
    ekf_P[1][1] += q_pos;
    ekf_P[2][2] += q_th;
    ekf_P[3][3] += q_v;
    ekf_P[4][4] += q_w;
    ekf_P[5][5] += q_b;

    Ekf_Symmetrize();
}

/* 观测: 轮速计解算出的车体 (v, wz) */
static void Ekf_UpdateWheel(float v_meas, float w_meas)
{
    float S00, S01, S10, S11, det, inv00, inv01, inv10, inv11;
    float K[EKF_N][2];
    float y0 = v_meas - ekf_x[3];
    float y1 = w_meas - ekf_x[4];
    float row3[EKF_N];
    float row4[EKF_N];
    uint8_t i, j;

    S00 = ekf_P[3][3] + EKF_SIGMA_V * EKF_SIGMA_V;
    S01 = ekf_P[3][4];
    S10 = ekf_P[4][3];
    S11 = ekf_P[4][4] + EKF_SIGMA_W * EKF_SIGMA_W;

    det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-12f) return;

    inv00 =  S11 / det;
    inv01 = -S01 / det;
    inv10 = -S10 / det;
    inv11 =  S00 / det;

    /* K = P H' S^-1 ;  H 选中第 3、4 个状态, 所以 P H' 的两列就是 P 的第 3、4 列 */
    for (i = 0; i < EKF_N; i++)
    {
        float a = ekf_P[i][3];
        float b = ekf_P[i][4];
        K[i][0] = a * inv00 + b * inv10;
        K[i][1] = a * inv01 + b * inv11;
    }

    for (i = 0; i < EKF_N; i++)
    {
        ekf_x[i] += K[i][0] * y0 + K[i][1] * y1;
    }
    ekf_x[2] = WrapPi(ekf_x[2]);

    /* P = (I - K H) P ;  先把要用的两行拷出来, 否则原地更新会串味 */
    for (j = 0; j < EKF_N; j++)
    {
        row3[j] = ekf_P[3][j];
        row4[j] = ekf_P[4][j];
    }
    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            ekf_P[i][j] -= K[i][0] * row3[j] + K[i][1] * row4[j];
        }
    }

    Ekf_Symmetrize();
}

/* 观测: 陀螺仪 Z 轴角速度, 同时估计零偏 */
static void Ekf_UpdateGyro(float gz)
{
    float S;
    float K[EKF_N];
    float y = gz - (ekf_x[4] + ekf_x[5]);
    float row4[EKF_N];
    float row5[EKF_N];
    uint8_t i, j;

    S = ekf_P[4][4] + ekf_P[4][5] + ekf_P[5][4] + ekf_P[5][5] + EKF_SIGMA_GZ * EKF_SIGMA_GZ;
    if (S < 1e-12f) return;

    for (i = 0; i < EKF_N; i++)
    {
        K[i] = (ekf_P[i][4] + ekf_P[i][5]) / S;
    }
    for (i = 0; i < EKF_N; i++)
    {
        ekf_x[i] += K[i] * y;
    }
    ekf_x[2] = WrapPi(ekf_x[2]);

    for (j = 0; j < EKF_N; j++)
    {
        row4[j] = ekf_P[4][j];
        row5[j] = ekf_P[5][j];
    }
    for (i = 0; i < EKF_N; i++)
    {
        for (j = 0; j < EKF_N; j++)
        {
            ekf_P[i][j] -= K[i] * (row4[j] + row5[j]);
        }
    }

    Ekf_Symmetrize();
}

/*======================= IMU 陀螺仪接口 (给本地 EKF 用) =============
 * 本地卡尔曼只要陀螺仪 Z 轴角速度(rad/s, 逆时针为正)。
 * 这里返回缓存值, 真正去读 I2C 的是 IMU_Tick()。
 *
 * ★ 车体逆时针为正, 但 IMU 装反了/坐标系不同的话符号可能相反 ——
 *   症状是轮速和陀螺仪互相打架, 航向估计反而更差。这时候改
 *   IMU_GYRO_Z_SIGN 为 -1。
 *
 * 返回 0 表示这次没有可信的陀螺仪数据, EKF 会自动跳过这一步。
 *=================================================================*/
static uint8_t IMU_ReadGyroZ(float *gz)
{
    if (!imu_gyro_ok)
    {
        return 0;
    }
    *gz = imu_gyro[2] * (float)IMU_GYRO_Z_SIGN;
    return 1;
}

/*======================= IMU (YbImu) ==============================
 * 按 ODOM_PERIOD_MS 采一次。读失败不清零 —— 保留上一次的值, 同时把 imu_ok
 * 置 0, 上位机看 imu_ok 就知道这次的数据可不可信。
 *
 * 读失败时最多每秒做一次诊断(查总线空闲电平 + 扫地址), 结果通过遥测帧
 * 报给上位机 —— 否则"读取失败"这四个字什么信息都没有, 没法修。
 *=================================================================*/
#define IMU_DIAG_PERIOD_MS  1000

static void IMU_Tick(void)
{
    float accel[3];
    float gyro[3];
    uint8_t err;

    err = YbImu_ReadMotion(accel, gyro);

    /* YBIMU_ST_GYRO_ZERO 表示加速度读到了但陀螺仪恒为 0 —— 加速度还是能用的,
       所以不算完全失败, 但要如实报上去。 */
    if ((err == YBIMU_ST_OK) || (err == YBIMU_ST_GYRO_ZERO))
    {
        imu_accel_g[0] = accel[0];
        imu_accel_g[1] = accel[1];
        imu_accel_g[2] = accel[2];
        imu_ok = 1;
        imu_status = err;
        imu_found_addr = YBIMU_I2C_ADDR;

        if (err == YBIMU_ST_OK)
        {
            imu_gyro[0] = gyro[0];
            imu_gyro[1] = gyro[1];
            imu_gyro[2] = gyro[2];
            imu_gyro_ok = 1;
        }
        else
        {
            imu_gyro[0] = 0.0f;
            imu_gyro[1] = 0.0f;
            imu_gyro[2] = 0.0f;
            imu_gyro_ok = 0;
        }
        return;
    }

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

/*======================= 轮速采样 + (可选)滤波 ===================*/
static void Odom_Tick(void)
{
    Wheel_Update();
    IMU_Tick();     /* 位翻转 I2C 读 IMU, 大约 1~3ms */

#if EKF_ON_STM32
    {
        const float dt = (float)ODOM_PERIOD_MS / 1000.0f;
        float gz;

        Ekf_Predict(dt);
        Ekf_UpdateWheel(body_vx, body_wz);
        if (IMU_ReadGyroZ(&gz))
        {
            Ekf_UpdateGyro(gz);
        }
    }
#endif
}

/*=========================== 状态回传 ==============================
 * 主遥测帧严格按协议 24 字节; 位姿放在扩展的里程计帧里(头 0x7E)。
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

static void Send_OdomFrame(uart_port_t *p)
{
    uint8_t f[ODOM_FRAME_SIZE];
    uint8_t i;
    uint8_t bcc = 0;
    uint8_t flags = 0;
    int32_t x_mm = (int32_t)(ekf_x[0] * 1000.0f);
    int32_t y_mm = (int32_t)(ekf_x[1] * 1000.0f);
    int16_t head_cdeg = (int16_t)(ekf_x[2] * 57.29578f * 100.0f);   /* rad -> 0.01度 */

    if (ekf_ready)          flags |= 0x01;
    if (failsafe_latched)   flags |= 0x02;
    if (ever_linked)        flags |= 0x04;
    if (imu_ok)             flags |= 0x08;      /* IMU 最近一次读取成功 */

    f[0] = ODOM_HEAD;
    f[1] = flags;
    PutI32(&f[2], x_mm);
    PutI32(&f[6], y_mm);
    PutI16(&f[10], head_cdeg);
    PutI16(&f[12], (int16_t)(ekf_x[3] * 1000.0f));    /* 滤波后 vx  mm/s */
    PutI16(&f[14], (int16_t)(ekf_x[4] * 1000.0f));    /* 滤波后 wz  mrad/s */
    PutI16(&f[16], wheel_left_rpm);
    PutI16(&f[18], wheel_right_rpm);
    PutI16(&f[20], target_left_rpm);
    PutI16(&f[22], target_right_rpm);
    f[24] = m1_fault;
    f[25] = m2_fault;
    f[26] = can_error_count;
    f[27] = (uint8_t)(ports[PORT_MAIN].bad_count + ports[PORT_AUX].bad_count);
    f[28] = (uint8_t)(ports[PORT_MAIN].ok_count + ports[PORT_AUX].ok_count);
    f[29] = (uint8_t)(ports[PORT_MAIN].overflow + ports[PORT_AUX].overflow);
    f[30] = relay_state;                       /* bit0 电机 bit1 水泵 */
    f[31] = (ctrl_owner == OWNER_NONE) ? 0 : (uint8_t)(ctrl_owner + 1);
    f[32] = imu_status;                        /* YBIMU_ST_* 诊断结果 */
    f[33] = imu_found_addr;                    /* 扫描到的 I2C 器件地址, 0 = 没扫到 */
    for (i = 0; i < ODOM_FRAME_SIZE - 2; i++) bcc ^= f[i];
    f[ODOM_FRAME_SIZE - 2] = bcc;
    f[ODOM_FRAME_SIZE - 1] = ODOM_TAIL;

    for (i = 0; i < ODOM_FRAME_SIZE; i++) Port_SendByte(p, f[i]);
}

/*============================== main ===============================*/
int main(void)
{
    Tick_Init();
    CAN1_Init();
    USART1_Init();          /* PA9/PA10 -> CH340 / RK3588 */
    USART2_Init();          /* PA2/PA3  -> ESP32 */
    Relay_Init();
#if EKF_ON_STM32
    Ekf_Init();
#endif
    ctrl_owner = OWNER_NONE;
    Delay_ms(200);

    target_vx = 0.0f;
    target_wz = 0.0f;
    CAN1_SendMotor(MOTOR1_CAN_ID, STOP_CTRL, 0);
    CAN1_SendMotor(MOTOR2_CAN_ID, STOP_CTRL, 0);

    last_can_ms = g_tick_ms;
    last_odom_ms = g_tick_ms;
    last_tel_ms = g_tick_ms;
    last_cmd_ms = g_tick_ms;

    while (1)
    {
        Port_ProcessCommands(&ports[PORT_MAIN]);
        Port_ProcessCommands(&ports[PORT_AUX]);
        CAN1_Poll();

        /* 控制权超时 -> 释放, 让另一个口能接管 */
        if ((ctrl_owner != OWNER_NONE) &&
            ((g_tick_ms - ctrl_owner_ms) > OWNER_TIMEOUT_MS))
        {
            ctrl_owner = OWNER_NONE;
        }

        /* 看门狗: 当前控制方停发就自动停车。
           注意只认控制方的帧 —— 否则从口的心跳会把看门狗一直喂着,
           主口掉线了车也不会停。 */
        if ((failsafe_latched == 0) &&
            (ever_linked) &&
            ((g_tick_ms - last_cmd_ms) > LINK_TIMEOUT_MS))
        {
            target_vx = 0.0f;
            target_wz = 0.0f;
            failsafe_latched = 1;
            ctrl_owner = OWNER_NONE;
#if RELAY_OFF_ON_LINK_LOSS
            Relay_Set(0);       /* 断线了顺便把电机/水泵继电器也断开 */
#endif
        }

        /* 轮速采样 + (可选)卡尔曼 */
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

        /* 上报: 标准 24 字节帧两个口都发 */
        if ((g_tick_ms - last_tel_ms) >= TELEMETRY_PERIOD_MS)
        {
            last_tel_ms = g_tick_ms;

            Send_MainTelemetry(&ports[PORT_MAIN]);
            Send_MainTelemetry(&ports[PORT_AUX]);

#if EKF_ON_STM32
            /* 位姿帧是本工程扩展, 厂商驱动不认识, 默认只发给调试口 */
            Send_OdomFrame(&ports[PORT_AUX]);
#if ODOM_FRAME_TO_MAIN
            Send_OdomFrame(&ports[PORT_MAIN]);
#endif
#endif
        }

        CAN1_Poll();
        Delay_ms(1);
    }
}
