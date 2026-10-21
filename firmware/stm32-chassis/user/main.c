#include "stm32f10x.h"
#include "Delay.h"

/*==========================================================================
 * 两驱差速底盘 —— 达妙 DM-H65 轮毂电机 (DM6540-1EC) 最小"忠实桥"固件
 *
 *   本文件只做两件事:
 *     1. 忠实执行上位机命令 —— 收 11 字节速度帧, 差速逆解后原样下发给
 *        两台达妙电机 (CAN 标准帧, 1 Mbps, 速度模式)。
 *     2. 忠实上报电机状态 —— 20 Hz 发 44 字节二进制状态帧, 内容全部来自
 *        DM 的反馈帧, 不做滤波、不做外环、不做任何"智能"修正。
 *
 *   旧固件里的 Boost 助推、Trim 轮速外环、力矩模式、偏航捕获、上电自检、
 *   IMU、里程计、继电器、死锁保护、FOC 协议、扩展帧、500 kbps 全部删除。
 *   原因很简单: 底盘电机换成了带自己速度环的达妙伺服, 上位机也换成了
 *   RK3588。STM32 再叠一层控制律, 只会和电机内部的速度环打架; 而"桥"
 *   越简单, 出问题时越容易定位是"命令没到"还是"电机没执行"。
 *
 *   硬件连接:
 *     USART1  PA9(TX) / PA10(RX) <-> RK3588        115200 8N1
 *     CAN1    PA12(TX) / PA11(RX) -> 左 DM (ID 1) / 右 DM (ID 2), 1 Mbps
 *     PB9     LED 心跳, 500 ms 翻转
 *     IWDG    主循环卡死 -> 复位; 复位后重新写模式并使能
 *
 *   本文件不实现: IMU、里程计、继电器、OLED、参数读取、浮点 printf。
 *   状态帧是定长二进制, 上位机自己按小端/大端解析, 不需要 printf。
 *========================================================================*/

/*============================== 配置 ====================================
 * 所有长度单位统一成 m、rad、s。协议里的 mm/s、mrad/s 只在收发的边界上
 * 做一次换算, 中间控制量不混用单位 —— 混单位的 bug 最难查。
 *=======================================================================*/
#define MOTOR1_CAN_ID       1           /* 左轮 DM 节点号 (控制帧 0x201) */
#define MOTOR1_MST_ID       0x11        /* 左轮反馈帧 ID */
#define MOTOR2_CAN_ID       2           /* 右轮 DM 节点号 (控制帧 0x202) */
#define MOTOR2_MST_ID       0x12        /* 右轮反馈帧 ID */

#define WHEEL_R_M           0.0855f     /* 轮半径 m (171 mm 轮径 / 2) */
#define TRACK_WIDTH_M       0.425f      /* 轮距 m, 按实车架修改! */
#define MAX_OMEGA_RAD_S     30.0f       /* 单轮角速度上限 rad/s (~2.57 m/s) */

/*----------------------------- 时间参数 ---------------------------------
 * 20 ms 控制周期 = 50 Hz。达妙速度模式内部跑自己的电流/速度环, 50 Hz
 * 下发足够; 再快只会增加总线负载, 不会让车响应更好。
 * 50 ms 上报 = 20 Hz。44 字节在 115200 下约 3.8 ms, 占用可控。
 *-----------------------------------------------------------------------*/
#define CMD_TIMEOUT_MS      800         /* 上位机命令超时 */
#define FB_TIMEOUT_MS       2000        /* 电机反馈超时 */
#define CTRL_PERIOD_MS      20          /* 控制/下发周期 */
#define STATUS_PERIOD_MS    50          /* 状态上报周期 */

#define LED_BLINK_MS        500         /* PB9 心跳翻转周期 */
#define STATUS_VERSION      1           /* 状态帧 [30] 版本号 */
#define STATUS_FRAME_SIZE   44
#define STATUS_HEAD         0x7E
#define STATUS_TAIL         0x7D

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

/*=========================== 全局状态 =================================*/
volatile uint32_t g_tick_ms;            /* TIM2 1 ms 时基, 中断里自增 */

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
static uint8_t  g_cmd_timeout;          /* 1 = 超过 800 ms 没收到命令 */
static float    g_target_omega[2];      /* 下发给电机的目标 rad/s */
static uint8_t  g_can_tx_fail;          /* CAN 发送失败计数, 饱和 8 位 */

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
static void CAN1_Init(void)
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
    CAN_Init(CAN1, &can);

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
#define USART_TX_TIMEOUT    20000

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

/* 非阻塞发字节, 但带超时:
 * 原旧固件这里是 `while (TXE == RESET) {}` 死等, USART 一旦状态异常就
 * 永久卡死; 配合软件看门狗一起死, 车会保持最后速度。现在最多等
 * 20000 次循环 (~几 ms), 发不出去就丢这一字节, 由上位机的帧校验兜底。
 */
static void Port_SendByte(uint8_t value)
{
    uint32_t timeout = USART_TX_TIMEOUT;

    while ((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) &&
           (--timeout != 0))
    {
    }
    if (timeout == 0)
    {
        return;
    }
    USART_SendData(USART1, value);
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

static void Cmd_Apply(const uint8_t *f)
{
    /* 能进到这里说明帧头、帧尾、BCC 全对, 足以证明上位机活着。 */
    g_last_cmd_ms = g_tick_ms;

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

    g_target_omega[0] = ClampF(v_left / WHEEL_R_M,
                               -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);
    g_target_omega[1] = ClampF(v_right / WHEEL_R_M,
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
 *   [2..3]  最近一次收到的 vx (mm/s) [4..5] 最近一次收到的 wz (mrad/s)
 *   [6..7]  左实际 omega x100        [8..9] 右实际 omega x100
 *   [10..11] 左目标 omega x100       [12..13] 右目标 omega x100
 *   [14..15] 左扭矩 x100 Nm          [16..17] 右扭矩 x100 Nm
 *   [18] 左 ERR  [19] 右 ERR
 *   [20] 左 MOS 温度  [21] 右 MOS 温度
 *   [22] 左线圈温度   [23] 右线圈温度
 *   [24..25] 左位置 x100 rad         [26..27] 右位置 x100 rad
 *   [28] CAN 发送失败计数 (饱和)     [29] 反馈帧总数 (饱和)
 *   [30] 版本号 = 1  [31] 保留 0
 *   [32..33] 左反馈年龄 ms           [34..35] 右反馈年龄 ms
 *   [36..37] tick 低 16 位           [38..41] 保留 0
 *   [42] BCC = XOR(f[0..41])         [43] 0x7D
 *
 * 为什么把"目标"也报上去: 上位机只有同时看到"目标"和"实际", 才能区分
 * "电机没执行"和"我正在被轮速上限夹"。目标已经是逆解+限幅后的最终值。
 *=====================================================================*/
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
    f[31] = 0;
    PutU16(&f[32], FbAge(0, now));
    PutU16(&f[34], FbAge(1, now));
    PutU16(&f[36], (uint16_t)g_tick_ms);
    f[38] = 0;
    f[39] = 0;
    f[40] = 0;
    f[41] = 0;

    for (i = 0; i < STATUS_FRAME_SIZE - 2; i++)
    {
        bcc ^= f[i];
    }
    f[STATUS_FRAME_SIZE - 2] = bcc;
    f[STATUS_FRAME_SIZE - 1] = STATUS_TAIL;

    /* 阻塞发送。115200 下 44 字节约 3.8 ms; 控制周期 20 ms, 留有余量。
       不引入 TX 中断/双缓冲, 是为了让这份"桥"尽量少的状态机。 */
    for (i = 0; i < STATUS_FRAME_SIZE; i++)
    {
        Port_SendByte(f[i]);
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
 *=====================================================================*/
static void Motors_Startup(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++)
    {
        DM_ParamWrite(MOTOR1_CAN_ID, DM_RID_CTRL_MODE, DM_MODE_VELOCITY);
        DM_ParamWrite(MOTOR2_CAN_ID, DM_RID_CTRL_MODE, DM_MODE_VELOCITY);
        Delay_ms(30);
        IWDG_ReloadCounter();
    }

    for (i = 0; i < 3; i++)
    {
        DM_SendEnable(MOTOR1_CAN_ID);
        DM_SendEnable(MOTOR2_CAN_ID);
        Delay_ms(30);
        IWDG_ReloadCounter();
    }
}

/*============================== main =================================*/
int main(void)
{
    uint32_t now;
    uint32_t last_ctrl_ms;
    uint32_t last_status_ms;

    Tick_Init();            /* 先有时基, 后面所有调度都靠它 */
    CAN1_Init();            /* 1 Mbps, 标准帧 */

    /* ★ CAN 一通先发 0 速度, 不等 USART/LED/看门狗初始化:
       看门狗复位时驱动器还在执行复位前最后一条速度命令, 每多等 1 ms
       车就多冲一点。DM 在速度模式下收到 0 会主动刹住; 若它还在 MIT
       模式, 这个 4 字节帧会被忽略, 不会造成动作。 */
    DM_SendSpeed(MOTOR1_CAN_ID, 0.0f);
    DM_SendSpeed(MOTOR2_CAN_ID, 0.0f);

    USART1_Init();          /* PA9/PA10, 115200, 中断收 */
    LED_Init();             /* PB9 心跳 */
    IWDG_Init();            /* 开狗; 后面初始化步骤都会喂 */

    /* 写模式 + 使能。整个启动约 180 ms, 远小于 1 s 看门狗。 */
    Motors_Startup();

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

        if ((now - last_ctrl_ms) >= CTRL_PERIOD_MS)
        {
            last_ctrl_ms = now;
            Control_Tick();         /* 逆解 + 限幅 + 两帧速度命令 */
        }

        if ((now - last_status_ms) >= STATUS_PERIOD_MS)
        {
            last_status_ms = now;
            Status_Send(now);       /* 44 字节状态帧 */
        }

        /* 1 ms 让出 CPU。所有周期任务都是基于 g_tick_ms 的非阻塞调度,
           这里的延时只影响最小轮询粒度, 不影响控制/上报周期。 */
        Delay_ms(1);
    }
}
