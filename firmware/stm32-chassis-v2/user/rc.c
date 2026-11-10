#include "rc.h"
#include "board.h"

/**
  ******************************************************************************
  * @file    rc.c
  * @brief   遥控输入: USART3 中断收字节 -> 环形缓冲 -> 主循环解析 15 字节原始帧
  *
  * 为什么不用"帧头+长度+校验"的包装: CH554G 只做原样转发, 接收器 500Hz 的
  * 15 字节报告本身就带 3 个固定 0x00 和 4 位十字键范围可以做同步, 加包装反而
  * 要两边都改。丢字节时靠 Rc_Resync() 滑一个字节重新找帧头, 500Hz 下 30ms 内
  * 必恢复。
  *
  * 中断里只做一件事: 把 DR 读进环形缓冲 (7500 次/秒, 每次十几个周期)。
  * 解析全部在主循环 (10ms 一拍, 每拍 75 字节左右), 不占中断时间。
  ******************************************************************************
  */

/* ------------------------------ 环形缓冲 ------------------------------ */
#define RC_RING_SIZE 256u
#define RC_RING_MASK (RC_RING_SIZE - 1u)

static volatile uint8_t s_ring[RC_RING_SIZE];
static volatile uint16_t s_ring_head; /* 中断写 */
static volatile uint16_t s_ring_tail; /* 主循环读 */

static volatile uint16_t s_drop_count;
static volatile uint16_t s_overrun_count;

static void Rc_PushByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_ring_head + 1u) & RC_RING_MASK);

    if (next == s_ring_tail)
    {
        /* 缓冲满: 丢掉最老的半圈, 保证新数据能进来 */
        s_ring_tail = (uint16_t)((s_ring_tail + 1u) & RC_RING_MASK);
        s_drop_count++;
    }
    s_ring[s_ring_head] = byte;
    s_ring_head = next;
}

static uint8_t Rc_PopByte(uint8_t *out)
{
    if (s_ring_tail == s_ring_head)
    {
        return 0;
    }
    *out = s_ring[s_ring_tail];
    s_ring_tail = (uint16_t)((s_ring_tail + 1u) & RC_RING_MASK);
    return 1;
}

/* USART3 接收中断 (startup 里的弱定义被这里覆盖) */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t value = (uint8_t)USART_ReceiveData(USART3);
        Rc_PushByte(value);
    }
    else if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        /* 溢出: 读一次 DR 清标志, 丢掉的字节由帧同步自恢复 */
        (void)USART_ReceiveData(USART3);
        s_overrun_count++;
    }
}

/* ------------------------------ 解析状态 ------------------------------ */
#define RC_CYCLES_PER_MS 72000u /* 72MHz DWT */

static uint8_t s_frame[RC_FRAME_SIZE];
static uint8_t s_frame_len;

static uint8_t s_raw[RC_FRAME_SIZE];       /* 最近一帧原始数据 */
static uint8_t s_sent_raw[RC_FRAME_SIZE];  /* 上一帧发出去的原始数据 */
static Rc_State s_state;

static uint32_t s_last_cycles;
static uint8_t s_have_frame;

static uint16_t s_frames;
static uint16_t s_bad;
static uint16_t s_inject;
static uint16_t s_uplink_frames;

static int16_t s_vx_mm_s;
static int16_t s_wz_mrad_s;

/* ------------------------------ 小工具 ------------------------------ */

static uint8_t Rc_FrameValid(const uint8_t *frame)
{
    /* 三个固定 0x00 + 十字键范围 = 强约束, 误同步概率极低 */
    if (frame[0] != 0x00u)
    {
        return 0;
    }
    if ((frame[13] != 0x00u) || (frame[14] != 0x00u))
    {
        return 0;
    }
    if ((frame[12] & 0xE0u) != 0u)
    {
        return 0;
    }
    if (((frame[12] >> 2) & 0x0Fu) > 8u)
    {
        return 0;
    }
    return 1;
}

static uint16_t Rc_U16Le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* 摇杆原始值 -> 千分制 (-1000..1000), 扣死区并线性拉回满量程 */
static int16_t Rc_StickPermille(uint16_t raw, uint16_t center)
{
    int32_t diff = (int32_t)raw - (int32_t)center;
    int32_t permille = (diff * 1000) / 32768;
    int32_t magnitude;

    if ((permille > -RC_DEADZONE_PERMILLE) && (permille < RC_DEADZONE_PERMILLE))
    {
        return 0;
    }

    if (permille < 0)
    {
        magnitude = -permille - RC_DEADZONE_PERMILLE;
        magnitude = (magnitude * 1000) / (1000 - RC_DEADZONE_PERMILLE);
        if (magnitude > 1000)
        {
            magnitude = 1000;
        }
        return (int16_t)(-magnitude);
    }

    magnitude = permille - RC_DEADZONE_PERMILLE;
    magnitude = (magnitude * 1000) / (1000 - RC_DEADZONE_PERMILLE);
    if (magnitude > 1000)
    {
        magnitude = 1000;
    }
    return (int16_t)magnitude;
}

/* 采纳一帧: 解码 + 更新统计/在线状态 + 算速度指令 */
static void Rc_Accept(const uint8_t *frame)
{
    uint8_t i;
    int16_t stick_x;
    int16_t stick_y;

    for (i = 0; i < RC_FRAME_SIZE; i++)
    {
        s_raw[i] = frame[i];
    }

    s_state.x = Rc_U16Le(&frame[1]);
    s_state.y = Rc_U16Le(&frame[3]);
    s_state.rx = Rc_U16Le(&frame[5]);
    s_state.ry = Rc_U16Le(&frame[7]);
    s_state.z = Rc_U16Le(&frame[9]);
    s_state.buttons = frame[11];
    s_state.buttons2 = (uint8_t)(frame[12] & 0x03u);
    s_state.hat = (uint8_t)((frame[12] >> 2) & 0x0Fu);
    s_state.valid = 1u;

    /* 左摇杆: Y 向上(变大) = 前进; X 向右(变大) = 右转(wz 负) */
    stick_x = Rc_StickPermille(s_state.x, 32768u);
    stick_y = Rc_StickPermille(s_state.y, 32767u);
    s_vx_mm_s = (int16_t)((stick_y * RC_VX_MAX_MM_S) / 1000);
    s_wz_mrad_s = (int16_t)(-(stick_x * RC_WZ_MAX_MRAD_S) / 1000);

    s_last_cycles = BOARD_DWT_CYCCNT;
    s_have_frame = 1u;
    s_frames++;
}

/* 校验失败后重新找帧头: 缓冲区里下一个 0x00 之后当作新帧开头 */
static void Rc_Resync(void)
{
    uint8_t i;

    for (i = 1u; i < RC_FRAME_SIZE; i++)
    {
        if (s_frame[i] == 0x00u)
        {
            uint8_t j;
            uint8_t remain = (uint8_t)(RC_FRAME_SIZE - i);

            for (j = 0; j < remain; j++)
            {
                s_frame[j] = s_frame[i + j];
            }
            s_frame_len = remain;
            return;
        }
    }
    s_frame_len = 0u;
}

static void Rc_FeedByte(uint8_t byte)
{
    if (s_frame_len == 0u)
    {
        if (byte != 0x00u)
        {
            s_bad++;
            return;
        }
        s_frame[0] = byte;
        s_frame_len = 1u;
        return;
    }

    s_frame[s_frame_len] = byte;
    s_frame_len++;

    if (s_frame_len < RC_FRAME_SIZE)
    {
        return;
    }

    if (Rc_FrameValid(s_frame) != 0u)
    {
        Rc_Accept(s_frame);
        s_frame_len = 0u;
    }
    else
    {
        s_bad++;
        Rc_Resync();
    }
}

/* ------------------------------ 接口 ------------------------------ */

void Rc_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    s_ring_head = 0;
    s_ring_tail = 0;
    s_frame_len = 0;
    s_have_frame = 0;
    s_last_cycles = 0;
    s_frames = 0;
    s_bad = 0;
    s_inject = 0;
    s_uplink_frames = 0;
    s_drop_count = 0;
    s_overrun_count = 0;
    s_vx_mm_s = 0;
    s_wz_mrad_s = 0;
    s_state.valid = 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PB10 = USART3_TX (复用推挽, 本固件目前不发, 但要配成复用免得悬空) */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    /* PB11 = USART3_RX (上拉输入, 断线时读到高电平不会误判成帧) */
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART3, ENABLE);
}

void Rc_Poll(void)
{
    uint8_t byte;

    /* 每拍把缓冲排空 (500Hz * 15B * 10ms = 75 字节) */
    while (Rc_PopByte(&byte) != 0u)
    {
        Rc_FeedByte(byte);
    }

    /* 300ms 没有有效帧就认为手柄掉线 */
    if (s_have_frame != 0u)
    {
        uint32_t age_ms = (BOARD_DWT_CYCCNT - s_last_cycles) / RC_CYCLES_PER_MS;
        if (age_ms > 300u)
        {
            s_have_frame = 0u;
            s_state.valid = 0u;
            s_vx_mm_s = 0;
            s_wz_mrad_s = 0;
        }
    }
}

const Rc_State *Rc_Get(void)
{
    return &s_state;
}

uint8_t Rc_Online(void)
{
    return s_have_frame;
}

void Rc_GetCmd(int16_t *vx_mm_s, int16_t *wz_mrad_s)
{
    if (vx_mm_s != 0)
    {
        *vx_mm_s = s_vx_mm_s;
    }
    if (wz_mrad_s != 0)
    {
        *wz_mrad_s = s_wz_mrad_s;
    }
}

void Rc_GetStickPermille(int16_t *x_permille, int16_t *y_permille)
{
    if (x_permille != 0)
    {
        *x_permille = Rc_StickPermille(s_state.x, 32768u);
    }
    if (y_permille != 0)
    {
        *y_permille = Rc_StickPermille(s_state.y, 32767u);
    }
}

uint16_t Rc_FrameCount(void)
{
    return s_frames;
}

uint16_t Rc_BadCount(void)
{
    return s_bad;
}

uint16_t Rc_InjectCount(void)
{
    return s_inject;
}

uint16_t Rc_DropCount(void)
{
    return (uint16_t)(s_drop_count + s_overrun_count);
}

uint8_t Rc_InjectFrame(const uint8_t *frame)
{
    if (Rc_FrameValid(frame) == 0u)
    {
        s_bad++;
        return 0;
    }
    s_inject++;
    Rc_Accept(frame);
    return 1;
}

uint8_t Rc_SendFrame(void)
{
    uint8_t frame[19];
    uint8_t bcc = 0;
    uint8_t i;
    uint8_t changed = 0;

    frame[0] = 0x79u;
    frame[1] = (uint8_t)((s_have_frame != 0u) ? 0x01u : 0x00u);

    for (i = 0; i < RC_FRAME_SIZE; i++)
    {
        if (s_raw[i] != s_sent_raw[i])
        {
            changed = 1u;
        }
    }
    if (changed != 0u)
    {
        frame[1] = (uint8_t)(frame[1] | 0x02u);
        for (i = 0; i < RC_FRAME_SIZE; i++)
        {
            s_sent_raw[i] = s_raw[i];
        }
    }

    for (i = 0; i < RC_FRAME_SIZE; i++)
    {
        frame[2u + i] = s_raw[i];
    }

    for (i = 0; i < 17u; i++)
    {
        bcc = (uint8_t)(bcc ^ frame[i]);
    }
    frame[17] = bcc;
    frame[18] = 0x7Du;

    Board_UartSendBytes(frame, (uint16_t)sizeof(frame));
    s_uplink_frames++;
    return bcc;
}
