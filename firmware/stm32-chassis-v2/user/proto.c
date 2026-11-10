#include "proto.h"
#include "board.h"
#include "mode.h"

/**
  ******************************************************************************
  * @file    proto.c
  * @brief   上位机串口协议的字节流解析 (帧格式见 proto.h)
  *
  * 做法: 一个字节一个字节收进缓冲, 每收一个就判断"按当前帧头还需要几个字节"。
  *   0x7B -> 还需要 11 字节   0x7A -> 还需要 8 字节   其它 -> 立刻丢掉这个字节
  * 收满后先验帧尾 0x7D 和 BCC, 都对才交给 Proto_Dispatch(); 不对就丢掉最老的
  * 一个字节重新找帧头(这样即使中间丢了字节也能自己恢复同步)。
  ******************************************************************************
  */

static uint8_t s_buf[PROTO_CMD_SIZE];
static uint8_t s_len;

static int16_t s_host_vx_mm_s;
static int16_t s_host_wz_mrad_s;

static uint16_t s_host_cmds;
static uint16_t s_host_rejected;
static uint16_t s_bad_frames;
static uint16_t s_func_frames;

/* 每次进入非 HOST 模式只提醒一次"命令被丢弃", 免得 30Hz 刷屏 */
static uint8_t s_reject_warned;

/* ------------------------------ 小工具 ------------------------------ */

static uint8_t Proto_Xor(const uint8_t *data, uint8_t count)
{
    uint8_t i;
    uint8_t value = 0;

    for (i = 0; i < count; i++)
    {
        value = (uint8_t)(value ^ data[i]);
    }
    return value;
}

static int16_t Proto_I16Be(const uint8_t *data)
{
    return (int16_t)(((int16_t)data[0] << 8) | (int16_t)data[1]);
}

static void Proto_Log(const char *text)
{
    Board_UartSend("[proto] ");
    Board_UartSend(text);
    Board_UartSend("\r\n");
}

/* ------------------------------ 分发 ------------------------------ */

static void Proto_Dispatch(const uint8_t *frame)
{
    /* ---------- 速度帧 ---------- */
    if (frame[0] == PROTO_CMD_HEAD)
    {
        if (Mode_Get() != MODE_HOST)
        {
            /* 遥控/推行模式下不接受上位机速度命令 */
            s_host_rejected++;
            if (s_reject_warned == 0)
            {
                s_reject_warned = 1;
                Board_UartSend("[proto] speed cmd ignored, mode = ");
                Board_UartSend(Mode_Name(Mode_Get()));
                Board_UartSend("\r\n");
            }
            return;
        }

        s_reject_warned = 0;
        s_host_vx_mm_s = Proto_I16Be(&frame[3]);
        s_host_wz_mrad_s = Proto_I16Be(&frame[7]);
        s_host_cmds++;
        return;
    }

    /* ---------- 功能帧 ---------- */
    s_func_frames++;

    switch (frame[1])
    {
        case PROTO_FUNC_ID_SET_MODE:
            if (frame[2] < MODE_COUNT)
            {
                Mode_Set(frame[2], MODE_SRC_UART);
            }
            else
            {
                Proto_Log("set_mode: bad mode id");
            }
            break;

        case PROTO_FUNC_ID_AUX_BTN:
            /* V2 板没有刷盘/风机这级硬件, 收到只计数, 不做事 */
            break;

        default:
            break;
    }
}

/* ------------------------------ 解析 ------------------------------ */

/* 当前缓冲开头那个字节, 按帧头判断整帧要多长。
   返回 0 = 缓冲空 / 1 = 不是帧头(丢一个字节) / 11 或 8 = 整帧长度 */
static uint8_t Proto_FrameNeed(void)
{
    if (s_len == 0u)
    {
        return 0u;
    }

    if (s_buf[0] == PROTO_CMD_HEAD)
    {
        return (uint8_t)PROTO_CMD_SIZE;
    }
    if (s_buf[0] == PROTO_FUNC_HEAD)
    {
        return (uint8_t)PROTO_FUNC_SIZE;
    }
    return 1u;
}

static void Proto_DropFirst(void)
{
    uint8_t i;

    if (s_len == 0u)
    {
        return;
    }
    for (i = 1u; i < s_len; i++)
    {
        s_buf[i - 1u] = s_buf[i];
    }
    s_len--;
}

/* 整帧校验: 帧尾 0x7D + BCC */
static uint8_t Proto_FrameOk(uint8_t need)
{
    if (s_buf[need - 1u] != PROTO_TAIL)
    {
        return 0;
    }
    /* 0x7B 帧: [9]=XOR([0..8])；0x7A 帧: [6]=XOR([0..5]) */
    return (Proto_Xor(s_buf, (uint8_t)(need - 2u)) == s_buf[need - 2u]) ? 1u : 0u;
}

void Proto_Init(void)
{
    s_len = 0;
    s_host_vx_mm_s = 0;
    s_host_wz_mrad_s = 0;
    s_host_cmds = 0;
    s_host_rejected = 0;
    s_bad_frames = 0;
    s_func_frames = 0;
    s_reject_warned = 0;
}

void Proto_FeedByte(uint8_t byte)
{
    uint8_t need;

    /* 缓冲满了先丢掉最老的, 保证永远有新字节的位置 */
    if (s_len >= (uint8_t)PROTO_CMD_SIZE)
    {
        Proto_DropFirst();
    }
    s_buf[s_len] = byte;
    s_len++;

    for (;;)
    {
        need = Proto_FrameNeed();

        if (need == 0u)
        {
            return;
        }
        if (need == 1u)
        {
            /* 不是帧头: 丢掉, 继续找 */
            s_bad_frames++;
            Proto_DropFirst();
            if (s_len == 0u)
            {
                return;
            }
            continue;
        }
        if (s_len < need)
        {
            return; /* 还没收够 */
        }

        if (Proto_FrameOk(need) != 0u)
        {
            Proto_Dispatch(s_buf);
            s_len = 0;
            return;
        }

        /* 校验不过: 丢掉最老一个字节重新同步 */
        s_bad_frames++;
        Proto_DropFirst();
    }
}

/* ------------------------------ 统计 ------------------------------ */

uint16_t Proto_HostCmdCount(void)
{
    return s_host_cmds;
}

uint16_t Proto_HostRejectCount(void)
{
    return s_host_rejected;
}

uint16_t Proto_BadFrameCount(void)
{
    return s_bad_frames;
}

uint16_t Proto_FuncFrameCount(void)
{
    return s_func_frames;
}

void Proto_GetHostCmd(int16_t *vx_mm_s, int16_t *wz_mrad_s)
{
    if (vx_mm_s != 0)
    {
        *vx_mm_s = s_host_vx_mm_s;
    }
    if (wz_mrad_s != 0)
    {
        *wz_mrad_s = s_host_wz_mrad_s;
    }
}
