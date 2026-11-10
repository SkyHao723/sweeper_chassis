#include "stm32f10x.h"
#include "board.h"
#include "telemetry.h"

/**
  ******************************************************************************
  * @file    telemetry.c
  * @brief   上行遥测帧组帧与发送 (32 字节, 20Hz)
  ******************************************************************************
  */

static uint16_t s_sent;

/* 最近一次发出的帧留一份在 RAM 里。
   用途: 板子没插 USB(没有串口)时, 可以用调试器直接 dump 这段内存核对传感器数据,
   见 tools/v2_monitor.py --ram。volatile 保证编译器不会把它优化掉。 */
static volatile uint8_t s_last_frame[TELEMETRY_FRAME_SIZE];

static void PutU16(uint8_t *frame, uint8_t offset, uint16_t value)
{
    frame[offset] = (uint8_t)(value >> 8);
    frame[offset + 1u] = (uint8_t)(value & 0xFFu);
}

static uint8_t Xor(const uint8_t *data, uint8_t length)
{
    uint8_t value = 0;
    uint8_t index;

    for (index = 0; index < length; index++)
    {
        value ^= data[index];
    }
    return value;
}

void Telemetry_Build(const Telemetry_Data *data, uint8_t *frame)
{
    uint8_t flags;
    uint8_t index;

    if ((data == 0) || (frame == 0))
    {
        return;
    }

    for (index = 0; index < TELEMETRY_FRAME_SIZE; index++)
    {
        frame[index] = 0;
    }

    flags = 0;
    if (data->imu_ok != 0u)
    {
        flags |= TELEMETRY_FLAG_IMU_OK;
    }
    if (data->aht_ok != 0u)
    {
        flags |= TELEMETRY_FLAG_AHT_OK;
    }
    flags |= (uint8_t)((data->mode << TELEMETRY_MODE_SHIFT) & TELEMETRY_MODE_MASK);

    frame[0] = TELEMETRY_HEAD;
    frame[1] = flags;
    frame[2] = data->mode;
    PutU16(frame, 3, (uint16_t)data->vx_mm_s);
    PutU16(frame, 5, (uint16_t)data->wz_mrad_s);
    PutU16(frame, 7, (uint16_t)data->acc_mg[0]);
    PutU16(frame, 9, (uint16_t)data->acc_mg[1]);
    PutU16(frame, 11, (uint16_t)data->acc_mg[2]);
    PutU16(frame, 13, (uint16_t)data->gyro_0d1dps[0]);
    PutU16(frame, 15, (uint16_t)data->gyro_0d1dps[1]);
    PutU16(frame, 17, (uint16_t)data->gyro_0d1dps[2]);
    PutU16(frame, 19, (uint16_t)data->imu_temp_c100);
    PutU16(frame, 21, data->rh_100);
    PutU16(frame, 23, (uint16_t)data->aht_temp_c100);
    PutU16(frame, 25, data->tick_ms);
    frame[27] = data->rx_count;
    frame[28] = data->rj_count;
    frame[29] = TELEMETRY_VERSION;
    frame[30] = Xor(frame, 30u);
    frame[31] = TELEMETRY_TAIL;
}

uint8_t Telemetry_Send(const Telemetry_Data *data)
{
    uint8_t frame[TELEMETRY_FRAME_SIZE];
    uint8_t index;

    Telemetry_Build(data, frame);
    for (index = 0; index < TELEMETRY_FRAME_SIZE; index++)
    {
        s_last_frame[index] = frame[index];
    }
    Board_UartSendBytes(frame, (uint16_t)TELEMETRY_FRAME_SIZE);
    s_sent = (uint16_t)(s_sent + 1u);

    return frame[30];
}

const volatile uint8_t *Telemetry_LastFrame(void)
{
    return s_last_frame;
}

uint16_t Telemetry_SentCount(void)
{
    return s_sent;
}
