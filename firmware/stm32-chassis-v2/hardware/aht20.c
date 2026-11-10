#include "stm32f10x.h"
#include "board.h"
#include "softi2c.h"
#include "aht20.h"

/**
  ******************************************************************************
  * @file    aht20.c
  * @brief   AHT20 温湿度传感器驱动 (软件 I2C, 与 OLED 共用 PB8/PB9)
  *
  * 时序(手册):
  *   上电 -> 等 >=40ms -> 读状态字节: bit3 = CAL 校准位
  *         若 CAL=0, 发 0xBE 0x08 0x00 初始化(软复位+校准), 等 10ms
  *   测量: 发 0xAC 0x33 0x00 -> 等 80ms -> 读 7 字节
  *         [0] 状态, [1..3] 湿度高 20bit 的一部分, [3..6] 温度
  *         湿度 = ((b1<<12)|(b2<<4)|(b3>>4)) / 2^20 * 100 %RH
  *         温度 = (((b3&0x0F)<<16)|(b4<<8)|b5) / 2^20 * 200 - 50 °C
  ******************************************************************************
  */

#define AHT20_CYCLES_PER_MS 72000u   /* 72MHz 主频下 1ms 的周期数 */
#define AHT20_POWERUP_MS 45u         /* 手册要求 >=40ms, 留点余量 */
#define AHT20_CALIB_MS 10u

static uint8_t s_ok;
static uint8_t s_busy;
static uint32_t s_trigger_cycles;

/* 调试用: 最近一次读到的 7 个原始字节, 以及统计。
   没接串口时可以用调试器 dump 这段 RAM 判断是"没 ACK / 一直 BUSY / 数据不对"。 */
static volatile uint8_t s_raw[7];
static volatile uint16_t s_reads;   /* 成功走完 7 字节读的次数 */
static volatile uint16_t s_busy_hits; /* 读到 BUSY=1 的次数 */
static volatile uint16_t s_ok_reads;  /* 真正解析成功的次数 */

uint8_t Aht20_Init(void)
{
    uint8_t status = 0;
    static const uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};

    s_ok = 0;
    s_busy = 0;

    /* 上电稳定 */
    Board_DwtDelayCycles(AHT20_CYCLES_PER_MS * AHT20_POWERUP_MS);

    /* 读状态字节 (单字节读: 不写寄存器地址, 直接读) */
    if (SoftI2c_ReadBytes(AHT20_I2C_ADDRESS, &status, 1u) != 0u)
    {
        return 0;
    }

    /* bit3 = CAL, 0 表示没校准过, 需要发一次初始化命令 */
    if ((status & 0x08u) == 0u)
    {
        if (SoftI2c_WriteBytes(AHT20_I2C_ADDRESS, init_cmd, 3u) != 0u)
        {
            return 0;
        }
        Board_DwtDelayCycles(AHT20_CYCLES_PER_MS * AHT20_CALIB_MS);
    }

    s_ok = 1;
    return 1;
}

uint8_t Aht20_Trigger(void)
{
    static const uint8_t trigger_cmd[3] = {0xAC, 0x33, 0x00};

    if (s_ok == 0u)
    {
        return 0;
    }
    if (SoftI2c_WriteBytes(AHT20_I2C_ADDRESS, trigger_cmd, 3u) != 0u)
    {
        s_ok = 0; /* 掉线了, 上层下次会重新 Init */
        return 0;
    }

    s_trigger_cycles = BOARD_DWT_CYCCNT;
    s_busy = 1;
    return 1;
}

uint8_t Aht20_Poll(int16_t *temp_c100, uint16_t *rh_100)
{
    uint8_t raw[7];
    uint8_t index;
    uint32_t humidity;
    uint32_t temperature;

    if ((s_busy == 0u) || (s_ok == 0u))
    {
        return 0;
    }

    /* 还没到 80ms, 直接返回, 让主循环继续干别的 */
    if ((BOARD_DWT_CYCCNT - s_trigger_cycles) < (AHT20_CYCLES_PER_MS * AHT20_TRIGGER_MS))
    {
        return 0;
    }

    s_busy = 0;

    if (SoftI2c_ReadBytes(AHT20_I2C_ADDRESS, raw, 7u) != 0u)
    {
        s_ok = 0;
        return 0;
    }

    for (index = 0; index < 7u; index++)
    {
        s_raw[index] = raw[index];
    }
    s_reads++;

    /* 忙位: bit7 BUSY, 还忙就等下一轮 */
    if ((raw[0] & 0x80u) != 0u)
    {
        s_busy_hits++;
        s_busy = 1;
        return 0;
    }

    humidity = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | ((uint32_t)raw[3] >> 4);
    temperature = (((uint32_t)raw[3] & 0x0Fu) << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[5];

    s_ok_reads++;

    if (temp_c100 != 0)
    {
        /* T = raw/2^20*200 - 50 ; 0.01°C = raw*200*100/2^20 = raw*625/32768
           (先乘 625 再除, raw*625 最大 655M, 不会溢出 uint32) */
        *temp_c100 = (int16_t)((temperature * 625u) / 32768u - 5000u);
    }
    if (rh_100 != 0)
    {
        /* RH = raw/2^20*100 ; 0.01%RH = raw*10000/2^20 = raw*625/65536 */
        *rh_100 = (uint16_t)((humidity * 625u) / 65536u);
    }

    return 1;
}

uint8_t Aht20_IsOk(void)
{
    return s_ok;
}

uint8_t Aht20_HasSample(void)
{
    return (s_ok_reads > 0u) ? 1u : 0u;
}
