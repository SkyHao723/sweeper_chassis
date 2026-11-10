#include "stm32f10x.h"
#include "board.h"
#include "softi2c.h"

/**
  ******************************************************************************
  * @file    softi2c.c
  * @brief   PB8/PB9 软件模拟 I2C (开漏 + 上拉, 主机模式)
  ******************************************************************************
  */

#define SI2C_PORT GPIOB
#define SI2C_SCL_PIN GPIO_Pin_8
#define SI2C_SDA_PIN GPIO_Pin_9
#define SI2C_CLK RCC_APB2Periph_GPIOB

/* ------------------------------ 底层时序 ------------------------------ */

static void Si2c_Delay(void)
{
    Board_DwtDelayCycles(SOFTI2C_HALF_DELAY);
}

static void Si2c_Scl(uint8_t high)
{
    if (high != 0u)
    {
        GPIO_SetBits(SI2C_PORT, SI2C_SCL_PIN);
    }
    else
    {
        GPIO_ResetBits(SI2C_PORT, SI2C_SCL_PIN);
    }
    Si2c_Delay();
}

static void Si2c_Sda(uint8_t high)
{
    if (high != 0u)
    {
        GPIO_SetBits(SI2C_PORT, SI2C_SDA_PIN);
    }
    else
    {
        GPIO_ResetBits(SI2C_PORT, SI2C_SDA_PIN);
    }
    Si2c_Delay();
}

/* ------------------------------ 对外接口 ------------------------------ */

void SoftI2c_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(SI2C_CLK, ENABLE);

    /* 先当普通推挽输出把两根都拉高(给总线一个确定的空闲状态), 再切开漏 */
    GPIO_SetBits(SI2C_PORT, SI2C_SCL_PIN | SI2C_SDA_PIN);
    gpio.GPIO_Pin = SI2C_SCL_PIN | SI2C_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SI2C_PORT, &gpio);
    Board_DwtDelayCycles(72000u * 2u); /* 等 2ms, 让挂在总线上的器件上电稳定 */

    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SI2C_PORT, &gpio);

    Si2c_Scl(1);
    Si2c_Sda(1);
}

void SoftI2c_BusReset(void)
{
    uint8_t i;

    /* 从机如果正卡在"输出低"的状态, 给它补 9 个时钟让它把字节吐完 */
    Si2c_Sda(1);
    for (i = 0; i < 9u; i++)
    {
        Si2c_Scl(1);
        Si2c_Scl(0);
    }
    SoftI2c_Stop();
}

void SoftI2c_Start(void)
{
    Si2c_Sda(1);
    Si2c_Scl(1);
    Si2c_Sda(0);
    Si2c_Scl(0);
}

void SoftI2c_Stop(void)
{
    Si2c_Sda(0);
    Si2c_Scl(1);
    Si2c_Sda(1);
}

uint8_t SoftI2c_WriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t mask;

    for (bit = 0u; bit < 8u; bit++)
    {
        mask = (uint8_t)(0x80u >> bit);
        if ((value & mask) != 0u)
        {
            Si2c_Sda(1);
        }
        else
        {
            Si2c_Sda(0);
        }
        Si2c_Scl(1);
        Si2c_Scl(0);
    }

    /* 第 9 个时钟: 释放 SDA, 读从机的应答 (低 = ACK) */
    Si2c_Sda(1);
    Si2c_Scl(1);
    if (GPIO_ReadInputDataBit(SI2C_PORT, SI2C_SDA_PIN) == Bit_RESET)
    {
        Si2c_Scl(0);
        return 0u;
    }
    Si2c_Scl(0);
    return 1u;
}

uint8_t SoftI2c_ReadByte(uint8_t ack)
{
    uint8_t bit;
    uint8_t value = 0u;

    Si2c_Sda(1); /* 释放 SDA, 让从机驱动 */
    for (bit = 0u; bit < 8u; bit++)
    {
        Si2c_Scl(1);
        value = (uint8_t)(value << 1);
        if (GPIO_ReadInputDataBit(SI2C_PORT, SI2C_SDA_PIN) != Bit_RESET)
        {
            value = (uint8_t)(value | 0x01u);
        }
        Si2c_Scl(0);
    }

    /* 第 9 个时钟: 主机回 ACK(0) 或 NAK(1) */
    if (ack != 0u)
    {
        Si2c_Sda(0);
    }
    else
    {
        Si2c_Sda(1);
    }
    Si2c_Scl(1);
    Si2c_Scl(0);
    Si2c_Sda(1);

    return value;
}

uint8_t SoftI2c_WriteBytes(uint8_t address, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t result = 0u;

    SoftI2c_Start();
    result = SoftI2c_WriteByte(address);
    for (i = 0u; (i < len) && (result == 0u); i++)
    {
        result = SoftI2c_WriteByte(data[i]);
    }
    SoftI2c_Stop();

    return result;
}

uint8_t SoftI2c_ReadBytes(uint8_t address, uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t result;

    SoftI2c_Start();
    /* ★ 读操作必须把地址字节的最低位置 1 (R/W=1), 否则从机还以为主机在写,
         它只会回 ACK 不驱动 SDA, 读回来全是 0xFF (这个 bug 真踩过) */
    result = SoftI2c_WriteByte((uint8_t)(address | 0x01u));
    if (result != 0u)
    {
        SoftI2c_Stop();
        return result;
    }

    for (i = 0u; i < len; i++)
    {
        /* 最后一个字节回 NAK, 让从机知道不用再发了 */
        data[i] = SoftI2c_ReadByte((i + 1u < len) ? 1u : 0u);
    }
    SoftI2c_Stop();

    return 0u;
}
