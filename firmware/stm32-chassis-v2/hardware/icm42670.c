#include "stm32f10x.h"
#include "board.h"
#include "icm42670.h"

/**
  ******************************************************************************
  * @file    icm42670.c
  * @brief   ICM-42670-P 驱动 (硬件 I2C1, PB6/PB7, 400kHz 主机轮询)
  *
  * 关键寄存器:
  *   0x75 WHO_AM_I        固定 0x67
  *   0x1F PWR_MGMT0       bit1:0 ACCEL_MODE, bit3:2 GYRO_MODE  (3 = 低噪声)
  *   0x20 GYRO_CONFIG0    bit7:5 FS_SEL, bit3:0 ODR
  *   0x21 ACCEL_CONFIG0   bit7:5 FS_SEL, bit3:0 ODR
  *   0x09 TEMP_DATA1..2   温度 16bit 有符号, T = raw/128 + 25
  *   0x0B..0x10           ACCEL X1/X0,Y1/Y0,Z1/Z0
  *   0x11..0x16           GYRO  X1/X0,Y1/Y0,Z1/Z0
  *
  * 量程选择:
  *   ACCEL_FS 010 = ±4g   -> 8192  LSB/g
  *   GYRO_FS  001 = ±1000dps -> 32.8 LSB/(°/s)
  *   ODR 6 = 200Hz (加速度和陀螺都选这个)
  ******************************************************************************
  */

#define ICM_I2C I2C1
#define ICM_I2C_CLK RCC_APB1Periph_I2C1
#define ICM_GPIO_CLK RCC_APB2Periph_GPIOB
#define ICM_SCL_PIN GPIO_Pin_6
#define ICM_SDA_PIN GPIO_Pin_7

#define ICM_REG_WHO_AM_I 0x75u
#define ICM_REG_PWR_MGMT0 0x1Fu
#define ICM_REG_GYRO_CONFIG0 0x20u
#define ICM_REG_ACCEL_CONFIG0 0x21u
#define ICM_REG_TEMP_DATA1 0x09u
#define ICM_REG_ACCEL_DATA_X1 0x0Bu
#define ICM_REG_GYRO_DATA_X1 0x11u

#define ICM_ADDR_PRIMARY 0x68u
#define ICM_ADDR_ALT 0x69u

/* 配置值: 低噪声模式 / 200Hz / ±4g / ±1000dps */
#define ICM_PWR_MGMT0_LN 0x0Fu
#define ICM_ACCEL_CONFIG0_VAL 0x46u /* 010(±4g) << 5 | 6(200Hz) */
#define ICM_GYRO_CONFIG0_VAL 0x26u  /* 001(±1000dps) << 5 | 6(200Hz) */

/* I2C 等待超时(循环次数)。I2C 卡死时不能死等, 要让上层继续跑 */
#define ICM_TIMEOUT 20000u

static uint8_t s_addr;
static uint8_t s_who_am_i;

/* ------------------------------ I2C 底层 ------------------------------ */

static uint8_t Icm_WaitFlag(uint32_t flag, FlagStatus status)
{
    uint32_t guard = 0;

    while (I2C_GetFlagStatus(ICM_I2C, flag) != status)
    {
        guard++;
        if (guard > ICM_TIMEOUT)
        {
            return 0;
        }
    }
    return 1;
}

static void Icm_I2cInit(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_APB2PeriphClockCmd(ICM_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(ICM_I2C_CLK, ENABLE);

    /* PB6/PB7 = 复用开漏 (AF_OD); F103 的 I2C 引脚必须开漏 + 外部上拉 */
    gpio.GPIO_Pin = ICM_SCL_PIN | ICM_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(ICM_I2C);
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x00;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000; /* 400kHz 快速模式 */
    I2C_Init(ICM_I2C, &i2c);
    I2C_Cmd(ICM_I2C, ENABLE);
}

/* 写一个寄存器 */
static uint8_t Icm_WriteReg(uint8_t reg, uint8_t value)
{
    if (Icm_WaitFlag(I2C_FLAG_BUSY, RESET) == 0)
    {
        return 0;
    }

    I2C_GenerateSTART(ICM_I2C, ENABLE);
    if (Icm_WaitFlag(I2C_FLAG_SB, SET) == 0)
    {
        return 0;
    }

    I2C_Send7bitAddress(ICM_I2C, (uint8_t)(s_addr << 1), I2C_Direction_Transmitter);
    if (Icm_WaitFlag(I2C_FLAG_ADDR, SET) == 0)
    {
        return 0;
    }
    (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR1);
    (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR2);

    if (Icm_WaitFlag(I2C_FLAG_TXE, SET) == 0)
    {
        return 0;
    }
    I2C_SendData(ICM_I2C, reg);

    if (Icm_WaitFlag(I2C_FLAG_TXE, SET) == 0)
    {
        return 0;
    }
    I2C_SendData(ICM_I2C, value);

    if (Icm_WaitFlag(I2C_FLAG_BTF, SET) == 0)
    {
        return 0;
    }
    I2C_GenerateSTOP(ICM_I2C, ENABLE);

    return 1;
}

/* 从寄存器开始连读 len 个字节 (自动递增地址) */
static uint8_t Icm_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t index;

    if (Icm_WaitFlag(I2C_FLAG_BUSY, RESET) == 0)
    {
        return 0;
    }

    I2C_GenerateSTART(ICM_I2C, ENABLE);
    if (Icm_WaitFlag(I2C_FLAG_SB, SET) == 0)
    {
        return 0;
    }

    I2C_Send7bitAddress(ICM_I2C, (uint8_t)(s_addr << 1), I2C_Direction_Transmitter);
    if (Icm_WaitFlag(I2C_FLAG_ADDR, SET) == 0)
    {
        return 0;
    }
    (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR1);
    (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR2);

    if (Icm_WaitFlag(I2C_FLAG_TXE, SET) == 0)
    {
        return 0;
    }
    I2C_SendData(ICM_I2C, reg);

    if (Icm_WaitFlag(I2C_FLAG_BTF, SET) == 0)
    {
        return 0;
    }

    /* 重复起始 + 读方向 */
    I2C_GenerateSTART(ICM_I2C, ENABLE);
    if (Icm_WaitFlag(I2C_FLAG_SB, SET) == 0)
    {
        return 0;
    }
    I2C_Send7bitAddress(ICM_I2C, (uint8_t)(s_addr << 1), I2C_Direction_Receiver);
    if (Icm_WaitFlag(I2C_FLAG_ADDR, SET) == 0)
    {
        return 0;
    }

    for (index = 0; index < len; index++)
    {
        if ((uint8_t)(index + 1u) == len)
        {
            /* 最后一个字节: 先关 ACK 再清 ADDR, 读完发 STOP */
            I2C_AcknowledgeConfig(ICM_I2C, DISABLE);
        }
        if (index == 0u)
        {
            (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR1);
            (void)I2C_ReadRegister(ICM_I2C, I2C_Register_SR2);
        }

        if (Icm_WaitFlag(I2C_FLAG_RXNE, SET) == 0)
        {
            I2C_AcknowledgeConfig(ICM_I2C, ENABLE);
            return 0;
        }
        buffer[index] = I2C_ReceiveData(ICM_I2C);

        if ((uint8_t)(index + 1u) == len)
        {
            I2C_GenerateSTOP(ICM_I2C, ENABLE);
        }
    }

    I2C_AcknowledgeConfig(ICM_I2C, ENABLE);
    return 1;
}

/* ------------------------------ 对外接口 ------------------------------ */

uint8_t Icm42670_WhoAmI(void)
{
    return s_who_am_i;
}

uint8_t Icm42670_Address(void)
{
    return s_addr;
}

uint8_t Icm42670_Init(void)
{
    uint8_t value = 0;

    s_addr = 0;
    s_who_am_i = 0;

    Icm_I2cInit();

    /* 先试 0x68, 不行再试 0x69 (AD0 接高就是 0x69) */
    s_addr = ICM_ADDR_PRIMARY;
    if ((Icm_ReadRegs(ICM_REG_WHO_AM_I, &value, 1u) != 0u) && (value == ICM_WHO_AM_I_EXPECTED))
    {
        s_who_am_i = value;
    }
    else
    {
        s_addr = ICM_ADDR_ALT;
        value = 0;
        if ((Icm_ReadRegs(ICM_REG_WHO_AM_I, &value, 1u) != 0u) && (value == ICM_WHO_AM_I_EXPECTED))
        {
            s_who_am_i = value;
        }
        else
        {
            s_addr = 0;
            return 0;
        }
    }

    /* 配置量程与输出速率, 然后打开低噪声模式 */
    if (Icm_WriteReg(ICM_REG_GYRO_CONFIG0, (uint8_t)ICM_GYRO_CONFIG0_VAL) == 0u)
    {
        return 0;
    }
    if (Icm_WriteReg(ICM_REG_ACCEL_CONFIG0, (uint8_t)ICM_ACCEL_CONFIG0_VAL) == 0u)
    {
        return 0;
    }
    if (Icm_WriteReg(ICM_REG_PWR_MGMT0, (uint8_t)ICM_PWR_MGMT0_LN) == 0u)
    {
        return 0;
    }

    /* 陀螺从 standby 到稳定输出需要 ~45ms */
    Board_DwtDelayCycles(72000u * 60u);

    return 1;
}

uint8_t Icm42670_Read(Icm42670_Sample *sample)
{
    uint8_t raw[14];
    int32_t value;

    if (sample == 0)
    {
        return 0;
    }
    if (s_addr == 0u)
    {
        return 0;
    }

    /* 0x09..0x16 共 14 字节: 温度(2) + 加速度(6) + 陀螺(6) */
    if (Icm_ReadRegs(ICM_REG_TEMP_DATA1, raw, 14u) == 0u)
    {
        return 0;
    }

    /* 温度: T = raw / 128 + 25  ->  0.01°C = raw*100/128 + 2500 */
    value = (int32_t)(int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    sample->temp_c100 = (int16_t)((value * 100) / 128 + 2500);

    /* 加速度: ±4g -> 8192 LSB/g ; mg = raw * 1000 / 8192 */
    sample->acc_mg[0] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[2] << 8) | raw[3])) * 1000 / 8192);
    sample->acc_mg[1] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[4] << 8) | raw[5])) * 1000 / 8192);
    sample->acc_mg[2] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[6] << 8) | raw[7])) * 1000 / 8192);

    /* 陀螺: ±1000dps -> 32.8 LSB/(°/s) ; 0.1dps = raw * 10 / 32.8 */
    sample->gyro_0d1dps[0] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[8] << 8) | raw[9])) * 1000 / 3280);
    sample->gyro_0d1dps[1] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[10] << 8) | raw[11])) * 1000 / 3280);
    sample->gyro_0d1dps[2] = (int16_t)(((int32_t)(int16_t)(((uint16_t)raw[12] << 8) | raw[13])) * 1000 / 3280);

    return 1;
}
