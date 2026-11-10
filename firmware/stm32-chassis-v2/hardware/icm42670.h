#ifndef __ICM42670_H
#define __ICM42670_H

/**
  ******************************************************************************
  * @file    icm42670.h
  * @brief   ICM-42670-P 六轴 IMU (硬件 I2C1: PB6=SCL, PB7=SDA)
  *
  * 为什么走硬件 I2C1:
  *   F103 的 I2C1 默认就在 PB6/PB7, 而 I2C2 在 PB10/PB11 (被 CH554G 占了),
  *   显示+AHT20 那条用了软件 I2C, 两条总线互不干扰。
  *
  * 配置: 加速度 ±4g / 陀螺 ±1000dps, 输出数据率 200Hz, 低噪声模式。
  *       量程换算: 加速度 8192 LSB/g, 陀螺 32.8 LSB/(°/s)。
  ******************************************************************************
  */

#include "stm32f10x.h"

#define ICM_WHO_AM_I_EXPECTED 0x67u

/* 一次采样的结果 */
typedef struct
{
    int16_t acc_mg[3];       /* 加速度, 单位 0.001 g (mg) */
    int16_t gyro_0d1dps[3];  /* 角速度, 单位 0.1 °/s */
    int16_t temp_c100;       /* 芯片内部温度, 单位 0.01 °C */
} Icm42670_Sample;

/* 初始化: 探测 WHO_AM_I, 写 PWR_MGMT0 上电并配置量程/速率。1 = 成功 */
uint8_t Icm42670_Init(void);

/* 最近一次读到的 WHO_AM_I (0 = 没读到) */
uint8_t Icm42670_WhoAmI(void);

/* 探测到的 7 位从机地址 (0x68 / 0x69 / 0 = 没探测到) */
uint8_t Icm42670_Address(void);

/* 读一帧数据。1 = 成功; 0 = I2C 出错 */
uint8_t Icm42670_Read(Icm42670_Sample *sample);

#endif
