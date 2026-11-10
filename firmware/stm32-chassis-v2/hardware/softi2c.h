#ifndef __SOFTI2C_H
#define __SOFTI2C_H

/**
  ******************************************************************************
  * @file    softi2c.h
  * @brief   PB8/PB9 软件模拟 I2C 总线 (显示 + 温湿度共用一条)
  *
  * 为什么用软件 I2C:
  *   STM32F103 的 I2C2 只在 PB10/PB11 (这两个脚已经给 CH554G 用的 USART3),
  *   PB8/PB9 其实是 I2C1 的重映射脚 —— 硬件 I2C1 留给 IMU (PB6/PB7),
  *   所以显示和 AHT20 这条总线就用软件模拟, 引脚随便挑。
  *
  * 电气: PB8/PB9 配成开漏输出(Out_OD), 外部必须有上拉(模块自带 4.7k 即可)。
  *       写 1 = 释放总线(靠上拉变高), 写 0 = 主动拉低;
  *       读 IDR 就是读实际电平, 所以读 ACK 不用切换输入模式。
  ******************************************************************************
  */

#include "stm32f10x.h"

/* 半周期延时(CPU 周期数)。1 圈约 13.9ns, 200 ≈ 2.8us => SCL ≈ 180kHz */
#define SOFTI2C_HALF_DELAY 200u

/* 把 PB8/PB9 配成开漏, 并给总线一个空闲状态(两根都释放) */
void SoftI2c_Init(void);

/* 总线复位: 如果从机卡住把 SDA 拉低, 打 9 个时钟再发 STOP 把它放出来 */
void SoftI2c_BusReset(void);

void SoftI2c_Start(void);
void SoftI2c_Stop(void);

/* 写一个字节, 返回 0 = 收到 ACK, 1 = 没有 ACK */
uint8_t SoftI2c_WriteByte(uint8_t value);

/* 读一个字节; ack != 0 表示读完要回 ACK(继续读), 0 表示回 NAK(最后一个字节) */
uint8_t SoftI2c_ReadByte(uint8_t ack);

/* ------------------------------ 常用事务 ------------------------------ */

/* 写: START + addr + data[0..len-1] + STOP。返回 0 = 成功 */
uint8_t SoftI2c_WriteBytes(uint8_t address, const uint8_t *data, uint8_t len);

/* 读(带重复起始): START + addr|R + 读 len 字节 + STOP。返回 0 = 成功 */
uint8_t SoftI2c_ReadBytes(uint8_t address, uint8_t *data, uint8_t len);

#endif
