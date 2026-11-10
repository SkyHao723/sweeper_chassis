#ifndef __SSD1306_H
#define __SSD1306_H

/**
  ******************************************************************************
  * @file    ssd1306.h
  * @brief   0.96 寸 OLED (SSD1306, 128x64) 驱动 —— 软件模拟 I2C (PB8/PB9)
  *
  * 为什么不硬件 I2C:
  *   STM32F103 的 I2C2 只在 PB10/PB11 上, 而这块板子把 PB10/PB11 给了 CH554G,
  *   PB8/PB9 是 I2C1 的重映射脚。IMU(ICM-42670-P) 走硬件 I2C1(PB6/PB7),
  *   显示 + AHT20 这一路就用软件 I2C, 互不干扰。
  *
  * 总线地址: OLED = 0x3C (7bit) -> 0x78 (8bit 写), AHT20 = 0x38
  * 页寻址: 8 页 x 128 列, 每页 8 像素高, 5x7 字模 + 1 列间隔 => 每页 21 字符
  ******************************************************************************
  */

#include "stm32f10x.h"

/* 返回 1 = 初始化成功(器件应答), 0 = 无应答(没接线/地址错/上拉缺失) */
uint8_t Ssd1306_Init(void);

/* 单独探测一次器件是否应答, 适合拿来点灯式排查 */
uint8_t Ssd1306_Probe(void);

void Ssd1306_Clear(void);

/* page 0~7, 每行最多 21 个字符 (超出部分自动截断/补空) */
void Ssd1306_ShowLine(uint8_t page, const char *text);

/* 显示 "前缀 + 十进制数", 例如 Ssd1306_ShowLineU32(3, "CNT ", n) -> "CNT 123" */
void Ssd1306_ShowLineU32(uint8_t page, const char *prefix, uint32_t value);

#endif
