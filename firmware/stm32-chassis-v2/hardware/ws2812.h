#ifndef __WS2812_H
#define __WS2812_H

/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   WS2812/XL-5050 单线 RGB 灯驱动 (PB5)
  *
  * 时序 (72MHz, 1 周期 = 13.89ns):
  *   逻辑 0: 高 350ns  -> 25 周期, 低 900ns
  *   逻辑 1: 高 700ns  -> 50 周期, 低 550ns
  *   位周期: 1250ns    -> 90 周期
  *   复位:   低电平 >= 50us (这里给 300us 更稳)
  * 数据顺序: 每个灯 24bit, 高字节在前, 顺序是 G -> R -> B (注意不是 RGB)
  ******************************************************************************
  */

#include "stm32f10x.h"

/* RGB 灯数量 (板上 XL-5050 是 1 颗, 以后串更多灯改这里) */
#define WS2812_LED_COUNT 1

/* 亮度百分比: 0~100。WS2812 全亮(255,255,255)非常刺眼, 默认只给 30%。
   注意这是"软件调光", 直接缩小 RGB 数值; 真正的亮度上限由灯珠和限流决定。 */
#define WS2812_BRIGHTNESS_PERCENT 30

void Ws2812_Init(void);
void Ws2812_SetPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
void Ws2812_SetAll(uint8_t red, uint8_t green, uint8_t blue);
void Ws2812_Show(void);

#endif
