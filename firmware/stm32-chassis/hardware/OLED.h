#ifndef OLED_H
#define OLED_H

#include "stm32f10x.h"

/* 0.96 inch SSD1306 I2C module, PB6=SCL and PB7=SDA. */
void OLED_Init(void);
void OLED_ShowLine(uint8_t page, const char *text);

#endif
