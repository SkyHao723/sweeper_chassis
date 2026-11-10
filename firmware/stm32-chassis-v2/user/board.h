#ifndef __BOARD_H
#define __BOARD_H

/**
  ******************************************************************************
  * @file    board.h
  * @brief   底盘主控 V2 板级定义 (STM32F103C8T6)
  *
  * 新板引脚分配 (来自原理图, 2026-11-10 评审)
  * ---------------------------------------------------------------------------
  * 功能            引脚      说明
  * 8MHz 晶振       PD0/PD1   X1 + 18pF, SystemInit() 自动 PLL 到 72MHz
  * SWD 调试        PA13/PA14 SWDIO/SWCLK, 千万不能占用
  * 调试串口        PA9/PA10  USART1_TX/RX -> CH343P -> USB
  * CAN             PA11/PA12 CAN_RX/CAN_TX -> TJA1050 (注: TJA1050 需 5V)
  * IMU (I2C1)      PB6/PB7   ICM-42670-P, SCL/SDA (硬件 I2C1)
  * 显示 (软件I2C)  PB8/PB9   SSD1306 128x64 + AHT20, 软件模拟 I2C
  * USB 协处理      PB10/PB11 USART3 (CH554G: USB-A 主机口)
  * RGB 灯          PB5       XL-5050 (WS2812 单线协议)
  * 按键 SW1        PA15      上拉输入, 按下为低 (复用 JTDI, 需关 JTAG)
  * ---------------------------------------------------------------------------
  * 扩展排针 CN5: PA0(ADC/WKUP) PA1 PA2 PA4
  * 扩展排针 CN6: PA7 PB0 PB1 PB2(注意: PB2=BOOT1, 复位时拉高会从 SRAM 启动)
  * 未连接(打X): PC13/14/15 PA5 PA6 PA8 PB3 PB4 PB12~PB15
  ******************************************************************************
  */

#include "stm32f10x.h"

/* ------------------------------ 引脚 ------------------------------ */
/* RGB 灯 (WS2812 单线, 5V 供电, 3.3V 电平需确认灯珠 VIH) 的引脚
   定义在 ws2812.h 里, 因为时序代码要直接操作 BSRR/BRR */

/* 按键 SW1 */
#define BOARD_KEY_PORT GPIOA
#define BOARD_KEY_PIN GPIO_Pin_15
#define BOARD_KEY_CLK RCC_APB2Periph_GPIOA

/* 软件 I2C 显示屏/温湿度 */
#define BOARD_I2C_PORT GPIOB
#define BOARD_I2C_SCL_PIN GPIO_Pin_8
#define BOARD_I2C_SDA_PIN GPIO_Pin_9
#define BOARD_I2C_CLK RCC_APB2Periph_GPIOB

/* 调试串口 USART1 */
#define BOARD_UART USART1
#define BOARD_UART_CLK RCC_APB2Periph_USART1
#define BOARD_UART_GPIO_CLK RCC_APB2Periph_GPIOA

/* ------------------------------ 接口 ------------------------------ */

/* DWT 周期计数器: 工程里的 core_cm3.h 是精简版, 没有 DWT/CoreDebug 定义,
   这里直接给出寄存器地址 (Cortex-M3 内核自带的调试单元) */
#define BOARD_DEMCR (*(volatile uint32_t *)0xE000EDFCUL)
#define BOARD_DWT_CTRL (*(volatile uint32_t *)0xE0001000UL)
#define BOARD_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define BOARD_DEMCR_TRCENA (1UL << 24)
#define BOARD_DWT_CTRL_CYCCNTENA (1UL << 0)

/* DWT 周期计数器 (WS2812 / 软件 I2C 用来自旋微秒级延时) */
void Board_DwtInit(void);
void Board_DwtDelayCycles(uint32_t cycles);

/* 关掉 JTAG, 只保留 SWD (PA15/PB3/PB4 才能当普通 IO 用) */
void Board_SwdOnly(void);

/* 未使用的引脚统一配成模拟输入, 降低功耗与干扰 */
void Board_ParkUnusedPins(void);

/* 调试串口 USART1 115200 8N1 */
void Board_UartInit(uint32_t baudrate);
void Board_UartPutChar(char ch);
void Board_UartSend(const char *text);
void Board_UartSendU32(uint32_t value);
void Board_UartSendHex8(uint8_t value);

/* 板载按键: 1 = 按下, 0 = 松开 */
void Board_KeyInit(void);
uint8_t Board_KeyDown(void);

#endif
