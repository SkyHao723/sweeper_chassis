#include "stm32f10x.h"
#include "board.h"
#include "ws2812.h"

/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   WS2812 单线 RGB 灯驱动
  *
  * 说明:
  *  - 用 DWT 周期计数器做位定时, 比 Delay_us 精确, 也不会被 SysTick 干扰
  *  - 发送期间关中断: 一旦被打断, 后面的位全会错位 (整条灯带乱闪)
  *  - 时序余量: WS2812B 的 T0H 容差 ±150ns, 我们取 350ns/700ns 在中间
  ******************************************************************************
  */

#define WS_LED_PORT GPIOB
#define WS_LED_PIN GPIO_Pin_5
#define WS_LED_CLK RCC_APB2Periph_GPIOB

#define WS_T0H_CYCLES 25    /* 350ns @72MHz */
#define WS_T1H_CYCLES 50    /* 700ns @72MHz */
#define WS_PERIOD_CYCLES 90 /* 1250ns @72MHz */
#define WS_RESET_CYCLES 21600 /* 300us @72MHz */

/* 每个灯 3 字节, 顺序 G, R, B */
static uint8_t ws_buf[WS2812_LED_COUNT * 3];

void Ws2812_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(WS_LED_CLK, ENABLE);

    /* 空闲保持低电平 (WS2812 靠 >50us 低电平复位) */
    GPIO_ResetBits(WS_LED_PORT, WS_LED_PIN);

    gpio.GPIO_Pin = WS_LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WS_LED_PORT, &gpio);

    Ws2812_SetAll(0, 0, 0);
    Ws2812_Show();
}

void Ws2812_SetPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t base;

    if (index >= WS2812_LED_COUNT)
    {
        return;
    }

    /* 软件调光: 按 WS2812_BRIGHTNESS_PERCENT 等比缩小, 灯的"颜色比例"不变 */
    base = (uint16_t)(index * 3);
    ws_buf[base + 0] = (uint8_t)(((uint16_t)green * WS2812_BRIGHTNESS_PERCENT) / 100);
    ws_buf[base + 1] = (uint8_t)(((uint16_t)red * WS2812_BRIGHTNESS_PERCENT) / 100);
    ws_buf[base + 2] = (uint8_t)(((uint16_t)blue * WS2812_BRIGHTNESS_PERCENT) / 100);
}

void Ws2812_SetAll(uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t index;

    for (index = 0; index < WS2812_LED_COUNT; index++)
    {
        Ws2812_SetPixel(index, red, green, blue);
    }
}

void Ws2812_Show(void)
{
    uint16_t index;
    uint8_t value;
    uint8_t mask;
    uint32_t bit_start;
    uint32_t elapsed;

    __disable_irq();

    for (index = 0; index < (uint16_t)sizeof(ws_buf); index++)
    {
        value = ws_buf[index];
        for (mask = 0x80; mask != 0; mask = (uint8_t)(mask >> 1))
        {
            bit_start = BOARD_DWT_CYCCNT;

            if ((value & mask) != 0)
            {
                WS_LED_PORT->BSRR = WS_LED_PIN; /* 直接写寄存器, 省掉函数调用开销 */
                do
                {
                    elapsed = BOARD_DWT_CYCCNT - bit_start;
                } while (elapsed < WS_T1H_CYCLES);
            }
            else
            {
                WS_LED_PORT->BSRR = WS_LED_PIN; /* 直接写寄存器, 省掉函数调用开销 */
                do
                {
                    elapsed = BOARD_DWT_CYCCNT - bit_start;
                } while (elapsed < WS_T0H_CYCLES);
            }

            WS_LED_PORT->BRR = WS_LED_PIN;
            do
            {
                elapsed = BOARD_DWT_CYCCNT - bit_start;
            } while (elapsed < WS_PERIOD_CYCLES);
        }
    }

    __enable_irq();

    /* 复位: 拉低保持 >50us, 锁存数据 */
    Board_DwtDelayCycles(WS_RESET_CYCLES);
}
