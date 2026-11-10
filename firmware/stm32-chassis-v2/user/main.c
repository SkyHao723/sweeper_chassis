#include "stm32f10x.h"
#include "board.h"
#include "Delay.h"
#include "ws2812.h"
#include "ssd1306.h"

/**
  ******************************************************************************
  * @file    main.c
  * @brief   底盘主控 V2 (STM32F103C8T6) 第一步点灯程序
  *
  * 这一版只做两件事, 用来验证新板焊接与供电是否正常:
  *   1) OLED (SSD1306 128x64, 软件 I2C PB8/PB9) 显示板子信息 + 运行计数
  *   2) RGB 灯 (XL-5050 / WS2812 协议, PB5) 四色循环
  * 同时把同样的信息从 USART1 (PA9/PA10, 115200) 打出来, 方便没屏的时候看。
  *
  * 上电后屏幕应该看到:
  *   TB01 CHASSIS V2
  *   MCU F103C8T6
  *   CLK 72 MHZ
  *   OLED OK
  *   LED PB5 WS2812
  *   KEY UP
  *   (empty)
  *   RUN 000001
  * 如果第三行不是 72 MHZ, 说明 8MHz 晶振没起振 (退到内部 HSI 8MHz);
  * 如果第四行是 OLED NO ACK, 说明显示这一路 I2C 没应答 (接线/上拉/地址)。
  ******************************************************************************
  */

/* 颜色表: 红 绿 蓝 白 灭 */
static const uint8_t color_table[5][3] = {
    {255, 0, 0},
    {0, 255, 0},
    {0, 0, 255},
    {255, 255, 255},
    {0, 0, 0}};

static const char *Color_Name(uint8_t index)
{
    switch (index)
    {
        case 0: return "RED";
        case 1: return "GREEN";
        case 2: return "BLUE";
        case 3: return "WHITE";
        default: return "OFF";
    }
}

/* 开场: 屏幕固定 6 行 + 串口横幅 */
static void Show_BoardInfo(uint8_t oled_ok)
{
    Board_UartSend("\r\n\r\n");
    Board_UartSend("======================================\r\n");
    Board_UartSend(" TB01 chassis controller V2 bring-up\r\n");
    Board_UartSend("======================================\r\n");
    Board_UartSend("SYSCLK = ");
    Board_UartSendU32((uint32_t)(SystemCoreClock / 1000000));
    Board_UartSend(" MHz\r\n");
    Board_UartSend("OLED   = ");
    if (oled_ok != 0)
    {
        Board_UartSend("OK (SSD1306 @0x3C, soft I2C PB8/PB9)\r\n");
    }
    else
    {
        Board_UartSend("NO ACK (check wiring / pull-up / address)\r\n");
    }
    Board_UartSend("LED    = WS2812 on PB5\r\n");
    Board_UartSend("KEY    = PA15 (pull-up, active low)\r\n");
    Board_UartSend("\r\n");

    if (oled_ok == 0)
    {
        return;
    }

    Ssd1306_ShowLine(0, "TB01 CHASSIS V2");
    Ssd1306_ShowLine(1, "MCU F103C8T6");
    Ssd1306_ShowLineU32(2, "CLK ", (uint32_t)(SystemCoreClock / 1000000));
    Ssd1306_ShowLine(3, "OLED OK");
    Ssd1306_ShowLine(4, "LED PB5 WS2812");
    Ssd1306_ShowLine(5, "KEY UP");
}

int main(void)
{
    uint8_t oled_ok;
    uint8_t key_last = 0;
    uint8_t key_now;
    uint8_t color_index = 0;
    uint32_t run_count = 0;
    uint32_t next_color_ms = 1000;
    uint32_t elapsed_ms = 0;

    Board_DwtInit();
    Board_SwdOnly();       /* 先关 JTAG, PA15 才能当普通 IO */
    Board_KeyInit();
    Board_ParkUnusedPins();
    Board_UartInit(115200);

    Ws2812_Init();
    oled_ok = Ssd1306_Init();

    Show_BoardInfo(oled_ok);

    Ws2812_SetAll(color_table[color_index][0], color_table[color_index][1],
                  color_table[color_index][2]);
    Ws2812_Show();

    for (;;)
    {
        Delay_ms(100);
        elapsed_ms = elapsed_ms + 100;
        run_count++;

        /* 100ms 一次: 刷新计数行 */
        if (oled_ok != 0)
        {
            Ssd1306_ShowLineU32(7, "RUN ", run_count);
        }

        /* 按键: 按下时显示 KEY DOWN, 松开恢复, 边沿变化才动屏幕 */
        key_now = Board_KeyDown();
        if (key_now != key_last)
        {
            key_last = key_now;
            if (key_now != 0)
            {
                Board_UartSend("[key] SW1 pressed\r\n");
                if (oled_ok != 0)
                {
                    Ssd1306_ShowLine(5, "KEY DOWN");
                }
            }
            else
            {
                Board_UartSend("[key] SW1 released\r\n");
                if (oled_ok != 0)
                {
                    Ssd1306_ShowLine(5, "KEY UP");
                }
            }
        }

        /* 每 1s 换一个颜色 */
        if (elapsed_ms >= next_color_ms)
        {
            next_color_ms = next_color_ms + 1000;

            color_index = (uint8_t)((color_index + 1) % 5);
            Ws2812_SetAll(color_table[color_index][0], color_table[color_index][1],
                          color_table[color_index][2]);
            Ws2812_Show();

            Board_UartSend("[led] color = ");
            Board_UartSend(Color_Name(color_index));
            Board_UartSend("\r\n");
        }
    }
}
