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
  * 功能:
  *   1) OLED (SSD1306 128x64, 软件 I2C PB8/PB9) 显示板子信息、当前颜色、运行计数
  *   2) RGB 灯 (XL-5050 / WS2812 协议, PB5) —— 上电红色, 按一下 PA15 换下一个颜色
  *      颜色顺序: 红 -> 绿 -> 蓝 -> 白 -> 灭 -> 红 ... (亮度见 WS2812_BRIGHTNESS_PERCENT, 默认 50%)
  *   3) 同样的信息从 USART1 (PA9/PA10, 115200) 打出来, 方便没屏时看
  *
  * 上电后屏幕应该看到:
  *   TB01 CHASSIS V2
  *   MCU F103C8T6
  *   CLK 72 MHZ
  *   OLED OK
  *   LED PB5 WS2812
  *   KEY UP
  *   COLOR RED
  *   RUN 000123
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

/* 屏幕第 6 页显示 "COLOR 颜色名" */
static void Show_ColorLine(uint8_t oled_ok, uint8_t index)
{
    char line[16];
    const char *name;
    uint8_t pos = 0;
    uint8_t i;

    if (oled_ok == 0)
    {
        return;
    }

    name = Color_Name(index);

    /* 前缀 "COLOR " */
    for (i = 0; i < 6; i++)
    {
        line[pos] = "COLOR "[i];
        pos++;
    }
    /* 颜色名 */
    while ((name[0] != '\0') && (pos < 15))
    {
        line[pos] = name[0];
        pos++;
        name++;
    }
    line[pos] = '\0';

    Ssd1306_ShowLine(6, line);
}

/* 切换颜色: 点灯 + 屏幕 + 串口, 三处一致 */
static void Apply_Color(uint8_t oled_ok, uint8_t index)
{
    Ws2812_SetAll(color_table[index][0], color_table[index][1],
                  color_table[index][2]);
    Ws2812_Show();

    Show_ColorLine(oled_ok, index);

    Board_UartSend("[led] color = ");
    Board_UartSend(Color_Name(index));
    Board_UartSend("\r\n");
}

/* 开场: 屏幕固定行 + 串口横幅 */
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
    Board_UartSend("LED    = WS2812 on PB5, brightness ");
    Board_UartSendU32(WS2812_BRIGHTNESS_PERCENT);
    Board_UartSend(" %\r\n");
    Board_UartSend("KEY    = PA15 (pull-up, active low), press to change color\r\n");
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
    uint8_t color_index = 0;
    uint8_t run_count_lo = 0;
    uint32_t run_count = 0;

    /* 按键消抖用 */
    uint8_t key_raw = 0;
    uint8_t key_state = 0;
    uint8_t key_stable_cnt = 0;

    Board_DwtInit();
    Board_SwdOnly();       /* 先关 JTAG, PA15 才能当普通 IO */
    Board_KeyInit();
    Board_ParkUnusedPins();
    Board_UartInit(115200);

    Ws2812_Init();
    oled_ok = Ssd1306_Init();

    Show_BoardInfo(oled_ok);
    Apply_Color(oled_ok, color_index);

    for (;;)
    {
        Delay_ms(10);

        /* 每 100ms 刷一次运行计数 (软件 I2C 刷一整页约 3ms, 别刷太勤) */
        run_count_lo = (uint8_t)(run_count_lo + 1);
        if (run_count_lo >= 10)
        {
            run_count_lo = 0;
            run_count++;
            if (oled_ok != 0)
            {
                Ssd1306_ShowLineU32(7, "RUN ", run_count);
            }
        }

        /* 按键: 连续 3 次采样(30ms)一致才认, 只在"按下"的边沿换色 */
        if (Board_KeyDown() != 0)
        {
            if (key_raw == 0)
            {
                key_raw = 1;
                key_stable_cnt = 0;
            }
            else if (key_stable_cnt < 3)
            {
                key_stable_cnt++;
            }
        }
        else
        {
            if (key_raw != 0)
            {
                key_raw = 0;
                key_stable_cnt = 0;
            }
            else if (key_stable_cnt < 3)
            {
                key_stable_cnt++;
            }
        }

        if ((key_stable_cnt >= 3) && (key_raw != key_state))
        {
            key_state = key_raw;

            if (key_state != 0)
            {
                /* 按下: 换下一个颜色 */
                color_index = (uint8_t)((color_index + 1) % 5);
                Apply_Color(oled_ok, color_index);

                if (oled_ok != 0)
                {
                    Ssd1306_ShowLine(5, "KEY DOWN");
                }
                Board_UartSend("[key] SW1 pressed\r\n");
            }
            else
            {
                if (oled_ok != 0)
                {
                    Ssd1306_ShowLine(5, "KEY UP");
                }
                Board_UartSend("[key] SW1 released\r\n");
            }
        }
    }
}
