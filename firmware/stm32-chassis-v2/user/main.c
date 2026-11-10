#include "stm32f10x.h"
#include "board.h"
#include "Delay.h"
#include "ws2812.h"
#include "ssd1306.h"
#include "mode.h"
#include "proto.h"

/**
  ******************************************************************************
  * @file    main.c
  * @brief   底盘主控 V2 (STM32F103C8T6) —— 点灯 + 工作模式
  *
  * 屏幕 8 行:
  *   0  TB01 CHASSIS V2
  *   1  MCU F103C8T6
  *   2  CLK 72 MHZ          ← 不是 72 说明 8MHz 晶振没起振
  *   3  OLED OK             ← NO ACK 说明显示 I2C 不通
  *   4  MODE HOST           ← 当前工作模式
  *   5  UART CMD OK         ← 该模式的一句话说明
  *   6  RX 12 RJ 0          ← 接受的速度帧数 / 因模式不对被丢的帧数
  *   7  RUN 000123
  *
  * 三种模式 (见 mode.h):
  *   HOST 上位机控制  灯=蓝   接受上位机速度命令
  *   RC   遥控        灯=绿   忽略上位机速度命令
  *   PUSH 推行        灯=橙   电机释放, 忽略上位机速度命令
  *
  * 切换方式:
  *   1) 按板载按键 PA15: 每按一下切下一个 (HOST -> RC -> PUSH -> HOST)
  *   2) 上位机发功能帧: 7A 02 <mode> 00 00 00 <bcc> 7D
  *      上位机控制 7A 02 00 00 00 00 78 7D
  *      遥控       7A 02 01 00 00 00 79 7D
  *      推行       7A 02 02 00 00 00 7A 7D
  *
  * 串口 (USART1, 115200 8N1) 同时打印横幅、模式切换、命令统计。
  ******************************************************************************
  */

/* 每种模式的灯色 (R,G,B)。0~255 会再按 WS2812_BRIGHTNESS_PERCENT 缩放 */
static const uint8_t mode_led_color[MODE_COUNT][3] = {
    {0, 0, 255},   /* HOST 蓝 */
    {0, 255, 0},   /* RC   绿 */
    {255, 120, 0}, /* PUSH 橙 */
};

/* ---------------------------- 字符串拼装 ---------------------------- */

static void Line_AppendText(char *line, uint8_t *pos, const char *text)
{
    while ((*text != '\0') && (*pos < 21u))
    {
        line[*pos] = *text;
        (*pos)++;
        text++;
    }
}

static void Line_AppendU32(char *line, uint8_t *pos, uint32_t value)
{
    char digits[11];
    uint8_t count = 0;

    if (value == 0u)
    {
        digits[count] = '0';
        count++;
    }
    while ((value > 0u) && (count < 10u))
    {
        digits[count] = (char)('0' + (value % 10u));
        value = value / 10u;
        count++;
    }
    while ((count > 0u) && (*pos < 21u))
    {
        count--;
        line[*pos] = digits[count];
        (*pos)++;
    }
    line[*pos] = '\0';
}

static void Line_Reset(char *line, uint8_t *pos)
{
    *pos = 0;
    line[0] = '\0';
}

/* ---------------------------- 显示 / 灯 ---------------------------- */

/* 把当前模式刷到灯和屏幕上 (模式或统计变化时调用) */
static void Show_Mode(uint8_t oled_ok, uint8_t mode)
{
    char line[22];
    uint8_t pos;

    /* 灯 */
    Ws2812_SetAll(mode_led_color[mode][0], mode_led_color[mode][1],
                  mode_led_color[mode][2]);
    Ws2812_Show();

    if (oled_ok == 0u)
    {
        return;
    }

    /* 第 4 行: MODE xxx */
    Line_Reset(line, &pos);
    Line_AppendText(line, &pos, "MODE ");
    Line_AppendText(line, &pos, Mode_Name(mode));
    Ssd1306_ShowLine(4, line);

    /* 第 5 行: 该模式的一句话说明 */
    Ssd1306_ShowLine(5, Mode_Hint(mode));
}

/* 第 6 行: 命令统计 */
static void Show_Counters(uint8_t oled_ok)
{
    char line[22];
    uint8_t pos;

    if (oled_ok == 0u)
    {
        return;
    }

    Line_Reset(line, &pos);
    Line_AppendText(line, &pos, "RX ");
    Line_AppendU32(line, &pos, Proto_HostCmdCount());
    Line_AppendText(line, &pos, " RJ ");
    Line_AppendU32(line, &pos, Proto_HostRejectCount());
    Ssd1306_ShowLine(6, line);
}

static void Show_BoardInfo(uint8_t oled_ok)
{
    Board_UartSend("\r\n\r\n");
    Board_UartSend("======================================\r\n");
    Board_UartSend(" TB01 chassis controller V2\r\n");
    Board_UartSend(" modes: HOST / RC / PUSH\r\n");
    Board_UartSend("======================================\r\n");
    Board_UartSend("SYSCLK = ");
    Board_UartSendU32((uint32_t)(SystemCoreClock / 1000000));
    Board_UartSend(" MHz\r\n");
    Board_UartSend("OLED   = ");
    if (oled_ok != 0u)
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
    Board_UartSend("KEY    = PA15: press to switch mode\r\n");
    Board_UartSend("UART   = mode cmd: 7A 02 <mode> 00 00 00 <bcc> 7D\r\n");
    Board_UartSend("\r\n");

    if (oled_ok == 0u)
    {
        return;
    }

    Ssd1306_ShowLine(0, "TB01 CHASSIS V2");
    Ssd1306_ShowLine(1, "MCU F103C8T6");
    Ssd1306_ShowLineU32(2, "CLK ", (uint32_t)(SystemCoreClock / 1000000));
    Ssd1306_ShowLine(3, "OLED OK");
}

/* 模式切换时打一行日志 */
static void Log_ModeChange(uint8_t old_mode, uint8_t new_mode, uint8_t source)
{
    Board_UartSend("[mode] ");
    Board_UartSend(Mode_Name(old_mode));
    Board_UartSend(" -> ");
    Board_UartSend(Mode_Name(new_mode));
    Board_UartSend(" by ");
    Board_UartSend(Mode_SourceName(source));
    Board_UartSend("\r\n");
}

/* ------------------------------- 主程序 ------------------------------- */

int main(void)
{
    uint8_t oled_ok;
    uint8_t run_lo = 0;
    uint32_t run_count = 0;
    uint32_t i;
    uint8_t byte;
    uint8_t old_mode;
    uint8_t source;

    /* 按键消抖 */
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

    Mode_Init();
    Proto_Init();

    Show_BoardInfo(oled_ok);

    /* 上电默认模式: 灯色 + 屏幕 + 一行日志 */
    old_mode = Mode_Get();
    Show_Mode(oled_ok, old_mode);
    Board_UartSend("[mode] power on = ");
    Board_UartSend(Mode_Name(old_mode));
    Board_UartSend("\r\n");

    for (;;)
    {
        Delay_ms(10);

        /* ---- 收上位机串口数据 (每次最多 32 字节, 不阻塞) ---- */
        for (i = 0; i < 32u; i++)
        {
            if (Board_UartReadByte(&byte) == 0u)
            {
                break;
            }
            Proto_FeedByte(byte);
        }

        /* ---- 按键: 连续 3 次采样(30ms)一致才认边沿, 按下时切模式 ---- */
        if (Board_KeyDown() != 0u)
        {
            if (key_raw == 0u)
            {
                key_raw = 1u;
                key_stable_cnt = 0;
            }
            else if (key_stable_cnt < 3u)
            {
                key_stable_cnt++;
            }
        }
        else
        {
            if (key_raw != 0u)
            {
                key_raw = 0u;
                key_stable_cnt = 0;
            }
            else if (key_stable_cnt < 3u)
            {
                key_stable_cnt++;
            }
        }

        if ((key_stable_cnt >= 3u) && (key_raw != key_state))
        {
            key_state = key_raw;
            if (key_state != 0u)
            {
                old_mode = Mode_Get();
                Mode_Next(MODE_SRC_KEY);
            }
        }

        /* ---- 模式变了: 刷灯 + 刷屏 + 日志 (按键和上位机命令都会走这里) ---- */
        if (Mode_TakeChanged() != 0u)
        {
            source = Mode_ChangedSource();
            Show_Mode(oled_ok, Mode_Get());
            Log_ModeChange(old_mode, Mode_Get(), source);
            old_mode = Mode_Get();
        }

        /* ---- 每 100ms 刷一次计数和运行时间 ---- */
        run_lo++;
        if (run_lo >= 10u)
        {
            run_lo = 0;
            run_count++;
            Show_Counters(oled_ok);
            if (oled_ok != 0u)
            {
                Ssd1306_ShowLineU32(7, "RUN ", run_count);
            }
        }
    }
}
