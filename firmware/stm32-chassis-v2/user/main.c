#include "stm32f10x.h"
#include "board.h"
#include "Delay.h"
#include "ws2812.h"
#include "ssd1306.h"
#include "mode.h"
#include "proto.h"
#include "icm42670.h"
#include "aht20.h"
#include "telemetry.h"
#include "rc.h"

/**
  ******************************************************************************
  * @file    main.c
  * @brief   底盘主控 V2 (STM32F103C8T6) —— 点灯 + 模式 + IMU + 温湿度 + 遥控输入 + 上行遥测
  *
  * 屏幕 8 行:
  *   0  TB01 CHASSIS V2
  *   1  CLK 72 MODE HOST    ← CLK 不是 72 说明 8MHz 晶振没起振
  *   2  IMU 0x67 AHT OK     ← IMU 探测到的地址(0=没找到) / AHT20 状态
  *   3  RC OK X310 Y-120 h5 ← 手柄在线? 左摇杆千分制 + 十字键
  *   4  a 0.01 -0.02 1.00   ← 加速度, 单位 g
  *   5  g 0.1 -0.2 0.3      ← 角速度, 单位 °/s
  *   6  T 25.3C H 45.2%     ← AHT20 温度(°C) / 湿度(%RH)
  *   7  RX 12 RJ 0 RUN 5    ← 上位机速度帧数 / 被丢帧数 / 运行秒数
  *
  * 三种模式 (见 mode.h):
  *   HOST 上位机控制  灯=蓝   接受上位机速度命令
  *   RC   遥控        灯=绿   用左摇杆开车(指令进遥测的 vx/wz), 忽略上位机速度命令
  *   PUSH 推行        灯=橙   电机释放, 忽略上位机速度命令
  *
  * 切换方式:
  *   1) 按板载按键 PA15: 每按一下切下一个 (HOST -> RC -> PUSH -> HOST)
  *   2) 上位机发功能帧: 7A 02 <mode> 00 00 00 <bcc> 7D
  *      上位机控制 7A 02 00 00 00 00 78 7D
  *      遥控       7A 02 01 00 00 00 79 7D
  *      推行       7A 02 02 00 00 00 7A 7D
  *
  * 手柄链路 (见 rc.h):
  *   接收器(USB, 15B HID @500Hz) -> CH554G -> USART3(PB10/PB11) 115200 -> 本固件解析
  *   没接 CH554G 时, 可以用 tools/v2_gamepad.py 从调试串口发 0x78 注入帧验证同一套逻辑。
  *
  * 上行 (20Hz 两种帧, 都走 USART1):
  *   0x7E 32 字节遥测: 模式/速度指令/加速度/角速度/IMU 温度/湿度/AHT20 温度/统计 (telemetry.h)
  *   0x79 19 字节遥控: 手柄 15 字节原始数据 + 在线标志 (rc.h)
  *
  * 传感器:
  *   ICM-42670-P  硬件 I2C1 (PB6=SCL, PB7=SDA)  加速度 ±4g / 陀螺 ±1000dps @200Hz
  *   AHT20        软件 I2C  (PB8=SCL, PB9=SDA)  与 OLED 共总线, 500ms 采一次
  ******************************************************************************
  */

/* 每种模式的灯色 (R,G,B)。0~255 会再按 WS2812_BRIGHTNESS_PERCENT 缩放 */
static const uint8_t mode_led_color[MODE_COUNT][3] = {
    {0, 0, 255},   /* HOST 蓝 */
    {0, 255, 0},   /* RC   绿 */
    {255, 120, 0}, /* PUSH 橙 */
};

/* 主循环节拍 (ms)。所有周期都用它换算 */
#define TICK_MS 10u

/* 传感器 / 遥测周期 (以 tick 为单位) */
#define IMU_PERIOD_TICKS 5u    /* 50ms -> 20Hz */
#define TELEMETRY_TICKS 5u     /* 50ms -> 20Hz */
#define AHT_TRIGGER_TICKS 50u  /* 500ms 采一次温湿度 */
#define SCREEN_TICKS 20u       /* 200ms 刷一次传感器数值 */

/* ---------------------------- 字符串拼装 ---------------------------- */

static void Line_AppendText(char *line, uint8_t *pos, const char *text)
{
    while ((*text != '\0') && (*pos < 21u))
    {
        line[*pos] = *text;
        (*pos)++;
        text++;
    }
    line[*pos] = '\0';
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

/* 有符号定点数: value=1234, decimals=2 -> "12.34"; value=-5, decimals=2 -> "-0.05" */
static void Line_AppendFixed(char *line, uint8_t *pos, int32_t value, uint8_t decimals)
{
    uint32_t magnitude;
    uint32_t scale = 1u;
    uint32_t int_part;
    uint32_t frac_part;
    uint8_t index;

    if (value < 0)
    {
        if (*pos < 21u)
        {
            line[*pos] = '-';
            (*pos)++;
        }
        magnitude = (uint32_t)(-value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    for (index = 0; index < decimals; index++)
    {
        scale *= 10u;
    }

    int_part = magnitude / scale;
    frac_part = magnitude % scale;

    Line_AppendU32(line, pos, int_part);

    if (decimals > 0u)
    {
        if (*pos < 21u)
        {
            line[*pos] = '.';
            (*pos)++;
        }
        while ((decimals > 0u) && (*pos < 21u))
        {
            scale /= 10u;
            line[*pos] = (char)('0' + ((frac_part / scale) % 10u));
            (*pos)++;
            decimals--;
        }
    }
    line[*pos] = '\0';
}

static void Line_AppendHex8(char *line, uint8_t *pos, uint8_t value)
{
    static const char hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                       '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    if (*pos < 20u)
    {
        line[*pos] = '0';
        (*pos)++;
        line[*pos] = 'x';
        (*pos)++;
        line[*pos] = hex_chars[(value >> 4) & 0x0Fu];
        (*pos)++;
        line[*pos] = hex_chars[value & 0x0Fu];
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

    /* 第 1 行: CLK 72 MODE HOST */
    Line_Reset(line, &pos);
    Line_AppendText(line, &pos, "CLK ");
    Line_AppendU32(line, &pos, (uint32_t)(SystemCoreClock / 1000000));
    Line_AppendText(line, &pos, " MODE ");
    Line_AppendText(line, &pos, Mode_Name(mode));
    Ssd1306_ShowLine(1, line);
}

/* 第 7 行: 命令统计 + 运行秒数 */
static void Show_Counters(uint8_t oled_ok, uint32_t run_seconds)
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
    Line_AppendText(line, &pos, " RUN ");
    Line_AppendU32(line, &pos, run_seconds);
    Ssd1306_ShowLine(7, line);
}

/* 第 3 行: 手柄状态 + 左摇杆千分制 + 十字键 + 按钮 */
static void Show_Rc(uint8_t oled_ok, uint8_t online, int16_t x_permille,
                    int16_t y_permille, uint8_t hat, uint8_t buttons)
{
    char line[22];
    uint8_t pos;

    if (oled_ok == 0u)
    {
        return;
    }

    Line_Reset(line, &pos);
    if (online == 0u)
    {
        Line_AppendText(line, &pos, "RC -- (no pad)");
        Ssd1306_ShowLine(3, line);
        return;
    }

    Line_AppendText(line, &pos, "RC X");
    Line_AppendFixed(line, &pos, (int32_t)x_permille, 0u);
    Line_AppendText(line, &pos, " Y");
    Line_AppendFixed(line, &pos, (int32_t)y_permille, 0u);
    Line_AppendText(line, &pos, " h");
    Line_AppendU32(line, &pos, hat);
    if (buttons != 0u)
    {
        Line_AppendText(line, &pos, " b");
        Line_AppendHex8(line, &pos, buttons);
    }
    Ssd1306_ShowLine(3, line);
}

/* 第 2/3/4/5 行: 传感器状态与数值 */
static void Show_Sensors(uint8_t oled_ok, uint8_t imu_ok, uint8_t aht_ok,
                         const Icm42670_Sample *imu, int16_t aht_temp_c100,
                         uint16_t rh_100)
{
    char line[22];
    uint8_t pos;

    if (oled_ok == 0u)
    {
        return;
    }

    /* 第 2 行: IMU 0x68 AHT OK */
    Line_Reset(line, &pos);
    Line_AppendText(line, &pos, "IMU ");
    Line_AppendHex8(line, &pos, Icm42670_Address());
    Line_AppendText(line, &pos, " AHT ");
    Line_AppendText(line, &pos, (aht_ok != 0u) ? "OK" : "ERR");
    Ssd1306_ShowLine(2, line);

    if (imu_ok == 0u)
    {
        Ssd1306_ShowLine(4, "a -- no imu --");
        Ssd1306_ShowLine(5, "g -- no imu --");
    }
    else
    {
        /* 第 4 行: 加速度, 单位 g, 两位小数 (mg -> 0.01g 是 /10) */
        Line_Reset(line, &pos);
        Line_AppendText(line, &pos, "a ");
        Line_AppendFixed(line, &pos, (int32_t)(imu->acc_mg[0] / 10), 2u);
        Line_AppendText(line, &pos, " ");
        Line_AppendFixed(line, &pos, (int32_t)(imu->acc_mg[1] / 10), 2u);
        Line_AppendText(line, &pos, " ");
        Line_AppendFixed(line, &pos, (int32_t)(imu->acc_mg[2] / 10), 2u);
        Ssd1306_ShowLine(4, line);

        /* 第 5 行: 角速度, 单位 °/s, 一位小数 (0.1dps -> /1) */
        Line_Reset(line, &pos);
        Line_AppendText(line, &pos, "g ");
        Line_AppendFixed(line, &pos, (int32_t)imu->gyro_0d1dps[0], 1u);
        Line_AppendText(line, &pos, " ");
        Line_AppendFixed(line, &pos, (int32_t)imu->gyro_0d1dps[1], 1u);
        Line_AppendText(line, &pos, " ");
        Line_AppendFixed(line, &pos, (int32_t)imu->gyro_0d1dps[2], 1u);
        Ssd1306_ShowLine(5, line);
    }

    /* 第 6 行: T 25.3C H 45.2% */
    Line_Reset(line, &pos);
    if (aht_ok != 0u)
    {
        Line_AppendText(line, &pos, "T ");
        Line_AppendFixed(line, &pos, (int32_t)(aht_temp_c100 / 10), 1u);
        Line_AppendText(line, &pos, "C H ");
        Line_AppendFixed(line, &pos, (int32_t)(rh_100 / 10), 1u);
        Line_AppendText(line, &pos, "%");
    }
    else
    {
        Line_AppendText(line, &pos, "T -- H -- (no aht20)");
    }
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
    Board_UartSend("UPLINK = telemetry 32B @20Hz + rc 19B @20Hz\r\n");
    Board_UartSend("IMU    = ICM-42670-P on hardware I2C1 (PB6/PB7)\r\n");
    Board_UartSend("AHT20  = on soft I2C (PB8/PB9), 500ms sample\r\n");
    Board_UartSend("RC     = USART3 PB11 RX, 115200, 15B HID @500Hz\r\n");
    Board_UartSend("\r\n");

    if (oled_ok == 0u)
    {
        return;
    }

    Ssd1306_ShowLine(0, "TB01 CHASSIS V2");
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
    uint8_t imu_ok;
    uint8_t aht_ok;
    uint8_t imu_last_ok;
    uint8_t aht_last_ok;
    uint8_t byte;
    uint8_t old_mode;

    uint32_t tick = 0;
    uint32_t run_count = 0;

    /* 按键消抖 */
    uint8_t key_raw = 0;
    uint8_t key_state = 0;
    uint8_t key_stable_cnt = 0;

    /* 传感器数据 */
    Icm42670_Sample imu;
    Icm42670_Sample imu_last;
    int16_t aht_temp_c100 = 0;
    uint16_t rh_100 = 0;
    Telemetry_Data telemetry;
    uint16_t rx_count;
    uint16_t rj_count;
    int16_t rc_x_permille = 0;
    int16_t rc_y_permille = 0;

    Board_DwtInit();
    Board_SwdOnly();       /* 先关 JTAG, PA15 才能当普通 IO */
    Board_KeyInit();
    Board_ParkUnusedPins();
    Board_UartInit(115200);
    Rc_Init();             /* USART3: 遥控手柄数据 (CH554G 转发) */

    Ws2812_Init();
    /* 屏幕初始化内部会做 SoftI2c_Init + BusReset, AHT20 与它共总线, 必须之后初始化 */
    oled_ok = Ssd1306_Init();
    aht_ok = Aht20_Init();
    imu_ok = Icm42670_Init();

    imu_last_ok = 0;
    aht_last_ok = aht_ok;
    for (byte = 0; byte < 3u; byte++)
    {
        imu.acc_mg[byte] = 0;
        imu.gyro_0d1dps[byte] = 0;
        imu_last.acc_mg[byte] = 0;
        imu_last.gyro_0d1dps[byte] = 0;
    }
    imu.temp_c100 = 0;
    imu_last.temp_c100 = 0;

    Mode_Init();
    Proto_Init();

    Show_BoardInfo(oled_ok);

    /* 传感器自检结果打到串口 */
    Board_UartSend("[imu] addr=");
    Board_UartSendHex8(Icm42670_Address());
    Board_UartSend(" whoami=");
    Board_UartSendHex8(Icm42670_WhoAmI());
    Board_UartSend((imu_ok != 0u) ? " OK\r\n" : " FAIL\r\n");
    Board_UartSend("[aht20] ");
    Board_UartSend((aht_ok != 0u) ? "OK\r\n" : "FAIL\r\n");

    /* 上电默认模式: 灯色 + 屏幕 + 一行日志 */
    old_mode = Mode_Get();
    Show_Mode(oled_ok, old_mode);
    Show_Rc(oled_ok, 0u, 0, 0, 0u, 0u);
    Show_Sensors(oled_ok, imu_last_ok, aht_last_ok, &imu_last, aht_temp_c100, rh_100);
    Show_Counters(oled_ok, 0u);
    Board_UartSend("[mode] power on = ");
    Board_UartSend(Mode_Name(old_mode));
    Board_UartSend("\r\n");

    for (;;)
    {
        Delay_ms(TICK_MS);
        tick++;

        /* ---- 收上位机串口数据 (每次最多 128 字节, 不阻塞)
               字节由 USART1 中断搬进 256B 环形缓冲, 这里排空;
               10ms 一轮最多可以排 128 字节 = 12.8 kB/s, 比 115200 的 11.5 kB/s 略大,
               所以连续高速发也不会溢出（真溢出了看 Board_UartRxOverrun()）。 ---- */
        for (byte = 0; byte < 128u; byte++)
        {
            uint8_t value;
            if (Board_UartReadByte(&value) == 0u)
            {
                break;
            }
            Proto_FeedByte(value);
        }

        /* ---- 遥控手柄: 解析 USART3 收到的字节 (CH554G 转发的 15B 原始帧) ---- */
        Rc_Poll();

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
            Show_Mode(oled_ok, Mode_Get());
            Log_ModeChange(old_mode, Mode_Get(), Mode_ChangedSource());
            old_mode = Mode_Get();
        }

        /* ---- IMU: 20Hz 读一帧 ---- */
        if ((tick % IMU_PERIOD_TICKS) == 0u)
        {
            if (Icm42670_Read(&imu) != 0u)
            {
                for (byte = 0; byte < 3u; byte++)
                {
                    imu_last.acc_mg[byte] = imu.acc_mg[byte];
                    imu_last.gyro_0d1dps[byte] = imu.gyro_0d1dps[byte];
                }
                imu_last.temp_c100 = imu.temp_c100;
                imu_last_ok = 1u;
            }
            else
            {
                imu_last_ok = 0u;
            }
        }

        /* ---- AHT20: 每 500ms 触发一次, 每拍都问一次结果(不阻塞) ---- */
        if ((tick % AHT_TRIGGER_TICKS) == 0u)
        {
            (void)Aht20_Trigger();
        }
        if (Aht20_Poll(&aht_temp_c100, &rh_100) != 0u)
        {
            aht_last_ok = 1u;
        }
        else
        {
            aht_last_ok = Aht20_IsOk();
        }

        /* ---- 上行遥测: 20Hz ---- */
        if ((tick % TELEMETRY_TICKS) == 0u)
        {
            rx_count = Proto_HostCmdCount();
            rj_count = Proto_HostRejectCount();

            telemetry.mode = Mode_Get();
            telemetry.imu_ok = imu_last_ok;
            telemetry.aht_ok = Aht20_HasSample();

            /* 速度指令: HOST = 上位机命令, RC = 左摇杆映射, PUSH = 0 */
            if (Mode_Get() == MODE_RC)
            {
                Rc_GetCmd(&telemetry.vx_mm_s, &telemetry.wz_mrad_s);
            }
            else if (Mode_Get() == MODE_HOST)
            {
                Proto_GetHostCmd(&telemetry.vx_mm_s, &telemetry.wz_mrad_s);
            }
            else
            {
                telemetry.vx_mm_s = 0;
                telemetry.wz_mrad_s = 0;
            }

            for (byte = 0; byte < 3u; byte++)
            {
                telemetry.acc_mg[byte] = imu_last.acc_mg[byte];
                telemetry.gyro_0d1dps[byte] = imu_last.gyro_0d1dps[byte];
            }
            telemetry.imu_temp_c100 = imu_last.temp_c100;
            telemetry.rh_100 = rh_100;
            telemetry.aht_temp_c100 = aht_temp_c100;
            telemetry.tick_ms = (uint16_t)((tick * TICK_MS) & 0xFFFFu);
            telemetry.rx_count = (uint8_t)((rx_count > 255u) ? 255u : rx_count);
            telemetry.rj_count = (uint8_t)((rj_count > 255u) ? 255u : rj_count);

            (void)Telemetry_Send(&telemetry);
            (void)Rc_SendFrame(); /* 手柄 15B 原始数据也 20Hz 带给上位机 */
        }

        /* ---- 屏幕: 200ms 刷一次传感器数值 ---- */
        if ((tick % SCREEN_TICKS) == 0u)
        {
            Rc_GetStickPermille(&rc_x_permille, &rc_y_permille);
            Show_Rc(oled_ok, Rc_Online(), rc_x_permille, rc_y_permille,
                    Rc_Get()->hat, Rc_Get()->buttons);
            Show_Sensors(oled_ok, imu_last_ok, aht_last_ok, &imu_last, aht_temp_c100, rh_100);
        }

        /* ---- 1s 刷一次计数和运行时间 ---- */
        if ((tick % 100u) == 0u)
        {
            run_count++;
            Show_Counters(oled_ok, run_count);
        }
    }
}
