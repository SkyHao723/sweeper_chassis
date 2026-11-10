#include "stm32f10x.h"
#include "board.h"
#include "ssd1306.h"

/**
  ******************************************************************************
  * @file    ssd1306.c
  * @brief   SSD1306 128x64 OLED 驱动, 软件模拟 I2C (PB8=SCL, PB9=SDA)
  *
  * 硬件说明:
  *   PB8/PB9 都配成开漏输出 (Out_OD)。开漏写 1 = 释放总线(靠上拉变高),
  *   写 0 = 主动拉低; 读 IDR 就是读实际电平。所以读 ACK 不需要切换输入模式。
  ******************************************************************************
  */

#define SSD_PORT GPIOB
#define SSD_SCL_PIN GPIO_Pin_8
#define SSD_SDA_PIN GPIO_Pin_9
#define SSD_CLK RCC_APB2Periph_GPIOB

#define SSD_I2C_ADDRESS 0x78 /* 0x3C << 1 */
#define SSD_I2C_HALF_DELAY 200 /* 约 2.8us 半周期 => SCL ~180kHz */
#define SSD_PAGE_COUNT 8
#define SSD_COLUMN_COUNT 128

/* --------------------------- 软件 I2C 底层 --------------------------- */

static void Ssd_I2cDelay(void)
{
    Board_DwtDelayCycles(SSD_I2C_HALF_DELAY);
}

static void Ssd_Scl(uint8_t high)
{
    if (high != 0)
    {
        GPIO_SetBits(SSD_PORT, SSD_SCL_PIN);
    }
    else
    {
        GPIO_ResetBits(SSD_PORT, SSD_SCL_PIN);
    }
    Ssd_I2cDelay();
}

static void Ssd_Sda(uint8_t high)
{
    if (high != 0)
    {
        GPIO_SetBits(SSD_PORT, SSD_SDA_PIN);
    }
    else
    {
        GPIO_ResetBits(SSD_PORT, SSD_SDA_PIN);
    }
    Ssd_I2cDelay();
}

static void Ssd_I2cStart(void)
{
    Ssd_Sda(1);
    Ssd_Scl(1);
    Ssd_Sda(0);
    Ssd_Scl(0);
}

static void Ssd_I2cStop(void)
{
    Ssd_Sda(0);
    Ssd_Scl(1);
    Ssd_Sda(1);
}

/* 写一个字节, 返回 0 = 收到 ACK, 1 = 没有 ACK */
static uint8_t Ssd_I2cWriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t ack;
    uint8_t mask;

    for (bit = 0; bit < 8; bit++)
    {
        mask = (uint8_t)(0x80 >> bit);
        if ((value & mask) != 0)
        {
            Ssd_Sda(1);
        }
        else
        {
            Ssd_Sda(0);
        }
        Ssd_Scl(1);
        Ssd_Scl(0);
    }

    /* 第 9 个时钟: 释放 SDA, 读从机的应答 */
    Ssd_Sda(1);
    Ssd_Scl(1);
    ack = (GPIO_ReadInputDataBit(SSD_PORT, SSD_SDA_PIN) == Bit_RESET) ? 0 : 1;
    Ssd_Scl(0);

    return ack;
}

/* --------------------------- SSD1306 命令/数据 --------------------------- */

static uint8_t Ssd_WriteCommand(uint8_t command)
{
    uint8_t result;

    Ssd_I2cStart();
    result = Ssd_I2cWriteByte(SSD_I2C_ADDRESS);
    if (result == 0)
    {
        result = Ssd_I2cWriteByte(0x00); /* 0x00 = 后面是命令 */
    }
    if (result == 0)
    {
        result = Ssd_I2cWriteByte(command);
    }
    Ssd_I2cStop();

    return result;
}

static void Ssd_SetPage(uint8_t page)
{
    (void)Ssd_WriteCommand((uint8_t)(0xB0 + page));
    (void)Ssd_WriteCommand(0x00); /* 列地址低 4 位 */
    (void)Ssd_WriteCommand(0x10); /* 列地址高 4 位 */
}

/* ------------------------------ 字模 ------------------------------ */

static const uint8_t *Ssd_Glyph(char character)
{
    static const uint8_t space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t question[5] = {0x02, 0x01, 0x51, 0x09, 0x06};
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}};
    static const uint8_t letters[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
        {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
        {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
        {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
        {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
        {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
        {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
        {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x03, 0x04, 0x78, 0x04, 0x03}, {0x61, 0x51, 0x49, 0x45, 0x43}};

    /* 小写统一按大写显示 (字库只有大写) */
    if ((character >= 'a') && (character <= 'z'))
    {
        character = (char)(character - 'a' + 'A');
    }

    if ((character >= '0') && (character <= '9'))
    {
        return digits[character - '0'];
    }
    if ((character >= 'A') && (character <= 'Z'))
    {
        return letters[character - 'A'];
    }
    if (character == ':')
    {
        return colon;
    }
    if (character == '-')
    {
        return dash;
    }
    if (character == '/')
    {
        return slash;
    }
    if (character == '.')
    {
        return dot;
    }
    if (character == ' ')
    {
        return space;
    }

    return question;
}

/* ------------------------------ 对外接口 ------------------------------ */

uint8_t Ssd1306_Probe(void)
{
    uint8_t result;

    Ssd_I2cStart();
    result = Ssd_I2cWriteByte(SSD_I2C_ADDRESS);
    Ssd_I2cStop();

    if (result == 0)
    {
        return 1;
    }
    return 0;
}

uint8_t Ssd1306_Init(void)
{
    GPIO_InitTypeDef gpio;
    static const uint8_t commands[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF};
    uint8_t index;

    RCC_APB2PeriphClockCmd(SSD_CLK, ENABLE);

    /* 先当普通推挽输出把两根线都拉高(给总线一个空闲状态), 再切开漏 */
    GPIO_SetBits(SSD_PORT, SSD_SCL_PIN | SSD_SDA_PIN);
    gpio.GPIO_Pin = SSD_SCL_PIN | SSD_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SSD_PORT, &gpio);
    Board_DwtDelayCycles(72000 * 2); /* 等 2ms 让 OLED 上电稳定 */

    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SSD_PORT, &gpio);
    Ssd_Scl(1);
    Ssd_Sda(1);

    if (Ssd1306_Probe() == 0)
    {
        return 0; /* 没应答: 不继续发初始化命令, 交给上层显示报错 */
    }

    for (index = 0; index < (uint8_t)sizeof(commands); index++)
    {
        if (Ssd_WriteCommand(commands[index]) != 0)
        {
            return 0;
        }
    }

    Ssd1306_Clear();

    return 1;
}

void Ssd1306_Clear(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0; page < SSD_PAGE_COUNT; page++)
    {
        Ssd_SetPage(page);
        Ssd_I2cStart();
        (void)Ssd_I2cWriteByte(SSD_I2C_ADDRESS);
        (void)Ssd_I2cWriteByte(0x40); /* 0x40 = 后面是显存数据 */
        for (column = 0; column < SSD_COLUMN_COUNT; column++)
        {
            (void)Ssd_I2cWriteByte(0x00);
        }
        Ssd_I2cStop();
    }
}

void Ssd1306_ShowLine(uint8_t page, const char *text)
{
    uint8_t column;
    uint8_t glyph_column;
    uint8_t character_index;
    uint8_t value;

    if (page >= SSD_PAGE_COUNT)
    {
        return;
    }

    Ssd_SetPage(page);
    Ssd_I2cStart();
    (void)Ssd_I2cWriteByte(SSD_I2C_ADDRESS);
    (void)Ssd_I2cWriteByte(0x40);

    for (column = 0; column < SSD_COLUMN_COUNT; column++)
    {
        glyph_column = (uint8_t)(column % 6);
        character_index = (uint8_t)(column / 6);
        value = 0;

        if ((text[character_index] != '\0') && (glyph_column < 5))
        {
            value = Ssd_Glyph(text[character_index])[glyph_column];
        }
        (void)Ssd_I2cWriteByte(value);
    }
    Ssd_I2cStop();
}

void Ssd1306_ShowLineU32(uint8_t page, const char *prefix, uint32_t value)
{
    char line[22];
    uint8_t pos = 0;
    char digits[11];
    uint8_t count = 0;

    while ((prefix[pos] != '\0') && (pos < 21))
    {
        line[pos] = prefix[pos];
        pos++;
    }

    if (value == 0)
    {
        digits[count] = '0';
        count++;
    }
    while ((value > 0) && (count < 10))
    {
        digits[count] = (char)('0' + (value % 10));
        value = value / 10;
        count++;
    }

    while ((count > 0) && (pos < 21))
    {
        count--;
        line[pos] = digits[count];
        pos++;
    }

    line[pos] = '\0';
    Ssd1306_ShowLine(page, line);
}
