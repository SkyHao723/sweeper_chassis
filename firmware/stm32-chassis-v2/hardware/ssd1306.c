#include "stm32f10x.h"
#include "board.h"
#include "softi2c.h"
#include "ssd1306.h"

/**
  ******************************************************************************
  * @file    ssd1306.c
  * @brief   SSD1306 128x64 OLED 驱动, 走 PB8/PB9 的软件 I2C (见 softi2c.c)
  *
  * 这条总线上还挂着 AHT20 温湿度 (见 aht20.c), 两边共用 softi2c 的底层。
  ******************************************************************************
  */

#define SSD_I2C_ADDRESS 0x78 /* 0x3C << 1 */
#define SSD_PAGE_COUNT 8
#define SSD_COLUMN_COUNT 128

/* --------------------------- SSD1306 命令/数据 --------------------------- */

static uint8_t Ssd_WriteCommand(uint8_t command)
{
    uint8_t result;

    SoftI2c_Start();
    result = SoftI2c_WriteByte(SSD_I2C_ADDRESS);
    if (result == 0)
    {
        result = SoftI2c_WriteByte(0x00); /* 0x00 = 后面是命令 */
    }
    if (result == 0)
    {
        result = SoftI2c_WriteByte(command);
    }
    SoftI2c_Stop();

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

    SoftI2c_Start();
    result = SoftI2c_WriteByte(SSD_I2C_ADDRESS);
    SoftI2c_Stop();

    if (result == 0)
    {
        return 1;
    }
    return 0;
}

uint8_t Ssd1306_Init(void)
{
    static const uint8_t commands[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF};
    uint8_t index;

    /* 总线初始化(PB8/PB9 开漏 + 上拉空闲), 总线可能被卡住就先复位一下 */
    SoftI2c_Init();
    SoftI2c_BusReset();

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
        SoftI2c_Start();
        (void)SoftI2c_WriteByte(SSD_I2C_ADDRESS);
        (void)SoftI2c_WriteByte(0x40); /* 0x40 = 后面是显存数据 */
        for (column = 0; column < SSD_COLUMN_COUNT; column++)
        {
            (void)SoftI2c_WriteByte(0x00);
        }
        SoftI2c_Stop();
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
    SoftI2c_Start();
    (void)SoftI2c_WriteByte(SSD_I2C_ADDRESS);
    (void)SoftI2c_WriteByte(0x40);

    for (column = 0; column < SSD_COLUMN_COUNT; column++)
    {
        glyph_column = (uint8_t)(column % 6);
        character_index = (uint8_t)(column / 6);
        value = 0;

        if ((text[character_index] != '\0') && (glyph_column < 5))
        {
            value = Ssd_Glyph(text[character_index])[glyph_column];
        }
        (void)SoftI2c_WriteByte(value);
    }
    SoftI2c_Stop();
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
