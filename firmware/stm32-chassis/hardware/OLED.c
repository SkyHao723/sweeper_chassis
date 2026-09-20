#include "OLED.h"
#include "Delay.h"

/* Vendor I2C example uses 0x78 (7-bit I2C address 0x3C). */
#define OLED_I2C_ADDRESS  0x78
#define OLED_SCL_PIN      GPIO_Pin_6
#define OLED_SDA_PIN      GPIO_Pin_7

#define OLED_SCL_HIGH()   GPIO_SetBits(GPIOB, OLED_SCL_PIN)
#define OLED_SCL_LOW()    GPIO_ResetBits(GPIOB, OLED_SCL_PIN)
#define OLED_SDA_HIGH()   GPIO_SetBits(GPIOB, OLED_SDA_PIN)
#define OLED_SDA_LOW()    GPIO_ResetBits(GPIOB, OLED_SDA_PIN)

/* Keep the software I2C clock safely below the OLED's 400 kHz limit. */
static void OLED_I2C_Delay(void)
{
    volatile uint8_t ticks;

    for (ticks = 0; ticks < 15; ticks++)
    {
        __NOP();
    }
}

static void OLED_I2C_Start(void)
{
    OLED_SDA_HIGH();
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_LOW();
    OLED_I2C_Delay();
    OLED_SCL_LOW();
    OLED_I2C_Delay();
}

static void OLED_I2C_Stop(void)
{
    OLED_SDA_LOW();
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    OLED_SDA_HIGH();
    OLED_I2C_Delay();
}

static uint8_t OLED_I2C_WriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0; bit < 8; bit++)
    {
        if (value & 0x80)
        {
            OLED_SDA_HIGH();
        }
        else
        {
            OLED_SDA_LOW();
        }
        OLED_I2C_Delay();
        OLED_SCL_HIGH();
        OLED_I2C_Delay();
        OLED_SCL_LOW();
        OLED_I2C_Delay();
        value <<= 1;
    }

    OLED_SDA_HIGH();
    OLED_I2C_Delay();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();
    acknowledged = (GPIO_ReadInputDataBit(GPIOB, OLED_SDA_PIN) == Bit_RESET);
    OLED_SCL_LOW();
    OLED_I2C_Delay();
    return acknowledged;
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_I2C_Start();
    (void)OLED_I2C_WriteByte(OLED_I2C_ADDRESS);
    (void)OLED_I2C_WriteByte(0x00);
    (void)OLED_I2C_WriteByte(command);
    OLED_I2C_Stop();
}

static void OLED_SetPosition(uint8_t page)
{
    OLED_WriteCommand((uint8_t)(0xB0 + page));
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
}

static const uint8_t *OLED_Glyph(char character)
{
    static const uint8_t space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t question[5] = {0x02, 0x01, 0x51, 0x09, 0x06};
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
    };
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
        {0x03, 0x04, 0x78, 0x04, 0x03}, {0x61, 0x51, 0x49, 0x45, 0x43}
    };

    if (character >= '0' && character <= '9')
    {
        return digits[character - '0'];
    }
    if (character >= 'A' && character <= 'Z')
    {
        return letters[character - 'A'];
    }
    if (character == ':') return colon;
    if (character == '-') return dash;
    if (character == ' ') return space;
    return question;
}

void OLED_ShowLine(uint8_t page, const char *text)
{
    uint8_t column;

    if (page > 7)
    {
        return;
    }

    OLED_SetPosition(page);
    OLED_I2C_Start();
    (void)OLED_I2C_WriteByte(OLED_I2C_ADDRESS);
    (void)OLED_I2C_WriteByte(0x40);
    for (column = 0; column < 128; column++)
    {
        uint8_t glyph_column = column % 6;
        uint8_t character_index = column / 6;
        uint8_t value = 0;

        if (text[character_index] != '\0' && glyph_column < 5)
        {
            value = OLED_Glyph(text[character_index])[glyph_column];
        }
        (void)OLED_I2C_WriteByte(value);
    }
    OLED_I2C_Stop();
}

void OLED_Init(void)
{
    GPIO_InitTypeDef gpio;
    static const uint8_t commands[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF
    };
    uint8_t index;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    OLED_SCL_HIGH();
    OLED_SDA_HIGH();
    Delay_ms(50);

    for (index = 0; index < sizeof(commands); index++)
    {
        OLED_WriteCommand(commands[index]);
    }

    for (index = 0; index < 8; index++)
    {
        OLED_ShowLine(index, "");
    }
}
