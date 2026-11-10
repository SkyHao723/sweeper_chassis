#include "stm32f10x.h"
#include "board.h"

/**
  ******************************************************************************
  * @file    board.c
  * @brief   底盘主控 V2 板级初始化与工具函数
  *
  * 时钟: 上电后 startup_stm32f10x_md.s -> SystemInit() (system_stm32f10x.c)
  *       已经把 HSE 8MHz 经 PLL x9 配成 72MHz (SYSCLK_FREQ_72MHz)。
  *       这里不再重复配时钟, 只在屏幕上把 SystemCoreClock 显示出来自证。
  ******************************************************************************
  */

/* ----------------------------- DWT 延时 ----------------------------- */

void Board_DwtInit(void)
{
    /* TRCENA 打开 DWT 的电源/时钟, CYCCNT 才开始计数 (不需要调试器在场) */
    BOARD_DEMCR |= BOARD_DEMCR_TRCENA;
    BOARD_DWT_CYCCNT = 0;
    BOARD_DWT_CTRL |= BOARD_DWT_CTRL_CYCCNTENA;
}

/* 忙等指定 CPU 周期数 (72MHz 下 1 周期 = 13.89ns) */
void Board_DwtDelayCycles(uint32_t cycles)
{
    uint32_t start = BOARD_DWT_CYCCNT;

    while ((BOARD_DWT_CYCCNT - start) < cycles)
    {
        ;
    }
}

/* --------------------------- 调试口/引脚 --------------------------- */

void Board_SwdOnly(void)
{
    /* PA15/JTDI、PB3/JTDO、PB4/JNTRST 默认被 JTAG 占着, 关掉 JTAG 只留 SWD
       之后这三个脚才能当普通 IO 用 (SWD 用的 PA13/PA14 不受影响) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

void Board_ParkUnusedPins(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                               RCC_APB2Periph_GPIOC,
                           ENABLE);

    /* 模拟输入 = 高阻, 不驱动也不上拉, 避免悬空脚漏电和噪声 */
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    /* PA0(WKUP)/PA1/PA2/PA4 排针 CN5, PA5/PA6/PA7/PA8 未用
       注意: 不碰 PA9/PA10(串口) PA11/PA12(CAN) PA13/PA14(SWD) PA15(按键) */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 |
                    GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio);

    /* PB0/PB1/PB2 排针 CN6, PB3/PB4 与 PB12~PB15 未用
       注意: 不碰 PB5(RGB) PB6/PB7(IMU) PB8/PB9(显示) PB10/PB11(CH554) */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_12 | GPIO_Pin_13 |
                    GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);

    /* PC13/PC14/PC15 未用 (PC14/PC15 还接着 32k 晶振的脚) */
    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &gpio);
}

/* ------------------------------ 串口 ------------------------------ */

void Board_UartInit(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    RCC_APB2PeriphClockCmd(BOARD_UART_GPIO_CLK | BOARD_UART_CLK, ENABLE);

    /* PA9 = USART1_TX, 复用推挽输出 */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX, 上拉输入 (接收脚空闲时保持高电平) */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate = baudrate;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BOARD_UART, &uart);

    USART_Cmd(BOARD_UART, ENABLE);
}

void Board_UartPutChar(char ch)
{
    while (USART_GetFlagStatus(BOARD_UART, USART_FLAG_TXE) == RESET)
    {
        ;
    }
    USART_SendData(BOARD_UART, (uint16_t)((uint8_t)ch));
}

void Board_UartSend(const char *text)
{
    while (*text != '\0')
    {
        Board_UartPutChar(*text);
        text++;
    }
}

void Board_UartSendU32(uint32_t value)
{
    char digits[11];
    uint8_t index = 0;

    if (value == 0)
    {
        Board_UartPutChar('0');
        return;
    }

    while ((value > 0) && (index < 10))
    {
        digits[index] = (char)('0' + (value % 10));
        value = value / 10;
        index++;
    }

    while (index > 0)
    {
        index--;
        Board_UartPutChar(digits[index]);
    }
}

void Board_UartSendHex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    Board_UartPutChar(hex[(value >> 4) & 0x0F]);
    Board_UartPutChar(hex[value & 0x0F]);
}

/* ------------------------------ 按键 ------------------------------ */

void Board_KeyInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(BOARD_KEY_CLK, ENABLE);

    /* PA15 上拉输入, 按键按下把脚拉到 GND (必须先调 Board_SwdOnly 关掉 JTAG) */
    gpio.GPIO_Pin = BOARD_KEY_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(BOARD_KEY_PORT, &gpio);
}

uint8_t Board_KeyDown(void)
{
    if (GPIO_ReadInputDataBit(BOARD_KEY_PORT, BOARD_KEY_PIN) == Bit_RESET)
    {
        return 1;
    }
    return 0;
}
