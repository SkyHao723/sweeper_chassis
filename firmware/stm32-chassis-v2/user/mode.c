#include "mode.h"

/**
  ******************************************************************************
  * @file    mode.c
  * @brief   底盘工作模式的状态机 (只有状态, 不碰屏幕/灯, 由 main.c 负责显示)
  ******************************************************************************
  */

static uint8_t g_mode = MODE_POWER_ON;
static uint8_t g_changed = 0;
static uint8_t g_changed_src = MODE_SRC_BOOT;

void Mode_Init(void)
{
    g_mode = MODE_POWER_ON;
    g_changed = 0;
    g_changed_src = MODE_SRC_BOOT;
}

uint8_t Mode_Get(void)
{
    return g_mode;
}

const char *Mode_Name(uint8_t mode)
{
    switch (mode)
    {
        case MODE_HOST: return "HOST";
        case MODE_RC: return "RC";
        case MODE_PUSH: return "PUSH";
        default: return "?";
    }
}

const char *Mode_Hint(uint8_t mode)
{
    switch (mode)
    {
        case MODE_HOST: return "UART CMD OK";
        case MODE_RC: return "REMOTE ONLY";
        case MODE_PUSH: return "MOTOR FREE";
        default: return "?";
    }
}

const char *Mode_SourceName(uint8_t source)
{
    switch (source)
    {
        case MODE_SRC_BOOT: return "boot";
        case MODE_SRC_KEY: return "key";
        case MODE_SRC_UART: return "uart";
        default: return "?";
    }
}

uint8_t Mode_Set(uint8_t mode, uint8_t source)
{
    if (mode >= MODE_COUNT)
    {
        return 0;
    }
    if (mode == g_mode)
    {
        return 0;
    }

    g_mode = mode;
    g_changed = 1;
    g_changed_src = source;
    return 1;
}

uint8_t Mode_Next(uint8_t source)
{
    uint8_t next = (uint8_t)((g_mode + 1u) % MODE_COUNT);
    return Mode_Set(next, source);
}

uint8_t Mode_TakeChanged(void)
{
    uint8_t flag = g_changed;
    g_changed = 0;
    return flag;
}

uint8_t Mode_ChangedSource(void)
{
    return g_changed_src;
}
