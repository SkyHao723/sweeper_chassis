#ifndef __MODE_H
#define __MODE_H

/**
  ******************************************************************************
  * @file    mode.h
  * @brief   底盘工作模式 (上位机控制 / 遥控 / 推行)
  *
  * 三种模式:
  *   MODE_HOST (0)  正常的上位机控制模式 —— 接受上位机速度命令, 灯=蓝
  *   MODE_RC   (1)  遥控模式           —— 不接受上位机速度命令(等遥控输入), 灯=绿
  *   MODE_PUSH (2)  推行模式           —— 电机释放, 人工推车, 不接受任何运动命令, 灯=橙
  *
  * 切换方式(两条路, 互不冲突):
  *   1) 板载按键 PA15: 每按一下切到下一个模式 (HOST -> RC -> PUSH -> HOST)
  *   2) 上位机命令: 功能帧 0x7A, func=0x02, p0=模式号 (见 proto.h)
  *
  * 屏幕第 4 行显示 "MODE <名字>", 第 5 行显示该模式的一句话说明;
  * 每次切换都会在串口打一行 "[mode] HOST -> RC by key"。
  ******************************************************************************
  */

#include "stm32f10x.h"

/* 模式编号 (上位机命令里用的就是这个数) */
#define MODE_HOST 0u
#define MODE_RC 1u
#define MODE_PUSH 2u
#define MODE_COUNT 3u

/* 切换来源, 只用于日志/屏幕 */
#define MODE_SRC_BOOT 0u
#define MODE_SRC_KEY 1u
#define MODE_SRC_UART 2u

/* 上电进入的模式 (默认上位机控制) */
#define MODE_POWER_ON MODE_HOST

void Mode_Init(void);

/* 当前模式 */
uint8_t Mode_Get(void);

/* 名字/说明, 用于屏幕和日志。非法编号返回 "?" */
const char *Mode_Name(uint8_t mode);
const char *Mode_Hint(uint8_t mode);
const char *Mode_SourceName(uint8_t source);

/* 设置模式: 返回 1 表示模式真的变了(需要刷新屏幕/灯), 0 表示没变或编号非法 */
uint8_t Mode_Set(uint8_t mode, uint8_t source);

/* 切到下一个模式 */
uint8_t Mode_Next(uint8_t source);

/* 取出"模式已变"标志(取一次就清), 以及最后一次切换的来源 */
uint8_t Mode_TakeChanged(void);
uint8_t Mode_ChangedSource(void);

#endif
