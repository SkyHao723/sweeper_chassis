#ifndef __PROTO_H
#define __PROTO_H

/**
  ******************************************************************************
  * @file    proto.h
  * @brief   上位机串口协议 (USART1 115200 8N1)
  *
  * 这一版只需要"能收命令切换模式", 但帧格式直接沿用旧底盘那套, 以后做运动控制
  * 不用改上位机。两帧都用 0x7D 结尾、BCC = 前面所有字节异或。
  *
  * 1) 速度帧 (11 字节, 上位机 20~30 Hz 发)
  *      [0]=0x7B  [1]=cmd(保留, 0)  [2]=保留
  *      [3..4]=vx  int16 大端  mm/s
  *      [5..6]=vy  int16 大端  mm/s   (差速底盘不用, 填 0)
  *      [7..8]=wz  int16 大端  mrad/s
  *      [9]=BCC = XOR([0..8])   [10]=0x7D
  *    ★ 只在 MODE_HOST 模式下接受; 遥控/推行模式下丢弃并计数(REJ)。
  *
  * 2) 功能帧 (8 字节, 偶发事件用)
  *      [0]=0x7A  [1]=func  [2..5]=p0..p3  [6]=BCC = XOR([0..5])  [7]=0x7D
  *      func=0x01 按辅助按钮(旧底盘的刷盘/风机, V2 板没有这级硬件, 收到只计数)
  *      func=0x02 切换模式: p0 = 0 上位机控制 / 1 遥控 / 2 推行
  *
  * 现成可抄的切换模式帧:
  *      上位机控制  7A 02 00 00 00 00 78 7D
  *      遥控        7A 02 01 00 00 00 79 7D
  *      推行        7A 02 02 00 00 00 7A 7D
  *
  * 3) 调试注入帧 (18 字节, 只给调试用)
 *      [0]=0x78  [1..15]=15 字节手柄原始 HID 报告  [16]=BCC=XOR([0..15])  [17]=0x7D
 *    ★ 正常的手柄数据是从 USART3 (CH554G) 进来的; 这一帧让上位机在没有接收器/
 *      没有 CH554G 时也能把 15 字节塞进同一条解析+映射路径, 用来验证遥控模式
 *      和上行遥控帧(0x79)。见 tools/v2_gamepad.py。
 *
 * 串口只做"收", 不做应答/回包; 状态从屏幕和调试打印上看(以后再加回包)。
  ******************************************************************************
  */

#include "stm32f10x.h"

#define PROTO_CMD_HEAD 0x7Bu
#define PROTO_CMD_SIZE 11u
#define PROTO_FUNC_HEAD 0x7Au
#define PROTO_FUNC_SIZE 8u
#define PROTO_PAD_HEAD 0x78u
#define PROTO_PAD_SIZE 18u
#define PROTO_TAIL 0x7Du

#define PROTO_FUNC_ID_AUX_BTN 0x01u
#define PROTO_FUNC_ID_SET_MODE 0x02u

void Proto_Init(void);

/* 逐字节喂数据 (主循环从 USART1 读一个喂一个) */
void Proto_FeedByte(uint8_t byte);

/* ---------------------------- 统计 / 查询 ---------------------------- */

/* 收到并接受的速度帧数 (只在 HOST 模式下累加) */
uint16_t Proto_HostCmdCount(void);

/* 因为不在 HOST 模式被丢弃的速度帧数 */
uint16_t Proto_HostRejectCount(void);

/* BCC/帧尾不对被丢掉的帧数 (线路噪声/波特率不对的直观指标) */
uint16_t Proto_BadFrameCount(void);

/* 收到的功能帧数 */
uint16_t Proto_FuncFrameCount(void);

/* 最近一条被接受的速度命令 (mm/s, mrad/s) */
void Proto_GetHostCmd(int16_t *vx_mm_s, int16_t *wz_mrad_s);

#endif
