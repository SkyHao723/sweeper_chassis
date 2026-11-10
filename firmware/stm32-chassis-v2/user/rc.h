#ifndef __RC_H
#define __RC_H

/**
  ******************************************************************************
  * @file    rc.h
  * @brief   遥控输入: CH554G (USB-A 主机口) -> USART3 -> 手柄接收器原始数据
  *
  * 硬件链路
  * ---------------------------------------------------------------------------
  *   手柄接收器 (Xbox 360 兼容, VID 045E PID 028E, HID, 500Hz)
  *        | USB
  *   CH554G (板上另一颗 MCU, 负责 USB 主机口)
  *        | UART, 115200 8N1, 原样转发
  *   STM32: PB11 = USART3_RX   (PB10 = USART3_TX, 本固件目前不发)
  *
  * CH554G 要发的就是接收器的 15 字节 HID 输入报告, 一个字节都不改:
  * ---------------------------------------------------------------------------
  *   [0]      0x00            固定 0x00 (当帧头用)
  *   [1..2]   X   int16 LE    左摇杆 X, 0..65535, 中位 32768
  *   [3..4]   Y   int16 LE    左摇杆 Y, 0..65535, 中位 32767 (向上 = 变大)
  *   [5..6]   Rx  int16 LE    右摇杆 X, 中位 32768
  *   [7..8]   Ry  int16 LE    右摇杆 Y, 中位 32767
  *   [9..10]  Z   int16 LE    = 32768 + (LT - RT) * 128, 中位 32768
  *   [11]     按钮 低 8 位    bit0=A bit1=B bit2=X bit3=Y bit4=LB bit5=RB
  *                            bit6=Back bit7=Start
  *   [12]     按钮高 2 位 + 十字键
  *                            bit0=LS(左摇杆按下) bit1=RS(右摇杆按下)
  *                            bit2..5 = 十字键 0=中 1=上 2=右上 3=右
  *                                      4=右下 5=下 6=左下 7=左 8=左上
  *   [13]     0x00            固定 0x00
  *   [14]     0x00            固定 0x00
  *
  * 这份布局不是猜的, 是对着 D:\dsh\115 (2)\sample_capture.csv 里 17400 帧
  * 原始数据 + Windows HID 解码结果逐字节对出来的 (见 README §1.3)。
  *
  * 帧同步
  * ---------------------------------------------------------------------------
  * 裸字节流没有长度/校验, 所以用帧内三个固定字节 + 十字键范围做同步:
  *   [0]==0x00 且 [13]==0x00 且 [14]==0x00 且 ([12] & 0xE0)==0 且 十字键<=8
  * 连续 3 个都成立才认一帧, 校验不过就滑一个字节重新找 (丢字节能自恢复)。
  * 500Hz 下每 2ms 一帧, 偶尔重同步完全不影响手感。
  ******************************************************************************
  */

#include "stm32f10x.h"

/* 一帧原始数据的长度 (接收器 HID 报告长度) */
#define RC_FRAME_SIZE 15u

/* 波形参数 (可按手感改) */
#define RC_VX_MAX_MM_S 600      /* 左摇杆推到顶 = 前进 0.6 m/s */
#define RC_WZ_MAX_MRAD_S 800    /* 左摇杆推到最右 = 右转 0.8 rad/s */
#define RC_DEADZONE_PERMILLE 100 /* 摇杆死区 10% (千分制) */

/* 遥控状态 (全部是接收器原始值, 不做归一化, 上位机/屏幕自己换算) */
typedef struct
{
    uint16_t x;        /* 左摇杆 X, 中位 32768 */
    uint16_t y;        /* 左摇杆 Y, 中位 32767, 向上变大 */
    uint16_t rx;       /* 右摇杆 X */
    uint16_t ry;       /* 右摇杆 Y */
    uint16_t z;        /* 扳机差, 中位 32768 */
    uint8_t buttons;   /* bit0=A ... bit7=Start */
    uint8_t buttons2;  /* bit0=LS bit1=RS */
    uint8_t hat;       /* 十字键 0..8 */
    uint8_t valid;     /* 1 = 这一份数据是有效的 */
} Rc_State;

/* 初始化 USART3 (PB10=TX/PB11=RX, 115200 8N1, 只开接收中断) */
void Rc_Init(void);

/* 主循环里每拍调一次: 把环形缓冲里的字节喂给解析器, 更新在线状态 */
void Rc_Poll(void);

/* 最近一帧遥控数据 (没收到过时返回全 0 + valid=0) */
const Rc_State *Rc_Get(void);

/* 1 = 最近 RC_ONLINE_MS(300ms) 内收到过有效帧 */
uint8_t Rc_Online(void);

/* 遥控输入映射成底盘速度指令 (遥控模式下用), 单位 mm/s 与 mrad/s */
void Rc_GetCmd(int16_t *vx_mm_s, int16_t *wz_mrad_s);

/* 左摇杆归一化到千分制 (-1000..1000), 已扣死区, 屏幕/调试用 */
void Rc_GetStickPermille(int16_t *x_permille, int16_t *y_permille);

/* 统计 */
uint16_t Rc_FrameCount(void);  /* 解析成功的帧数 */
uint16_t Rc_BadCount(void);    /* 校验失败/同步丢弃的字节数 */
uint16_t Rc_InjectCount(void); /* 调试串口注入的帧数 (见 Proto_FUNC 0x78) */
uint16_t Rc_DropCount(void);   /* 环形缓冲溢出丢的字节数 */

/* 上行遥控帧 (19 字节, 20Hz 发一次): 把最近一帧原始数据原样带给上位机
 *   [0]     0x79 帧头
 *   [1]     flags bit0=在线 bit1=这一帧和上一帧不同
 *   [2..16] 15 字节原始 HID 报告 (布局同上面说明)
 *   [17]    BCC = XOR([0..16])
 *   [18]    0x7D 帧尾
 * 返回这一帧的 BCC (方便上层打印) */
uint8_t Rc_SendFrame(void);

/* 调试用: 直接把 15 字节原始帧塞进来 (等效于 CH554G 转发了一帧),
 * 走和串口完全一样的校验/解码路径。返回 1 = 校验通过并已采纳 */
uint8_t Rc_InjectFrame(const uint8_t *frame);

#endif
