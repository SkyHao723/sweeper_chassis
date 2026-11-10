#ifndef __TELEMETRY_H
#define __TELEMETRY_H

/**
  ******************************************************************************
  * @file    telemetry.h
  * @brief   上行遥测帧: 把电机指令 / IMU / 温湿度 打包发给上位机
  *
  * 帧格式 (32 字节, 大端 int16 与底盘速度帧一致, 方便上位机复用解析代码):
  *   [0]     0x7E  帧头
  *   [1]     flags bit0=IMU 正常, bit1=AHT20 正常, bit2..3=当前模式
  *   [2]     mode  0=HOST 1=RC 2=PUSH
  *   [3..4]  vx 指令 mm/s      (int16 BE)
  *   [5..6]  wz 指令 mrad/s    (int16 BE)
  *   [7..8]  acc X mg          (int16 BE)
  *   [9..10] acc Y mg
  *   [11..12]acc Z mg
  *   [13..14]gyro X 0.1°/s     (int16 BE)
  *   [15..16]gyro Y 0.1°/s
  *   [17..18]gyro Z 0.1°/s
  *   [19..20]IMU 内部温度 0.01°C (int16 BE)
  *   [21..22]湿度 0.01%RH      (uint16 BE)
  *   [23..24]AHT20 温度 0.01°C (int16 BE)
  *   [25..26]tick ms 低 16 位  (uint16 BE)
  *   [27]    接受的host命令数 (饱和到 255)
  *   [28]    拒绝的命令数     (饱和到 255)
  *   [29]    协议版本 (TELEMETRY_VERSION)
  *   [30]    BCC = XOR([0..29])
  *   [31]    0x7D 帧尾
  ******************************************************************************
  */

#include "stm32f10x.h"

#define TELEMETRY_HEAD 0x7Eu
#define TELEMETRY_TAIL 0x7Du
#define TELEMETRY_FRAME_SIZE 32u
#define TELEMETRY_VERSION 1u

#define TELEMETRY_FLAG_IMU_OK 0x01u
#define TELEMETRY_FLAG_AHT_OK 0x02u
#define TELEMETRY_MODE_SHIFT 2u
#define TELEMETRY_MODE_MASK 0x0Cu

typedef struct
{
    uint8_t mode;
    uint8_t imu_ok;
    uint8_t aht_ok;
    int16_t vx_mm_s;
    int16_t wz_mrad_s;
    int16_t acc_mg[3];
    int16_t gyro_0d1dps[3];
    int16_t imu_temp_c100;
    uint16_t rh_100;
    int16_t aht_temp_c100;
    uint16_t tick_ms;
    uint8_t rx_count;
    uint8_t rj_count;
} Telemetry_Data;

/* 组一帧到 frame (必须 >= TELEMETRY_FRAME_SIZE 字节) */
void Telemetry_Build(const Telemetry_Data *data, uint8_t *frame);

/* 组帧并通过 USART1 发出去, 返回本帧的 BCC (方便上层打印自检) */
uint8_t Telemetry_Send(const Telemetry_Data *data);

/* 已发送帧数 */
uint16_t Telemetry_SentCount(void);

/* 最近一次发出的帧 (调试用: 没接串口时可用调试器 dump 这段 RAM 核对数据) */
const volatile uint8_t *Telemetry_LastFrame(void);

#endif
