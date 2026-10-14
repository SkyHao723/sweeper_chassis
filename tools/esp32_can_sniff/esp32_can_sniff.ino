/*============================================================================
 *  ESP32-S3 CAN **总线探测器**(sniffer) —— 只听, 一个字节都不发
 *
 *  【用途】把 ESP32 挂到总线上, 它会把**经过总线的每一帧**(任何 ID)打印出来:
 *      - STM32 发的命令帧 (0x1101E600 / 0x1201E600)
 *      - 驱动器的控制回码 (0x1101E601 / 0x1201E601)
 *      - 驱动器的定时上报 (0x1101E603 / 0x1201E603)
 *      - 任何别的东西
 *
 *  ★★ 因为**完全不发送**, 所以可以安全地和 STM32 同时挂在一条总线上 —— 它不会
 *     参与仲裁、不会抢总线、不会影响任何东西。这正是"两个主控 + 驱动器"那种
 *     局面下最需要的工具: 它能回答"总线上到底有没有活动、谁在说话"。
 *
 *  【怎么用它定位问题】
 *     总线上一片安静              -> 没有任何节点在工作(供电/接线/收发器)
 *     能看到 STM32 的 0x1101E600  -> STM32 在发, ESP32 的**接收**也是好的
 *     能看到驱动器回码/上报        -> 驱动器活着且在总线上
 *     只有 STM32 的帧、没有驱动器  -> 驱动器没收到 / 它不回(但它串口明明活着)
 *     什么都没有                  -> 线路/终端电阻/收发器
 *
 *  【接线】ESP32-S3 --(外接 CAN 收发器)-- 底盘 CAN_H/CAN_L, 两端 120Ω
 *    ★ 这个固件**不需要** STM32 断电(它不发东西, 不抢总线)
 *
 *  【用法】烧进去, 串口 115200。会实时打印每一帧 + 每 3 秒报一次各 ID 计数。
 *==========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

/*--------------------------- 按你的接线改 ---------------------------*/
#define CAN_TX_PIN      GPIO_NUM_4
#define CAN_RX_PIN      GPIO_NUM_5
/*-------------------------------------------------------------------*/

static uint32_t total = 0;
static uint32_t err_bus = 0;   /* 观察到的错误计数变化 */

/* 计数表: 最多记 16 种 ID */
typedef struct { uint32_t id; uint32_t n; } idcnt_t;
static idcnt_t cnts[16];
static int ncnt = 0;

static void bump(uint32_t id)
{
    for (int i = 0; i < ncnt; i++) if (cnts[i].id == id) { cnts[i].n++; return; }
    if (ncnt < 16) { cnts[ncnt].id = id; cnts[ncnt].n = 1; ncnt++; }
}

static const char *id_name(uint32_t id)
{
    switch (id) {
        case 0x1101E600UL: return "STM32->1号 命令";
        case 0x1201E600UL: return "STM32->2号 命令";
        case 0x1101E601UL: return "1号 控制回码";
        case 0x1201E601UL: return "2号 控制回码";
        case 0x1101E603UL: return "1号 定时上报";
        case 0x1201E603UL: return "2号 定时上报";
        default: return "";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1200);
    Serial.println();
    Serial.println("========================================================");
    Serial.println(" ESP32-S3 CAN 总线探测器 (**只听, 不发任何帧**)");
    Serial.println(" 可以安全地和 STM32 同时挂在一条总线上");
    Serial.println(" 接线: CAN_TX=4 CAN_RX=5 -> 外接收发器 -> CAN_H/CAN_L, 两端120Ω");
    Serial.println("========================================================");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        Serial.println("★ CAN 启动失败 —— 检查引脚"); while (1) delay(1000);
    }
    Serial.println("CAN 已启动(500kbps)。开始监听…\n");
}

void loop()
{
    static uint32_t last_stat = 0;
    twai_message_t m;

    while (twai_receive(&m, 0) == ESP_OK) {
        total++;
        uint32_t id = m.identifier;
        bump(id);

        Serial.printf("[%8lu] ID=%08lX DLC=%u  %s  |", (unsigned long)millis(),
                      (unsigned long)id, (unsigned)m.data_length_code, id_name(id));
        for (int i = 0; i < m.data_length_code && i < 8; i++)
            Serial.printf(" %02X", m.data[i]);
        if (m.extd) Serial.print("   (扩展帧)");
        Serial.println();
    }

    if (millis() - last_stat >= 3000) {
        last_stat = millis();
        twai_status_info_t st;
        twai_get_status_info(&st);
        Serial.printf("\n--- 统计: 共 %lu 帧 | 状态=%d | rx错误=%lu 总线错误=%lu 丢收=%lu ---\n",
                      (unsigned long)total, (int)st.state,
                      (unsigned long)st.rx_error_counter,
                      (unsigned long)st.bus_error_count,
                      (unsigned long)st.rx_missed_count);
        if (ncnt == 0) {
            Serial.println("    **总线上一个帧都没有** —— 没有任何节点在发。查:");
            Serial.println("      供电(驱动器灯) / CAN_H-CAN_L 接线 / 终端电阻 / 收发器 / 波特率");
        } else {
            for (int i = 0; i < ncnt; i++)
                Serial.printf("   ID %08lX %-18s : %lu 帧\n",
                              (unsigned long)cnts[i].id, id_name(cnts[i].id),
                              (unsigned long)cnts[i].n);
            Serial.println("   ★ 若只看到 0x1101E600/0x1201E600(STM32的命令) 而没有任何");
            Serial.println("     驱动器的回码/上报 -> 驱动器没在总线上说话");
        }
        Serial.println();
    }
    delay(2);
}
