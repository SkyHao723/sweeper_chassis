/*============================================================================
 *  ESP32-S3 CAN **链路**诊断 —— 只查通信, 不发任何运动命令
 *
 *  【为什么单独做这一个】上一个诊断程序收不到回码, 但那种情况有两种完全不同的原因,
 *    而它们在串口上长得一模一样:
 *      ① 我发不出去(总线没人 ACK -> TWAI 进入 Bus-Off)  -> 驱动器没电/线断/终端电阻
 *      ② 我发得出去但驱动器不回                        -> 驱动器侧问题
 *    这个程序把 TWAI 的**内部状态**打出来, 直接分开这两种。
 *
 *  【上一版的一个真问题】esp32_can_check 里**没有处理 Bus-Off**:
 *    TX 失败时 TWAI 会自动重传 -> 错误计数累积 -> 进 Bus-Off -> **彻底停止收发**,
 *    而且必须主动恢复。所以"发不出去"在上一版里被表现成了"什么都没有"。
 *    这一版: 检测 Bus-Off 并自动恢复, 同时把错误计数打出来。
 *
 *  【安全性】这个版本**只发"使能 + 值 0"和"刹车"两种帧, 不产生任何运动**
 *    (值 0 = 目标转速 0)。所以**架空还是落地都无所谓**, 但架空更保险。
 *
 *  【接线】ESP32-S3 --(外接 CAN 收发器)-- 底盘 CAN_H/CAN_L, 两端 120Ω
 *    ★★ STM32 必须断电或拔掉它的 CAN
 *==========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

/*--------------------------- 按你的接线改 ---------------------------*/
#define CAN_TX_PIN      GPIO_NUM_4
#define CAN_RX_PIN      GPIO_NUM_5
/*-------------------------------------------------------------------*/

#define ID_M1_CMD       0x1101E600UL
#define ID_M2_CMD       0x1201E600UL
#define ID_M1_REPLY     0x1101E601UL
#define ID_M2_REPLY     0x1201E601UL
#define ID_M1_REPORT    0x1101E603UL
#define ID_M2_REPORT    0x1201E603UL

#define MODE_SPEED      0x05
#define CTRL_ENABLE     0x01
#define CTRL_BRAKE      0x03

static uint32_t tx_ok = 0, tx_fail = 0;
static uint32_t rx_reply1 = 0, rx_reply2 = 0, rx_rep1 = 0, rx_rep2 = 0, rx_other = 0;
static bool     tried_recover = false;

static const char *state_name(twai_state_t s)
{
    switch (s) {
        case TWAI_STATE_STOPPED:     return "STOPPED";
        case TWAI_STATE_RUNNING:     return "RUNNING";
        case TWAI_STATE_BUS_OFF:     return "★BUS_OFF";
        case TWAI_STATE_RECOVERING:  return "RECOVERING";
        default:                     return "?";
    }
}

static void send_one(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t v)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id; m.extd = 1; m.data_length_code = 8;
    m.data[0] = mode; m.data[1] = ctrl;
    m.data[2] = (uint8_t)(((uint16_t)v) >> 8);
    m.data[3] = (uint8_t)((uint16_t)v & 0xFF);
    m.data[4] = 0; m.data[5] = 0; m.data[6] = 0; m.data[7] = 0;
    if (twai_transmit(&m, pdMS_TO_TICKS(5)) == ESP_OK) tx_ok++; else tx_fail++;
}

static void poll_rx(void)
{
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd) continue;
        if      (m.identifier == ID_M1_REPLY)  rx_reply1++;
        else if (m.identifier == ID_M2_REPLY)  rx_reply2++;
        else if (m.identifier == ID_M1_REPORT) rx_rep1++;
        else if (m.identifier == ID_M2_REPORT) rx_rep2++;
        else                                   rx_other++;
    }
}

/* 打印 TWAI 内部状态 —— 这个程序的核心 */
static void dump_status(const char *tag)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        Serial.println("  ★ twai_get_status_info 失败");
        return;
    }
    Serial.printf("  [%s] 状态=%-10s | tx错误=%3lu rx错误=%3lu | 发送 成功%lu 失败%lu | "
                  "总线错误=%lu 仲裁丢失=%lu | 待发%lu 待收%lu 丢收%lu 溢出%lu\n",
                  tag, state_name(st.state),
                  (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                  (unsigned long)tx_ok, (unsigned long)tx_fail,
                  (unsigned long)st.bus_error_count, (unsigned long)st.arb_lost_count,
                  (unsigned long)st.msgs_to_tx, (unsigned long)st.msgs_to_rx,
                  (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count);

    /* ★ Bus-Off 自动恢复(上一版缺的就是这个) */
    if (st.state == TWAI_STATE_BUS_OFF) {
        Serial.println("    ★★ 控制器处于 BUS_OFF —— 主动发起恢复 …");
        twai_initiate_recovery();
        tried_recover = true;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1200);
    Serial.println();
    Serial.println("========================================================");
    Serial.println(" ESP32-S3 CAN **链路**诊断 (只查通信, 不发运动命令)");
    Serial.println(" 只发「使能+值0」和「刹车」两种帧, 不会产生运动");
    Serial.println(" ★ STM32 必须断电或拔掉它的 CAN; 两端 120Ω 终端电阻");
    Serial.println("========================================================");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("★ twai_driver_install 失败(引脚?)");
        while (1) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("★ twai_start 失败");
        while (1) delay(1000);
    }
    Serial.println("CAN 已启动 (500kbps, 扩展帧), 引脚 TX=4 RX=5");
    dump_status("初始化后");

    /*---- 阶段 A: 只收不发 3 秒(看有没有 E603 主动上报) ----*/
    Serial.println("\n[A] 只收不发 3 秒 —— 看驱动器有没有自己上报(E603)");
    { uint32_t u = millis() + 3000; while ((int32_t)(millis()-u) < 0) { poll_rx(); delay(5); } }
    Serial.printf("  收到: 1号回码%lu 2号回码%lu | 1号上报%lu 2号上报%lu | 其它%lu\n",
                  (unsigned long)rx_reply1, (unsigned long)rx_reply2,
                  (unsigned long)rx_rep1, (unsigned long)rx_rep2, (unsigned long)rx_other);
    dump_status("只收3秒后");

    /*---- 阶段 B: 发 5 秒(每秒报告一次状态) ----*/
    Serial.println("\n[B] 发 5 秒(使能+值0, 不会动), 每秒报一次状态");
    for (int sec = 0; sec < 5; sec++) {
        uint32_t u = millis() + 1000;
        while ((int32_t)(millis()-u) < 0) {
            send_one(ID_M1_CMD, MODE_SPEED, CTRL_ENABLE, 0);
            send_one(ID_M2_CMD, MODE_SPEED, CTRL_ENABLE, 0);
            poll_rx();
            delay(20 - 1);
        }
        char tag[16]; snprintf(tag, sizeof(tag), "第%ds", sec + 1);
        dump_status(tag);
    }

    /*---- 阶段 C: 结论 ----*/
    twai_status_info_t st;
    twai_get_status_info(&st);
    Serial.println();
    Serial.println("================= 判读 =================");
    Serial.printf(" 最终状态: %s | tx成功%lu 失败%lu | 收到回码 1号%lu 2号%lu\n",
                  state_name(st.state), (unsigned long)tx_ok, (unsigned long)tx_fail,
                  (unsigned long)rx_reply1, (unsigned long)rx_reply2);
    if (st.state == TWAI_STATE_BUS_OFF || st.tx_error_counter > 100)
        Serial.println(" ★ 我**发不出去** -> 总线没人 ACK。查: 驱动器有没有电(空开/指示灯)、"
                       "CAN_H/CAN_L 有没有接反或断、终端电阻、收发器供电");
    else if (tx_fail > 0)
        Serial.println(" ★ 发送有失败 -> 总线物理层可疑(终端电阻/线长/接触)");
    else if (rx_reply1 == 0 && rx_reply2 == 0)
        Serial.println(" ★ 我发得出去, 但**驱动器一个都没回** -> 驱动器侧: 没电 / 已损坏 / "
                       "ID 不对 / 只接受特定帧");
    else
        Serial.println(" ✓ 通信是通的");

    Serial.println("\n★ 请把上面**每一行状态打印**都发给我 —— 错误计数在不在涨、");
    Serial.println("  状态有没有变成 BUS_OFF, 是分开'代码问题'和'硬件问题'的关键。");
    Serial.println("已进入持续刹车(值0/刹车帧)。");
}

void loop()
{
    /* 保持通信监测, 每 2 秒报一次状态; 不发运动命令 */
    static uint32_t last = 0;
    poll_rx();
    send_one(ID_M1_CMD, MODE_SPEED, CTRL_BRAKE, 0);
    send_one(ID_M2_CMD, MODE_SPEED, CTRL_BRAKE, 0);

    if (millis() - last >= 2000) {
        last = millis();
        dump_status("心跳");
        Serial.printf("       累计收到: 1号回码%lu 2号回码%lu 上报%lu/%lu 其它%lu\n",
                      (unsigned long)rx_reply1, (unsigned long)rx_reply2,
                      (unsigned long)rx_rep1, (unsigned long)rx_rep2, (unsigned long)rx_other);
    }
    delay(20 - 1);
}
