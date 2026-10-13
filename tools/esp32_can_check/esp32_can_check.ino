/*============================================================================
 *  ESP32-S3 安全诊断 —— 查"另一个轮子 + 两个驱动器"坏没坏
 *
 *  ⚠️ 背景: 已经有一个轮子烧坏了。所以这个程序**只有一个目的: 用最小的
 *     电应力把好/坏区分开**。它不是性能测试, 不是调参工具。
 *
 *  【三条硬设计原则 —— 都是为了不再烧电机】
 *   ① **单轮测**: 一次只让一个轮子动, 另一个发刹车。这样坏的那一侧不会
 *      被另一个拖着转, 而且能干净地分清"是哪一侧的问题"。
 *   ② **极低速 + 短时**: 目标 15 RPM(150mm轮 = 0.12 m/s), 每项只跑 **2 秒**,
 *      项间冷却 3 秒。架空时这个负载只需要约 0.3A。
 *   ③ **异常立刻自停**: 每 20ms 检查一次 —— 只要出现"电流大但转速几乎不动"
 *      (>2.0A 且 <5RPM 持续 300ms) 或"转速冲高"(>120RPM), **立刻刹车并跳过后续**。
 *      ★ 这一条就是上次烧电机的直接教训: 堵转时电流会一直灌进去, 而全变热。
 *
 *  【必须**架空**】两条: 1) 不用担心车撞 2) 空载电流小, 好/坏容易分
 *    ★ 用手转一下两个轮子, 确认都不是卡死的, 再上电。
 *
 *  【接线】ESP32-S3 --(外接 CAN 收发器)-- 底盘 CAN_H/CAN_L
 *    ★★ STM32 必须断电或拔掉它的 CAN(两个主控同时发会抢总线)
 *    ★★ CAN_TX_PIN / CAN_RX_PIN 改成你实际接的脚
 *
 *  【用法】烧进去, 串口 115200。它会自动跑完并打印一张诊断表。
 *    ★ 中途发现异常会自动停那一路并继续下一步(不会卡住不动)。
 *    ★ 串口发 'x' 立刻全部停止。
 *
 *  【怎么判读】(代码最后也会打印一张同样的表)
 *    转速跟得上目标, 电流 < 1A          -> 电机 + 驱动器都好 ✓
 *    有转速回码但明显跟不上(如只到 3RPM) -> 电机侧有问题(堵转/绕组/机械)
 *    **电流 > 2A 而转速 ≈ 0**            -> ★ 电机堵转 或 驱动器输出异常
 *    转速回码剧烈跳动(0↔目标几倍)         -> 霍尔/编码器异常
 *    收不到回码                          -> 驱动器或 CAN 问题
 *    故障码非 0                          -> 查 docs/foc-can-protocol.md 的故障表
 *==========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

/*--------------------------- 按你的接线改 ---------------------------*/
#define CAN_TX_PIN      GPIO_NUM_4
#define CAN_RX_PIN      GPIO_NUM_5
/*-------------------------------------------------------------------*/

#define ID_M1_CMD       0x1101E600UL      /* 1号(左轮) */
#define ID_M2_CMD       0x1201E600UL      /* 2号(右轮) */
#define ID_M1_REPLY     0x1101E601UL
#define ID_M2_REPLY     0x1201E601UL
#define ID_M1_REPORT    0x1101E603UL      /* 定时上报(含电压/温度) */
#define ID_M2_REPORT    0x1201E603UL

#define MODE_TORQUE     0x04
#define MODE_SPEED      0x05
#define CTRL_ENABLE     0x01
#define CTRL_BRAKE      0x03

/*---------------------- ★ 安全参数(别调大) ----------------------*/
#define LOW_RPM         15        /* 第一档目标转速: 很低 */
#define HIGH_RPM        30        /* 第二档(只有第一档通过才做) */
#define TEST_MS         2000      /* 每项只跑 2 秒 */
#define COOLDOWN_MS     3000      /* 项间冷却 */
#define ABORT_CUR_A10   20        /* ★ 电流 > 2.0A 且转速 < 5RPM... */
#define ABORT_STILL_RPM 5
#define ABORT_HOLD_MS   300       /* ...持续 300ms -> 立刻刹车 */
#define ABORT_HIGH_RPM  120       /* ★ 转速 > 120 RPM 也立刻刹车 */
#define CMD_PERIOD_MS   20

typedef struct {
    uint8_t  mode, fault;
    int16_t  rpm, cur;
    uint8_t  mos_temp;              /* 来自 E603 (0xFF = 没收到) */
    uint16_t volt_v10;
    uint32_t n_reply, n_report;
    /* 统计 */
    int16_t  rpm_max, cur_max;
    int32_t  rpm_sum, cur_sum; uint32_t n_stat;
    bool     aborted;
    uint32_t abort_ms;
} drv_t;

static drv_t d1, d2;
static bool  g_stop = false;

static void reset_stat(drv_t *d)
{
    d->rpm_max = 0; d->cur_max = 0; d->rpm_sum = 0; d->cur_sum = 0; d->n_stat = 0;
    d->aborted = false; d->abort_ms = 0;
}

/*======================= CAN 基本操作 =======================*/
static void send_one(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t v)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id; m.extd = 1; m.data_length_code = 8;
    m.data[0] = mode; m.data[1] = ctrl;
    m.data[2] = (uint8_t)(((uint16_t)v) >> 8);
    m.data[3] = (uint8_t)((uint16_t)v & 0xFF);
    m.data[4] = 0; m.data[5] = 0; m.data[6] = 0; m.data[7] = 0;
    twai_transmit(&m, pdMS_TO_TICKS(5));
}

static void poll_rx(void)
{
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd || m.data_length_code < 4) continue;
        if (m.identifier == ID_M1_REPLY) {
            d1.mode = m.data[0]; d1.fault = m.data[1];
            d1.rpm = (int16_t)(((uint16_t)m.data[2] << 8) | m.data[3]);
            if (m.data_length_code >= 6)
                d1.cur = (int16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
            d1.n_reply++;
        } else if (m.identifier == ID_M2_REPLY) {
            d2.mode = m.data[0]; d2.fault = m.data[1];
            d2.rpm = (int16_t)(((uint16_t)m.data[2] << 8) | m.data[3]);
            if (m.data_length_code >= 6)
                d2.cur = (int16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
            d2.n_reply++;
        } else if (m.identifier == ID_M1_REPORT) {
            if (m.data_length_code >= 6) d1.mos_temp = m.data[3];
            if (m.data_length_code >= 6)
                d1.volt_v10 = (uint16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
            d1.n_report++;
        } else if (m.identifier == ID_M2_REPORT) {
            if (m.data_length_code >= 6) d2.mos_temp = m.data[3];
            if (m.data_length_code >= 6)
                d2.volt_v10 = (uint16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
            d2.n_report++;
        }
    }
}

static void poll_serial(void)
{
    /* ★ 必须先把读到的字节存下来再判断 —— 写成
       `if (Serial.read()=='x' || Serial.read()=='X')` 会把 read() 调用两次,
       第二次读到的是**下一个字节**(或者 -1), 'x' 就不可靠了。 */
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'x' || c == 'X') g_stop = true;
    }
}

/* 一个轮子发命令, 另一个发刹车。返回前把统计收好。 */
static void run_one(const char *tag, uint32_t id_cmd, int16_t rpm, drv_t *me,
                    uint32_t id_other, drv_t *other)
{
    uint32_t t0 = millis();
    uint32_t still_since = 0;
    reset_stat(me); reset_stat(other);

    Serial.printf("\n--- %s  (目标 %d RPM, 持续 %d ms) ---\n", tag, rpm, TEST_MS);

    while ((int32_t)(millis() - t0) < (int32_t)TEST_MS) {
        poll_serial();
        if (g_stop) break;

        send_one(id_cmd,   MODE_SPEED, CTRL_ENABLE, rpm);
        send_one(id_other, MODE_SPEED, CTRL_BRAKE,  0);
        poll_rx();

        int ar = (me->rpm < 0) ? -me->rpm : me->rpm;
        int ac = (me->cur < 0) ? -me->cur : me->cur;
        if (ar > me->rpm_max) me->rpm_max = (int16_t)ar;
        if (ac > me->cur_max) me->cur_max = (int16_t)ac;
        me->rpm_sum += ar; me->cur_sum += ac; me->n_stat++;

        /* ★★ 异常自停: 电流大却不转 */
        if (ac > ABORT_CUR_A10 && ar < ABORT_STILL_RPM) {
            if (still_since == 0) still_since = millis();
            else if ((millis() - still_since) >= ABORT_HOLD_MS) {
                me->aborted = true; me->abort_ms = millis() - t0;
                Serial.printf("  ★★ 自停: 电流 %d(0.1A) 但转速只有 %d RPM —— 堵转/输出异常, 立刻刹车\n", ac, ar);
                break;
            }
        } else if (ar >= ABORT_STILL_RPM) {
            still_since = 0;                 /* 转起来就把计时清零 */
        } else {
            still_since = 0;                 /* ★ 条件不满足(电流掉下来了)也要清零,
                                                否则计时会跨过"电流已经不大了"的间隙算错 */
        }

        /* ★ 转速冲高也停 */
        if (ar > ABORT_HIGH_RPM) {
            me->aborted = true; me->abort_ms = millis() - t0;
            Serial.printf("  ★★ 自停: 转速 %d RPM 超过 %d —— 立刻刹车\n", ar, ABORT_HIGH_RPM);
            break;
        }

        delay(CMD_PERIOD_MS - 1);
    }

    /* 收尾: 两个都刹车 */
    {
        uint32_t until = millis() + 200;
        while ((int32_t)(millis() - until) < 0) {
            send_one(ID_M1_CMD, MODE_SPEED, CTRL_BRAKE, 0);
            send_one(ID_M2_CMD, MODE_SPEED, CTRL_BRAKE, 0);
            poll_rx();
            delay(CMD_PERIOD_MS - 1);
        }
    }
}

static void report_line(const char *who, const drv_t *d, const char *tag)
{
    int avg_rpm = d->n_stat ? (int)(d->rpm_sum / (int32_t)d->n_stat) : 0;
    int avg_cur = d->n_stat ? (int)(d->cur_sum / (int32_t)d->n_stat) : 0;
    Serial.printf("  %s %-22s 回码%5lu帧 模式=%-4s 故障=%3u | 转速 均%+5d 峰%+5d | 电流 均%+5d 峰%+5d (0.1A) %s\n",
                  who, tag, (unsigned long)d->n_reply,
                  (d->mode == 4 ? "力矩" : d->mode == 5 ? "速度" : "?"),
                  (unsigned)d->fault,
                  avg_rpm, (int)d->rpm_max, avg_cur, (int)d->cur_max,
                  d->aborted ? "★本项被自停" : "");
}

void setup()
{
    Serial.begin(115200);
    delay(1200);
    Serial.println();
    Serial.println("========================================================");
    Serial.println(" ESP32-S3 安全诊断: 查另一个轮子 + 两个驱动器");
    Serial.println(" ⚠️ 必须**架空**! 上电前先用手转一下两个轮子, 确认都不卡");
    Serial.println(" ⚠️ STM32 必须断电或拔掉它的 CAN");
    Serial.println(" 参数: 目标 15/30 RPM, 每项 2 秒, 项间冷却 3 秒");
    Serial.println("       电流>2.0A 且转速<5RPM 持续 300ms -> 立刻自停");
    Serial.println(" 串口发 'x' 随时全部停止");
    Serial.println("========================================================");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
        Serial.println("★ CAN 启动失败 —— 检查 CAN_TX_PIN / CAN_RX_PIN");
        while (1) delay(1000);
    }
    d1.mos_temp = 0xFF; d2.mos_temp = 0xFF;
    Serial.println("CAN 已启动 (500kbps)");

    /*---------- 阶段 0: 静默监听 2 秒(不发任何命令) ----------*/
    Serial.println("\n[0] 静默监听 2 秒(不发命令, 只看有没有主动上报) …");
    d1.n_reply = d1.n_report = d2.n_reply = d2.n_report = 0;
    { uint32_t u = millis() + 2000; while ((int32_t)(millis()-u) < 0) { poll_rx(); delay(5); } }
    Serial.printf("  1号: 回码 %lu 帧, 主动上报 %lu 帧\n",
                  (unsigned long)d1.n_reply, (unsigned long)d1.n_report);
    Serial.printf("  2号: 回码 %lu 帧, 主动上报 %lu 帧\n",
                  (unsigned long)d2.n_reply, (unsigned long)d2.n_report);
    Serial.println("  (不发命令时回码通常是 0, 只有 E603 主动上报会来 —— 收到就说明驱动器活着)");

    /*---------- 阶段 1: 通信检查(值=0, 不会动) ----------*/
    Serial.println("\n[1] 通信检查: mode5 值=0, 两个都使能 (不会动)");
    d1.n_reply = d2.n_reply = 0;
    { uint32_t u = millis() + 2000;
      while ((int32_t)(millis()-u) < 0) {
        send_one(ID_M1_CMD, MODE_SPEED, CTRL_ENABLE, 0);
        send_one(ID_M2_CMD, MODE_SPEED, CTRL_ENABLE, 0);
        poll_rx(); delay(CMD_PERIOD_MS - 1);
      } }
    Serial.printf("  1号: 回码 %lu 帧  模式=%u 故障=%u  MOS温度=%d\n",
                  (unsigned long)d1.n_reply, (unsigned)d1.mode, (unsigned)d1.fault,
                  (d1.mos_temp == 0xFF) ? -1 : (int)d1.mos_temp);
    Serial.printf("  2号: 回码 %lu 帧  模式=%u 故障=%u  MOS温度=%d\n",
                  (unsigned long)d2.n_reply, (unsigned)d2.mode, (unsigned)d2.fault,
                  (d2.mos_temp == 0xFF) ? -1 : (int)d2.mos_temp);
    Serial.println("  (MOS温度 -1 = 没收到 E603 上报帧; 单位 ℃, 厂家说超过 100℃ 会保护停机)");
    if (d1.n_reply == 0 || d2.n_reply == 0)
        Serial.println("  ★★ 有驱动器收不到回码 —— 先解决 CAN/驱动器供电, 下面的运动测试不必做");
    { uint32_t u = millis() + COOLDOWN_MS; while ((int32_t)(millis()-u) < 0) { poll_serial(); if (g_stop) break; delay(10); } }

    /*---------- 阶段 2: 单轮低速 15 RPM ----------*/
    if (!g_stop) run_one("2a 只测 1号(左), 2号刹车", ID_M1_CMD, LOW_RPM, &d1, ID_M2_CMD, &d2);
    { uint32_t u = millis() + COOLDOWN_MS; while ((int32_t)(millis()-u) < 0) { poll_serial(); if (g_stop) break; delay(10); } }
    if (!g_stop) run_one("2b 只测 2号(右), 1号刹车", ID_M2_CMD, LOW_RPM, &d2, ID_M1_CMD, &d1);

    /*---------- 阶段 3: 稍微高一点(30 RPM), 只有前一步没自停才做 ----------*/
    { uint32_t u = millis() + COOLDOWN_MS; while ((int32_t)(millis()-u) < 0) { poll_serial(); if (g_stop) break; delay(10); } }
    if (!g_stop && !d1.aborted) run_one("3a 只测 1号(左) @30RPM", ID_M1_CMD, HIGH_RPM, &d1, ID_M2_CMD, &d2);
    { uint32_t u = millis() + COOLDOWN_MS; while ((int32_t)(millis()-u) < 0) { poll_serial(); if (g_stop) break; delay(10); } }
    if (!g_stop && !d2.aborted) run_one("3b 只测 2号(右) @30RPM", ID_M2_CMD, HIGH_RPM, &d2, ID_M1_CMD, &d1);

    /*---------- 汇总 ----------*/
    Serial.println();
    Serial.println("================= 诊断汇总 =================");
    Serial.println(" (目标 15/30 RPM, 150mm 轮; 架空时电流应该 < 1.0A)");
    report_line("1号(左)", &d1, "最后一项");
    report_line("2号(右)", &d2, "最后一项");
    Serial.println();
    Serial.println(" 怎么判读:");
    Serial.println("   转速跟得上目标 + 电流 < 1.0A           -> 电机+驱动器都好 ✓");
    Serial.println("   有回码但转速明显跟不上(如只到 3 RPM)    -> 电机侧问题(堵转/绕组/机械)");
    Serial.println("   **电流 > 2.0A 而转速 ≈ 0 (本程序会自停)** -> ★ 电机堵转 或 驱动器输出异常");
    Serial.println("   转速回码剧烈跳动                        -> 霍尔/编码器异常");
    Serial.println("   收不到回码                              -> 驱动器或 CAN 问题");
    Serial.println("   故障码非 0                              -> 查 docs/foc-can-protocol.md 故障表");
    Serial.println("============================================");
    Serial.println("已全部结束, 持续刹车中。");
}

void loop()
{
    /* 结束后保持刹车, 不再动 */
    send_one(ID_M1_CMD, MODE_SPEED, CTRL_BRAKE, 0);
    send_one(ID_M2_CMD, MODE_SPEED, CTRL_BRAKE, 0);
    poll_rx();
    delay(CMD_PERIOD_MS - 1);
}
