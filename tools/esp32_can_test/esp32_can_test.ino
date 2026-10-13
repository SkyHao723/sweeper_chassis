/*============================================================================
 *  ESP32-S3 裸 CAN 测试 —— 力矩模式(mode 4)判决性实验
 *
 *  【为什么要有它】STM32 固件里叠了七八层逻辑:
 *      逆解 -> 起步下限 floor -> PI -> 防打滑 -> 限幅 -> Torque_Guard(三条保护)
 *    所以一直有个可能性排除不掉: "是不是我们自己这些层把命令改坏了"。
 *    (成帧已和厂商参考 .doc\...\pwm_adc.c 逐字节核对一致, 但那只是成帧。)
 *    这个固件**只发原始命令**, 一个字节都不加工 —— 于是:
 *      ESP32 发 12A 车不动  ->  彻底排除我们的固件, 是驱动器/mode 4 的问题
 *      ESP32 发 12A 车能走  ->  我们的固件有问题, 可以逐层加回去定位
 *
 *  【接线】
 *    ESP32-S3 --(外接 CAN 收发器)-- 底盘 CAN_H / CAN_L
 *    ★★ STM32 必须断电或拔掉它的 CAN! 两个主控同时发会抢总线,
 *       而且 STM32 的看门狗会一直发刹车帧。总线两端要有 120Ω 终端电阻。
 *    ★★ CAN_TX_PIN / CAN_RX_PIN 改成你实际接的脚。
 *
 *  【安全】
 *    - 自动跑一遍安全序列, 结束后**持续发刹车**, 不会自己跑掉
 *    - 串口发 'x' 立刻停车
 *    - 电流从小档开始逐步加大
 *    - ★ 车要么架空, 要么前方留足空场
 *
 *  【用法】Arduino IDE, 板选 ESP32S3, 串口监视器 115200
 *    序列: ① 通信检查  ② 对照组 mode5 速度 38RPM  ③ mode4 力矩阶梯 2/4/8/12A
 *    每一步都打印驱动器回码(模式/故障/实际RPM/电流), **不做任何解释**。
 *==========================================================================*/

#include <Arduino.h>
#include "driver/twai.h"

/*--------------------------- 按你的接线改这里 ---------------------------*/
#define CAN_TX_PIN      GPIO_NUM_4      /* ★ 改成你接的 TX 脚 */
#define CAN_RX_PIN      GPIO_NUM_5      /* ★ 改成你接的 RX 脚 */
/*------------------------------------------------------------------------*/

#define ID_M1_CMD       0x1101E600UL
#define ID_M2_CMD       0x1201E600UL
#define ID_M1_REPLY     0x1101E601UL
#define ID_M2_REPLY     0x1201E601UL

#define MODE_TORQUE     0x04
#define MODE_SPEED      0x05
#define CTRL_ENABLE     0x01
#define CTRL_BRAKE      0x03

#define CMD_PERIOD_MS   20          /* 50Hz 重发, 和 STM32 一样 */

typedef struct {
    uint8_t  mode;
    uint8_t  fault;
    int16_t  rpm;
    int16_t  cur_a10;       /* A x10 */
    uint32_t n;             /* 收到多少帧 */
} drv_state;

static drv_state st1;
static drv_state st2;
static bool g_stop = false;     /* 串口 'x' 置位 */
/* ★ g_arm_torque(串口 '4' 解锁力矩档)必须声明在这里 ——
   poll_serial() 在下面几行就要用它, 放到后面会报 "was not declared in this scope"。 */
static bool g_arm_torque = false;

/* ★ 必须在每个阻塞循环里都调 —— 否则 hold() 跑那 3 秒期间按 'x' 停不下来。
   (第一版把串口检查只放在 loop() 里, 那是个安全漏洞。) */
static void poll_serial(void)
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'x' || c == 'X') {
            Serial.println("★★ 收到 x -> 立刻刹车并停止");
            g_stop = true;
        }
        if (c == '4') {
            Serial.println("★★ 收到 4 -> 解锁力矩阶梯(0.5A 起), 3 秒后开始");
            g_arm_torque = true;
        }
    }
}

/*======================= 工具 =======================*/
static void send_cmd(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t value)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier       = id;
    m.extd             = 1;
    m.data_length_code = 8;
    m.data[0] = mode;
    m.data[1] = ctrl;
    m.data[2] = (uint8_t)(((uint16_t)value) >> 8);   /* 高字节在前 */
    m.data[3] = (uint8_t)((uint16_t)value & 0xFF);
    m.data[4] = 0;      /* 加速间隔 Δt: 0 = 直跳(和厂商上位机默认一致) */
    m.data[5] = 0;      /* 减速间隔 Δt */
    m.data[6] = 0;      /* 低压保护: 0 = 用驱动器默认 */
    m.data[7] = 0;
    twai_transmit(&m, pdMS_TO_TICKS(5));
}

static void send_both(uint8_t mode, uint8_t ctrl, int16_t v1, int16_t v2)
{
    send_cmd(ID_M1_CMD, mode, ctrl, v1);
    send_cmd(ID_M2_CMD, mode, ctrl, v2);
}

static void poll_rx(void)
{
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK)
    {
        if (!m.extd || m.data_length_code < 4) continue;
        drv_state *s = NULL;
        if      (m.identifier == ID_M1_REPLY) s = &st1;
        else if (m.identifier == ID_M2_REPLY) s = &st2;
        else continue;
        s->mode  = m.data[0];
        s->fault = m.data[1];
        s->rpm   = (int16_t)(((uint16_t)m.data[2] << 8) | m.data[3]);
        if (m.data_length_code >= 6)
            s->cur_a10 = (int16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
        s->n++;
    }
}

static const char *mode_name(uint8_t m)
{
    if (m == 1) return "电压";
    if (m == 4) return "力矩";
    if (m == 5) return "速度";
    if (m == 6) return "位置";
    return "?";
}

static void dump_row(const char *who, const drv_state *s)
{
    Serial.printf("%s: 模式=%-4s 故障=%3u 实际RPM=%+7d 电流=%+6.2fA 帧=%lu\n",
                  who, mode_name(s->mode), (unsigned)s->fault,
                  (int)s->rpm, s->cur_a10 / 10.0f, (unsigned long)s->n);
}

/*======================= 发一条命令并保持 N 秒 =======================*/
static void hold(uint8_t mode, uint8_t ctrl, int16_t v1, int16_t v2,
                 uint16_t seconds, const char *tag)
{
    uint32_t t0    = millis();
    uint32_t until = t0 + (uint32_t)seconds * 1000UL;
    uint32_t next_print = t0;

    Serial.printf("\n--- %s\n", tag);
    Serial.printf("    发: mode=%u(%s) ctrl=%u  M1值=%d  M2值=%d  持续 %us\n",
                  (unsigned)mode, mode_name(mode), (unsigned)ctrl,
                  (int)v1, (int)v2, (unsigned)seconds);

    while ((int32_t)(millis() - until) < 0)
    {
        poll_serial();
        if (g_stop) break;
        send_both(mode, ctrl, v1, v2);
        poll_rx();

        uint32_t t = millis();
        if ((int32_t)(t - next_print) >= 0) {
            next_print = t + 500;
            Serial.printf("  %5lums  ", (unsigned long)(t - t0));
            Serial.printf("M1(%s %u %+d %+.1fA)  M2(%s %u %+d %+.1fA)\n",
                          mode_name(st1.mode), (unsigned)st1.fault, (int)st1.rpm, st1.cur_a10 / 10.0f,
                          mode_name(st2.mode), (unsigned)st2.fault, (int)st2.rpm, st2.cur_a10 / 10.0f);
        }
        delay(CMD_PERIOD_MS - 1);
    }
}

static void brake_for(uint16_t ms)
{
    uint32_t until = millis() + ms;
    while ((int32_t)(millis() - until) < 0) {
        poll_serial();
        if (g_stop) break;
        send_both(MODE_SPEED, CTRL_BRAKE, 0, 0);
        poll_rx();
        delay(CMD_PERIOD_MS - 1);
    }
}

/*======================= 序列状态机 =======================*/
/* ★ 2026-10-13 改成"地面安全版":
 *   实测**架空**时 mode4 给 2.0A 就把轮子推到 **489 RPM**(3.8 m/s) —— 因为力矩模式是
 *   **纯扭矩、没有任何速度限制**, 空载下会一直加速到"扭矩 = 摩擦"的平衡点。
 *   => 地面上跑 mode4 会冲。所以: ①② 自动跑(② 有速度闭环, 安全);
 *      ③ 力矩阶梯**必须串口按 '4' 才跑**, 而且从 0.5A 起。
 *   ★ 地面测试这个门槛是必须的, 别去掉。 */
enum { PH_COM, PH_SPEED, PH_WAIT, PH_TORQUE, PH_DONE };
#define TORQUE_STEPS  6
static const int16_t g_tq_val[TORQUE_STEPS]  = { 5, 10, 20, 40, 80, 120 };  /* 0.5~12.0A */
static const char   *g_tq_name[TORQUE_STEPS] = { "0.5A", "1.0A", "2.0A", "4.0A", "8.0A", "12.0A" };

static int      g_phase = PH_COM;
static int      g_step  = 0;
/* (g_arm_torque 声明在文件上面 poll_serial 之前 —— 它先用) */
static bool     g_abort = false;

void setup()
{
    Serial.begin(115200);
    delay(1200);
    Serial.println();
    Serial.println("====================================================");
    Serial.println(" ESP32-S3 裸 CAN 力矩模式判决实验");
    Serial.println(" ★ 先确认: STM32 已断电/断开 CAN, 车架空或前方留空");
    Serial.println(" ★ 串口发 'x' 随时停车");
    Serial.println("====================================================");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("★ twai_driver_install 失败 —— 检查引脚");
        while (1) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("★ twai_start 失败");
        while (1) delay(1000);
    }
    Serial.println("CAN 已启动 (500kbps, 扩展帧)。3 秒后开始… (按 x 可取消, 按 4 解锁力矩档)");
    for (int i = 0; i < 30; i++) { poll_serial(); if (g_stop) break; delay(100); }
}

/* 每个阶段跑一次; 返回后由 loop 推进 */
static void run_phase(void)
{
    switch (g_phase)
    {
    case PH_COM:
        st1.n = st2.n = 0;
        hold(MODE_SPEED, CTRL_ENABLE, 0, 0, 2, "① 通信检查 (速度模式, 值=0)");
        Serial.println();
        dump_row("M1", &st1);
        dump_row("M2", &st2);
        if (st1.n == 0 && st2.n == 0) {
            Serial.println("★★ 一个回码都没收到 —— 先解决 CAN 接线/终端电阻/波特率, 后面不用测了");
            g_abort = true;
        } else {
            Serial.println("(回码收到了, 通信通)");
        }
        brake_for(1000);
        g_phase = PH_SPEED;
        break;

    case PH_SPEED:
        /* 对照组: 速度模式 38RPM(150mm 轮 = 0.3m/s) —— 实车这个速度能正常走 */
        hold(MODE_SPEED, CTRL_ENABLE, 38, 38, 4, "② 对照组 mode5 速度 38RPM (实车能走的速度)");
        brake_for(1500);
        g_phase = PH_WAIT;
        break;

    case PH_WAIT:
        /* ★ 停在这里等人按 '4' —— 力矩模式是纯扭矩、无速度限制, 自动跑会冲。
           到这一步 ①② 已经跑完, 你可以先判断"速度模式到底能不能驱动"。 */
        Serial.println();
        Serial.println("========================================================");
        Serial.println(" ①② 已跑完。**力矩阶梯(mode4)不会自动跑** —— 它是纯扭矩、");
        Serial.println(" 没有任何速度限制, 架空实测 2.0A 就冲到 489 RPM。");
        Serial.println(" 确认车已架空或前方留足空场后, 串口发 '4' 才开始(从 0.5A 起)。");
        Serial.println(" 不发就保持刹车。串口 'x' 随时停车。");
        Serial.println("========================================================");
        /* 等 '4' 或 'x'; 期间持续刹车 */
        for (;;) {
            poll_serial();
            if (g_stop || g_abort) { g_phase = PH_DONE; break; }
            if (g_arm_torque)     { g_phase = PH_TORQUE; break; }
            send_both(MODE_SPEED, CTRL_BRAKE, 0, 0);
            poll_rx();
            delay(CMD_PERIOD_MS - 1);
        }
        break;

    case PH_TORQUE:
        {
            if (g_step >= TORQUE_STEPS) { g_phase = PH_DONE; break; }

            char tag[96];
            snprintf(tag, sizeof(tag), "③ 力矩 mode4 阶梯 %s (第 %d/%d 档)",
                     g_tq_name[g_step], g_step + 1, TORQUE_STEPS);
            hold(MODE_TORQUE, CTRL_ENABLE, g_tq_val[g_step], g_tq_val[g_step], 3, tag);
            brake_for(2000);
            g_step++;
        }
        break;

    default:
        break;
    }
}

void loop()
{
    poll_serial();

    if (g_phase != PH_DONE && !g_abort && !g_stop) {
        run_phase();
        return;
    }

    /* 结束: 持续刹车, 不再动 */
    static bool said = false;
    if (!said) {
        said = true;
        Serial.println();
        Serial.println("==== 序列结束, 持续刹车中。按复位键可重跑。 ====");
    }
    send_both(MODE_SPEED, CTRL_BRAKE, 0, 0);
    poll_rx();
    delay(CMD_PERIOD_MS - 1);
}
