/*============================================================================
 *  ESP32-S3 驱动器**串口**探活 —— 独立于 CAN 的另一条路
 *
 *  【为什么做这个】CAN 那边两个主控(ESP32 和 STM32)都读到"驱动器完全不回应":
 *     STM32: can_err=255 饱和, 驱动器未就绪, 诊断帧里连 bat= 都没有(说明 E603 也没来)
 *     ESP32: 发送 346 次失败, 总线错误 15993, 一次 ACK 都没有
 *   但驱动器**同时支持串口**(厂商文档 2.2 节)。所以:
 *     串口能通 -> 驱动器活着, 是 CAN 那一侧/驱动器 CAN 部分的问题
 *     串口也不通 -> 驱动器没电 / 没接 / 已损坏
 *
 *  ★★ 这个固件**默认只听, 什么都不发** —— 零风险。
 *     驱动器有电就会每 ~5 秒主动发一次 E603 上报帧; 听到就说明它活着。
 *
 *  【接线 —— 和 CAN 收发器相反, 串口要**交叉**】
 *      ESP32-S3             驱动器串口
 *      GPIO17 (TX) ──────> RX      ★ 第一阶段**不用接这根**
 *      GPIO18 (RX) <────── TX
 *      GND         ──────── GND
 *
 *  ⚠️⚠️ 电平!! 先确认驱动器的串口是:
 *        TTL(3.3V/5V, 排针 TX/RX)  -> 可直接接(5V 的 TX 建议串 1k 电阻)
 *        RS232(±12V, DB9 口)       -> **会烧 ESP32! 必须先加 MAX3232 转换**
 *        RS485(A/B 差分)           -> 需要 485 收发器
 *
 *  【串口协议】115200 8-N-1, 帧 = FA + 4字节ID + 8字节数据 + AF (共 14 字节)
 *        ID: 1号电机 11 01 E6 00 | 2号 12 01 E6 00
 *        回码 ID: 末字节 01 | 定时上报(E603): 末字节 03
 *        8 字节数据和 CAN 完全一样:
 *          [0]=模式 01电压/04力矩/05速度/06位置
 *          [1]=控制 01使能/02失能/03刹车/0F复位
 *          [2..3]=设定值(大端 int16)
 *          [4][5]=加/减速间隔  [6]=低压保护  [7]=预留
 *
 *  【用法】烧进去, 串口监视器 115200。它会:
 *     [1] 只听 10 秒 -> 打印收到的每一帧(谁发的/内容), 并统计
 *     [2] 若听到了 E603 -> 说明驱动器活着, 然后才做一次极轻的通信检查
 *     [3] 若一帧都没听到 -> 明确告诉你"驱动器没在工作"
 *==========================================================================*/

#include <Arduino.h>

/*--------------------------- 按你的接线改 ---------------------------*/
/* ★ 默认用 GPIO4/5 —— 和之前 CAN 测试用的是同一对脚, 所以你的板子上一定有。
   如果想换, ESP32-S3 上**安全可用**的脚(能通过 GPIO 矩阵任意路由):
       1~18, 21, 38, 39, 40, 41, 42, 47, 48
   ❌ 避开: 0/3/45/46 (strapping 脚, 影响启动)
            19/20     (USB D-/D+, 用板载 USB 看串口时会断)
            26~37     (SPI Flash / 八线 PSRAM; 你这块是 8MB 内嵌 PSRAM, 这些多半被占) */
#define UART_TX_PIN   4         /* ★ 第一阶段不用接它 */
#define UART_RX_PIN   5         /* ★ 驱动器的 TX 接这个脚 */
#define UART_BAUD     115200
#define LISTEN_MS     10000     /* 只听多久(驱动器 E603 默认约 5 秒一次) */
/*-------------------------------------------------------------------*/

static uint8_t rxbuf[512];
static size_t  rxlen = 0;

/* 统计 */
static uint32_t n_frame = 0;
static uint32_t n_m1_reply = 0, n_m2_reply = 0;
static uint32_t n_m1_report = 0, n_m2_report = 0, n_other = 0;
static uint8_t  last_m1[8], last_m2[8];
static bool     have_m1 = false, have_m2 = false;

static const char *mode_name(uint8_t m)
{
    if (m == 1) return "电压";
    if (m == 4) return "力矩";
    if (m == 5) return "速度";
    if (m == 6) return "位置";
    return "?";
}

/* 试着从缓冲区里找一个 FA ... AF 的 14 字节帧 */
static void parse_frames(void)
{
    size_t i = 0;
    while ((rxlen - i) >= 14) {
        if (rxbuf[i] != 0xFA) { i++; continue; }
        if (rxbuf[i + 13] != 0xAF) { i++; continue; }

        const uint8_t *f = &rxbuf[i + 1];     /* 4 字节 ID + 8 字节数据 */
        uint32_t id = ((uint32_t)f[0] << 24) | ((uint32_t)f[1] << 16) |
                      ((uint32_t)f[2] << 8)  |  (uint32_t)f[3];
        const uint8_t *d = &f[4];

        n_frame++;
        Serial.printf("  帧#%lu  ID=%02X%02X%02X%02X  数据: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      (unsigned long)n_frame, f[0], f[1], f[2], f[3],
                      d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

        if (id == 0x1101E601UL) {
            n_m1_reply++;
            memcpy(last_m1, d, 8); have_m1 = true;
            Serial.printf("     ^^ 1号电机**控制回码** 模式=%u(%s) 故障=%u 实际RPM=%d 电流=%d(0.1A)\n",
                          d[0], mode_name(d[0]), d[1],
                          (int)(int16_t)(((uint16_t)d[2] << 8) | d[3]),
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]));
        } else if (id == 0x1201E601UL) {
            n_m2_reply++;
            memcpy(last_m2, d, 8); have_m2 = true;
            Serial.printf("     ^^ 2号电机**控制回码** 模式=%u(%s) 故障=%u 实际RPM=%d 电流=%d(0.1A)\n",
                          d[0], mode_name(d[0]), d[1],
                          (int)(int16_t)(((uint16_t)d[2] << 8) | d[3]),
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]));
        } else if (id == 0x1101E603UL) {
            n_m1_report++;
            Serial.printf("     ^^ 1号电机**定时上报(E603)** 版本%u.%u 故障=%u MOS温度=%u℃ 电源=%d(Vx10) 转速=%d\n",
                          d[0], d[1], d[2], d[3],
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]),
                          (int)(int16_t)(((uint16_t)d[6] << 8) | d[7]));
        } else if (id == 0x1201E603UL) {
            n_m2_report++;
            Serial.printf("     ^^ 2号电机**定时上报(E603)** 版本%u.%u 故障=%u MOS温度=%u℃ 电源=%d(Vx10) 转速=%d\n",
                          d[0], d[1], d[2], d[3],
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]),
                          (int)(int16_t)(((uint16_t)d[6] << 8) | d[7]));
        } else {
            n_other++;
        }
        i += 14;
    }
    /* 把已经处理掉的移出缓冲 */
    if (i > 0) {
        size_t left = rxlen - i;
        if (left) memmove(rxbuf, rxbuf + i, left);
        rxlen = left;
    }
    if (rxlen > sizeof(rxbuf) - 64) rxlen = 0;   /* 保险: 垃圾太多就清掉 */
}

static void listen(uint32_t ms, const char *tag)
{
    Serial.printf("  [%s] 听 %lu ms …\n", tag, (unsigned long)ms);
    uint32_t t0 = millis();
    while ((int32_t)(millis() - t0) < (int32_t)ms) {
        while (Serial1.available() && rxlen < sizeof(rxbuf))
            rxbuf[rxlen++] = (uint8_t)Serial1.read();
        /* 顺便把原始字节也打出来, 免得帧格式和我们想的不一样时什么都看不到 */
        static size_t shown = 0;
        while (shown < rxlen) {
            Serial.printf("%02X ", rxbuf[shown]);
            if ((++shown % 16) == 0) Serial.println();
        }
        parse_frames();
        delay(5);
    }
    if (rxlen) { Serial.println(); }
    Serial.printf("  [%s] 结束: 累计收到 %lu 帧\n", tag, (unsigned long)n_frame);
}

void setup()
{
    Serial.begin(115200);            /* USB 调试口 */
    delay(1500);
    Serial.println();
    Serial.println("========================================================");
    Serial.println(" ESP32-S3 驱动器**串口**探活 (默认只听, 什么都不发)");
    Serial.println(" 接线: GPIO18(RX) <- 驱动器TX, GND <- GND   [先别接 GPIO17]");
    Serial.println(" ⚠️ 先确认电平! RS232(±12V, DB9口) 会烧 ESP32, 必须加 MAX3232");
    Serial.println(" 参数: 115200 8-N-1, 帧 = FA + 4字节ID + 8字节数据 + AF");
    Serial.println("========================================================");

    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("UART1 已打开: RX=%d TX=%d @%d\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD);

    Serial.println("\n[1] 只听 10 秒(不发任何字节) —— 驱动器有电的话会发 E603 上报");
    listen(LISTEN_MS, "只听");

    Serial.println();
    Serial.println("================= 结果 =================");
    Serial.printf(" 总帧数 %lu | 1号: 回码%lu 上报%lu | 2号: 回码%lu 上报%lu | 其它%lu\n",
                  (unsigned long)n_frame, (unsigned long)n_m1_reply, (unsigned long)n_m1_report,
                  (unsigned long)n_m2_reply, (unsigned long)n_m2_report, (unsigned long)n_other);

    if (n_m1_report + n_m2_report > 0) {
        Serial.println(" ✓ **收到驱动器主动上报 -> 驱动器活着!** 串口这条路通了。");
        Serial.println("   (那么问题只在 CAN 那一侧, 或者驱动器里 CAN 的配置/接线)");
    } else if (n_frame > 0) {
        Serial.println(" ? 收到了帧, 但 ID 不是我们预期的 —— 看上面的原始 hex 确认协议");
    } else {
        Serial.println(" ✗ **一帧都没收到。** 逐一排查:");
        Serial.println("    1) 驱动器有电吗(指示灯)?");
        Serial.println("    2) TX/RX 有没有接反?(要交叉: ESP32 RX <- 驱动器 TX)");
        Serial.println("    3) 电平对不对? RS232 需要 MAX3232");
        Serial.println("    4) 波特率是 115200 吗?");
        Serial.println("    5) 共地了吗(GND 必须接)?");
    }
    Serial.println("========================================");
}

void loop()
{
    /* 继续保持只听, 每收到就打印; 什么都不发 */
    while (Serial1.available() && rxlen < sizeof(rxbuf))
        rxbuf[rxlen++] = (uint8_t)Serial1.read();
    parse_frames();
    delay(5);
}
