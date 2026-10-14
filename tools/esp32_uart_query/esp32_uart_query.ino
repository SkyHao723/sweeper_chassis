/*============================================================================
 *  ESP32-S3 驱动器**串口**主动查询 —— 发一帧、看它回不回
 *
 *  【背景】上一版("只听")在用户那边收不到任何东西 —— 驱动器**不主动上报**
 *    (厂商文档说 E603 默认 0.2Hz 上报, 但实测没有)。所以改成**主动发**。
 *
 *  【安全】★ 这个固件**只发两种帧, 都不会产生运动**:
 *       ① 速度模式 + 使能 + 值=0   -> 目标转速 0 = 不动
 *       ② 刹车帧 (控制字 03)
 *     绝对不会发任何非零设定值, 所以电机**不可能转**。
 *
 *  【接线 —— 串口要**交叉**, 和 CAN 收发器相反】
 *      ESP32-S3             驱动器串口
 *      GPIO4 (TX) ──────> RX        ← 要发就必须接这根
 *      GPIO5 (RX) <────── TX
 *      GND        ──────── GND
 *    ⚠️⚠️ 电平: TTL(3.3/5V 排针) 可直连; **RS232(±12V, DB9) 会烧 ESP32, 必须加 MAX3232**
 *
 *  【帧格式】115200 8-N-1, 最小间隔 5ms
 *      FA + 4字节ID + 8字节数据 + AF   (共 14 字节)
 *      ID: 1号 = 11 01 E6 00   2号 = 12 01 E6 00
 *      回码 ID 末字节 01, 定时上报末字节 03
 *      8 字节数据: [0]模式 [1]控制 [2..3]设定值(大端) [4][5]加减速间隔 [6]低压保护 [7]预留
 *
 *  【做什么】
 *      每 100ms 给两个电机各发一帧 "速度模式+使能+值0"
 *      同时把收到的**每一个字节**打出来(hex)并尝试解出 FA...AF 帧
 *      每 2 秒报一次累计统计
 *      串口发 'x' 停止发送并持续发刹车
 *==========================================================================*/

#include <Arduino.h>

/*--------------------------- 按你的接线改 ---------------------------*/
#define UART_TX_PIN   4
#define UART_RX_PIN   5
#define UART_BAUD     115200
#define SEND_PERIOD   100       /* 发帧间隔 ms (厂商要求 >=5ms) */
/*-------------------------------------------------------------------*/

#define MODE_SPEED    0x05
#define CTRL_ENABLE   0x01
#define CTRL_BRAKE    0x03

static uint8_t rxbuf[1024];
static size_t  rxlen = 0;
static uint32_t tx_count = 0, tx_bytes = 0;
static uint32_t rx_bytes = 0, n_frame = 0;
static uint32_t n_m1_reply = 0, n_m2_reply = 0, n_m1_rep = 0, n_m2_rep = 0, n_other = 0;
static bool     g_send = true;
static bool     printed_any = false;

static const char *mode_name(uint8_t m)
{
    if (m == 1) return "电压";
    if (m == 4) return "力矩";
    if (m == 5) return "速度";
    if (m == 6) return "位置";
    return "?";
}

/* 发一帧: FA + 4字节ID + 8字节数据 + AF */
static void send_frame(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t value)
{
    uint8_t f[14];
    f[0] = 0xFA;
    f[1] = (uint8_t)(id >> 24); f[2] = (uint8_t)(id >> 16);
    f[3] = (uint8_t)(id >> 8);  f[4] = (uint8_t)id;
    f[5] = mode; f[6] = ctrl;
    f[7] = (uint8_t)(((uint16_t)value) >> 8);
    f[8] = (uint8_t)(((uint16_t)value) & 0xFF);
    f[9] = 0;      /* 加速间隔 Δt */
    f[10] = 0;     /* 减速间隔 Δt */
    f[11] = 0;     /* 低压保护 */
    f[12] = 0;     /* 预留 */
    f[13] = 0xAF;
    Serial1.write(f, 14);
    Serial1.flush();
    tx_count++; tx_bytes += 14;

    /* 把发出去的也打出来(方便对照) */
    static uint32_t shown = 0;
    if (shown < 3) {                    /* 只打前 3 次, 免得刷屏 */
        shown++;
        Serial.print("  发: ");
        for (int i = 0; i < 14; i++) Serial.printf("%02X ", f[i]);
        Serial.println();
    }
}

static void parse_frames(void)
{
    size_t i = 0;
    while ((rxlen - i) >= 14) {
        if (rxbuf[i] != 0xFA || rxbuf[i + 13] != 0xAF) { i++; continue; }
        uint32_t id = ((uint32_t)rxbuf[i+1] << 24) | ((uint32_t)rxbuf[i+2] << 16) |
                      ((uint32_t)rxbuf[i+3] << 8)  |  (uint32_t)rxbuf[i+4];
        const uint8_t *d = &rxbuf[i + 5];
        n_frame++;
        Serial.printf("  ★收到帧#%lu ID=%02X%02X%02X%02X 数据 %02X %02X %02X %02X %02X %02X %02X %02X\n",
                      (unsigned long)n_frame, rxbuf[i+1], rxbuf[i+2], rxbuf[i+3], rxbuf[i+4],
                      d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
        if (id == 0x1101E601UL || id == 0x1201E601UL) {
            if (id == 0x1101E601UL) n_m1_reply++; else n_m2_reply++;
            Serial.printf("     ^^ %s 控制回码: 模式=%u(%s) 故障=%u 实际RPM=%d 电流=%d(0.1A) 位置=%d\n",
                          (id == 0x1101E601UL) ? "1号" : "2号",
                          d[0], mode_name(d[0]), d[1],
                          (int)(int16_t)(((uint16_t)d[2] << 8) | d[3]),
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]),
                          (int)(int16_t)(((uint16_t)d[6] << 8) | d[7]));
        } else if (id == 0x1101E603UL || id == 0x1201E603UL) {
            if (id == 0x1101E603UL) n_m1_rep++; else n_m2_rep++;
            Serial.printf("     ^^ %s 定时上报: 版本%u.%u 故障=%u MOS温度=%u℃ 电源=%d(Vx10) 转速=%d\n",
                          (id == 0x1101E603UL) ? "1号" : "2号",
                          d[0], d[1], d[2], d[3],
                          (int)(int16_t)(((uint16_t)d[4] << 8) | d[5]),
                          (int)(int16_t)(((uint16_t)d[6] << 8) | d[7]));
        } else {
            n_other++;
        }
        i += 14;
    }
    if (i > 0) {
        size_t left = rxlen - i;
        if (left) memmove(rxbuf, rxbuf + i, left);
        rxlen = left;
    }
    if (rxlen > sizeof(rxbuf) - 128) rxlen = 0;
}

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println();
    Serial.println("========================================================");
    Serial.println(" ESP32-S3 驱动器**串口**主动查询");
    Serial.println(" ★ 只发「速度模式+使能+值0」和「刹车」, 电机不可能转");
    Serial.println(" 接线(交叉): GPIO4(TX)->驱动器RX, GPIO5(RX)<-驱动器TX, GND-GND");
    Serial.println(" ⚠️ 电平: RS232(DB9, ±12V) 会烧 ESP32, 必须加 MAX3232");
    Serial.println(" 帧 = FA + 4字节ID + 8字节数据 + AF, 115200 8-N-1");
    Serial.println("========================================================");

    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(100);
    while (Serial1.available()) Serial1.read();      /* 清掉开机垃圾 */
    Serial.printf("UART1 已打开: TX=%d RX=%d @%d\n", UART_TX_PIN, UART_RX_PIN, UART_BAUD);
    Serial.println("开始每 100ms 发一帧给两个电机(值=0, 不会动)…");
    Serial.println("★ 串口发 'x' 可停止发送");
}

void loop()
{
    static uint32_t last_send = 0, last_stat = 0;

    /* 收。★ 原始字节**收到就立刻打**, 不要事后从 rxbuf 里按索引取 ——
       parse_frames() 会 memmove 移位缓冲区, 那样索引就错位打出乱码了。 */
    while (Serial1.available() && rxlen < sizeof(rxbuf)) {
        uint8_t b = (uint8_t)Serial1.read();
        rxbuf[rxlen++] = b;
        rx_bytes++;
        if (!printed_any) {
            Serial.println("\n★ 收到第一个字节了! 原始流(每 16 字节一行):");
            printed_any = true;
        }
        Serial.printf("%02X ", b);
        if ((rx_bytes % 16) == 0) Serial.println();
    }
    parse_frames();

    /* 发 */
    uint32_t now = millis();
    if (g_send && (now - last_send) >= SEND_PERIOD) {
        last_send = now;
        send_frame(0x1101E600UL, MODE_SPEED, CTRL_ENABLE, 0);
        send_frame(0x1201E600UL, MODE_SPEED, CTRL_ENABLE, 0);
    }
    if (!g_send && (now - last_send) >= SEND_PERIOD) {
        last_send = now;
        send_frame(0x1101E600UL, MODE_SPEED, CTRL_BRAKE, 0);
        send_frame(0x1201E600UL, MODE_SPEED, CTRL_BRAKE, 0);
    }

    /* 统计 */
    if ((now - last_stat) >= 2000) {
        last_stat = now;
        Serial.printf("\n[统计] 发送 %lu 次(%lu 字节) | 收到 %lu 字节, 解出 %lu 帧"
                      " | 1号 回码%lu 上报%lu | 2号 回码%lu 上报%lu | 其它%lu\n",
                      (unsigned long)tx_count, (unsigned long)tx_bytes,
                      (unsigned long)rx_bytes, (unsigned long)n_frame,
                      (unsigned long)n_m1_reply, (unsigned long)n_m1_rep,
                      (unsigned long)n_m2_reply, (unsigned long)n_m2_rep,
                      (unsigned long)n_other);
        if (rx_bytes == 0 && tx_count > 10)
            Serial.println("  ★ 发了不少但一个字节都没收到 —— 查: TX 线接了吗 / TX-RX 是否接反 / 电平 / 波特率 / 共地");
    }

    /* 串口指令 */
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'x' || c == 'X') {
            g_send = false;
            Serial.println("★★ 收到 x -> 停止发送, 改发刹车帧");
        }
    }
    delay(2);
}
