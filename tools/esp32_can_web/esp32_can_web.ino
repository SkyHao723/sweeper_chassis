/*============================================================================
 *  ESP32-S3 CAN 调试网页 —— 手动调力矩电流 / 速度
 *
 *  【为什么要有它】力矩模式是**纯扭矩、没有任何速度限制**(架空实测 2.0A 就冲到 489RPM),
 *    而厂商确认**力矩模式上限 10.0A**。要搞清楚"这台车到底需要多少电流才推得动",
 *    最直接的办法就是**手动一点点加, 同时看轮子**。这个网页就是干这个的。
 *
 *  【用法】
 *    1. 上电后 ESP32 开一个热点:  SSID = ESP32-CAN   密码 = 12345678
 *    2. 手机/电脑连上它, 浏览器打开  http://192.168.4.1
 *    3. 网页上: 选模式(力矩/速度) -> 拖滑块设值 -> 按"运行"
 *       ★ "运行"期间网页每 200ms 会把当前值发一次; 关掉网页/断网 -> 1 秒后自动刹车
 *    4. 大红色"停止并刹车"按钮随时拍
 *
 *  【三道安全闸, 都不是可选项】
 *    ① 值域硬夹: 力矩 <= 100 (10.0A, 厂商上限)  速度 <= 500
 *    ② 超速保护: 任一实际转速 |RPM| > RPM_ABORT -> 立刻刹车并锁存, 网页上红字报告
 *       (力矩模式没有速度闭环, 这是唯一的兜底)
 *    ③ 看门狗: 1 秒收不到网页的心跳 -> 刹车
 *
 *  【接线】同 esp32_can_test: ESP32-S3 --(外接 CAN 收发器)-- 底盘 CAN_H/CAN_L
 *    ★★ STM32 必须断电或拔掉它的 CAN! 两个主控同时发会抢总线。
 *    ★★ CAN_TX_PIN / CAN_RX_PIN 改成你实际接的脚。
 *==========================================================================*/

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"

/*--------------------------- 配置 ---------------------------*/
#define CAN_TX_PIN      GPIO_NUM_4      /* ★ 改成你接的 TX 脚 */
#define CAN_RX_PIN      GPIO_NUM_5      /* ★ 改成你接的 RX 脚 */
#define AP_SSID         "ESP32-CAN"
#define AP_PASS         "12345678"      /* 至少 8 位 */
/*------------------------------------------------------------*/

#define ID_M1_CMD       0x1101E600UL
#define ID_M2_CMD       0x1201E600UL
#define ID_M1_REPLY     0x1101E601UL
#define ID_M2_REPLY     0x1201E601UL

#define MODE_TORQUE     0x04
#define MODE_SPEED      0x05
#define CTRL_ENABLE     0x01
#define CTRL_BRAKE      0x03

/* ---- 三道安全闸 ---- */
#define MAX_A10         100         /* ★ 厂商确认: 力矩模式上限 10.0A */
#define MAX_RPM_CMD     500         /* 速度模式量程 */
#define RPM_ABORT       150         /* ★ 超速保护: 超过就刹车锁存(力矩模式唯一的兜底) */
#define CMD_TIMEOUT_MS  1000        /* ★ 看门狗: 这么久没心跳就刹车 */

#define CMD_PERIOD_MS   20

typedef struct { uint8_t mode, fault; int16_t rpm, cur; uint32_t n; } drv_state;
static drv_state m1, m2;

static int  g_mode   = MODE_TORQUE;   /* 默认力矩(方便手动加电流) */
static int  g_val    = 0;             /* A×10 或 RPM */
static bool g_run    = false;
static bool g_tripped = false;        /* 超速锁存 */
static int  g_trip_rpm = 0;
static unsigned long g_last_cmd_ms = 0;

WebServer server(80);

/*======================= CAN =======================*/
static void send_cmd(uint32_t id, uint8_t mode, uint8_t ctrl, int16_t value)
{
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id; m.extd = 1; m.data_length_code = 8;
    m.data[0] = mode;
    m.data[1] = ctrl;
    m.data[2] = (uint8_t)(((uint16_t)value) >> 8);
    m.data[3] = (uint8_t)((uint16_t)value & 0xFF);
    m.data[4] = 0; m.data[5] = 0; m.data[6] = 0; m.data[7] = 0;
    twai_transmit(&m, pdMS_TO_TICKS(5));
}

static void poll_rx(void)
{
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd || m.data_length_code < 4) continue;
        drv_state *s = NULL;
        if      (m.identifier == ID_M1_REPLY) s = &m1;
        else if (m.identifier == ID_M2_REPLY) s = &m2;
        else continue;
        s->mode  = m.data[0];
        s->fault = m.data[1];
        s->rpm   = (int16_t)(((uint16_t)m.data[2] << 8) | m.data[3]);
        if (m.data_length_code >= 6)
            s->cur = (int16_t)(((uint16_t)m.data[4] << 8) | m.data[5]);
        s->n++;
    }
}

/* 当前该发什么: 锁存/没运行/超时 -> 刹车 */
static void service_can(void)
{
    int16_t v;
    uint8_t mode;

    if (g_tripped) {
        send_cmd(ID_M1_CMD, MODE_SPEED, CTRL_BRAKE, 0);
        send_cmd(ID_M2_CMD, MODE_SPEED, CTRL_BRAKE, 0);
        return;
    }
    if (!g_run || (millis() - g_last_cmd_ms) > CMD_TIMEOUT_MS) {
        send_cmd(ID_M1_CMD, MODE_SPEED, CTRL_BRAKE, 0);
        send_cmd(ID_M2_CMD, MODE_SPEED, CTRL_BRAKE, 0);
        return;
    }
    mode = (uint8_t)g_mode;
    v    = (int16_t)g_val;
    if (mode == MODE_TORQUE && v >  MAX_A10)     v =  MAX_A10;
    if (mode == MODE_TORQUE && v < -MAX_A10)     v = -MAX_A10;
    if (mode == MODE_SPEED  && v >  MAX_RPM_CMD) v =  MAX_RPM_CMD;
    if (mode == MODE_SPEED  && v < -MAX_RPM_CMD) v = -MAX_RPM_CMD;
    send_cmd(ID_M1_CMD, mode, CTRL_ENABLE, v);
    send_cmd(ID_M2_CMD, mode, CTRL_ENABLE, v);
}

/* 超速保护 */
static void check_abort(void)
{
    int a1 = (m1.rpm < 0) ? -m1.rpm : m1.rpm;
    int a2 = (m2.rpm < 0) ? -m2.rpm : m2.rpm;
    if (a1 > RPM_ABORT || a2 > RPM_ABORT) {
        g_tripped = true;
        g_run     = false;
        g_trip_rpm = (a1 > a2) ? a1 : a2;
    }
}

/*======================= 网页 =======================*/
static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN 调试</title><style>
body{font-family:system-ui,-apple-system,sans-serif;margin:0;padding:16px;background:#111;color:#eee}
h2{margin:.2em 0 .6em}
.card{background:#1c1c1e;border-radius:10px;padding:14px;margin-bottom:12px}
input[type=range]{width:100%;height:34px}
button{font-size:16px;padding:10px 16px;border:0;border-radius:8px;margin:4px 6px 4px 0;color:#fff}
.b{background:#0a84ff}.g{background:#30d158}.o{background:#ff9f0a}
.r{background:#ff3b30;font-weight:700;font-size:18px;padding:14px 22px}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:5px 4px;text-align:left;border-bottom:1px solid #333}
.big{font-size:30px;font-weight:700;color:#30d158}
.warn{color:#ff453a;font-weight:700}
#trip{display:none;background:#ff3b30;color:#fff;padding:12px;border-radius:8px;margin-bottom:12px;font-weight:700}
</style></head><body>
<h2>CAN 底盘调试</h2>
<div id="trip">⚠ 超速保护已触发 —— 只发刹车。确认车安全后按「解除锁存」继续。</div>

<div class="card">
  <div>模式&nbsp;
    <button id="bm4" class="b" onclick="setMode(4)">力矩 mode4</button>
    <button id="bm5" class="o" onclick="setMode(5)">速度 mode5</button>
  </div>
  <div style="margin-top:10px">设定值 <span class="big" id="v">0</span>
    <span id="unit" style="color:#888">(0.1A)</span></div>
  <input type="range" id="sl" min="0" max="100" value="0" oninput="onSlide()">
  <div style="color:#888;font-size:13px" id="hint">力矩: 0~10.0A &nbsp;(1 格 = 0.1A)</div>
  <div style="margin-top:8px">
    <button class="g" onclick="run(true)">▶ 运行</button>
    <button class="b" onclick="run(false)">■ 停(保持刹车)</button>
    <button class="r" onclick="stopAll()">停止并刹车</button>
    <button class="o" onclick="unlock()">解除锁存</button>
  </div>
</div>

<div class="card">
  <table><tr><th></th><th>模式</th><th>故障</th><th>实际RPM</th><th>电流A</th></tr>
  <tr><td>1号</td><td id="m1m">-</td><td id="m1f">-</td><td id="m1r">-</td><td id="m1c">-</td></tr>
  <tr><td>2号</td><td id="m2m">-</td><td id="m2f">-</td><td id="m2r">-</td><td id="m2c">-</td></tr></table>
  <div style="color:#888;font-size:13px;margin-top:8px">回码帧数: 1号 <span id="m1n">0</span> / 2号 <span id="m2n">0</span>
    &nbsp;|&nbsp; 状态: <span id="st">待机</span></div>
</div>
<script>
var mode=4, running=false;
var MAX=[0,0,0,0,100,500];   // 索引=模式
function nm(m){return m==4?'力矩':m==5?'速度':m==1?'电压':m==6?'位置':'?'}
function fmt(){
  var sl=document.getElementById('sl');
  sl.max = MAX[mode];
  document.getElementById('v').textContent = mode==4 ? (sl.value/10).toFixed(1) : sl.value;
  document.getElementById('unit').textContent = mode==4 ? 'A' : 'RPM';
  document.getElementById('hint').textContent = mode==4
     ? '力矩: 0~10.0A (1 格 = 0.1A)  ★厂商上限 10.0A'
     : '速度: 0~500 RPM';
  document.getElementById('bm4').style.background = mode==4?'#0a84ff':'#333';
  document.getElementById('bm5').style.background = mode==5?'#ff9f0a':'#333';
}
function setMode(m){mode=m; if(m==4)document.getElementById('sl').value=0; fmt(); if(running)push();}
function onSlide(){fmt(); if(running)push();}
function run(on){running=on; if(on)push();}
function stopAll(){running=false; fetch('/stop');}
function unlock(){fetch('/unlock');}
function push(){
  fetch('/set?m='+mode+'&v='+document.getElementById('sl').value);
}
setInterval(function(){
  /* ★ 心跳: 运行期间每 200ms 喂一次 /set。
     关掉网页/断网 -> 不再喂 -> 固件 1 秒后自动刹车(第三道安全闸)。 */
  if(running) push();
  fetch('/status').then(function(r){return r.json()}).then(function(d){
    document.getElementById('m1m').textContent=nm(d.m1.mode);
    document.getElementById('m1f').textContent=d.m1.fault;
    document.getElementById('m1r').textContent=d.m1.rpm;
    document.getElementById('m1c').textContent=(d.m1.cur/10).toFixed(1);
    document.getElementById('m2m').textContent=nm(d.m2.mode);
    document.getElementById('m2f').textContent=d.m2.fault;
    document.getElementById('m2r').textContent=d.m2.rpm;
    document.getElementById('m2c').textContent=(d.m2.cur/10).toFixed(1);
    document.getElementById('m1n').textContent=d.m1.n;
    document.getElementById('m2n').textContent=d.m2.n;
    document.getElementById('st').textContent =
        d.trip ? '★超速锁存' : (d.run ? '运行中' : '待机(刹车)');
    document.getElementById('trip').style.display = d.trip ? 'block':'none';
  }).catch(function(){});
},200);
fmt();
</script></body></html>)HTML";

static void handle_root(void)
{
    server.send_P(200, "text/html; charset=utf-8", PAGE);
}

/* /set?m=<mode>&v=<val>  —— 网页每 200ms 喂一次 */
static void handle_set(void)
{
    if (server.hasArg("m")) g_mode = server.arg("m").toInt();
    if (server.hasArg("v")) {
        int v = server.arg("v").toInt();
        if (v < 0) v = 0;
        if (g_mode == MODE_TORQUE && v > MAX_A10)     v = MAX_A10;
        if (g_mode == MODE_SPEED  && v > MAX_RPM_CMD) v = MAX_RPM_CMD;
        g_val = v;
    }
    g_last_cmd_ms = millis();
    g_run = true;
    server.send(200, "text/plain", "ok");
}

static void handle_stop(void)
{
    g_run = false;
    server.send(200, "text/plain", "stopped");
}

static void handle_unlock(void)
{
    g_tripped = false;
    g_run     = false;
    g_trip_rpm = 0;
    server.send(200, "text/plain", "unlocked");
}

static void handle_status(void)
{
    char b[320];
    snprintf(b, sizeof(b),
        "{\"m1\":{\"mode\":%u,\"fault\":%u,\"rpm\":%d,\"cur\":%d,\"n\":%lu},"
        "\"m2\":{\"mode\":%u,\"fault\":%u,\"rpm\":%d,\"cur\":%d,\"n\":%lu},"
        "\"run\":%d,\"trip\":%d,\"mode\":%d,\"val\":%d}",
        (unsigned)m1.mode, (unsigned)m1.fault, (int)m1.rpm, (int)m1.cur, (unsigned long)m1.n,
        (unsigned)m2.mode, (unsigned)m2.fault, (int)m2.rpm, (int)m2.cur, (unsigned long)m2.n,
        g_run ? 1 : 0, g_tripped ? 1 : 0, g_mode, g_val);
    server.send(200, "application/json", b);
}

/*======================= setup/loop =======================*/
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32-S3 CAN 调试网页 ===");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK)
        Serial.println("CAN 已启动 500kbps");
    else
        Serial.println("★ CAN 启动失败 —— 检查引脚");

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("热点: %s   密码: %s\n", AP_SSID, AP_PASS);
    Serial.printf("浏览器打开: http://%s\n", WiFi.softAPIP().toString().c_str());

    server.on("/",       handle_root);
    server.on("/set",    handle_set);
    server.on("/stop",   handle_stop);
    server.on("/unlock", handle_unlock);
    server.on("/status", handle_status);
    server.begin();
    Serial.println("网页服务已启动");
    Serial.println("★ 力矩模式没有速度闭环, 网页里请一点点加电流, 盯着轮子");
}

void loop()
{
    static unsigned long last_can = 0;

    server.handleClient();
    poll_rx();
    check_abort();

    unsigned long now = millis();
    if ((now - last_can) >= CMD_PERIOD_MS) {
        last_can = now;
        service_can();
    }
}
