#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

/*==========================================================================
 * 上位机替身 —— ESP32 (WiFi + 网页遥控 + OLED)
 *
 * 这个程序扮演 Wheeltec 上位机(likewheeltec_robot_node)的角色:
 *   把网页上的操作打包成 11 字节速度帧发给 STM32, 再读回遥测显示出来。
 *   所有底盘运算(逆解、电机闭环、里程计、卡尔曼)都在 STM32 里。
 *
 *   ESP32 GPIO17 (TX2) -> STM32 PA10 (USART1_RX)
 *   ESP32 GPIO16 (RX2) <- STM32 PA9  (USART1_TX)
 *   ESP32 GND          <-> STM32 GND
 *
 * 【命令帧 11 字节  ESP32 -> STM32】
 *   [0] 0x7B  [1] AutoRecharge  [2] SecurityPLY
 *   [3-4] int16 BE vx*1000 mm/s   [5-6] vy*1000 mm/s   [7-8] wz*1000 mrad/s
 *   [9] BCC = XOR([0..8])   [10] 0x7D
 *
 * 【主遥测帧 24 字节  STM32 -> ESP32】
 *   [0] 0x7B  [1] Flag_Stop
 *   [2-3] vx mm/s  [4-5] vy mm/s  [6-7] wz mrad/s
 *   [8-19] IMU 六轴原始值  [20-21] 电池 mV  [22] BCC  [23] 0x7D
 *
 * 【里程计帧 32 字节  STM32 -> ESP32】(本工程扩展, 头 0x7E)
 *   [0] 0x7E  [1] flags
 *   [2-5] x mm  [6-9] y mm  [10-11] 航向 0.01度
 *   [12-13] 滤波 vx mm/s  [14-15] 滤波 wz mrad/s
 *   [16-17] 左轮物理 RPM  [18-19] 右轮物理 RPM
 *   [20-21] 左轮目标 RPM  [22-23] 右轮目标 RPM
 *   [24] 1号故障  [25] 2号故障  [26] CAN错误  [27] 坏帧
 *   [28] 有效命令数  [29] 串口溢出  [30] BCC  [31] 0x7D
 *========================================================================*/

/*=========================== 配置 ===========================*/
static const char *WIFI_SSID = "ZTE-s9yACH";
static const char *WIFI_PASSWORD = "Goodhans@8185311";

static const int STM32_RX_PIN = 16;
static const int STM32_TX_PIN = 17;
static const int OLED_SDA_PIN = 8;
static const int OLED_SCL_PIN = 9;

#define MAX_LIN_SPEED   2.10f   /* 与 STM32 的 MAX_RPM=200 对应 */
#define MAX_YAW_RATE    3.00f

#define REPEAT_MS       150     /* 按住时重发周期 */
/*===========================================================*/

#define CMD_FRAME_SIZE  11
#define TEL_FRAME_SIZE  24
#define ODOM_FRAME_SIZE 36
#define HEAD_CMD_TEL    0x7B
#define HEAD_ODOM       0x7E
#define FRAME_TAIL      0x7D

#define FUNC_RELAY      0x05    /* 命令帧 f[1]=0x05 时, f[2] 是继电器掩码 */

#define RAD2DEG 57.29578f
#define DEG2RAD 0.01745329f

/* IMU 换算系数 —— 直接来自 docs/chassis-serial-protocol.md 5.1 节。
   前提是 STM32 上 IMU 初始化量程必须是 ±2g 和 ±500dps。 */
#define ACCEL_RATIO  1671.84f     /* 原始值 / 1671.84  = m/s^2 */
#define GYRO_RATIO   0.00026644f  /* 原始值 * 0.00026644 = rad/s */

HardwareSerial STM32Serial(2);
WebServer server(80);
#define OLED_I2C_ADDRESS 0x3C

/*=========================== OLED ===========================*/
static const uint8_t IP_FONT[11][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
  {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
  {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E},
  {0x00, 0x00, 0x60, 0x60, 0x00}
};

void oledCommand(uint8_t command) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.endTransmission();
}

void oledSetPosition(uint8_t page, uint8_t column) {
  oledCommand(0xB0 | page);
  oledCommand(0x00 | (column & 0x0F));
  oledCommand(0x10 | (column >> 4));
}

void oledClear() {
  uint8_t zeros[16] = {0};
  for (uint8_t page = 0; page < 8; page++) {
    oledSetPosition(page, 0);
    for (uint8_t block = 0; block < 8; block++) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write(0x40);
      Wire.write(zeros, sizeof(zeros));
      Wire.endTransmission();
    }
  }
}

void oledShowIp(const String &ip) {
  oledClear();
  oledSetPosition(3, 0);
  for (size_t i = 0; i < ip.length(); i++) {
    int index = ip[i] == '.' ? 10 : (ip[i] >= '0' && ip[i] <= '9' ? ip[i] - '0' : -1);
    if (index < 0) continue;
    Wire.beginTransmission(OLED_I2C_ADDRESS);
    Wire.write(0x40);
    Wire.write(IP_FONT[index], 5);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
  }
}

void oledInit() {
  delay(20);
  oledCommand(0xAE); oledCommand(0xD5); oledCommand(0x80);
  oledCommand(0xA8); oledCommand(0x3F); oledCommand(0xD3); oledCommand(0x00);
  oledCommand(0x40); oledCommand(0x8D); oledCommand(0x14); oledCommand(0x20);
  oledCommand(0x00); oledCommand(0xA1); oledCommand(0xC8); oledCommand(0xDA);
  oledCommand(0x12); oledCommand(0x81); oledCommand(0xCF); oledCommand(0xD9);
  oledCommand(0xF1); oledCommand(0xDB); oledCommand(0x40); oledCommand(0xA4);
  oledCommand(0xA6); oledCommand(0xAF);
  oledShowIp("0.0.0.0");
}

/*========================= 遥测缓存 =========================*/
struct Telemetry {
  bool valid = false;
  uint32_t lastOk = 0;

  /* 主遥测 */
  bool stopFlag = false;
  float vx = 0, vy = 0, wz = 0;
  uint16_t batteryMv = 0;

  /* 主遥测里的 IMU (协议规定量程 ±2g / ±500dps) */
  int16_t imuRaw[6] = {0};           /* ax ay az gx gy gz 原始值 */
  float ax = 0, ay = 0, az = 0;      /* m/s^2 */
  float gx = 0, gy = 0, gz = 0;      /* rad/s */

  /* 本机(模拟 RK3588)从车体速度积分出的位姿 —— MD 5.2 节那套 */
  float odoX = 0, odoY = 0, odoPsi = 0;
  bool haveMain = false;
  uint32_t lastMainMs = 0;
  uint32_t mainFrames = 0;

  /* 里程计帧 (STM32 自己算的位姿, 留着做对照) */
  bool everLinked = false;
  bool failsafe = false;
  bool imuOk = false;                /* STM32 那边 IMU 最近一次读取是否成功 */
  float x = 0, y = 0, heading = 0;
  float fVx = 0, fWz = 0;
  int16_t wheelL = 0, wheelR = 0, targetL = 0, targetR = 0;
  uint8_t fault1 = 0, fault2 = 0;
  uint8_t canErr = 0, badCmd = 0, cmdCount = 0, rxOverflow = 0;
  uint8_t relay = 0;                 /* bit0 电机继电器 bit1 水泵继电器 */
  uint8_t owner = 0;                 /* 当前控制方: 0=无 1=主口(CH340/RK3588) 2=从口(本ESP32) */
  uint8_t imuStatus = 0;             /* STM32 的 IMU 诊断码 */
  uint8_t imuAddr = 0;               /* STM32 扫到的 I2C 器件地址 */
};

Telemetry telemetry;
static uint32_t lastDriveCmdMs = 0;

static inline int16_t rdI16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline uint16_t rdU16(const uint8_t *p) {
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline int32_t rdI32(const uint8_t *p) {
  return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                   ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

/*========================= 命令下发 =========================*/
void sendVelocity(float vx, float vy, float wz) {
  uint8_t f[CMD_FRAME_SIZE];
  long vxi = lroundf(vx * 1000.0f);
  long vyi = lroundf(vy * 1000.0f);
  long wzi = lroundf(wz * 1000.0f);

  vxi = constrain(vxi, -32768L, 32767L);
  vyi = constrain(vyi, -32768L, 32767L);
  wzi = constrain(wzi, -32768L, 32767L);

  f[0] = HEAD_CMD_TEL;
  f[1] = 0;                              /* AutoRecharge */
  f[2] = 0;                              /* SecurityPLY */
  f[3] = (uint8_t)((uint16_t)vxi >> 8);
  f[4] = (uint8_t)vxi;
  f[5] = (uint8_t)((uint16_t)vyi >> 8);
  f[6] = (uint8_t)vyi;
  f[7] = (uint8_t)((uint16_t)wzi >> 8);
  f[8] = (uint8_t)wzi;

  uint8_t bcc = 0;
  for (uint8_t i = 0; i < 9; i++) bcc ^= f[i];
  f[9] = bcc;
  f[10] = FRAME_TAIL;

  STM32Serial.write(f, sizeof(f));
}

/* 功能帧: 控继电器。掩码 bit0 = 电机继电器, bit1 = 水泵继电器。
   STM32 会锁存这个状态, 不需要周期重发。 */
void sendRelayCommand(uint8_t mask) {
  uint8_t f[CMD_FRAME_SIZE];
  memset(f, 0, sizeof(f));
  f[0] = HEAD_CMD_TEL;
  f[1] = FUNC_RELAY;
  f[2] = (uint8_t)(mask & 0x03);

  uint8_t bcc = 0;
  for (uint8_t i = 0; i < 9; i++) bcc ^= f[i];
  f[9] = bcc;
  f[10] = FRAME_TAIL;

  STM32Serial.write(f, sizeof(f));
}

/*========================= 遥测解析 =========================*/
static void parseMainFrame(const uint8_t *b) {
  uint32_t now = millis();

  telemetry.stopFlag = b[1] != 0;
  telemetry.vx = rdI16(&b[2]) / 1000.0f;
  telemetry.vy = rdI16(&b[4]) / 1000.0f;
  telemetry.wz = rdI16(&b[6]) / 1000.0f;

  /* IMU 六轴 (偏移 8..19), 按协议系数换算成物理量 */
  for (uint8_t i = 0; i < 6; i++) telemetry.imuRaw[i] = rdI16(&b[8 + i * 2]);
  telemetry.ax = telemetry.imuRaw[0] / ACCEL_RATIO;
  telemetry.ay = telemetry.imuRaw[1] / ACCEL_RATIO;
  telemetry.az = telemetry.imuRaw[2] / ACCEL_RATIO;
  telemetry.gx = telemetry.imuRaw[3] * GYRO_RATIO;
  telemetry.gy = telemetry.imuRaw[4] * GYRO_RATIO;
  telemetry.gz = telemetry.imuRaw[5] * GYRO_RATIO;

  telemetry.batteryMv = rdU16(&b[20]);

  /*================= 模拟 RK3588: 自己积分里程计 =================
   * 公式照抄 MD 5.2 节, 注意用的是**更新前的航向** (欧拉积分):
   *     x += (vx*cosψ − vy*sinψ) * dt
   *     y += (vx*sinψ + vy*cosψ) * dt
   *     ψ += wz * dt
   * dt 是相邻两帧成功解析的时间差。
   *=============================================================*/
  if (telemetry.haveMain) {
    float dt = (now - telemetry.lastMainMs) / 1000.0f;
    if (dt > 0.0f && dt < 0.5f) {          /* 断流/重连导致的跳变丢掉不算 */
      float psi = telemetry.odoPsi;
      telemetry.odoX += (telemetry.vx * cosf(psi) - telemetry.vy * sinf(psi)) * dt;
      telemetry.odoY += (telemetry.vx * sinf(psi) + telemetry.vy * cosf(psi)) * dt;
      telemetry.odoPsi += telemetry.wz * dt;
      if (telemetry.odoPsi >  PI) telemetry.odoPsi -= 2.0f * PI;
      if (telemetry.odoPsi < -PI) telemetry.odoPsi += 2.0f * PI;
    }
  }
  telemetry.haveMain = true;
  telemetry.lastMainMs = now;
  telemetry.mainFrames++;

  telemetry.valid = true;
  telemetry.lastOk = now;
}

static void parseOdomFrame(const uint8_t *b) {
  telemetry.everLinked = (b[1] & 0x04) != 0;
  telemetry.failsafe = (b[1] & 0x02) != 0;
  telemetry.imuOk = (b[1] & 0x08) != 0;
  telemetry.x = rdI32(&b[2]) / 1000.0f;
  telemetry.y = rdI32(&b[6]) / 1000.0f;
  telemetry.heading = rdI16(&b[10]) * 0.01f * DEG2RAD;
  telemetry.fVx = rdI16(&b[12]) / 1000.0f;
  telemetry.fWz = rdI16(&b[14]) / 1000.0f;
  telemetry.wheelL = rdI16(&b[16]);
  telemetry.wheelR = rdI16(&b[18]);
  telemetry.targetL = rdI16(&b[20]);
  telemetry.targetR = rdI16(&b[22]);
  telemetry.fault1 = b[24];
  telemetry.fault2 = b[25];
  telemetry.canErr = b[26];
  telemetry.badCmd = b[27];
  telemetry.cmdCount = b[28];
  telemetry.rxOverflow = b[29];
  telemetry.relay = b[30] & 0x03;
  telemetry.owner = b[31];
  telemetry.imuStatus = b[32];
  telemetry.imuAddr = b[33];
  telemetry.valid = true;
  telemetry.lastOk = millis();
}

void pollTelemetry() {
  static uint8_t buf[ODOM_FRAME_SIZE];
  static uint8_t len = 0;
  static uint8_t need = 0;
  static uint8_t prev = 0;

  while (STM32Serial.available()) {
    uint8_t b = (uint8_t)STM32Serial.read();

    if (need == 0) {
      /* 拼帧同步规则照抄 MD 第 5 节:
       *   "上一字节是某帧帧尾、本字节是另一帧帧头时开始计数。
       *    主遥测在上一字节为 0x7D 或回充帧尾 0x7F、本字节为 0x7B 时入帧。"
       * ★ 这里故意和厂商驱动保持一致。如果用更宽松的规则(见到帧头就入帧),
       *   ESP32 能跑但 RK3588 可能跑不了 —— 那样这次调试就白做了。
       */
      uint8_t afterTail = (prev == FRAME_TAIL) || (prev == 0x7F);
      if (afterTail && (b == HEAD_CMD_TEL))   { buf[0] = b; len = 1; need = TEL_FRAME_SIZE; }
      else if (afterTail && (b == HEAD_ODOM)) { buf[0] = b; len = 1; need = ODOM_FRAME_SIZE; }
      prev = b;
      continue;
    }

    buf[len++] = b;
    prev = b;
    if (len < need) continue;

    uint8_t n = need;
    len = 0;
    need = 0;

    if (buf[n - 1] != FRAME_TAIL) continue;
    uint8_t bcc = 0;
    for (uint8_t i = 0; i < n - 2; i++) bcc ^= buf[i];
    if (bcc != buf[n - 2]) continue;

    if (n == TEL_FRAME_SIZE) parseMainFrame(buf);
    else                     parseOdomFrame(buf);
  }
}

/*=========================== 网页 ===========================*/
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>底盘遥控器</title><style>
:root{color-scheme:dark}*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;padding:14px;background:#0e131b;color:#e8f0ff;font-family:system-ui,-apple-system,"PingFang SC",sans-serif;user-select:none;-webkit-user-select:none;overscroll-behavior:none}
h1{font-size:19px;margin:0 0 3px}.sub{font-size:12px;color:#8fa3bf;margin-bottom:12px}
.card{background:#182231;border:1px solid #2c3d54;border-radius:14px;padding:12px;margin:0 auto 12px;max-width:460px}
.bar{display:flex;align-items:center;gap:8px;font-size:13px}
.dot{width:10px;height:10px;border-radius:50%;background:#c0392b;flex:0 0 auto}.dot.on{background:#2ecc71}
.bar .st{margin-left:auto;color:#8fa3bf}
.sl{display:flex;align-items:center;gap:10px;margin-top:11px;font-size:13px}
.sl input{flex:1;accent-color:#2e86de}.sl b{min-width:66px;text-align:right;font-variant-numeric:tabular-nums}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:12px}
.pad button{height:74px;border:0;border-radius:14px;font-size:28px;font-weight:700;color:#fff;background:#2b6cb0;box-shadow:0 4px 0 #17456f;touch-action:none;cursor:pointer}
.pad button.active{transform:translateY(3px);box-shadow:0 1px 0 #17456f;background:#3b8ae0}
.pad .gap{visibility:hidden}.pad .stop{background:#c0392b;box-shadow:0 4px 0 #7d1f16;font-size:24px}
.pad .stop.active{background:#e04b3a;box-shadow:0 1px 0 #7d1f16}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;font-size:12px}
.grid div{background:#101825;border-radius:9px;padding:8px}.grid span{color:#8fa3bf}.grid b{float:right;font-variant-numeric:tabular-nums}
.cv{width:100%;height:240px;display:block;border-radius:10px;background:#0b1017}
.hdr{display:flex;align-items:center;font-size:13px;color:#a8bdd8;margin-bottom:8px}
.hdr button{margin-left:auto;padding:5px 10px;border:0;border-radius:7px;background:#3d4f68;color:#fff;font-size:12px}
.rowbtn{display:flex;gap:10px;margin-top:10px}
.rowbtn button{flex:1;padding:15px 8px;border:0;border-radius:11px;background:#3a4b62;color:#cfdcee;font-size:15px;font-weight:600;cursor:pointer;transition:background .12s}
.rowbtn button.on{background:#1f8a4c;color:#fff;box-shadow:inset 0 0 0 2px #35c273}
.note{font-size:11px;color:#7d90aa;margin-top:9px;line-height:1.55}
</style></head><body>
<h1>底盘遥控器</h1><div class="sub">按住方向键行驶，松手立即刹车；断线约 0.8 秒后底盘自动停车</div>

<section class="card">
<div class="bar"><i id="dot" class="dot"></i><span id="link">等待 STM32…</span><span class="st" id="state">待机</span></div>
<div class="sl"><span>速度</span><input id="speed" type="range" min="10" max="210" step="5" value="60"><b id="speedv">0.60 m/s</b></div>
<div class="sl"><span>转向</span><input id="turn" type="range" min="20" max="300" step="10" value="120"><b id="turnv">1.20 rad/s</b></div>
<div class="pad">
<button class="gap"></button><button data-dir="forward">▲</button><button class="gap"></button>
<button data-dir="left">◀</button><button class="stop" id="stopbtn">■</button><button data-dir="right">▶</button>
<button class="gap"></button><button data-dir="back">▼</button><button class="gap"></button>
</div></section>

<section class="card">
<h1 style="font-size:15px;margin:0 0 2px">外设开关</h1>
<div class="sub" style="margin:2px 0 0">PB0 电机继电器 · PB1 水泵继电器</div>
<div class="rowbtn">
<button id="rly_motor">电机 关</button>
<button id="rly_pump">水泵 关</button>
</div>
<div class="note" id="rlynote">状态由 STM32 锁存。急停会同时断开这两个继电器；链路断开也会自动断开。</div>
</section>

<section class="card">
<div class="hdr">轨迹（ESP32 本机积分，模拟 RK3588）<button id="clr">清空显示</button></div>
<canvas id="cv" class="cv"></canvas>
<div class="note">只用车体速度 (vx,vy,wz) 按 MD 5.2 节积分，不依赖 STM32 的位姿帧 —— 这样才等于在上位机上跑。</div>
</section>

<section class="card grid">
<div style="grid-column:1/-1;background:none;padding:0 0 6px;color:#a8bdd8">① 轮速计 → 车体速度（24 字节帧 偏移 2..7）</div>
<div><span>vx</span><b id="pvx">--</b></div><div><span>vy</span><b id="pvy">--</b></div>
<div><span>wz</span><b id="pwz">--</b></div><div><span>积分里程</span><b id="pdist">--</b></div>
<div><span>X</span><b id="px">--</b></div><div><span>Y</span><b id="py">--</b></div>
<div><span>航向 ψ</span><b id="ph">--</b></div><div><span>主遥测帧数</span><b id="pmf">--</b></div>
</section>

<section class="card grid">
<div style="grid-column:1/-1;background:none;padding:0 0 6px;color:#a8bdd8">② IMU（24 字节帧 偏移 8..19，协议量程 ±2g / ±500dps）<b id="pimu" style="float:right">--</b></div>
<div><span>ax</span><b id="pax">--</b></div><div><span>ay</span><b id="pay">--</b></div>
<div><span>az</span><b id="paz">--</b></div><div><span>gx</span><b id="pgx">--</b></div>
<div><span>gy</span><b id="pgy">--</b></div><div><span>gz</span><b id="pgz">--</b></div>
<div style="grid-column:1/-1"><span>原始值</span><b id="praw">--</b></div>
<div style="grid-column:1/-1"><span>读取失败原因</span><b id="pdia">--</b></div>
<div style="grid-column:1/-1"><span>I2C 扫描结果</span><b id="padr">--</b></div>
</section>

<section class="card grid">
<div style="grid-column:1/-1;background:none;padding:0 0 6px;color:#a8bdd8">③ 底盘诊断（STM32 扩展帧 0x7E）</div>
<div><span>STM32位姿</span><b id="pstm">--</b></div><div><span>左轮 实测/目标</span><b id="pw1">--</b></div>
<div><span>右轮 实测/目标</span><b id="pw2">--</b></div><div><span>1号故障</span><b id="f1">--</b></div>
<div><span>2号故障</span><b id="f2">--</b></div><div><span>电压</span><b id="bat">--</b></div>
<div><span>有效命令</span><b id="cmd">--</b></div><div><span>控制方</span><b id="own">--</b></div>
<div><span>坏帧</span><b id="bad">--</b></div><div><span>CAN 错误</span><b id="cerr">--</b></div>
<div><span>串口溢出</span><b id="ovf">--</b></div><div><span>状态</span><b id="fs">--</b></div>
</section>

<script>
var speedEl=document.getElementById('speed'),turnEl=document.getElementById('turn');
var NAMES={forward:'前进',back:'后退',left:'左转',right:'右转'};
var timer=null,active=null;
function fmt(){document.getElementById('speedv').textContent=(speedEl.value/100).toFixed(2)+' m/s';
               document.getElementById('turnv').textContent=(turnEl.value/100).toFixed(2)+' rad/s'}
speedEl.oninput=fmt;turnEl.oninput=fmt;fmt();
function setState(t){document.getElementById('state').textContent=t}
function drive(vx,wz){
  fetch('/api/drive?vx='+vx.toFixed(3)+'&wz='+wz.toFixed(3),{cache:'no-store'})
    .then(function(r){if(!r.ok)throw new Error('HTTP '+r.status)})
    .catch(function(){setState('发送失败!')});
}
function begin(d){
  if(active===d)return;end();active=d;
  function send(){
    var S=speedEl.value/100,T=turnEl.value/100,v=0,w=0;
    if(d==='forward')v=S;else if(d==='back')v=-S;else if(d==='left')w=T;else if(d==='right')w=-T;
    drive(v,w);
  }
  setState(NAMES[d]);send();
  timer=setInterval(send,150);
}
function end(){
  if(timer){clearInterval(timer);timer=null}
  if(active!==null){active=null;drive(0,0);setState('刹车')}
}
document.querySelectorAll('[data-dir]').forEach(function(b){
  var d=b.dataset.dir;
  b.addEventListener('pointerdown',function(e){e.preventDefault();b.classList.add('active');begin(d)});
  ['pointerup','pointercancel','pointerleave'].forEach(function(ev){
    b.addEventListener(ev,function(){b.classList.remove('active');end()})});
});
var stopbtn=document.getElementById('stopbtn');
stopbtn.addEventListener('pointerdown',function(e){e.preventDefault();stopbtn.classList.add('active');end();drive(0,0);sendRelayMask(0);setState('急停')});
['pointerup','pointercancel','pointerleave'].forEach(function(ev){stopbtn.addEventListener(ev,function(){stopbtn.classList.remove('active')})});
window.addEventListener('pointerup',function(){end()});
window.addEventListener('blur',function(){end()});
document.addEventListener('visibilitychange',function(){if(document.hidden)end()});
document.addEventListener('keydown',function(e){var m={ArrowUp:'forward',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right',w:'forward',s:'back',a:'left',d:'right'};var k=m[e.key];if(k){e.preventDefault();begin(k)}});
document.addEventListener('keyup',function(e){var m={ArrowUp:1,ArrowDown:1,ArrowLeft:1,ArrowRight:1,w:1,s:1,a:1,d:1};if(m[e.key])end()});

/* ---- 继电器 ---- */
var rlyM=document.getElementById('rly_motor'),rlyP=document.getElementById('rly_pump');
var relayState=0;
function paintRelay(){
  rlyM.textContent='电机 '+((relayState&1)?'开':'关');rlyM.className=(relayState&1)?'on':'';
  rlyP.textContent='水泵 '+((relayState&2)?'开':'关');rlyP.className=(relayState&2)?'on':'';
}
function sendRelayMask(mask){
  fetch('/api/relay?mask='+(mask&3),{cache:'no-store'})
    .then(function(r){if(!r.ok)throw new Error('HTTP '+r.status)})
    .catch(function(){setState('继电器命令失败')});
}
rlyM.addEventListener('click',function(){sendRelayMask((relayState&1)?(relayState&~1):(relayState|1))});
rlyP.addEventListener('click',function(){sendRelayMask((relayState&2)?(relayState&~2):(relayState|2))});
paintRelay();

/* ---- 轨迹绘制 ---- */
var cv=document.getElementById('cv'),cx=cv.getContext('2d'),pts=[],lastPt=null;
function resize(){var r=cv.getBoundingClientRect();cv.width=r.width*2;cv.height=r.height*2;draw()}
window.addEventListener('resize',resize);
document.getElementById('clr').addEventListener('click',function(){pts=[];lastPt=null;draw()});
function pushPoint(x,y,h){
  if(lastPt&&Math.hypot(x-lastPt[0],y-lastPt[1])<0.02)  {lastPt[2]=h;return}
  pts.push([x,y,h]);lastPt=pts[pts.length-1];
  if(pts.length>1500)pts.shift();
}
function draw(){
  var W=cv.width,H=cv.height;
  cx.setTransform(1,0,0,1,0,0);cx.clearRect(0,0,W,H);
  if(!pts.length){cx.fillStyle='#4a5b73';cx.font=(18*2)+'px sans-serif';cx.textAlign='center';
    cx.fillText('还没有轨迹数据',W/2,H/2);return}
  var minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
  for(var i=0;i<pts.length;i++){var p=pts[i];
    if(p[0]<minx)minx=p[0];if(p[0]>maxx)maxx=p[0];if(p[1]<miny)miny=p[1];if(p[1]>maxy)maxy=p[1];}
  var span=Math.max(maxx-minx,maxy-miny,1.0)*1.25;
  var sc=Math.min(W,H)/span, ox=W/2-((minx+maxx)/2)*sc, oy=H/2+((miny+maxy)/2)*sc;
  function tx(x){return ox+x*sc} function ty(y){return oy-y*sc}
  cx.strokeStyle='#1e2b3d';cx.lineWidth=2;
  for(var g=Math.floor(minx);g<=maxx+1;g++){cx.beginPath();cx.moveTo(tx(g),0);cx.lineTo(tx(g),H);cx.stroke()}
  for(var g2=Math.floor(miny);g2<=maxy+1;g2++){cx.beginPath();cx.moveTo(0,ty(g2));cx.lineTo(W,ty(g2));cx.stroke()}
  cx.strokeStyle='#2ecc71';cx.lineWidth=3;cx.beginPath();
  for(var j=0;j<pts.length;j++){j?cx.lineTo(tx(pts[j][0]),ty(pts[j][1])):cx.moveTo(tx(pts[j][0]),ty(pts[j][1]))}
  cx.stroke();
  var e=pts[pts.length-1],X=tx(e[0]),Y=ty(e[1]),a=e[2];
  cx.save();cx.translate(X,Y);cx.rotate(-a);
  cx.fillStyle='#f1c40f';cx.beginPath();cx.moveTo(30,0);cx.lineTo(-16,16);cx.lineTo(-16,-16);cx.closePath();cx.fill();
  cx.restore();
  cx.fillStyle='#8fa3bf';cx.font=(11*2)+'px sans-serif';cx.textAlign='left';
  cx.fillText('1格 = 1 m',12,22);
}

function hex(v){return '0x'+(v>>>0).toString(16).toUpperCase()}
function put(id,v){document.getElementById(id).textContent=v}
async function refresh(){
  try{
    var s=await (await fetch('/api/status',{cache:'no-store'})).json();
    var online=s.valid&&s.age_ms<600;
    document.getElementById('dot').className='dot'+(online?' on':'');
    put('link',!s.valid?'STM32 无数据':(online?'STM32 在线':'STM32 离线'));

    /* ① 轮速计 -> 车体速度 -> 本机积分出的位姿 */
    put('pvx',s.valid?s.vx.toFixed(3)+' m/s':'--');
    put('pvy',s.valid?s.vy.toFixed(3)+' m/s':'--');
    put('pwz',s.valid?s.wz.toFixed(3)+' rad/s':'--');
    put('px',s.valid?s.odo_x.toFixed(3)+' m':'--');
    put('py',s.valid?s.odo_y.toFixed(3)+' m':'--');
    put('ph',s.valid?(s.odo_psi*57.29578).toFixed(1)+' °':'--');
    put('pdist',s.valid?Math.hypot(s.odo_x,s.odo_y).toFixed(3)+' m':'--');
    put('pmf',s.valid?s.main_frames:'--');

    /* ② IMU */
    var ip=s.imu_phys||[0,0,0,0,0,0];
    var ST={0:'正常',1:'SCL 拉不高（缺上拉/没接/短路到地）',
            2:'SDA 拉不高（缺上拉/没接/短路到地）',
            3:'总线被拉死',4:'地址 0x23 没有应答',5:'地址应答了但读数据出错',
            6:'陀螺仪恒为 0（加速度正常）'};
    put('pimu',s.valid?(s.imu_ok?(s.imu_status?'部分异常':'读取正常'):'读取失败'):'--');
    put('pdia',s.valid?(s.imu_status?(ST[s.imu_status]||('未知 '+s.imu_status)):'—'):'--');
    put('padr',s.valid?(s.imu_addr?('0x'+s.imu_addr.toString(16).toUpperCase()):'没扫到任何器件'):'--');
    put('pax',s.valid?ip[0].toFixed(3)+' m/s²':'--');
    put('pay',s.valid?ip[1].toFixed(3)+' m/s²':'--');
    put('paz',s.valid?ip[2].toFixed(3)+' m/s²':'--');
    put('pgx',s.valid?ip[3].toFixed(4)+' rad/s':'--');
    put('pgy',s.valid?ip[4].toFixed(4)+' rad/s':'--');
    put('pgz',s.valid?ip[5].toFixed(4)+' rad/s':'--');
    put('praw',s.valid?(s.imu_raw||[]).join(' / '):'--');

    /* ③ 底盘诊断 */
    put('pstm',s.valid?(s.x.toFixed(2)+', '+s.y.toFixed(2)+', '+(s.heading*57.29578).toFixed(0)+'°'):'--');
    put('pw1',s.valid?(s.wheel_l+' / '+s.target_l):'--');
    put('pw2',s.valid?(s.wheel_r+' / '+s.target_r):'--');
    put('f1',s.valid?s.fault1:'--');put('f2',s.valid?s.fault2:'--');
    put('bat',s.valid&&s.battery_mv?(s.battery_mv/1000).toFixed(1)+' V':'--');
    put('cmd',s.valid?s.cmd_count:'--');put('bad',s.valid?s.bad_cmd:'--');
    put('cerr',s.valid?s.can_errors:'--');put('ovf',s.valid?s.rx_overflow:'--');
    put('fs',s.valid?(s.failsafe?'看门狗停车':'正常'):'--');
    put('own',s.valid?({0:'无',1:'主口(RK3588)',2:'ESP32'}[s.owner]||'?'):'--');

    /* 轨迹画的是本机积分的结果 */
    if(s.valid){relayState=s.relay&3;paintRelay();pushPoint(s.odo_x,s.odo_y,s.odo_psi)}
    if(active===null){
      if(!s.valid)setState('STM32 无数据');
      else if(s.failsafe)setState('看门狗已停车');
      else if(Math.abs(s.vx)>0.02||Math.abs(s.wz)>0.02)setState('行驶中');
      else setState('待机');
    }
    draw();
  }catch(e){document.getElementById('dot').className='dot';put('link','与 ESP32 断连')}
}
resize();refresh();setInterval(refresh,300);
</script></body></html>)rawliteral";

/*========================= HTTP 处理 ========================*/
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleDrive() {
  if (!server.hasArg("vx") || !server.hasArg("wz")) {
    server.send(400, "text/plain", "需要 vx 和 wz 参数");
    return;
  }
  float vx = server.arg("vx").toFloat();
  float wz = server.arg("wz").toFloat();
  if (vx > MAX_LIN_SPEED) vx = MAX_LIN_SPEED;
  if (vx < -MAX_LIN_SPEED) vx = -MAX_LIN_SPEED;
  if (wz > MAX_YAW_RATE) wz = MAX_YAW_RATE;
  if (wz < -MAX_YAW_RATE) wz = -MAX_YAW_RATE;

  sendVelocity(vx, 0.0f, wz);
  lastDriveCmdMs = millis();
  server.send(200, "application/json", "{\"ok\":true}");
}

/* 继电器: /api/relay?mask=N   bit0 电机 bit1 水泵
   整帧下发而不是按位改, 否则急停同时关两个继电器时会互相覆盖。 */
void handleRelay() {
  uint8_t mask = (uint8_t)(telemetry.relay & 0x03);
  if (server.hasArg("mask")) {
    mask = (uint8_t)(server.arg("mask").toInt() & 0x03);
  }
  sendRelayCommand(mask);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  uint32_t age = telemetry.valid ? (millis() - telemetry.lastOk) : 999999;
  String j = "{\"valid\":" + String(telemetry.valid ? "true" : "false");
  j += ",\"age_ms\":" + String(age);
  j += ",\"x\":" + String(telemetry.x, 3);
  j += ",\"y\":" + String(telemetry.y, 3);
  j += ",\"heading\":" + String(telemetry.heading, 4);
  j += ",\"vx\":" + String(telemetry.vx, 3);
  j += ",\"vy\":" + String(telemetry.vy, 3);
  j += ",\"wz\":" + String(telemetry.wz, 3);
  j += ",\"odo_x\":" + String(telemetry.odoX, 3);
  j += ",\"odo_y\":" + String(telemetry.odoY, 3);
  j += ",\"odo_psi\":" + String(telemetry.odoPsi, 4);
  j += ",\"main_frames\":" + String(telemetry.mainFrames);
  j += ",\"imu_raw\":[";
  for (uint8_t i = 0; i < 6; i++) { if (i) j += ","; j += String(telemetry.imuRaw[i]); }
  j += "],\"imu_phys\":[";
  j += String(telemetry.ax, 3) + "," + String(telemetry.ay, 3) + "," + String(telemetry.az, 3) + ",";
  j += String(telemetry.gx, 4) + "," + String(telemetry.gy, 4) + "," + String(telemetry.gz, 4);
  j += "]";
  j += ",\"fvx\":" + String(telemetry.fVx, 3);
  j += ",\"fwz\":" + String(telemetry.fWz, 3);
  j += ",\"wheel_l\":" + String(telemetry.wheelL);
  j += ",\"wheel_r\":" + String(telemetry.wheelR);
  j += ",\"target_l\":" + String(telemetry.targetL);
  j += ",\"target_r\":" + String(telemetry.targetR);
  j += ",\"fault1\":" + String(telemetry.fault1);
  j += ",\"fault2\":" + String(telemetry.fault2);
  j += ",\"battery_mv\":" + String(telemetry.batteryMv);
  j += ",\"can_errors\":" + String(telemetry.canErr);
  j += ",\"bad_cmd\":" + String(telemetry.badCmd);
  j += ",\"cmd_count\":" + String(telemetry.cmdCount);
  j += ",\"rx_overflow\":" + String(telemetry.rxOverflow);
  j += ",\"failsafe\":" + String(telemetry.failsafe ? "true" : "false");
  j += ",\"imu_ok\":" + String(telemetry.imuOk ? "true" : "false");
  j += ",\"imu_status\":" + String(telemetry.imuStatus);
  j += ",\"imu_addr\":" + String(telemetry.imuAddr);
  j += ",\"ever_linked\":" + String(telemetry.everLinked ? "true" : "false");
  j += ",\"relay\":" + String(telemetry.relay & 0x03);
  j += ",\"owner\":" + String(telemetry.owner);
  j += "}";
  server.send(200, "application/json", j);
}

/*=========================== setup ==========================*/
void setup() {
  Serial.begin(115200);
  STM32Serial.begin(115200, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  oledInit();

  /* 上电先发一帧零速度, 保证底盘不动 */
  delay(300);
  sendVelocity(0, 0, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("Wi-Fi IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/drive", HTTP_GET, handleDrive);
  server.on("/api/relay", HTTP_GET, handleRelay);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.begin();
}

void loop() {
  pollTelemetry();
  server.handleClient();

  /* 空闲心跳: 400ms 没收到遥控请求(比如手机锁屏、页面被关掉)就补发零速度。
     这样底盘不会等 800ms 看门狗才停; 同时链路一直有数据, 看门狗标志不会误报。 */
  static uint32_t lastIdleMs = 0;
  if (millis() - lastDriveCmdMs > 400 && millis() - lastIdleMs > 400) {
    lastIdleMs = millis();
    sendVelocity(0, 0, 0);
  }

  static uint32_t lastOledUpdate = 0;
  if (millis() - lastOledUpdate >= 1000) {
    lastOledUpdate = millis();
    String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "0.0.0.0";
    oledShowIp(ip);
  }
  delay(2);
}
