"""底盘主控 V2 网页监视/控制面板（单文件，只用标准库 + pyserial）。

把板子调试口（CH343P，如 COM18）上的两路上行帧解出来展示在网页上：
  - 0x7E 32 字节遥测帧：模式 / IMU 加速度陀螺 / 温湿度 / 速度指令 / 计数
  - 0x79 19 字节遥控帧：CH554G 转发的手柄 15 字节原始报告

还能从网页下发：
  - 切模式（0x7A func=0x02：HOST / RC / PUSH）
  - HOST 模式下的速度指令（0x7B 11 字节速度帧）
  - 停车

用法：
    python tools/v2_web.py --port COM18
    python tools/v2_web.py --port COM18 --web-port 8010
然后浏览器打开 http://127.0.0.1:8010

注意：串口是独占的，跑这个脚本时不要同时跑 v2_monitor.py。
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

import v2_monitor as mon

MODE_NAMES = mon.MODE_NAMES
MODE_LED = {0: "#2b6cff", 1: "#20c060", 2: "#ff8a00"}   # 与板上 LED 颜色一致: 蓝/绿/橙


# ----------------------------- 串口 & 帧解析 -----------------------------


class BoardLink:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.ser = None
        self.stop_evt = threading.Event()
        self.buffer = bytearray()

        self.telemetry = None          # 最近一帧遥测的解码结果
        self.rc = None                 # 最近一帧遥控的解码结果
        self.telemetry_count = 0
        self.rc_count = 0
        self.bad_bytes = 0
        self.tel_ts = 0.0
        self.rc_ts = 0.0
        self.started = time.time()
        self.error = ""

        self._tel_hist = []            # (时间, 累计帧数) 用来算帧率

    # ---- 打开串口并起读线程 ----
    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as exc:      # 串口被拔 / 驱动异常
                with self.lock:
                    self.error = str(exc)
                break
            if not chunk:
                continue
            self.buffer.extend(chunk)
            self._pump()

    def _pump(self):
        while True:
            if not self.buffer:
                return
            head = self.buffer[0]
            if head == mon.HEAD:
                need, decoder = mon.FRAME_SIZE, mon.decode
            elif head == mon.RC_HEAD:
                need, decoder = mon.RC_SIZE, mon.decode_rc
            else:
                with self.lock:
                    self.bad_bytes += 1
                del self.buffer[:1]
                continue
            if len(self.buffer) < need:
                return
            info = decoder(bytes(self.buffer[:need]))
            if info is None:
                with self.lock:
                    self.bad_bytes += 1
                del self.buffer[:1]
                continue
            del self.buffer[:need]
            now = time.time()
            with self.lock:
                if head == mon.HEAD:
                    self.telemetry = info
                    self.telemetry_count += 1
                    self.tel_ts = now
                    self._tel_hist.append((now, self.telemetry_count))
                    while len(self._tel_hist) > 100:
                        self._tel_hist.pop(0)
                else:
                    self.rc = info
                    self.rc_count += 1
                    self.rc_ts = now

    # ---- 下行 ----
    def send(self, data):
        if self.ser is None:
            return False
        try:
            self.ser.write(data)
            return True
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            return False

    def frame_speed(self, vx_mm_s, wz_mrad_s=0):
        vx = int(max(-32768, min(32767, vx_mm_s)))
        wz = int(max(-32768, min(32767, wz_mrad_s)))
        body = bytes([0x7B, 0x00, 0x00,
                      (vx >> 8) & 0xFF, vx & 0xFF,
                      0x00, 0x00,
                      (wz >> 8) & 0xFF, wz & 0xFF])
        return body + bytes([mon.xor_bcc(body), 0x7D])

    def frame_set_mode(self, mode):
        body = bytes([0x7A, 0x02, mode & 0xFF, 0, 0, 0])
        return body + bytes([mon.xor_bcc(body), 0x7D])

    # ---- 对外快照 ----
    def snapshot(self):
        with self.lock:
            tel = dict(self.telemetry) if self.telemetry else None
            rc = dict(self.rc) if self.rc else None
            if tel is not None:
                tel.pop("raw", None)
            if rc is not None:
                rc.pop("raw", None)

            hist = list(self._tel_hist)
            hz = 0.0
            if len(hist) >= 2:
                span = hist[-1][0] - hist[0][0]
                if span > 0.5:
                    hz = (hist[-1][1] - hist[0][1]) / span
            age_tel = (time.time() - self.tel_ts) if self.tel_ts else None
            age_rc = (time.time() - self.rc_ts) if self.rc_ts else None

            return {
                "ok": True,
                "port": self.port,
                "uptime_s": round(time.time() - self.started, 1),
                "telemetry": tel,
                "rc": rc,
                "link": {
                    "telemetry_frames": self.telemetry_count,
                    "rc_frames": self.rc_count,
                    "telemetry_hz": round(hz, 1),
                    "bad_bytes": self.bad_bytes,
                    "telemetry_age_s": None if age_tel is None else round(age_tel, 2),
                    "rc_age_s": None if age_rc is None else round(age_rc, 2),
                    "error": self.error,
                },
                "mode_names": MODE_NAMES,
                "mode_led": MODE_LED,
            }


# ----------------------------- 网页 -----------------------------

PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TB01 底盘主控 V2</title>
<style>
:root{--bg:#0f1216;--card:#171c22;--line:#242c35;--fg:#e7edf3;--dim:#8b98a5;--ok:#20c060;--bad:#e04a4a}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Microsoft YaHei",sans-serif}
header{display:flex;align-items:center;gap:14px;padding:14px 20px;border-bottom:1px solid var(--line);position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:16px;margin:0;font-weight:600;letter-spacing:.5px}
#mode{font-weight:700;font-size:18px;padding:4px 14px;border-radius:8px;background:#222;color:#fff}
.wrap{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;padding:16px 20px 40px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px 16px}
.card h2{font-size:12px;margin:0 0 10px;color:var(--dim);font-weight:600;letter-spacing:1px;text-transform:uppercase}
.row{display:flex;justify-content:space-between;align-items:baseline;padding:3px 0}
.k{color:var(--dim)}
.v{font-variant-numeric:tabular-nums;font-family:ui-monospace,Consolas,monospace}
.big{font-size:26px;font-weight:700}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--bad);margin-right:7px;vertical-align:middle}
.dot.on{background:var(--ok)}
.bar{position:relative;height:8px;background:#0b0e12;border-radius:4px;overflow:hidden;margin:4px 0 8px}
.bar>i{position:absolute;top:0;bottom:0;left:50%;width:2px;background:#4a5563}
.bar>b{position:absolute;top:0;bottom:0;background:#3b82f6;border-radius:4px}
button{background:#1f2731;color:var(--fg);border:1px solid var(--line);border-radius:9px;padding:9px 14px;font-size:13px;cursor:pointer;margin:0 6px 6px 0}
button:hover{background:#27313d}
button.on{background:#2b6cff;border-color:#2b6cff}
button.stop{background:#8c1d1d;border-color:#a52a2a}
small{color:var(--dim)}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.grid3 div{background:#0b0e12;border-radius:8px;padding:8px 4px}
.grid3 b{display:block;font-size:17px;font-variant-numeric:tabular-nums}
.grid3 span{color:var(--dim);font-size:11px}
</style>
</head>
<body>
<header>
  <h1>TB01 底盘主控 V2</h1>
  <span id="mode" style="background:#222">--</span>
  <small id="link">连接中…</small>
</header>

<div class="wrap">
  <div class="card">
    <h2>IMU（ICM-42670-P）</h2>
    <div class="grid3" id="acc"></div>
    <div style="height:10px"></div>
    <div class="grid3" id="gyro"></div>
    <div style="height:10px"></div>
    <div class="row"><span class="k">加速度</span><span class="v" id="accmag">--</span></div>
    <div class="row"><span class="k">倾角</span><span class="v" id="tilt">--</span></div>
    <div class="row"><span class="k">IMU 温度</span><span class="v" id="timu">--</span></div>
  </div>

  <div class="card">
    <h2>温湿度（AHT20）</h2>
    <div class="row"><span class="k">温度</span><span class="v big" id="taht">--</span></div>
    <div class="row"><span class="k">湿度</span><span class="v big" id="rh">--</span></div>
    <div class="row"><span class="k">状态</span><span class="v" id="ahtok">--</span></div>
  </div>

  <div class="card">
    <h2>速度指令（mm/s, mrad/s）</h2>
    <div class="row"><span class="k">vx</span><span class="v big" id="vx">--</span></div>
    <div class="row"><span class="k">wz</span><span class="v" id="wz">--</span></div>
    <div class="row"><span class="k">上位机命令数</span><span class="v" id="rx">--</span></div>
    <div class="row"><span class="k">被拒（模式不对）</span><span class="v" id="rj">--</span></div>
    <div class="row"><span class="k">固件计时</span><span class="v" id="tick">--</span></div>
  </div>

  <div class="card">
    <h2>遥控手柄（CH554G）</h2>
    <div class="row"><span class="k"><span id="rcdot" class="dot"></span>在线</span><span class="v" id="rconline">--</span></div>
    <div class="row"><span class="k">左摇杆 X</span><span class="v" id="rcx">--</span></div>
    <div class="bar"><i></i><b id="rcxbar" style="width:0%;left:50%"></b></div>
    <div class="row"><span class="k">左摇杆 Y</span><span class="v" id="rcy">--</span></div>
    <div class="bar"><i></i><b id="rcybar" style="width:0%;left:50%"></b></div>
    <div class="row"><span class="k">Z（扳机差）</span><span class="v" id="rcz">--</span></div>
    <div class="row"><span class="k">十字键</span><span class="v" id="rchat">--</span></div>
    <div class="row"><span class="k">按钮</span><span class="v" id="rcbtn">--</span></div>
  </div>

  <div class="card">
    <h2>链路</h2>
    <div class="row"><span class="k">端口</span><span class="v" id="lport">--</span></div>
    <div class="row"><span class="k">遥测帧率</span><span class="v" id="lhz">--</span></div>
    <div class="row"><span class="k">遥测帧数</span><span class="v" id="lcount">--</span></div>
    <div class="row"><span class="k">遥控帧数</span><span class="v" id="lrc">--</span></div>
    <div class="row"><span class="k">重同步字节</span><span class="v" id="lbad">--</span></div>
    <div class="row"><span class="k">程序版本</span><span class="v" id="lver">--</span></div>
    <div class="row"><span class="k">错误</span><span class="v" id="lerr">--</span></div>
  </div>

  <div class="card">
    <h2>控制</h2>
    <div>
      <button data-mode="0" class="modebtn">HOST（上位机）</button>
      <button data-mode="1" class="modebtn">RC（遥控）</button>
      <button data-mode="2" class="modebtn">PUSH（推行）</button>
    </div>
    <div style="height:8px"></div>
    <div>
      <button data-speed="100">前进 0.10 m/s</button>
      <button data-speed="200">前进 0.20 m/s</button>
      <button data-speed="500">前进 0.50 m/s</button>
    </div>
    <div>
      <button data-speed="-100">后退 0.10 m/s</button>
      <button data-turn="-300">右转</button>
      <button data-turn="300">左转</button>
    </div>
    <div style="height:6px"></div>
    <button class="stop" id="stopbtn">停车（vx=0, wz=0）</button>
    <div style="height:8px"></div>
    <small>PUSH 模式会拒收速度指令；切回 HOST 才能动。</small>
  </div>
</div>

<script>
const MODE_LED = %MODE_LED%;

function num(v, d) { return (v === null || v === undefined) ? "--" : Number(v).toFixed(d); }
function setText(id, s) { const e = document.getElementById(id); if (e) e.textContent = s; }
function setBar(id, permille) {
  const e = document.getElementById(id);
  const p = Math.max(-1000, Math.min(1000, permille)) / 1000;
  const w = Math.abs(p) * 50;
  e.style.width = w + "%";
  e.style.left = p >= 0 ? "50%" : (50 - w) + "%";
  e.style.background = p >= 0 ? "#3b82f6" : "#c07a20";
}

async function tick() {
  let s;
  try { s = await (await fetch("/api/state", {cache: "no-store"})).json(); }
  catch (e) { setText("link", "连不上服务"); return; }

  const t = s.telemetry, rc = s.rc, L = s.link;

  // 模式
  const mode = t ? t.mode : null;
  const name = (mode !== null && s.mode_names[mode]) ? s.mode_names[mode] : "--";
  const badge = document.getElementById("mode");
  badge.textContent = name;
  badge.style.background = (mode !== null ? MODE_LED[mode] : "#222") || "#222";
  document.querySelectorAll(".modebtn").forEach(b => {
    b.classList.toggle("on", mode !== null && Number(b.dataset.mode) === mode);
  });

  setText("link", L.error ? ("串口错误: " + L.error) :
    ("遥测 " + L.telemetry_hz + " Hz  ·  " + (L.telemetry_age_s ?? "-") + " s 前"));

  if (t) {
    const acc = t.acc_g, gyro = t.gyro_dps;
    document.getElementById("acc").innerHTML =
      acc.map((v, i) => `<div><b>${num(v, 3)}</b><span>${"XYZ"[i]} (g)</span></div>`).join("");
    document.getElementById("gyro").innerHTML =
      gyro.map((v, i) => `<div><b>${num(v, 1)}</b><span>${"XYZ"[i]} (°/s)</span></div>`).join("");
    const mag = Math.sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    setText("accmag", num(mag, 3) + " g");
    const tilt = Math.atan2(Math.hypot(acc[0], acc[1]), acc[2]) * 180 / Math.PI;
    setText("tilt", num(tilt, 2) + " °");
    setText("timu", num(t.imu_temp_c, 2) + " °C");
    setText("taht", num(t.aht_temp_c, 1) + " °C");
    setText("rh", num(t.rh_percent, 1) + " %");
    setText("ahtok", t.aht_ok ? "正常" : "无数据");
    setText("vx", String(t.vx_mm_s));
    setText("wz", String(t.wz_mrad_s));
    setText("rx", String(t.rx_count));
    setText("rj", String(t.rj_count));
    setText("tick", (t.tick_ms / 1000).toFixed(1) + " s");
    setText("lver", "v" + t.version);
  }

  if (rc) {
    document.getElementById("rcdot").className = "dot" + (rc.online ? " on" : "");
    setText("rconline", rc.online ? "在线" : "离线");
    setText("rcx", String(rc.x));
    setText("rcy", String(rc.y));
    setText("rcz", String(rc.z));
    setText("rchat", rc.hat_name);
    setText("rcbtn", (rc.button_names && rc.button_names.length) ? rc.button_names.join("+") : "-");
    setBar("rcxbar", Math.round((rc.x - 32768) / 327.68));
    setBar("rcybar", Math.round((rc.y - 32767) / 327.67));
  } else {
    document.getElementById("rcdot").className = "dot";
    setText("rconline", "还没收到");
  }

  setText("lport", s.port);
  setText("lhz", L.telemetry_hz + " Hz");
  setText("lcount", String(L.telemetry_frames));
  setText("lrc", String(L.rc_frames));
  setText("lbad", String(L.bad_bytes));
  setText("lerr", L.error || "-");
}

async function post(url, body) {
  try {
    await fetch(url, {method: "POST", headers: {"Content-Type": "application/json"},
                       body: JSON.stringify(body || {})});
  } catch (e) { /* 下一轮 tick 会把链路状态显示出来 */ }
}

document.querySelectorAll(".modebtn").forEach(b => b.onclick = () => post("/api/mode", {mode: Number(b.dataset.mode)}));
document.querySelectorAll("[data-speed]").forEach(b => b.onclick = () => post("/api/cmd", {vx: Number(b.dataset.speed), wz: 0}));
document.querySelectorAll("[data-turn]").forEach(b => b.onclick = () => post("/api/cmd", {vx: 0, wz: Number(b.dataset.turn)}));
document.getElementById("stopbtn").onclick = () => post("/api/stop");

tick();
setInterval(tick, 200);
</script>
</body>
</html>
"""


# ----------------------------- HTTP 服务 -----------------------------


class Handler(BaseHTTPRequestHandler):
    link = None

    def log_message(self, *args):     # 安静点，不往控制台刷访问日志
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            page = PAGE.replace("%MODE_LED%", json.dumps(MODE_LED))
            self._send(200, page, "text/html; charset=utf-8")
        elif self.path.startswith("/api/state"):
            self._send(200, json.dumps(self.link.snapshot(), ensure_ascii=False))
        else:
            self._send(404, json.dumps({"ok": False, "error": "not found"}))

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw.decode("utf-8") or "{}")
        except ValueError:
            body = {}

        if self.path.startswith("/api/mode"):
            mode = int(body.get("mode", 0))
            if mode < 0 or mode > 2:
                self._send(400, json.dumps({"ok": False, "error": "mode 只能是 0/1/2"}))
                return
            ok = self.link.send(self.link.frame_set_mode(mode))
            self._send(200, json.dumps({"ok": ok, "mode": mode}))

        elif self.path.startswith("/api/cmd"):
            vx = int(body.get("vx", 0))
            wz = int(body.get("wz", 0))
            ok = self.link.send(self.link.frame_speed(vx, wz))
            self._send(200, json.dumps({"ok": ok, "vx": vx, "wz": wz}))

        elif self.path.startswith("/api/stop"):
            ok = self.link.send(self.link.frame_speed(0, 0))
            self._send(200, json.dumps({"ok": ok}))

        else:
            self._send(404, json.dumps({"ok": False, "error": "not found"}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM18", help="板子调试口, 如 COM18")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 则局域网可访问")
    ap.add_argument("--web-port", type=int, default=8010)
    args = ap.parse_args()

    link = BoardLink(args.port, args.baud)
    try:
        link.start()
    except Exception as exc:
        raise SystemExit("打开串口 %s 失败: %s（是不是 v2_monitor.py 还占着？）" % (args.port, exc))

    Handler.link = link
    server = ThreadingHTTPServer((args.host, args.web_port), Handler)
    print("串口 %s @%d  网页 http://%s:%d" % (args.port, args.baud, args.host, args.web_port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        link.stop_evt.set()
        server.server_close()


if __name__ == "__main__":
    main()
