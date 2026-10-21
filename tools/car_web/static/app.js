/* 小车遥控器前端：原生 JS，无构建、无外网依赖。
 *
 * 三个不变量（改代码时别破坏）：
 *   1. 只有"摇杆被按住 / 键盘按住"时目标才非 0；松手/回中发 0（后端软停），
 *      页面隐藏/失焦/关闭发 0 + hard:true（后端立即停），断连一律停发。
 *   2. 指令按 20 Hz 持续发，服务端据此判活；停发 = 500 ms 后服务端自己归零。
 *   3. 急停是锁存的：本地立刻归零 + 通知服务端锁存，必须点"复位"才恢复。
 */
(() => {
  "use strict";

  // 转向指令符号 —— 整个项目唯一一处"驾驶意图 → 协议 wz"的取反点。
  //
  // 固件逆解：v_left = vx - wz*TRACK/2，v_right = vx + wz*TRACK/2，
  // 所以 wz > 0 = 左轮慢/右轮快 = 车向左转（逆时针）。
  // 要让"网页右推摇杆 = 车右转"，就必须发 wz < 0。
  //
  // 约定：本文件的 nx / x 是**驾驶意图**（右为正）；WZ_CMD_SIGN 只在
  // steerWz() 里乘一次，产生发给后端的协议值。后端 car_web.py 原样送到
  // 串口、遥测/轮速按实际值显示，都不再取反。以后方向要改，只改这里。
  const WZ_CMD_SIGN = -1;

  // 与服务端 GEARS 一致（6 档）；这里也夹一次，保证界面显示的就是实际会发的值。
  // 默认 low：第一次打开页面、第一次推杆不会蹿出去。
  const GEARS = {
    low:    { name: "低速", vx: 0.15, wz: 0.50 },
    mid:    { name: "中速", vx: 0.30, wz: 0.75 },
    high:   { name: "高速", vx: 0.50, wz: 1.00 },
    fast:   { name: "快速", vx: 0.80, wz: 1.15 },
    sprint: { name: "极速", vx: 1.20, wz: 1.30 },
    ultra:  { name: "超快", vx: 1.60, wz: 1.50 },
  };
  const GEAR_KEYS = ["low", "mid", "high", "fast", "sprint", "ultra"];

  // 高速视觉提示：≥0.8 m/s 橙、≥1.2 m/s 红（README 也写了"高速档请空旷场地测试"）。
  const SPEED_WARN = 0.8;
  const SPEED_DANGER = 1.2;

  // 条形图满量程：固件单轮上限是 30 rad/s（≈2.57 m/s），但正常遥控只会用到
  // 0~5 rad/s；按 30 画条子几乎看不出变化，所以显示用 ±6，超出贴边。
  const W_OMEGA_MAX = 6.0;
  const TQ_MAX = 10.0;        // 扭矩条形图满量程（±10 Nm，再大就贴边）
  const CMD_HZ = 20;          // 网页指令频率
  const WAVE_N = 150;         // 波形保留点数（20 Hz × 7.5 s）

  const $ = (id) => document.getElementById(id);
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
  const fmt = (v, n) => (v >= 0 ? "+" : "") + Number(v).toFixed(n);

  // 驾驶意图（右为正，[-1,1]）→ 协议 wz。唯一取反点，见 WZ_CMD_SIGN。
  function steerWz(steer, max) {
    return WZ_CMD_SIGN * clamp(steer, -1, 1) * max;
  }

  const S = {
    ws: null,
    connected: false,
    gear: localStorage.getItem("carweb.gear") || "low",
    estop: false,
    // 缓启缓停：服务端默认开，这里只是界面镜像；以 /api/state 的 ramp.on 为准。
    rampOn: true,
    rampPending: null,          // 刚点开关、还没收到服务端回显时的期望值
    rampPendingAt: 0,
    target: { vx: 0, wz: 0 },
    dragId: null,
    keys: new Set(),
    last: null,
    lastCmdSent: 0,
    reconnectDelay: 500,
    lastEventsJson: "",
    wave: { gz: [], ax: [] },
    canvasW: 0,
    canvasH: 0,
  };
  if (!GEARS[S.gear]) S.gear = "low";

  // ---------------------------------------------------------------------
  // 连接
  // ---------------------------------------------------------------------
  function connect() {
    if (S.ws && (S.ws.readyState === WebSocket.CONNECTING || S.ws.readyState === WebSocket.OPEN)) return;
    const proto = location.protocol === "https:" ? "wss" : "ws";
    let ws;
    try {
      ws = new WebSocket(`${proto}://${location.host}/ws`);
    } catch (err) {
      scheduleReconnect();
      return;
    }
    S.ws = ws;

    ws.onopen = () => {
      S.connected = true;
      S.reconnectDelay = 500;
      send({ t: "gear", gear: S.gear });
      // 断线期间点过急停：重连时补一次，服务端也要处于锁存状态
      if (S.estop) send({ t: "estop" });
      startCmdLoop();
      renderSendState(S.last);
    };

    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (err) { return; }
      if (msg && msg.t === "state") applyState(msg);
    };

    ws.onclose = () => {
      S.connected = false;
      stopCmdLoop();
      resetKnob();
      S.dragId = null;
      $("pad").classList.remove("active");
      renderSendState(S.last);
      updateBadges(S.last);
      scheduleReconnect();
    };

    ws.onerror = () => { try { ws.close(); } catch (err) {} };
  }

  function scheduleReconnect() {
    setTimeout(connect, S.reconnectDelay);
    S.reconnectDelay = Math.min(5000, Math.round(S.reconnectDelay * 1.7));
  }

  function send(obj) {
    if (!S.ws || S.ws.readyState !== WebSocket.OPEN) return false;
    try { S.ws.send(JSON.stringify(obj)); return true; } catch (err) { return false; }
  }

  // ---------------------------------------------------------------------
  // 指令
  // ---------------------------------------------------------------------
  function limits() { return GEARS[S.gear] || GEARS.low; }

  function renderSpeedClass(vx) {
    const el = $("cmdVx");
    if (!el) return;
    const a = Math.abs(vx);
    el.classList.toggle("speed-warn", a >= SPEED_WARN && a < SPEED_DANGER);
    el.classList.toggle("speed-danger", a >= SPEED_DANGER);
  }

  function applyTarget(vx, wz, force) {
    if (S.estop) { vx = 0; wz = 0; }
    const g = limits();
    S.target.vx = clamp(vx, -g.vx, g.vx);
    S.target.wz = clamp(wz, -g.wz, g.wz);
    $("cmdVx").textContent = fmt(S.target.vx, 3);
    $("cmdWz").textContent = fmt(S.target.wz, 3);
    renderSpeedClass(S.target.vx);
    const now = performance.now();
    // 指针移动事件可能到 60~120 Hz，限到 30 Hz 再发；20 Hz 的定时器兜底。
    if (force || now - S.lastCmdSent >= 33) sendCmd();
  }

  function sendCmd() {
    if (!S.connected) return;
    if (send({ t: "cmd", vx: S.target.vx, wz: S.target.wz })) {
      S.lastCmdSent = performance.now();
    }
  }

  let cmdTimer = null;
  function startCmdLoop() {
    stopCmdLoop();
    // 持续发（包括 0）：既是控制，也是"网页还活着"的心跳。
    cmdTimer = setInterval(sendCmd, 1000 / CMD_HZ);
  }
  function stopCmdLoop() {
    if (cmdTimer) { clearInterval(cmdTimer); cmdTimer = null; }
  }

  // 普通停车（松手/回中/主动停止）：目标发 0，后端按减速斜率软停。
  function zeroTarget() {
    S.target.vx = 0;
    S.target.wz = 0;
    $("cmdVx").textContent = fmt(0, 3);
    $("cmdWz").textContent = fmt(0, 3);
    renderSpeedClass(0);
    sendCmd();
  }

  // 安全停车（页面隐藏/失焦/pagehide）：带 hard:true 发 0，后端立即归零，
  // 不经过减速斜坡 —— 此时用户可能已经看不到车了。
  function hardStop() {
    S.target.vx = 0;
    S.target.wz = 0;
    $("cmdVx").textContent = fmt(0, 3);
    $("cmdWz").textContent = fmt(0, 3);
    renderSpeedClass(0);
    send({ t: "cmd", vx: 0, wz: 0, hard: true });
  }

  // ---------------------------------------------------------------------
  // 摇杆（鼠标 + 触摸统一走 Pointer Events）
  // ---------------------------------------------------------------------
  const pad = $("pad");
  const knob = $("knob");

  function knobGeometry() {
    const r = pad.getBoundingClientRect();
    return { cx: r.left + r.width / 2, cy: r.top + r.height / 2, max: Math.max(20, r.width / 2 - 38) };
  }

  function moveKnob(dx, dy) {
    const g = knobGeometry();
    const m = Math.hypot(dx, dy);
    const k = m > g.max ? g.max / m : 1;
    knob.style.transform = `translate(${dx * k}px, ${dy * k}px)`;
  }

  function resetKnob() {
    // 键盘控制时也把旋钮挪到对应位置，视觉和实际指令一致
    if (S.keys.size) { updateFromKeys(); return; }
    knob.style.transform = "translate(0px, 0px)";
  }

  function updateFromPointer(ev) {
    const g = knobGeometry();
    let dx = ev.clientX - g.cx;
    let dy = ev.clientY - g.cy;
    const m = Math.hypot(dx, dy);
    if (m > g.max) { dx = dx * g.max / m; dy = dy * g.max / m; }
    moveKnob(dx, dy);

    const nx = dx / g.max;
    const ny = dy / g.max;
    const mag = Math.hypot(nx, ny);
    if (mag < 0.10) {                     // 中心死区，避免手抖让车蠕动
      applyTarget(0, 0);
      return;
    }
    const lim = limits();
    // 屏幕坐标：上 = 前进(+vx)，右 = 右转。右转对应协议 wz<0，
    // 由 steerWz() 里的 WZ_CMD_SIGN 统一取反，这里不要写裸负号。
    applyTarget(-ny * lim.vx, steerWz(nx, lim.wz));
  }

  pad.addEventListener("pointerdown", (ev) => {
    if (ev.button !== undefined && ev.button !== 0) return;
    if (S.estop) return;
    ev.preventDefault();
    S.dragId = ev.pointerId;
    pad.classList.add("active");
    try { pad.setPointerCapture(ev.pointerId); } catch (err) {}
    updateFromPointer(ev);
  });

  pad.addEventListener("pointermove", (ev) => {
    if (S.dragId !== ev.pointerId) return;
    ev.preventDefault();
    updateFromPointer(ev);
  });

  function endDrag(ev) {
    if (S.dragId !== ev.pointerId) return;
    S.dragId = null;
    pad.classList.remove("active");
    resetKnob();
    applyTarget(0, 0, true);              // 松手立刻发 0，不等定时器
  }
  pad.addEventListener("pointerup", endDrag);
  pad.addEventListener("pointercancel", endDrag);
  pad.addEventListener("lostpointercapture", endDrag);
  pad.addEventListener("contextmenu", (ev) => ev.preventDefault());
  pad.addEventListener("dragstart", (ev) => ev.preventDefault());

  // ---------------------------------------------------------------------
  // 键盘（桌面/无触摸屏时方便，也是在 Windows 上自测的主要手段）
  // ---------------------------------------------------------------------
  const KEY_MAP = {
    KeyW: [0, -1], KeyS: [0, 1], KeyA: [-1, 0], KeyD: [1, 0],
    ArrowUp: [0, -1], ArrowDown: [0, 1], ArrowLeft: [-1, 0], ArrowRight: [1, 0],
  };

  function updateFromKeys() {
    if (S.dragId !== null) return;
    let x = 0, y = 0;
    if (S.keys.has("KeyW") || S.keys.has("ArrowUp")) y -= 1;
    if (S.keys.has("KeyS") || S.keys.has("ArrowDown")) y += 1;
    if (S.keys.has("KeyA") || S.keys.has("ArrowLeft")) x -= 1;
    if (S.keys.has("KeyD") || S.keys.has("ArrowRight")) x += 1;
    const g = knobGeometry();
    moveKnob(x * g.max * 0.82, y * g.max * 0.82);
    if (x === 0 && y === 0) { applyTarget(0, 0); return; }
    const lim = limits();
    // 斜向不做 0.707 缩放：键盘本来就是离散控制，满幅更直观。
    // 键盘 D / → 是"右"，同样走 steerWz() 的唯一取反点。
    applyTarget(-y * lim.vx, steerWz(x, lim.wz));
  }

  window.addEventListener("keydown", (ev) => {
    const tag = (ev.target && ev.target.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (ev.code === "Space") {
      ev.preventDefault();
      S.keys.clear();
      resetKnob();
      applyTarget(0, 0, true);
      return;
    }
    if (/^Digit[1-6]$/.test(ev.code)) {
      setGear(GEAR_KEYS[Number(ev.code.slice(-1)) - 1]);
      return;
    }
    if (KEY_MAP[ev.code]) {
      ev.preventDefault();
      if (S.estop) return;
      S.keys.add(ev.code);
      updateFromKeys();
    }
  });

  window.addEventListener("keyup", (ev) => {
    if (!KEY_MAP[ev.code]) return;
    S.keys.delete(ev.code);
    updateFromKeys();
  });

  // ---------------------------------------------------------------------
  // 档位 / 急停
  // ---------------------------------------------------------------------
  function renderGear() {
    document.querySelectorAll(".gear-btn").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.gear === S.gear);
    });
    const g = limits();
    $("limitPill").textContent = `${g.name} · ${g.vx.toFixed(2)} m/s · ${g.wz.toFixed(2)} rad/s`;
    const pill = $("limitPill");
    pill.classList.toggle("warn", g.vx >= SPEED_WARN && g.vx < SPEED_DANGER);
    pill.classList.toggle("danger", g.vx >= SPEED_DANGER);
    // 高速档醒目标签：≥0.8 橙、≥1.2 红。
    const warn = $("gearWarn");
    if (warn) {
      const fast = g.vx >= SPEED_DANGER ? "danger" : (g.vx >= SPEED_WARN ? "warn" : "");
      warn.className = "gear-warn" + (fast ? " " + fast : "");
      warn.hidden = !fast;
      warn.textContent = g.vx >= SPEED_DANGER
        ? `⚠ ${g.name} ${g.vx.toFixed(2)} m/s · 高速危险，只在空旷场地测试`
        : (fast ? `⚠ ${g.name} ${g.vx.toFixed(2)} m/s · 请在空旷场地测试` : "");
    }
    renderSpeedClass(S.target.vx);
  }

  function setGear(key) {
    if (!GEARS[key] || key === S.gear) return;
    S.gear = key;
    localStorage.setItem("carweb.gear", key);
    send({ t: "gear", gear: key });
    renderGear();
    // 新档位可能更小，立刻把当前目标压回范围内
    applyTarget(S.target.vx, S.target.wz, true);
  }

  document.querySelectorAll(".gear-btn").forEach((btn) => {
    btn.addEventListener("click", () => setGear(btn.dataset.gear));
  });

  // ---------------------------------------------------------------------
  // 缓启缓停开关（真正的斜坡在后端；前端只负责开关状态和显示）
  // ---------------------------------------------------------------------
  function renderRamp() {
    const sw = $("rampToggle");
    if (sw) sw.checked = !!S.rampOn;
    const state = $("rampState");
    if (state) {
      state.textContent = S.rampOn ? "开" : "关";
      state.className = "ramp-state " + (S.rampOn ? "on" : "off");
    }
    const row = $("rampRow");
    if (row) row.classList.toggle("on", !!S.rampOn);
    const hint = $("rampHint");
    if (hint) {
      hint.textContent = S.rampOn
        ? "加速 1.0、减速 0.40 m/s²；松手/回中会“缓停中”，切页/失焦/急停/超时立即归零"
        : "已关闭：加速和停车都直接阶跃到目标（旧行为）";
    }
  }

  function setRamp(on) {
    S.rampOn = !!on;
    S.rampPending = !!on;
    S.rampPendingAt = performance.now();
    renderRamp();
    // 服务端会在 20 Hz 的 state 里回显 ramp.on；断线时开关只改本地显示，
    // 重连后以服务端状态为准。
    send({ t: "ramp", on: S.rampOn });
  }

  $("rampToggle").addEventListener("change", (ev) => setRamp(ev.target.checked));

  $("btnEstop").addEventListener("click", () => {
    S.estop = true;               // 本地先禁摇杆，不等服务器往返
    S.keys.clear();
    S.dragId = null;
    pad.classList.remove("active");
    resetKnob();
    zeroTarget();
    send({ t: "estop" });
    updateEstopUI();
    renderSendState(S.last);
  });

  $("btnReset").addEventListener("click", () => {
    send({ t: "estop_reset" });
    $("btnReset").hidden = true;
  });

  $("btnStop").addEventListener("click", () => {
    S.keys.clear();
    S.dragId = null;
    pad.classList.remove("active");
    resetKnob();
    applyTarget(0, 0, true);
  });

  $("btnOdomReset").addEventListener("click", () => send({ t: "odom_reset" }));

  function updateEstopUI() {
    $("btnEstop").classList.toggle("active", S.estop);
    $("btnReset").hidden = !S.estop;
  }

  // ---------------------------------------------------------------------
  // 状态渲染
  // ---------------------------------------------------------------------
  function text(id, value) {
    const el = $(id);
    if (el && el.textContent !== value) el.textContent = value;
  }

  function updateBadges(m) {
    const wsBadge = $("wsBadge");
    wsBadge.classList.toggle("ok", S.connected);
    wsBadge.classList.toggle("bad", !S.connected);
    const linkOk = !!(m && m.link && m.link.open);
    const linkBadge = $("linkBadge");
    linkBadge.classList.toggle("ok", linkOk);
    linkBadge.classList.toggle("bad", !linkOk);
    $("simBadge").hidden = !(m && m.sim);
  }

  function renderSendState(m) {
    const dot = $("sendDot");
    const txt = $("sendText");
    dot.className = "send-dot";
    if (!S.connected) {
      dot.classList.add("bad");
      txt.textContent = "未连接服务器：不会发任何指令（正在重连）";
      return;
    }
    if (!m) { txt.textContent = "已连接，等待第一帧状态…"; return; }
    const reason = m.cmd ? m.cmd.reason : "timeout";
    if (reason === "estop") {
      dot.classList.add("bad");
      txt.textContent = "急停锁存中：只发 0，点“复位”才能恢复";
    } else if (reason === "timeout") {
      dot.classList.add("warn");
      txt.textContent = "服务端 500 ms 未收到指令，已自动归零";
    } else {
      dot.classList.add("ok");
      txt.textContent = `正在发送 · ${CMD_HZ} Hz · 实发 vx ${fmt(m.cmd.tx_vx, 3)} / wz ${fmt(m.cmd.tx_wz, 3)}`;
    }
  }

  function setBar(id, value, max) {
    const el = $(id);
    const pct = clamp(value / max, -1, 1) * 50;
    if (pct >= 0) {
      el.style.left = "50%";
      el.style.width = pct + "%";
    } else {
      el.style.left = (50 + pct) + "%";
      el.style.width = (-pct) + "%";
    }
    el.classList.toggle("neg", value < 0);
  }

  function setMark(id, value, max) {
    $(id).style.left = (50 + clamp(value / max, -1, 1) * 50) + "%";
  }

  function renderWheel(i, w) {
    const p = "w" + i;
    text(p + "omega", fmt(w.omega, 2));
    text(p + "target", fmt(w.target, 2));
    setBar(p + "fill", w.omega, W_OMEGA_MAX);
    setMark(p + "mark", w.target, W_OMEGA_MAX);

    text(p + "torque", fmt(w.torque, 2));
    setBar(p + "tqfill", w.torque, TQ_MAX);

    const on = $(p + "online");
    on.textContent = w.online ? "在线" : "离线";
    on.className = "chip " + (w.online ? "ok" : "bad");

    const en = $(p + "enable");
    en.textContent = w.enabled ? "使能" : "失能";
    en.className = "chip " + (w.enabled ? "ok" : "");

    const err = $(p + "err");
    err.textContent = `ERR ${w.err} · ${w.err_text}`;
    err.className = "err " + (w.err > 1 ? "bad" : "ok");

    const temp = $(p + "temp");
    temp.textContent = `MOS ${w.mos_temp}℃ · 线圈 ${w.coil_temp}℃`;
    const hottest = Math.max(w.mos_temp, w.coil_temp);
    temp.className = "temp " + (hottest >= 75 ? "bad" : hottest >= 60 ? "warn" : "");

    $("wheel" + i).classList.toggle("online", !!w.online);

    // 反馈年龄：>500 ms 基本等于掉线，用黄色提示（红色交给"离线"chip）
    if (w.fb_age_ms >= 500 || w.fb_age_ms === 0xFFFF) {
      temp.classList.add("warn");
      temp.textContent += " · 反馈陈旧";
    }
  }

  function renderFlags(m) {
    const box = $("flags");
    const flags = m.flags || [];
    if (box.children.length !== flags.length) {
      box.textContent = "";
      flags.forEach((f) => {
        const span = document.createElement("span");
        span.className = "flag";
        span.dataset.key = f.key;
        const dot = document.createElement("i");
        dot.className = "dot";
        span.appendChild(dot);
        span.appendChild(document.createTextNode(f.label));
        box.appendChild(span);
      });
    }
    flags.forEach((f) => {
      const el = box.querySelector(`[data-key="${f.key}"]`);
      if (el) el.className = "flag " + (f.good ? "ok" : "bad");
    });
    text("flagsRaw", m.status ? `flags 0x${m.status.flags_raw.toString(16).padStart(2, "0").toUpperCase()}` : "flags —");
  }

  function renderBanner(m) {
    const banner = $("banner");
    const txt = $("bannerText");
    let msg = "";
    let bad = false;

    if (!S.connected) {
      msg = "⚠ 与服务器断开：已停止发送指令，正在重连…";
      bad = true;
    } else if (m && m.safety && m.safety.estop) {
      msg = "🛑 急停已锁存：目标恒为 0。确认现场安全后点右上角“复位”。";
      bad = true;
    } else if (m && m.link && !m.link.open) {
      msg = `⚠ 串口未打开：${m.link.error || "未知原因"}。遥控指令发不出去。`;
      bad = true;
    } else if (m && m.link && m.link.rx_age_ms !== null && m.link.rx_age_ms > 1000) {
      msg = `⚠ 串口已打开但 ${m.link.rx_age_ms} ms 没有遥测帧：检查 STM32 是否上电、TX/RX 是否接反。`;
      bad = false;
    } else if (m && m.status) {
      const lf = m.status.left, rf = m.status.right;
      if (lf.fault || rf.fault) {
        msg = `⚠ 电机故障：左 ${lf.err_text} / 右 ${rf.err_text}。请断电检查后再试。`;
        bad = true;
      } else if (m.status.flags.cmd_timeout && m.cmd && m.cmd.has_client) {
        msg = "⚠ STM32 报告命令超时（800 ms）：命令帧没送达，检查串口/波特率。";
        bad = false;
      }
    }

    banner.hidden = !msg;
    banner.classList.toggle("bad", bad);
    if (msg) txt.textContent = msg;
  }

  function renderImu(tel, st) {
    if (!tel) {
      ["ax", "ay", "az", "gx", "gy", "gz"].forEach((k) => text(k, "—"));
      text("tilt", "—");
      text("imuHint", "等待数据");
      return;
    }
    text("ax", tel.ax.toFixed(2));
    text("ay", tel.ay.toFixed(2));
    text("az", tel.az.toFixed(2));
    text("gx", tel.gx.toFixed(3));
    text("gy", tel.gy.toFixed(3));
    text("gz", tel.gz.toFixed(3));

    const k = 1 / 4.0;   // ±4 m/s² 对应圆边
    const ox = clamp(tel.ax * k, -1, 1) * 30;
    const oy = clamp(tel.ay * k, -1, 1) * 30;
    $("bubbleDot").style.transform = `translate(${ox}px, ${oy}px)`;
    const tilt = Math.atan2(Math.hypot(tel.ax, tel.ay), Math.abs(tel.az) || 9.80665) * 180 / Math.PI;
    text("tilt", tilt.toFixed(1) + "°");

    if (st) {
      text("imuHint", `${st.imu_status_text} · 地址 0x${st.imu_addr.toString(16).toUpperCase()} · ${st.imu_gyro_ok ? "陀螺已读到" : "陀螺未读到"}`);
    }
  }

  function renderOdom(m) {
    const od = m.odom || {};
    const tel = m.telemetry;
    text("distance", Number(od.distance || 0).toFixed(2));
    // 角度按罗盘习惯折到 [-180, 180)：里程里积分的 yaw 是无界的，
    // 连续转几圈后直接显示会出现 -235° 这种读数。
    const deg = Number(od.yaw || 0) * 180 / Math.PI;
    text("yaw", ((((deg + 180) % 360) + 360) % 360 - 180).toFixed(1));
    text("bodyV", tel ? tel.vx.toFixed(2) : "0.00");
    text("battery", tel && tel.bat_mv > 0 ? String(tel.bat_mv) : "—");
  }

  function renderDiag(m) {
    const link = m.link || {};
    const st = m.status;
    text("dPort", m.sim ? "内部仿真（不占用串口）" : `${link.port} @ ${link.baud}`);
    text("dRxAge", link.rx_age_ms === null || link.rx_age_ms === undefined ? "—" : `${link.rx_age_ms} ms`);
    text("dTx", `${link.tx_hz} Hz · ${link.tx_count} 帧`);
    text("dFrames", `遥测 ${link.telemetry_count} · 状态 ${link.status_count}`);
    text("dBad", `${link.bad_frames} / ${link.resync_bytes} B`);
    text("dEcho", st ? `vx ${fmt(st.cmd_vx, 3)} · wz ${fmt(st.cmd_wz, 3)}` : "—");
    text("dBoot", st ? st.boot_stage_text : "—");
    text("dImu", st ? `${st.imu_status_text} · ver ${st.imu_ver}` : "—");
    text("dCan", st ? `${st.can_fail} / ${st.fb_count}` : "—");
  }

  function renderEvents(m) {
    const list = m.events || [];
    const key = JSON.stringify(list);
    if (key === S.lastEventsJson) return;
    S.lastEventsJson = key;
    const ul = $("events");
    ul.textContent = "";
    if (!list.length) {
      const li = document.createElement("li");
      li.className = "ev";
      li.textContent = "—";
      ul.appendChild(li);
      return;
    }
    list.slice().reverse().forEach((ev) => {
      const li = document.createElement("li");
      const t = document.createElement("span");
      t.className = "t";
      t.textContent = ev.t;
      const s = document.createElement("span");
      s.textContent = ev.msg;
      li.appendChild(t);
      li.appendChild(s);
      ul.appendChild(li);
    });
  }

  function applyState(m) {
    if (!m || m.t !== "state") return;
    S.last = m;

    updateBadges(m);

    const link = m.link || {};
    const sim = m.sim ? "仿真" : "真机";
    const target = m.sim ? "内部仿真链路" : `${link.port} @${link.baud}`;
    let sub = `${sim} · ${target} · ${link.tx_hz} Hz`;
    if (!link.open) sub += " · 串口未打开";
    else if (link.rx_age_ms > 1000) sub += " · 无遥测";
    text("subtitle", sub);

    if (m.gear && m.gear.key !== S.gear) {
      S.gear = m.gear.key;
      localStorage.setItem("carweb.gear", S.gear);
      renderGear();
    }

    // 缓启缓停：以服务端为准；刚点开关的 ~1.5 s 内用本地期望值，
    // 避免服务端回显慢半拍导致开关闪一下。
    if (m.ramp) {
      const srvOn = !!m.ramp.on;
      if (S.rampPending !== null) {
        if (srvOn === S.rampPending || performance.now() - S.rampPendingAt > 1500) {
          S.rampPending = null;
          S.rampOn = srvOn;
          renderRamp();
        }
      } else if (srvOn !== S.rampOn) {
        S.rampOn = srvOn;
        renderRamp();
      }
      const live = $("rampLive");
      const mode = m.ramp.mode || (m.ramp.active ? "accel" : "idle");
      const slowing = mode === "decel";
      if (live) {
        live.textContent = slowing ? "缓停中"
          : (mode === "accel" ? "斜坡中"
            : (mode === "hard" && m.cmd && m.cmd.reason === "web" ? "立即停" : ""));
        live.className = "ramp-live" + (slowing ? " warn" : "");
      }
      const row = $("rampRow");
      if (row) {
        row.classList.toggle("ramping", !!m.ramp.active);
        row.classList.toggle("stopping", slowing);
      }
    }

    const wasEstop = S.estop;
    S.estop = !!(m.safety && m.safety.estop);
    if (wasEstop && !S.estop) {
      // 服务端已复位：本地目标保持 0，必须重新推摇杆才动
      zeroTarget();
      S.lastCmdSent = performance.now();
    }
    updateEstopUI();

    const st = m.status;
    const tel = m.telemetry;
    if (st) {
      renderWheel(0, st.left);
      renderWheel(1, st.right);
    }
    renderFlags(m);
    renderImu(tel, st);
    renderOdom(m);
    renderDiag(m);
    renderEvents(m);
    renderBanner(m);
    renderSendState(m);

    if (tel) pushWave(tel);
    drawWave();
  }

  // ---------------------------------------------------------------------
  // 波形（canvas，无第三方库）
  // ---------------------------------------------------------------------
  function sizeCanvas() {
    const c = $("wave");
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(10, c.clientWidth);
    const h = Math.max(10, c.clientHeight);
    if (S.canvasW === w && S.canvasH === h) return;
    S.canvasW = w;
    S.canvasH = h;
    c.width = Math.round(w * dpr);
    c.height = Math.round(h * dpr);
    const ctx = c.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function pushWave(tel) {
    S.wave.gz.push(tel.gz);
    S.wave.ax.push(tel.ax);
    if (S.wave.gz.length > WAVE_N) {
      S.wave.gz.shift();
      S.wave.ax.shift();
    }
  }

  function drawWave() {
    const c = $("wave");
    if (!c) return;
    sizeCanvas();
    const ctx = c.getContext("2d");
    const w = S.canvasW, h = S.canvasH;
    ctx.clearRect(0, 0, w, h);

    // 网格 + 零线
    ctx.strokeStyle = "rgba(255,255,255,.055)";
    ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
      const y = (h * i) / 4;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }
    ctx.strokeStyle = "rgba(255,255,255,.14)";
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();

    const drawTrace = (arr, scale, color, width) => {
      if (arr.length < 2) return;
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineJoin = "round";
      arr.forEach((v, i) => {
        const x = (i / (WAVE_N - 1)) * w;
        const y = h / 2 - clamp(v / scale, -1, 1) * (h / 2 - 4);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
    };
    drawTrace(S.wave.ax, 2.0, "rgba(251,146,60,.75)", 1.2);
    drawTrace(S.wave.gz, 2.0, "rgba(34,211,238,.95)", 1.8);
  }

  window.addEventListener("resize", () => { S.canvasW = S.canvasH = 0; drawWave(); });

  // ---------------------------------------------------------------------
  // 页面生命周期：切后台/失焦/关闭 = **立即停**（安全停车，不软停）
  //
  // 和"松手/回中"的区别：
  //   * 松手/回中：用户还在看着页面，走减速斜坡，手感软；
  //   * 页面隐藏/失焦/关闭：用户可能已经看不到车了，必须立即归零。
  // 实现上给后端发 hard:true，后端在 step_output() 里直接置 0。
  // ---------------------------------------------------------------------
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      S.keys.clear();
      S.dragId = null;
      pad.classList.remove("active");
      resetKnob();
      hardStop();
    }
  });

  window.addEventListener("blur", () => {
    S.keys.clear();
    if (S.dragId === null) resetKnob();
    hardStop();
  });

  window.addEventListener("pagehide", () => {
    hardStop();
    if (S.ws) { try { S.ws.close(); } catch (err) {} }
  });

  // ---------------------------------------------------------------------
  // 启动
  // ---------------------------------------------------------------------
  renderGear();
  renderRamp();
  updateEstopUI();
  updateBadges(null);
  renderSendState(null);
  sizeCanvas();
  drawWave();
  connect();
})();
