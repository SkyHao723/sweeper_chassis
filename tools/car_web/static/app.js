/* 小车遥控器前端：原生 JS，无构建、无外网依赖。
 *
 * 三个不变量（改代码时别破坏）：
 *   1. 只有"摇杆被按住 / 键盘按住"时目标才非 0；松手/回中发 0（后端软停），
 *      页面隐藏/失焦/关闭发 0 + hard:true（后端立即停），断连一律停发。
 *   2. 指令按 30 Hz 持续发（requestAnimationFrame 兜底 + pointermove 事件驱动），
 *      服务端据此判活；停发 = 500 ms 后服务端开始分级衰减并归零。
 *   3. 急停是锁存的：本地立刻归零 + 通知服务端锁存，必须点"复位"才恢复。
 *
 * 为什么不用 setInterval 发指令（1.4.0 改）：
 *   手机浏览器在触摸滚动/省电/省流模式下会把 setInterval 拉长到 100 ms~1 s，
 *   20 Hz 心跳一旦被拉长到 >500 ms，后端就判超时 → 车"加速-停-加速"。
 *   rAF 跟着屏幕刷新，触摸时不会被节流；再叠加 pointermove 里立即发一次，
 *   即使 rAF 被系统暂停，事件流也能把最新目标送出去。
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
  const CMD_HZ = 30;          // 网页指令心跳频率（rAF 驱动）
  const CMD_GAP_MS = 1000 / CMD_HZ;       // rAF 心跳最小间隔（33 ms）
  const CMD_EVENT_GAP_MS = 20;            // 触摸/键盘事件里最多 50 Hz，避免刷爆 WS
  const DIAG_WINDOW_MS = 2000;            // 链路抖动诊断窗口（和 /status 一致）
  const DEADZONE_ENTER = 0.10;            // 居中 → 开始控制：需要推过 10%
  const DEADZONE_EXIT = 0.06;             // 控制中 → 回中：低于 6% 才归零（防抖）
  const WAVE_N = 150;         // 波形保留点数（20 Hz 状态推送 × 7.5 s）

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
    deadzoneActive: false,      // 双阈值滞回，避免手指在死区边缘来回抖
    keys: new Set(),
    last: null,
    lastCmdSent: 0,
    reconnectDelay: 500,
    lastEventsJson: "",
    flagSig: "",
    flagEls: Object.create(null),
    wave: { gz: [], ax: [] },
    canvasW: 0,
    canvasH: 0,
    canvasDirty: true,
    canvasObs: null,
    padGeom: null,              // 拖动期间缓存摇杆几何，避免每帧 getBoundingClientRect

    // ---- 指令会话/序号：后端据此识别"旧包/乱序/页面重载" ----
    seq: 0,
    // 每一次页面加载一个会话 id：换页/刷新后 seq 从 0 重新开始，
    // 后端看到 session 变了就重置基线，不会把新页面的指令当成旧包丢掉。
    session: "s" + Math.random().toString(36).slice(2, 10),

    // ---- 链路抖动诊断（最近 2 s 滚动窗口）----
    sendTimes: [],              // 真正发出 cmd 的时刻
    touchTimes: [],             // 摇杆 pointermove 到达前台的时刻
    frameTimes: [],             // rAF 时刻：用来看主线程有没有卡顿
    lastDiagRender: 0,
    rafId: 0,
    watchdogId: 0,
    hardStops: { blur: 0, hidden: 0, pagehide: 0 },
    cancels: { pointercancel: 0, lostcapture: 0, pointerup: 0 },
    lastPointerType: "-",
  };
  if (!GEARS[S.gear]) S.gear = "low";

  // 2 秒滑动窗口：只保留窗口内的样本（数组很小，最多 100 个）。
  function pruneSamples(arr, now) {
    while (arr.length && now - arr[0] > DIAG_WINDOW_MS) arr.shift();
  }
  function pushSample(arr, t) {
    arr.push(t);
    pruneSamples(arr, t);
  }

  // 间隔统计：min/avg/max + 最大抖动（每个间隔与平均值之差的最大绝对值）。
  function intervalStats(arr) {
    const n = arr.length;
    if (n < 2) return { n: n, min: null, avg: null, max: null, jitter: null };
    let min = Infinity, max = 0, sum = 0;
    for (let i = 1; i < n; i++) {
      const d = arr[i] - arr[i - 1];
      if (d < min) min = d;
      if (d > max) max = d;
      sum += d;
    }
    const avg = sum / (n - 1);
    let jitter = 0;
    for (let i = 1; i < n; i++) {
      const dev = Math.abs((arr[i] - arr[i - 1]) - avg);
      if (dev > jitter) jitter = dev;
    }
    return {
      n: n,
      min: Math.round(min * 10) / 10,
      avg: Math.round(avg * 10) / 10,
      max: Math.round(max * 10) / 10,
      jitter: Math.round(jitter * 10) / 10,
    };
  }

  function msText(v) {
    if (v === null || v === undefined) return "—";
    return v < 10 ? v.toFixed(1) : String(Math.round(v));
  }

  function statsText(label, s) {
    if (!s || s.n < 2) return `${label} —`;
    return `${label} ${msText(s.min)}/${msText(s.avg)}/${msText(s.max)}ms 抖${msText(s.jitter)}`;
  }


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
    text("cmdVx", fmt(S.target.vx, 3));
    text("cmdWz", fmt(S.target.wz, 3));
    renderSpeedClass(S.target.vx);
    const now = performance.now();
    // 事件驱动：触摸/键盘一改目标就立刻发（最多 50 Hz），不等下一帧 rAF。
    // rAF 那条路只负责"没事件时的心跳"（30 Hz）。
    if (force || now - S.lastCmdSent >= CMD_EVENT_GAP_MS) sendCmd();
  }

  function sendCmd(hard) {
    if (!S.connected) return false;
    S.seq += 1;
    const msg = {
      t: "cmd",
      vx: S.target.vx,
      wz: S.target.wz,
      seq: S.seq,
      session: S.session,
      ts: Math.round(performance.now()),
    };
    if (hard) msg.hard = true;
    if (!send(msg)) return false;
    S.lastCmdSent = performance.now();
    pushSample(S.sendTimes, S.lastCmdSent);
    return true;
  }

  let cmdLoopOn = false;
  function startCmdLoop() {
    stopCmdLoop();
    cmdLoopOn = true;
    S.lastCmdSent = 0;
    // 主循环：requestAnimationFrame。屏幕刷新（60/90/120 Hz）驱动，
    // 触摸时不会被浏览器当作后台定时器节流；按 CMD_GAP_MS 限到 30 Hz。
    const tick = (ts) => {
      if (!cmdLoopOn) return;
      S.rafId = requestAnimationFrame(tick);
      pushSample(S.frameTimes, ts);
      // ★ 这里必须用 performance.now() 做间隔判断，不能混用 rAF 的 ts：
      // ts 是"这一帧的开始时刻"，比回调里真正执行的时刻早，拿它减
      // lastCmdSent（performance.now）会偶尔差 1~2 ms 从而多跳一帧，
      // 实际心跳会从 33 ms 变成 50 ms（22 Hz）。阈值留 4 ms 余量，
      // 保证 60 Hz 下稳定"每 2 帧发一次"=30 Hz。
      const now = performance.now();
      if (!document.hidden && now - S.lastCmdSent >= CMD_GAP_MS - 4) sendCmd();
      if (now - S.lastDiagRender >= 200) {
        S.lastDiagRender = now;
        renderDiagLine();
      }
    };
    S.rafId = requestAnimationFrame(tick);
    // 兜底：某些 WebView 在屏幕常亮/不合成时会暂停 rAF，但 setTimeout 还在。
    // 只在"页面可见且超过 150 ms 没发"时补一帧，保证 500 ms 心跳不断。
    const watchdog = () => {
      if (!cmdLoopOn) return;
      const now = performance.now();
      if (!document.hidden && now - S.lastCmdSent >= 150) sendCmd();
      S.watchdogId = setTimeout(watchdog, 100);
    };
    S.watchdogId = setTimeout(watchdog, 100);
  }
  function stopCmdLoop() {
    cmdLoopOn = false;
    if (S.rafId) { cancelAnimationFrame(S.rafId); S.rafId = 0; }
    if (S.watchdogId) { clearTimeout(S.watchdogId); S.watchdogId = 0; }
    S.sendTimes.length = 0;
    S.frameTimes.length = 0;
  }

  // 普通停车（松手/回中/主动停止）：目标发 0，后端按减速斜率软停。
  function zeroTarget() {
    S.target.vx = 0;
    S.target.wz = 0;
    text("cmdVx", fmt(0, 3));
    text("cmdWz", fmt(0, 3));
    renderSpeedClass(0);
    sendCmd();
  }

  // 安全停车（页面隐藏/失焦/pagehide）：带 hard:true 发 0，后端立即归零，
  // 不经过减速斜坡 —— 此时用户可能已经看不到车了。
  function hardStop() {
    S.target.vx = 0;
    S.target.wz = 0;
    text("cmdVx", fmt(0, 3));
    text("cmdWz", fmt(0, 3));
    renderSpeedClass(0);
    sendCmd(true);
  }

  // ---------------------------------------------------------------------
  // 摇杆（鼠标 + 触摸统一走 Pointer Events）
  // ---------------------------------------------------------------------
  const pad = $("pad");
  const knob = $("knob");

  function knobGeometry() {
    // 拖动期间几何不变：缓存起来，避免每个 pointermove 都做一次
    // getBoundingClientRect（会强制同步布局，手机上是实打实的卡顿源）。
    if (S.padGeom) return S.padGeom;
    const r = pad.getBoundingClientRect();
    S.padGeom = {
      cx: r.left + r.width / 2,
      cy: r.top + r.height / 2,
      max: Math.max(20, r.width / 2 - 38),
    };
    return S.padGeom;
  }

  function invalidatePadGeom() { S.padGeom = null; }

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

  function updateFromPointer(ev, force) {
    const g = knobGeometry();
    let dx = ev.clientX - g.cx;
    let dy = ev.clientY - g.cy;
    const m = Math.hypot(dx, dy);
    if (m > g.max) { dx = dx * g.max / m; dy = dy * g.max / m; }
    moveKnob(dx, dy);

    const nx = dx / g.max;
    const ny = dy / g.max;
    const mag = Math.hypot(nx, ny);
    // 双阈值滞回：居中时 10% 才起步，已经在控制时降到 6% 才回中。
    // 单阈值在手机上会被手指微抖反复跨过 → 目标 0/有 之间跳 → 车一顿一顿。
    const inDeadzone = S.deadzoneActive
      ? mag < DEADZONE_EXIT
      : mag < DEADZONE_ENTER;
    if (inDeadzone) {
      S.deadzoneActive = true;
      applyTarget(0, 0, force);
      return;
    }
    S.deadzoneActive = false;
    const lim = limits();
    // 屏幕坐标：上 = 前进(+vx)，右 = 右转。右转对应协议 wz<0，
    // 由 steerWz() 里的 WZ_CMD_SIGN 统一取反，这里不要写裸负号。
    applyTarget(-ny * lim.vx, steerWz(nx, lim.wz), force);
  }

  pad.addEventListener("pointerdown", (ev) => {
    if (ev.button !== undefined && ev.button !== 0) return;
    if (S.estop) return;
    ev.preventDefault();
    S.dragId = ev.pointerId;
    S.lastPointerType = ev.pointerType || "?";
    S.deadzoneActive = false;             // 每次新触摸都从"居中"状态开始
    invalidatePadGeom();                  // 按下时量一次，整个拖动复用
    pad.classList.add("active");
    // 捕获指针：手指滑出圆形区域后仍能收到 pointermove，不会"拖到一半断掉"。
    try { pad.setPointerCapture(ev.pointerId); } catch (err) {}
    updateFromPointer(ev, true);          // 按下立即发一帧，不等 rAF
  });

  pad.addEventListener("pointermove", (ev) => {
    if (S.dragId !== ev.pointerId) return;
    ev.preventDefault();
    // 触摸事件间隔诊断：这里记"前台真正处理到"的时刻，能看出主线程卡顿。
    pushSample(S.touchTimes, performance.now());
    updateFromPointer(ev, false);
  });

  function endDrag(ev, how) {
    if (S.dragId === null || S.dragId !== ev.pointerId) return;
    S.dragId = null;
    S.deadzoneActive = false;
    pad.classList.remove("active");
    resetKnob();
    applyTarget(0, 0, true);              // 松手立刻发 0，不等 rAF
    if (how && S.cancels[how] !== undefined) S.cancels[how] += 1;
  }

  // lostpointercapture 的 target 必须是 pad 本身才算"松手"：
  // 触摸指针默认隐式捕获在子元素上，setPointerCapture(pad) 转移捕获时
  // 会从子元素冒泡一个 lostpointercapture 上来 —— 那不代表用户松手，
  // 按旧写法会把刚按下的拖动立刻清零（手机上手感=按下去不动/一顿）。
  pad.addEventListener("lostpointercapture", (ev) => {
    if (ev.target === pad) endDrag(ev, "lostcapture");
  });
  pad.addEventListener("pointerup", (ev) => endDrag(ev, "pointerup"));
  pad.addEventListener("pointercancel", (ev) => endDrag(ev, "pointercancel"));
  // 兜底：捕获意外丢失/事件在 pad 外抬起时，window 上也能收到。
  window.addEventListener("pointerup", (ev) => endDrag(ev, "pointerup"));
  window.addEventListener("pointercancel", (ev) => endDrag(ev, "pointercancel"));
  pad.addEventListener("contextmenu", (ev) => ev.preventDefault());
  pad.addEventListener("dragstart", (ev) => ev.preventDefault());
  pad.addEventListener("dblclick", (ev) => ev.preventDefault());

  // 老 WebView / 部分 Android 浏览器对 touch-action 支持不完整：
  // 摇杆上的 touchmove 仍可能触发页面滚动/下拉刷新，浏览器会发 pointercancel
  // 把拖动打断。这里只在摇杆上非被动（passive:false）地 preventDefault，
  // 不动页面其它区域，也不参与控制逻辑（控制仍走 Pointer Events）。
  const blockGesture = (ev) => { if (ev.cancelable) ev.preventDefault(); };
  pad.addEventListener("touchstart", blockGesture, { passive: false });
  pad.addEventListener("touchmove", blockGesture, { passive: false });
  // iOS Safari 的双指缩放/长按呼出也不该在摇杆上触发。
  document.addEventListener("gesturestart", (ev) => {
    if (ev.target === pad || (ev.target && pad.contains(ev.target))) ev.preventDefault();
  });

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
      const j = m.jitter || {};
      const n = j.timeout_zero_count || 0;
      txt.textContent = `服务端 500 ms 未收到指令，已归零（累计超时清零 ${n} 次）`;
    } else if (reason === "stale") {
      // 300~500 ms 没新指令：后端按 70%→50%→30% 分级衰减，不再一步归零。
      const pct = Math.round((m.cmd.degrade_scale || 0) * 100);
      dot.classList.add("warn");
      txt.textContent = `链路抖动：${m.cmd.age_ms} ms 没收到新指令，目标暂降到 ${pct}%`;
    } else {
      dot.classList.add("ok");
      txt.textContent = `正在发送 · ${CMD_HZ} Hz · 实发 vx ${fmt(m.cmd.tx_vx, 3)} / wz ${fmt(m.cmd.tx_wz, 3)}`;
    }
  }

  function setBar(id, value, max) {
    const el = $(id);
    if (!el) return;
    // 值没变就别写 style：20 Hz × 4 个条，手机上这些写操作会带来
    // 不必要的样式重算/重绘（和 canvas 的布局读取叠加成卡顿）。
    const key = value + "|" + max;
    if (el.__barKey === key) return;
    el.__barKey = key;
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
    const el = $(id);
    if (!el) return;
    const key = value + "|" + max;
    if (el.__markKey === key) return;
    el.__markKey = key;
    el.style.left = (50 + clamp(value / max, -1, 1) * 50) + "%";
  }

  // 后端已经把分级算好（temp_level / age_level / boot_level），前端只套 CSS。
  // 这样阈值只有一份，不会出现"后端 70℃ 变橙、前端 75℃ 变橙"的漂移。
  const lvlClass = (lvl) => "lvl " + (lvl || "ok");
  const ageText = (ms) => (ms === 0xFFFF || ms === 65535 ? "从未收到" : `${ms} ms`);

  function renderWheel(i, w) {
    const p = "w" + i;
    text(p + "omega", fmt(w.omega, 2));
    text(p + "target", fmt(w.target, 2));
    setBar(p + "fill", w.omega, W_OMEGA_MAX);
    setMark(p + "mark", w.target, W_OMEGA_MAX);

    // 偏差幅值 = |实际 − 目标|（44B 帧 [6..9] 与 [10..13]）；界面只显示大小，
    // 方向看上面的实际/目标两个数（电机原始方向下前进为负，带符号反而容易误读）。
    text(p + "dev", Math.abs(w.dev).toFixed(2));
    const devEl = $(p + "dev");
    if (devEl) {
      const ad = Math.abs(w.dev);
      devEl.className = ad <= 0.5 ? "ok" : (ad <= 2 ? "warn" : "bad");
    }
    // 跟随度只在 |目标|≥0.5 rad/s 时由后端给出，否则显示 "—"（不编 0%）
    text(p + "follow", w.follow_pct === null || w.follow_pct === undefined
      ? "—" : `${w.follow_pct.toFixed(0)}%`);

    text(p + "torque", fmt(w.torque, 2));
    setBar(p + "tqfill", w.torque, TQ_MAX);
    const tqfill = $(p + "tqfill");
    if (tqfill) tqfill.classList.toggle("hot", Math.abs(w.torque) >= 8);

    const on = $(p + "online");
    on.textContent = w.online ? "在线" : "离线";
    on.className = "chip " + (w.online ? "ok" : "bad");

    const en = $(p + "enable");
    en.textContent = w.enabled ? "使能" : "失能";
    en.className = "chip " + (w.enabled ? "ok" : "");

    // ERR：0=失能、1=使能都算正常（绿色/中性），其它码中文 + 红色高亮
    const err = $(p + "err");
    err.textContent = `ERR ${w.err} · ${w.err_text}`;
    err.className = "err " + (w.err > 1 ? "bad" : "ok");

    const age = $(p + "age");
    if (age) {
      age.textContent = ageText(w.fb_age_ms);
      age.className = lvlClass(w.age_level);
    }

    const pos = $(p + "pos");
    if (pos) pos.textContent = fmt(w.pos, 2);

    const mos = $(p + "mos");
    if (mos) {
      mos.textContent = w.mos_temp;
      mos.className = lvlClass(w.mos_level);
    }
    const coil = $(p + "coil");
    if (coil) {
      coil.textContent = w.coil_temp;
      coil.className = lvlClass(w.coil_level);
    }

    const wheel = $("wheel" + i);
    wheel.classList.toggle("online", !!w.online);
    // 温度/反馈年龄异常时整张轮卡描边：一眼能看出是哪一侧出问题
    wheel.classList.toggle("hot", w.mos_level === "bad" || w.coil_level === "bad");
    wheel.classList.toggle("warm", !wheel.classList.contains("hot")
      && (w.mos_level === "warn" || w.coil_level === "warn"));
    wheel.classList.toggle("stale", w.age_level === "bad");
  }

  function renderFlags(m) {
    const box = $("flags");
    const flags = m.flags || [];
    const sig = flags.map((f) => f.key).join(",");
    if (sig !== S.flagSig) {
      // 重建一次并缓存元素：不要每帧对 8 个 flag 做 querySelector。
      S.flagSig = sig;
      S.flagEls = Object.create(null);
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
        S.flagEls[f.key] = span;
      });
    }
    flags.forEach((f) => {
      const el = S.flagEls[f.key] || box.querySelector(`[data-key="${f.key}"]`);
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

  function renderImu(tel, st, imu) {
    if (!tel) {
      ["ax", "ay", "az", "gx", "gy", "gz"].forEach((k) => text(k, "—"));
      text("tilt", "—");
      text("imuMag", "—");
      text("imuMagDev", "—");
      text("imuHint", "等待数据");
      text("imuMeta", "地址 — · 版本 —");
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

    // 模长由 24B 帧 ax/ay/az 算出（后端 snapshot.imu.accel_mag）。
    // 静止应≈9.81；偏离说明振动/碰撞/模块装歪，或某轴被 ±2g 量程夹住。
    const mag = imu && Number.isFinite(imu.accel_mag)
      ? imu.accel_mag : Math.hypot(tel.ax, tel.ay, tel.az);
    const dev = mag - 9.80665;
    text("imuMag", mag.toFixed(2));
    text("imuMagDev", fmt(dev, 2));
    const magEl = $("imuMag");
    if (magEl) {
      const ad = Math.abs(dev);
      magEl.className = "lvl " + (ad <= 0.6 ? "ok" : (ad <= 2.0 ? "warn" : "bad"));
    }

    if (st) {
      const addr = st.imu_addr ? `0x${st.imu_addr.toString(16).toUpperCase()}` : "未扫到";
      text("imuHint", `${st.imu_status_text} · 地址 ${addr} · ${st.imu_gyro_ok ? "陀螺已读到" : "陀螺未读到"}`);
      const probe = (imu && imu.probe) || {};
      const nProbe = ["version", "gyro", "mag", "gyro_ever_ok"]
        .filter((k) => !!probe[k]).length;
      text("imuMeta", `IMU 体检 ${nProbe}/4 · 版本 ${st.imu_ver} · tick ${st.tick}`);
      const setProbe = (id, ok) => {
        const el = $(id);
        if (el) el.className = "flag " + (ok ? "ok" : "bad");
      };
      setProbe("imuProbeVersion", !!probe.version);
      setProbe("imuProbeGyro", !!probe.gyro);
      setProbe("imuProbeMag", !!probe.mag);
      setProbe("imuProbeEver", !!probe.gyro_ever_ok);
    } else {
      ["imuProbeVersion", "imuProbeGyro", "imuProbeMag", "imuProbeEver"]
        .forEach((id) => { const el = $(id); if (el) el.className = "flag"; });
      text("imuMeta", "地址 — · 版本 —");
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
    text("dStatusAge", link.status_age_ms === null || link.status_age_ms === undefined
      ? "—" : `${link.status_age_ms} ms`);
    const sa = $("dStatusAge");
    if (sa && link.status_age_ms !== null && link.status_age_ms !== undefined) {
      sa.className = "lvl " + (link.status_age_ms > 500 ? "bad"
        : (link.status_age_ms > 200 ? "warn" : "ok"));
    }
    text("dTx", `${link.tx_hz} Hz · ${link.tx_count} 帧`);
    text("dFrames", `遥测 ${link.telemetry_count} · 状态 ${link.status_count}`);
    text("dBad", `${link.bad_frames} / ${link.resync_bytes} B`);
    text("dEcho", st ? `vx ${fmt(st.cmd_vx, 3)} · wz ${fmt(st.cmd_wz, 3)}` : "—");
    text("dBoot", st ? st.boot_stage_text : "—");
    const bootEl = $("dBoot");
    if (bootEl) bootEl.className = "lvl " + (st ? (st.boot_level || "") : "");
    text("dImu", st ? `${st.imu_status_text} · ver ${st.imu_ver}` : "—");
    text("dProto", st ? `v${st.version}` : "—");
    // 反馈年龄是判断"电机掉线"最直接的指标：左右各一个，取更差的那个上色
    text("dFbAge", st ? `L ${ageText(st.left.fb_age_ms)} · R ${ageText(st.right.fb_age_ms)}` : "—");
    const fbEl = $("dFbAge");
    if (fbEl && st) {
      const levels = [st.left.age_level, st.right.age_level];
      fbEl.className = "lvl " + (levels.includes("bad") ? "bad"
        : (levels.includes("warn") ? "warn" : "ok"));
    }
    text("dCan", st ? `${st.can_fail} / ${st.fb_count}` : "—");
    const canEl = $("dCan");
    if (canEl) {
      canEl.className = "lvl " + (st
        ? (st.can_fail > 10 ? "bad" : (st.can_fail > 0 ? "warn" : "ok")) : "");
    }
  }

  // ---------------------------------------------------------------------
  // 链路抖动诊断（每 200 ms 刷新一行，数据是最近 2 s 的滚动窗口）
  //
  // 怎么看：
  //   * "指令"：前端真正发出 cmd 的间隔。正常 30~50 Hz → 20~35 ms；
  //     出现 100~500 ms 的尖峰 = 主线程被卡（渲染/GC）或定时器被节流。
  //   * "触摸"：pointermove 到达前台的间隔。手指一直动时正常 8~20 ms；
  //     出现 >80 ms 的尖峰 = 主线程卡顿，不是网络问题。
  //   * "帧"：rAF 间隔。和触摸同时尖峰 → 页面渲染太重/手机 CPU 忙。
  //   * "触摸取消"：浏览器把触摸抢去滚动/缩放（pointercancel）的次数。
  //     涨 = 手势没被 touch-action:none 完全拦住。
  //   * "硬停"：blur/切后台/pagehide 触发的立即归零次数。blur 在手机上
  //     可能是浏览器地址栏/通知栏抢焦点，涨且车顿 → 是这个原因。
  //   * "超时清零/原因"：后端 step_output 的统计。超时清零涨 = 真·网络/心跳
  //     断流；不涨而车仍顿 → 看触摸/帧，是前端输入或主线程问题。
  // ---------------------------------------------------------------------
  function renderDiagLine(m) {
    m = m || S.last;
    const now = performance.now();
    // 读的时候也裁剪：手指/心跳停下来后，旧样本不能一直显示成"最近 2 s"。
    pruneSamples(S.sendTimes, now);
    pruneSamples(S.touchTimes, now);
    pruneSamples(S.frameTimes, now);
    const sendS = intervalStats(S.sendTimes);
    const touchS = intervalStats(S.touchTimes);
    const frameS = intervalStats(S.frameTimes);
    const lineEl = $("jitterLine");
    if (lineEl) {
      lineEl.textContent = [
        statsText("指令", sendS),
        statsText("触摸", touchS),
        statsText("帧", frameS),
      ].join(" | ");
    }

    const j = (m && m.jitter) || null;
    const reason = (j && j.reason) || null;
    const arr = (j && j.arrival) || null;
    const cancels = S.cancels.pointercancel + S.cancels.lostcapture;
    const hardStops = S.hardStops.blur + S.hardStops.hidden + S.hardStops.pagehide;
    const backendEl = $("jitterBackend");
    if (backendEl) {
      if (!j) {
        backendEl.textContent = "超时清零 — · 原因 — · 服务端诊断等待中";
      } else {
        backendEl.textContent =
          `超时清零 ${j.timeout_zero_count} 次 · ` +
          `原因 web ${reason ? reason.web : 0}/stale ${reason ? reason.stale : 0}` +
          `/timeout ${reason ? reason.timeout : 0}/hard ${reason ? reason.hard : 0} · ` +
          `后端到达 ${arr && arr.n >= 2 ? msText(arr.avg) : "—"}ms · ` +
          `触摸取消 ${cancels} · 硬停 ${hardStops}(blur ${S.hardStops.blur})`;
      }
    }

    // 链路诊断卡片里的三行（和上面同一份数据，方便截图/README 对照）
    text("dJitter", statsText("指令", sendS) + " · " + statsText("触摸", touchS));
    const tz = $("dTimeoutZero");
    if (tz && j) {
      tz.textContent = `超时清零 ${j.timeout_zero_count} 次 · 近2s timeout ${reason ? reason.timeout : 0} 帧`;
      // 只在最近 10 s 内真的清零过才标黄，否则累计值会让这一行一直黄着。
      const last = j.timeout_zero_last_s;
      const recent = last !== null && last !== undefined && m && m.ts !== undefined
        && (m.ts - last) < 10;
      tz.className = "lvl " + (recent ? "warn" : "ok");
    } else if (tz) {
      tz.textContent = "—";
    }
    text("dArrival", arr && arr.n >= 2
      ? `${msText(arr.min)}/${msText(arr.avg)}/${msText(arr.max)}ms 抖${msText(arr.jitter)} · ${arr.n} 帧/2s`
      : "—");
  }

  // 给 selftest / 现场控制台看的只读诊断快照（不参与控制）。
  window.__carwebDiag = () => ({
    connected: S.connected,
    raf: !!S.rafId,
    session: S.session,
    seq: S.seq,
    lastCmdSentAgo: Math.round(performance.now() - S.lastCmdSent),
    send: intervalStats(S.sendTimes),
    touch: intervalStats(S.touchTimes),
    frame: intervalStats(S.frameTimes),
    hardStops: { ...S.hardStops },
    cancels: { ...S.cancels },
    pointerType: S.lastPointerType,
    deadzoneActive: S.deadzoneActive,
    dragId: S.dragId,
    backend: (S.last && S.last.jitter) || null,
    reason: (S.last && S.last.cmd && S.last.cmd.reason) || null,
    degradeScale: (S.last && S.last.cmd && S.last.cmd.degrade_scale) || 1,
    timeoutZeroCount: (S.last && S.last.jitter && S.last.jitter.timeout_zero_count) || 0,
  });

  function renderVersion(m) {
    const el = $("footerVer");
    if (!el) return;
    const v = (m && m.version) || "1.4.0";
    text("footerVer", `car_web ${v}`);
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
    renderImu(tel, st, m.imu);
    renderOdom(m);
    renderDiag(m);
    renderDiagLine(m);
    renderEvents(m);
    renderBanner(m);
    renderSendState(m);
    renderVersion(m);

    if (tel) pushWave(tel);
    drawWave();
  }

  // ---------------------------------------------------------------------
  // 波形（canvas，无第三方库）
  // ---------------------------------------------------------------------
  function sizeCanvas(force) {
    const c = $("wave");
    if (!c) return;
    // ★ 只在尺寸变化时读 clientWidth/clientHeight。每帧读一次布局会
    // 强制同步重排（前面刚写过一堆 style），手机上这就是掉帧的常见原因。
    if (!force && !S.canvasDirty) return;
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(10, c.clientWidth);
    const h = Math.max(10, c.clientHeight);
    S.canvasDirty = false;
    if (S.canvasW === w && S.canvasH === h) return;
    S.canvasW = w;
    S.canvasH = h;
    c.width = Math.round(w * dpr);
    c.height = Math.round(h * dpr);
    const ctx = c.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function watchCanvasSize() {
    const c = $("wave");
    if (!c || S.canvasObs || typeof ResizeObserver !== "function") return;
    // 容器尺寸变化（旋转屏幕、侧栏折行）时标脏，下一帧重新量。
    S.canvasObs = new ResizeObserver(() => { S.canvasDirty = true; });
    S.canvasObs.observe(c);
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

  window.addEventListener("resize", () => {
    S.canvasW = S.canvasH = 0;
    S.canvasDirty = true;
    invalidatePadGeom();
    drawWave();
  });
  // 页面滚动/地址栏伸缩后摇杆的视口坐标会变，缓存必须失效。
  window.addEventListener("scroll", invalidatePadGeom, { passive: true });

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
      S.hardStops.hidden += 1;
      S.keys.clear();
      S.dragId = null;
      pad.classList.remove("active");
      resetKnob();
      hardStop();
    }
  });

  window.addEventListener("blur", () => {
    S.hardStops.blur += 1;
    S.keys.clear();
    if (S.dragId === null) resetKnob();
    hardStop();
  });

  window.addEventListener("pagehide", () => {
    S.hardStops.pagehide += 1;
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
  renderDiagLine(null);
  sizeCanvas();
  watchCanvasSize();
  drawWave();
  connect();
})();
