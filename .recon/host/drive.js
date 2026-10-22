"use strict";

/* TB01 底盘网页驾驶前端
 * - 虚拟摇杆 / WASD / 方向键 → 20Hz 发 {type:"cmd", v, w}
 * - 松手=停车（发 0，不撤销使能）；失焦/切标签页/空格/急停=停车+撤销使能
 * - 服务端回传 state（odom / chassis_diag / 发布状态）
 */

const $ = (sel, root = document) => root.querySelector(sel);

const Drive = {
  ws: null,
  wsOpen: false,
  enabled: false,
  state: null,
  limits: { max_vx: 0.3, max_wz: 0.8 },
  gear: { v: 0.2, w: 0.5 },
  stick: { active: false, nx: 0, ny: 0 },
  keys: new Set(),
  sendTimer: null,
  lastStateTs: 0,
  logs: [],
};

const GEARS = {
  slow: { v: 0.10, w: 0.30 },
  normal: { v: 0.20, w: 0.50 },
  fast: { v: 0.30, w: 0.80 },
};

/* ---------------------------------------------------------------- 工具 */
function toast(msg, kind = "info") {
  const wrap = document.createElement("div");
  wrap.className = "log-item " + kind;
  wrap.textContent = msg;
  $("#logList").appendChild(wrap);
  while ($("#logList").children.length > 200) $("#logList").removeChild($("#logList").firstChild);
  $("#logList").scrollTop = $("#logList").scrollHeight;
}

function fmt(v, d = 2) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "—";
  return Number(v).toFixed(d);
}

/* ---------------------------------------------------------------- WebSocket */
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  Drive.ws = ws;
  ws.onopen = () => {
    Drive.wsOpen = true;
    setLink(true);
    toast("已连接驾驶服务", "info");
  };
  ws.onclose = () => {
    Drive.wsOpen = false;
    setLink(false);
    Drive.enabled = false;
    updateEnableButton();
    toast("连接断开，已请求停车", "error");
    setTimeout(connect, 1500);
  };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    if (msg.type !== "state") return;
    Drive.state = msg;
    Drive.lastStateTs = Date.now();
    if (typeof msg.enabled === "boolean") {
      Drive.enabled = msg.enabled;
      updateEnableButton();
    }
    if (msg.limits) Drive.limits = msg.limits;
    render(msg);
  };
}

function send(obj) {
  if (Drive.ws && Drive.ws.readyState === WebSocket.OPEN) {
    Drive.ws.send(JSON.stringify(obj));
  }
}

function setLink(ok) {
  const dot = $("#linkStatus .dot");
  dot.className = "dot " + (ok ? "ok" : "err");
  $("#linkText").textContent = ok ? "驾驶服务已连接" : "连接断开";
  $("#subtitle").textContent = ok
    ? `ROS /cmd_vel · 限速 ${Drive.limits.max_vx.toFixed(2)} m/s · ${Drive.limits.max_wz.toFixed(2)} rad/s`
    : "等待连接…";
}

function updateEnableButton() {
  const btn = $("#btnEnable");
  btn.textContent = Drive.enabled ? "驾驶中（点击停车）" : "启动驾驶";
  btn.className = "btn big " + (Drive.enabled ? "on" : "primary");
  $("#stateHint").textContent = Drive.enabled ? "已使能：摇杆/键盘可行驶" : "未使能";
  $("#stateHint").style.color = Drive.enabled ? "var(--green)" : "var(--muted)";
}

/* ---------------------------------------------------------------- 摇杆 */
function setupStick() {
  const stick = $("#stick");
  const knob = $("#knob");
  const R = () => stick.clientWidth * 0.32;   // 摇杆最大位移半径

  const setKnob = (nx, ny) => {
    const r = R();
    knob.style.transform = `translate(calc(-50% + ${nx * r}px), calc(-50% + ${ny * r}px))`;
  };

  const updateFromEvent = (ev) => {
    const rect = stick.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = ev.clientX - cx;
    let dy = ev.clientY - cy;
    const r = R();
    const len = Math.hypot(dx, dy);
    if (len > r) { dx = dx / len * r; dy = dy / len * r; }
    const nx = dx / r;
    const ny = dy / r;
    const dead = 0.08;
    Drive.stick.nx = Math.abs(nx) < dead ? 0 : nx;
    Drive.stick.ny = Math.abs(ny) < dead ? 0 : ny;
    setKnob(nx, ny);
  };

  stick.addEventListener("pointerdown", (ev) => {
    if (!Drive.enabled) { toast("请先点『启动驾驶』", "warning"); return; }
    try { stick.setPointerCapture(ev.pointerId); } catch (e) { /* 合成事件没有活动指针 */ }
    Drive.stick.active = true;
    stick.classList.add("active");
    updateFromEvent(ev);
  });
  stick.addEventListener("pointermove", (ev) => {
    if (!Drive.stick.active) return;
    updateFromEvent(ev);
  });
  const release = () => {
    Drive.stick.active = false;
    Drive.stick.nx = 0;
    Drive.stick.ny = 0;
    stick.classList.remove("active");
    setKnob(0, 0);
  };
  stick.addEventListener("pointerup", release);
  stick.addEventListener("pointercancel", release);
  stick.addEventListener("pointerleave", (ev) => { if (Drive.stick.active && ev.buttons === 0) release(); });
}

function currentCmd() {
  // 摇杆优先；否则键盘
  let nx = Drive.stick.nx;
  let ny = Drive.stick.ny;
  if (!Drive.stick.active) {
    nx = 0; ny = 0;
    if (Drive.keys.has("a") || Drive.keys.has("arrowleft")) nx -= 1;
    if (Drive.keys.has("d") || Drive.keys.has("arrowright")) nx += 1;
    if (Drive.keys.has("w") || Drive.keys.has("arrowup")) ny -= 1;
    if (Drive.keys.has("s") || Drive.keys.has("arrowdown")) ny += 1;
    const len = Math.hypot(nx, ny);
    if (len > 1) { nx /= len; ny /= len; }
  }
  const v = -ny * Drive.gear.v;      // 上推 = 前进
  const w = -nx * Drive.gear.w;      // 左推 = 左转（ROS +z）
  const maxV = Drive.limits.max_vx || Drive.gear.v;
  const maxW = Drive.limits.max_wz || Drive.gear.w;
  return {
    v: Math.max(-maxV, Math.min(maxV, v)),
    w: Math.max(-maxW, Math.min(maxW, w)),
  };
}

/* ---------------------------------------------------------------- 心跳发送 */
function startSending() {
  if (Drive.sendTimer) return;
  Drive.sendTimer = setInterval(() => {
    if (!Drive.wsOpen) return;
    if (!Drive.enabled) return;
    const { v, w } = currentCmd();
    send({ type: "cmd", v: Number(v.toFixed(3)), w: Number(w.toFixed(3)) });
  }, 50);   // 20 Hz
}

/* ---------------------------------------------------------------- 渲染 */
function render(st) {
  const odom = st.odom || {};
  const diag = st.diag || null;

  // ROS / 发布状态
  const rosDot = $("#rosDot");
  const rosOk = !!st.ros_ok && st.odom_age !== null && st.odom_age !== undefined && st.odom_age < 1.0;
  rosDot.className = "dot " + (rosOk ? "ok" : "warn");
  $("#rosText").textContent = st.sim ? "SIM" : (rosOk ? "ROS 在线" : "ROS 无数据");

  $("#cmdV").textContent = fmt(st.cmd && st.cmd.v, 2);
  $("#cmdW").textContent = fmt(st.cmd && st.cmd.w, 2);
  $("#pubState").textContent = st.publishing ? "发布中" : (Drive.enabled ? "等待指令" : "已停止");
  $("#pubState").style.color = st.publishing ? "var(--green)" : "var(--muted)";

  $("#odomX").innerHTML = fmt(odom.x, 3) + '<span class="u">m</span>';
  $("#odomY").innerHTML = fmt(odom.y, 3) + '<span class="u">m</span>';
  const deg = odom.theta === undefined ? null : odom.theta * 180 / Math.PI;
  $("#odomTh").innerHTML = fmt(deg, 1) + "°";
  $("#odomVx").innerHTML = fmt(odom.vx, 2) + '<span class="u">m/s</span>';
  $("#odomWz").innerHTML = fmt(odom.wz, 2) + '<span class="u">rad/s</span>';
  $("#limitText").textContent = `限速 ${Drive.limits.max_vx.toFixed(2)} m/s · ${Drive.limits.max_wz.toFixed(2)} rad/s`;
  $("#odomAge").textContent = st.odom_age === null || st.odom_age === undefined
    ? "无 /odom" : `/odom ${fmt(st.odom_age, 2)}s 前`;

  // 诊断
  if (diag) {
    $("#diagCmd").innerHTML = `${fmt(diag.cmd_vx, 2)} / ${fmt(diag.cmd_wz, 2)}`;
    $("#wheelL").textContent = `${fmt(diag.wheel_vel && diag.wheel_vel[0], 1)} / ${fmt(diag.wheel_target && diag.wheel_target[0], 1)}`;
    $("#wheelR").textContent = `${fmt(diag.wheel_vel && diag.wheel_vel[1], 1)} / ${fmt(diag.wheel_target && diag.wheel_target[1], 1)}`;
    $("#temps").innerHTML = `${fmt(diag.mos_temp && diag.mos_temp[0], 0)}/${fmt(diag.mos_temp && diag.mos_temp[1], 0)} · ${fmt(diag.coil_temp && diag.coil_temp[0], 0)}/${fmt(diag.coil_temp && diag.coil_temp[1], 0)}<span class="u">℃</span>`;
    const problems = [];
    if (!diag.left_online || !diag.right_online) problems.push("驱动器掉线");
    if (diag.left_fault || diag.right_fault) problems.push("驱动器故障");
    if (!diag.left_enabled || !diag.right_enabled) problems.push("驱动器失能");
    if (diag.cmd_timeout) problems.push("命令超时");
    if (diag.imu_ok === false) problems.push("IMU 异常");
    const el = $("#diagState");
    if (problems.length) {
      el.className = "v bad";
      el.textContent = problems.join(" · ");
    } else if (diag.flags !== undefined) {
      el.className = "v ok";
      el.textContent = "正常（双轮在线/使能）";
    } else {
      el.className = "v warn";
      el.textContent = "—";
    }
    $("#diagRaw").textContent = JSON.stringify(diag, null, 2);
  } else {
    $("#diagCmd").textContent = "—";
    $("#wheelL").textContent = "—";
    $("#wheelR").textContent = "—";
    $("#temps").textContent = "—";
    $("#diagState").textContent = st.sim ? "仿真模式无诊断帧" : "未收到 /chassis_diag";
    $("#diagRaw").textContent = "—";
  }
  updateDrvWarn(st, diag);
}

function updateDrvWarn(st, diag) {
  const warn = $("#drvWarn");
  if (!warn) return;
  const disabled = !!(diag && (!diag.left_enabled || !diag.right_enabled));
  warn.hidden = !disabled;
  if (!disabled) return;
  const fw = st.firmware_version;
  const supported = !!st.reinit_supported;
  $("#drvWarnHint").textContent = supported
    ? `固件 v${fw} 支持上位机使能：点右侧按钮让底盘重新“写速度模式 + 使能”。`
    : `固件 v${fw || "?"} 不支持上位机使能 —— 请复位 STM32 底盘或整车断电重启（或升级固件 v2）。`;
  $("#btnReinit").disabled = !supported;
}

/* ---------------------------------------------------------------- 停车/急停 */
function stopAll(reason) {
  Drive.enabled = false;
  Drive.stick.active = false;
  Drive.stick.nx = 0;
  Drive.stick.ny = 0;
  Drive.keys.clear();
  $("#stick").classList.remove("active");
  $("#knob").style.transform = "translate(-50%, -50%)";
  send({ type: "stop" });
  updateEnableButton();
  if (reason) toast("停车：" + reason, "warning");
}

function setupControls() {
  $("#gear").addEventListener("change", (ev) => {
    Drive.gear = GEARS[ev.target.value] || GEARS.normal;
    toast(`档位：${ev.target.selectedOptions[0].textContent}`, "info");
  });

  $("#btnEnable").addEventListener("click", () => {
    if (Drive.enabled) { stopAll("手动停车"); return; }
    send({ type: "enable", on: true });
    Drive.enabled = true;
    updateEnableButton();
    toast("已使能，推动摇杆开始行驶", "ok");
  });

  $("#btnStop").addEventListener("click", () => stopAll("急停按钮"));

  $("#btnReinit").addEventListener("click", async () => {
    try {
      const resp = await fetch("/api/reinit", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: "{}",
      });
      const st = await resp.json();
      if (st.reinit_sent) {
        toast("已发送电机重新使能命令，1~2 秒后看状态栏 err 是否变 [1,1]", "ok");
      } else {
        toast("底盘固件不支持上位机使能（需要 v2+），请复位 STM32 或断电重启", "error");
      }
    } catch (e) {
      toast("重新使能请求失败: " + e, "error");
    }
  });

  $("#btnClearLog").addEventListener("click", () => { $("#logList").innerHTML = ""; });

  // 键盘
  const KEYMAP = new Set(["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"]);
  window.addEventListener("keydown", (ev) => {
    const key = ev.key.toLowerCase();
    if (key === " " || key === "escape") { ev.preventDefault(); stopAll("键盘急停"); return; }
    if (KEYMAP.has(key)) { ev.preventDefault(); Drive.keys.add(key); }
  });
  window.addEventListener("keyup", (ev) => { Drive.keys.delete(ev.key.toLowerCase()); });

  // 失焦/切标签页/页面隐藏 = 停车+撤销使能
  window.addEventListener("blur", () => stopAll("窗口失焦"));
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopAll("页面隐藏"); });
  window.addEventListener("pagehide", () => stopAll("页面关闭"));
}

/* ---------------------------------------------------------------- 启动 */
function init() {
  setupStick();
  setupControls();
  connect();
  startSending();
  updateEnableButton();
  setLink(false);
  toast("等待驾驶服务连接…", "info");

  // 每 2s 检查状态新鲜度，长时间无状态提示
  setInterval(() => {
    if (Drive.lastStateTs && Date.now() - Drive.lastStateTs > 3000) {
      $("#linkStatus .dot").className = "dot warn";
      $("#linkText").textContent = "状态超时";
    }
  }, 2000);
}

window.addEventListener("DOMContentLoaded", init);
