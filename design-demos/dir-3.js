// Direction C · Floor plan. All commands go to the in-page simulator (shared/sim.js); nothing reaches hardware.
"use strict";
const $ = (s, el = document) => el.querySelector(s), $$ = (s, el = document) => [...el.querySelectorAll(s)];
const { deg, rad, hwText } = EmioSim;
const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? "—" : v.toFixed(d);
const WHERE = { 0: "top right · pair A (with 2)", 1: "top left · pair B (with 3)", 2: "bottom left · pair A (with 0)", 3: "bottom right · pair B (with 1)" };
const MAP_ORDER = [1, 0, 2, 3];   // grid order: top-left, top-right, bottom-left, bottom-right

// ---- theme (before first paint) ----
(function () {
  const q = new URLSearchParams(location.search).get("theme");
  let t = q === "night" || q === "day" ? q : null;
  if (!t) { try { t = localStorage.getItem("dir3-theme"); } catch { /* storage blocked */ } }
  document.documentElement.dataset.theme = t === "night" ? "night" : "day";
})();

const sim = EmioSim.createSim();
EmioHarness.init(sim);
const acks = EmioSim.createAckStore();
const ui = {
  panel: "operate", view: "top", second: "camera", motor: 0, msMotor: 0, msDirty: false, target: null, camMode: null,
  cam3d: { ...EmioViews.CAMERA_PRESETS.front }, opts3d: { cloud: true, surface: true, zone: true, armCam: true, path: true },
  play: null, alertsOpen: false, dragging: false, alertOpen: new Set(), seen: new Set(),
};
let snap = null, meta = { ws: "open" }, model = null;
const mappers = {};

// ---- commands ----
function showError(msg) { $("#cmdErr").hidden = !msg; $("#cmdErrText").textContent = msg || ""; }
$("#cmdErrClose").addEventListener("click", () => showError(""));
function call(path, body) {
  return sim.api(path, body).then(r => { showError(""); return r; })
    .catch(e => { showError(e.detail || e.message || "Command failed"); throw e; });
}
const quiet = p => p.catch(() => {});

// ---- confirm dialog (native modal: focus stays inside, Esc / Cancel closes) ----
function confirmBox(title, body, yes) {
  const d = $("#confirm");
  $("#cfT").textContent = title; $("#cfB").textContent = body; $("#cfYes").textContent = yes;
  return new Promise(res => {
    const done = v => { d.close(); $("#cfYes").onclick = $("#cfNo").onclick = d.oncancel = null; res(v); };
    $("#cfYes").onclick = () => done(true); $("#cfNo").onclick = () => done(false); d.oncancel = e => { e.preventDefault(); done(false); };
    d.showModal(); $("#cfNo").focus();
  });
}
const rebootMotor = i => confirmBox(`Reboot motor ${i}?`, `Reboot motor ${i}? Release cable tension first. It restarts with torque off.`, `Reboot motor ${i}`)
  .then(ok => { if (ok) quiet(call(`/api/motor/${i}/reboot`, {})); });

// ---- global bar ----
$("#holdAll").addEventListener("click", () => quiet(call("/api/hold", {})));
$("#torqueOff").addEventListener("click", () => quiet(call("/api/torque_off_all", {})));
document.addEventListener("keydown", e => { if (e.key === "Escape") quiet(call("/api/torque_off_all", {})); });
const speed = $("#speed");
speed.addEventListener("input", () => $("#speedVal").textContent = `${(speed.value * 0.229).toFixed(1)} rpm`);
speed.addEventListener("change", () => quiet(call("/api/speed", { profile_velocity: +speed.value })));
$("#summBtn").addEventListener("click", () => {
  ui.alertsOpen = !ui.alertsOpen;
  $("#alerts").classList.toggle("open", ui.alertsOpen); $("#summBtn").setAttribute("aria-expanded", ui.alertsOpen);
  if (ui.alertsOpen) $("#alerts").scrollTop = 0;
});
$("#themeBtn").addEventListener("click", () => {
  const t = document.documentElement.dataset.theme === "night" ? "day" : "night";
  document.documentElement.dataset.theme = t;
  try { localStorage.setItem("dir3-theme", t); } catch { /* storage blocked */ }
  syncTheme(); drawAll();
});
function syncTheme() {
  const night = document.documentElement.dataset.theme === "night";
  $("#themeBtn").setAttribute("aria-pressed", night); $("#themeLbl").textContent = night ? "Night" : "Day";
  $("#themeBtn").title = night ? "Night display (press for day)" : "Day display (press for night)";
}
syncTheme();

// ---- rail ----
function setPanel(p) {
  ui.panel = p;
  $$(".rail [data-panel]").forEach(b => b.dataset.panel === p ? b.setAttribute("aria-current", "page") : b.removeAttribute("aria-current"));
  ["operate", "motors", "setup"].forEach(k => $(`#p-${k}`).hidden = k !== p);
  if (snap) render(snap, meta);
}
$$(".rail [data-panel]").forEach(b => b.addEventListener("click", () => setPanel(b.dataset.panel)));

// ---- alerts (global area + beside the motor) ----
function alertRow(a) {
  const row = document.createElement("div");
  row.className = "al"; row.dataset.id = a.id;
  row.innerHTML = `<div class="al-lv"></div><div class="al-main"><span class="al-src"></span> · <span class="al-title"></span><span class="al-ack">acknowledged · still active</span>
    <div class="al-more" hidden><div class="al-detail"></div><dl class="al-meas"></dl><code class="al-raw"></code></div></div>
    <div class="al-acts"><span class="al-act"></span><button class="b sm detb" aria-expanded="false">Details</button><button class="b sm ackb">Acknowledge</button></div>`;
  $(".detb", row).addEventListener("click", () => {
    const open = !ui.alertOpen.has(a.id + row.dataset.where);
    open ? ui.alertOpen.add(a.id + row.dataset.where) : ui.alertOpen.delete(a.id + row.dataset.where);
    $(".al-more", row).hidden = !open; $(".detb", row).setAttribute("aria-expanded", open);
  });
  $(".ackb", row).addEventListener("click", () => { acks.ack(a.id); render(snap, meta); });
  return row;
}
const LV_WORD = { fault: "FAULT", warning: "WARNING", caution: "CAUTION", notice: "NOTICE" };
function updateAlertRow(row, a) {
  const lv = $(".al-lv", row);
  lv.className = `al-lv lv-${a.level}`; lv.textContent = LV_WORD[a.level];
  $(".al-src", row).textContent = a.source; $(".al-title", row).textContent = a.title;
  const acked = acks.isAcked(a.id);
  row.classList.toggle("acked", acked); $(".al-ack", row).hidden = !acked;
  $(".al-detail", row).textContent = a.detail;
  $(".al-meas", row).innerHTML = "";
  a.measure.forEach(([k, v]) => { const dt = document.createElement("dt"), dd = document.createElement("dd"); dt.textContent = k; dd.textContent = v; $(".al-meas", row).append(dt, dd); });
  $(".al-raw", row).textContent = a.raw ? `Raw: ${a.raw}` : "";
  const key = a.actions.map(x => x.id + x.motor).join();
  if (row.dataset.acts !== key) {
    row.dataset.acts = key;
    const box = $(".al-act", row); box.innerHTML = "";
    a.actions.forEach(x => {
      const b = document.createElement("button"); b.className = "b sm"; b.textContent = x.label;
      if (x.id === "reboot") b.addEventListener("click", () => rebootMotor(x.motor));
      if (x.id === "connect") b.addEventListener("click", () => quiet(call("/api/connect", { port: $("#port").value || null })));
      if (x.id === "setup-motor") b.addEventListener("click", () => { ui.msMotor = x.motor; ui.msDirty = false; setPanel("setup"); $("#hCal").scrollIntoView({ block: "start" }); });
      box.appendChild(b);
    });
  }
  const open = ui.alertOpen.has(a.id + row.dataset.where);
  $(".al-more", row).hidden = !open; $(".detb", row).setAttribute("aria-expanded", open);
}
function syncAlertList(box, alerts, where) {
  const have = new Map($$(":scope > .al", box).map(r => [r.dataset.id, r]));
  alerts.forEach((a, i) => {
    let row = have.get(a.id);
    if (!row) { row = alertRow(a); row.dataset.where = where; }
    have.delete(a.id);
    updateAlertRow(row, a);
    if (box.children[i] !== row) box.insertBefore(row, box.children[i] || null);
  });
  have.forEach(r => r.remove());
}
function announce(alerts) {
  alerts.forEach(a => {
    if (ui.seen.has(a.id)) return;
    ui.seen.add(a.id);
    $(a.level === "fault" ? "#liveA" : "#liveP").textContent = `${LV_WORD[a.level]}: ${a.source}, ${a.title}`;
  });
  const live = new Set(alerts.map(a => a.id));
  [...ui.seen].forEach(id => { if (!live.has(id)) ui.seen.delete(id); });
}

// ---- motor map ----
function motorStatus(m, s, stale) {
  if (stale) return { cls: "unk", word: "Stale", sub: "no updates" };
  if (!s.connected) return { cls: "unk", word: "No data", sub: "disconnected" };
  if (!m.online) return { cls: "fault", word: "No response", sub: "not answering" };
  const tq = m.torque ? "torque on" : "torque off";
  if (m.hw_error) return { cls: "fault", word: "Fault", sub: hwText(m.hw_error) };
  if (m.overridden) return { cls: "warn", word: "Overridden", sub: "other program" };
  if (!m.calibrated) return { cls: "caution", word: "Not calibrated", sub: tq };
  if (!m.torque) return { cls: "off", word: "Torque off", sub: "released" };
  if (m.moving) return { cls: "on", word: "Moving", sub: tq, moving: true };
  return { cls: "on", word: "Holding", sub: tq };
}
const curText = m => m.ma_per_unit ? `${Math.round(m.current_raw * m.ma_per_unit)} mA` : `${m.current_raw} u (raw)`;
function buildMap(box, full) {
  box.innerHTML = `<svg class="pairs" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
      <line x1="25" y1="25" x2="75" y2="75" stroke="currentColor" stroke-width="0.8" vector-effect="non-scaling-stroke" style="stroke-width:3px"/>
      <line x1="75" y1="25" x2="25" y2="75" stroke="currentColor" stroke-width="0.8" vector-effect="non-scaling-stroke" style="stroke-width:3px"/></svg>`;
  MAP_ORDER.forEach(i => {
    const b = document.createElement("button");
    b.className = "mc"; b.dataset.m = i;
    b.innerHTML = `<span class="mn">${i}</span><span class="ms"></span><span class="mx"></span>` + (full ? `<span class="mrows"></span>` : "") + `<span class="mv"></span>`;
    b.addEventListener("click", () => { ui.motor = i; if (ui.panel !== "motors") setPanel("motors"); else render(snap, meta); });
    box.appendChild(b);
  });
}
buildMap($("#mapOp"), false); buildMap($("#mapM"), true);
function renderMap(box, s, stale, full) {
  s.motors.forEach(m => {
    const c = $(`.mc[data-m="${m.id}"]`, box), st = motorStatus(m, s, stale), known = !stale && s.connected && m.online;
    c.className = `mc s-${st.cls}${st.moving ? " s-moving" : ""}${full && ui.motor === m.id ? " sel" : ""}`;
    $(".ms", c).textContent = st.word; $(".mx", c).textContent = st.sub;
    $(".mv", c).innerHTML = known ? `${fmt(deg(m.angle_rel_rad), 1)}°<small>angle</small>` : `—<small>${stale ? "stale" : "no data"}</small>`;
    if (full) $(".mrows", c).innerHTML = known
      ? `<span>Goal</span><span>${m.goal_rel_rad != null ? fmt(deg(m.goal_rel_rad), 1) + "°" : "—"}</span><span>Current</span><span>${curText(m)}</span><span>Temp.</span><span>${m.temp_c != null ? m.temp_c + " °C" : "—"}</span>`
      : `<span>Goal</span><span>—</span><span>Current</span><span>—</span><span>Temp.</span><span>—</span>`;
    c.setAttribute("aria-label", `Motor ${m.id}, ${WHERE[m.id]}: ${st.word}, ${st.sub}` + (known ? `, angle ${fmt(deg(m.angle_rel_rad), 1)} degrees` : ""));
    if (full) c.setAttribute("aria-pressed", ui.motor === m.id);
  });
}

// ---- operate: views ----
const stage = $("#stage"), canv = Object.fromEntries($$("canvas.main", stage).map(c => [c.dataset.kind, c]));
const VIEW_NAMES = { camera: "Camera", top: "Top view", side: "Side view", "3d": "3D view" };
function setView(v) {
  if (v === ui.view) return;
  ui.second = ui.view; ui.view = v;
  $$(".tabs [role=tab]").forEach(t => { t.setAttribute("aria-selected", t.dataset.view === v); t.tabIndex = t.dataset.view === v ? 0 : -1; });
  Object.entries(canv).forEach(([k, c]) => c.hidden = k !== v);
  drawAll(); renderOperateText();
}
$$(".tabs [role=tab]").forEach(t => {
  t.tabIndex = t.getAttribute("aria-selected") === "true" ? 0 : -1;
  t.addEventListener("click", () => setView(t.dataset.view));
  t.addEventListener("keydown", e => {
    const tabs = $$(".tabs [role=tab]"), i = tabs.indexOf(t);
    if (e.key === "ArrowRight" || e.key === "ArrowLeft") { const n = tabs[(i + (e.key === "ArrowRight" ? 1 : tabs.length - 1)) % tabs.length]; n.focus(); setView(n.dataset.view); }
  });
});
$("#swapBtn").addEventListener("click", () => setView(ui.second));

function cameraState() {
  if (meta.ws === "closed") return { off: false, note: "Server unreachable: image is stale" };
  const age = snap && snap.camera && snap.camera.age_s;
  if (age == null) return { off: true, note: "Camera feed off: no image" };
  if (age >= 1.5) return { off: false, note: `Frame ${age.toFixed(1)} s old: not live` };
  return { off: false, note: "" };
}
function drawKind(kind, cv, main) {
  if (!snap || !cv.clientWidth) return;
  const th = EmioViews.theme(stage), s = snap;
  if (kind === "top") {
    const st = s.tip.status, tgt = ui.target || (s.tip.busy && st.target) || null;
    const m = EmioViews.drawTop(cv, s, model, th, { target: tgt, obsMap: model ? sim.obstacleMap() : null });
    if (main) mappers.top = m;
  } else if (kind === "side") EmioViews.drawSide(cv, s, model, th);
  else if (kind === "camera") {
    const cs = cameraState(), box = EmioViews.drawCamera(cv, s, model, th, { off: cs.off, marks: ui.camMode && ui.camMode.mark ? ui.camMode.mark : [] });
    if (main) mappers.camera = box;
  } else {
    let ss = s;
    if (ui.play && s.tip.plan) {
      const k = Math.max(1, Math.ceil((performance.now() - ui.play) / 2500 * s.tip.plan.arms.length));
      ss = { ...s, tip: { ...s.tip, plan: { ...s.tip.plan, arms: s.tip.plan.arms.slice(0, k) } } };
      if (k >= s.tip.plan.arms.length) ui.play = null;
    }
    const m = EmioViews.draw3D(cv, ss, sim, model, th, main ? ui.cam3d : { ...EmioViews.CAMERA_PRESETS.front, zoom: 0.8 }, ui.opts3d);
    if (main) mappers.d3 = m;
  }
}
function drawAll() {
  if (!snap) return;
  if (ui.panel === "operate") { drawKind(ui.view, canv[ui.view], true); drawKind(ui.second, $("#second"), false); }
  if (ui.panel === "setup") {
    const cs = cameraState();
    mappers.setupCam = EmioViews.drawCamera($("#setupCam"), snap, model, EmioViews.theme(stage), { off: cs.off, marks: ui.camMode && ui.camMode.mark ? ui.camMode.mark : [] });
  }
}
new ResizeObserver(() => drawAll()).observe(stage);

// clicks on the views
function setTarget(x, z) { $("#tipX").value = x.toFixed(0); $("#tipZ").value = z.toFixed(0); ui.target = [x, z]; }
canv.top.addEventListener("click", e => {
  if (!mappers.top) return;
  const r = e.target.getBoundingClientRect(), [x, z] = mappers.top.toWorld(e.clientX - r.left, e.clientY - r.top);
  setTarget(x, z);
  if ($("#tipClick").checked && !(snap && snap.tip.busy)) tipGoto();
  drawAll();
});
function camClick(e, box) {
  if (!ui.camMode || !box) return;
  const r = e.target.getBoundingClientRect(), u = (e.clientX - r.left - box.x) / box.w, v = (e.clientY - r.top - box.y) / box.h;
  if (u < 0 || u > 1 || v < 0 || v > 1) return;
  if (ui.camMode === "pick") { quiet(call("/api/obstacle/pick", { u, v })); setCamMode(null); return; }
  ui.camMode.mark.push([u, v]);
  if (ui.camMode.mark.length === 2) { quiet(call("/api/obstacle/mark", { points: ui.camMode.mark, radius_mm: 5 })); setCamMode(null); }
  else renderObstacleSetup();
  drawAll();
}
canv.camera.addEventListener("click", e => camClick(e, mappers.camera));
$("#setupCam").addEventListener("click", e => camClick(e, mappers.setupCam));
EmioViews.attachOrbit(canv["3d"], ui.cam3d, () => drawAll());
canv["3d"].addEventListener("dblclick", e => {
  if (!mappers.d3) return;
  const r = e.target.getBoundingClientRect(), p = mappers.d3.pick(e.clientX - r.left, e.clientY - r.top);
  if (!p) return;
  setTarget(p.x, p.z);
  quiet(call("/api/tip/preview", { x: p.x, z: p.z }));
});
$$("[data-preset]").forEach(b => b.addEventListener("click", () => { Object.assign(ui.cam3d, EmioViews.CAMERA_PRESETS[b.dataset.preset]); if (ui.panel === "operate" && ui.view !== "3d") setView("3d"); drawAll(); }));
function play() { if (!snap || !snap.tip.plan) return; ui.play = performance.now(); if (ui.panel !== "operate") setPanel("operate"); if (ui.view !== "3d") setView("3d"); }
$("#playBtn").addEventListener("click", play); $("#playBtn2").addEventListener("click", play);
setInterval(() => { if (ui.play) drawAll(); }, 60);

// ---- operate: move tip ----
function tipGoto() {
  if ($("#tipX").value === "" || $("#tipZ").value === "") return showError("Enter a target X and Z");
  const x = +$("#tipX").value, z = +$("#tipZ").value;
  ui.target = [x, z];
  quiet(call("/api/tip/goto", { x, z, closed_loop: $("#tipClosed").checked }));
}
$("#tipGo").addEventListener("click", tipGoto);
$("#tipStop").addEventListener("click", () => quiet(call("/api/tip/stop", {})));
["#tipX", "#tipZ"].forEach(id => $(id).addEventListener("keydown", e => { if (e.key === "Enter") tipGoto(); }));

// ---- operate: joystick (on-screen stick + simulated gamepad state) ----
const stick = { active: false, x: 0, y: 0 };
EmioViews.attachStick($("#stick"), stick);
let joyPending = false, wasDragging = false;
function joySend(x, y, enable) {
  const mode = $("#joyMode").value;
  let sx = x, sy = y;
  if (mode === "tip" && model) { const vm = EmioViews.viewMap(model.motor_dirs_deg); [sx, sy] = [vm.x(x, y), vm.z(x, y)]; }
  else if (mode === "bend") [sx, sy] = EmioViews.stickToPairs(x, y);
  joyPending = true;
  sim.api("/api/tip/joy", { mode, control: $("#joyControl").value, x: sx, y: sy, enable })
    .catch(e => showError(e.detail || e.message)).finally(() => joyPending = false);
}
setInterval(() => {
  if (!snap) return;
  const j = snap.tip.joy, active = !!j.active, pad = sim.gamepad(), stale = meta.ws === "closed";
  const st = stick.active ? { x: stick.x, y: stick.y, enable: true } : pad ? { x: pad.x, y: pad.y, enable: pad.enable } : { x: 0, y: 0, enable: false };
  if ($("#stick").clientWidth > 20) EmioViews.drawStick($("#stick"), st.x, st.y, st.enable && active, EmioViews.theme(stage), { labelSize: 14 });  // shared drawStick fails on a hidden (0-width) canvas
  $("#joyPad").textContent = pad ? `${pad.id.replace(/\s*\(.*$/, "")} (simulated)${pad.standard ? "" : " · hold RB to move"}` : "none";
  $("#joyEnable").textContent = st.enable ? (stick.active ? "dragging: moving" : "trigger held: moving") : "released";
  $("#joyStart").disabled = active || !snap.connected || snap.tip.busy || stale;
  $("#joyStop").disabled = !active;
  $("#joyMode").disabled = $("#joyControl").disabled = active;
  $("#joyStatus").textContent = active ? `On · ${j.message}` : j.message ? `Off (${j.message})` : "Off";
  if (active && stick.active && !joyPending) joySend(stick.x, stick.y, true);
  else if (active && wasDragging && !stick.active) joySend(0, 0, false);
  wasDragging = stick.active;
}, 50);
$("#joyStart").addEventListener("click", () => {
  if ($("#joyMode").value === "tip" && !model) return showError("The top-view mapping needs a tip model; choose the motor-pair mapping");
  quiet(call("/api/tip/joy/start", {}));
});
$("#joyStop").addEventListener("click", () => quiet(call("/api/tip/stop", {})));
$("#joyHelpBtn").addEventListener("click", e => { const h = $("#joyHelp"); h.hidden = !h.hidden; e.currentTarget.setAttribute("aria-expanded", !h.hidden); });

// ---- motors panel ----
const sendGoal = (() => { let last = 0, timer = null, args; return (i, a) => { args = [i, a]; const t = Date.now();
  if (t - last >= 60) { last = t; quiet(call(`/api/motor/${args[0]}/goal`, { angle_rad: args[1], relative: true })); }
  else { clearTimeout(timer); timer = setTimeout(() => { last = Date.now(); quiet(call(`/api/motor/${args[0]}/goal`, { angle_rad: args[1], relative: true })); }, 60 - (t - last)); } }; })();
const mdSlider = $("#mdSlider");
mdSlider.addEventListener("pointerdown", () => ui.dragging = true);
document.addEventListener("pointerup", () => ui.dragging = false);
mdSlider.addEventListener("input", () => sendGoal(ui.motor, rad(+mdSlider.value)));
$$("[data-jog]").forEach(b => b.addEventListener("click", () => {
  const m = snap.motors[ui.motor], base = m.goal_rel_rad ?? m.angle_rel_rad;
  quiet(call(`/api/motor/${ui.motor}/goal`, { angle_rad: base + rad(+b.dataset.jog), relative: true }));
}));
$("#mdGo").addEventListener("click", () => { if ($("#mdTarget").value !== "") quiet(call(`/api/motor/${ui.motor}/goal`, { angle_rad: rad(+$("#mdTarget").value), relative: true })); });
$("#mdTorque").addEventListener("click", () => { const m = snap.motors[ui.motor]; quiet(call(`/api/motor/${ui.motor}/torque`, { on: !m.torque })); });
$("#mdReboot").addEventListener("click", () => rebootMotor(ui.motor));
$("#mdZero").addEventListener("click", () => quiet(call(`/api/motor/${ui.motor}/zero`, {})));
$("#mdSettings").addEventListener("click", () => { ui.msMotor = ui.motor; ui.msDirty = false; setPanel("setup"); });

function renderMotorDetail(s, stale, alerts) {
  const m = s.motors[ui.motor], st = motorStatus(m, s, stale), known = !stale && s.connected && m.online;
  $("#mdTitle").textContent = `Motor ${m.id}`; $("#mdWhere").textContent = WHERE[m.id];
  const ms = $("#mdState"); ms.className = `ms s-${st.cls}`; ms.textContent = `${st.word} · ${st.sub}`;
  syncAlertList($("#mdAlerts"), alerts.filter(a => a.motor === m.id), "motor");
  const tq = $("#mdTorque");
  tq.textContent = known ? (m.torque ? "Torque on" : "Torque off") : "Torque —";
  tq.setAttribute("aria-pressed", known && !!m.torque);
  tq.disabled = !known || (!!m.hw_error && !m.torque);
  $("#mdTorqueNote").textContent = !known ? "no data" : m.hw_error && !m.torque ? "reboot first: hardware error" : m.torque ? "holds position; press to release" : "press to hold the current position";
  const zero = m.settings.zero_rad ?? 0, [lo, hi] = m.limits_rad.map(r => deg(r - zero));
  mdSlider.min = lo; mdSlider.max = hi;
  $("#mdLo").textContent = `${Math.round(lo) || 0}°`; $("#mdHi").textContent = `${Math.round(hi) || 0}°`;
  if (!ui.dragging && known) mdSlider.value = deg(m.torque && m.goal_rel_rad != null ? m.goal_rel_rad : m.angle_rel_rad);
  const canMove = known && m.torque;
  mdSlider.disabled = !canMove; $$("[data-jog]").forEach(b => b.disabled = !canMove); $("#mdGo").disabled = !canMove;
  $("#mdClamp").hidden = !m.clamped_recently;
  const rb = $("#mdReboot"); rb.hidden = !(known && m.hw_error); rb.textContent = `Reboot motor ${m.id}…`;
  $("#mdZero").disabled = !known;
  const T = (k, v, u) => `<div><span>${k}</span><b>${v}${u ? `<small>${u}</small>` : ""}</b></div>`, nd = stale ? "stale" : "no data";
  $("#mdTele").innerHTML = known
    ? T("Angle", fmt(deg(m.angle_rel_rad), 2), "°") + T("Goal", m.goal_rel_rad != null ? fmt(deg(m.goal_rel_rad), 1) : "—", "°")
      + T("Measured current", m.ma_per_unit ? Math.round(m.current_raw * m.ma_per_unit) : m.current_raw, m.ma_per_unit ? "mA" : "u (raw)")
      + T("Temperature", m.temp_c != null ? m.temp_c : "—", "°C") + T("Velocity", fmt(m.velocity_rpm, 1), "rpm")
      + T("Cable", m.cable_mm !== undefined ? fmt(m.cable_mm, 2) : "—", m.cable_mm !== undefined ? "mm" : "not calibrated")
    : ["Angle", "Goal", "Measured current", "Temperature", "Velocity", "Cable"].map(k => T(k, "—", nd)).join("");
  const E = (k, v) => `<dt>${k}</dt><dd>${v}</dd>`;
  $("#mdEng").innerHTML = known
    ? E("Pulses", m.pulse) + E("Goal pulses", m.goal_pulse) + E("Radians", fmt(m.angle_rel_rad, 4)) + E("Absolute angle", `${fmt(deg(m.angle_rad), 2)}°`)
      + E("Zero at", m.calibrated ? `${fmt(deg(zero), 2)}° abs` : "not set") + E("Model", m.model || "—") + E("Operating mode", m.mode)
      + E("Soft limits (abs)", `${m.settings.min_deg}° to ${m.settings.max_deg}°`) + E("Current (raw)", `${m.current_raw} u × ${m.ma_per_unit} mA/u`)
      + E("HW error status", m.hw_error ? `${m.hw_error} (${hwText(m.hw_error)})` : "0 (none)") + E("Read glitches", m.read_glitches ?? 0)
    : E("Telemetry", nd) + E("Model", m.model || "—") + E("Soft limits (abs)", `${m.settings.min_deg}° to ${m.settings.max_deg}°`);
}

// ---- setup ----
async function loadPorts() {
  const ports = await sim.api("/api/ports").catch(() => []), sel = $("#port"), cur = sel.value;
  sel.innerHTML = '<option value="">auto (first FTDI)</option>' + ports.map(p => `<option value="${p.device}">${p.device}${p.emio ? " · EMIO" : ""}</option>`).join("");
  sel.value = cur;
}
$("#port").addEventListener("focus", loadPorts); $("#refreshPorts").addEventListener("click", loadPorts);
$("#connect").addEventListener("click", () => quiet(call("/api/connect", { port: $("#port").value || null })));
$("#disconnect").addEventListener("click", () => quiet(call("/api/disconnect", {})));
$("#calAll").addEventListener("click", () => confirmBox("Calibrate all motors?", "Set the current position of every motor as 0?", "Calibrate all")
  .then(ok => { if (ok) quiet(call("/api/zero_all", {})); }));
$("#calList").innerHTML = [0, 1, 2, 3].map(i => `<div><span id="calTxt${i}">Motor ${i}</span><button class="b sm" data-cal="${i}">Calibrate: set 0 here</button></div>`).join("");
$$("[data-cal]").forEach(b => b.addEventListener("click", () => quiet(call(`/api/motor/${b.dataset.cal}/zero`, {}))));
$("#msSeg").innerHTML = [0, 1, 2, 3].map(i => `<button class="b sm" data-ms="${i}" aria-pressed="false">Motor ${i}</button>`).join("");
$$("[data-ms]").forEach(b => b.addEventListener("click", () => { ui.msMotor = +b.dataset.ms; ui.msDirty = false; renderSetup(snap, meta.ws === "closed"); }));
["#msRadius", "#msInvert", "#msMin", "#msMax"].forEach(id => $(id).addEventListener("input", () => ui.msDirty = true));
$("#msSave").addEventListener("click", () => quiet(call(`/api/motor/${ui.msMotor}/settings`, {
  radius_mm: +$("#msRadius").value, invert: $("#msInvert").checked, min_deg: +$("#msMin").value, max_deg: +$("#msMax").value })
  .then(() => ui.msDirty = false)));
$("#msClear").addEventListener("click", () => quiet(call(`/api/motor/${ui.msMotor}/settings`, { clear_zero: true })));
$("#obsAvoid").addEventListener("change", e => quiet(call("/api/obstacle/settings", { avoid: e.target.checked })));
$("#obsClear").addEventListener("change", e => quiet(call("/api/obstacle/settings", { clearance_mm: +e.target.value })));
function setCamMode(m) { ui.camMode = m; renderObstacleSetup(); drawAll(); }
$("#obsPick").addEventListener("click", () => setCamMode(ui.camMode === "pick" ? null : "pick"));
$("#obsMark").addEventListener("click", () => setCamMode(ui.camMode && ui.camMode.mark ? null : { mark: [] }));
$("#obsForget").addEventListener("click", () => quiet(call("/api/obstacle/clear", {})));
$("#invX").addEventListener("change", () => {}); $("#invY").addEventListener("change", () => {});   // gamepad only (simulated pad is scripted)
const O3 = { o3Cloud: "cloud", o3Surface: "surface", o3Zone: "zone", o3ArmCam: "armCam", o3Path: "path" };
Object.entries(O3).forEach(([id, k]) => $(`#${id}`).addEventListener("change", e => { ui.opts3d[k] = e.target.checked; drawAll(); }));

function renderObstacleSetup() {
  if (!snap) return;
  const o = snap.tip.obstacle || {}, feed = !!o.feed && meta.ws !== "closed";
  if (ui.camMode && !feed) ui.camMode = null;
  $("#obsPick").disabled = $("#obsMark").disabled = !feed; $("#obsForget").disabled = !o.rod;
  $("#obsPick").setAttribute("aria-pressed", ui.camMode === "pick"); $("#obsMark").setAttribute("aria-pressed", !!(ui.camMode && ui.camMode.mark));
  const s = $("#obsStatus");
  s.textContent = ui.camMode === "pick" ? "Click the rod in the camera image to use its colour."
    : ui.camMode ? `Click the rod's ${ui.camMode.mark.length ? "other" : "first"} end in the camera image.`
    : o.picking ? "Picking the colour…" : o.marking ? "Measuring the marked ends…" : o.error ? `Failed: ${o.error}`
    : !feed ? "Camera feed off: picking and marking need the camera." : o.rod ? `Rod defined (${o.source === "marked" ? "marked ends" : "found by colour"}).` : "No rod defined.";
  s.style.color = o.error && !ui.camMode ? "var(--fault)" : "";
  const hint = $("#pickHint"); hint.hidden = !ui.camMode; hint.textContent = s.textContent;
}
function renderSetup(s, stale) {
  $("#connInfo").textContent = stale ? "Server unreachable: retrying." : s.connected ? `Connected · ${s.port}${s.mock ? " (simulated)" : ""}` : `Disconnected${s.error ? ` · ${s.error}` : ""}`;
  $("#connect").disabled = stale; $("#disconnect").disabled = stale || !s.connected;
  s.motors.forEach(m => { $(`#calTxt${m.id}`).textContent = `Motor ${m.id} · ${m.calibrated ? `zero at ${fmt(deg(m.settings.zero_rad), 1)}° abs` : "not calibrated"}`; });
  $$("[data-cal]").forEach(b => b.disabled = stale || !s.connected || !s.motors[+b.dataset.cal].online);
  $("#calAll").disabled = stale || !s.connected;
  $$("[data-ms]").forEach(b => b.setAttribute("aria-pressed", +b.dataset.ms === ui.msMotor));
  if (!ui.msDirty) { const st = s.motors[ui.msMotor].settings; $("#msRadius").value = st.radius_mm; $("#msInvert").checked = st.invert; $("#msMin").value = st.min_deg; $("#msMax").value = st.max_deg; }
  const o = s.tip.obstacle || {};
  if (document.activeElement !== $("#obsClear")) $("#obsClear").value = o.clearance_mm;
  $("#obsAvoid").checked = !!o.avoid;
  renderObstacleSetup();
  const mi = $("#modelInfo");
  mi.innerHTML = model ? `<dt>Status</dt><dd>Fitted ${model.fitted}</dd><dt>Samples</dt><dd>${model.n}</dd><dt>Held-out error</dt><dd>median ${model.cv_error_mm.median.toFixed(1)} mm, max ${model.cv_error_mm.max.toFixed(0)} mm</dd><dt>Arm length</dt><dd>${model.length_mm.toFixed(0)} mm</dd>`
    : `<dt>Status</dt><dd><span class="tag caution">No tip model</span></dd><dt>Effect</dt><dd>Tip mapping, Move tip and planning unavailable</dd><dt>Fix</dt><dd><code>fit_tip_model.py data/tip_&lt;time&gt;.csv</code></dd>`;
}

// ---- operate text ----
function setRo(id, value, tagHtml, stale) {
  const el = $(id); $(".v", el).innerHTML = value; $(".k i", el).innerHTML = tagHtml || ""; el.classList.toggle("stale", !!stale);
}
function renderOperateText() {
  if (!snap) return;
  const s = snap, t = s.tip, stale = meta.ws === "closed", f = t.feed, o = t.obstacle || {};
  // camera tip
  if (stale) setRo("#roCam", "—<small>stale</small>", `<span class="tag">no updates</span>`, true);
  else if (!s.connected) setRo("#roCam", "—<small>no data</small>", "", true);
  else if (!f) setRo("#roCam", "—<small>feed off</small>", `<span class="tag caution">off</span>`, true);
  else if (f.age_s >= 0.5) setRo("#roCam", f.markers ? `${fmt(f.x, 0)}, ${fmt(f.z, 0)}<small>mm</small>` : "—", `<span class="tag warn">stale ${f.age_s.toFixed(1)} s</span>`, true);
  else if (!f.markers) setRo("#roCam", "—<small>marker not seen</small>", `<span class="tag warn">lost</span>`, true);
  else setRo("#roCam", `${fmt(f.x, 0)}, ${fmt(f.z, 0)}<small>mm</small>`, `<span class="tag live">live</span>`);
  setRo("#roModel", !stale && t.predicted ? `${fmt(t.predicted[0], 0)}, ${fmt(t.predicted[2], 0)}<small>mm</small>` : `—<small>${stale ? "stale" : !t.model ? "no model" : "no data"}</small>`, "", stale || !t.predicted);
  const camY = f && f.markers && f.age_s < 0.5, y = camY ? f.y : t.predicted ? t.predicted[1] : null;
  setRo("#roY", !stale && y != null ? `${fmt(y, 0)}<small>mm · ${camY ? "camera" : "model"}</small>` : `—<small>${stale ? "stale" : "no data"}</small>`, "", stale || y == null);
  $("#roBend").textContent = `Bend ${!stale && t.pose ? `${fmt(t.pose.A, 2)} / ${fmt(t.pose.B, 2)}` : "—"}`;
  // clearance
  const cl = $("#roClear");
  if (!o.rod) { setRo("#roClear", "—<small>no rod defined</small>", ""); $(".sub", cl).textContent = "Define the rod in Setup to plan around it."; }
  else if (!o.avoid) { setRo("#roClear", o.clear_model_mm != null ? `${fmt(o.clear_model_mm, 0)}<small>mm (model)</small>` : "—", `<span class="tag caution">avoidance off</span>`); $(".sub", cl).textContent = "Moves do not plan around the rod."; }
  else {
    const c = o.clear_model_mm, lvl = c == null ? "" : c < 0 ? "fault" : c < o.clearance_mm ? "warn" : "live";
    const word = c == null ? "" : c < 0 ? "touching" : c < o.clearance_mm ? "below setting" : "clear";
    setRo("#roClear", !stale && c != null ? `${fmt(c, 0)}<small>mm model</small> · ${o.clear_camera_mm != null ? fmt(o.clear_camera_mm, 0) + "<small>mm camera</small>" : "—<small>camera unknown</small>"}` : `—<small>${stale ? "stale" : "no data"}</small>`,
      lvl && !stale ? `<span class="tag ${lvl}">${word}</span>` : "", stale || c == null);
    $(".sub", cl).textContent = `Setting ≥ ${o.clearance_mm} mm · model estimate, not a guarantee (±8 mm)`;
  }
  const rod = $("#roRod .sub");
  rod.textContent = stale ? "Unknown (stale)" : !o.rod ? "No rod" : !o.feed ? `Camera off · using last known` : !o.seen ? `Not seen · last known ${fmt(o.rod_age_s, 0)} s` : `Tracked · ${o.source === "marked" ? "marked ends" : "by colour"}`;
  $("#roRod .k i").innerHTML = stale || !o.rod ? "" : !o.feed || !o.seen ? `<span class="tag warn">degraded</span>` : `<span class="tag live">ok</span>`;
  // banner on the view: preview vs moving vs joystick
  const b = $("#banner"), plan = t.plan;
  b.hidden = false;
  if (t.joy.active) { b.className = "banner joy"; b.textContent = `Joystick control · ${t.joy.mode === "bend" ? "motor pairs" : "tip"} · ${t.joy.message}`; }
  else if (t.busy && t.status.state === "moving") { b.className = "banner moving"; b.textContent = `Moving to ${fmt(t.status.target[0], 0)}, ${fmt(t.status.target[1], 0)} · ${t.status.message}`; }
  else if (plan && plan.preview) { b.className = "banner preview"; b.innerHTML = `<b>Preview: not moving.</b> Path to ${fmt(plan.target[0], 0)}, ${fmt(plan.target[1], 0)}${plan.clear_min_mm != null ? ` · passes ${fmt(plan.clear_min_mm, 0)} mm from the rod` : ""}. Press Move tip to go.`; }
  else b.hidden = true;
  // view note (camera off/stale, server stale)
  const vn = $("#viewNote"), cs = cameraState();
  const note = ui.view === "camera" ? cs.note : stale ? "Server unreachable: view is stale" : !t.model && ui.view !== "camera" ? "" : "";
  vn.hidden = !note; $("#viewNoteText").textContent = note;
  $("#viewNoteTxt").textContent = ui.view === "top" ? "Click to set a target" : ui.view === "3d" ? "Drag to orbit · double-click the surface to preview" : ui.view === "camera" ? (ui.camMode ? "Click the rod" : "Live camera with rod outline") : "Bending plane";
  $("#v3tools").hidden = ui.view !== "3d";
  $("#playBtn").disabled = $("#playBtn2").disabled = !(plan && plan.arms && plan.arms.length > 1);
  const sec = ui.second === "camera" && cameraState().note ? `${VIEW_NAMES.camera} (${cameraState().note.split(":")[0].toLowerCase()})` : VIEW_NAMES[ui.second];
  $("#swapBtn").textContent = `${sec}: show larger`;
  $("#tgtSrc").textContent = ui.target ? `target ${fmt(ui.target[0], 0)}, ${fmt(ui.target[1], 0)}` : "";
}

// ---- main render ----
function render(s, m) {
  if (!s) return;
  snap = s; meta = m || { ws: "open" };
  const stale = meta.ws === "closed";
  model = s.tip && s.tip.model ? (model || sim.model()) : null;
  document.body.classList.toggle("stale", stale);
  // bar
  const sq = $("#connSq");
  sq.className = `sq ${stale ? "fault" : s.connected ? "ok" : "fault"}`;
  $("#connText").innerHTML = stale ? "Server unreachable — retrying" : s.connected ? `Connected${s.mock ? '<span class="simtag">SIMULATED</span>' : ""}` : "Disconnected";
  $("#connSub").textContent = stale ? "readings are stale" : s.connected ? s.port : s.error || "no robot";
  const t = s.tip, j = t.joy;
  $("#modeText").textContent = stale ? "Unknown (no updates)" : !s.connected ? "No control: robot disconnected"
    : j.active ? `Joystick · ${j.mode === "bend" ? "motor pairs" : "tip"} · ${j.control === "position" ? "position" : "jog"} · ${j.message}`
    : t.busy ? `Tip move · ${t.status.message || t.status.state}` : `Idle${t.status.state !== "idle" ? ` · last move ${t.status.state}` : ""}`;
  if (document.activeElement !== speed) speed.value = s.profile_velocity;
  $("#speedVal").textContent = `${(speed.value * 0.229).toFixed(1)} rpm`;
  // alerts
  const alerts = EmioSim.deriveAlerts(s, meta);
  acks.prune(alerts);
  syncAlertList($("#alerts"), alerts, "global");
  announce(alerts);
  const n = l => alerts.filter(a => a.level === l).length, parts = [];
  ["fault", "warning", "caution", "notice"].forEach(l => { if (n(l)) parts.push(`<span class="lvb lv-${l}">${n(l)}</span>`); });
  $("#summCnt").innerHTML = parts.join("");
  $("#summText").textContent = alerts.length ? `${alerts.length} active${alerts.some(a => acks.isAcked(a.id)) ? ` (${alerts.filter(a => acks.isAcked(a.id)).length} acknowledged)` : ""}` : "No active alerts";
  $("#summBtn").disabled = !alerts.length;
  // panels
  if (ui.panel === "operate") {
    renderMap($("#mapOp"), s, stale, false);
    const st = t.status, el = $("#tipStatus");
    el.className = `status st-${stale ? "unknown" : st.state}`;
    el.innerHTML = stale ? "<b>Unknown</b> · no updates" : `<b>${st.state}</b>${st.error_mm != null ? ` · error ${st.error_mm.toFixed(1)} mm` : ""}${st.iteration ? ` · iteration ${st.iteration}` : ""}<br>${st.message || ""}`;
    $("#tipGo").disabled = t.busy || !model || !s.connected || stale;
    $("#tipStop").disabled = !(t.busy && !j.active);
    renderOperateText();
  } else if (ui.panel === "motors") { renderMap($("#mapM"), s, stale, true); renderMotorDetail(s, stale, alerts); }
  else {
    renderSetup(s, stale);
    $("#diagRows").innerHTML = alerts.length ? alerts.map(a => `<tr><td><span class="lvb lv-${a.level}">${LV_WORD[a.level]}</span></td><td>${a.source}</td><td>${a.title}</td><td><code>${a.raw || ""}</code></td></tr>`).join("")
      : `<tr><td colspan="4">No active alerts.</td></tr>`;
  }
  drawAll();
}
sim.subscribe(render);
loadPorts();
