// B · Pilot — behaviour. Everything goes through the shared simulator (shared/sim.js); nothing reaches hardware.
(() => {
  "use strict";
  const $ = (s, el = document) => el.querySelector(s), $$ = (s, el = document) => [...el.querySelectorAll(s)];
  const { deg, rad, hwText } = EmioSim;
  const sim = EmioSim.createSim();
  EmioHarness.init(sim);
  const acks = EmioSim.createAckStore();
  const root = document.documentElement, stage = $("#stage"), mainCv = $("#mainView"), insetCv = $("#insetView");

  // ---------- small helpers ----------
  const f = (v, d = 0) => v == null || Number.isNaN(v) ? "—" : (v < 0 ? "−" : "") + Math.abs(v).toFixed(d);
  const curText = m => m.ma_per_unit ? `${Math.round(m.current_raw * m.ma_per_unit)} mA` : `${m.current_raw} u (raw)`;
  const LEVEL_WORD = { fault: "Fault", warning: "Warning", caution: "Caution", notice: "Notice", ok: "OK", unknown: "No data" };
  function icon(level) {
    const p = {
      fault: '<path d="M7 1h6l6 6v6l-6 6H7l-6-6V7z" fill="currentColor"/><path d="M10 5v6M10 14v1.5" stroke="var(--icon-ink,#fff)" stroke-width="2.4" stroke-linecap="round"/>',
      warning: '<path d="M10 1.5 19 18H1z" fill="currentColor"/><path d="M10 7.5v5M10 15v1" stroke="var(--icon-ink,#000)" stroke-width="2.2" stroke-linecap="round"/>',
      caution: '<circle cx="10" cy="10" r="8.5" fill="none" stroke="currentColor" stroke-width="2.2"/><path d="M10 9v5.5M10 5.6v.8" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/>',
      notice: '<circle cx="10" cy="10" r="8.5" fill="none" stroke="currentColor" stroke-width="2"/><path d="M6 10h8" stroke="currentColor" stroke-width="2.2"/>',
      ok: '<circle cx="10" cy="10" r="8.5" fill="none" stroke="currentColor" stroke-width="2.2"/><path d="m6 10.5 3 3 5-6" fill="none" stroke="currentColor" stroke-width="2.2"/>',
      unknown: '<circle cx="10" cy="10" r="8.5" fill="none" stroke="currentColor" stroke-width="2" stroke-dasharray="3 2.5"/><path d="M10 14.5v.5M8 8a2 2 0 1 1 3 1.7c-.7.4-1 .9-1 1.6" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/>',
      neutral: '<circle cx="10" cy="10" r="4" fill="currentColor"/>',
    }[level] || "";
    return `<svg class="ico" viewBox="0 0 20 20" aria-hidden="true">${p}</svg>`;
  }
  function throttle(fn, ms) {
    let last = 0, timer = null, args;
    return (...a) => { args = a; const n = Date.now(); if (n - last >= ms) { last = n; fn(...args); } else { clearTimeout(timer); timer = setTimeout(() => { last = Date.now(); fn(...args); }, ms - (n - last)); } };
  }
  // keyed list: keeps DOM nodes (and focus) for alerts that stay reported
  function keyed(container, items, key, create, update) {
    const old = new Map([...container.children].filter(n => n.dataset.key).map(n => [n.dataset.key, n]));
    items.forEach((it, i) => {
      const k = key(it); let n = old.get(k);
      if (!n) { n = create(it); n.dataset.key = k; } else old.delete(k);
      update(n, it);
      if (container.children[i] !== n) container.insertBefore(n, container.children[i] || null);
    });
    old.forEach(n => n.remove());
  }

  // ---------- state ----------
  let snap = null, meta = { ws: "open" }, model = null, alerts = [], th = null;
  const qs = new URLSearchParams(location.search);
  const ui = {
    main: "camera", inset: "top", target: null, camMode: null, marks: [], sheet: null, tab: "motors", motor: 0, calMotor: 0,
    calLoaded: null, sec: "conn", cam3d: { ...EmioViews.CAMERA_PRESETS.front }, play: null, announced: new Set(),
  };
  let topMap = null, pick3d = null, camBox = null, toastTimer = 0;
  const stale = () => meta.ws === "closed";
  const unknown = () => !snap || stale() || !snap.connected;
  const store = (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* storage blocked */ } };
  const load = k => { try { return localStorage.getItem(k); } catch (e) { return null; } };

  // ---------- theme ----------
  function setTheme(t) {
    root.dataset.theme = t === "day" ? "day" : "night";
    $("#themeBtn").setAttribute("aria-pressed", String(t === "day"));
    store("dir2-theme", root.dataset.theme);
    th = EmioViews.theme(stage);
    drawViews();
  }
  $("#themeBtn").addEventListener("click", () => setTheme(root.dataset.theme === "day" ? "night" : "day"));

  // ---------- commands ----------
  function showToast(msg) {
    $("#toastMsg").textContent = msg; $("#toast").hidden = false;
    clearTimeout(toastTimer); toastTimer = setTimeout(() => { $("#toast").hidden = true; }, 7000);
  }
  $("#toastClose").addEventListener("click", () => { $("#toast").hidden = true; });
  const api = (path, body) => sim.api(path, body).catch(e => { showToast(e.status === 0 ? "Server unreachable: command not sent." : (e.detail || e.message || "Command failed")); throw e; });
  const fire = (path, body) => api(path, body).catch(() => {});
  const goal = (i, a) => api(`/api/motor/${i}/goal`, { angle_rad: a, relative: true }).catch(() => {});

  // ---------- confirmation dialog (focus trapped; Esc/Cancel close it without any command) ----------
  let dlgResolve = null, dlgReturn = null;
  function confirmDlg(title, text, ok) {
    return new Promise(res => {
      dlgResolve = res; dlgReturn = document.activeElement;
      $("#dlgT").textContent = title; $("#dlgD").textContent = text; $("#dlgOk").textContent = ok;
      $("#modal").hidden = false; $("#dlgCancel").focus();
    });
  }
  function closeDlg(v) {
    $("#modal").hidden = true; const r = dlgResolve; dlgResolve = null;
    if (dlgReturn && dlgReturn.focus) dlgReturn.focus();
    if (r) r(v);
  }
  $("#dlgCancel").addEventListener("click", () => closeDlg(false));
  $("#dlgOk").addEventListener("click", () => closeDlg(true));
  $("#modal").addEventListener("keydown", e => {
    if (e.key === "Escape") { e.stopPropagation(); e.preventDefault(); closeDlg(false); }
    if (e.key === "Tab") {
      const a = [$("#dlgCancel"), $("#dlgOk")], i = a.indexOf(document.activeElement);
      e.preventDefault(); a[(i + (e.shiftKey ? -1 : 1) + a.length) % a.length].focus();
    }
  });
  const askReboot = i => confirmDlg(`Reboot motor ${i}?`, `Reboot motor ${i}? Release cable tension first. It restarts with torque off.`, `Reboot motor ${i}`)
    .then(ok => { if (ok) fire(`/api/motor/${i}/reboot`, {}); });

  // ---------- global bar ----------
  $("#hold").addEventListener("click", () => fire("/api/hold", {}));
  $("#off").addEventListener("click", () => fire("/api/torque_off_all", {}));
  document.addEventListener("keydown", e => { if (e.key === "Escape" && $("#modal").hidden) fire("/api/torque_off_all", {}); });
  const speed = $("#speed");
  let speedDrag = false;
  speed.addEventListener("input", () => { speedDrag = true; $("#speedVal").textContent = `${(speed.value * 0.229).toFixed(1)} rpm`; });
  speed.addEventListener("change", () => { speedDrag = false; fire("/api/speed", { profile_velocity: +speed.value }); });

  function renderBar(s) {
    const dot = $("#connDot"), t = s.tip;
    dot.className = "dot " + (stale() ? "fault" : s.connected ? "ok" : "fault");
    const tag = s.mock ? '<span class="sim-tag">Simulated</span>' : "";
    if (stale()) { $("#connText").textContent = "Server unreachable"; $("#connSub").textContent = "retrying · all readings stale"; }
    else if (s.connected) { $("#connText").textContent = "Connected"; $("#connSub").innerHTML = `${s.port || ""}${tag}`; }
    else { $("#connText").textContent = "Disconnected"; $("#connSub").innerHTML = `${s.error || "not connected"}`; }
    $("#connSub").title = $("#connSub").textContent;
    let mt, ms;
    if (stale()) { mt = "Status unknown"; ms = "no updates from the server"; }
    else if (!s.connected) { mt = "No control"; ms = "robot disconnected"; }
    else if (t.joy.active) { mt = `Joystick · ${t.joy.enable ? "moving" : "holding"}`; ms = `${t.joy.mode === "bend" ? "motor pairs" : "tip direction"} · ${t.joy.control === "position" ? "position" : "jog"}`; }
    else {
      mt = { idle: "Tip control · idle", moving: "Tip · moving", done: "Tip · move done", limited: "Tip · stopped short", failed: "Tip · move failed", stopped: "Tip · stopped" }[t.status.state] || t.status.state;
      ms = t.plan && t.plan.preview && !t.busy ? "preview shown · not moving" : (t.status.message || "ready");
    }
    $("#modeText").textContent = mt; $("#modeSub").textContent = ms; $("#modeSub").title = ms;
    if (!speedDrag && document.activeElement !== speed) { speed.value = s.profile_velocity; $("#speedVal").textContent = `${(s.profile_velocity * 0.229).toFixed(1)} rpm`; }
    // alert summary
    const n = { fault: 0, warning: 0, caution: 0, notice: 0 };
    alerts.forEach(a => n[a.level]++);
    const parts = [["fault", "fault", "faults"], ["warning", "warning", "warnings"], ["caution", "caution", "cautions"]]
      .filter(([k]) => n[k]).map(([k, one, many]) => `<span class="c ${k}">${icon(k)}${n[k]}<span class="lbl">&nbsp;${n[k] === 1 ? one : many}</span></span>`);
    $("#alertSum").innerHTML = parts.length ? parts.join("") : `<span class="c ok">${icon("ok")}No alerts</span>`;
    $("#alertSum").setAttribute("aria-label", `Alerts: ${alerts.length ? Object.entries(n).filter(([, v]) => v).map(([k, v]) => `${v} ${k}`).join(", ") : "none"}. Open the alert list.`);
  }

  // ---------- alerts ----------
  function measureLine(a) { return a.measure.some(([, v]) => v !== "—") ? a.measure.map(([k, v]) => `${k} ${v}`).join(" · ") : a.detail; }
  function runAction(act) {
    if (act.id === "reboot") askReboot(act.motor);
    if (act.id === "connect") fire("/api/connect", { port: $("#port").value || null });
    if (act.id === "setup-motor") { ui.calMotor = act.motor; ui.calLoaded = null; ui.sec = "cal"; openSheet("setup"); }
  }
  function actionButtons(a, cls) {
    return a.actions.map((act, i) => `<button class="btn ${cls || ""}" data-act="${i}">${act.label}</button>`).join("");
  }
  function wireAlert(n, a) {
    n.addEventListener("click", e => {
      const b = e.target.closest("button"); if (!b) return;
      const cur = alerts.find(x => x.id === n.dataset.key); if (!cur) return;
      if (b.dataset.act !== undefined) runAction(cur.actions[+b.dataset.act]);
      if (b.classList.contains("ack")) { acks.ack(cur.id); render(); }
      if (b.classList.contains("det")) openDrawer(cur.id);
    });
  }
  const bannerList = document.createElement("div"); bannerList.style.cssText = "display:flex;flex-direction:column;gap:6px;align-items:center;width:100%";
  const pickbar = document.createElement("div"); pickbar.className = "plate pickbar"; pickbar.hidden = true;
  pickbar.innerHTML = `<b id="pickText"></b><button class="btn" id="pickCancel">Cancel</button>`;
  const moreBtn = document.createElement("button"); moreBtn.className = "more"; moreBtn.hidden = true; moreBtn.addEventListener("click", () => openDrawer());
  $("#banners").append(pickbar, bannerList, moreBtn);
  $("#pickCancel").addEventListener("click", () => { ui.camMode = null; ui.marks = []; render(); });

  function renderAlerts() {
    // banners: faults always (acknowledged ones collapse), unacknowledged warnings; at most 3
    const faults = alerts.filter(a => a.level === "fault"), warns = alerts.filter(a => a.level === "warning" && !acks.isAcked(a.id));
    const shown = [...faults, ...warns].slice(0, 3), rest = alerts.length - shown.length;
    keyed(bannerList, shown, a => a.id, a => {
      const n = document.createElement("div");
      n.setAttribute("role", "group");
      n.innerHTML = `<span class="lvl"></span><div class="t"><span class="src"></span><span class="tt"></span></div><div class="m"></div>
        <div class="row">${actionButtons(a)}<button class="btn ack">Acknowledge</button><span class="ackd">Acknowledged · still active</span><button class="btn det">Details</button></div>`;
      wireAlert(n, a); return n;
    }, (n, a) => {
      const acked = acks.isAcked(a.id);
      n.className = `banner ${a.level}${acked ? " acked" : ""}`;
      n.style.setProperty("--icon-ink", a.level === "fault" ? (acked ? "var(--plate)" : "var(--fault-fill)") : "var(--warn-fill)");
      $(".lvl", n).innerHTML = icon(a.level) + `<span class="sr">${LEVEL_WORD[a.level]}</span>`;
      $(".src", n).textContent = a.source; $(".tt", n).textContent = a.title; $(".m", n).textContent = measureLine(a);
      $(".ack", n).hidden = acked; $(".ackd", n).hidden = !acked;
      n.setAttribute("aria-label", `${LEVEL_WORD[a.level]}: ${a.source}, ${a.title}${acked ? ", acknowledged, still active" : ""}`);
    });
    moreBtn.hidden = rest <= 0; moreBtn.textContent = `${rest} more alert${rest === 1 ? "" : "s"} · open list`;
    // drawer list
    keyed($("#alertList"), alerts, a => a.id, a => {
      const n = document.createElement("article");
      n.innerHTML = `<div class="lv"></div><h3></h3><p class="d"></p><dl></dl>
        <div class="row">${actionButtons(a)}<button class="btn ack">Acknowledge</button><span class="ackd">Acknowledged · still active</span></div>
        <details><summary>Raw code</summary><code></code></details>`;
      wireAlert(n, a); return n;
    }, (n, a) => {
      const acked = acks.isAcked(a.id);
      n.className = `al ${a.level}`;
      $(".lv", n).innerHTML = `${icon(a.level)}${LEVEL_WORD[a.level]} · ${a.source}`;
      $("h3", n).textContent = a.title; $(".d", n).textContent = a.detail;
      $("dl", n).innerHTML = a.measure.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
      $("code", n).textContent = a.raw;
      $(".ack", n).hidden = acked; $(".ackd", n).hidden = !acked;
    });
    if (!alerts.length) $("#alertList").innerHTML = `<p class="empty">No active alerts.</p>`;
    else { const e = $("#alertList .empty"); if (e) e.remove(); }
    // announce new alerts once
    alerts.forEach(a => {
      if (ui.announced.has(a.id)) return;
      ui.announced.add(a.id);
      const text = `${LEVEL_WORD[a.level]}: ${a.source}. ${a.title}.`;
      if (a.level === "fault") $("#liveA").textContent = text; else if (a.level === "warning") $("#liveP").textContent = text;
    });
    const live = new Set(alerts.map(a => a.id)); [...ui.announced].forEach(id => { if (!live.has(id)) ui.announced.delete(id); });
  }
  function openDrawer(focusId) {
    $("#drawer").hidden = false; $("#alertSum").setAttribute("aria-expanded", "true");
    const target = focusId && $(`#alertList [data-key="${focusId}"]`);
    if (target) { target.setAttribute("tabindex", "-1"); target.focus(); } else $("#drawerH").focus();
  }
  function closeDrawer() { $("#drawer").hidden = true; $("#alertSum").setAttribute("aria-expanded", "false"); $("#alertSum").focus(); }
  $("#alertSum").addEventListener("click", () => $("#drawer").hidden ? openDrawer() : closeDrawer());
  $("#closeDrawer").addEventListener("click", closeDrawer);

  // ---------- telemetry (left edge) ----------
  function setRow(id, val, sub, staleRow) {
    $(`#${id}`).textContent = val;
    const row = $(`#${id}`).closest(".trow"); row.classList.toggle("stale", !!staleRow);
    if (sub !== undefined) { const s = $(".sub", row); if (s) s.textContent = sub; }
  }
  function renderTele(s) {
    const t = s.tip, tag = $("#feedTag"), unk = unknown();
    const feedOk = t.feed && t.feed.age_s < 0.5 && t.feed.markers;
    let tg, tl;
    if (unk) [tg, tl] = ["unknown", "No data"];
    else if (!t.feed) [tg, tl] = ["caution", "Camera off"];
    else if (t.feed.age_s >= 0.5) [tg, tl] = ["warning", `Stale ${t.feed.age_s.toFixed(1)} s`];
    else if (!t.feed.markers) [tg, tl] = ["warning", "Marker lost"];
    else [tg, tl] = ["ok", "Camera live"];
    tag.className = `tag ${tg}`; tag.textContent = tl;
    const xz = p => `${f(p[0], 1)}  ${f(p[2], 1)}`;
    if (unk) setRow("tCam", "—", stale() ? "stale: server unreachable" : "no data", true);
    else if (feedOk) setRow("tCam", xz([t.feed.x, t.feed.y, t.feed.z]), "", false);
    else setRow("tCam", "—", !t.feed ? "camera feed off" : t.feed.age_s >= 0.5 ? `last frame ${t.feed.age_s.toFixed(1)} s ago` : "marker not detected", true);
    if (unk) setRow("tMod", "—", "no data", true);
    else setRow("tMod", t.predicted ? xz(t.predicted) : "—", t.predicted ? "" : t.model ? "motors not ready" : "no tip model", !t.predicted);
    if (unk) { setRow("tY", "—", "no data", true); $("#tYLab").textContent = "Height y"; }
    else if (feedOk) { setRow("tY", `${f(t.feed.y, 1)}`, "", false); $("#tYLab").textContent = "Height y · camera"; }
    else if (t.predicted) { setRow("tY", `${f(t.predicted[1], 1)}`, "", false); $("#tYLab").textContent = "Height y · model"; }
    else { setRow("tY", "—", "no data", true); $("#tYLab").textContent = "Height y"; }
    const arm = t.arm_model;
    if (!unk && arm && t.pose) {
      const b = arm[0], p = arm[arm.length - 1], bend = 2 * Math.atan2(Math.hypot(p[0] - b[0], p[2] - b[2]), b[1] - p[1]) * 180 / Math.PI;
      setRow("tBend", `${bend.toFixed(0)}°`, "", false); $("#tBendLab").textContent = `Bend · dA ${f(t.pose.A, 2)} dB ${f(t.pose.B, 2)}`;
    } else { setRow("tBend", "—", unk ? "no data" : "needs model + motors", true); $("#tBendLab").textContent = "Bend"; }
    const o = t.obstacle || {}, el = $("#tClr");
    el.className = "val num";
    if (unk) { setRow("tClr", "—", "no data", true); }
    else if (!t.model) setRow("tClr", "—", "needs the tip model", true);
    else if (!o.rod) setRow("tClr", "None", "no rod known", false);
    else {
      const camStale = t.feed && t.feed.age_s >= 0.5;
      const cm = o.clear_model_mm, track = !t.feed || !o.feed ? "camera off · last known position" : camStale ? "camera stale · last known position"
        : o.seen ? "rod seen" : `rod not seen · last known ${o.rod_age_s.toFixed(0)} s`;
      setRow("tClr", cm == null ? "—" : `${f(cm, 0)} mm`, `${o.avoid ? `keep ${o.clearance_mm} mm` : "avoidance OFF"} · camera ${o.clear_camera_mm != null ? f(o.clear_camera_mm, 0) + " mm" : "unknown"} · ${track}`, cm == null);
      if (cm != null) el.classList.add(cm < 0 ? "clr-fault" : cm < o.clearance_mm ? "clr-warn" : "clr-ok");
    }
  }

  // ---------- views ----------
  function camOff(s) { return stale() || !s.camera || s.camera.age_s == null; }
  function setMain(v) { if (v === ui.main) return; if (v === ui.inset) ui.inset = ui.main; ui.main = v; drawViews(); renderViewsPlate(); }
  $$(".vswitch .btn").forEach(b => b.addEventListener("click", () => setMain(b.dataset.view)));
  $("#inset").addEventListener("click", () => { [ui.main, ui.inset] = [ui.inset, ui.main]; drawViews(); renderViewsPlate(); });
  const VIEW_NAME = { camera: "Camera", top: "Top view", side: "Side view", "3d": "3D view" };
  const g = (cls, extra = "") => `<i class="glyph ${cls}" aria-hidden="true" ${extra}></i>`;
  const LEGEND = {
    camera: `<span>magenta outline, red line: rod as detected</span><span>cyan ring: tracked tip marker</span>`,
    top: `<span>${g("cam")}camera tip</span><span>${g("mod")}model tip</span><span>red ⊕ target</span><span>dashed: path (blue = preview)</span><span>orange / red cells: within clearance / touching</span>`,
    side: `<span>solid: arm (model)</span><span>green dashed: arm through camera tip</span><span>orange: rod</span>`,
    "3d": `<span>solid: arm (model)</span><span>green dashed: arm (camera)</span><span>faint: arm along the path (blue = preview)</span><span>double-click surface: preview</span>`,
  };
  function renderViewsPlate() {
    $$(".vswitch .btn").forEach(b => b.setAttribute("aria-pressed", String(b.dataset.view === ui.main)));
    $("#insetCap").textContent = `${VIEW_NAME[ui.inset]} · tap to swap`;
    $("#inset").setAttribute("aria-label", `Secondary view: ${VIEW_NAME[ui.inset]}. Press to make it the main view.`);
    $("#legend").innerHTML = LEGEND[ui.main];
    mainCv.setAttribute("aria-label", `Main view: ${VIEW_NAME[ui.main]}`);
  }
  function playSnap(s) {
    const p = s.tip.plan; if (!ui.play || !p || p.arms.length < 2) return s;
    const arms = p.arms, fr = Math.min(1, (performance.now() - ui.play.t0) / 2500) * (arms.length - 1);
    const i = Math.min(arms.length - 2, Math.floor(fr)), k = fr - i;
    const arm = arms[i].map((pt, j) => pt.map((v, c) => v + (arms[i + 1][j][c] - v) * k));
    return { ...s, tip: { ...s.tip, plan: { ...p, arms: [arm] } } };
  }
  function drawInto(cv, which, isMain) {
    const s = snap;
    const target = ui.target || (s.tip.busy && s.tip.status.target) || null;
    if (which === "camera") {
      const box = EmioViews.drawCamera(cv, s, model, th, { marks: isMain ? ui.marks : [], off: camOff(s) });
      if (isMain) camBox = box;
    } else if (which === "top") {
      const r = EmioViews.drawTop(cv, s, model, th, { target, obsMap: model ? sim.obstacleMap() : null });
      if (isMain) topMap = r;
    } else if (which === "side") EmioViews.drawSide(cv, s, model, th);
    else {
      const o = { cloud: $("#o3cloud").checked && !camOff(s), surface: $("#o3surface").checked, zone: $("#o3zone").checked, armCam: $("#o3armcam").checked, path: $("#o3path").checked };
      const r = EmioViews.draw3D(cv, isMain ? playSnap(s) : s, sim, model, th, isMain ? ui.cam3d : { ...EmioViews.CAMERA_PRESETS.front, zoom: 0.9 }, o);
      if (isMain) pick3d = r;
    }
  }
  function layoutCuts() {
    if (!matchMedia("(orientation: portrait)").matches) return;
    const st = stage.getBoundingClientRect();
    const top = Math.max($(".tele").getBoundingClientRect().bottom, $(".views").getBoundingClientRect().bottom) - st.top + 4;
    const bot = st.bottom - $(".center").getBoundingClientRect().top + 4;
    stage.style.setProperty("--top-cut", `${Math.round(top)}px`); stage.style.setProperty("--bot-cut", `${Math.round(bot)}px`);
  }
  function drawViews() {
    if (!snap || !th) return;
    layoutCuts();
    // the camera image goes edge to edge unless the space between the plates already shows most of it
    const sr = stage.getBoundingClientRect(), centreW = sr.width - parseFloat(getComputedStyle(root).getPropertyValue("--col-l")) - parseFloat(getComputedStyle(root).getPropertyValue("--col-r")) - 48;
    const camFits = !matchMedia("(orientation: portrait)").matches && centreW * 0.75 >= sr.height * 0.72;
    mainCv.className = "main" + (ui.main !== "camera" || camFits ? " diagram" : "") + ((ui.main === "top" || (ui.main === "camera" && ui.camMode)) ? " click" : "") + (ui.main === "3d" ? " orbit" : "");
    drawInto(mainCv, ui.main, true);
    drawInto(insetCv, ui.inset, false);
    // camera state on the main view
    const s = snap, note = $("#viewNote"), chip = $("#frameChip");
    note.hidden = true; chip.hidden = true;
    if (ui.main === "camera") {
      if (camOff(s)) { note.hidden = false; note.innerHTML = stale() ? "<b>No camera image</b><span>Server unreachable.</span>" : "<b>Camera feed off</b><span>Start tip_feed.py on the robot computer. Moves run open loop (model only).</span>"; }
      else if (s.camera.age_s >= 0.5) { chip.hidden = false; chip.textContent = `Last frame · ${s.camera.age_s.toFixed(1)} s old`; }
    }
    if (ui.main !== "camera" && !model) { note.hidden = false; note.innerHTML = "<b>No tip model</b><span>Top, side and 3D views need fit_tip_model.py. The camera view still works.</span>"; }
  }
  // clicks on the main view: camera picking / top-view target
  mainCv.addEventListener("click", e => {
    const r = mainCv.getBoundingClientRect(), cx = e.clientX - r.left, cy = e.clientY - r.top;
    if (ui.main === "camera" && ui.camMode && camBox) {
      const u = (cx - camBox.x) / camBox.w, v = (cy - camBox.y) / camBox.h;
      if (u < 0 || u > 1 || v < 0 || v > 1) return;
      if (ui.camMode === "pick") { fire("/api/obstacle/pick", { u, v }); ui.camMode = null; }
      else { ui.marks.push([u, v]); if (ui.marks.length === 2) { fire("/api/obstacle/mark", { points: ui.marks, radius_mm: 5 }); ui.camMode = null; setTimeout(() => { ui.marks = []; drawViews(); }, 1200); } }
      render(); return;
    }
    if (ui.main === "top" && topMap) {
      const [x, z] = topMap.toWorld(cx, cy); setTarget(x, z);
      if ($("#clickMove").checked && !(snap && snap.tip.busy)) goto(); else drawViews();
    }
  });
  mainCv.addEventListener("dblclick", e => {
    if (ui.main !== "3d" || !pick3d) return;
    const r = mainCv.getBoundingClientRect(), p = pick3d.pick(e.clientX - r.left, e.clientY - r.top);
    if (p) { setTarget(p.x, p.z); preview(p.x, p.z); }
  });
  EmioViews.attachOrbit(mainCv, ui.cam3d, () => { if (ui.main === "3d") drawViews(); });
  $$("[data-preset]").forEach(b => b.addEventListener("click", () => { Object.assign(ui.cam3d, EmioViews.CAMERA_PRESETS[b.dataset.preset]); setMain("3d"); drawViews(); }));
  $$("#o3cloud, #o3surface, #o3zone, #o3armcam, #o3path").forEach(c => c.addEventListener("change", drawViews));
  $("#playPath").addEventListener("click", () => {
    const p = snap && snap.tip.plan; if (!p || p.arms.length < 2) return;
    setMain("3d"); closeSheet(); ui.play = { t0: performance.now() };
    const step = () => { drawViews(); if (ui.play && performance.now() - ui.play.t0 < 2500) requestAnimationFrame(step); else setTimeout(() => { ui.play = null; drawViews(); }, 700); };
    requestAnimationFrame(step);
  });

  // ---------- move tip (right edge) ----------
  function setTarget(x, z) { $("#tx").value = x.toFixed(0); $("#tz").value = z.toFixed(0); ui.target = [x, z]; }
  function readTarget() {
    if ($("#tx").value === "" || $("#tz").value === "") { showToast("Enter a target X and Z"); return null; }
    return [+$("#tx").value, +$("#tz").value];
  }
  function goto() {
    const t = readTarget(); if (!t) return;
    ui.target = t;
    fire("/api/tip/goto", { x: t[0], z: t[1], closed_loop: $("#closed").checked });
  }
  function preview(x, z) {
    sim.api("/api/tip/preview", { x, z }).catch(e => showToast(`No path: ${e.detail || e.message}`));
  }
  $("#go").addEventListener("click", goto);
  $("#preview").addEventListener("click", () => { const t = readTarget(); if (t) { ui.target = t; preview(t[0], t[1]); } });
  $("#tipStop").addEventListener("click", () => fire("/api/tip/stop", {}));
  ["tx", "tz"].forEach(id => $(`#${id}`).addEventListener("keydown", e => { if (e.key === "Enter") goto(); }));

  function readiness(s) {
    if (stale()) return "Server unreachable.";
    if (!s.connected) return "Not connected.";
    if (!s.tip.model) return "Needs a tip model (fit_tip_model.py).";
    if (s.tip.joy.active) return "Joystick control is on: stop it first.";
    if (s.tip.busy) return "Moving: wait, or press Stop tip.";
    if (s.motors.some(m => !m.online)) return "Not all motors are responding.";
    if (s.motors.some(m => m.hw_error)) return "A motor has a hardware error: reboot it first.";
    if (s.motors.some(m => !m.calibrated)) return "Calibrate every motor's taut zero first.";
    return "";
  }
  function renderMove(s) {
    const why = readiness(s);
    $("#go").disabled = !!why; $("#goWhy").textContent = why;
    $("#preview").disabled = stale() || !s.tip.model;
    $("#tipStop").disabled = stale() || !s.tip.busy;
    if (ui.target == null && s.tip.status.target && document.activeElement !== $("#tx") && document.activeElement !== $("#tz") && $("#tx").value === "") {
      $("#tx").value = Math.round(s.tip.status.target[0]); $("#tz").value = Math.round(s.tip.status.target[1]);
    }
  }
  function renderStatus(s) {
    const t = s.tip, st = t.status, plan = t.plan, el = $("#status"), tag = $("#stTag");
    let cls = "", tg = "neutral", tl = "", msg = "", metaTxt = "";
    if (stale()) { tg = "unknown"; tl = "Unknown"; msg = "No updates from the server."; }
    else if (!s.connected) { tg = "unknown"; tl = "No control"; msg = "Robot disconnected: no commands reach the motors and their state is unknown. Connect from the alert or in Setup."; }
    else if (plan && plan.preview && !t.busy) {
      cls = "preview"; tg = "preview"; tl = "Preview · not moving";
      msg = `Path to ${f(plan.target[0])}, ${f(plan.target[1])}${plan.clear_min_mm != null ? `, passing ${f(plan.clear_min_mm)} mm from the rod` : ""}. ${plan.note ? plan.note.charAt(0).toUpperCase() + plan.note.slice(1) : "Straight there."} Press Move tip to go.`;
    } else if (t.joy.active) { cls = "moving"; tg = t.joy.walls && t.joy.walls.length ? "warning" : "neutral"; tl = "Joystick"; msg = t.joy.message; }
    else {
      const map = { idle: ["neutral", "Idle"], moving: ["neutral", "Moving"], done: ["ok", "Done"], limited: ["warning", "Stopped short"], failed: ["fault", "Failed"], stopped: ["neutral", "Stopped"] };
      [tg, tl] = map[st.state] || ["neutral", st.state]; cls = st.state;
      msg = st.message ? st.message.charAt(0).toUpperCase() + st.message.slice(1) : st.state === "idle" ? "Ready. Set a target, or start the joystick." : "";
      const bits = [];
      if (st.target) bits.push(`target ${f(st.target[0])}, ${f(st.target[1])}`);
      if (st.error_mm != null) bits.push(`error ${st.error_mm.toFixed(1)} mm`);
      if (st.iteration) bits.push(`correction ${st.iteration}`);
      metaTxt = bits.join(" · ");
    }
    el.className = `plate status ${cls}`; tag.className = `tag ${tg}`; tag.textContent = tl;
    $("#stMsg").textContent = msg; $("#stMeta").textContent = metaTxt; $("#stMeta").hidden = !metaTxt;
  }

  // ---------- joystick (left edge) ----------
  const stick = { active: false, x: 0, y: 0 };
  const stickCv = $("#stick");
  EmioViews.attachStick(stickCv, stick);
  let wasDragging = false, joyPending = false;
  $("#joyHelpBtn").addEventListener("click", () => { const h = $("#joyHelp"); h.hidden = !h.hidden; $("#joyHelpBtn").setAttribute("aria-expanded", String(!h.hidden)); });
  $("#joyStart").addEventListener("click", () => {
    if ($("#joyMode").value === "tip" && !model) return showToast("The top-view mapping needs a tip model; choose the motor-pair mapping");
    api("/api/tip/joy/start", {}).then(() => api("/api/tip/joy", { mode: $("#joyMode").value, control: $("#joyControl").value, x: 0, y: 0, enable: false })).catch(() => {});
  });
  $("#joyStop").addEventListener("click", () => fire("/api/tip/stop", {}));
  function stickToSend(x, y) {
    if ($("#joyMode").value === "tip" && model) { const vm = EmioViews.viewMap(model.motor_dirs_deg); return [vm.x(x, y), vm.z(x, y)]; }
    return EmioViews.stickToPairs(x, y);
  }
  function joyTick() {
    if (!snap || !th) return;
    const j = snap.tip.joy, active = j.active && !stale(), pad = sim.gamepad();
    let dx = 0, dy = 0, en = false;
    if (stick.active) { dx = stick.x; dy = stick.y; en = true; }
    else if (pad) {
      if (model && j.mode === "tip") { const vm = EmioViews.viewMap(model.motor_dirs_deg); dx = vm.u(pad.x, pad.y); dy = vm.v(pad.x, pad.y); } else { dx = pad.x; dy = pad.y; }
      en = pad.enable;
    }
    EmioViews.drawStick(stickCv, dx, dy, en && active, th, { labelSize: 13 });
    $("#padName").textContent = pad ? pad.id + (pad.standard ? "" : " (hold RB to move)") : "none";
    $("#padEnable").textContent = stick.active ? "dragging: moving" : pad && pad.enable ? "trigger held: moving" : "released";
    if (active && stick.active && !joyPending) {
      joyPending = true;
      const [x, y] = stickToSend(stick.x, stick.y);
      sim.api("/api/tip/joy", { mode: $("#joyMode").value, control: $("#joyControl").value, x, y, enable: true }).catch(e => showToast(e.detail || e.message)).finally(() => { joyPending = false; });
    } else if (active && !stick.active && wasDragging) {
      sim.api("/api/tip/joy", { mode: $("#joyMode").value, control: $("#joyControl").value, x: 0, y: 0, enable: false }).catch(() => {});
    }
    wasDragging = stick.active;
  }
  setInterval(joyTick, 50);
  function renderJoy(s) {
    const j = s.tip.joy, active = j.active && !stale(), tag = $("#joyTag");
    if (!active) { tag.className = "tag neutral"; tag.textContent = "Off"; }
    else if (j.walls && j.walls.length) { tag.className = "tag warning"; tag.textContent = "Force limit"; }
    else if (j.enable) { tag.className = "tag ok"; tag.textContent = "Moving"; }
    else { tag.className = "tag neutral"; tag.textContent = "Holding"; }
    $("#joyMode").disabled = $("#joyControl").disabled = active;
    if (active) { $("#joyMode").value = j.mode || "tip"; $("#joyControl").value = j.control || "rate"; }
    $("#joyStart").disabled = active || stale() || !s.connected || s.tip.busy;
    $("#joyStop").disabled = !active;
    const msg = $("#joyMsg");
    msg.textContent = stale() ? "status unknown" : active ? j.message : j.message ? `off (${j.message})` : "off";
    msg.classList.toggle("warn", !!(active && j.walls && j.walls.length));
  }

  // ---------- dock: mini motor map + panels ----------
  const LAYOUT = [1, 0, 2, 3];  // seen from above: 1 top-left, 0 top-right, 2 bottom-left, 3 bottom-right
  function mstat(m) {
    if (unknown()) return { level: "unknown", word: "No data" };
    if (!m.online) return { level: "fault", word: "Not responding" };
    if (m.hw_error) return { level: "fault", word: `Fault · ${hwText(m.hw_error)}` };
    if (m.overridden) return { level: "warning", word: "Overridden" };
    if (!m.calibrated) return { level: "caution", word: "Uncalibrated" };
    return { level: m.torque ? "ok" : "neutral", word: (m.torque ? "Torque on" : "Torque off") + (m.moving ? " · moving" : "") };
  }
  const mini = $("#mini");
  LAYOUT.forEach(i => { const b = document.createElement("button"); b.dataset.m = i; b.addEventListener("click", () => { ui.motor = i; openSheet("motors"); }); mini.appendChild(b); });
  function renderDock(s) {
    $$("button", mini).forEach(b => {
      const m = s.motors[+b.dataset.m], st = mstat(m);
      b.className = ["fault", "warning", "caution", "unknown"].includes(st.level) ? st.level : "";
      b.innerHTML = `${b.dataset.m}${["fault", "warning", "caution", "unknown"].includes(st.level) ? icon(st.level) : ""}`;
      b.style.setProperty("--icon-ink", st.level === "fault" ? "var(--fault-fill)" : "var(--warn-fill)");
      b.setAttribute("aria-label", `Motor ${b.dataset.m}: ${st.word}. Open motor details.`);
    });
  }

  // ---------- bottom sheet ----------
  function openSheet(tab) {
    ui.sheet = true; ui.tab = tab; $("#sheet").hidden = false;
    $("#openMotors").setAttribute("aria-expanded", String(tab === "motors")); $("#openSetup").setAttribute("aria-expanded", String(tab === "setup"));
    renderSheet(); const tb = tab === "motors" ? $("#tabMotors") : $("#tabSetup"); tb.focus();
  }
  function closeSheet() { if (!ui.sheet) return; ui.sheet = false; $("#sheet").hidden = true; $("#openMotors").setAttribute("aria-expanded", "false"); $("#openSetup").setAttribute("aria-expanded", "false"); }
  $("#openMotors").addEventListener("click", () => ui.sheet && ui.tab === "motors" ? closeSheet() : openSheet("motors"));
  $("#openSetup").addEventListener("click", () => ui.sheet && ui.tab === "setup" ? closeSheet() : openSheet("setup"));
  $("#closeSheet").addEventListener("click", () => { closeSheet(); $("#openMotors").focus(); });
  $("#tabMotors").addEventListener("click", () => openSheet("motors"));
  $("#tabSetup").addEventListener("click", () => openSheet("setup"));
  $(".tabs").addEventListener("keydown", e => { if (e.key === "ArrowRight" || e.key === "ArrowLeft") openSheet(ui.tab === "motors" ? "setup" : "motors"); });
  function renderSheet() {
    if (!ui.sheet || !snap) return;
    $("#tabMotors").setAttribute("aria-selected", String(ui.tab === "motors")); $("#tabSetup").setAttribute("aria-selected", String(ui.tab === "setup"));
    $("#tabMotors").setAttribute("aria-pressed", String(ui.tab === "motors")); $("#tabSetup").setAttribute("aria-pressed", String(ui.tab === "setup"));
    $("#pMotors").hidden = ui.tab !== "motors"; $("#pSetup").hidden = ui.tab !== "setup";
    const j = snap.tip.joy;
    $("#sheetNote").textContent = (j.active ? "Joystick control is still on (the gamepad keeps working). " : "") + "Joystick and Move tip are under this sheet. Hold all and Torque off all stay in the top bar.";
    if (ui.tab === "motors") renderMotors(snap); else renderSetup(snap);
  }

  // ---------- motors ----------
  const mmap = $("#mmap");
  LAYOUT.forEach(i => {
    const b = document.createElement("button"); b.className = "tile"; b.dataset.m = i;
    b.innerHTML = `<span class="h"><b>Motor ${i}</b><span class="s pairlbl">${i % 2 === 0 ? "pair A" : "pair B"}</span></span><span class="st"></span><span class="v num"></span><span class="s sub"></span>`;
    b.addEventListener("click", () => { ui.motor = i; renderMotors(snap); });
    mmap.appendChild(b);
  });
  const pairs = document.createElement("p"); pairs.className = "pairs"; pairs.textContent = "Laid out as seen from above. Pairs oppose: A = 0 · 2, B = 1 · 3."; mmap.appendChild(pairs);
  const slider = $("#mdSlider"); let sliderDrag = false;
  const sendGoal = throttle(d => goal(ui.motor, rad(d)), 60);
  slider.addEventListener("pointerdown", () => { sliderDrag = true; });
  document.addEventListener("pointerup", () => { sliderDrag = false; });
  slider.addEventListener("input", () => sendGoal(+slider.value));
  slider.addEventListener("keyup", () => { sliderDrag = false; });
  $$(".jog [data-jog]").forEach(b => b.addEventListener("click", () => {
    const m = snap.motors[ui.motor], base = m.goal_rel_rad ?? m.angle_rel_rad; goal(ui.motor, base + rad(+b.dataset.jog));
  }));
  $("#mdGo").addEventListener("click", () => { if ($("#mdTarget").value !== "") goal(ui.motor, rad(+$("#mdTarget").value)); });
  $("#mdTorque").addEventListener("click", () => fire(`/api/motor/${ui.motor}/torque`, { on: $("#mdTorque").getAttribute("aria-checked") !== "true" }));
  $("#mdZero").addEventListener("click", () => fire(`/api/motor/${ui.motor}/zero`, {}));
  $("#mdLimits").addEventListener("click", () => { ui.calMotor = ui.motor; ui.calLoaded = null; ui.sec = "cal"; openSheet("setup"); });
  function renderMotors(s) {
    const unk = unknown();
    $$(".tile", mmap).forEach(b => {
      const i = +b.dataset.m, m = s.motors[i], st = mstat(m);
      b.setAttribute("aria-pressed", String(i === ui.motor));
      b.classList.toggle("fault", st.level === "fault");
      $(".st", b).className = `st ${st.level}`; $(".st", b).innerHTML = `${icon(st.level)}${st.word}`;
      $(".v", b).textContent = unk || !m.online ? "—" : `${f(deg(m.angle_rel_rad), 1)}°`;
      $(".sub", b).textContent = unk || !m.online ? (unk ? "no data" : "no reply") : `goal ${m.goal_rel_rad != null ? f(deg(m.goal_rel_rad), 1) + "°" : "—"} · ${curText(m)} · ${m.temp_c} °C`;
      b.setAttribute("aria-label", `Motor ${i}: ${st.word}`);
    });
    const i = ui.motor, m = s.motors[i], st = mstat(m), zero = m.settings.zero_rad ?? 0, on = !unk && m.online;
    $("#mdTitle").textContent = `Motor ${i}`;
    const tags = [`<span class="tag ${st.level === "ok" ? "ok" : st.level}">${st.word}</span>`];
    if (on && m.moving) tags.push(`<span class="tag neutral">Moving</span>`);
    $("#mdTags").innerHTML = tags.join("");
    const tq = $("#mdTorque");
    tq.setAttribute("aria-checked", String(!!(on && m.torque)));
    tq.disabled = !on || (!!m.hw_error && !m.torque);
    tq.title = tq.disabled ? (m.hw_error ? "Reboot the motor first" : "No data from this motor") : "Torque on holds the current position";
    // this motor's alerts, beside it
    keyed($("#mdAlerts"), alerts.filter(a => a.motor === i), a => a.id, a => {
      const n = document.createElement("div");
      n.innerHTML = `<b></b><p class="d"></p><p class="ms"></p><div class="row">${actionButtons(a).replace(/data-act/g, "data-act")}<button class="btn ack">Acknowledge</button><span class="ackd k">Acknowledged · still active</span></div>`;
      n.addEventListener("click", e => {
        const btn = e.target.closest("button"); if (!btn) return;
        const cur = alerts.find(x => x.id === n.dataset.key); if (!cur) return;
        if (btn.dataset.act !== undefined) runAction(cur.actions[+btn.dataset.act]);
        if (btn.classList.contains("ack")) { acks.ack(cur.id); render(); }
      });
      return n;
    }, (n, a) => {
      n.className = `malert ${a.level}`;
      $("b", n).innerHTML = `${icon(a.level)} ${LEVEL_WORD[a.level]}: ${a.title}`;
      $(".d", n).textContent = a.detail; $(".ms", n).textContent = a.measure.map(([k, v]) => `${k}: ${v}`).join(" · ");
      $(".ack", n).hidden = acks.isAcked(a.id); $(".ackd", n).hidden = !acks.isAcked(a.id);
    });
    const [lo, hi] = m.limits_rad.map(r => deg(r - zero));
    slider.min = lo; slider.max = hi;
    $("#mdLo").textContent = `${Math.round(lo)}°`; $("#mdHi").textContent = `${Math.round(hi)}°`;
    if (!sliderDrag && on && document.activeElement !== slider) slider.value = deg(m.torque && m.goal_rel_rad != null ? m.goal_rel_rad : m.angle_rel_rad);
    const canMove = on && m.torque;
    slider.disabled = !canMove; $$(".jog [data-jog]").forEach(b => { b.disabled = !canMove; }); $("#mdGo").disabled = !canMove;
    $("#mdClamp").hidden = !m.clamped_recently;
    $("#mdZero").disabled = !on;
    $("#mdAngle").textContent = on ? `${f(deg(m.angle_rel_rad), 2)}°` : "—";
    $("#mdGoal").textContent = on && m.goal_rel_rad != null ? `${f(deg(m.goal_rel_rad), 1)}°` : "—";
    $("#mdCur").textContent = on ? curText(m) : "—";
    $("#mdTemp").textContent = on && m.temp_c != null ? `${m.temp_c} °C` : "—";
    $("#mdVel").textContent = on ? `${f(m.velocity_rpm, 1)} rpm` : "—";
    $("#mdCable").textContent = on && m.cable_mm !== undefined ? `${f(m.cable_mm, 2)} mm` : on ? "uncalibrated" : "—";
    const eng = [
      ["Pulses", on ? m.pulse : "—"], ["Radians", on ? m.angle_rel_rad.toFixed(4) : "—"], ["Absolute", on ? `${f(deg(m.angle_rad), 2)}°` : "—"],
      ["Zero at", m.calibrated ? `${f(deg(zero), 2)}° abs` : "not set"], ["Model", m.model || "—"], ["Operating mode", m.mode ?? "—"],
      ["Current (raw)", on ? `${m.current_raw} u` : "—"], ["Hardware error", on ? (m.hw_error ? `${m.hw_error} (${hwText(m.hw_error)})` : "none") : "—"],
      ["Read glitches", on ? m.read_glitches : "—"],
    ];
    $("#mdEng").innerHTML = eng.map(([k, v]) => `<div><dt>${k}</dt><dd class="num">${v}</dd></div>`).join("");
  }

  // ---------- setup ----------
  $$("#snav .btn").forEach(b => b.addEventListener("click", () => { ui.sec = b.dataset.sec; renderSetup(snap); }));
  $("#port").addEventListener("focus", () => sim.api("/api/ports").then(ports => {
    const sel = $("#port"), cur = sel.value;
    sel.innerHTML = '<option value="">auto (first FTDI)</option>' + ports.map(p => `<option value="${p.device}">${p.device}${p.emio ? " · EMIO" : ""}</option>`).join("");
    sel.value = cur;
  }).catch(() => {}));
  $("#connect").addEventListener("click", () => fire("/api/connect", { port: $("#port").value || null }));
  $("#disconnect").addEventListener("click", () => fire("/api/disconnect", {}));
  $("#calAll").addEventListener("click", () => confirmDlg("Calibrate all motors?", "Set the current position of every motor as 0?", "Set 0 on all").then(ok => { if (ok) fire("/api/zero_all", {}); }));
  const calSel = $("#calSel");
  [0, 1, 2, 3].forEach(i => { const b = document.createElement("button"); b.className = "btn"; b.textContent = `${i}`; b.setAttribute("aria-label", `Motor ${i}`); b.addEventListener("click", () => { ui.calMotor = i; ui.calLoaded = null; renderSetup(snap); }); calSel.appendChild(b); });
  $("#sSave").addEventListener("click", () => api(`/api/motor/${ui.calMotor}/settings`, { radius_mm: +$("#sRadius").value, invert: $("#sInvert").checked, min_deg: +$("#sMin").value, max_deg: +$("#sMax").value })
    .then(() => { ui.calLoaded = null; showSaved("Saved."); }).catch(() => {}));
  $("#sZero").addEventListener("click", () => fire(`/api/motor/${ui.calMotor}/zero`, {}));
  $("#sClear").addEventListener("click", () => fire(`/api/motor/${ui.calMotor}/settings`, { clear_zero: true }));
  function showSaved(t) { $("#liveP").textContent = t; }
  $("#obsAvoid").addEventListener("change", e => fire("/api/obstacle/settings", { avoid: e.target.checked }));
  $("#obsClear").addEventListener("change", e => fire("/api/obstacle/settings", { clearance_mm: +e.target.value }));
  function startCamMode(m) { ui.camMode = m; ui.marks = []; closeSheet(); setMain("camera"); render(); }
  $("#obsPick").addEventListener("click", () => startCamMode("pick"));
  $("#obsMark").addEventListener("click", () => startCamMode("mark"));
  $("#obsForget").addEventListener("click", () => fire("/api/obstacle/clear", {}));

  function renderSetup(s) {
    if (!s) return;
    $$("#snav .btn").forEach(b => b.setAttribute("aria-current", b.dataset.sec === ui.sec ? "true" : "false"));
    $$("#snav .btn").forEach(b => b.setAttribute("aria-pressed", String(b.dataset.sec === ui.sec)));
    $$(".ssec").forEach(sec => { sec.hidden = sec.dataset.sec !== ui.sec; });
    const unk = unknown();
    $("#scState").textContent = stale() ? "Server unreachable — retrying" : s.connected ? `Connected · ${s.port}${s.mock ? " (simulated)" : ""}` : "Disconnected";
    $("#scErr").textContent = s.error || "none";
    $("#connect").disabled = stale(); $("#disconnect").disabled = stale() || !s.connected;
    // calibration
    $$("button", calSel).forEach(b => b.setAttribute("aria-pressed", String(+b.textContent === ui.calMotor)));
    const m = s.motors[ui.calMotor];
    $("#calZero").textContent = m.calibrated ? `set at ${f(deg(m.settings.zero_rad), 2)}° absolute` : "not set (angles relative to pulse 2048)";
    if (ui.calLoaded !== ui.calMotor) {
      $("#sRadius").value = m.settings.radius_mm; $("#sInvert").checked = m.settings.invert;
      $("#sMin").value = m.settings.min_deg; $("#sMax").value = m.settings.max_deg; ui.calLoaded = ui.calMotor;
    }
    $("#calAll").disabled = unk; $("#sZero").disabled = unk || !m.online;
    // model
    const mi = model;
    $("#modelInfo").innerHTML = mi
      ? `<p class="lead">Constant-curvature fit: the arm bends as one arc. Tip mode stays below 120° of bend.</p><dl class="kv"><dt>Samples</dt><dd class="num">${mi.samples}</dd><dt>Held-out error</dt><dd class="num">median ${mi.cv_error_mm.median.toFixed(1)} mm · max ${mi.cv_error_mm.max.toFixed(0)} mm</dd><dt>Fitted</dt><dd>${mi.fitted}</dd><dt>Arm length</dt><dd class="num">${mi.length_mm.toFixed(0)} mm</dd><dt>Attachment</dt><dd class="num">${mi.base.map(v => f(v, 1)).join(", ")}</dd></dl>`
      : `<p class="lead"><b>No tip model.</b> Tip mapping, Move tip and obstacle planning are unavailable; motor controls and the motor-pair joystick still work.</p><pre>~/emio-venv/bin/python collect_tip_data.py
~/emio-venv/bin/python fit_tip_model.py data/tip_&lt;time&gt;.csv</pre>`;
    // obstacle
    const o = s.tip.obstacle || {};
    if (document.activeElement !== $("#obsAvoid")) $("#obsAvoid").checked = !!o.avoid;
    if (document.activeElement !== $("#obsClear")) $("#obsClear").value = o.clearance_mm;
    $("#obsRod").textContent = o.rod ? `${o.source === "marked" ? "marked ends" : "found by colour"} · radius ${o.rod.radius.toFixed(0)} mm · ${o.rod.visible_mm ? `${o.rod.visible_mm.toFixed(0)} mm seen` : "length from marks"}` : "none";
    $("#obsCam").textContent = unk ? "no data" : !o.feed ? "camera feed off: pick and mark unavailable" : o.seen ? "rod seen" : "rod not seen";
    $("#obsPick").disabled = $("#obsMark").disabled = unk || !o.feed;
    $("#obsForget").disabled = !o.rod;
    $("#obsStatus").textContent = o.picking ? "Picking the colour…" : o.marking ? "Measuring the marked ends…" : o.error ? `Last attempt failed: ${o.error}.` : "";
    $("#playPath").disabled = !(s.tip.plan && s.tip.plan.arms.length > 1);
    // diagnostics
    $("#diagRows").innerHTML = alerts.length ? alerts.map(a => `<tr><td>${LEVEL_WORD[a.level]}</td><td>${a.source}</td><td>${a.title}</td><td><code>${a.raw}</code></td></tr>`).join("")
      : `<tr><td colspan="4">No active alerts.</td></tr>`;
  }

  // ---------- camera pick/mark bar ----------
  function renderPickbar(s) {
    const o = s.tip.obstacle || {};
    if (ui.camMode && (unknown() || !o.feed)) { ui.camMode = null; ui.marks = []; showToast("Camera feed off: picking cancelled."); }
    pickbar.hidden = !ui.camMode;
    if (ui.camMode) $("#pickText").textContent = ui.camMode === "pick" ? "Pick rod colour: click the rod in the camera view" : `Mark rod ends: click the rod's ${ui.marks.length ? "other" : "first"} end`;
  }

  // ---------- main render ----------
  function render() {
    const s = snap; if (!s) return;
    model = s.tip && s.tip.model ? (model || sim.model()) : null;
    alerts = EmioSim.deriveAlerts(s, meta); acks.prune(alerts);
    renderBar(s); renderAlerts(); renderTele(s); renderJoy(s); renderMove(s); renderStatus(s); renderDock(s); renderPickbar(s);
    renderViewsPlate(); renderSheet(); drawViews();
  }
  setTheme(qs.get("theme") || load("dir2-theme") || "night");
  sim.subscribe((s, m) => { snap = s; meta = m; render(); });
  window.addEventListener("resize", drawViews);
  matchMedia("(orientation: portrait)").addEventListener("change", drawViews);
})();
