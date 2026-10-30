// Direction A · Ward monitor: behaviour. Simulated backend only (shared/sim.js); commands go through sim.api with the
// same paths and bodies as the real GUI (motor_gui/app/static/index.html).
(function () {
  "use strict";
  const sim = EmioSim.createSim();
  EmioHarness.init(sim);
  const V = EmioViews, { deg, rad, hwText } = EmioSim;
  const $ = (s, el = document) => el.querySelector(s), $$ = (s, el = document) => [...el.querySelectorAll(s)];
  const fmt = (v, d = 1) => (v == null || Number.isNaN(v)) ? "—" : (v < 0 ? "−" : "") + Math.abs(v).toFixed(d);
  const esc = s => String(s).replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const POS = ["top right", "top left", "bottom left", "bottom right"], PAIR = ["A", "B", "A", "B"];

  // ---------------- state ----------------
  let snap = null, meta = { ws: "open" }, model = null, th = null;
  const ui = {
    task: "operate", view: "camera", motor: 0, setupMotor: 0, target: null, camMode: null, bandAll: false, expanded: new Set(),
    cam3d: { ...V.CAMERA_PRESETS.front }, opts3d: { cloud: true, crop: 250, surface: true, zone: true, armCam: true, path: true },
    play: null, inv: { x: false, y: false }, setupDirty: false, seen: new Set(),
  };
  const acks = EmioSim.createAckStore();

  // ---------------- theme ----------------
  function setTheme(t) {
    document.documentElement.dataset.theme = t;
    $("#themeBtn").textContent = t === "night" ? "Day" : "Night";
    $("#themeBtn").setAttribute("aria-pressed", t === "night");
    $("#themeBtn").setAttribute("aria-label", t === "night" ? "Switch to day theme" : "Switch to night theme");
    try { localStorage.setItem("emio-a-theme", t); } catch { /* storage unavailable */ }
    th = V.theme(document.documentElement);
    draw();
  }
  {
    const q = new URLSearchParams(location.search).get("theme");
    let t = q === "night" || q === "day" ? q : null;
    if (!t) { try { t = localStorage.getItem("emio-a-theme"); } catch { t = null; } }
    document.documentElement.dataset.theme = t || "day";
  }
  $("#themeBtn").addEventListener("click", () => setTheme(document.documentElement.dataset.theme === "night" ? "day" : "night"));

  // ---------------- commands, errors, confirmation ----------------
  let toastTimer = null;
  function showError(msg) {
    $("#toastText").textContent = msg; $("#toast").classList.toggle("on", !!msg);
    clearTimeout(toastTimer); if (msg) toastTimer = setTimeout(() => showError(""), 8000);
  }
  $("#toastX").addEventListener("click", () => showError(""));
  const api = (p, b) => sim.api(p, b).then(r => { showError(""); return r; }, e => { showError(e.detail || e.message || "Command failed"); throw e; });
  const fire = (p, b) => api(p, b).catch(() => {});

  let dlgResolve = null, dlgReturn = null;
  function confirmDlg(title, text, yes) {
    $("#dlgT").textContent = title; $("#dlgD").textContent = text; $("#dlgYes").textContent = yes;
    dlgReturn = document.activeElement; $("#dlgBack").hidden = false; $("#dlgNo").focus();
    return new Promise(r => dlgResolve = r);
  }
  function closeDlg(v) { if (!dlgResolve) return; $("#dlgBack").hidden = true; const r = dlgResolve; dlgResolve = null; if (dlgReturn) dlgReturn.focus(); r(v); }
  $("#dlgYes").addEventListener("click", () => closeDlg(true));
  $("#dlgNo").addEventListener("click", () => closeDlg(false));
  $("#dlgBack").addEventListener("keydown", e => {
    if (e.key === "Tab") { const f = [$("#dlgNo"), $("#dlgYes")], i = f.indexOf(document.activeElement); e.preventDefault(); f[(i + (e.shiftKey ? 1 : 1)) % 2].focus(); }
  });
  const rebootMotor = i => confirmDlg(`Reboot motor ${i}?`, `Reboot motor ${i}? Release cable tension first. It restarts with torque off.`, `Reboot motor ${i}`)
    .then(ok => { if (ok) fire(`/api/motor/${i}/reboot`, {}); });

  document.addEventListener("keydown", e => {
    if (e.key !== "Escape") return;
    if (dlgResolve) { e.preventDefault(); closeDlg(false); return; }   // like the native confirm(): Esc cancels the dialog
    fire("/api/torque_off_all", {});
  });

  // ---------------- global bar ----------------
  const speed = $("#speed");
  speed.addEventListener("input", () => $("#speedOut").textContent = `${(speed.value * 0.229).toFixed(1)} rpm`);
  speed.addEventListener("change", () => fire("/api/speed", { profile_velocity: +speed.value }));
  $("#holdAll").addEventListener("click", () => fire("/api/hold", {}));
  $("#torqueOff").addEventListener("click", () => fire("/api/torque_off_all", {}));
  const known = () => !!(snap && meta.ws === "open" && snap.connected);

  function renderBar() {
    const s = snap, wsDown = meta.ws === "closed";
    const conn = $("#conn");
    conn.classList.toggle("bad", wsDown || !s.connected);
    $("#connText").textContent = wsDown ? "Server unreachable — retrying" : s.connected ? `Connected · ${s.port}` : "Disconnected";
    $("#simTag").hidden = !s.mock;
    const t = s.tip, j = t.joy;
    let mode;
    if (wsDown) mode = `Unknown <span>— no updates from the server</span>`;
    else if (!s.connected) mode = `No control <span>— robot disconnected</span>`;
    else if (j.active) mode = `Joystick · ${j.mode === "bend" ? "motor pairs" : "tip"}, ${j.control === "position" ? "position" : "jog"} <span>— ${esc(j.message)}</span>`;
    else mode = `Tip control · ${esc(t.status.state)}${t.status.message ? ` <span>— ${esc(t.status.message)}</span>` : ""}`;
    $("#modeText").innerHTML = mode; $("#modeText").title = $("#modeText").textContent;
    if (document.activeElement !== speed) { speed.value = s.profile_velocity; $("#speedOut").textContent = `${(s.profile_velocity * 0.229).toFixed(1)} rpm`; }
    speed.disabled = wsDown;
    $("#holdAll").disabled = wsDown || !s.connected;
    $("#torqueOff").disabled = wsDown;
  }

  // ---------------- alerts ----------------
  const LEVEL = { fault: { ic: "!!", word: "Fault" }, warning: { ic: "!", word: "Warning" }, caution: { ic: "i", word: "Caution" }, notice: { ic: "–", word: "Notice" } };
  function alertRow(a, scope) {
    const key = `${scope}:${a.id}`;
    const el = document.createElement("div");
    el.dataset.id = a.id;
    el.innerHTML = `<span class="ic" aria-hidden="true"></span>
      <div class="tx"><span class="vh lv"></span><span class="asrc"></span> · <span class="ti"></span><span class="ms"></span><span class="ackd" hidden>Acknowledged · still active</span></div>
      <div class="ac"><button class="btn small" data-a="more" aria-expanded="false">Details</button><button class="btn small" data-a="ack">Acknowledge</button></div>
      <div class="more" hidden><p class="de"></p><dl></dl><div class="row acts"></div>
        <details><summary>Technical detail</summary><code class="raw"></code></details></div>`;
    const acts = $(".acts", el);
    (a.actions || []).forEach(x => {
      const b = document.createElement("button"); b.className = "btn small"; b.textContent = x.label;
      b.addEventListener("click", () => alertAction(x)); acts.appendChild(b);
    });
    acts.hidden = !(a.actions || []).length;
    el.dataset.key = key;
    $("[data-a=more]", el).addEventListener("click", () => { ui.expanded.has(key) ? ui.expanded.delete(key) : ui.expanded.add(key); renderAlerts(); });
    $("[data-a=ack]", el).addEventListener("click", () => { acks.ack(a.id); renderAlerts(); });
    return el;
  }
  function updateRow(el, a) {
    const acked = acks.isAcked(a.id), open = ui.expanded.has(el.dataset.key);
    el.className = `al ${a.level}${acked ? " acked" : ""}`;
    $(".ic", el).textContent = LEVEL[a.level].ic;
    $(".lv", el).textContent = `${LEVEL[a.level].word}${acked ? ", acknowledged" : ""}: `;
    $(".asrc", el).textContent = a.source;
    $(".ti", el).textContent = a.title;
    $(".ms", el).textContent = open ? "" : a.measure && a.measure.length ? a.measure.map(([k, v]) => `${k} ${v}`).join(" · ") : `— ${a.detail}`;
    $(".ackd", el).hidden = !acked;
    const ack = $("[data-a=ack]", el); ack.hidden = acked || a.transient;
    const more = $("[data-a=more]", el); more.setAttribute("aria-expanded", open); more.textContent = open ? "Hide" : "Details";
    $(".more", el).hidden = !open;
    if (open) {
      $(".de", el).textContent = a.detail;
      $("dl", el).innerHTML = (a.measure || []).map(([k, v]) => `<div><dt>${esc(k)}</dt><dd>${esc(v)}</dd></div>`).join("");
      $(".raw", el).textContent = a.raw || "";
    }
  }
  function alertAction(x) {
    if (x.id === "reboot") rebootMotor(x.motor);
    else if (x.id === "connect") fire("/api/connect", { port: $("#port") ? $("#port").value || null : null });
    else if (x.id === "setup-motor") { ui.setupMotor = x.motor; ui.setupDirty = false; setTask("setup"); requestAnimationFrame(() => { const s = $("#sec-cal"); if (s) s.scrollIntoView({ block: "start" }); }); }
  }
  // keyed list so rows (and their focused buttons) survive the 10 Hz updates
  function syncList(container, alerts, limit, scope = container.id) {
    const have = new Map($$(":scope > .al", container).map(e => [e.dataset.id, e]));
    const shown = limit ? alerts.slice(0, limit) : alerts;
    shown.forEach((a, i) => {
      let el = have.get(a.id);
      if (!el) el = alertRow(a, scope);
      have.delete(a.id);
      updateRow(el, a);
      if (container.children[i] !== el) container.insertBefore(el, container.children[i] || null);
    });
    have.forEach(e => e.remove());
  }
  let alerts = [];
  function renderAlerts() {
    const band = $("#band");
    acks.prune(alerts);
    if (!alerts.length) {
      band.className = "band quiet";
      band.innerHTML = meta.ws === "open" && snap && snap.connected ? `<b>●</b> No active alerts` : `No alerts reported`;
    } else {
      band.className = "band";
      $$(":scope > :not(.al):not(.bandmore)", band).forEach(e => e.remove());
      const limit = ui.bandAll ? 0 : 3;
      let wrap = band;
      syncList(wrap, alerts, limit);
      let more = $(".bandmore", band);
      const extra = alerts.length - 3;
      if (extra > 0) {
        if (!more) { more = document.createElement("button"); more.className = "bandmore"; more.addEventListener("click", () => { ui.bandAll = !ui.bandAll; renderAlerts(); }); band.appendChild(more); }
        more.textContent = ui.bandAll ? "Show fewer alerts" : `+ ${extra} more alert${extra > 1 ? "s" : ""}`;
        band.appendChild(more);
      } else if (more) more.remove();
    }
    const n = l => alerts.filter(a => a.level === l).length, f = n("fault"), w = n("warning"), c = n("caution") + n("notice");
    $("#summaryCounts").innerHTML = !alerts.length ? "none" :
      [f && `<span class="c f">${f} fault${f > 1 ? "s" : ""}</span>`, w && `<span class="c w">${w} warning${w > 1 ? "s" : ""}</span>`, c && `<span class="c a">${c} other</span>`].filter(Boolean).join(" ");
    $("#summary").setAttribute("aria-expanded", ui.bandAll);
    const mf = alerts.filter(a => a.motor != null && (a.level === "fault" || a.level === "warning")).length;
    $("#motorFaultN").hidden = !mf; $("#motorFaultN").textContent = mf;
    // announcements: new faults assertive, the rest polite
    const ids = new Set(alerts.map(a => a.id));
    alerts.forEach(a => { if (!ui.seen.has(a.id)) { (a.level === "fault" ? $("#liveA") : $("#liveP")).textContent = `${LEVEL[a.level].word}: ${a.source}, ${a.title}`; } });
    ui.seen = ids;
    if (ui.task === "motors") syncList($("#mAlerts"), alerts.filter(a => a.motor === ui.motor), 0);
  }
  $("#summary").addEventListener("click", () => { ui.bandAll = !ui.bandAll; renderAlerts(); const first = $("#band .al button"); if (ui.bandAll && first) first.focus(); });

  // ---------------- tasks ----------------
  function setTask(t) {
    ui.task = t;
    $$(".tab").forEach(b => b.setAttribute("aria-selected", b.dataset.task === t));
    ["operate", "motors", "setup"].forEach(k => $(`#p-${k}`).hidden = k !== t);
    if (t === "setup") { ui.setupDirty = false; loadPorts(); }
    if (snap) render(snap, meta);
  }
  $$(".tab").forEach(b => b.addEventListener("click", () => setTask(b.dataset.task)));
  $(".tabs").addEventListener("keydown", e => {
    const tabs = $$(".tab"), i = tabs.indexOf(document.activeElement);
    if (i < 0 || !["ArrowLeft", "ArrowRight"].includes(e.key)) return;
    const n = tabs[(i + (e.key === "ArrowRight" ? 1 : tabs.length - 1)) % tabs.length]; n.focus(); setTask(n.dataset.task);
  });

  // ---------------- operate: views ----------------
  const mainCv = $("#mainCv"), insetCv = $("#insetCv");
  const VIEW_NAME = { camera: "Camera", top: "Top view", side: "Side view", "3d": "3D view" };
  const secondary = () => ui.view === "camera" ? "top" : "camera";
  function setView(v) {
    ui.view = v;
    $$(".vtab").forEach(b => b.setAttribute("aria-selected", b.dataset.view === v));
    $("#ctx3d").hidden = v !== "3d";
    mainCv.classList.toggle("click", v === "top");
    $("#insetLabel").textContent = VIEW_NAME[secondary()];
    $("#inset").setAttribute("aria-label", `${VIEW_NAME[secondary()]} (secondary). Activate to make it the main view`);
    renderOperateText(); draw();
  }
  $$(".vtab").forEach(b => b.addEventListener("click", () => setView(b.dataset.view)));
  $("#inset").addEventListener("click", () => setView(secondary()));
  $$("[data-preset]").forEach(b => b.addEventListener("click", () => { Object.assign(ui.cam3d, V.CAMERA_PRESETS[b.dataset.preset]); draw(); }));
  V.attachOrbit(mainCv, ui.cam3d, () => { if (ui.view === "3d") draw(); });
  let topMap = null, pick3d = null;
  mainCv.addEventListener("click", e => {
    if (ui.view !== "top" || !topMap || !snap) return;
    const r = mainCv.getBoundingClientRect(), [x, z] = topMap.toWorld(e.clientX - r.left, e.clientY - r.top);
    setTarget(x, z);
    if ($("#clickMove").checked && !snap.tip.busy) tipGoto();
  });
  mainCv.addEventListener("dblclick", e => {
    if (ui.view !== "3d" || !pick3d) return;
    const r = mainCv.getBoundingClientRect(), p = pick3d.pick(e.clientX - r.left, e.clientY - r.top);
    if (!p) return;
    setTarget(p.x, p.z);
    api("/api/tip/preview", { x: p.x, z: p.z }).then(() => { ui.play = { t0: performance.now() }; draw(); }).catch(() => {});
  });
  $("#playPath").addEventListener("click", () => { ui.play = { t0: performance.now() }; draw(); });

  function planFor(s) {      // hide a preview that no longer matches the target the operator has set
    const p = s.tip.plan;
    if (p && p.preview && ui.target && p.target && Math.hypot(p.target[0] - ui.target[0], p.target[1] - ui.target[1]) > 1) return null;
    return p;
  }
  function viewSnap() {
    const s = snap, p = planFor(s);
    let tip = { ...s.tip, plan: p };
    if (ui.play && p && p.arms.length > 1) {
      const f = Math.min(1, (performance.now() - ui.play.t0) / 2500) * (p.arms.length - 1), i = Math.min(p.arms.length - 2, Math.floor(f)), k = f - i;
      tip.arm_model = p.arms[i].map((q, j) => q.map((v, c) => v + (p.arms[i + 1][j][c] - v) * k));
      if (f >= p.arms.length - 1) ui.play = null; else requestAnimationFrame(draw);
    }
    if (meta.ws === "closed") tip = { ...tip, feed: null, arm_camera: null };
    return { ...s, tip };
  }
  function drawView(cv, which, s, big) {
    const opts = { ...ui.opts3d };
    if (which === "top") { const m = V.drawTop(cv, s, model, th, { target: ui.target || (s.tip.busy && s.tip.status.target) || null, obsMap: model ? sim.obstacleMap() : null }); if (big) topMap = m; }
    else if (which === "side") V.drawSide(cv, s, model, th);
    else if (which === "camera") V.drawCamera(cv, s, model, th, { off: camOff(s) });
    else { const p = V.draw3D(cv, s, sim, model, th, ui.cam3d, opts); if (big) pick3d = p; }
  }
  const camOff = s => meta.ws === "closed" || !s.camera || s.camera.age_s == null;
  let raf = 0;
  function draw() {
    if (raf) return;
    raf = requestAnimationFrame(() => {
      raf = 0;
      if (!snap || !th) return;
      const s = viewSnap();
      if (ui.task === "operate") { drawView(mainCv, ui.view, s, true); drawView(insetCv, secondary(), s, false); }
      if (ui.task === "setup") drawSetupCam(s);
    });
  }

  function renderOperateText() {
    if (!snap) return;
    const s = snap, t = s.tip, p = planFor(s), ban = $("#vbanner");
    let b = null;
    if (ui.view === "camera" && camOff(s)) b = ["off", meta.ws === "closed" ? "Camera image unavailable — server unreachable" : "Camera feed off — no image"];
    else if (ui.view === "camera" && s.camera.age_s >= 1.5) b = ["stale", `Camera image stale — ${s.camera.age_s.toFixed(1)} s old`];
    else if (p && p.preview) b = ["preview", `Preview — not moving · press Move tip to go${p.clear_min_mm != null ? ` · passes ${p.clear_min_mm.toFixed(0)} mm from rod` : ""}`];
    else if (t.busy && t.joy.active) b = ["moving", t.joy.enable ? "Moving — joystick" : "Joystick control on — holding"];
    else if (t.busy) b = ["moving", "Moving — along the planned path"];
    ban.hidden = !b; if (b) { ban.className = `vbanner ${b[0]}`; ban.textContent = b[1]; }
    $("#playPath").disabled = !(p && p.arms && p.arms.length > 1);
    $("#vhint").textContent = {
      camera: "Simulated camera · tip marker ringed cyan · rod outlined magenta, fitted line red",
      top: $("#clickMove").checked ? "Click: the tip moves there" : "Click: set the target only",
      side: "Bending plane through the arm · dashed: rod and camera arc",
      "3d": "Drag to orbit · scroll to zoom · double-click the tip surface to preview (does not move)",
    }[ui.view];
    const L = {
      camera: [],
      top: [["var(--view-camera)", "tip, camera", 1], ["transparent;border:2px solid var(--view-model)", "tip, model", 1], ["var(--view-target)", "target", 0],
            ["var(--view-rod)", "rod", 0], ["var(--view-near)", "inside clearance", 0], ["var(--view-hit)", "would touch", 0]],
      side: [["var(--view-arm)", "arm, model", 0], ["var(--view-arm-cam)", "arm through camera tip", 0], ["var(--view-rod)", "rod", 0]],
      "3d": [["var(--view-arm)", "arm, model", 0], ["var(--view-arm-cam)", "arm, camera", 0], ["var(--view-rod)", "rod + clearance zone", 0],
             ["var(--view-path)", "planned path", 0], ["var(--view-preview)", "preview path", 0]],
    }[ui.view];
    $("#vlegend").innerHTML = L.length ? L.map(([c, l, r]) => `<span><i style="background:${c};${r ? "border-radius:50%" : ""}"></i>${l}</span>`).join("")
      : `<span>Live camera from tip_feed.py (simulated here). The tip position readings at right say whether they come from the camera or the model.</span>`;
  }

  // ---------------- operate: move controls ----------------
  function setTarget(x, z) { ui.target = [x, z]; $("#tx").value = x.toFixed(0); $("#tz").value = z.toFixed(0); draw(); }
  ["#tx", "#tz"].forEach(id => $(id).addEventListener("change", () => { if ($("#tx").value !== "" && $("#tz").value !== "") { ui.target = [+$("#tx").value, +$("#tz").value]; draw(); } }));
  function tipGoto() {
    if ($("#tx").value === "" || $("#tz").value === "") return showError("Enter a target X and Z");
    const x = +$("#tx").value, z = +$("#tz").value; ui.target = [x, z];
    fire("/api/tip/goto", { x, z, closed_loop: $("#closedLoop").checked });
  }
  $("#moveTip").addEventListener("click", tipGoto);
  $("#stopTip").addEventListener("click", () => fire("/api/tip/stop", {}));
  $("#clickMove").addEventListener("change", renderOperateText);
  function renderMove() {
    const s = snap, t = s.tip, st = t.status, wsDown = meta.ws === "closed";
    $("#moveTip").disabled = wsDown || t.busy || !t.model || !s.connected;
    $("#stopTip").disabled = wsDown || !(t.busy && !t.joy.active);
    const tg = st.target ? `target ${fmt(st.target[0], 0)}, ${fmt(st.target[1], 0)} mm` : "";
    $("#moveStat").innerHTML = wsDown ? `<b>Unknown</b> — no updates from the server` :
      `<b class="st-${esc(st.state)}">${esc(st.state)}</b>${st.error_mm != null ? ` · error ${st.error_mm.toFixed(1)} mm` : ""}${st.iteration ? ` · correction ${st.iteration}` : ""}${tg ? ` · ${tg}` : ""}<br>${esc(st.message || (t.model ? "" : "No tip model: Move tip is unavailable."))}`;
  }

  // ---------------- operate: joystick ----------------
  const stickCv = $("#stick"), vs = { active: false, x: 0, y: 0 };
  V.attachStick(stickCv, vs);
  let joyPending = false, wasDragging = false;
  function joySend(x, y, enable) {
    const mode = $("#joyMode").value, vm = model ? V.viewMap(model.motor_dirs_deg) : null;
    const [sx, sy] = mode === "tip" ? (vm ? [vm.x(x, y), vm.z(x, y)] : [x, y]) : V.stickToPairs(x, y);
    joyPending = true;
    sim.api("/api/tip/joy", { mode, control: $("#joyControl").value, x: sx, y: sy, enable })
      .catch(e => showError(e.detail || e.message)).finally(() => joyPending = false);
  }
  function joyTick() {
    if (!snap) return;
    const j = snap.tip.joy, active = !!j.active && meta.ws === "open", pad = sim.gamepad();
    const st = vs.active ? { x: vs.x, y: vs.y, enable: true } : pad ? { x: pad.x, y: pad.y, enable: pad.enable } : { x: 0, y: 0, enable: false };
    if (stickCv.clientWidth > 20) V.drawStick(stickCv, st.x, st.y, st.enable && active, th, { labelSize: 13 });  // shared drawStick can't draw a hidden (0 px) canvas
    if (active && vs.active && !joyPending) joySend(vs.x, vs.y, true);
    else if (active && !vs.active && wasDragging) joySend(0, 0, false);
    wasDragging = vs.active;
    const en = st.enable ? (vs.active ? "dragging: moving" : "trigger held: moving") : "released";
    $("#padSt").innerHTML = `${pad ? `Gamepad <b>${esc(pad.id.replace(/\s*\(.*$/, "").slice(0, 32))}</b>${pad.standard ? "" : " (hold RB to move)"} + on-screen stick`
      : "On-screen stick (no gamepad)"} · <span class="en${st.enable && active ? " on" : ""}">${en}</span>`;
  }
  setInterval(joyTick, 50);
  $("#joyStart").addEventListener("click", () => {
    if ($("#joyMode").value === "tip" && !model) return showError("The top-view mapping needs a tip model; choose the motor-pair mapping");
    fire("/api/tip/joy/start", {});
  });
  $("#joyStop").addEventListener("click", () => fire("/api/tip/stop", {}));
  const showCtl = k => { $(".controls").dataset.show = k; $$("[data-ctl]").forEach(x => x.setAttribute("aria-selected", x.dataset.ctl === k)); };
  $$("[data-ctl]").forEach(b => b.addEventListener("click", () => showCtl(b.dataset.ctl)));
  let joyWasActive = false;
  function renderJoy() {
    const s = snap, j = s.tip.joy, wsDown = meta.ws === "closed", active = j.active && !wsDown;
    $("#joyStart").disabled = wsDown || active || !s.connected || s.tip.busy;
    $("#joyStop").disabled = !active;
    $("#joyMode").disabled = $("#joyControl").disabled = active;
    const m = $("#joyMsg");
    m.textContent = wsDown ? "Unknown — no updates from the server" : active ? j.message : j.message ? `Off (${j.message})` : "Off. Start joystick control, then drag the stick (or hold RT / R2 on a gamepad). Gamepad B = torque off all.";
    m.classList.toggle("warn", active && j.walls && j.walls.length > 0);
    $("#joyOnTag").textContent = active ? (j.enable ? "moving" : "on") : "";
    if (active && !joyWasActive) showCtl("joy");   // compact screens: show the joystick panel when joystick control starts (display only)
    joyWasActive = active;
  }

  // ---------------- operate: parameter column ----------------
  const P = $("#params");
  P.innerHTML = `
    <div class="prmhead">Tip position</div>
    <div class="prm" id="pCam"><span class="l">Tip x, z <span class="src cam">CAMERA</span></span><span class="v"></span><span class="s"><span class="a"></span><span class="b t"></span></span></div>
    <div class="prm minor" id="pMod"><span class="l">Tip x, z <span class="src mod">MODEL</span></span><span class="v"></span><span class="s"><span class="a"></span><span class="b t"></span></span></div>
    <div class="prmhead">Obstacle</div>
    <div class="prm" id="pClr"><span class="l">Arm–rod clearance <span class="src mod">MODEL</span></span><span class="v"></span><span class="s"><span class="a t"></span><span class="b"></span></span></div>
    <div class="prm minor" id="pRod"><span class="l">Rod</span><span class="v" style="font-family:var(--sans);font-size:17px"></span><span class="s"><span class="a t"></span><span class="b t"></span></span></div>
    <div class="prmhead">Motors · angle from taut zero</div>
    ${[0, 1, 2, 3].map(i => `<button class="prm" data-m="${i}" id="pM${i}" aria-label="Motor ${i}: open motor controls"><span class="l">Motor ${i} <span style="font-weight:500;color:var(--muted)">· ${POS[i]}</span><span class="flags"></span></span><span class="v"></span><span class="s"><span class="a"></span><span class="b"></span></span></button>`).join("")}`;
  $$("[data-m]", P).forEach(b => b.addEventListener("click", () => { ui.motor = +b.dataset.m; setTask("motors"); }));
  function setPrm(id, { v, a = "", b = "", cls = "" }) {
    const el = $(id); el.className = `prm${el.classList.contains("minor") ? " minor" : ""}${cls ? " " + cls : ""}`;
    $(".v", el).innerHTML = v; $(".s .a", el).textContent = a; $(".s .b", el).textContent = b;
  }
  function motorFlags(m, s) {
    const out = [];
    if (!known()) return [["n", "no data"]];
    if (!m.online) return [["f", "no response"]];
    if (m.hw_error) out.push(["f", hwText(m.hw_error)]);
    if (m.overridden) out.push(["w", "overridden"]);
    if (!m.calibrated) out.push(["a", "uncalibrated"]);
    if (!m.torque) out.push(["n", "torque off"]);
    if (m.moving) out.push(["n", "moving"]);
    if (m.clamped_recently) out.push(["n", "clamped"]);
    return out;
  }
  const cur = m => m.ma_per_unit ? `${Math.round(m.current_raw * m.ma_per_unit)} mA` : `${m.current_raw} u (raw)`;
  function renderParams() {
    const s = snap, t = s.tip, ok = meta.ws === "open", f = t.feed;
    if (!ok) setPrm("#pCam", { v: "—", a: "stale", b: "server unreachable", cls: "unknown" });
    else if (!f) setPrm("#pCam", { v: "—", a: "no data", b: "camera feed off", cls: "unknown" });
    else if (f.age_s >= 0.5) setPrm("#pCam", { v: "—", a: `stale · ${f.age_s.toFixed(1)} s old`, b: "use model", cls: "unknown warnv" });
    else if (!f.markers) setPrm("#pCam", { v: "—", a: "marker not seen", b: "use model", cls: "unknown warnv" });
    else setPrm("#pCam", { v: `${fmt(f.x)}, ${fmt(f.z)}`, a: `y ${fmt(f.y)} mm`, b: `live · ${(f.age_s * 1000).toFixed(0)} ms` });
    if (ok && t.predicted && s.connected) setPrm("#pMod", { v: `${fmt(t.predicted[0])}, ${fmt(t.predicted[2])}`, a: `y ${fmt(t.predicted[1])} · dA ${fmt(t.pose.A, 2)} dB ${fmt(t.pose.B, 2)}`, b: "from motor angles" });
    else setPrm("#pMod", { v: "—", a: "no data", b: !t.model ? "no tip model" : !s.connected ? "motors disconnected" : "motors not ready", cls: "unknown" });
    const o = t.obstacle;
    if (!o.rod) setPrm("#pClr", { v: "—", a: "no rod known", b: "", cls: "unknown" });
    else if (!ok || o.clear_model_mm == null) setPrm("#pClr", { v: "—", a: `limit ${o.clearance_mm} mm`, b: "no data", cls: "unknown" });
    else {
      const c = o.clear_model_mm, bad = o.avoid && c < o.clearance_mm;
      setPrm("#pClr", { v: `${fmt(c, 0)}<small>mm</small>`, a: `limit ${o.clearance_mm} mm${o.avoid ? "" : " (avoid off)"}`, b: `camera ${o.clear_camera_mm != null ? fmt(o.clear_camera_mm, 0) + " mm" : "unknown"}`, cls: c < 0 ? "bad" : bad ? "warnv" : "" });
    }
    const rodState = !o.rod ? "None" : !o.feed ? "Last known" : o.seen ? "Tracked" : "Not seen";
    setPrm("#pRod", { v: rodState, a: o.rod ? `${o.source === "marked" ? "marked ends" : "colour"} · r ${o.rod.radius} mm` : "orange rod detected by tip_feed.py",
      b: o.rod ? (o.avoid ? `planning around it` : "avoidance off") + (!o.feed ? " · camera off" : !o.seen ? ` · ${o.rod_age_s.toFixed(0)} s old` : "") : "", cls: o.rod && (!o.feed || !o.seen) ? "warnv" : "" });
    s.motors.forEach(m => {
      const el = $(`#pM${m.id}`), fl = motorFlags(m, s), on = known() && m.online;
      $(".flags", el).innerHTML = fl.map(([c, l]) => `<span class="flag ${c}">${esc(l)}</span>`).join("");
      $(".v", el).innerHTML = !on ? "—" : m.hw_error ? `<span style="color:var(--fault-text)">FAULT</span>` : `${fmt(deg(m.angle_rel_rad))}°`;
      $(".s .a", el).textContent = on ? `goal ${m.goal_rel_rad != null ? fmt(deg(m.goal_rel_rad)) + "°" : "—"}` : "";
      $(".s .b", el).textContent = on ? `${cur(m)} · ${m.temp_c != null ? m.temp_c + " °C" : "—"}` : "no data";
      el.className = `prm${!on ? " unknown" : ""}${on && m.hw_error ? " bad" : ""}`;
    });
  }

  // ---------------- motors panel ----------------
  $("#mlist").innerHTML = `<div class="layout">Physical layout from above: 1 top left, 0 top right, 2 bottom left, 3 bottom right. Pairs A = 0/2, B = 1/3.</div>` +
    [0, 1, 2, 3].map(i => `<button class="mrow" role="tab" data-mm="${i}" aria-selected="false" aria-controls="mdet"><span class="id">${i}<small>pair ${PAIR[i]}</small></span>
      <span><span class="flags"></span><span class="meta"></span></span><span class="ang"></span></button>`).join("");
  $$("[data-mm]").forEach(b => b.addEventListener("click", () => { ui.motor = +b.dataset.mm; render(snap, meta); }));
  $("#mlist").addEventListener("keydown", e => {
    if (!["ArrowUp", "ArrowDown"].includes(e.key)) return; e.preventDefault();
    ui.motor = (ui.motor + (e.key === "ArrowDown" ? 1 : 3)) % 4; render(snap, meta); $(`[data-mm="${ui.motor}"]`).focus();
  });
  $("#mdet").innerHTML = `
    <h2><span id="mTitle"></span><span class="flags" id="mFlags"></span><span style="flex:1"></span>
      <button class="switch" id="mTorque" role="switch" aria-checked="false"><span class="knob" aria-hidden="true"></span><span id="mTorqueL">Torque</span></button></h2>
    <div class="alerts" id="mAlerts" aria-label="This motor's alerts"></div>
    <div class="bigang"><div><span class="k">Angle</span><span class="v" id="mAng"></span></div><div><span class="k">Goal</span><span class="v sm" id="mGoal"></span></div>
      <div><span class="k">Measured current</span><span class="v sm" id="mCur"></span></div><div><span class="k">Temperature</span><span class="v sm" id="mTemp"></span></div></div>
    <div class="slider"><label class="vh" for="mSlider">Goal angle</label><input type="range" id="mSlider" step="0.1"></div>
    <div class="rng"><span id="mLo"></span><span class="clamp" id="mClamp" hidden>at limit: command clamped</span><span id="mHi"></span></div>
    <div class="row" style="margin-top:8px">
      <button class="btn" data-jog="-5">−5°</button><button class="btn" data-jog="-1">−1°</button><button class="btn" data-jog="1">+1°</button><button class="btn" data-jog="5">+5°</button>
      <label class="field" style="margin-left:10px">Target <input type="number" id="mTarget" step="0.1"> °</label><button class="btn primary" id="mGo">Go</button>
    </div>
    <p class="note" id="mWhy"></p>
    <div class="group"><h3>Recovery and calibration</h3><div class="row">
      <button class="btn" id="mReboot" hidden title="Clears the hardware error. Release cable tension first: the motor comes back with torque off.">Reboot motor…</button>
      <button class="btn" id="mZero" title="The current position becomes 0 (angle and cable displacement)">Calibrate: set 0 here</button>
      <button class="btn quiet" id="mToSetup">Drum radius, sign and soft limits…</button></div></div>
    <div class="group"><h3>Telemetry</h3><div class="tele" id="mTele"></div>
      <details class="eng"><summary>Engineering details</summary><div class="tele" id="mEng"></div></details></div>`;
  let mDragging = false, lastGoalSend = 0, goalTimer = null;
  const sendGoal = d => {
    const go = () => { lastGoalSend = performance.now(); fire(`/api/motor/${ui.motor}/goal`, { angle_rad: rad(d), relative: true }); };
    clearTimeout(goalTimer);
    if (performance.now() - lastGoalSend >= 60) go(); else goalTimer = setTimeout(go, 60);
  };
  const mSlider = $("#mSlider");
  mSlider.addEventListener("pointerdown", () => mDragging = true);
  document.addEventListener("pointerup", () => mDragging = false);
  mSlider.addEventListener("input", () => sendGoal(+mSlider.value));
  $$("[data-jog]").forEach(b => b.addEventListener("click", () => {
    const m = snap.motors[ui.motor], base = m.goal_rel_rad ?? m.angle_rel_rad;
    fire(`/api/motor/${ui.motor}/goal`, { angle_rad: base + rad(+b.dataset.jog), relative: true });
  }));
  $("#mGo").addEventListener("click", () => { if ($("#mTarget").value !== "") fire(`/api/motor/${ui.motor}/goal`, { angle_rad: rad(+$("#mTarget").value), relative: true }); });
  $("#mTorque").addEventListener("click", () => fire(`/api/motor/${ui.motor}/torque`, { on: !snap.motors[ui.motor].torque }));
  $("#mReboot").addEventListener("click", () => rebootMotor(ui.motor));
  $("#mZero").addEventListener("click", () => fire(`/api/motor/${ui.motor}/zero`, {}));
  $("#mToSetup").addEventListener("click", () => { ui.setupMotor = ui.motor; ui.setupDirty = false; setTask("setup"); });

  function renderMotors() {
    const s = snap;
    s.motors.forEach(m => {
      const b = $(`[data-mm="${m.id}"]`), on = known() && m.online;
      b.setAttribute("aria-selected", m.id === ui.motor); b.tabIndex = m.id === ui.motor ? 0 : -1;
      b.classList.toggle("off", !on);
      $(".flags", b).innerHTML = motorFlags(m, s).map(([c, l]) => `<span class="flag ${c}">${esc(l)}</span>`).join(" ") || `<span class="flag n">torque on</span>`;
      $(".meta", b).textContent = on ? `${cur(m)} · ${m.temp_c} °C` : "no data";
      $(".ang", b).innerHTML = !on ? "—" : m.hw_error ? `<span style="color:var(--fault-text)">FAULT</span>` : `${fmt(deg(m.angle_rel_rad))}°`;
    });
    const m = s.motors[ui.motor], on = known() && m.online, zero = m.settings.zero_rad ?? 0;
    $("#mTitle").textContent = `Motor ${m.id} · ${POS[m.id]}, pair ${PAIR[m.id]}`;
    $("#mFlags").innerHTML = motorFlags(m, s).map(([c, l]) => `<span class="flag ${c}">${esc(l)}</span>`).join(" ");
    const tq = $("#mTorque");
    tq.setAttribute("aria-checked", !!(on && m.torque)); $("#mTorqueL").textContent = !on ? "Torque — unknown" : m.torque ? "Torque on" : "Torque off";
    tq.disabled = !on || (!!m.hw_error && !m.torque);
    $("#mAng").textContent = on ? `${fmt(deg(m.angle_rel_rad), 2)}°` : "—";
    $("#mGoal").textContent = on && m.goal_rel_rad != null ? `${fmt(deg(m.goal_rel_rad))}°` : "—";
    $("#mCur").textContent = on ? cur(m) : "—";
    $("#mTemp").textContent = on && m.temp_c != null ? `${m.temp_c} °C` : "—";
    const [lo, hi] = m.limits_rad.map(r => deg(r - zero));
    mSlider.min = lo; mSlider.max = hi;
    if (!mDragging && on) mSlider.value = deg(m.torque && m.goal_rel_rad != null ? m.goal_rel_rad : m.angle_rel_rad);
    $("#mLo").textContent = `${Math.round(lo) || 0}°`; $("#mHi").textContent = `${Math.round(hi) || 0}°`;
    $("#mClamp").hidden = !m.clamped_recently;
    const canMove = on && m.torque && !m.hw_error;
    mSlider.disabled = !canMove; $$("[data-jog]").forEach(b => b.disabled = !canMove); $("#mGo").disabled = !canMove; $("#mTarget").disabled = !canMove;
    $("#mWhy").textContent = !on ? "No data from this motor: controls are unavailable." : m.hw_error ? "Hardware error: release cable tension, reboot the motor, then switch its torque on again."
      : !m.torque ? "Torque is off: switch it on to move this motor (it holds its current position)." : s.tip.busy ? "Tip control is moving the arm: motor commands are refused until you stop it." : "";
    $("#mReboot").hidden = !(on && m.hw_error); $("#mReboot").textContent = `Reboot motor ${m.id}…`;
    $("#mZero").disabled = !on;
    $("#mTele").innerHTML = [["Velocity", on ? `${fmt(m.velocity_rpm)} rpm` : "—"], ["Cable", on && m.cable_mm !== undefined ? `${fmt(m.cable_mm, 2)} mm` : "—"],
      ["Moving", on ? (m.moving ? "yes" : "no") : "—"], ["Calibrated zero", m.calibrated ? `${fmt(deg(zero), 2)}° abs` : "not set"]]
      .map(([k, v]) => `<div><span class="k">${k}</span><span class="v">${v}</span></div>`).join("");
    $("#mEng").innerHTML = [["Pulses", on ? m.pulse : "—"], ["Radians", on ? fmt(m.angle_rel_rad, 4) : "—"], ["Absolute angle", on ? `${fmt(deg(m.angle_rad), 2)}°` : "—"],
      ["Current, raw", on ? `${m.current_raw} u × ${m.ma_per_unit ?? "?"} mA/u` : "—"], ["HW error status", on ? `${m.hw_error} (0x${(m.hw_error || 0).toString(16)})` : "—"],
      ["Model", m.model || "—"], ["Operating mode", m.mode ?? "—"], ["Soft limits", `${m.settings.min_deg}° to ${m.settings.max_deg}° abs`], ["Read glitches", on ? m.read_glitches : "—"]]
      .map(([k, v]) => `<div><span class="k">${k}</span><span class="v" style="font-size:15px">${v}</span></div>`).join("");
  }

  // ---------------- setup panel ----------------
  $("#setup").innerHTML = `
    <section class="sec" aria-labelledby="h-conn"><h2 id="h-conn">Serial connection</h2><p class="lead" id="connLead"></p>
      <div class="row"><label class="field" for="port">Port</label><select id="port"><option value="">auto (first FTDI)</option></select>
        <button class="btn primary" id="connect">Connect</button><button class="btn" id="disconnect">Disconnect</button></div>
      <p class="note">Close Emio Labs / SOFA first: a running scene overrides the motors. Connecting never moves anything.</p></section>
    <section class="sec" id="sec-cal" aria-labelledby="h-cal"><h2 id="h-cal">Calibration</h2>
      <p class="lead">Set each motor's taut zero where its cable just goes taut. The zero is stored in software; the motor isn't changed.</p>
      <div id="calRows"></div><div class="row" style="margin-top:10px"><button class="btn" id="calAll">Calibrate all…</button></div></section>
    <section class="sec" aria-labelledby="h-ms"><h2 id="h-ms">Motor settings</h2><p class="lead">Drum radius, cable sign and soft limits (absolute degrees).</p>
      <div class="row" style="margin-bottom:10px"><span class="seg" role="group" aria-label="Motor" id="setSeg">${[0, 1, 2, 3].map(i => `<button data-sm="${i}" aria-pressed="false">${i}</button>`).join("")}</span></div>
      <div class="form"><label for="sRad">Drum radius (mm)</label><input type="number" id="sRad" step="0.1" min="0.1">
        <label for="sInv">Invert cable sign</label><input type="checkbox" id="sInv" style="width:22px;height:22px;accent-color:var(--ink)">
        <label for="sMin">Soft min (° absolute)</label><input type="number" id="sMin" step="1">
        <label for="sMax">Soft max (° absolute)</label><input type="number" id="sMax" step="1"></div>
      <div class="row" style="margin-top:10px"><button class="btn primary" id="sSave">Save</button><button class="btn" id="sClear">Clear calibration</button></div></section>
    <section class="sec" aria-labelledby="h-model"><h2 id="h-model">Tip model</h2><div id="modelInfo"></div></section>
    <section class="sec" aria-labelledby="h-obs"><h2 id="h-obs">Obstacle (rod)</h2><p class="lead" id="obsLead"></p>
      <div class="row"><label class="chk"><input type="checkbox" id="obsAvoid"> Plan moves around it</label>
        <label class="field">Clearance <input type="number" id="obsClr" min="0" max="100" step="1"> mm</label></div>
      <div class="row" style="margin:8px 0"><button class="btn" id="obsPick">Pick rod colour</button><button class="btn" id="obsMark">Mark rod ends</button><button class="btn" id="obsForget">Forget rod</button></div>
      <p class="note" id="obsStatus" aria-live="polite"></p>
      <canvas id="obsCam" aria-label="Camera image for picking the rod colour or marking its ends"></canvas>
      <p class="note">Clearance is from the arm's centreline to the rod's surface: allow for the disc radius plus the model error (about ±8 mm). It is a model estimate, not a guarantee.</p></section>
    <section class="sec" aria-labelledby="h-joy"><h2 id="h-joy">Joystick</h2>
      <div class="row"><label class="chk"><input type="checkbox" id="invX"> Invert gamepad x</label><label class="chk"><input type="checkbox" id="invY"> Invert gamepad y</label></div>
      <p class="note">Gamepad: left stick steers; hold <b>RT / R2</b> (or RB / R1) to move, let go to hold; <b>B / Circle</b> = torque off all. Press any button first so the browser sees it.
        Non-standard layouts: hold <b>RB</b>. On-screen stick: dragging is the hold-to-move; release and the arm holds. Bluetooth: pair once with <code>./pair_controller.sh</code>.</p></section>
    <section class="sec" aria-labelledby="h-3d"><h2 id="h-3d">3D view</h2>
      <div class="row" style="gap:4px 18px"><label class="chk"><input type="checkbox" data-o="cloud"> Camera point cloud</label>
        <label class="field" style="font-weight:500">within <input type="number" id="o3crop" min="0" step="25"> mm of the arm</label></div>
      <div class="row" style="gap:4px 18px"><label class="chk"><input type="checkbox" data-o="surface"> Tip surface</label><label class="chk"><input type="checkbox" data-o="zone"> Clearance zone</label>
        <label class="chk"><input type="checkbox" data-o="armCam"> Arm from camera</label><label class="chk"><input type="checkbox" data-o="path"> Planned path</label></div>
      <div class="row" style="margin-top:8px"><span class="note" style="margin-right:6px">Reference view:</span><button class="btn quiet" data-preset2="front">Front</button><button class="btn quiet" data-preset2="top">Top</button><button class="btn quiet" data-preset2="camera">Camera's view</button></div></section>
    <section class="sec" aria-labelledby="h-diag" style="grid-column:1/-1"><h2 id="h-diag">Diagnostics</h2>
      <div class="diag" id="diag" tabindex="0" aria-label="Diagnostic messages"></div>
      <details class="eng"><summary>Engineering notes and developer commands</summary><div class="note" style="max-width:none">
        <p>Angles follow emioapi: angle = (2048 − pulse) × 2π / 4096 (pulse 2048 = 0°) until calibrated; afterwards angle, goal, slider and jog are relative to the calibrated zero. "Absolute" is the emioapi angle. Cable displacement = (angle − zero) × drum radius. Soft limits stay in absolute angles.</p>
        <p>Camera feed: <code>~/emio-venv/bin/python tip_feed.py</code> (<code>--show</code>, <code>--rod-color</code>, <code>--no-rod</code>). Tip model: <code>collect_tip_data.py</code>, then <code>fit_tip_model.py data/tip_&lt;time&gt;.csv</code>. GUI: <code>cd motor_gui &amp;&amp; docker compose up -d --build</code>.</p>
        <p>Moves follow the pair rules (motors 0/2 and 1/3 oppose; one cable of each pair pulls; a cable is released before its opposite pulls), back off above 1000 mA and release everything above 1100 mA.</p></div></details></section>`;
  async function loadPorts() {
    const sel = $("#port"), cur0 = sel.value;
    const ports = await sim.api("/api/ports").catch(() => []);
    sel.innerHTML = `<option value="">auto (first FTDI)</option>` + ports.map(p => `<option value="${esc(p.device)}">${esc(p.device)}${p.emio ? " · EMIO" : ""}</option>`).join("");
    sel.value = cur0;
  }
  $("#port").addEventListener("focus", loadPorts);
  $("#connect").addEventListener("click", () => fire("/api/connect", { port: $("#port").value || null }));
  $("#disconnect").addEventListener("click", () => fire("/api/disconnect", {}));
  $("#calAll").addEventListener("click", () => confirmDlg("Calibrate all motors?", "Set the current position of every motor as 0?", "Set all to 0").then(ok => { if (ok) fire("/api/zero_all", {}); }));
  $("#calRows").addEventListener("click", e => { const b = e.target.closest("[data-zero]"); if (b) fire(`/api/motor/${b.dataset.zero}/zero`, {}); });
  $$("[data-sm]").forEach(b => b.addEventListener("click", () => { ui.setupMotor = +b.dataset.sm; ui.setupDirty = false; renderSetup(); }));
  ["#sRad", "#sInv", "#sMin", "#sMax"].forEach(id => $(id).addEventListener("input", () => ui.setupDirty = true));
  $("#sSave").addEventListener("click", () => api(`/api/motor/${ui.setupMotor}/settings`, { radius_mm: +$("#sRad").value, invert: $("#sInv").checked, min_deg: +$("#sMin").value, max_deg: +$("#sMax").value })
    .then(() => { ui.setupDirty = false; }).catch(() => {}));
  $("#sClear").addEventListener("click", () => fire(`/api/motor/${ui.setupMotor}/settings`, { clear_zero: true }));
  $("#obsAvoid").addEventListener("change", e => fire("/api/obstacle/settings", { avoid: e.target.checked }));
  $("#obsClr").addEventListener("change", e => fire("/api/obstacle/settings", { clearance_mm: +e.target.value }));
  const setCamMode = m => { ui.camMode = m; $("#obsCam").classList.toggle("picking", !!m); $("#obsPick").setAttribute("aria-pressed", m === "pick"); $("#obsMark").setAttribute("aria-pressed", !!(m && m.mark)); draw(); };
  $("#obsPick").addEventListener("click", () => setCamMode(ui.camMode === "pick" ? null : "pick"));
  $("#obsMark").addEventListener("click", () => setCamMode(ui.camMode && ui.camMode.mark ? null : { mark: [] }));
  $("#obsForget").addEventListener("click", () => fire("/api/obstacle/clear", {}));
  let camBox = null;
  $("#obsCam").addEventListener("click", e => {
    if (!ui.camMode || !camBox) return;
    const r = $("#obsCam").getBoundingClientRect(), u = (e.clientX - r.left - camBox.x) / camBox.w, v = (e.clientY - r.top - camBox.y) / camBox.h;
    if (u < 0 || u > 1 || v < 0 || v > 1) return;
    if (ui.camMode === "pick") { fire("/api/obstacle/pick", { u, v }); setCamMode(null); return; }
    ui.camMode.mark.push([u, v]);
    if (ui.camMode.mark.length === 2) { fire("/api/obstacle/mark", { points: ui.camMode.mark, radius_mm: 5 }); setCamMode(null); } else draw();
  });
  function drawSetupCam(s) { camBox = V.drawCamera($("#obsCam"), s, model, th, { off: camOff(s), marks: ui.camMode && ui.camMode.mark ? ui.camMode.mark : [] }); }
  $("#invX").addEventListener("change", e => ui.inv.x = e.target.checked);
  $("#invY").addEventListener("change", e => ui.inv.y = e.target.checked);
  $$("[data-o]").forEach(c => c.addEventListener("change", () => { ui.opts3d[c.dataset.o] = c.checked; draw(); }));
  $("#o3crop").addEventListener("change", e => ui.opts3d.crop = +e.target.value);
  $$("[data-preset2]").forEach(b => b.addEventListener("click", () => { Object.assign(ui.cam3d, V.CAMERA_PRESETS[b.dataset.preset2]); ui.view = "3d"; setTask("operate"); setView("3d"); }));

  function renderSetup() {
    const s = snap, wsDown = meta.ws === "closed", t = s.tip, o = t.obstacle;
    $("#connLead").textContent = wsDown ? "Server unreachable: no connection information." : s.connected ? `Connected to ${s.port}${s.mock ? " (simulated)" : ""}.` : `Disconnected.${s.error ? " " + s.error + "." : ""}`;
    $("#connect").disabled = wsDown; $("#disconnect").disabled = wsDown || !s.connected;
    if (!$("#calRows").children.length) $("#calRows").innerHTML = [0, 1, 2, 3].map(i => `<div class="calrow"><b>Motor ${i}</b><span class="num" id="cal${i}"></span>
      <button class="btn small" data-zero="${i}" aria-label="Motor ${i}: calibrate, set 0 here">Set 0 here</button></div>`).join("");
    s.motors.forEach(m => {
      $(`#cal${m.id}`).innerHTML = m.calibrated ? `zero at ${fmt(deg(m.settings.zero_rad), 2)}° abs` : `<span style="color:var(--caution-text);font-family:var(--sans);font-weight:700">not calibrated</span>`;
      $(`[data-zero="${m.id}"]`).disabled = !(known() && m.online);
    });
    $("#calAll").disabled = !known();
    $$("[data-sm]").forEach(b => b.setAttribute("aria-pressed", +b.dataset.sm === ui.setupMotor));
    if (!ui.setupDirty) {
      const ms = s.motors[ui.setupMotor].settings;
      $("#sRad").value = ms.radius_mm; $("#sInv").checked = ms.invert; $("#sMin").value = ms.min_deg; $("#sMax").value = ms.max_deg;
    }
    $("#modelInfo").innerHTML = model ? `<dl class="kv"><dt>Samples</dt><dd>${model.samples}</dd><dt>Median error</dt><dd>±${model.cv_error_mm.median.toFixed(1)} mm (max ${model.cv_error_mm.max.toFixed(0)} mm)</dd>
      <dt>Fitted</dt><dd>${esc(model.fitted)}</dd><dt>Arm length</dt><dd>${model.length_mm.toFixed(0)} mm</dd><dt>Shape</dt><dd>constant curvature (whole arm)</dd></dl>`
      : `<p class="lead" style="color:var(--caution-text);font-weight:700">No tip model.</p><p class="note">Tip mapping, Move tip and obstacle planning are unavailable. Collect data with <code>collect_tip_data.py</code>, then fit with <code>fit_tip_model.py</code>; the GUI picks it up automatically.</p>`;
    $("#obsLead").textContent = !o.rod ? "No rod known. tip_feed.py finds an orange rod by itself, or pick its colour, or mark its ends."
      : `Rod ${o.source === "marked" ? "from marked ends" : "detected by colour"}, radius ${o.rod.radius} mm${o.feed ? (o.seen ? ", seen by the camera now" : `, not seen now (last ${o.rod_age_s.toFixed(0)} s ago)`) : ", camera feed off"}.`;
    if (document.activeElement !== $("#obsAvoid")) $("#obsAvoid").checked = o.avoid;
    if (document.activeElement !== $("#obsClr")) $("#obsClr").value = o.clearance_mm;
    $("#obsPick").disabled = $("#obsMark").disabled = !o.feed || wsDown;
    $("#obsForget").disabled = !o.rod || wsDown;
    if (ui.camMode && !o.feed) setCamMode(null);
    const st = $("#obsStatus");
    st.textContent = ui.camMode === "pick" ? "Click the rod in the camera image below to use its colour."
      : ui.camMode ? `Click the rod's ${ui.camMode.mark.length ? "other" : "first"} end in the camera image below.`
      : o.picking ? "Picking the colour…" : o.marking ? "Measuring the marked ends…" : o.error ? `Failed: ${o.error}` : !o.feed ? "Pick and mark need the camera feed (tip_feed.py)." : "";
    st.style.color = o.error && !ui.camMode ? "var(--fault-text)" : "";
    $$("[data-o]").forEach(c => { if (document.activeElement !== c) c.checked = ui.opts3d[c.dataset.o]; });
    if (document.activeElement !== $("#o3crop")) $("#o3crop").value = ui.opts3d.crop;
    const lines = [`connection: ${wsDown ? "server unreachable" : s.connected ? s.port : "disconnected"}${s.error ? ` (${s.error})` : ""}`,
      `tip status: ${t.status.state}${t.status.message ? ` — ${t.status.message}` : ""}`, `joystick: ${t.joy.active ? "active" : "off"}${t.joy.message ? ` — ${t.joy.message}` : ""}`,
      `camera feed: ${t.feed ? `age ${t.feed.age_s.toFixed(2)} s, markers ${t.feed.markers}` : "off"}`, ...alerts.map(a => `${a.level.toUpperCase()} ${a.source}: ${a.title} [${a.raw}]`)];
    $("#diag").textContent = lines.join("\n");
  }

  // ---------------- main render (10 Hz from the simulated WebSocket) ----------------
  function render(s, m) {
    if (!s) return;
    snap = s; meta = m || { ws: "open" };
    model = s.tip.model ? sim.model() : null;
    if (!th) th = V.theme(document.documentElement);
    alerts = EmioSim.deriveAlerts(s, meta);
    renderBar(); renderAlerts();
    if (ui.task === "operate") { renderOperateText(); renderMove(); renderJoy(); renderParams(); }
    if (ui.task === "motors") renderMotors();
    if (ui.task === "setup") renderSetup();
    draw();
  }
  setTheme(document.documentElement.dataset.theme || "day");
  setView("camera");
  sim.subscribe((s, m) => render(s, m));
  new ResizeObserver(draw).observe($("#vstage"));
})();
