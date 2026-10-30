// Simulated EMIO controller for the design prototypes. NOTHING here talks to a network or to hardware.
// It mirrors the real backend (motor_gui/app/server.py, tipctl.py): the same snapshot shape as GET /api/state /
// the WebSocket, the same command endpoints with the same refusals, plus demo scenarios and injectable
// conditions. Simplifications are marked "sim:". All prototypes share it, so they differ only in design.
(function () {
  "use strict";
  const IDS = [0, 1, 2, 3];
  const MA_PER_UNIT = 2.69;                         // XM430-W210 (motors.MODELS)
  const LAYOUT_DEG = [45, 135, 225, 315];           // screen direction of motors 0..3 (index.html)
  const HW_ERRORS = { 1: "input voltage", 4: "overheating", 8: "encoder", 16: "electrical shock", 32: "overload" };
  const hwText = e => Object.entries(HW_ERRORS).filter(([b]) => e & b).map(([, t]) => t).join(", ") || `code ${e}`;
  const TAU = 2 * Math.PI, deg = r => r * 180 / Math.PI, rad = d => d * Math.PI / 180;
  const toAngle = p => (2048 - p) * TAU / 4096, toPulse = a => 2048 - a * 4096 / TAU;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const now = () => performance.now() / 1000;

  // ---- tip model (constant-curvature arm; numbers from the robot's fitted model, 2026-10-26) ----
  const MODEL = {
    base: [1.4, -33.5, -3.0], length_mm: 196.0, samples: 62, n: 62, fitted: "2026-10-26 15:22",
    cv_error_mm: { median: 7.8, max: 17.0 }, motor_dirs_deg: [87.2, -4.6, -90.3, 179.8],
  };
  const GAIN = 2.0, MAX_BEND = rad(120);           // sim: bend per unit pull; tip-mode limit 120°
  const PHI = MODEL.motor_dirs_deg.map(rad);
  function bendVec(A, B) {
    const pulls = [Math.max(A, 0), Math.max(B, 0), Math.max(-A, 0), Math.max(-B, 0)];
    let bx = 0, bz = 0;
    pulls.forEach((p, i) => { bx += GAIN * p * Math.cos(PHI[i]); bz += GAIN * p * Math.sin(PHI[i]); });
    return [bx, bz];
  }
  const bendRad = (A, B) => Math.hypot(...bendVec(A, B));
  function backbone(A, B, n = 16) {               // attachment + n points to the tip
    const [bx, bz] = bendVec(A, B), th = Math.hypot(bx, bz), L = MODEL.length_mm, b = MODEL.base;
    const ux = th > 1e-6 ? bx / th : 1, uz = th > 1e-6 ? bz / th : 0, pts = [b.slice()];
    for (let k = 1; k <= n; k++) {
      const s = k / n, a = th * s;
      const h = a < 1e-6 ? L * s * a / 2 : L * s * (1 - Math.cos(a)) / a, v = a < 1e-6 ? L * s : L * s * Math.sin(a) / a;
      pts.push([b[0] + h * ux, b[1] - v, b[2] + h * uz]);
    }
    return pts;
  }
  const predict = (A, B) => backbone(A, B, 1)[1];
  MODEL.rest_tip = predict(0, 0);
  function arcThroughTip(tip, n = 16) {            // the arm's shape from the measured tip alone (tipctl)
    const b = MODEL.base, hx = tip[0] - b[0], hz = tip[2] - b[2], h = Math.hypot(hx, hz), v = b[1] - tip[1], pts = [b.slice()];
    for (let k = 1; k <= n; k++) {
      const s = k / n;
      if (h < 1e-3) { pts.push([b[0], b[1] - v * s, b[2]]); continue; }
      const th = 2 * Math.atan2(h, v), R = (h * h + v * v) / (2 * h);
      pts.push([b[0] + R * (1 - Math.cos(th * s)) * hx / h, b[1] - R * Math.sin(th * s), b[2] + R * (1 - Math.cos(th * s)) * hz / h]);
    }
    return pts;
  }
  // grid over the bend square (41 x 41), tip positions, reachable outline
  const G = 41, grid = [];
  for (let i = 0; i < G; i++) for (let j = 0; j < G; j++) {
    const A = -1 + 2 * i / (G - 1), B = -1 + 2 * j / (G - 1), tip = predict(A, B);
    grid.push({ A, B, tip, inside: bendRad(A, B) <= MAX_BEND + 1e-9 });
  }
  function clipD(A, B) {                           // nearest bend values along the same ray inside the workspace
    A = clamp(A, -1, 1); B = clamp(B, -1, 1);
    if (bendRad(A, B) <= MAX_BEND) return [A, B];
    let lo = 0, hi = 1;
    for (let k = 0; k < 30; k++) { const m = (lo + hi) / 2; bendRad(A * m, B * m) <= MAX_BEND ? lo = m : hi = m; }
    return [A * lo, B * lo];
  }
  MODEL.outline = [];
  for (let a = 0; a < 360; a += 3) {
    const c = Math.cos(rad(a)), s = Math.sin(rad(a)), k = Math.max(Math.abs(c), Math.abs(s));
    const [A, B] = clipD(c / k, s / k), t = predict(A, B);
    MODEL.outline.push([+t[0].toFixed(1), +t[2].toFixed(1)]);
  }
  function inverse(x, z) {                         // nearest grid point, refined
    let best = null, bd = Infinity;
    for (const g of grid) if (g.inside) { const d = Math.hypot(g.tip[0] - x, g.tip[2] - z); if (d < bd) { bd = d; best = g; } }
    let A = best.A, B = best.B;
    for (let it = 0; it < 12; it++) {
      const p = predict(A, B), e = [x - p[0], z - p[2]], h = 0.01;
      const pa = predict(A + h, B), pb = predict(A, B + h);
      const J = [[(pa[0] - p[0]) / h, (pb[0] - p[0]) / h], [(pa[2] - p[2]) / h, (pb[2] - p[2]) / h]];
      const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
      if (Math.abs(det) < 1e-6) break;
      const dA = (J[1][1] * e[0] - J[0][1] * e[1]) / det, dB = (-J[1][0] * e[0] + J[0][0] * e[1]) / det;
      [A, B] = clipD(A + clamp(dA, -0.05, 0.05), B + clamp(dB, -0.05, 0.05));
    }
    const p = predict(A, B);
    return { A, B, pred: [p[0], p[2]], dist: Math.hypot(p[0] - x, p[2] - z) };
  }

  // ---- obstacle: an orange rod at mid-height (sim: the measured prototype rod moved 50 mm away from the arm) ----
  const ROD = { p0: [-31, -125, -95], p1: [55, -130, -7], radius: 6.0 };
  function segDist(P, rod) {
    const a = rod.p0, b = rod.p1, ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ap = [P[0] - a[0], P[1] - a[1], P[2] - a[2]];
    const t = clamp((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / (ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2), 0, 1);
    return Math.hypot(ap[0] - t * ab[0], ap[1] - t * ab[1], ap[2] - t * ab[2]) - rod.radius;
  }
  const armClearance = (pts, rod) => Math.min(...pts.slice(1).map(p => segDist(p, rod)));
  const clearanceAt = (A, B, rod) => armClearance(backbone(A, B), rod);
  grid.forEach(g => g.clear = clearanceAt(g.A, g.B, ROD));

  // ---- scenarios ----
  const SCENARIOS = {
    idle: "Connected, idle",
    joystick: "Joystick moving",
    overload: "Motor 2 overload",
    camera: "Camera stale / tip lost",
    clearance: "Obstacle clearance",
    disconnected: "Disconnected",
  };
  const INJECT = {   // injectable conditions (the Simulation harness toggles these)
    server_down: "Server unreachable",
    no_model: "No tip model",
    feed_off: "Camera feed off",
    feed_stale: "Camera data stale",
    marker_lost: "Tip marker not detected",
    rod_unseen: "Rod not seen by camera",
    pick_error: "Rod colour pick failed",
    force_limit: "Backend force limit (tip move)",
    joy_wall: "Joystick force-limit wall",
    move_failed: "Move failed (hard current limit)",
  };
  const MOTOR_INJECT = { hw: "HW error", offline: "Not responding", overridden: "Overridden", uncal: "Uncalibrated" };

  function makeMotor(i) {
    const zero = [0.30, 0.25, 0.35, 0.28][i];
    return {
      id: i, pulse: toPulse(zero), goal: toPulse(zero), torque: true, hw_error: 0, online: true, temp_c: [33, 34, 32, 35][i],
      overridden: false, current_raw: 0, velocity_raw: 0, moving: false, glitches: 0,
      settings: { radius_mm: 20.0, zero_rad: zero, invert: false, min_deg: 0.0, max_deg: [163, 152.6, 122.4, 129.4][i] },
    };
  }

  function createSim(opts = {}) {
    const S = {
      connected: true, port: "/dev/ttyUSB0", error: null, profile_velocity: 30,
      motors: IDS.map(makeMotor), model: true,
      status: { state: "idle", target: null, iteration: 0, error_mm: null, message: "" },
      joy: { active: false, enable: false, message: "", walls: [], cmd: null, mode: "tip", control: "rate", stick: null, tgt: null },
      obs: { avoid: true, clearance_mm: 20, hsv: null, rod: { ...ROD, open: [false, true], visible_mm: 88 }, rod_t: 0, source: "colour",
             version: 1, picking: false, marking: false, error: "", seen: true },
      feed: { on: true, t: 0, markers: 1, bias: [3.1, 1.2, -2.4] }, cameraOn: true,
      plan: null, planId: 0, move: null, inject: {}, motorInject: {}, serverUp: true, serverDownAt: 0,
      clampAt: {}, log: [], scenario: "idle", padScript: false, lastUiStick: -10,
    };
    const listeners = new Set(), logListeners = new Set();
    let lastSnap = null;

    const zeroOf = m => m.settings.zero_rad ?? 0;
    const maxPull = m => rad(m.settings.max_deg) - zeroOf(m);
    const limits = m => [rad(m.settings.min_deg), rad(m.settings.max_deg)];
    function pose() {                              // bend values from motor angles (tipctl.pose)
      const d = {};
      for (const [p, [pos, neg]] of [["A", [0, 2]], ["B", [1, 3]]]) {
        const rp = toAngle(S.motors[pos].pulse) - zeroOf(S.motors[pos]), rn = toAngle(S.motors[neg].pulse) - zeroOf(S.motors[neg]);
        d[p] = rp > rad(0.5) ? Math.min(1, rp / maxPull(S.motors[pos])) : rn > rad(0.5) ? -Math.min(1, rn / maxPull(S.motors[neg])) : 0;
      }
      return d;
    }
    function goalsFor(A, B) {                      // targets_abs: the pulling motor winds in, the other rests at absolute 0
      const out = {};
      for (const [v, pos, neg] of [[A, 0, 2], [B, 1, 3]]) {
        out[pos] = v > 0 ? zeroOf(S.motors[pos]) + v * maxPull(S.motors[pos]) : 0;
        out[neg] = v < 0 ? zeroOf(S.motors[neg]) - v * maxPull(S.motors[neg]) : 0;
      }
      return out;
    }
    function setPose(A, B, settle = true) {
      const g = goalsFor(A, B);
      S.motors.forEach(m => { m.goal = toPulse(g[m.id]); if (settle) m.pulse = m.goal; });
    }
    const knownPose = () => S.connected && S.motors.every(m => m.online && m.settings.zero_rad != null) ? pose() : null;

    // ---- scenarios ----
    function reset() {
      Object.assign(S, { connected: true, port: "/dev/ttyUSB0", error: null, model: true, inject: {}, motorInject: {},
        serverUp: true, plan: null, move: null, padScript: false });
      S.motors = IDS.map(makeMotor);
      S.status = { state: "idle", target: null, iteration: 0, error_mm: null, message: "" };
      S.joy = { active: false, enable: false, message: "", walls: [], cmd: null, mode: "tip", control: "rate", stick: null, tgt: null };
      S.obs = { ...S.obs, avoid: true, clearance_mm: 20, rod: { ...ROD, open: [false, true], visible_mm: 88 }, source: "colour",
                picking: false, marking: false, error: "", seen: true, rod_t: now() };
      S.feed = { on: true, t: now(), markers: 1, bias: [3.1, 1.2, -2.4] };
    }
    function setScenario(key) {
      reset();
      S.scenario = key;
      if (key === "idle") {
        setPose(0.42, -0.18);
        const p = predict(0.42, -0.18);
        S.status = { state: "done", target: [+p[0].toFixed(0), +p[2].toFixed(0)], iteration: 2, error_mm: 2.4, message: "reached within 3 mm" };
      } else if (key === "joystick") {
        setPose(0.2, 0.1);
        S.joy = { ...S.joy, active: true, enable: true, mode: "tip", control: "rate", message: "moving · cables taut" };
        S.padScript = true;
      } else if (key === "overload") {
        setPose(0, 0);
        const m = S.motors[2];
        Object.assign(m, { hw_error: 32, torque: false, temp_c: 51 });
        S.status = { state: "failed", target: [-62, -48], iteration: 0, error_mm: null, message: "motor 2 hardware error 32; released motors to rest" };
      } else if (key === "camera") {
        setPose(0.35, 0.3);
        S.inject.feed_stale = true;
        S.feed.t = now() - 4.2;
        S.status = { state: "done", target: [-40, 30], iteration: 0, error_mm: null, message: "camera feed not running, open loop (model only, typically ±8 mm)" };
      } else if (key === "clearance") {
        setPose(-0.25, 0.4);   // moved there with the motor controls (they don't plan around the rod): ~9 mm from it
        S.status = { state: "idle", target: null, iteration: 0, error_mm: null, message: "" };
      } else if (key === "disconnected") {
        setPose(0, 0);
        S.connected = false; S.port = null; S.error = "Lost connection: No motor answered";
        S.feed.on = false; S.cameraOn = false;
      }
      S.cameraOn = key !== "disconnected";
      emit();
      log("scenario", { scenario: key });
    }

    // ---- injections ----
    function setInject(key, on) {
      S.inject[key] = on;
      if (key === "server_down") { S.serverUp = !on; S.serverDownAt = now(); }
      if (key === "feed_off") S.feed.on = !on;
      if (key === "feed_stale" && on) S.feed.t = now() - 3.5;
      if (key === "feed_stale" && !on) S.feed.t = now();
      if (key === "marker_lost") S.feed.markers = on ? 0 : 1;
      if (key === "rod_unseen") S.obs.seen = !on;
      if (key === "no_model") S.model = !on;
      if (key === "pick_error") S.obs.error = on ? "that spot isn't a bright colour (saturation 31/255); click the coloured rod" : "";
      if (key === "force_limit" && on) S.status = { state: "limited", target: S.status.target || [40, -20], iteration: 0, error_mm: null,
        message: "force limit (motor 1 at 1043 mA); stopped short" };
      if (key === "move_failed" && on) S.status = { state: "failed", target: S.status.target || [40, -20], iteration: 0, error_mm: null,
        message: "motor 1 current 1127 mA > 1100 mA; released motors to rest" };
      if ((key === "force_limit" || key === "move_failed") && !on) S.status = { state: "idle", target: null, iteration: 0, error_mm: null, message: "" };
      if (key === "joy_wall") S.joy.walls = on ? [{ motor: 1, t: now() }] : [];
      emit();
    }
    function setMotorInject(i, key, value) {       // value: true/false, or an hw_error code
      const m = S.motors[i];
      S.motorInject[`${i}:${key}`] = value;
      if (key === "hw") { m.hw_error = value ? +value : 0; if (value) m.torque = false; }
      if (key === "offline") m.online = !value;
      if (key === "overridden") m.overridden = !!value;
      if (key === "uncal") m.settings.zero_rad = value ? null : [0.30, 0.25, 0.35, 0.28][i];
      emit();
    }

    // ---- simulated motion ----
    let tLast = now(), padPhase = 0;
    function tick() {
      const t = now(), dt = Math.min(0.1, t - tLast); tLast = t;
      if (S.connected) {
        // tip move along its planned path
        if (S.move) {
          const mv = S.move;
          mv.s = Math.min(mv.path.length - 1, mv.s + dt * 0.35 * S.profile_velocity / 30 * 20);
          const k = Math.floor(mv.s), f = mv.s - k, a = mv.path[k], b = mv.path[Math.min(k + 1, mv.path.length - 1)];
          const g = goalsFor(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f);
          S.motors.forEach(m => m.goal = toPulse(g[m.id]));
          if (mv.s >= mv.path.length - 1 && S.motors.every(m => Math.abs(m.goal - m.pulse) < 2)) finishMove();
        }
        // joystick
        if (S.joy.active) joyStep(t, dt);
        // motors follow their goals at the profile velocity
        const maxStep = S.profile_velocity * 0.229 * 360 / 60 * dt * 4096 / 360;
        S.motors.forEach(m => {
          if (!m.online) return;
          if (m.overridden && m.torque && Math.random() < 0.02) m.goal = toPulse(rad(40 + Math.random() * 30));
          const e = m.torque && !m.hw_error ? m.goal - m.pulse : 0, step = clamp(e, -maxStep, maxStep);
          m.pulse += step;
          m.moving = Math.abs(step) > 0.3;
          m.velocity_raw = Math.round(step / dt * 60 / 4096 / 0.229);
          const pull = Math.max(0, toAngle(m.pulse) - zeroOf(m)) / Math.max(0.1, maxPull(m));
          m.current_raw = m.torque && !m.hw_error ? Math.round(14 + pull * 240 + (Math.random() - 0.5) * 6) : 0;
        });
      }
      if (S.feed.on && !S.inject.feed_stale && S.serverUp) S.feed.t = t;
      if (S.inject.joy_wall && S.joy.walls.length) S.joy.walls[0].t = Math.max(S.joy.walls[0].t, t - 1.9);
      emit();
    }
    function joyStep(t, dt) {
      const j = S.joy;
      if (S.padScript && t - S.lastUiStick > 1.0) {   // sim: a simulated gamepad slowly circles with the trigger held
        padPhase += dt * 0.6;
        j.stick = { x: 0.7 * Math.cos(padPhase), y: 0.7 * Math.sin(padPhase), enable: true, t, pad: true };
      }
      const st = j.stick, fresh = st && t - st.t < 0.3, live = fresh && st.enable;
      j.enable = !!live;
      let avoided = "";
      if (live && !j.walls.length) {
        const d = pose();
        let A = d.A, B = d.B;
        if (j.mode === "bend") { A += st.x * 0.5 * dt; B += st.y * 0.5 * dt; }
        else {
          const p = predict(A, B), r = inverse(p[0] + st.x * 70 * dt, p[2] + st.y * 70 * dt);
          A = r.A; B = r.B;
        }
        [A, B] = j.mode === "bend" ? [clamp(A, -1, 1), clamp(B, -1, 1)] : clipD(A, B);
        if (S.obs.avoid && S.obs.rod && S.model && clearanceAt(A, B, S.obs.rod) < S.obs.clearance_mm
            && clearanceAt(A, B, S.obs.rod) < clearanceAt(d.A, d.B, S.obs.rod)) { avoided = "blocked"; }
        else { const g = goalsFor(A, B); S.motors.forEach(m => m.goal = toPulse(g[m.id])); }
        j.cmd = { A, B };
        j.tgt = predict(A, B);
      }
      const taut = j.mode === "bend";
      j.message = (live ? "moving" : fresh ? "holding (trigger released)" : "holding (no stick input)") + (taut ? " · cables taut" : "")
        + (avoided === "blocked" ? " · stopped at the obstacle" : "");
      if (j.walls.length) {
        const w = j.walls[0];
        j.message = `force limit on motor ${w.motor}: ease off, or keep pushing to retry in ${Math.max(0, 2 - (t - w.t)).toFixed(1)} s`;
      }
    }
    function finishMove() {
      const mv = S.move; S.move = null;
      const st = S.status, feedOk = S.feed.on && !S.inject.feed_stale;
      if (!mv.closed || !feedOk) {
        st.state = "done";
        st.error_mm = feedOk && S.feed.markers ? 5.8 : null;
        st.message = mv.note + `${mv.closed ? "camera feed not running, " : ""}open loop (model only, typically ±${MODEL.cv_error_mm.median.toFixed(0)} mm)`;
      } else if (!S.feed.markers) {
        st.state = "done"; st.message = mv.note + "tip not visible to the camera; stopped (open loop result)";
      } else {
        st.state = "done"; st.iteration = 2; st.error_mm = 2.1; st.message = mv.note + "reached within 3 mm";
      }
    }

    // ---- planning (sim: straight path, else over a corner; same messages as tipctl.Planner) ----
    function plan(d0, d1) {
      const rod = S.obs.rod, need = S.obs.clearance_mm;
      if (!S.obs.avoid || !rod) return { path: [d1], used: d1, note: "" };
      const c = (A, B) => clearanceAt(A, B, rod), c0 = c(...d0), floor = Math.min(need, c0 - 1);
      const segMin = (a, b) => { let m = Infinity; for (let k = 0; k <= 24; k++) { const t = k / 24; m = Math.min(m, c(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)); } return m; };
      let note = "";
      if (c(...d1) < need) {
        let best = null, bd = Infinity;
        const tgt = predict(...d1);
        for (const g of grid) if (g.clear >= need) { const dd = Math.hypot(g.tip[0] - tgt[0], g.tip[2] - tgt[2]); if (dd < bd) { bd = dd; best = g; } }
        if (!best) throw new Error(`the arm can't get ${need.toFixed(0)} mm clear of the obstacle anywhere`);
        d1 = [best.A, best.B]; note = "target too close to the obstacle; going to the nearest clear point. ";
      }
      if (segMin(d0, d1) >= floor) return { path: [d1], used: d1, note };
      let best = null;
      for (const w of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
        const m = Math.min(segMin(d0, w), segMin(w, d1));
        if (!best || m > best.m) best = { w, m };
      }
      if (best.m < floor) throw new Error(`no path keeps the arm ${need.toFixed(0)} mm clear of the obstacle; the widest way passes ${Math.max(0, best.m).toFixed(0)} mm from it (lower the clearance to below that to allow it)`);
      return { path: [best.w, d1], used: d1, note: note + "going around the obstacle (1 waypoints). " };
    }
    function densify(d0, path, k = 6) {
      const pts = [d0, ...path], out = [d0];
      for (let i = 1; i < pts.length; i++) for (let s = 1; s <= k; s++) {
        const t = s / k; out.push([pts[i - 1][0] + (pts[i][0] - pts[i - 1][0]) * t, pts[i - 1][1] + (pts[i][1] - pts[i - 1][1]) * t]);
      }
      return out;
    }
    function planShapes(ds, preview, target) {
      const idx = [...new Set(Array.from({ length: 9 }, (_, k) => Math.round(k * (ds.length - 1) / 8)))];
      const out = { tips: ds.map(d => predict(...d)), arms: idx.map(i => backbone(...ds[i])), preview, target, ds };
      if (S.obs.rod) out.clear_min_mm = Math.min(...ds.map(d => clearanceAt(d[0], d[1], S.obs.rod)));
      return out;
    }

    // ---- API (same paths and refusals as server.py) ----
    function err(status, detail) { const e = new Error(detail); e.status = status; e.detail = detail; return e; }
    function needBus() { if (!S.connected) throw err(409, "Not connected"); }
    function checkReady(needModel = true) {
      if (!S.connected) throw err(409, "Not connected");
      if (needModel && !S.model) throw err(409, "No tip model at /data/tip_model.json; run fit_tip_model.py first");
      if (S.motors.some(m => m.settings.zero_rad == null)) throw err(409, "Calibrate every motor's taut zero first");
      if (S.motors.some(m => !m.online)) throw err(409, "Not all motors are responding");
      if (S.motors.some(m => m.hw_error)) throw err(409, "A motor has a hardware error; reboot it first");
      if (S.move || S.joy.active) throw err(409, "Already moving; press Stop first");
    }
    function stopTip() {
      if (S.move) { S.move = null; S.status.state = "stopped"; S.status.message = "stopped"; S.motors.forEach(m => m.goal = m.pulse); }
      if (S.joy.active) { S.joy.active = false; S.joy.enable = false; S.joy.message = "stopped"; S.padScript = false; S.motors.forEach(m => m.goal = m.pulse); }
    }
    const routes = [
      ["GET", /^\/api\/ports$/, () => [{ device: "/dev/ttyUSB0", emio: true }, { device: "/dev/ttyS0", emio: false }]],
      ["POST", /^\/api\/connect$/, b => {
        if (S.inject.server_down) throw err(0, "server unreachable");
        Object.assign(S, { connected: true, port: b.port || "/dev/ttyUSB0", error: null });
        S.motors.forEach(m => { if (!S.motorInject[`${m.id}:offline`]) m.online = true; });
        if (!S.inject.feed_off) { S.feed.on = true; S.cameraOn = true; }
        return { ok: true };
      }],
      ["POST", /^\/api\/disconnect$/, () => { stopTip(); Object.assign(S, { connected: false, port: null, error: null }); return { ok: true }; }],
      ["POST", /^\/api\/motor\/(\d)\/torque$/, (b, [, i]) => {
        needBus(); const m = S.motors[+i];
        if (b.on && m.hw_error) throw err(409, `Motor ${i} has hardware error ${m.hw_error}; reboot it first`);
        if (!m.online) throw err(500, `Motor ${i} did not answer`);
        m.torque = !!b.on; if (b.on) m.goal = m.pulse;
        return { ok: true };
      }],
      ["POST", /^\/api\/motor\/(\d)\/reboot$/, (b, [, i]) => {
        needBus(); const m = S.motors[+i];
        Object.assign(m, { hw_error: 0, torque: false, temp_c: Math.min(m.temp_c, 40) });
        S.motorInject[`${i}:hw`] = 0;
        return { ok: true };
      }],
      ["POST", /^\/api\/motor\/(\d)\/goal$/, (b, [, i]) => {
        needBus();
        if (S.move || S.joy.active) throw err(409, "Tip control is moving the arm; press Stop first");
        const m = S.motors[+i];
        if (!m.online) throw err(404, "Motor not available");
        if (!m.torque) throw err(409, "Enable torque on this motor first");
        const target = b.angle_rad + (b.relative ? zeroOf(m) : 0), [lo, hi] = limits(m), a = clamp(target, lo, hi);
        m.goal = toPulse(a);
        const clamped = Math.abs(a - target) > 1e-9;
        if (clamped) S.clampAt[i] = now();
        return { angle_rad: a, angle_rel_rad: a - zeroOf(m), clamped };
      }],
      ["POST", /^\/api\/hold$/, () => { needBus(); stopTip(); S.motors.forEach(m => { if (m.torque) m.goal = m.pulse; }); return { ok: true, errors: [] }; }],
      ["POST", /^\/api\/torque_off_all$/, () => { stopTip(); needBus(); S.motors.forEach(m => m.torque = false); return { ok: true, errors: [] }; }],
      ["POST", /^\/api\/motor\/(\d)\/zero$/, (b, [, i]) => { needBus(); const m = S.motors[+i]; if (!m.online) throw err(404, `Motor ${i} not available`); m.settings.zero_rad = toAngle(m.pulse); return { ok: true }; }],
      ["POST", /^\/api\/zero_all$/, () => { needBus(); S.motors.forEach(m => { if (m.online) m.settings.zero_rad = toAngle(m.pulse); }); return { ok: true }; }],
      ["POST", /^\/api\/motor\/(\d)\/settings$/, (b, [, i]) => {
        const s = S.motors[+i].settings, lo = b.min_deg ?? s.min_deg, hi = b.max_deg ?? s.max_deg;
        if (lo >= hi) throw err(422, "Min must be below max");
        for (const k of ["radius_mm", "invert", "min_deg", "max_deg"]) if (b[k] !== undefined && b[k] !== null) s[k] = b[k];
        if (b.clear_zero) s.zero_rad = null;
        return { ...s };
      }],
      ["POST", /^\/api\/speed$/, b => { if (!(b.profile_velocity >= 1 && b.profile_velocity <= 200)) throw err(422, "profile_velocity must be 1–200"); S.profile_velocity = b.profile_velocity; return { profile_velocity: b.profile_velocity }; }],
      ["GET", /^\/api\/tip\/model$/, () => { if (!S.model) throw err(404, "No tip model; run fit_tip_model.py"); return model(); }],
      ["POST", /^\/api\/tip\/goto$/, b => {
        checkReady();
        const r = inverse(b.x, b.z), cur = pose();
        let note = r.dist > 15 ? `target is ${r.dist.toFixed(0)} mm outside the reachable area; going to the nearest reachable point. ` : "";
        let p;
        try { p = plan([cur.A, cur.B], [r.A, r.B]); }
        catch (e) { S.status = { state: "failed", target: [b.x, b.z], iteration: 0, error_mm: null, message: e.message }; return S.status; }
        note += p.note;
        const ds = densify([cur.A, cur.B], p.path);
        S.plan = planShapes(ds, false, [b.x, b.z]); S.planId++;
        const tgt = predict(...p.used);
        S.status = { state: "moving", target: [tgt[0], tgt[2]], iteration: 0, error_mm: null, message: p.path.length > 1 ? "planning" : "moving" };
        S.move = { path: ds, s: 0, closed: !!b.closed_loop, note };
        return S.status;
      }],
      ["POST", /^\/api\/tip\/preview$/, b => {
        if (!S.model) throw err(409, "No tip model at /data/tip_model.json; run fit_tip_model.py first");
        const r = inverse(b.x, b.z), cur = knownPose(), d0 = cur ? [cur.A, cur.B] : [0, 0];
        let note = cur ? "" : "from rest (motors not connected). ";
        if (r.dist > 15) note += `target is ${r.dist.toFixed(0)} mm outside the reachable area; nearest reachable point. `;
        let p;
        try { p = plan(d0, [r.A, r.B]); } catch (e) { S.plan = null; S.planId++; throw err(409, e.message); }
        S.plan = { ...planShapes(densify(d0, p.path), true, [b.x, b.z]), note: (note + p.note).trim() }; S.planId++;
        return S.plan;
      }],
      ["POST", /^\/api\/tip\/stop$/, () => { stopTip(); return { ok: true }; }],
      ["POST", /^\/api\/tip\/joy\/start$/, () => {
        checkReady(false);
        S.joy = { ...S.joy, active: true, enable: false, walls: S.inject.joy_wall ? [{ motor: 1, t: now() }] : [], message: "hold the trigger (RT / R2) to move" };
        return { ok: true };
      }],
      ["POST", /^\/api\/tip\/joy$/, b => {
        if (!S.joy.active) throw err(409, "Joystick control is not active; press Start first");
        if (b.mode === "tip" && !S.model) throw err(409, "The top-view mapping needs a tip model; choose the motor-pair mapping");
        S.joy.mode = b.mode; S.joy.control = b.control || "rate";
        S.joy.stick = { x: +b.x, y: +b.y, enable: !!b.enable, t: now() };
        S.lastUiStick = now();
        return { ok: true };
      }],
      ["POST", /^\/api\/obstacle\/settings$/, b => {
        if (b.avoid !== undefined && b.avoid !== null) S.obs.avoid = !!b.avoid;
        if (b.clearance_mm !== undefined && b.clearance_mm !== null) {
          if (!(b.clearance_mm >= 0 && b.clearance_mm <= 100)) throw err(422, "clearance_mm must be 0–100");
          S.obs.clearance_mm = +b.clearance_mm; S.obs.version++;
        }
        return { ok: true };
      }],
      ["POST", /^\/api\/obstacle\/pick$/, b => {
        S.obs.picking = true;
        setTimeout(() => { S.obs.picking = false; if (S.inject.pick_error) S.obs.error = INJECT_MSG.pick; else { S.obs.hsv = [[[8, 120, 90], [22, 255, 255]]]; S.obs.error = ""; S.obs.source = "colour"; } }, 700);
        return { ok: true };
      }],
      ["POST", /^\/api\/obstacle\/mark$/, b => {
        S.obs.marking = true;
        setTimeout(() => { S.obs.marking = false; S.obs.rod = { ...ROD, radius: b.radius_mm ?? 5 }; S.obs.source = "marked"; S.obs.version++; S.obs.rod_t = now(); }, 800);
        return { ok: true };
      }],
      ["POST", /^\/api\/obstacle\/clear$/, () => { S.obs.rod = null; S.obs.source = null; S.obs.version++; return { ok: true }; }],
      ["GET", /^\/api\/obstacle\/map$/, () => obstacleMap()],
    ];
    const INJECT_MSG = { pick: "that spot isn't a bright colour (saturation 31/255); click the coloured rod" };
    function api(path, body) {
      const method = body === undefined ? "GET" : "POST";
      return new Promise((resolve, reject) => setTimeout(() => {
        try {
          if (!S.serverUp) throw err(0, "Server unreachable");
          const r = routes.find(([m, re]) => m === method && re.test(path));
          if (!r) throw err(404, `Not found: ${method} ${path}`);
          const out = r[2](body || {}, path.match(r[1]));
          log(path, body, out); resolve(out); emit();
        } catch (e) { log(path, body, { error: e.detail || e.message, status: e.status }); reject(e); }
      }, 60));
    }
    function log(path, body, result) {
      const entry = { t: new Date(), path, body, result };
      S.log.push(entry); if (S.log.length > 200) S.log.shift();
      logListeners.forEach(f => f(entry));
    }

    // ---- snapshot (same shape as the backend's full_snapshot) ----
    function model() {
      return { base: MODEL.base, length_mm: MODEL.length_mm, rest_tip: MODEL.rest_tip, samples: MODEL.samples, n: MODEL.n,
               cv_error_mm: MODEL.cv_error_mm, fitted: MODEL.fitted, outline: MODEL.outline, motor_dirs_deg: MODEL.motor_dirs_deg };
    }
    let mapCache = null;
    function obstacleMap() {
      const key = `${S.obs.version}-${S.obs.clearance_mm}-${!!S.obs.rod}-${S.obs.avoid}`;
      if (mapCache && mapCache.key === key) return mapCache.map;
      const out = { version: S.obs.version, rod: S.obs.rod, near: [], hit: [], step_mm: 9 };
      if (S.obs.rod && S.obs.avoid) for (const g of grid) {
        if (!g.inside) continue;
        const c = S.obs.rod === ROD || S.obs.rod.p0 === ROD.p0 ? g.clear : clearanceAt(g.A, g.B, S.obs.rod), xz = [g.tip[0], g.tip[2]];
        if (c < 0) out.hit.push(xz); else if (c < S.obs.clearance_mm) out.near.push(xz);
      }
      mapCache = { key, map: out };
      return out;
    }
    function snapshot() {
      const t = now();
      const motors = S.motors.map(m => {
        const zero = zeroOf(m), [lo, hi] = limits(m), a = toAngle(m.pulse), online = S.connected && m.online;
        const e = { id: m.id, online, settings: { ...m.settings }, limits_rad: [lo, hi], calibrated: m.settings.zero_rad != null,
                    model: "XM430-W210", ma_per_unit: MA_PER_UNIT, mode: 3 };
        if (online) Object.assign(e, {
          pulse: Math.round(m.pulse), goal_pulse: Math.round(m.goal), torque: m.torque, moving: m.moving, hw_error: m.hw_error,
          current_raw: m.current_raw, velocity_raw: m.velocity_raw, temp_c: m.temp_c, angle_rad: a, goal_rad: toAngle(m.goal),
          angle_rel_rad: a - zero, goal_rel_rad: toAngle(m.goal) - zero, velocity_rpm: m.velocity_raw * 0.229, overridden: m.overridden,
          current_sustained_raw: m.current_raw, read_glitches: m.glitches,
          cable_mm: m.settings.zero_rad != null ? (a - zero) * m.settings.radius_mm * (m.settings.invert ? -1 : 1) : undefined,
        });
        e.clamped_recently = S.clampAt[m.id] != null && t - S.clampAt[m.id] < 2.5;   // sim: index.html tracks this client-side
        return e;
      });
      const d = S.model ? knownPose() : null;
      const tip = { model: S.model, status: { ...S.status }, busy: !!(S.move || S.joy.active), plan_id: S.planId };
      tip.joy = { active: S.joy.active, message: S.joy.message, enable: S.joy.enable, cmd: S.joy.cmd,
                  walls: S.joy.walls.map(w => w.motor), mode: S.joy.mode, control: S.joy.control };
      if (S.joy.active && S.joy.tgt && S.model) tip.joy.target_xz = [S.joy.tgt[0], S.joy.tgt[2]];
      const feedAge = t - S.feed.t;
      let measured = null;
      if (S.feed.on && S.connected !== undefined) {
        const base = predict(...(d ? [d.A, d.B] : [0, 0]));
        measured = base.map((v, k) => v + S.feed.bias[k] + Math.sin(t * 3 + k) * 0.3);
      }
      tip.feed = S.feed.on && feedAge <= 2.0 + (S.inject.feed_stale ? 1e9 : 0)
        ? (S.feed.markers ? { x: measured[0], y: measured[1], z: measured[2], markers: 1, age_s: feedAge } : { markers: 0, age_s: feedAge })
        : null;
      if (d) { tip.pose = d; tip.predicted = predict(d.A, d.B); tip.arm_model = backbone(d.A, d.B); }
      if (tip.feed && tip.feed.markers && tip.feed.age_s < 0.5) tip.arm_camera = arcThroughTip([tip.feed.x, tip.feed.y, tip.feed.z]);
      const o = S.obs;
      tip.obstacle = { rod: o.rod, source: o.source, avoid: o.avoid, clearance_mm: o.clearance_mm, version: o.version, hsv_set: !!o.hsv,
        rod_age_s: o.seen ? 0.2 : 12 + (t % 60), feed: !!(S.feed.on && feedAge < 2.0), seen: o.seen && S.feed.on, picking: o.picking,
        marking: o.marking, error: o.error, plan_xz: S.plan && !S.plan.preview ? S.plan.tips.map(p => [p[0], p[2]]) : null };
      if (o.rod && S.model) {
        if (d) tip.obstacle.clear_model_mm = clearanceAt(d.A, d.B, o.rod);
        if (tip.arm_camera) tip.obstacle.clear_camera_mm = armClearance(tip.arm_camera, o.rod);
      }
      tip.plan = S.plan;   // sim: the real GUI fetches GET /api/tip/plan or the preview response
      return { connected: S.connected, port: S.port, mock: true, error: S.error, profile_velocity: S.profile_velocity, motors, tip,
               camera: { age_s: S.cameraOn && S.feed.on ? feedAge : null }, cloud: { age_s: S.cameraOn ? 0.4 : null, seq: 1 },
               ui_version: "design-demo", scenario: S.scenario };
    }
    function emit() {
      if (!S.serverUp) { listeners.forEach(f => f(lastSnap, { ws: "closed", since: S.serverDownAt })); return; }
      lastSnap = snapshot();
      listeners.forEach(f => f(lastSnap, { ws: "open" }));
    }
    function gamepad() {                           // simulated gamepad (prototypes read this instead of navigator.getGamepads)
      if (!S.padScript) return null;
      const st = S.joy.stick || { x: 0, y: 0, enable: false };
      return { id: "Xbox Wireless Controller (simulated)", standard: true, x: st.x, y: st.y, enable: !!st.enable };
    }

    const timer = setInterval(tick, 100);         // sim: 10 Hz (the real WebSocket sends at 20 Hz)
    setScenario(opts.scenario || "idle");
    return {
      api, subscribe(f) { listeners.add(f); if (lastSnap) f(lastSnap, { ws: S.serverUp ? "open" : "closed", since: S.serverDownAt }); return () => listeners.delete(f); },
      onLog(f) { logListeners.add(f); return () => logListeners.delete(f); }, log: () => S.log.slice(),
      setScenario, setInject, setMotorInject, inject: () => ({ ...S.inject }), motorInject: () => ({ ...S.motorInject }),
      scenario: () => S.scenario, gamepad, model, obstacleMap, backbone, predict, clearanceAt, grid: () => grid, stop() { clearInterval(timer); },
    };
  }

  // ---- alerts: one derivation shared by all prototypes (text, severity, measurements, actions) ----
  // level: "fault" (red), "warning" (amber), "caution" (blue/cyan), "notice" (neutral, transient)
  const LEVEL_RANK = { fault: 0, warning: 1, caution: 2, notice: 3 };
  function deriveAlerts(s, meta = {}) {
    const out = [];
    const add = a => out.push(a);
    if (meta.ws === "closed") {
      add({ id: "server", level: "fault", source: "Control server", title: "Server unreachable — retrying",
            detail: "No updates from the control server. Every reading on screen is stale until it reconnects.",
            measure: [["Last update", meta.since ? new Date(Date.now() - (performance.now() / 1000 - meta.since) * 1000).toLocaleTimeString() : "—"]],
            raw: "WebSocket /ws closed", actions: [] });
    }
    if (!s) return out;
    if (!s.connected) {
      add({ id: "disconnected", level: "fault", source: "Robot connection", title: "Robot disconnected",
            detail: s.error ? `${s.error}. Motor readings are unknown. Check USB and power, then connect.` : "Not connected. Motor readings are unknown.",
            measure: [["Port", "—"]], raw: s.error || "connected = false", actions: [{ id: "connect", label: "Connect" }] });
    }
    s.motors.forEach(m => {
      const src = `Motor ${m.id}`;
      const cur = m.online ? (m.ma_per_unit ? `${Math.round(m.current_raw * m.ma_per_unit)} mA` : `${m.current_raw} u (raw)`) : "—";
      if (m.online && m.hw_error) add({ id: `hw${m.id}`, level: "fault", motor: m.id, source: src,
        title: `Hardware error: ${hwText(m.hw_error)}`,
        detail: "The motor switched its torque off. Release cable tension, then reboot it; it restarts with torque off.",
        measure: [["Measured current", cur], ["Temperature", m.temp_c != null ? `${m.temp_c} °C` : "—"], ["Torque", m.torque ? "on" : "off"]],
        raw: `Hardware Error Status = ${m.hw_error} (0x${m.hw_error.toString(16).padStart(2, "0")})`,
        actions: [{ id: "reboot", motor: m.id, label: `Reboot motor ${m.id}…` }] });
      if (s.connected && !m.online) add({ id: `off${m.id}`, level: "fault", motor: m.id, source: src, title: "Not responding",
        detail: "No reply on the motor bus. Check its cable and power.", measure: [], raw: "online = false", actions: [] });
      if (m.online && m.overridden) add({ id: `ovr${m.id}`, level: "warning", motor: m.id, source: src, title: "Commands overridden by another program",
        detail: "Its goal was changed by another program on the bus (e.g. an Emio Labs / SOFA scene with Robot connection on). Close that program.",
        measure: [["Goal", m.goal_rel_rad != null ? `${deg(m.goal_rel_rad).toFixed(1)}°` : "—"]], raw: "overridden = true", actions: [] });
      if (m.online && !m.calibrated) add({ id: `cal${m.id}`, level: "caution", motor: m.id, source: src, title: "Not calibrated",
        detail: "Angles are relative to emioapi's zero (pulse 2048). Tip control needs every motor's taut zero.",
        measure: [], raw: "settings.zero_rad = null", actions: [{ id: "setup-motor", motor: m.id, label: "Open calibration" }] });
      if (m.clamped_recently) add({ id: `clamp${m.id}`, level: "notice", motor: m.id, source: src, title: "Command clamped at limit",
        detail: `Goal limited to the soft range ${deg(m.limits_rad[0] - (m.settings.zero_rad ?? 0)).toFixed(0)}° to ${deg(m.limits_rad[1] - (m.settings.zero_rad ?? 0)).toFixed(0)}°.`,
        measure: [], raw: "goal response clamped = true", actions: [], transient: true });
    });
    const t = s.tip;
    if (t) {
      if (!t.model) add({ id: "nomodel", level: "caution", source: "Tip model", title: "No tip model",
        detail: "Tip mapping, Move tip and obstacle planning are unavailable. Motor controls and the motor-pair joystick still work.",
        measure: [], raw: "GET /api/tip/model → 404 (run fit_tip_model.py)", actions: [] });
      if (s.connected) {
        if (!t.feed) add({ id: "feedoff", level: "caution", source: "Camera", title: "Camera feed off",
          detail: "No tip position from the camera. Moves run open loop (model only, about ±8 mm) and the rod can't be re-detected.",
          measure: [], raw: "no POST /api/tip for > 2 s (start tip_feed.py)", actions: [] });
        else if (t.feed.age_s >= 0.5) add({ id: "feedstale", level: "warning", source: "Camera", title: `Camera data stale (${t.feed.age_s.toFixed(1)} s old)`,
          detail: "The tip position from the camera is out of date. Closed-loop correction is not used; the tip shown is the model's.",
          measure: [["Last frame", `${t.feed.age_s.toFixed(1)} s ago`]], raw: `feed.age_s = ${t.feed.age_s.toFixed(2)}`, actions: [] });
        else if (!t.feed.markers) add({ id: "marker", level: "warning", source: "Camera", title: "Tip marker not detected",
          detail: "The camera sees no green marker on the tip. Position shown is the model's; closed loop will stop at the open-loop result.",
          measure: [], raw: "feed.markers = 0", actions: [] });
      }
      const o = t.obstacle || {};
      if (o.rod && o.avoid && o.clear_model_mm != null && o.clear_model_mm < o.clearance_mm) {
        const touch = o.clear_model_mm < 0;
        add({ id: "clear", level: touch ? "fault" : "warning", source: "Obstacle",
          title: touch ? "Arm touching the rod (model)" : `Arm ${o.clear_model_mm.toFixed(0)} mm from the rod (setting ${o.clearance_mm.toFixed(0)} mm)`,
          detail: "Modelled clearance from motor angles: an estimate (model error about ±8 mm), not a guarantee.",
          measure: [["Model", `${o.clear_model_mm.toFixed(0)} mm`], ["Camera", o.clear_camera_mm != null ? `${o.clear_camera_mm.toFixed(0)} mm` : "unknown"], ["Setting", `${o.clearance_mm} mm`]],
          raw: `clear_model_mm = ${o.clear_model_mm.toFixed(1)}`, actions: [] });
      }
      if (o.rod && s.connected && t.feed && !o.seen) add({ id: "rodunseen", level: "caution", source: "Obstacle", title: "Rod not seen by the camera",
        detail: `Avoiding its last known position (${o.rod_age_s.toFixed(0)} s old).`, measure: [], raw: "obstacle.seen = false", actions: [] });
      if (o.error) add({ id: "obserr", level: "warning", source: "Obstacle", title: "Rod setup failed", detail: o.error.charAt(0).toUpperCase() + o.error.slice(1) + ".",
        measure: [], raw: o.error, actions: [] });
      if (t.status.state === "limited") add({ id: "limited", level: "warning", source: "Tip move", title: "Move stopped short: force limit",
        detail: "The backend stopped the move when a pulling motor's current passed its limit. The arm holds where it stopped.",
        measure: [], raw: t.status.message, actions: [] });
      if (t.status.state === "failed") add({ id: "failed", level: "fault", source: "Tip move", title: "Move failed",
        detail: t.status.message.charAt(0).toUpperCase() + t.status.message.slice(1) + ".", measure: [], raw: `tip.status = failed`, actions: [] });
      if (t.joy.active && t.joy.walls && t.joy.walls.length) add({ id: "joywall", level: "warning", source: "Joystick", title: `Force limit on motor ${t.joy.walls[0]}`,
        detail: "Ease the stick back; keep pushing to retry the limit.", measure: [], raw: t.joy.message, actions: [] });
    }
    return out.sort((a, b) => LEVEL_RANK[a.level] - LEVEL_RANK[b.level]);
  }
  // acknowledgement: kept per alert id while the alert is reported; cleared when the condition goes away
  function createAckStore() {
    const acked = new Set();
    return {
      ack(id) { acked.add(id); }, isAcked: id => acked.has(id),
      prune(alerts) { const live = new Set(alerts.map(a => a.id)); [...acked].forEach(id => { if (!live.has(id)) acked.delete(id); }); },
    };
  }

  window.EmioSim = { createSim, deriveAlerts, createAckStore, SCENARIOS, INJECT, MOTOR_INJECT, HW_ERRORS, hwText, LAYOUT_DEG,
                     MA_PER_UNIT, deg, rad, IDS };
})();
