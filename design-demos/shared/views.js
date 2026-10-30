// Workspace drawings for the design prototypes: top view, side view, simulated camera image, a light orbitable
// 3D view and the on-screen stick. Same content as the real GUI's views (index.html drawTop / drawSide,
// scene3d.js), drawn with canvas 2D so the prototypes stay standalone. Colours come from CSS variables on the
// element passed to theme(), so each design styles them.
(function () {
  "use strict";
  const { LAYOUT_DEG } = window.EmioSim;
  const rad = d => d * Math.PI / 180;

  function theme(el) {
    const cs = getComputedStyle(el), v = (n, d) => (cs.getPropertyValue(n).trim() || d);
    return {
      bg: v("--view-bg", "#f4f4f2"), ink: v("--view-ink", "#15171a"), muted: v("--view-muted", "#6a7078"), grid: v("--view-grid", "#d9dbde"),
      reach: v("--view-reach", "rgba(40,90,160,.08)"), reachLine: v("--view-reach-line", "#8aa0bb"), camera: v("--view-camera", "#14803c"),
      model: v("--view-model", "#15171a"), target: v("--view-target", "#c62828"), rod: v("--view-rod", "#d26a00"),
      near: v("--view-near", "rgba(210,130,0,.28)"), hit: v("--view-hit", "rgba(200,40,40,.4)"), path: v("--view-path", "#c62828"),
      preview: v("--view-preview", "#2a5fb0"), arm: v("--view-arm", "#2a5fb0"), armCam: v("--view-arm-cam", "#14803c"),
      pad: v("--view-pad", "#b06a00"), font: v("--view-font", "system-ui, sans-serif"),
    };
  }
  function setup(cv) {
    const dpr = window.devicePixelRatio || 1, w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const g = cv.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    return { g, w, h };
  }
  // camera (x, z) <-> screen (u right, v up), oriented by the physical motor layout (index.html setView)
  function viewMap(dirs) {
    const wrap = a => ((a % 360) + 540) % 360 - 180;
    const s = Math.sign(wrap(dirs[1] - dirs[0])) * Math.sign(wrap(LAYOUT_DEG[1] - LAYOUT_DEG[0])) || 1;
    const o = rad(LAYOUT_DEG[0] - s * dirs[0]), c = Math.cos(o), n = Math.sin(o);
    return s > 0
      ? { u: (x, z) => c * x - n * z, v: (x, z) => n * x + c * z, x: (u, v) => c * u + n * v, z: (u, v) => -n * u + c * v }
      : { u: (x, z) => c * x + n * z, v: (x, z) => n * x - c * z, x: (u, v) => c * u + n * v, z: (u, v) => n * u - c * v };
  }
  function dot(g, x, y, r, fill, stroke, lw = 2) {
    g.beginPath(); g.arc(x, y, r, 0, 2 * Math.PI);
    if (fill) { g.fillStyle = fill; g.fill(); }
    if (stroke) { g.strokeStyle = stroke; g.lineWidth = lw; g.stroke(); }
  }

  // ---- top view (x, z) ----
  function drawTop(cv, s, model, th, opts = {}) {
    const { g, w, h } = setup(cv);
    g.fillStyle = th.bg; g.fillRect(0, 0, w, h);
    if (!model) { g.fillStyle = th.muted; g.font = `14px ${th.font}`; g.textAlign = "center"; g.fillText("No tip model: top view unavailable", w / 2, h / 2); return null; }
    const vm = viewMap(model.motor_dirs_deg), uv = ([x, z]) => [vm.u(x, z), vm.v(x, z)];
    const pts = model.outline.map(uv), us = pts.map(p => p[0]), vs = pts.map(p => p[1]);
    const pad = 34, u0 = Math.min(...us), u1 = Math.max(...us), v0 = Math.min(...vs), v1 = Math.max(...vs);
    const k = Math.min((w - 2 * pad) / (u1 - u0), (h - 2 * pad) / (v1 - v0)), cu = (u0 + u1) / 2, cvv = (v0 + v1) / 2;
    const px = u => w / 2 + (u - cu) * k, py = v => h / 2 - (v - cvv) * k;
    const P = xz => { const [u, v] = uv(xz); return [px(u), py(v)]; };
    // grid every 20 mm
    g.strokeStyle = th.grid; g.lineWidth = 1;
    for (let u = Math.ceil((cu - w / 2 / k) / 20) * 20; u < cu + w / 2 / k; u += 20) { g.beginPath(); g.moveTo(px(u), 0); g.lineTo(px(u), h); g.stroke(); }
    for (let v = Math.ceil((cvv - h / 2 / k) / 20) * 20; v < cvv + h / 2 / k; v += 20) { g.beginPath(); g.moveTo(0, py(v)); g.lineTo(w, py(v)); g.stroke(); }
    // reachable area
    g.beginPath(); pts.forEach(([u, v], i) => i ? g.lineTo(px(u), py(v)) : g.moveTo(px(u), py(v))); g.closePath();
    g.fillStyle = th.reach; g.fill(); g.strokeStyle = th.reachLine; g.lineWidth = 1.5; g.stroke();
    // obstacle shading
    const om = opts.obsMap, o = s.tip && s.tip.obstacle;
    if (om && o && o.rod && o.avoid) {
      const sz = Math.max(2, (om.step_mm || 9) * k);
      g.fillStyle = th.near; om.near.forEach(xz => { const [x, y] = P(xz); g.fillRect(x - sz / 2, y - sz / 2, sz, sz); });
      g.fillStyle = th.hit; om.hit.forEach(xz => { const [x, y] = P(xz); g.fillRect(x - sz / 2, y - sz / 2, sz, sz); });
    }
    if (o && o.rod) {
      const a = P([o.rod.p0[0], o.rod.p0[2]]), b = P([o.rod.p1[0], o.rod.p1[2]]);
      g.strokeStyle = th.rod; g.lineCap = "round"; g.lineWidth = Math.max(4, 2 * o.rod.radius * k);
      g.beginPath(); g.moveTo(...a); g.lineTo(...b); g.stroke(); g.lineCap = "butt";
      g.fillStyle = th.rod; g.font = `600 12px ${th.font}`; g.textAlign = "left";
      g.fillText(`rod · y ${((o.rod.p0[1] + o.rod.p1[1]) / 2).toFixed(0)}`, b[0] + 8, b[1] - 6);
    }
    // planned path / preview
    const plan = s.tip && s.tip.plan;
    if (plan && plan.tips && plan.tips.length > 1) {
      g.setLineDash([6, 5]); g.strokeStyle = plan.preview ? th.preview : th.path; g.lineWidth = 2;
      g.beginPath(); plan.tips.forEach((p, i) => { const [x, y] = P([p[0], p[2]]); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); g.setLineDash([]);
    }
    // motor labels at their layout directions
    g.fillStyle = th.muted; g.font = `600 12px ${th.font}`; g.textAlign = "center";
    const r0 = model.rest_tip, [rx, ry] = P([r0[0], r0[2]]);
    LAYOUT_DEG.forEach((a, i) => g.fillText(`M${i}`, rx + (Math.min(w, h) / 2 - 16) * Math.cos(rad(a)), ry - (Math.min(w, h) / 2 - 16) * Math.sin(rad(a)) + 4));
    g.strokeStyle = th.muted; g.lineWidth = 1; g.beginPath(); g.moveTo(rx - 5, ry); g.lineTo(rx + 5, ry); g.moveTo(rx, ry - 5); g.lineTo(rx, ry + 5); g.stroke();
    // target, gamepad target, model tip, camera tip
    const t = s.tip;
    if (opts.target) { const [x, y] = P(opts.target); dot(g, x, y, 9, null, th.target, 2.5); g.beginPath(); g.moveTo(x - 13, y); g.lineTo(x + 13, y); g.moveTo(x, y - 13); g.lineTo(x, y + 13); g.strokeStyle = th.target; g.lineWidth = 1.5; g.stroke(); }
    if (t && t.joy && t.joy.target_xz) { const [x, y] = P(t.joy.target_xz); dot(g, x, y, 7, null, th.pad, 2); }
    if (t && t.predicted) { const [x, y] = P([t.predicted[0], t.predicted[2]]); dot(g, x, y, 7, null, th.model, 2.5); }
    if (t && t.feed && t.feed.markers) { const [x, y] = P([t.feed.x, t.feed.z]); dot(g, x, y, 6, t.feed.age_s < 0.5 ? th.camera : th.muted, null); }
    return { toWorld(cx, cy) { const u = cu + (cx - w / 2) / k, v = cvv - (cy - h / 2) / k; return [vm.x(u, v), vm.z(u, v)]; } };
  }

  // ---- side view: the bending plane ----
  function drawSide(cv, s, model, th) {
    const { g, w, h } = setup(cv);
    g.fillStyle = th.bg; g.fillRect(0, 0, w, h);
    if (!model) return;
    const t = s.tip, arm = t && (t.arm_model || null), cam = t && t.arm_camera, b = model.base, L = model.length_mm;
    const tipRef = arm ? arm[arm.length - 1] : cam ? cam[cam.length - 1] : model.rest_tip;
    let dx = tipRef[0] - b[0], dz = tipRef[2] - b[2]; const hn = Math.hypot(dx, dz); if (hn < 1) { dx = 1; dz = 0; } else { dx /= hn; dz /= hn; }
    const hor = p => (p[0] - b[0]) * dx + (p[2] - b[2]) * dz;
    const pad = 30, s0 = -0.35 * L, s1 = 1.1 * L, y0 = -245, y1 = b[1] + 20;
    const k = Math.min((w - 2 * pad) / (s1 - s0), (h - 2 * pad) / (y1 - y0));
    const X = v => w / 2 + (v - (s0 + s1) / 2) * k, Y = y => h / 2 - (y - (y0 + y1) / 2) * k;
    g.strokeStyle = th.grid; g.lineWidth = 1; g.beginPath(); g.moveTo(0, Y(b[1])); g.lineTo(w, Y(b[1])); g.stroke();
    g.strokeStyle = th.muted; g.beginPath(); g.moveTo(0, Y(-233)); g.lineTo(w, Y(-233)); g.stroke();
    g.fillStyle = th.muted; g.font = `12px ${th.font}`; g.textAlign = "left";
    g.fillText("attachment", X(0) + 8, Y(b[1]) - 6); g.fillText("platform", 8, Y(-233) - 6);
    const line = (pts, col, lw, dash) => { if (!pts) return; g.setLineDash(dash || []); g.strokeStyle = col; g.lineWidth = lw; g.lineCap = "round"; g.beginPath(); pts.forEach((p, i) => i ? g.lineTo(X(hor(p)), Y(p[1])) : g.moveTo(X(hor(p)), Y(p[1]))); g.stroke(); g.setLineDash([]); g.lineCap = "butt"; };
    // rod crossing the plane (faint)
    const o = t && t.obstacle;
    if (o && o.rod) {
      const a = o.rod.p0, c = o.rod.p1, nx = -dz, nz = dx, da = (a[0] - b[0]) * nx + (a[2] - b[2]) * nz, dc = (c[0] - b[0]) * nx + (c[2] - b[2]) * nz;
      line([a, c], th.rod, 3, [3, 4]);
      if (da * dc < 0) { const f = da / (da - dc), p = a.map((v, i) => v + (c[i] - v) * f); dot(g, X(hor(p)), Y(p[1]), Math.max(4, o.rod.radius * k), th.rod); }
    }
    line(cam, th.armCam, 4, [5, 4]);
    line(arm, th.arm, 6);
    if (arm) dot(g, X(hor(arm[arm.length - 1])), Y(arm[arm.length - 1][1]), 6, th.bg, th.model, 2.5);
    if (t && t.feed && t.feed.markers) dot(g, X(hor([t.feed.x, t.feed.y, t.feed.z])), Y(t.feed.y), 5.5, th.camera);
    if (arm) {
      const p = arm[arm.length - 1], bend = 2 * Math.atan2(Math.hypot(p[0] - b[0], p[2] - b[2]), b[1] - p[1]) * 180 / Math.PI;
      g.fillStyle = th.ink; g.font = `600 13px ${th.font}`; g.textAlign = "right"; g.fillText(`bend ${bend.toFixed(0)}°`, w - 10, 20);
    }
  }

  // ---- simulated camera image (what tip_feed.py sends: marker outlined, rod outlined) ----
  const CAM = (() => {
    const t = [-101, -14.8, -105.8], z = [0.494, -0.682, 0.539], nz = Math.hypot(...z), Z = z.map(v => v / nz);
    let X = [Z[1] * 0 - Z[2] * 1, Z[2] * 0 - Z[0] * 0, Z[0] * 1 - Z[1] * 0]; const nx = Math.hypot(...X); X = X.map(v => v / nx);
    let Y = [Z[1] * X[2] - Z[2] * X[1], Z[2] * X[0] - Z[0] * X[2], Z[0] * X[1] - Z[1] * X[0]];
    if (Y[1] > 0) { X = X.map(v => -v); Y = Y.map(v => -v); }
    return { t, X, Y, Z, fx: 386.4, fy: 386.4, ppx: 329.1, ppy: 242.1, W: 640, H: 480 };
  })();
  function camProject(p) {
    const d = [p[0] - CAM.t[0], p[1] - CAM.t[1], p[2] - CAM.t[2]];
    const x = d[0] * CAM.X[0] + d[1] * CAM.X[1] + d[2] * CAM.X[2], y = d[0] * CAM.Y[0] + d[1] * CAM.Y[1] + d[2] * CAM.Y[2], z = d[0] * CAM.Z[0] + d[1] * CAM.Z[1] + d[2] * CAM.Z[2];
    return z > 1 ? [CAM.fx * x / z + CAM.ppx, CAM.fy * y / z + CAM.ppy, z] : null;
  }
  function drawCamera(cv, s, model, th, opts = {}) {
    const { g, w, h } = setup(cv);
    g.fillStyle = "#0d0f11"; g.fillRect(0, 0, w, h);
    const sc = Math.min(w / CAM.W, h / CAM.H), ox = (w - CAM.W * sc) / 2, oy = (h - CAM.H * sc) / 2;
    const box = { x: ox, y: oy, w: CAM.W * sc, h: CAM.H * sc };
    if (opts.off) return box;
    g.save(); g.beginPath(); g.rect(ox, oy, box.w, box.h); g.clip();
    g.fillStyle = "#2a2724"; g.fillRect(ox, oy, box.w, box.h);
    const Pp = p => { const q = camProject(p); return q ? [ox + q[0] * sc, oy + q[1] * sc, q[2]] : null; };
    const poly = (pts, stroke, lw, fill) => { const q = pts.map(Pp).filter(Boolean); if (q.length < 2) return; g.beginPath(); q.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])); if (fill) { g.closePath(); g.fillStyle = fill; g.fill(); } if (stroke) { g.strokeStyle = stroke; g.lineWidth = lw; g.lineJoin = "round"; g.lineCap = "round"; g.stroke(); } };
    // table grid and platform
    g.globalAlpha = 0.5;
    for (let x = -200; x <= 200; x += 40) poly([[x, -262, -200], [x, -262, 220]], "#4a453f", 1);
    for (let z = -200; z <= 220; z += 40) poly([[-200, -262, z], [200, -262, z]], "#4a453f", 1);
    g.globalAlpha = 1;
    const circ = (c, r, y) => Array.from({ length: 49 }, (_, i) => [c[0] + r * Math.cos(i / 48 * 2 * Math.PI), y, c[2] + r * Math.sin(i / 48 * 2 * Math.PI)]);
    poly(circ(model ? model.base : [0, 0, 0], 95, -233), "#6d665d", 2, "#3a3631");
    for (const [x, z] of [[95, 95], [-95, 95], [95, -95]]) poly([[x, -30, z], [x, -262, z]], "#77706a", 10);
    poly(circ(model ? model.base : [0, 0, 0], 70, -30), "#8b847b", 3, "#55504a");
    // rod
    const o = s.tip && s.tip.obstacle;
    if (o && o.rod) { poly([o.rod.p0, o.rod.p1], "#e2731c", Math.max(5, 9 * sc)); }
    // arm (true shape ~ camera arc if known, else model)
    const arm = (s.tip && (s.tip.arm_camera || s.tip.arm_model)) || (model ? [model.base, model.rest_tip] : null);
    if (arm) { poly(arm, "#d9d4cc", 15 * sc); poly(arm, "#bdb6ab", 7 * sc); }
    const feed = s.tip && s.tip.feed;
    if (arm && feed && feed.markers) {
      const tip = Pp(arm[arm.length - 1]);
      if (tip) { dot(g, tip[0], tip[1], 8 * sc, "#2fbf4f"); dot(g, tip[0], tip[1], 12 * sc, null, "#00e5ff", 2.5); }
    }
    // tip_feed overlays: rod colour outline (magenta) and merged rod (red)
    if (o && o.rod && o.seen) {
      const a = Pp(o.rod.p0), b = Pp(o.rod.p1);
      if (a && b) {
        g.strokeStyle = "#ff3df2"; g.lineWidth = 1.5; const nx = -(b[1] - a[1]), ny = b[0] - a[0], nn = Math.hypot(nx, ny), off = 7 * sc;
        g.beginPath(); g.moveTo(a[0] + nx / nn * off, a[1] + ny / nn * off); g.lineTo(b[0] + nx / nn * off, b[1] + ny / nn * off);
        g.lineTo(b[0] - nx / nn * off, b[1] - ny / nn * off); g.lineTo(a[0] - nx / nn * off, a[1] - ny / nn * off); g.closePath(); g.stroke();
        g.strokeStyle = "#ff2a2a"; g.lineWidth = 2; g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
      }
    }
    (opts.marks || []).forEach(([u, v]) => dot(g, ox + u * box.w, oy + v * box.h, 7, null, "#ffd400", 2.5));
    g.restore();
    g.fillStyle = "rgba(255,255,255,.75)"; g.font = `600 11px ${th.font}`; g.textAlign = "left";
    g.fillText("SIMULATED CAMERA IMAGE", ox + 10, oy + 18);
    return box;
  }

  // ---- 3D view (orbit by drag, zoom by wheel, double-click the tip surface to preview) ----
  function draw3D(cv, s, sim, model, th, cam, opts = {}) {
    const { g, w, h } = setup(cv);
    g.fillStyle = th.bg; g.fillRect(0, 0, w, h);
    if (!model) { g.fillStyle = th.muted; g.font = `14px ${th.font}`; g.textAlign = "center"; g.fillText("No tip model: 3D view unavailable", w / 2, h / 2); return null; }
    const b = model.base, c = [b[0], b[1] - 110, b[2]];
    const cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch), f = Math.min(w, h) * 1.25 * cam.zoom;
    const proj = p => {
      const x = p[0] - c[0], y = p[1] - c[1], z = p[2] - c[2];
      const x1 = cy * x - sy * z, z1 = sy * x + cy * z, y2 = cp * y - sp * z1, z2 = sp * y + cp * z1, d = 650 - z2;
      return [w / 2 + f * x1 / d, h / 2 - f * y2 / d, d];
    };
    const line = (pts, col, lw, dash, alpha = 1) => { g.globalAlpha = alpha; g.setLineDash(dash || []); g.strokeStyle = col; g.lineWidth = lw; g.lineCap = "round"; g.lineJoin = "round"; g.beginPath(); pts.map(proj).forEach((q, i) => i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1])); g.stroke(); g.setLineDash([]); g.globalAlpha = 1; };
    // floor grid (stands in for the point cloud's table)
    if (opts.cloud !== false) {
      g.fillStyle = th.muted; g.globalAlpha = 0.45;
      for (let x = -180; x <= 180; x += 12) for (let z = -180; z <= 180; z += 12) { const q = proj([x, -262, z]); g.fillRect(q[0], q[1], 1.4, 1.4); }
      g.globalAlpha = 1;
    }
    const o = s.tip && s.tip.obstacle, need = o ? o.clearance_mm : 20;
    // tip surface wireframe, tinted where the arm would be inside the clearance
    if (opts.surface !== false) {
      const grid = sim.grid(), G = 41;
      for (let i = 0; i < G; i += 4) for (const along of [0, 1]) for (let j = 0; j < G - 1; j++) {
        const a = along ? grid[i * G + j] : grid[j * G + i], bb = along ? grid[i * G + j + 1] : grid[(j + 1) * G + i];
        if (!a.inside || !bb.inside) continue;
        const bad = o && o.rod && o.avoid && Math.min(a.clear, bb.clear) < need, hit = bad && Math.min(a.clear, bb.clear) < 0;
        line([a.tip, bb.tip], hit ? th.target : bad ? th.rod : th.reachLine, bad ? 1.6 : 1, null, bad ? 0.9 : 0.55);
      }
    }
    // base and motor labels
    line(Array.from({ length: 33 }, (_, i) => [b[0] + 30 * Math.cos(i / 32 * 2 * Math.PI), b[1], b[2] + 30 * Math.sin(i / 32 * 2 * Math.PI)]), th.muted, 2);
    g.fillStyle = th.muted; g.font = `600 12px ${th.font}`; g.textAlign = "center";
    model.motor_dirs_deg.forEach((a, i) => { const q = proj([b[0] + 62 * Math.cos(rad(a)), b[1] + 4, b[2] + 62 * Math.sin(rad(a))]); g.fillText(`M${i}`, q[0], q[1]); });
    // rod with its clearance zone (screen-space approximation of the capsule)
    if (o && o.rod) {
      const q0 = proj(o.rod.p0), q1 = proj(o.rod.p1), sc = f / ((q0[2] + q1[2]) / 2);
      if (o.avoid && opts.zone !== false) { g.globalAlpha = 0.16; g.strokeStyle = th.rod; g.lineCap = "round"; g.lineWidth = 2 * (o.rod.radius + need) * sc; g.beginPath(); g.moveTo(q0[0], q0[1]); g.lineTo(q1[0], q1[1]); g.stroke(); g.globalAlpha = 1; }
      g.strokeStyle = th.rod; g.lineCap = "round"; g.lineWidth = Math.max(4, 2 * o.rod.radius * sc); g.beginPath(); g.moveTo(q0[0], q0[1]); g.lineTo(q1[0], q1[1]); g.stroke();
    }
    // planned path and arm shapes along it
    const plan = s.tip && s.tip.plan;
    if (plan && opts.path !== false) {
      plan.arms.forEach((a, i) => line(a, plan.preview ? th.preview : th.path, 2.5, null, i === plan.arms.length - 1 ? 0.6 : 0.25));
      line(plan.tips, plan.preview ? th.preview : th.path, 2, [6, 5]);
      const e = proj(plan.tips[plan.tips.length - 1]); dot(g, e[0], e[1], 6, plan.preview ? th.preview : th.target);
    }
    if (s.tip && s.tip.arm_camera && opts.armCam !== false) line(s.tip.arm_camera, th.armCam, 5, [7, 5], 0.9);
    if (s.tip && s.tip.arm_model) { line(s.tip.arm_model, th.arm, 8); const e = proj(s.tip.arm_model[s.tip.arm_model.length - 1]); dot(g, e[0], e[1], 6, th.bg, th.model, 2.5); }
    // picking: nearest projected surface vertex
    return {
      pick(cx, cy2) {
        let best = null, bd = 22;
        for (const p of sim.grid()) if (p.inside) { const q = proj(p.tip), d = Math.hypot(q[0] - cx, q[1] - cy2); if (d < bd) { bd = d; best = p; } }
        return best ? { x: best.tip[0], y: best.tip[1], z: best.tip[2], clear: best.clear } : null;
      },
    };
  }
  function attachOrbit(cv, cam, redraw) {
    let drag = null;
    cv.addEventListener("pointerdown", e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; cv.setPointerCapture(e.pointerId); });
    cv.addEventListener("pointermove", e => { if (!drag) return; cam.yaw = drag.yaw + (e.clientX - drag.x) * 0.01; cam.pitch = Math.max(-0.2, Math.min(1.45, drag.pitch + (e.clientY - drag.y) * 0.01)); redraw(); });
    ["pointerup", "pointercancel"].forEach(t => cv.addEventListener(t, () => drag = null));
    cv.addEventListener("wheel", e => { e.preventDefault(); cam.zoom = Math.max(0.5, Math.min(3, cam.zoom * (e.deltaY < 0 ? 1.1 : 0.9))); redraw(); }, { passive: false });
  }
  const CAMERA_PRESETS = { front: { yaw: 0.6, pitch: 0.35, zoom: 1 }, top: { yaw: 0.6, pitch: 1.45, zoom: 1 }, camera: { yaw: -0.75, pitch: 0.75, zoom: 1.3 } };

  // ---- on-screen stick ----
  function drawStick(cv, x, y, enable, th, opts = {}) {
    const { g, w, h } = setup(cv), r = Math.min(w, h) / 2 - 6, cx = w / 2, cy = h / 2;
    if (r <= 0) return;  // hidden canvas (0 px)
    g.strokeStyle = th.grid; g.lineWidth = 2; g.beginPath(); g.arc(cx, cy, r, 0, 2 * Math.PI); g.stroke();
    g.beginPath(); g.moveTo(cx - r, cy); g.lineTo(cx + r, cy); g.moveTo(cx, cy - r); g.lineTo(cx, cy + r); g.stroke();
    g.fillStyle = th.muted; g.font = `600 ${opts.labelSize || 13}px ${th.font}`; g.textAlign = "center"; g.textBaseline = "middle";
    LAYOUT_DEG.forEach((a, i) => g.fillText(`${i}`, cx + 0.78 * r * Math.cos(rad(a)), cy - 0.78 * r * Math.sin(rad(a))));
    g.textBaseline = "alphabetic";
    dot(g, cx + x * r, cy - y * r, Math.max(14, r * 0.22), enable ? th.camera : th.muted);
  }
  // pointer handling for an on-screen stick: dragging is the dead-man; release recentres (index.html behaviour)
  function attachStick(cv, state) {
    let r = null;
    const update = e => { const rr = r.width / 2 - 6; let x = (e.clientX - r.left - r.width / 2) / rr, y = -(e.clientY - r.top - r.height / 2) / rr; const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; } state.x = x; state.y = y; };
    const release = () => { state.active = false; state.x = 0; state.y = 0; cv.classList.remove("dragging"); };
    cv.addEventListener("pointerdown", e => { r = cv.getBoundingClientRect(); cv.setPointerCapture(e.pointerId); state.active = true; cv.classList.add("dragging"); update(e); });
    cv.addEventListener("pointermove", e => { if (state.active) update(e); });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach(t => cv.addEventListener(t, release));
    window.addEventListener("blur", release);
  }
  // motor-pair stick: pushing toward a motor (in the layout) pulls it (index.html stickToPairs)
  function stickToPairs(x, y) {
    const r = LAYOUT_DEG.map(rad);
    return [x * Math.cos(r[0]) + y * Math.sin(r[0]), x * Math.cos(r[1]) + y * Math.sin(r[1])];
  }

  window.EmioViews = { theme, viewMap, drawTop, drawSide, drawCamera, draw3D, attachOrbit, CAMERA_PRESETS, drawStick, attachStick, stickToPairs };
})();
