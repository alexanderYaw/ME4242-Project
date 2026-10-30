// 3D view of the workspace (Tip control panel): the depth camera's point cloud, the arm as the model predicts it
// from the motor angles and as the camera sees it (the arc through the tracked tip), the reachable tip surface
// shaded by the arm's clearance from the obstacle, the marked rod with its clearance zone, the rods found by colour
// (shown in their colour, not avoided), and planned paths (arm shapes along the way). Double-click the tip surface to preview the path there; Move tip then goes there.
// World = the calibrated EMIO frame in mm, y up (the arm hangs down along -y).
import * as THREE from "three";
import { OrbitControls } from "/static/vendor/OrbitControls.js";

const el = id => document.getElementById(id);
const box = el("sceneBox"), canvas = el("sceneCanvas");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 2, 5000);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.addEventListener("change", requestRender);
scene.add(new THREE.HemisphereLight(0xffffff, 0x556677, 2.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(150, 400, 250);
scene.add(sun);

const v3 = a => new THREE.Vector3(a[0], a[1], a[2]);
const UP = new THREE.Vector3(0, 1, 0);

// ---- materials follow the page theme (CSS variables) ----
const themed = [];
function themedMat(Kind, cssVar, opts = {}) {
  const m = new Kind(opts);
  themed.push([m, cssVar]);
  return m;
}
const translucent = { transparent: true, depthWrite: false };
const M = {
  arm: themedMat(THREE.MeshLambertMaterial, "--accent"),
  armCam: themedMat(THREE.MeshLambertMaterial, "--ok", { ...translucent, opacity: 0.55 }),
  tipCam: themedMat(THREE.MeshLambertMaterial, "--ok"),
  rod: themedMat(THREE.MeshLambertMaterial, "--warn"),
  zone: themedMat(THREE.MeshLambertMaterial, "--warn", { ...translucent, opacity: 0.13, side: THREE.DoubleSide }),
  ghost: themedMat(THREE.MeshLambertMaterial, "--accent", { ...translucent, opacity: 0.35 }),
  ghostEnd: themedMat(THREE.MeshLambertMaterial, "--accent", { ...translucent, opacity: 0.6 }),
  play: themedMat(THREE.MeshLambertMaterial, "--danger", { ...translucent, opacity: 0.8 }),
  target: themedMat(THREE.MeshLambertMaterial, "--danger"),
  base: themedMat(THREE.MeshLambertMaterial, "--muted"),
  path: themedMat(THREE.LineDashedMaterial, "--danger", { dashSize: 4, gapSize: 3 }),
  frustum: themedMat(THREE.LineBasicMaterial, "--muted"),
  surfLines: themedMat(THREE.LineBasicMaterial, "--muted", { transparent: true, opacity: 0.35 }),
};
const surfMat = new THREE.MeshLambertMaterial({ vertexColors: true, ...translucent, opacity: 0.22, side: THREE.DoubleSide });
const cloudMat = new THREE.PointsMaterial({ size: 1.8, vertexColors: true, sizeAttenuation: true });
const flagMat = new THREE.PointsMaterial({ size: 3.6, vertexColors: true, sizeAttenuation: true });
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const color = v => new THREE.Color(css(v));
function applyTheme() {
  scene.background = color("--bg");
  for (const [m, v] of themed) m.color = color(v);
  if (sceneInfo) { buildSurface(sceneInfo); buildLabels(); }
  requestRender();
}
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyTheme);

// ---- objects ----
const mesh = (mat, geo = new THREE.BufferGeometry()) => { const m = new THREE.Mesh(geo, mat); scene.add(m); return m; };
const armModel = mesh(M.arm), armCam = mesh(M.armCam);
const tipCam = mesh(M.tipCam, new THREE.SphereGeometry(4, 16, 12));
const target = mesh(M.target, new THREE.SphereGeometry(3.5, 16, 12));
const base = mesh(M.base, new THREE.CylinderGeometry(22, 22, 3, 32));
const rod = mesh(M.rod), zone = mesh(M.zone);
const surface = mesh(surfMat);
surface.renderOrder = -1;  // translucent: drawn before the arm shapes so they show through it
const surfLines = new THREE.LineSegments(new THREE.BufferGeometry(), M.surfLines);
const pathLine = new THREE.Line(new THREE.BufferGeometry(), M.path);
const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), M.frustum);
const ghosts = new THREE.Group(), labels = new THREE.Group(), bars = new THREE.Group(), objects = new THREE.Group();
const playArm = mesh(M.play);
const cloudPts = new THREE.Points(new THREE.BufferGeometry(), cloudMat);
const flagPts = new THREE.Points(new THREE.BufferGeometry(), flagMat);
for (const o of [surfLines, pathLine, frustum, ghosts, labels, bars, objects, cloudPts, flagPts]) scene.add(o);
for (const o of [armModel, armCam, tipCam, target, rod, zone, playArm, base]) o.visible = false;
const axes = new THREE.Group();  // +x and +z arrows at the attachment (the X / Z boxes)
axes.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 45, 0xe0533d, 8, 5));
axes.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 45, 0x3d7be0, 8, 5));
scene.add(axes);

function setTube(m, pts, r) {
  m.geometry.dispose();
  m.visible = !!(pts && pts.length > 1);
  m.geometry = m.visible ? new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts.map(v3)), 40, r, 10, false)
    : new THREE.BufferGeometry();
}
function setLine(line, pts) {
  line.geometry.dispose();
  line.geometry = new THREE.BufferGeometry().setFromPoints((pts || []).map(v3));
  if (line.computeLineDistances) line.computeLineDistances();
}
function label(text, pos, cssVar = "--text") {
  const c = document.createElement("canvas"), g = c.getContext("2d"), font = "600 34px system-ui, sans-serif";
  g.font = font;
  c.width = Math.max(128, Math.ceil(g.measureText(text).width) + 16); c.height = 64;  // resizing resets the context
  g.font = font; g.fillStyle = css(cssVar); g.textAlign = "center"; g.textBaseline = "middle";
  g.fillText(text, c.width / 2, 32);
  const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(24 * c.width / 128, 12, 1); s.position.copy(pos);
  return s;
}

// ---- static part: tip surface, rod, attachment (GET /api/scene, changes with the model / obstacle) ----
let sceneInfo = null, sceneKey = null, sceneBusy = false, framed = false;
async function loadScene(key) {
  sceneBusy = true;
  try {
    const r = await fetch("/api/scene");
    sceneInfo = r.ok ? await r.json() : null;
    sceneKey = key;
  } catch { /* server restarting */ }
  sceneBusy = false;
  if (!sceneInfo) return;
  const b = v3(sceneInfo.base);
  base.position.copy(b).add(new THREE.Vector3(0, 2, 0)); base.visible = true;
  axes.position.copy(b);
  buildSurface(sceneInfo);
  buildRod(sceneInfo);
  buildLabels();
  if (cloud) buildCloud();
  if (!framed) { framed = true; setPreset("front"); }
  requestRender();
}
function buildSurface(sc) {
  const n = sc.grid, N = n * n, P = new Float32Array(N * 3), C = new Float32Array(N * 3);
  const cOk = color("--accent"), cNear = color("--warn"), cHit = color("--danger"), need = sc.clearance_mm;
  for (let i = 0; i < N; i++) {
    P.set(sc.tips[i], i * 3);
    const c = sc.clear ? sc.clear[i] : null;
    (c == null || c >= need ? cOk : c >= 0 ? cNear : cHit).toArray(C, i * 3);
  }
  const idx = [], seg = [], ins = sc.inside;
  for (let i = 0; i < n - 1; i++) for (let j = 0; j < n - 1; j++) {
    const a = i * n + j, b = a + 1, c = a + n, d = c + 1;
    if (ins[a] && ins[b] && ins[c] && ins[d]) idx.push(a, c, b, b, c, d);
  }
  for (let i = 0; i < n; i++) for (let j = 0; j < n - 1; j++) {  // grid lines every 5th bend value
    const along = [[i * n + j, i * n + j + 1], [j * n + i, (j + 1) * n + i]];
    if (i % 5 === 0) for (const [a, b] of along) if (ins[a] && ins[b]) seg.push(...sc.tips[a], ...sc.tips[b]);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(P, 3));
  g.setAttribute("color", new THREE.BufferAttribute(C, 3));
  g.setIndex(idx); g.computeVertexNormals();
  surface.geometry.dispose(); surface.geometry = g;
  surfLines.geometry.dispose();
  surfLines.geometry = new THREE.BufferGeometry().setAttribute("position", new THREE.Float32BufferAttribute(seg, 3));
}
function buildRod(sc) {
  rod.visible = zone.visible = false;
  const r = sc.rod; if (!r) return;
  const A = v3(r.p0), B = v3(r.p1), d = B.clone().sub(A), L = d.length();
  for (const m of [rod, zone]) {
    m.position.copy(A).add(B).multiplyScalar(0.5);
    m.quaternion.setFromUnitVectors(UP, d.clone().normalize());
  }
  rod.geometry.dispose(); rod.geometry = new THREE.CylinderGeometry(r.radius, r.radius, L, 24);
  zone.geometry.dispose(); zone.geometry = new THREE.CapsuleGeometry(r.radius + sc.clearance_mm, L, 8, 32);
  rod.visible = true; zone.visible = sc.avoid && el("sceneZone").checked;
}
function buildBars(rods) {  // rods found by colour (state.tip.obstacle.rods): shown, not avoided
  for (const m of bars.children) { m.geometry.dispose(); m.material.dispose(); }
  bars.clear();
  for (const r of rods || []) {
    const A = v3(r.p0), B = v3(r.p1), d = B.clone().sub(A);
    const c = typeof rodColour === "function" ? rodColour(r.colour) : css("--warn");
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r.radius, r.radius, d.length(), 16),
      new THREE.MeshLambertMaterial({ color: new THREE.Color(c) }));
    m.position.copy(A).add(B).multiplyScalar(0.5);
    m.quaternion.setFromUnitVectors(UP, d.normalize());
    bars.add(m);
  }
  requestRender();
}
let objectsBusy = false;
async function loadObjects(version) {  // other objects found by colour: their surface as the camera sees it
  objectsBusy = true;
  let info = null;
  try { const r = await fetch("/api/obstacle/objects"); info = r.ok ? await r.json() : null; } catch { /* restarting */ }
  objectsBusy = false;
  if (!info) return;
  lastObjects = version;
  for (const m of objects.children) { m.geometry?.dispose(); m.material?.map?.dispose(); m.material?.dispose(); }
  objects.clear();
  for (const ob of info.objects) {
    const c = new THREE.Color(typeof rodColour === "function" ? rodColour(ob.colour) : css("--warn"));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(ob.verts.flat(), 3));
    geo.setIndex(ob.faces.flat());
    geo.computeVertexNormals();
    objects.add(new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: c, side: THREE.DoubleSide })));
    const edge = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(ob.outline.map(v3)),
      new THREE.LineBasicMaterial({ color: c.clone().multiplyScalar(0.6) }));
    objects.add(edge);
    objects.add(label(ob.shape, v3(ob.centre).add(new THREE.Vector3(0, 14, 0))));
    for (const pk of ob.pockets || []) {  // shaped holes: their rim, and their name just above it
      const rim = pk.outline.map(v3);
      objects.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(rim),
        new THREE.LineBasicMaterial({ color: c.clone().multiplyScalar(0.35) })));
      const mid = rim.reduce((a, p) => a.add(p), new THREE.Vector3()).multiplyScalar(1 / rim.length);
      objects.add(label(`${pk.shape} pocket`, mid.add(new THREE.Vector3(0, 6, 0))));
    }
  }
  requestRender();
}
function buildLabels() {  // motor numbers in the direction each one bends the arm
  labels.clear();
  const dirs = typeof tipModel !== "undefined" && tipModel && tipModel.motor_dirs_deg;
  if (!dirs || !sceneInfo) return;
  const b = v3(sceneInfo.base);
  dirs.forEach((a, i) => {
    const r = a * Math.PI / 180;
    labels.add(label(`M${i}`, b.clone().add(new THREE.Vector3(70 * Math.cos(r), 8, 70 * Math.sin(r))), "--muted"));
  });
}

// ---- point cloud from tip_feed.py (binary, see CloudSender there) ----
let cloud = null, cloudSeq = -1, cloudBusy = false;
const toLinear = new Float32Array(256).map((_, i) => { const c = i / 255; return c <= 0.04045 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4; });
async function loadCloud() {
  cloudBusy = true;
  try {
    const r = await fetch(`/api/camera/cloud?after=${cloudSeq}`, { cache: "no-store" });
    if (r.status === 200) {
      const buf = await r.arrayBuffer(), hl = new DataView(buf).getUint32(0, true);
      const head = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, hl))), n = head.n, off = 4 + hl;
      cloud = { head, xyz: new Int16Array(buf, off, n * 3), rgb: new Uint8Array(buf, off + n * 6, n * 3),
                flag: new Uint8Array(buf, off + n * 9, n) };
      cloudSeq = +r.headers.get("X-Seq");
      buildCloud();
      buildFrustum(head.camera);
      requestRender();
    }
  } catch { /* server restarting / feed stopped */ }
  cloudBusy = false;
}
function buildCloud() {
  const { head, xyz, rgb, flag } = cloud, n = head.n, s = 1 / head.scale;
  const crop = +el("sceneCrop").value, sc = sceneInfo;
  const bx = sc ? sc.base[0] : 0, bz = sc ? sc.base[2] : 0, top = sc ? sc.base[1] + 60 : Infinity;
  const out = [[], []], col = [[], []];
  for (let i = 0; i < n; i++) {
    const x = xyz[3 * i] * s, y = xyz[3 * i + 1] * s, z = xyz[3 * i + 2] * s;
    if (sc && crop > 0 && (Math.hypot(x - bx, z - bz) > crop || y > top)) continue;
    const k = flag[i] ? 1 : 0;
    out[k].push(x, y, z);
    col[k].push(toLinear[rgb[3 * i]], toLinear[rgb[3 * i + 1]], toLinear[rgb[3 * i + 2]]);
  }
  [cloudPts, flagPts].forEach((p, k) => {
    p.geometry.dispose();
    p.geometry = new THREE.BufferGeometry();
    p.geometry.setAttribute("position", new THREE.Float32BufferAttribute(out[k], 3));
    p.geometry.setAttribute("color", new THREE.Float32BufferAttribute(col[k], 3));
  });
  el("sceneCloudInfo").textContent = `${(out[0].length + out[1].length) / 3} points` + (out[1].length ? `, ${out[1].length / 3} obstacle-coloured` : "");
}
let camPose = null;
function buildFrustum(c) {
  const R = c.R, t = v3(c.centre), d = 40;
  const w = (u, v) => {  // camera pixel (u, v) at depth d -> world
    const p = [(u - c.ppx) / c.fx * d, (v - c.ppy) / c.fy * d, d];
    return new THREE.Vector3(...[0, 1, 2].map(r => R[r][0] * p[0] + R[r][1] * p[1] + R[r][2] * p[2])).add(t);
  };
  const k = [w(0, 0), w(c.w, 0), w(c.w, c.h), w(0, c.h)], seg = [];
  for (let i = 0; i < 4; i++) seg.push(t, k[i], k[i], k[(i + 1) % 4]);
  frustum.geometry.dispose(); frustum.geometry = new THREE.BufferGeometry().setFromPoints(seg);
  camPose = { t, dir: new THREE.Vector3(R[0][2], R[1][2], R[2][2]), fov: 2 * Math.atan(c.h / 2 / c.fy) * 180 / Math.PI };
}

// ---- planned paths: a move's plan (GET /api/tip/plan) or a preview (POST /api/tip/preview) ----
let path = null, planId = null, play = null;
function showPath(p) {
  path = p && p.tips && p.tips.length ? p : null;
  ghosts.children.forEach(g => g.geometry.dispose()); ghosts.clear();
  setLine(pathLine, path ? path.tips : []);
  target.visible = !!path;
  if (path) {
    target.position.copy(v3(path.tips[path.tips.length - 1]));
    path.arms.forEach((a, i) => {
      const g = new THREE.Mesh(new THREE.BufferGeometry(), i === path.arms.length - 1 ? M.ghostEnd : M.ghost);
      setTube(g, a, 2.5); ghosts.add(g);
    });
  }
  el("scenePlay").disabled = !(path && path.arms.length > 1);
  stopPlay();
  applyToggles();
}
async function loadPlan(id) {
  planId = id;
  try { const r = await fetch("/api/tip/plan"); if (r.ok) { const p = await r.json(); if (p.tips.length) showPath(p); } } catch { }
}
function stopPlay() { play = null; playArm.visible = false; requestRender(); }
function animate(now) {  // a solid arm moving through the ghost shapes
  if (!play || !path) return;
  const arms = path.arms, f = Math.min(1, (now - play.t0) / 2500) * (arms.length - 1);
  const i = Math.min(arms.length - 2, Math.floor(f)), k = f - i;
  setTube(playArm, arms[i].map((p, j) => p.map((v, c) => v + (arms[i + 1][j][c] - v) * k)), 4.5);
  requestRender();
  if (f < arms.length - 1) requestAnimationFrame(animate); else setTimeout(stopPlay, 800);
}
el("scenePlay").addEventListener("click", () => { if (path) { play = { t0: performance.now() }; requestAnimationFrame(animate); } });

async function preview(x, z) {
  el("sceneMsg").textContent = "planning…";
  try {
    const r = await fetch("/api/tip/preview", { method: "POST", headers: { "Content-Type": "application/json" },
                                                 body: JSON.stringify({ x, z }) });
    const p = await r.json().catch(() => ({}));
    if (!r.ok) { showPath(null); el("sceneMsg").textContent = `No path: ${p.detail || r.statusText}`; return; }
    showPath(p);
    const clr = p.clear_min_mm != null ? ` · the arm passes ${p.clear_min_mm.toFixed(0)} mm from the rod` : "";
    el("sceneMsg").textContent = `Preview to (${x.toFixed(0)}, ${z.toFixed(0)}): ${p.note || "straight there"}${clr}. Press Move tip to go.`;
    play = { t0: performance.now() }; requestAnimationFrame(animate);
  } catch { el("sceneMsg").textContent = "server not reachable"; }
}

// ---- picking on the tip surface: hover shows the spot, double-click previews a move there ----
const ray = new THREE.Raycaster();
function surfaceHit(e) {
  if (!surface.visible || !sceneInfo) return null;
  const r = canvas.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), camera);
  const h = ray.intersectObject(surface)[0];
  if (!h) return null;
  const c = sceneInfo.clear ? Math.min(...[h.face.a, h.face.b, h.face.c].map(i => sceneInfo.clear[i])) : null;
  return { p: h.point, clear: c };
}
canvas.addEventListener("pointermove", e => {
  if (e.buttons) return;
  const h = surfaceHit(e);
  el("sceneHover").textContent = h ? `tip (${h.p.x.toFixed(0)}, ${h.p.y.toFixed(0)}, ${h.p.z.toFixed(0)})` +
    (h.clear != null ? ` · arm ${h.clear.toFixed(0)} mm from the rod` : "") + " · double-click to preview" : "";
});
canvas.addEventListener("dblclick", e => {
  const h = surfaceHit(e);
  if (!h) return;
  if (typeof setTipTarget === "function") setTipTarget(h.p.x, h.p.z);
  preview(h.p.x, h.p.z);
});

// ---- camera presets ----
function setPreset(name) {
  const b = sceneInfo ? v3(sceneInfo.base) : new THREE.Vector3(), L = sceneInfo ? sceneInfo.length_mm : 200;
  const mid = b.clone().add(new THREE.Vector3(0, -L * 0.55, 0));
  // "front" = the bottom edge of the top view (between motors 2 and 3), as laid out in the 2D view
  const up2d = typeof view !== "undefined" ? new THREE.Vector3(view.x(0, 1), 0, view.z(0, 1)).normalize() : new THREE.Vector3(0, 0, -1);
  camera.fov = 40;
  if (name === "camera" && camPose) {
    camera.fov = camPose.fov;
    camera.position.copy(camPose.t);
    controls.target.copy(camPose.t).addScaledVector(camPose.dir, 200);
  } else if (name === "top") {
    controls.target.copy(mid);
    camera.position.copy(mid).add(new THREE.Vector3(0, 2.6 * L, 0)).addScaledVector(up2d, -0.002 * L);
  } else {
    controls.target.copy(mid);
    camera.position.copy(mid).addScaledVector(up2d, -2.0 * L).add(new THREE.Vector3(0, 0.9 * L, 0));
  }
  camera.updateProjectionMatrix();
  controls.update();
  requestRender();
}
for (const [id, name] of [["sceneFront", "front"], ["sceneTop", "top"], ["sceneCam", "camera"]])
  el(id).addEventListener("click", () => setPreset(name));

// ---- toggles ----
function applyToggles() {
  cloudPts.visible = flagPts.visible = frustum.visible = el("sceneCloud").checked;
  surface.visible = surfLines.visible = el("sceneSurface").checked;
  zone.visible = !!(sceneInfo && sceneInfo.rod && sceneInfo.avoid && el("sceneZone").checked);
  ghosts.visible = pathLine.visible = el("scenePath").checked;
  target.visible = !!path && el("scenePath").checked;
  requestRender();
}
for (const id of ["sceneCloud", "sceneSurface", "sceneZone", "scenePath", "sceneArmCam"])
  el(id).addEventListener("change", () => { applyToggles(); if (id === "sceneArmCam") lastCam = null; });
el("sceneCrop").addEventListener("change", () => { if (cloud) buildCloud(); requestRender(); });

// ---- rendering: only on changes, and only while the view is on screen ----
let pending = false, onScreen = false;
function requestRender() {
  if (pending || !onScreen || document.hidden) return;
  pending = true;
  requestAnimationFrame(() => {
    pending = false;
    const w = box.clientWidth, h = box.clientHeight;
    if (canvas.width !== Math.round(w * renderer.getPixelRatio()) || canvas.height !== Math.round(h * renderer.getPixelRatio())) {
      renderer.setSize(w, h, false);
      camera.aspect = w / Math.max(1, h);
      camera.updateProjectionMatrix();
    }
    if (controls.update()) requestRender();  // still easing (damping)
    renderer.render(scene, camera);
  });
}
new IntersectionObserver(es => { onScreen = es[0].isIntersecting; requestRender(); }).observe(box);
new ResizeObserver(requestRender).observe(box);
document.addEventListener("visibilitychange", requestRender);

// ---- live update from the GUI's state (called by renderTip in index.html) ----
let lastModel = null, lastCam = null, lastBars = null, lastObjects = null;
function update(s) {
  const t = s.tip;
  el("sceneNote").style.display = t && t.model ? "none" : "grid";
  if (!t || !t.model) return;
  const o = t.obstacle || {}, key = `${o.version}-${o.avoid}-${typeof tipModel !== "undefined" && tipModel ? tipModel.fitted : ""}`;
  if (key !== sceneKey && !sceneBusy) loadScene(key);
  if (o.rods_version !== lastBars) { lastBars = o.rods_version; buildBars(o.rods); }
  if (o.objects_version !== undefined && o.objects_version !== lastObjects && !objectsBusy) loadObjects(o.objects_version);
  if (!onScreen || document.hidden) return;
  const am = JSON.stringify(t.arm_model || null);
  if (am !== lastModel) { lastModel = am; setTube(armModel, t.arm_model, 4); requestRender(); }
  const ac = el("sceneArmCam").checked ? t.arm_camera || null : null, acs = JSON.stringify(ac);
  if (acs !== lastCam) {
    lastCam = acs; setTube(armCam, ac, 3);
    tipCam.visible = !!ac; if (ac) tipCam.position.copy(v3(ac[ac.length - 1]));
    requestRender();
  }
  if (t.plan_id !== planId) loadPlan(t.plan_id);
  const c = s.cloud || {};
  if (el("sceneCloud").checked && c.seq > cloudSeq && !cloudBusy) loadCloud();
  el("sceneCloudInfo").hidden = !(c.age_s != null && c.age_s < 3);
  el("sceneCam").disabled = !camPose;
}
window.scene3d = { update };
applyTheme();
applyToggles();
