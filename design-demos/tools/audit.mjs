// Accessibility / layout audit for the prototypes, run in headless Chrome over DevTools.
// node tools/audit.mjs <url> [width] [height] [--port 9361]
// Checks: horizontal overflow, visible interactive elements smaller than 44x44 (24x24 hard floor), text under
// 12 px, form controls / buttons without an accessible name, text contrast below 4.5:1 (3:1 for >=18.66px bold /
// 24px), and whether :focus-visible produces a visible change on each focusable element (sampled).
import { spawn } from "node:child_process";
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";

const a = process.argv.slice(2);
const [url] = a, W = +(a[1] || 1440), H = +(a[2] || 900);
const port = +(a.includes("--port") ? a[a.indexOf("--port") + 1] : 9361);
const chrome = spawn("google-chrome", ["--headless=new", `--user-data-dir=${mkdtempSync(join(tmpdir(), "aud-"))}`, `--remote-debugging-port=${port}`,
  "--use-angle=swiftshader", "--enable-unsafe-swiftshader", `--window-size=${W},${H}`, "about:blank"], { stdio: "ignore" });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let list; for (let i = 0; i < 60; i++) { try { list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json(); break; } catch { await sleep(200); } }
const ws = new WebSocket(list.find(t => t.type === "page").webSocketDebuggerUrl); await new Promise(r => ws.onopen = r);
let id = 0; const pend = {}; ws.onmessage = e => { const m = JSON.parse(e.data); if (m.id && pend[m.id]) { pend[m.id](m); delete pend[m.id]; } };
const send = (method, params = {}) => new Promise(r => { pend[++id] = r; ws.send(JSON.stringify({ id, method, params })); });
await send("Runtime.enable"); await send("Page.enable");
await send("Emulation.setDeviceMetricsOverride", { width: W, height: H, deviceScaleFactor: 1, mobile: false });
await send("Page.navigate", { url }); await sleep(3000);
const expr = `(() => {
  const out = { overflowX: document.documentElement.scrollWidth - innerWidth, small: [], tiny: [], fonts: [], unnamed: [], contrast: [], nofocus: [] };
  const vis = el => { const r = el.getBoundingClientRect(), cs = getComputedStyle(el); return r.width > 0 && r.height > 0 && cs.visibility !== "hidden" && cs.display !== "none" && !el.closest("[hidden],.simh,.simp"); };
  const name = el => (el.getAttribute("aria-label") || el.getAttribute("aria-labelledby") && document.getElementById(el.getAttribute("aria-labelledby"))?.textContent || el.labels?.[0]?.textContent || el.closest("label")?.textContent || el.textContent || el.title || "").trim();
  const desc = el => (el.id ? "#" + el.id : el.tagName.toLowerCase() + (el.className && typeof el.className === "string" ? "." + el.className.trim().split(/\\s+/).slice(0, 2).join(".") : "")) + " '" + name(el).slice(0, 28) + "'";
  const inter = [...document.querySelectorAll("button, a[href], input, select, textarea, [role=button], [role=tab], [role=switch], [tabindex]:not([tabindex='-1']), canvas[tabindex]")].filter(vis);
  for (const el of inter) {
    const r = el.getBoundingClientRect(), t = el.type;
    // checkboxes/radios inside a label: measure the label (its whole area is the target)
    const box = (t === "checkbox" || t === "radio") && el.closest("label") ? el.closest("label").getBoundingClientRect() : r;
    if (box.width < 24 || box.height < 24) out.tiny.push(desc(el) + " " + Math.round(box.width) + "x" + Math.round(box.height));
    else if (box.width < 44 || box.height < 44) out.small.push(desc(el) + " " + Math.round(box.width) + "x" + Math.round(box.height));
    if (!name(el) && el.tagName !== "CANVAS") out.unnamed.push(desc(el));
  }
  const lum = c => { const m = c.match(/[\\d.]+/g); if (!m) return null; const [r, g, b] = m.slice(0, 3).map(v => { v /= 255; return v <= .03928 ? v / 12.92 : ((v + .055) / 1.055) ** 2.4; }); return { L: .2126 * r + .7152 * g + .0722 * b, a: m[3] === undefined ? 1 : +m[3] }; };
  const bgOf = el => { for (let e = el; e; e = e.parentElement) { const c = getComputedStyle(e).backgroundColor, l = lum(c); if (l && l.a > 0.9) return l; } return lum("rgb(255,255,255)"); };
  const seen = new Set();
  for (const el of document.querySelectorAll("body *")) {
    if (!vis(el) || el.closest("svg,canvas")) continue;
    const own = [...el.childNodes].some(n => n.nodeType === 3 && n.textContent.trim());
    if (!own) continue;
    const cs = getComputedStyle(el), fs = parseFloat(cs.fontSize);
    if (fs < 12) out.fonts.push(desc(el) + " " + fs + "px");
    const fg = lum(cs.color), bg = bgOf(el); if (!fg || !bg) continue;
    const ratio = (Math.max(fg.L, bg.L) + .05) / (Math.min(fg.L, bg.L) + .05), large = fs >= 24 || (fs >= 18.66 && +cs.fontWeight >= 700);
    if (ratio < (large ? 3 : 4.5) * (fg.a < 1 ? 1.15 : 1)) { const k = desc(el) + " " + ratio.toFixed(2); if (!seen.has(k)) { seen.add(k); out.contrast.push(k); } }
  }
  // focus visibility: compare outline/box-shadow/border/background before and after focus (sample up to 60)
  for (const el of inter.slice(0, 60)) {
    const before = getComputedStyle(el); const sig = s => [s.outlineStyle, s.outlineWidth, s.boxShadow, s.borderColor, s.backgroundColor].join("|");
    const b = sig(before); el.focus({ focusVisible: true }); const f = sig(getComputedStyle(el)); el.blur();
    if (b === f) out.nofocus.push(desc(el));
  }
  for (const k of ["small", "tiny", "fonts", "unnamed", "contrast", "nofocus"]) out[k] = { n: out[k].length, sample: out[k].slice(0, 8) };
  return out;
})()`;
const r = await send("Runtime.evaluate", { expression: expr, returnByValue: true });
console.log(JSON.stringify(r.result.result.value ?? r.result, null, 1));
ws.close(); chrome.kill(); process.exit(0);
