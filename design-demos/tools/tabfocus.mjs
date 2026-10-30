// Press Tab N times with real key events and report, for each focused element, whether its outline/box-shadow
// differs from its unfocused style. node tools/tabfocus.mjs <url> [n] [--port 9371]
import { spawn } from "node:child_process"; import { mkdtempSync } from "node:fs"; import { tmpdir } from "node:os"; import { join } from "node:path";
const [url, n = "12"] = process.argv.slice(2), port = 9371;
const chrome = spawn("google-chrome", ["--headless=new", `--user-data-dir=${mkdtempSync(join(tmpdir(), "tab-"))}`, `--remote-debugging-port=${port}`, "--window-size=1440,900", "about:blank"], { stdio: "ignore" });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let list; for (let i = 0; i < 60; i++) { try { list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json(); break; } catch { await sleep(200); } }
const ws = new WebSocket(list.find(t => t.type === "page").webSocketDebuggerUrl); await new Promise(r => ws.onopen = r);
let id = 0; const p = {}; ws.onmessage = e => { const m = JSON.parse(e.data); if (m.id && p[m.id]) { p[m.id](m); delete p[m.id]; } };
const send = (method, params = {}) => new Promise(r => { p[++id] = r; ws.send(JSON.stringify({ id, method, params })); });
await send("Page.enable"); await send("Emulation.setDeviceMetricsOverride", { width: 1440, height: 900, deviceScaleFactor: 1, mobile: false });
await send("Page.navigate", { url }); await sleep(2500);
const res = [];
for (let i = 0; i < +n; i++) {
  await send("Input.dispatchKeyEvent", { type: "keyDown", key: "Tab", code: "Tab", windowsVirtualKeyCode: 9 });
  await send("Input.dispatchKeyEvent", { type: "keyUp", key: "Tab", code: "Tab", windowsVirtualKeyCode: 9 });
  await sleep(80);
  const r = await send("Runtime.evaluate", { returnByValue: true, expression: `(() => { const el = document.activeElement; const s = getComputedStyle(el);
    return (el.id || el.tagName) + " " + (el.textContent || el.getAttribute("aria-label") || "").trim().slice(0, 20) + " → outline " + s.outlineStyle + " " + s.outlineWidth + (s.boxShadow !== "none" ? " +shadow" : "") + " focusVisible=" + el.matches(":focus-visible"); })()` });
  res.push(r.result.result.value);
}
console.log(res.join("\n")); ws.close(); chrome.kill(); process.exit(0);
