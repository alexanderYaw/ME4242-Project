// Screenshot a page with headless Chrome over DevTools (no Python Playwright needed).
// node tools/shot.mjs <url> <out.png> [width] [height] [--full] [--wait ms] [--js "expr"] [--dark] [--port 9341]
// Prints console errors / exceptions, so a clean run shows "(no console errors)".
import { spawn } from "node:child_process";
import { writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";

const a = process.argv.slice(2);
const flag = (n, d) => { const i = a.indexOf(n); return i >= 0 ? a[i + 1] : d; };
const [url, out] = a;
const W = +(a[2] && !a[2].startsWith("--") ? a[2] : 1440), H = +(a[3] && !a[3].startsWith("--") ? a[3] : 900);
const full = a.includes("--full"), dark = a.includes("--dark"), wait = +flag("--wait", 2500), js = flag("--js", ""), port = +flag("--port", 9341);
const profile = mkdtempSync(join(tmpdir(), "shot-"));
const chrome = spawn("google-chrome", ["--headless=new", `--user-data-dir=${profile}`, `--remote-debugging-port=${port}`,
  "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--hide-scrollbars", `--window-size=${W},${H}`, "about:blank"], { stdio: "ignore" });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let list;
for (let i = 0; i < 60; i++) { try { list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json(); break; } catch { await sleep(200); } }
const ws = new WebSocket(list.find(t => t.type === "page").webSocketDebuggerUrl);
await new Promise(r => ws.onopen = r);
let id = 0; const pending = {}, logs = [];
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.id && pending[m.id]) { pending[m.id](m); delete pending[m.id]; }
  if (m.method === "Runtime.exceptionThrown") logs.push("EXCEPTION: " + (m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text));
  if (m.method === "Runtime.consoleAPICalled" && ["error", "warning"].includes(m.params.type)) logs.push(m.params.type + ": " + m.params.args.map(x => x.value ?? x.description).join(" "));
  if (m.method === "Log.entryAdded" && m.params.entry.level === "error" && !/favicon/.test(m.params.entry.url || "")) logs.push("LOG: " + m.params.entry.text + " " + (m.params.entry.url || ""));
};
const send = (method, params = {}) => new Promise(r => { pending[++id] = r; ws.send(JSON.stringify({ id, method, params })); });
await send("Runtime.enable"); await send("Log.enable"); await send("Page.enable");
await send("Emulation.setDeviceMetricsOverride", { width: W, height: H, deviceScaleFactor: 1, mobile: false });
if (dark) await send("Emulation.setEmulatedMedia", { features: [{ name: "prefers-color-scheme", value: "dark" }] });
await send("Page.navigate", { url });
await sleep(wait);
if (js) { const r = await send("Runtime.evaluate", { expression: js, awaitPromise: true, returnByValue: true }); console.log("js:", JSON.stringify(r.result?.result?.value ?? r.result)); await sleep(1200); }
let clip;
if (full) {
  const m = await send("Page.getLayoutMetrics");
  const ch = Math.ceil(m.result.cssContentSize.height);
  await send("Emulation.setDeviceMetricsOverride", { width: W, height: ch, deviceScaleFactor: 1, mobile: false });
  await sleep(600);
}
const shot = await send("Page.captureScreenshot", { format: "png", captureBeyondViewport: full });
writeFileSync(out, Buffer.from(shot.result.data, "base64"));
console.log(logs.length ? logs.join("\n") : "(no console errors)");
ws.close(); chrome.kill(); process.exit(0);
