// Simulation harness for the design prototypes: a labelled strip (not part of any design) to switch demo
// scenarios, inject conditions and read the simulated command log. URL: ?scenario=overload&harness=0 hides it
// (for screenshots); the prototype reads --sim-h to leave room for it.
(function () {
  "use strict";
  const { SCENARIOS, INJECT, MOTOR_INJECT, HW_ERRORS } = window.EmioSim;
  const css = `
  .simh{position:fixed;left:0;right:0;top:0;height:34px;z-index:9000;display:flex;align-items:center;gap:6px;padding:0 10px;
    background:#101214;color:#d6d9dd;font:12px/1 ui-monospace,"JetBrains Mono",Menlo,Consolas,monospace;border-bottom:1px solid #2b3036}
  .simh b{color:#ffcf4a;letter-spacing:.06em;margin-right:6px}
  .simh button{font:inherit;color:#d6d9dd;background:#1d2126;border:1px solid #343a42;border-radius:4px;padding:5px 8px;cursor:pointer;min-height:26px}
  .simh button[aria-pressed=true]{background:#ffcf4a;color:#111;border-color:#ffcf4a}
  .simh button:focus-visible{outline:2px solid #ffcf4a;outline-offset:1px}
  .simh .sp{flex:1}
  .simp{position:fixed;top:34px;right:10px;z-index:9001;width:min(440px,calc(100vw - 20px));max-height:calc(100vh - 50px);overflow:auto;
    background:#15181b;color:#d6d9dd;border:1px solid #343a42;border-radius:0 0 6px 6px;padding:10px 12px;
    font:12px/1.45 ui-monospace,"JetBrains Mono",Menlo,Consolas,monospace;box-shadow:0 10px 30px rgba(0,0,0,.4)}
  .simp h4{margin:10px 0 6px;font-size:11px;color:#9aa1a9;text-transform:uppercase;letter-spacing:.08em}
  .simp label{display:flex;gap:8px;align-items:center;padding:2px 0;cursor:pointer}
  .simp select{font:inherit;background:#1d2126;color:#d6d9dd;border:1px solid #343a42;border-radius:4px;padding:2px 4px}
  .simp table{border-collapse:collapse;width:100%}.simp td{padding:3px 4px;border-top:1px solid #262b31}
  .simlog div{border-top:1px solid #262b31;padding:4px 0;word-break:break-word}.simlog .e{color:#ff8a80}`;

  function init(sim) {
    const qs = new URLSearchParams(location.search);
    if (qs.get("scenario") && SCENARIOS[qs.get("scenario")]) sim.setScenario(qs.get("scenario"));
    (qs.get("inject") || "").split(",").filter(k => INJECT[k]).forEach(k => sim.setInject(k, true));
    if (qs.get("harness") === "0") { document.documentElement.style.setProperty("--sim-h", "0px"); return; }
    document.documentElement.style.setProperty("--sim-h", "34px");
    const st = document.createElement("style"); st.textContent = css; document.head.appendChild(st);
    const bar = document.createElement("div"); bar.className = "simh"; bar.setAttribute("role", "region"); bar.setAttribute("aria-label", "Simulation controls");
    bar.innerHTML = `<b>SIMULATION</b><span style="color:#9aa1a9">no hardware ·</span>` +
      Object.entries(SCENARIOS).map(([k, l]) => `<button data-sc="${k}" aria-pressed="false">${l}</button>`).join("") +
      `<span class="sp"></span><button data-p="inject" aria-expanded="false">Inject conditions</button><button data-p="log" aria-expanded="false">Command log <span class="n">0</span></button>`;
    document.body.appendChild(bar);
    const panel = document.createElement("div"); panel.className = "simp"; panel.hidden = true; document.body.appendChild(panel);
    let open = null;
    const syncSc = () => bar.querySelectorAll("[data-sc]").forEach(b => b.setAttribute("aria-pressed", b.dataset.sc === sim.scenario()));
    bar.addEventListener("click", e => {
      const b = e.target.closest("button"); if (!b) return;
      if (b.dataset.sc) { sim.setScenario(b.dataset.sc); syncSc(); if (open === "inject") renderInject(); }
      if (b.dataset.p) { open = open === b.dataset.p ? null : b.dataset.p; render(); }
    });
    function render() {
      bar.querySelectorAll("[data-p]").forEach(b => b.setAttribute("aria-expanded", b.dataset.p === open));
      panel.hidden = !open;
      if (open === "inject") renderInject(); else if (open === "log") renderLog();
    }
    function renderInject() {
      const inj = sim.inject(), mi = sim.motorInject();
      panel.innerHTML = `<h4>System conditions</h4>` + Object.entries(INJECT).map(([k, l]) =>
        `<label><input type="checkbox" data-i="${k}" ${inj[k] ? "checked" : ""}> ${l}</label>`).join("") +
        `<h4>Per motor</h4><table>${[0, 1, 2, 3].map(i => `<tr><td>Motor ${i}</td><td><select data-hw="${i}" aria-label="Motor ${i} hardware error">
          <option value="0">no HW error</option>${Object.entries(HW_ERRORS).map(([c, t]) => `<option value="${c}" ${+mi[`${i}:hw`] === +c ? "selected" : ""}>${t} (${c})</option>`).join("")}
          <option value="2" ${+mi[`${i}:hw`] === 2 ? "selected" : ""}>other (2)</option></select></td>` +
          ["offline", "overridden", "uncal"].map(k => `<td><label><input type="checkbox" data-m="${i}:${k}" ${mi[`${i}:${k}`] ? "checked" : ""}>${MOTOR_INJECT[k]}</label></td>`).join("") + `</tr>`).join("")}</table>
        <p style="color:#9aa1a9;margin:10px 0 0">Command clamping: drag a motor's slider to its end, or type a target beyond its limit.</p>`;
    }
    panel.addEventListener("change", e => {
      const t = e.target;
      if (t.dataset.i) sim.setInject(t.dataset.i, t.checked);
      if (t.dataset.hw) sim.setMotorInject(+t.dataset.hw, "hw", +t.value);
      if (t.dataset.m) { const [i, k] = t.dataset.m.split(":"); sim.setMotorInject(+i, k, t.checked); }
    });
    function renderLog() {
      const L = sim.log().slice(-60).reverse();
      panel.innerHTML = `<h4>Simulated commands (newest first)</h4><div class="simlog">` + (L.map(e =>
        `<div class="${e.result && e.result.error ? "e" : ""}">${e.t.toLocaleTimeString()} ${e.path} ${e.body ? JSON.stringify(e.body) : ""}${e.result && e.result.error ? ` → ${e.result.status || ""} ${e.result.error}` : ""}</div>`).join("") || "<div>No commands yet.</div>") + `</div>`;
    }
    sim.onLog(() => { bar.querySelector(".n").textContent = sim.log().length; if (open === "log") renderLog(); });
    syncSc();
  }
  window.EmioHarness = { init };
})();
