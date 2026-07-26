import { makeScene } from "/static/scene.js";

const NAMES = ["rev_motor_01","rev_motor_02","rev_motor_03","rev_motor_04",
               "rev_motor_05","rev_motor_06","rev_motor_07","rev_motor_08"];
const LIM = { rev_motor_08: [0.0, 2.0] };      // gripper; arm joints ±1.8 display
const scene = makeScene(document.getElementById("scene3d"));
const rows = {};
const rail = document.getElementById("joints");
NAMES.forEach((n) => {
  const row = document.createElement("div");
  row.className = "joint";
  const [lo, hi] = LIM[n] ?? [-1.8, 1.8];
  row.innerHTML = `<label>${n}</label>
    <input type="range" min="${lo}" max="${hi}" step="0.005" value="0">
    <output>0.00</output>`;
  rail.appendChild(row);
  const inp = row.querySelector("input"), out = row.querySelector("output");
  inp.addEventListener("input", () => {
    out.textContent = (+inp.value).toFixed(2);
    if (document.body.hasAttribute("data-control") && ws.readyState === 1)
      ws.send(JSON.stringify({ type: "cmd",
        positions: Object.fromEntries(NAMES.map((k) =>
          [k, +rows[k].inp.value])) }));
  });
  rows[n] = { inp, out };
});

let ws;
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type !== "state") return;
    scene.setJoints(m.joints);
    setPill("driver", m.driver_alive, m.driver_alive ? "driver live" : "driver down");
    setPill("age", m.age_s < 0.5, `state ${m.age_s < 9 ? (m.age_s*1000)|0 : "----"} ms`);
    document.getElementById("banner").classList.toggle("show",
      m.control_on && m.commanders > 0);
    if (!document.body.hasAttribute("data-control"))    // monitor: track
      for (const n of NAMES) if (n in m.joints) {
        rows[n].inp.value = m.joints[n];
        rows[n].out.textContent = m.joints[n].toFixed(2);
      }
  };
  ws.onclose = () => { setControl(false); setTimeout(connect, 1000); };
}
connect();

async function setControl(on) {
  const r = await fetch("/api/control", { method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ on }) }).then((x) => x.json());
  document.body.toggleAttribute("data-control", r.control_on);
  master.toggleAttribute("data-on", r.control_on);
  master.setAttribute("aria-checked", r.control_on);
  for (const [n, v] of Object.entries(r.seed ?? {}))    // no-jump seeding
    if (rows[n]) { rows[n].inp.value = v; rows[n].out.textContent = v.toFixed(2); }
}
const master = document.getElementById("master");
master.addEventListener("click", () =>
  setControl(!document.body.hasAttribute("data-control")));

function setPill(id, ok, text) {
  const p = document.getElementById(`pill-${id}`);
  p.querySelector(".dot").className = "dot" + (ok ? "" : " warn");
  p.querySelector("span:last-child").textContent = text;
}

document.querySelectorAll('nav[role="tablist"] [role="tab"]').forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll('nav[role="tablist"] [role="tab"]')
      .forEach((b) => b.setAttribute("aria-selected", String(b === btn)));
    document.querySelectorAll(".panel").forEach((p) =>
      p.classList.toggle("active", p.id === `tab-${btn.dataset.tab}`));
  });
});
