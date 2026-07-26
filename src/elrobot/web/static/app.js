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
    renderRecord(m.record);
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

// Calibrate / Record are independent toggles, not tabs: the cameras and the
// 3D view are always on screen, and these panels open below them on demand.
const deck = document.querySelector(".deck");
document.querySelectorAll(".toggles button[data-panel]").forEach((btn) => {
  btn.addEventListener("click", () => {
    const panel = document.getElementById(`panel-${btn.dataset.panel}`);
    const open = !panel.classList.contains("active");
    panel.classList.toggle("active", open);
    btn.setAttribute("aria-pressed", String(open));
    // data-open drives the "Toggle Calibrate or Record above" hint
    deck.toggleAttribute("data-open",
      deck.querySelectorAll(".panel.active").length > 0);
  });
});

// ── calibration wizard ─────────────────────────────────────────────────
// Driver must be stopped before this tab does anything real - /api/calib/*
// enforces that server-side (409 while driver_alive()); this UI just
// reflects whatever state the session is actually in.

const ARM = Array.from({ length: 7 }, (_, i) =>
  `rev_motor_${String(i + 1).padStart(2, "0")}`);
const calibEl = {
  status: document.getElementById("calib-status"),
  gate: document.getElementById("calib-gate"),
  start: document.getElementById("calib-start"),
  sweepBegin: document.getElementById("calib-sweep-begin"),
  sweepEnd: document.getElementById("calib-sweep-end"),
  eepromOpen: document.getElementById("calib-eeprom-open"),
  finish: document.getElementById("calib-finish"),
  signsWrap: document.getElementById("calib-signs"),
  signRows: document.getElementById("calib-sign-rows"),
  dialog: document.getElementById("calib-eeprom-dialog"),
  eepromInput: document.getElementById("calib-eeprom-input"),
  eepromCancel: document.getElementById("calib-eeprom-cancel"),
  eepromConfirm: document.getElementById("calib-eeprom-confirm"),
};

async function calibApi(path, body) {
  const r = await fetch(path, body === undefined ? { method: "POST" } : {
    method: "POST", headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.detail || `${path} failed (${r.status})`);
  return data;
}

function renderCalib(snap) {
  calibEl.status.textContent = snap.state;
  const en = {
    start: snap.state === "idle" || snap.state === "done",
    sweepBegin: snap.state === "preflight" || snap.state === "eeprom_done",
    sweepEnd: snap.state === "sweeping" || snap.state === "fullturn",
    eepromOpen: snap.state === "gate",
    finish: snap.state === "signs",
  };
  calibEl.start.disabled = !en.start;
  calibEl.sweepBegin.disabled = !en.sweepBegin;
  calibEl.sweepEnd.disabled = !en.sweepEnd;
  calibEl.eepromOpen.disabled = !en.eepromOpen;
  calibEl.finish.disabled = !en.finish;

  calibEl.gate.textContent = (snap.gate || [])
    .map((g) => `${g.name}  ${(g.span_pct * 100).toFixed(0)}%  ${g.ok ? "ok" : "SUSPECT"}`)
    .join("\n");

  const showSigns = ["eeprom_done", "fullturn", "signs"].includes(snap.state);
  calibEl.signsWrap.style.display = showSigns ? "block" : "none";
  if (showSigns) {
    calibEl.signRows.innerHTML = "";
    for (const n of ARM) {
      const sign = (snap.signs || {})[n] ?? 1;
      const row = document.createElement("button");
      row.className = "sign-row" + (sign < 0 ? " flipped" : "");
      row.textContent = `${n}: ${sign > 0 ? "+" : "-"}`;
      row.addEventListener("click", async () => {
        await calibApi("/api/calib/sign", { joint: n, flip: true });
        refreshCalib();
      });
      calibEl.signRows.appendChild(row);
    }
  }
}

async function refreshCalib() {
  const r = await fetch("/api/calib/state");
  renderCalib(await r.json());
}

calibEl.start.addEventListener("click", () =>
  calibApi("/api/calib/start").then(renderCalib).catch(alert));
calibEl.sweepBegin.addEventListener("click", () =>
  calibApi("/api/calib/sweep/begin").then(renderCalib).catch(alert));
calibEl.sweepEnd.addEventListener("click", () =>
  calibApi("/api/calib/sweep/end").then(renderCalib).catch(alert));
calibEl.finish.addEventListener("click", () =>
  calibApi("/api/calib/finish").then(renderCalib).catch(alert));

calibEl.eepromOpen.addEventListener("click", () => {
  calibEl.eepromInput.value = "";
  calibEl.eepromConfirm.disabled = true;
  calibEl.dialog.showModal();
});
calibEl.eepromInput.addEventListener("input", () => {
  calibEl.eepromConfirm.disabled = calibEl.eepromInput.value !== "ERASE";
});
calibEl.eepromCancel.addEventListener("click", () => calibEl.dialog.close());
calibEl.eepromConfirm.addEventListener("click", async () => {
  try {
    const snap = await calibApi("/api/calib/eeprom", { confirm: calibEl.eepromInput.value });
    renderCalib(snap);
    calibEl.dialog.close();
  } catch (e) {
    alert(e.message);
  }
});

refreshCalib();
setInterval(refreshCalib, 2000);

// ── record panel ───────────────────────────────────────────────────────
// Status comes from the WS "record" field (relayed from /record/status by
// the backend); buttons just POST /api/record and let the next WS tick
// reconcile the UI - no separate polling needed.

const recordEl = {
  count: document.getElementById("record-count"),
  frames: document.getElementById("record-frames"),
  start: document.getElementById("record-start"),
  stop: document.getElementById("record-stop"),
  discard: document.getElementById("record-discard"),
};

function renderRecord(status) {
  const recording = !!(status && status.recording);
  recordEl.count.textContent = status ? status.episodes : "0";
  recordEl.frames.textContent = recording ? `recording - ${status.frames} frames` : "";
  recordEl.start.disabled = recording;
  recordEl.stop.disabled = !recording;
  recordEl.discard.disabled = !recording;
}
renderRecord(null);

function recordCmd(cmd) {
  fetch("/api/record", { method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ cmd }) }).catch(() => {});
}
recordEl.start.addEventListener("click", () => recordCmd("start"));
recordEl.stop.addEventListener("click", () => recordCmd("stop"));
recordEl.discard.addEventListener("click", () => recordCmd("discard"));

// ── camera polling ────────────────────────────────────────────────────
// Not <img src="/cam/x"> (multipart/x-mixed-replace): recent Chromium
// versions unreliably paint that inside <img> - confirmed live (correct
// headers/framing, real data transferring, image never visually updating).
// fetch() + Blob + createObjectURL works the same in every browser.

function pollCamera(name) {
  const img = document.getElementById(`cam-${name}`);
  let lastUrl = null;
  async function tick() {
    try {
      const blob = await fetch(`/cam/${name}/frame`).then((r) => r.blob());
      const url = URL.createObjectURL(blob);
      img.src = url;
      if (lastUrl) URL.revokeObjectURL(lastUrl);   // don't leak object URLs
      lastUrl = url;
    } catch {
      // transient fetch failure - next tick retries, no need to surface it
    }
  }
  tick();
  setInterval(tick, 1000 / 15);
}
pollCamera("wrist");
pollCamera("ext");
