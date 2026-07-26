import { makeScene } from "/static/scene.js";

// Declared up here, not next to the replay code at the bottom: the live
// camera pollers and the WS handler both read replay.active, and
// pollCamera() runs its first tick immediately at module load - a `const`
// further down would still be in the temporal dead zone at that point and
// throw ReferenceError.
const replay = {
  active: false,          // read by the WS handler and the camera pollers
  states: null,           // whole trajectory, fetched once per episode
  names: [],
  fps: 30,
  frame: 0,
  timer: null,
  episode: null,
  urls: { wrist: null, ext: null },
};

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
  });
  rows[n] = { inp, out };
});

// Publish the current setpoint CONTINUOUSLY while web control is on, not
// only when a slider changes. Both consumers need a dense stream:
//   - elrobot_driver's deadman freezes after 200 ms of /joint_command
//     silence, so an edge-triggered stream makes the arm latch on every
//     pause between slider nudges;
//   - episode_recorder drops any frame whose "action" is >500 ms stale, so
//     an edge-triggered stream records almost nothing (measured: a test
//     episode kept 56 frames, then skipped every frame for 5 s while
//     "action stale" climbed 0.5 -> 4.6 s).
// ik_node does exactly this on a timer; the cockpit now matches it.
// Deliberately CLIENT-side: if this tab dies the messages stop instantly
// and the driver's deadman freezes the arm, which is the stop guarantee we
// already rely on. A server-side republisher would keep commanding for as
// long as it took the socket to time out.
const CMD_HZ = 25;
setInterval(() => {
  if (!document.body.hasAttribute("data-control")) return;
  if (!isOwner) return;              // server drops these; don't spam them
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({
    type: "cmd",
    positions: Object.fromEntries(NAMES.map((k) => [k, +rows[k].inp.value])),
  }));
}, 1000 / CMD_HZ);

let ws;
let isOwner = true;      // server decides; assume yes until told otherwise

// Transient message in the banner, held against the 30 Hz state stream that
// would otherwise overwrite it on the very next tick.
let bannerHoldUntil = 0;
function flashBanner(msg, ms = 5000) {
  const b = document.getElementById("banner");
  b.textContent = msg;
  b.classList.add("show");
  bannerHoldUntil = Date.now() + ms;
}
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type !== "state") return;
    // While replaying, the 3D view shows the RECORDED pose, not the live
    // arm - otherwise the live stream fights the playback every 33 ms.
    if (!replay.active) scene.setJoints(m.joints);

    // Only the first-connected cockpit may command; the server drops cmd
    // messages from the rest. Without showing that, a second tab looked
    // fully in control while every command it sent was silently discarded.
    isOwner = m.is_owner !== false;
    const banner = document.getElementById("banner");
    if (Date.now() < bannerHoldUntil) {
      // a flashed message is up; don't let the 30 Hz stream wipe it
    } else if (!isOwner) {
      banner.textContent = `Monitor only — another cockpit tab has control `
        + `(${m.clients} connected). Close the other tab to take over.`;
      banner.classList.add("show");
      if (document.body.hasAttribute("data-control")) setControlUi(false);
    } else if (m.control_on && m.commanders > 0) {
      banner.textContent = "Another commander is also live (phone/jog) — web "
        + "control is NOT disabled, both command streams are being sent";
      banner.classList.add("show");
    } else {
      banner.classList.remove("show");
    }

    setPill("driver", m.driver_alive, m.driver_alive ? "driver live" : "driver down");
    setPill("age", m.age_s < 0.5, `state ${m.age_s < 9 ? (m.age_s*1000)|0 : "----"} ms`);
    if (!document.body.hasAttribute("data-control"))    // monitor: track
      for (const n of NAMES) if (n in m.joints) {
        rows[n].inp.value = m.joints[n];
        rows[n].out.textContent = m.joints[n].toFixed(2);
      }
    renderRecord(m.record);
    renderPhys(m.replay);        // live arm/seek/play progress
  };
  ws.onclose = () => { setControlUi(false); setTimeout(connect, 1000); };
}
connect();

// Local UI state only - does not touch the server. Used when the server
// tells us we are not the owner, and on socket close.
function setControlUi(on) {
  // Coerce: toggleAttribute(name, undefined) FLIPS the attribute rather than
  // clearing it (an omitted force argument means "toggle"). Passing a
  // rejected response's missing control_on therefore turned the switch ON
  // while the server had refused - the UI claimed control it did not have,
  // and the 25 Hz sender started firing commands that were being dropped.
  const want = !!on;
  document.body.toggleAttribute("data-control", want);
  master.toggleAttribute("data-on", want);
  master.setAttribute("aria-checked", String(want));
}

async function setControl(on) {
  if (on && !isOwner) return;        // server would drop our commands anyway
  const res = await fetch("/api/control", { method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ on }) });
  const r = await res.json().catch(() => ({}));
  if (!res.ok) {
    // e.g. 409 while physical replay is armed - say so instead of silently
    // leaving a switch that looks on but commands nothing.
    flashBanner(r.detail || "web control was refused");
    setControlUi(false);
    return;
  }
  setControlUi(r.control_on);
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
  backup: document.getElementById("calib-backup"),
  dialogBackup: document.getElementById("calib-eeprom-backup"),
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

// State order mirrors m1a_calibrate: the EEPROM write happens FIRST (from
// "preflight"), and only then are ranges swept - see calib.py's docstring
// for why sweeping before the homing write would produce a wrong table.
const CALIB_HINT = {
  idle: "Driver must be stopped. Rest the arm low - torque goes off.",
  preflight: "Park the arm relaxed, no joint near a hard stop, then write EEPROM.",
  homed: "Homed. Now sweep every joint through its full range.",
  sweeping: "Sweeping - move every joint end to end, then End sweep.",
  gate: "Check the spans below, then sweep joints 05 and 07 individually.",
  fullturn: "Sweeping a full-turn joint - move it to both stops, then End sweep.",
  signs: "Check each joint's sign, then Finish.",
  done: "Table written. Verify with a tape measure in a clearly bent pose.",
};

function renderCalib(snap) {
  if (snap.error) {
    calibEl.status.textContent = snap.error;
    calibEl.status.classList.add("err");
  } else {
    calibEl.status.classList.remove("err");
    calibEl.status.textContent =
      `${snap.state} - ${CALIB_HINT[snap.state] ?? ""}`;
  }
  // The snapshot taken at preflight is the ONLY way back from the EEPROM
  // write, so it is shown continuously and again inside the confirm dialog -
  // the operator should never type ERASE without seeing where the undo lives.
  const b = snap.backup;
  calibEl.backup.style.display = b ? "block" : "none";
  calibEl.backup.textContent = b ? `backup: ${b}  (restore: pixi run calib-restore)` : "";
  calibEl.dialogBackup.textContent = b
    ? `Recoverable: ${b} — pixi run calib-restore puts these servos back.`
    : "No backup recorded — do not proceed.";

  const en = {
    start: snap.state === "idle" || snap.state === "done",
    sweepBegin: snap.state === "homed" || snap.state === "gate",
    sweepEnd: snap.state === "sweeping" || snap.state === "fullturn",
    eepromOpen: snap.state === "preflight",
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

  const showSigns = snap.state === "signs";
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

// Show failures inline in the status line rather than a browser alert().
// The common one is a legitimate 409: the wizard refuses to open the serial
// port while the driver is running (single-owner rule), and the operator
// needs to read that, not dismiss a popup.
function calibError(e) {
  calibEl.status.textContent = e.message;
  calibEl.status.classList.add("err");
  setTimeout(refreshCalib, 4000);   // fall back to real state after reading
}

function calibAction(path) {
  calibEl.status.classList.remove("err");
  calibApi(path).then(renderCalib).catch(calibError);
}

calibEl.start.addEventListener("click", () => calibAction("/api/calib/start"));
calibEl.sweepBegin.addEventListener("click",
  () => calibAction("/api/calib/sweep/begin"));
calibEl.sweepEnd.addEventListener("click",
  () => calibAction("/api/calib/sweep/end"));
calibEl.finish.addEventListener("click", () => calibAction("/api/calib/finish"));
document.getElementById("calib-abort").addEventListener("click",
  () => calibAction("/api/calib/abort"));   // always available: frees the port

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
    calibEl.dialog.close();
    calibError(e);
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
  // status is null until a /record/status message arrives, i.e. until the
  // recorder node is actually running. Say so plainly and disable the
  // buttons: POST /api/record would happily return 200 (the web backend
  // publishes /record/cmd fine) while nothing at all is listening, which
  // looks exactly like a broken Start button.
  const online = !!status;
  const recording = !!(status && status.recording);
  recordEl.count.textContent = online ? status.episodes : "--";
  recordEl.start.disabled = !online || recording;
  recordEl.stop.disabled = !online || !recording;
  recordEl.discard.disabled = !online || !recording;
  recordEl.frames.textContent = !online
    ? "recorder not running - start it in its own terminal: pixi run record"
    : (recording ? `recording - ${status.frames} frames` : "");
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
    if (replay.active) return;    // panels are showing recorded frames
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

// ── episode replay (visual only) ───────────────────────────────────────
// Plays a recorded episode back through the same 3D view and camera panels
// used for live data. Nothing is ever sent to /joint_command from here - the
// arm does not move. Re-executing an episode on hardware is a separate,
// deliberately-not-built feature (see replay.py).

const rEl = {
  select: document.getElementById("replay-select"),
  play: document.getElementById("replay-play"),
  stop: document.getElementById("replay-stop"),
  refresh: document.getElementById("replay-refresh"),
  scrub: document.getElementById("replay-scrub"),
  status: document.getElementById("replay-status"),
};

async function loadEpisodeList(refresh = false) {
  const r = await fetch(`/api/episodes${refresh ? "?refresh=true" : ""}`)
    .then((x) => x.json()).catch(() => ({ episodes: [] }));
  const eps = r.episodes ?? [];
  rEl.select.innerHTML = eps.length
    ? '<option value="">— pick an episode —</option>'
    : '<option value="">— no episodes recorded —</option>';
  for (const e of eps) {
    const o = document.createElement("option");
    o.value = String(e.index);
    o.textContent = `#${e.index} — ${e.frames} frames (${e.seconds}s)`
      + (e.task ? ` — ${e.task}` : "");
    rEl.select.appendChild(o);
  }
  if (r.error) {
    rEl.status.textContent = r.error;
    rEl.status.classList.add("err");
  } else {
    rEl.status.classList.remove("err");
    rEl.status.textContent = eps.length
      ? "Pick an episode to watch it back."
      : `nothing recorded yet in ${r.root}`;
  }
}

async function showFrame(n) {
  if (!replay.states) return;
  const i = Math.max(0, Math.min(n, replay.states.length - 1));
  replay.frame = i;
  rEl.scrub.value = String(i);

  // 3D view follows the RECORDED joint positions
  const names = replay.names;
  const pose = {};
  names.forEach((nm, k) => { pose[nm] = replay.states[i][k]; });
  scene.setJoints(pose);
  for (const nm of NAMES) if (nm in pose) {
    rows[nm].inp.value = pose[nm];
    rows[nm].out.textContent = pose[nm].toFixed(2);
  }

  const secs = (i / replay.fps).toFixed(1);
  const total = (replay.states.length / replay.fps).toFixed(1);
  rEl.status.textContent =
    `episode #${replay.episode} — frame ${i + 1}/${replay.states.length}`
    + ` (${secs}s / ${total}s)`;

  // recorded camera frames into the same panels
  for (const cam of ["wrist", "ext"]) {
    const img = document.getElementById(`cam-${cam}`);
    try {
      const blob = await fetch(
        `/api/episodes/${replay.episode}/frame/${i}?cam=${cam}`)
        .then((x) => x.blob());
      const url = URL.createObjectURL(blob);
      img.src = url;
      if (replay.urls[cam]) URL.revokeObjectURL(replay.urls[cam]);
      replay.urls[cam] = url;
    } catch { /* frame unavailable; leave the last one up */ }
  }
}

async function selectEpisode(index) {
  stopReplay();
  if (index === "") {
    replay.states = null;
    replay.active = false;
    rEl.play.disabled = rEl.scrub.disabled = true;
    rEl.status.textContent = "Pick an episode to watch it back.";
    return;
  }
  rEl.status.textContent = "loading…";
  const s = await fetch(`/api/episodes/${index}/states`).then((x) => x.json());
  replay.episode = Number(index);
  replay.states = s.states;
  replay.names = s.names;
  replay.fps = s.fps || 30;
  replay.active = true;          // panels switch to recorded data
  rEl.scrub.max = String(s.states.length - 1);
  rEl.scrub.disabled = false;
  rEl.play.disabled = false;
  await showFrame(0);
}

function stopReplay() {
  if (replay.timer) { clearInterval(replay.timer); replay.timer = null; }
  rEl.play.textContent = "Play";
  rEl.stop.disabled = true;
}

function playReplay() {
  if (!replay.states) return;
  if (replay.timer) { stopReplay(); return; }     // Play doubles as Pause
  rEl.play.textContent = "Pause";
  rEl.stop.disabled = false;
  replay.timer = setInterval(() => {
    if (replay.frame >= replay.states.length - 1) { stopReplay(); return; }
    showFrame(replay.frame + 1);
  }, 1000 / replay.fps);
}

rEl.select.addEventListener("change", (e) => selectEpisode(e.target.value));
rEl.play.addEventListener("click", playReplay);
rEl.scrub.addEventListener("input", (e) => {
  stopReplay();
  showFrame(Number(e.target.value));
});
rEl.refresh.addEventListener("click", () => loadEpisodeList(true));
rEl.stop.addEventListener("click", () => {
  // Leave replay entirely: panels and 3D view go back to live data.
  stopReplay();
  replay.active = false;
  replay.states = null;
  rEl.select.value = "";
  rEl.scrub.disabled = rEl.play.disabled = true;
  rEl.status.textContent = "Live. Pick an episode to watch it back.";
});

loadEpisodeList();

// ── replay ON THE REAL ARM ─────────────────────────────────────────────
// Deliberately more friction than the visual player: arming is a separate
// act, running requires an armed session AND a selected episode, and STOP is
// always live. Everything published still passes through the driver's gates;
// stopping simply ceases publishing and the deadman freezes the arm.

const pEl = {
  arm: document.getElementById("phys-arm"),
  play: document.getElementById("phys-play"),
  stop: document.getElementById("phys-stop"),
  speed: document.getElementById("phys-speed"),
  status: document.getElementById("phys-status"),
};

async function physApi(path, body) {
  const r = await fetch(path, {
    method: "POST", headers: { "content-type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.detail || `${path} failed (${r.status})`);
  return data;
}

function renderPhys(s) {
  if (!s) return;
  const armed = !!s.armed;         // coerced: toggle(name, undefined) flips
  pEl.arm.textContent = armed ? "Disarm" : "Arm";
  pEl.arm.classList.toggle("primary", armed);
  // Web control cannot be taken while replay holds /joint_command; show that
  // on the switch rather than only telling them after they click it.
  master.classList.toggle("blocked", armed);
  master.title = armed ? "disarm replay first" : "";
  const running = s.phase === "seeking" || s.phase === "playing";
  pEl.play.disabled = !armed || running || replay.episode === null;
  pEl.stop.disabled = !running;
  if (s.error) {
    pEl.status.textContent = s.error;
    pEl.status.classList.add("err");
    return;
  }
  pEl.status.classList.remove("err");
  pEl.status.textContent =
    s.phase === "seeking"
      ? "moving to the episode's start pose (driver-limited speed)…"
      : s.phase === "playing"
        ? `running episode #${s.episode} — frame ${s.frame + 1}/${s.total}`
        : s.phase === "done"
          ? `finished episode #${s.episode}`
          : (armed ? "armed — the arm will move when you press Run"
                     : "disarmed");
}

pEl.arm.addEventListener("click", async () => {
  const armed = pEl.arm.textContent === "Disarm";
  try {
    renderPhys(await physApi("/api/replay/arm", { on: !armed }));
  } catch (e) {
    pEl.status.textContent = e.message;
    pEl.status.classList.add("err");
  }
});

pEl.play.addEventListener("click", async () => {
  if (replay.episode === null) return;
  try {
    renderPhys(await physApi("/api/replay/play", {
      episode: replay.episode, speed: Number(pEl.speed.value),
    }));
  } catch (e) {
    pEl.status.textContent = e.message;
    pEl.status.classList.add("err");
  }
});

pEl.stop.addEventListener("click", async () => {
  try { renderPhys(await physApi("/api/replay/stop")); } catch { /* ignore */ }
});
