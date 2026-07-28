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
  const id = `joint-${n}`;
  row.innerHTML = `<label for="${id}">${n}</label>
    <input id="${id}" type="range" min="${lo}" max="${hi}" step="0.005" value="0">
    <output for="${id}">0.00</output>`;
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
    renderCollection(m.collection, m.record);
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

// Calibration and replay are mutually exclusive operator workflows.
const deckTabs = [...document.querySelectorAll(".toggles [role=tab]")];
function selectDeckTab(selected) {
  for (const tab of deckTabs) {
    const active = tab === selected;
    tab.setAttribute("aria-selected", String(active));
    tab.tabIndex = active ? 0 : -1;
    document.getElementById(tab.getAttribute("aria-controls")).hidden = !active;
  }
}
deckTabs.forEach((tab, i) => {
  tab.addEventListener("click", () => selectDeckTab(tab));
  tab.addEventListener("keydown", (e) => {
    const step = e.key === "ArrowRight" ? 1 : e.key === "ArrowLeft" ? -1 : 0;
    if (!step) return;
    e.preventDefault();
    const next = deckTabs[(i + step + deckTabs.length) % deckTabs.length];
    selectDeckTab(next);
    next.focus();
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

// ── shared JSON helper ─────────────────────────────────────────────────

async function request(path, options = {}) {
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok)
    throw new Error(data.detail || `${path} failed (${response.status})`);
  return data;
}

function jsonRequest(path, method, body) {
  return request(path, {
    method,
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });
}

// ── top-level Teleop / Curate modes ────────────────────────────────────

const workspace = document.getElementById("workspace");
const modeTabs = [
  document.getElementById("mode-teleop"),
  document.getElementById("mode-curate"),
];

async function selectMode(mode) {
  await safeStopPhysical();
  stopReplay();
  replay.active = false;
  replay.states = null;
  workspace.dataset.mode = mode;
  for (const tab of modeTabs) {
    const active = tab.id === `mode-${mode}`;
    tab.setAttribute("aria-selected", String(active));
    tab.tabIndex = active ? 0 : -1;
  }
  if (mode === "curate") {
    if (document.body.hasAttribute("data-control")) await setControl(false);
    await refreshCurate();
  }
}

modeTabs.forEach((tab, index) => {
  tab.addEventListener("click", () => selectMode(tab.id.replace("mode-", "")));
  tab.addEventListener("keydown", (event) => {
    const step = event.key === "ArrowRight" ? 1
      : event.key === "ArrowLeft" ? -1 : 0;
    if (!step) return;
    event.preventDefault();
    const next = modeTabs[(index + step + modeTabs.length) % modeTabs.length];
    selectMode(next.id.replace("mode-", ""));
    next.focus();
  });
});

// ── task library and managed collection ───────────────────────────────

let tasks = [];
let collectionSnapshot = { state: "idle" };
const collectionEl = {
  task: document.getElementById("task-select"),
  taskEdit: document.getElementById("task-edit"),
  sessionName: document.getElementById("session-name"),
  sessionStart: document.getElementById("session-start"),
  episodeStart: document.getElementById("episode-start"),
  episodeStop: document.getElementById("episode-stop"),
  episodeDiscard: document.getElementById("episode-discard"),
  sessionFinish: document.getElementById("session-finish"),
  taskCreate: document.getElementById("task-create-open"),
  state: document.getElementById("collection-state"),
  status: document.getElementById("collection-status"),
};

function taskById(id) {
  return tasks.find((task) => task.id === id);
}

function fillTaskSelect(select, value, includeArchived = false) {
  select.innerHTML = "";
  for (const task of tasks) {
    if (task.archived && !includeArchived && task.id !== value) continue;
    const option = document.createElement("option");
    option.value = task.id;
    option.textContent = task.name + (task.archived ? " (archived)" : "");
    select.appendChild(option);
  }
  if (value && [...select.options].some((option) => option.value === value))
    select.value = value;
}

async function refreshTasks() {
  const current = collectionEl.task.value;
  tasks = (await request("/api/tasks")).tasks ?? [];
  fillTaskSelect(collectionEl.task, current || collectionSnapshot.task_id);
  collectionEl.taskEdit.disabled = !collectionEl.task.value
    || collectionSnapshot.state === "recording";
}

function renderCollection(snapshot, recorder) {
  if (!snapshot) return;
  collectionSnapshot = snapshot;
  const ready = snapshot.state === "ready";
  const recording = snapshot.state === "recording";
  collectionEl.state.textContent = snapshot.state;
  collectionEl.sessionStart.disabled = snapshot.state !== "idle"
    || !collectionEl.task.value;
  collectionEl.episodeStart.disabled = !ready;
  collectionEl.episodeStop.disabled = !recording;
  collectionEl.episodeDiscard.disabled = !recording;
  collectionEl.sessionFinish.disabled = !ready;
  collectionEl.task.disabled = recording;
  collectionEl.taskEdit.disabled = recording || !collectionEl.task.value;
  collectionEl.taskCreate.disabled = recording;
  collectionEl.sessionName.disabled = snapshot.state !== "idle";
  const task = taskById(snapshot.task_id);
  const frameText = recording && recorder
    ? ` · ${recorder.frames ?? 0} frames accepted` : "";
  collectionEl.status.textContent = snapshot.error
    || (recording
      ? `Recording ${task?.name ?? "episode"}${frameText}`
      : ready
        ? "Session ready. Choose a task and record the next episode."
        : snapshot.state === "finalizing"
          ? "Finalizing and validating the raw dataset…"
          : "Choose a task, then start a collection session.");
  collectionEl.status.classList.toggle("err", !!snapshot.error);
}

async function collectionAction(path, body) {
  collectionEl.status.classList.remove("err");
  try {
    const snapshot = await jsonRequest(path, "POST", body);
    renderCollection(snapshot);
    return snapshot;
  } catch (error) {
    collectionEl.status.textContent = error.message;
    collectionEl.status.classList.add("err");
    return null;
  }
}

collectionEl.sessionStart.addEventListener("click", () => collectionAction(
  "/api/collection/session/start",
  { task_id: collectionEl.task.value, name: collectionEl.sessionName.value },
));
collectionEl.episodeStart.addEventListener("click", () => collectionAction(
  "/api/collection/episode/start", { task_id: collectionEl.task.value },
));
collectionEl.episodeStop.addEventListener("click", () => collectionAction(
  "/api/collection/episode/stop",
));
collectionEl.episodeDiscard.addEventListener("click", () => collectionAction(
  "/api/collection/episode/discard",
));
collectionEl.sessionFinish.addEventListener("click", async () => {
  const done = await collectionAction("/api/collection/session/finish");
  if (done) {
    collectionEl.sessionName.value = "";
    refreshCurate();
  }
});
collectionEl.task.addEventListener("change", () => renderCollection(
  collectionSnapshot,
));

const taskDialog = {
  root: document.getElementById("task-dialog"),
  title: document.getElementById("task-dialog-title"),
  name: document.getElementById("task-name"),
  instruction: document.getElementById("task-instruction"),
  status: document.getElementById("task-dialog-status"),
  archive: document.getElementById("task-archive"),
  save: document.getElementById("task-save"),
  editing: null,
};

function openTaskDialog(task = null) {
  taskDialog.editing = task;
  taskDialog.title.textContent = task ? "Edit task" : "Create task";
  taskDialog.name.value = task?.name ?? "";
  taskDialog.instruction.value = task?.instruction ?? "";
  taskDialog.status.textContent = "";
  taskDialog.archive.hidden = !task;
  taskDialog.archive.textContent = task?.archived ? "Restore" : "Archive";
  taskDialog.root.showModal();
  taskDialog.name.focus();
}

document.getElementById("task-create-open").addEventListener(
  "click", () => openTaskDialog());
collectionEl.taskEdit.addEventListener(
  "click", () => openTaskDialog(taskById(collectionEl.task.value)));
taskDialog.save.addEventListener("click", async () => {
  try {
    const body = {
      name: taskDialog.name.value,
      instruction: taskDialog.instruction.value,
    };
    if (taskDialog.editing)
      await jsonRequest(`/api/tasks/${taskDialog.editing.id}`, "PATCH", body);
    else
      await jsonRequest("/api/tasks", "POST", body);
    taskDialog.root.close();
    await refreshTasks();
    await refreshCurate();
  } catch (error) {
    taskDialog.status.textContent = error.message;
    taskDialog.status.classList.add("err");
  }
});
taskDialog.archive.addEventListener("click", async () => {
  if (!taskDialog.editing) return;
  try {
    await jsonRequest(`/api/tasks/${taskDialog.editing.id}`, "PATCH", {
      archived: !taskDialog.editing.archived,
    });
    taskDialog.root.close();
    await refreshTasks();
    await refreshCurate();
  } catch (error) {
    taskDialog.status.textContent = error.message;
    taskDialog.status.classList.add("err");
  }
});

async function refreshRecoveries() {
  const list = document.getElementById("recovery-list");
  try {
    const sessions = (await request("/api/collection/recovery")).sessions ?? [];
    list.innerHTML = sessions.length ? "" : "No interrupted sessions.";
    for (const session of sessions) {
      const item = document.createElement("div");
      item.className = "recovery-item";
      const label = document.createElement("p");
      label.textContent = session.name || session.id;
      const actions = document.createElement("div");
      actions.className = "button-row";
      for (const [text, action] of [
        ["Recover & finish", "finish"],
        ["Archive incomplete", "archive"],
      ]) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "compact";
        button.textContent = text;
        button.addEventListener("click", async () => {
          try {
            await jsonRequest(
              `/api/collection/recovery/${session.id}/${action}`, "POST");
            await refreshRecoveries();
            await refreshCurate();
          } catch (error) {
            list.textContent = error.message;
            list.classList.add("err");
          }
        });
        actions.appendChild(button);
      }
      item.append(label, actions);
      list.appendChild(item);
    }
  } catch (error) {
    list.textContent = error.message;
    list.classList.add("err");
  }
}
document.getElementById("recovery-refresh").addEventListener(
  "click", refreshRecoveries);

// ── camera polling ────────────────────────────────────────────────────
// Live fetches pause while a recorded take owns the shared camera panels.

function pollCamera(name) {
  const img = document.getElementById(`cam-${name}`);
  let lastUrl = null;
  let busy = false;
  async function tick() {
    if (busy || replay.active || document.hidden) return;
    busy = true;
    try {
      const blob = await fetch(`/cam/${name}/frame`).then((r) => r.blob());
      const url = URL.createObjectURL(blob);
      img.src = url;
      if (lastUrl) URL.revokeObjectURL(lastUrl);
      lastUrl = url;
    } catch {
      // A transient camera miss is retried on the next tick.
    } finally {
      busy = false;
    }
  }
  tick();
  setInterval(tick, 1000 / 15);
}
pollCamera("wrist");
pollCamera("ext");

// ── curation browser and visual replay ─────────────────────────────────

let curateSessions = [];
let curateEpisodes = [];
let curateTaskId = null;
let selectedEpisode = null;
let selectionToken = 0;

const curateEl = {
  taskList: document.getElementById("curate-task-list"),
  episodeList: document.getElementById("curate-episode-list"),
  title: document.getElementById("curate-title"),
  review: document.getElementById("curate-review-state"),
  empty: document.getElementById("curate-empty"),
  controls: document.getElementById("curate-controls"),
  meta: document.getElementById("curate-meta"),
  task: document.getElementById("curate-task"),
  trimStart: document.getElementById("curate-trim-start"),
  trimEnd: document.getElementById("curate-trim-end"),
  raw: document.getElementById("curate-view-raw"),
  notes: document.getElementById("curate-notes"),
};

const rEl = {
  play: document.getElementById("replay-play"),
  stop: document.getElementById("replay-stop"),
  scrub: document.getElementById("replay-scrub"),
  status: document.getElementById("replay-status"),
};

function episodeKey(episode) {
  return `${episode.session_id}:${episode.source_index}`;
}

function effectiveTaskId(episode) {
  return episode.task_id || episode.source_task_id;
}

function renderTaskGroups() {
  curateEl.taskList.innerHTML = "";
  const groups = [
    { id: null, name: "All", episodes: curateEpisodes.length },
    ...tasks.map((task) => ({
      id: task.id,
      name: task.name,
      episodes: curateEpisodes.filter(
        (episode) => effectiveTaskId(episode) === task.id).length,
    })).filter((group) => group.episodes > 0),
  ];
  for (const group of groups) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "task-group";
    button.setAttribute("aria-pressed", String(curateTaskId === group.id));
    button.textContent = `${group.name} · ${group.episodes}`;
    button.addEventListener("click", () => {
      curateTaskId = group.id;
      renderTaskGroups();
      renderEpisodeList();
    });
    curateEl.taskList.appendChild(button);
  }
}

function renderEpisodeList() {
  const visible = curateEpisodes.filter(
    (episode) => !curateTaskId
      || effectiveTaskId(episode) === curateTaskId);
  curateEl.episodeList.innerHTML = "";
  if (!visible.length) {
    curateEl.episodeList.textContent = curateSessions.length
      ? "No episodes in this task group."
      : "No finalized sessions yet.";
    return;
  }
  for (const episode of visible) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "episode-item";
    button.setAttribute(
      "aria-current",
      String(selectedEpisode
        && episodeKey(selectedEpisode) === episodeKey(episode)),
    );
    const task = taskById(effectiveTaskId(episode));
    const title = document.createElement("strong");
    title.textContent = `${episode.session_name || "Session"} · #${episode.source_index}`;
    const details = document.createElement("span");
    details.textContent = `${episode.review} · ${episode.effective_frames} frames`
      + ` · ${task?.name ?? "unknown task"}`;
    button.append(title, details);
    button.addEventListener("click", () => selectCuratedEpisode(episode));
    curateEl.episodeList.appendChild(button);
  }
}

async function refreshCurate() {
  try {
    if (!tasks.length) await refreshTasks();
    curateSessions = (
      await request("/api/curation/sessions")).sessions ?? [];
    const groups = await Promise.all(curateSessions.map((session) =>
      request(`/api/curation/sessions/${session.id}/episodes`)));
    curateEpisodes = groups.flatMap((group) => group.episodes ?? []);
    if (selectedEpisode) {
      selectedEpisode = curateEpisodes.find(
        (episode) => episodeKey(episode) === episodeKey(selectedEpisode),
      ) ?? null;
    }
    renderTaskGroups();
    renderEpisodeList();
    if (selectedEpisode) renderEpisodeEditor();
  } catch (error) {
    curateEl.episodeList.textContent = error.message;
    curateEl.episodeList.classList.add("err");
  }
}

function renderEpisodeEditor() {
  const episode = selectedEpisode;
  curateEl.empty.hidden = !!episode;
  curateEl.controls.hidden = !episode;
  if (!episode) {
    curateEl.title.textContent = "No episode selected";
    curateEl.review.textContent = "—";
    return;
  }
  const task = taskById(effectiveTaskId(episode));
  curateEl.title.textContent =
    `${episode.session_name || "Session"} · #${episode.source_index}`;
  curateEl.review.textContent = episode.review;
  curateEl.meta.textContent =
    `${episode.frames} raw frames · ${episode.effective_frames} curated`
    + ` · ${task?.name ?? "unknown task"}`;
  fillTaskSelect(curateEl.task, effectiveTaskId(episode), true);
  curateEl.trimStart.value = String(episode.start_frame);
  curateEl.trimEnd.value = String(episode.end_frame_exclusive);
  curateEl.trimStart.max = String(episode.frames - 1);
  curateEl.trimEnd.max = String(episode.frames);
  curateEl.notes.value = episode.notes ?? "";
  for (const [id, review] of [
    ["curate-keep", "kept"],
    ["curate-reject", "rejected"],
    ["curate-unreview", "unreviewed"],
  ]) {
    document.getElementById(id).classList.toggle(
      "primary", episode.review === review);
  }
}

function stopReplay() {
  if (replay.timer) {
    clearInterval(replay.timer);
    replay.timer = null;
  }
  rEl.play.textContent = "Play";
  rEl.stop.disabled = !replay.states;
}

async function showFrame(n) {
  if (!replay.states || !selectedEpisode) return;
  const i = Math.max(0, Math.min(n, replay.states.length - 1));
  replay.frame = i;
  rEl.scrub.value = String(i);
  const pose = {};
  replay.names.forEach((name, index) => {
    pose[name] = replay.states[i][index];
  });
  scene.setJoints(pose);
  for (const name of NAMES) if (name in pose) {
    rows[name].inp.value = pose[name];
    rows[name].out.textContent = pose[name].toFixed(2);
  }
  const seconds = (i / replay.fps).toFixed(1);
  const total = (replay.states.length / replay.fps).toFixed(1);
  rEl.status.textContent =
    `frame ${i + 1}/${replay.states.length} · ${seconds}s / ${total}s`;
  const raw = curateEl.raw.checked ? "&raw=true" : "";
  for (const cam of ["wrist", "ext"]) {
    try {
      const response = await fetch(
        `/api/curation/episodes/${selectedEpisode.session_id}`
        + `/${selectedEpisode.source_index}/frame/${i}?cam=${cam}${raw}`,
      );
      if (!response.ok) continue;
      const url = URL.createObjectURL(await response.blob());
      document.getElementById(`cam-${cam}`).src = url;
      if (replay.urls[cam]) URL.revokeObjectURL(replay.urls[cam]);
      replay.urls[cam] = url;
    } catch {
      // Leave the last decoded frame visible.
    }
  }
}

async function loadCuratedReplay() {
  if (!selectedEpisode) return;
  const token = ++selectionToken;
  stopReplay();
  rEl.status.textContent = "Loading episode…";
  const raw = curateEl.raw.checked ? "?raw=true" : "";
  try {
    const states = await request(
      `/api/curation/episodes/${selectedEpisode.session_id}`
      + `/${selectedEpisode.source_index}/states${raw}`,
    );
    if (token !== selectionToken) return;
    replay.episode = {
      session_id: selectedEpisode.session_id,
      source_index: selectedEpisode.source_index,
    };
    replay.states = states.states;
    replay.names = states.names;
    replay.fps = states.fps || 30;
    replay.frame = 0;
    replay.active = true;
    rEl.scrub.max = String(states.states.length - 1);
    rEl.scrub.disabled = false;
    rEl.play.disabled = false;
    rEl.stop.disabled = false;
    await showFrame(0);
  } catch (error) {
    rEl.status.textContent = error.message;
    rEl.status.classList.add("err");
  }
}

async function selectCuratedEpisode(episode) {
  await safeStopPhysical();
  selectedEpisode = episode;
  curateEl.raw.checked = false;
  renderEpisodeList();
  renderEpisodeEditor();
  await loadCuratedReplay();
}

function playReplay() {
  if (!replay.states) return;
  if (replay.timer) {
    stopReplay();
    return;
  }
  rEl.play.textContent = "Pause";
  replay.timer = setInterval(() => {
    if (replay.frame >= replay.states.length - 1) {
      stopReplay();
      showFrame(0);
      return;
    }
    showFrame(replay.frame + 1);
  }, 1000 / replay.fps);
}

rEl.play.addEventListener("click", playReplay);
rEl.stop.addEventListener("click", () => {
  stopReplay();
  showFrame(0);
});
rEl.scrub.addEventListener("input", (event) => {
  stopReplay();
  showFrame(Number(event.target.value));
});
document.getElementById("curate-refresh").addEventListener(
  "click", refreshCurate);

async function patchSelected(patch) {
  if (!selectedEpisode) return;
  const replayChanged = Object.hasOwn(patch, "trim");
  await safeStopPhysical();
  try {
    await jsonRequest(
      `/api/curation/episodes/${selectedEpisode.session_id}`
      + `/${selectedEpisode.source_index}`,
      "PATCH",
      patch,
    );
    await refreshCurate();
    if (replayChanged && selectedEpisode) await loadCuratedReplay();
  } catch (error) {
    rEl.status.textContent = error.message;
    rEl.status.classList.add("err");
  }
}

document.getElementById("curate-keep").addEventListener(
  "click", () => patchSelected({ review: "kept" }));
document.getElementById("curate-reject").addEventListener(
  "click", () => patchSelected({ review: "rejected" }));
document.getElementById("curate-unreview").addEventListener(
  "click", () => patchSelected({ review: "unreviewed" }));
curateEl.task.addEventListener("change", () => {
  const value = curateEl.task.value;
  patchSelected({
    task_id: value === selectedEpisode.source_task_id ? null : value,
  });
});
document.getElementById("curate-trim-apply").addEventListener("click", () => {
  const start = Number(curateEl.trimStart.value);
  const end = Number(curateEl.trimEnd.value);
  patchSelected({
    trim: { start_frame: start, end_frame_exclusive: end },
  });
});
document.getElementById("curate-trim-reset").addEventListener(
  "click", () => patchSelected({ trim: null }));
curateEl.raw.addEventListener("change", async () => {
  await safeStopPhysical();
  await loadCuratedReplay();
});
document.getElementById("curate-notes-save").addEventListener(
  "click", () => patchSelected({ notes: curateEl.notes.value }));

// ── physical replay ────────────────────────────────────────────────────

const pEl = {
  arm: document.getElementById("phys-arm"),
  play: document.getElementById("phys-play"),
  stop: document.getElementById("phys-stop"),
  speed: document.getElementById("phys-speed"),
  status: document.getElementById("phys-status"),
};

function physApi(path, body) {
  return jsonRequest(path, "POST", body);
}

function replayLabel(selection) {
  if (selection && typeof selection === "object")
    return `${selection.session_id}:${selection.source_index}`;
  return String(selection ?? "");
}

function renderPhys(status) {
  if (!status) return;
  const armed = !!status.armed;
  pEl.arm.textContent = armed ? "Disarm" : "Arm";
  pEl.arm.classList.toggle("primary", armed);
  master.classList.toggle("blocked", armed);
  master.title = armed ? "disarm replay first" : "";
  const running = status.phase === "seeking" || status.phase === "playing";
  pEl.play.disabled = !armed || running || !selectedEpisode;
  pEl.stop.disabled = !running;
  if (status.error) {
    pEl.status.textContent = status.error;
    pEl.status.classList.add("err");
    return;
  }
  pEl.status.classList.remove("err");
  pEl.status.textContent = status.phase === "seeking"
    ? "Moving to the selected range start at driver-limited speed…"
    : status.phase === "playing"
      ? `Running ${replayLabel(status.episode)}`
        + ` · frame ${status.frame + 1}/${status.total}`
      : status.phase === "done"
        ? `Finished ${replayLabel(status.episode)}`
        : armed
          ? "Armed — the arm moves when you press Run"
          : "Disarmed";
}

async function safeStopPhysical() {
  if (!pEl) return;
  try {
    await physApi("/api/replay/stop");
    renderPhys(await physApi("/api/replay/arm", { on: false }));
  } catch {
    // The server also disarms on curation changes and last-client exit.
  }
}

pEl.arm.addEventListener("click", async () => {
  const armed = pEl.arm.textContent === "Disarm";
  try {
    renderPhys(await physApi("/api/replay/arm", { on: !armed }));
  } catch (error) {
    pEl.status.textContent = error.message;
    pEl.status.classList.add("err");
  }
});
pEl.play.addEventListener("click", async () => {
  if (!selectedEpisode) return;
  try {
    renderPhys(await physApi("/api/replay/play", {
      session_id: selectedEpisode.session_id,
      episode: selectedEpisode.source_index,
      raw: curateEl.raw.checked,
      speed: Number(pEl.speed.value),
    }));
  } catch (error) {
    pEl.status.textContent = error.message;
    pEl.status.classList.add("err");
  }
});
pEl.stop.addEventListener("click", async () => {
  try {
    renderPhys(await physApi("/api/replay/stop"));
  } catch {
    // STOP remains best effort; the driver deadman is the final freeze gate.
  }
});

// ── immutable LeRobot v3 export dialog ─────────────────────────────────

const exportEl = {
  root: document.getElementById("export-dialog"),
  name: document.getElementById("export-name"),
  tasks: document.getElementById("export-task-list"),
  preview: document.getElementById("export-preview"),
  summary: document.getElementById("export-summary"),
  start: document.getElementById("export-start"),
};
let exportPreview = null;

function selectedExportTasks() {
  return [...exportEl.tasks.querySelectorAll("input:checked")]
    .map((input) => input.value);
}

function renderExportTasks() {
  exportEl.tasks.innerHTML = "";
  const keptByTask = new Map();
  for (const episode of curateEpisodes) {
    if (episode.review !== "kept") continue;
    const taskId = effectiveTaskId(episode);
    keptByTask.set(taskId, (keptByTask.get(taskId) ?? 0) + 1);
  }
  for (const task of tasks) {
    const kept = keptByTask.get(task.id) ?? 0;
    if (!kept) continue;
    const label = document.createElement("label");
    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = task.id;
    input.checked = curateTaskId ? task.id === curateTaskId : true;
    input.addEventListener("change", invalidateExportPreview);
    label.append(input, `${task.name} · ${kept} kept`);
    exportEl.tasks.appendChild(label);
  }
  if (!exportEl.tasks.children.length)
    exportEl.tasks.textContent = "No kept episodes are available.";
}

function invalidateExportPreview() {
  exportPreview = null;
  exportEl.start.disabled = true;
  exportEl.summary.textContent = "Validate the updated selection before export.";
  exportEl.summary.classList.remove("err");
}

document.getElementById("export-open").addEventListener("click", () => {
  renderExportTasks();
  invalidateExportPreview();
  exportEl.root.showModal();
  exportEl.name.focus();
});
exportEl.name.addEventListener("input", invalidateExportPreview);
exportEl.preview.addEventListener("click", async () => {
  try {
    exportPreview = await jsonRequest("/api/exports/preview", "POST", {
      name: exportEl.name.value,
      task_ids: selectedExportTasks(),
    });
    exportEl.summary.textContent =
      `${exportPreview.kept_episodes} episodes · ${exportPreview.frames} frames`
      + ` · ${exportPreview.seconds.toFixed(1)}s`
      + ` → ${exportPreview.name}-v${String(exportPreview.next_version).padStart(3, "0")}`;
    exportEl.summary.classList.remove("err");
    exportEl.start.disabled = false;
  } catch (error) {
    exportEl.summary.textContent = error.message;
    exportEl.summary.classList.add("err");
    exportEl.start.disabled = true;
  }
});

async function pollExport(exportId) {
  const status = await request(`/api/exports/${exportId}`);
  if (status.state === "complete") {
    exportEl.summary.textContent =
      `Export complete · ${status.episodes} episodes · ${status.root}`;
    exportEl.start.disabled = false;
    return;
  }
  if (status.state === "failed") {
    exportEl.summary.textContent = status.error || "Export failed.";
    exportEl.summary.classList.add("err");
    exportEl.start.disabled = false;
    return;
  }
  exportEl.summary.textContent = `${status.state}… video encoding continues in the background`;
  setTimeout(() => pollExport(exportId).catch((error) => {
    exportEl.summary.textContent = error.message;
    exportEl.summary.classList.add("err");
  }), 1000);
}

exportEl.start.addEventListener("click", async () => {
  if (!exportPreview) return;
  exportEl.start.disabled = true;
  try {
    const record = await jsonRequest("/api/exports", "POST", {
      name: exportEl.name.value,
      task_ids: selectedExportTasks(),
    });
    exportEl.summary.textContent = "Queued…";
    await pollExport(record.id);
  } catch (error) {
    exportEl.summary.textContent = error.message;
    exportEl.summary.classList.add("err");
    exportEl.start.disabled = false;
  }
});

// Initial server-backed state.
refreshTasks().then(() => renderCollection(collectionSnapshot));
refreshRecoveries();
