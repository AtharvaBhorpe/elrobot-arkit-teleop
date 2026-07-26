# Web UI (LeLab-style cockpit) — Design

**Date:** 2026-07-26 · **Branch:** `web-ui` · **Status:** implemented v1

A single browser page that is the daily cockpit for the arm: joint sliders
that drive the real robot, a live 3D URDF that mirrors `/joint_states`, both
camera feeds, a guided calibration wizard, and episode-recording controls.
Modeled on [huggingface/leLab](https://github.com/huggingface/leLab) but built
on this repo's existing safety architecture instead of raw bus access.

## Decisions (from brainstorming)

| question | decision |
|---|---|
| Role | Daily cockpit first; architecture must allow demo polish + remote (LAN) use later |
| Calibration scope | FULL guided flow (M1a → M1b → signs → FK verify), EEPROM step behind typed confirmation |
| Arbitration | Explicit **WEB CONTROL** takeover switch; coexistence with phone teleop allowed, warned |
| Recording | In v1, via a command topic to the existing recorder |
| Frontend stack | No Node toolchain. Vanilla ES modules + vendored libraries |
| Look | rust-ui.com (shadcn/ui) neutral scheme: light + dark, oklch grayscale tokens, hairline borders, rounded-xl cards; native elements + custom CSS |

## Architecture

```
browser ── WS /ws (state 30 Hz down, slider setpoints 25 Hz up)
        ── GET /cam/{wrist,ext}/frame         (one JPEG, polled at 15 Hz)
        ── GET /cam/{wrist,ext}               (MJPEG stream, external viewers)
        ── GET /  /static/*  /urdf  /meshes/* (page, JS, viz URDF + DAE)
        ── POST /api/calib/*                  (wizard steps)
        ── POST /api/record                   (record start/stop/discard)
        ── GET  /api/episodes[/{i}/...]       (replay: list, states, frames)
        ── POST /api/replay/{arm,play,stop}   (replay ON THE ARM)
             │
   src/elrobot/web/server.py  (FastAPI + uvicorn, pixi env, :8080)
             │  embeds an rclpy node (background thread)
   ┌─────────┴─────────────────────────────┐
   │ normal mode: DDS citizen              │ calibration mode: exclusive
   │  sub /joint_states, /wrist_cam/image, │  serial access; REQUIRES the
   │      /ext_cam/image                   │  driver to be stopped (preflight
   │  pub /joint_command (control ON only) │  enforces port free)
   └───────────────────────────────────────┘
```

- The backend is an ordinary command source, exactly like the phone or jog
  sliders. **All safety stays in the driver** (slew, workspace box, sigma
  floor, deadman, grasp latch, joint limits) and applies to web commands
  unchanged.
- Hard rule 3 (one process per device) holds: the driver owns serial in
  normal mode; the wizard owns it in calibration mode; never both.
- LAN only, no auth in v1. This page commands a robot: **do not port-forward
  it.** (Auth is a v2 concern if remote use ever leaves the LAN.)

## Frontend

- `src/elrobot/web/static/` — `index.html`, ES modules, vendored libs
  (three.js, urdf-loader). **No CDN at runtime, no build step, no
  node_modules.** Vendored files are committed.
- Layout: 3D URDF viewer center-left (orbit controls, existing
  `docs/urdf_Elrobot_viz.urdf` + `data/viz_meshes/*.dae`); joints panel right
  (8 sliders + numeric readouts); tab strip: **Cameras / Calibrate / Record**;
  persistent status bar (driver alive, arbitration state, WS latency, torque
  state).
- rust-ui/shadcn neutral theme (light default + dark variant) via the oklch
  design tokens captured in the approved hero mockup; native elements +
  custom CSS supply all controls — no component library, only three.js +
  urdf-loader are vendored.

## Control & arbitration

- **WEB CONTROL toggle OFF (default):** monitor mode — sliders track
  `/joint_states`, grayed out. The URDF always mirrors the real arm.
- **ON:** sliders seed from the current pose (no jump — the jog lesson),
  then publish `/joint_command` CONTINUOUSLY at 25 Hz, joints by name +
  gripper. Not on slider-change: the driver's deadman freezes after 200 ms of
  silence and `episode_recorder` drops frames whose action is >500 ms stale,
  so an edge-triggered stream both latched the arm between nudges and
  destroyed recordings.
- Exactly ONE websocket may command. `control_on` is shared server state and
  `commanders()` filters by ROS node name, so it cannot see a second cockpit
  tab sharing this node; the first connection owns control and the rest are
  monitor-only. Control is reset server-side when the last client leaves.
- Another commander publishing simultaneously (e.g. phone ik_node) → warning
  banner, both streams stay live (user's explicit choice); the driver's slew
  limiter keeps last-write-wins fights slow rather than violent.
- WS drop / tab close → backend stops publishing at once → driver deadman
  freezes. The web UI inherits the phone's stop semantics.

## Calibration wizard

- `elrobot.calibration` interactive scripts are refactored into callable
  step functions (`steps.py`) shared by the CLI scripts and the wizard —
  behavior of the CLI procedures must not change.
- Wizard preflight: driver stopped + serial port free, else refuse to enter.
- Steps: M1a park/sweep/gate → **EEPROM write behind a typed-confirmation
  dialog** (states that it invalidates the M1b table) → M1b reconcile incl.
  joint-5/7 encoder-unwrap sweeps → per-joint sign verification (3D model as
  reference: move slider, move real joint, confirm/flip) → FK verify gate
  with tape-measure entry.
- Results land in `calibration/*.json` as today. A human clicking through
  typed confirmations satisfies hard rule 1's "explicit human decision".

## Recording

- `episode_recorder` gains a command topic `/record/cmd`
  (start / stop / discard) and publishes status; the web panel sends
  commands and shows episode count + state. Terminal ENTER keeps working.
- The recorder RESUMES an existing `--root` instead of always `create()`ing,
  so a collection campaign spans sessions; and it re-checks stream freshness
  before arming every episode, not just the first.
- `/record/status` is treated as stale after 3 s, so a recorder killed
  mid-episode stops reading as "still recording".

## Replay (added after v1)

- **Visual:** `ReplayLibrary` reads the dataset back; the 3D view follows the
  recorded `observation.state` and the camera panels show recorded frames.
  Never publishes to `/joint_command`.
- **On the real arm:** `PhysicalReplay` re-publishes the recorded `action`
  stream. Gated behind an explicit arm step, mutually exclusive with slider
  control in both directions, refuses without a live driver, caps speed at
  1.0, and seeks the episode's start pose by holding it and letting the
  DRIVER's slew limiter walk the arm there. Stop ceases publishing; the
  deadman freezes the arm.
- A dataset being written by a live recorder has no parquet footer yet;
  that is detected and reported as "quit the recorder", not as corruption.

## Error handling

- Backend never touches serial in normal mode; calibration endpoints return
  409 if the driver is alive.
- Camera topic silent → placeholder frame in the stream, not a dead socket.
- Slider commands are rejected server-side unless WEB CONTROL is ON
  (the toggle is backend state, not just UI state).

## Testing (joins `pixi run test` + CI)

- FastAPI TestClient: API surface, WS round-trip (states down, command up,
  toggle enforcement), camera endpoints, calibration wizard guards, replay
  including every gate on moving the real arm.
- Calibration step functions against the existing StubBus.
- All ROS-touching tests pin `ROS_DOMAIN_ID=77` (hard rule 2).
- Two limits worth knowing: TestClient runs the app on one blocking portal
  and cannot hold two concurrent websockets, so multi-tab arbitration is
  tested through `app.state` helpers; and no part of the cockpit has been
  exercised against real hardware by an automated test.

## Out of scope (v1)

Auth, HTTPS, training panels (LeLab has them; we don't yet), React
migration, mobile layout polish, multi-arm.

**Added after v1** (not in the original scope): episode replay - visual
playback of a recorded episode, and re-execution of one on the real arm
behind arm/seek/stop gates. See docs/web-cockpit-guide.md.
