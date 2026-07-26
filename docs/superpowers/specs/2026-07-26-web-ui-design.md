# Web UI (LeLab-style cockpit) — Design

**Date:** 2026-07-26 · **Branch:** `web-ui` · **Status:** approved design, pre-implementation

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
browser ── WS /ws (joint states 30 Hz down, slider targets up)
        ── GET /cam/{wrist,ext}  (MJPEG multipart)
        ── GET /  /static/*  /urdf/*          (page, JS, viz URDF + DAE meshes)
        ── POST /api/calib/*  /api/record/*   (wizard steps, record control)
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
  then publish `/joint_command` at slider-change rate, joints by name +
  gripper.
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

## Error handling

- Backend never touches serial in normal mode; calibration endpoints return
  409 if the driver is alive.
- Camera topic silent → placeholder frame in the stream, not a dead socket.
- Slider commands are rejected server-side unless WEB CONTROL is ON
  (the toggle is backend state, not just UI state).

## Testing (joins `pixi run test` + CI)

- FastAPI TestClient: API surface, WS round-trip (states down, command up,
  toggle enforcement), MJPEG endpoint smoke test.
- Calibration step functions against the existing StubBus.
- All ROS-touching tests pin `ROS_DOMAIN_ID=77` (hard rule 2).

## Out of scope (v1)

Auth, HTTPS, training/replay panels (LeLab has them; we don't yet), React
migration, mobile layout polish, multi-arm.
