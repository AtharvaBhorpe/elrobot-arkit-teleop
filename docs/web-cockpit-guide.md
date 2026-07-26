# Web Cockpit — Operator Guide

A browser cockpit for the Elrobot arm: live 3D model, both camera feeds,
joint sliders that drive the real arm, a guided calibration wizard, and
episode recording + replay. Runs at `http://localhost:8080`.

> **The cockpit never touches the serial port.** It is an ordinary ROS 2
> command source, exactly like the phone. Every safety mechanism — velocity
> clamp, workspace box, singularity floor, deadman freeze, grasp latch,
> joint limits — lives in `elrobot_driver` and applies to web commands
> unchanged. The one exception is the calibration wizard, which needs raw
> bus access and therefore **requires the driver to be stopped** (it refuses
> to start otherwise).

> **LAN only, no authentication.** This page can move a real robot. Do not
> port-forward it or expose it beyond your local network.

---

## 1. What to run alongside it

The cockpit displays and relays; it does not host the driver, the cameras,
or the recorder. Each of those is a separate process, and the cockpit tells
you which ones are missing.

| You want to… | Run these (each in its own terminal) |
|---|---|
| Just look at the 3D model / cameras | `pixi run web` (+ `pixi run cams`) |
| **Drive the arm from the browser** | `pixi run python -m elrobot.nodes.elrobot_driver` + `pixi run web` |
| Drive from the phone, watch in browser | `pixi run m3-arm` + `pixi run web` |
| Record episodes | add `pixi run record` |
| See camera feeds | add `pixi run cams` (see device note below) |
| Calibrate | `pixi run web` **only** — driver must be stopped |

**Cameras need the right devices.** `cams` defaults to `/dev/video0` and
`/dev/video2`, which on this machine are *not* the real cameras — they open
successfully and hand back uniformly black frames, which looks exactly like
a broken feed. Pass the real ones:

```bash
WRIST_DEV=/dev/video4 EXT_DEV=/dev/video6 pixi run cams
```

Unsure which is which? `pixi run campick` gives a dropdown with a live
preview. Close it before starting `cams` — each `/dev/video*` is
single-owner.

### The cockpit-only workflow (no phone)

This is the simplest way to drive the arm from the browser, and it avoids
the two-commander warning entirely because no IK node is running:

```bash
# terminal 1 — the only process touching hardware
pixi run python -m elrobot.nodes.elrobot_driver
#   add --no-torque for a dry run: reads and checks everything,
#   never enables torque, never writes a goal

# terminal 2
pixi run web

# terminal 3 (optional)
WRIST_DEV=/dev/video4 EXT_DEV=/dev/video6 pixi run cams
```

Then open `http://localhost:8080`, flip **Web control** on, and move the
sliders.

---

## 2. Reading the header

| Indicator | Meaning |
|---|---|
| `driver live` / `driver down` | Is anything publishing `/joint_states`? Green means the arm's real state is arriving. |
| `state N ms` | Age of the newest joint state. A few ms is healthy; `---- ms` means nothing is arriving. |
| **Web control** switch | Off = monitor mode (read-only). On = the sliders publish commands. |

**Only one cockpit tab may command.** The first websocket to connect owns
control; any other tab is monitor-only and says so in its banner ("another
cockpit tab has control"). They share one backend ROS node, so the
two-commander check below cannot see them — hence the separate rule. Close
the other tab to take over. Control is also released server-side when the
last tab disconnects.

A **red banner** appears when web control is on *and* another commander
(phone `ik_node`, `jog`) is also publishing. It is a warning, not a block:
**both command streams stay live** and the driver applies whichever arrived
last, slew-limited. Turn one off unless you specifically want that.

---

## 3. The layout

Three columns, all on one screen:

- **Left** — both camera feeds side by side, above the live 3D model. Always
  visible. Drag in the 3D view to orbit, scroll to zoom. The model mirrors
  the real arm from `/joint_states` whether or not you are commanding it.
- **Middle** — **Calibrate** and **Record** toggle buttons; their panels open
  here on demand and close again.
- **Right** — the joint rail: 8 sliders with live values.

Camera frames are shown whole (letterboxed if the tile is a different
aspect), never cropped — what you see is exactly what `episode_recorder`
saves, so composing a grasp on screen matches what a trained policy will see.

---

## 4. Driving the arm

1. Confirm the header reads **driver live**.
2. Flip **Web control** on. The sliders **seed from the arm's current pose**,
   so nothing jumps — the arm does not move at the moment you take control.
3. Move a slider. Each change publishes `/joint_command` with all 8 joints.

In monitor mode (switch off) the sliders are dimmed and inert, and they
*track* the real arm — useful as a live readout while the phone drives.

**Stopping is automatic.** Close the tab, navigate away, or lose the
WebSocket and the backend stops publishing immediately; the driver's deadman
latches the current pose within 200 ms. This is the same stop semantics as
releasing the phone clutch.

Slider ranges are ±1.8 rad for the arm joints and 0.0 → 2.0 for the gripper
(0 open, 2 closed). The driver clamps to real URDF limits regardless.

---

## 5. Cameras

Both feeds poll at ~15 fps. A tile showing **"no signal — wrist/ext"** means
no frames are arriving on that topic (cam node not running, or wrong device).

A tile showing a *black image with no text* is different and more subtle:
frames **are** arriving, and they are genuinely black — almost always the
wrong `/dev/video*` device. Fix it with `WRIST_DEV` / `EXT_DEV` above.

---

## 6. Recording

Requires `pixi run record` in its own terminal. If it is not running, the
panel says so and the buttons are disabled — the count reads `--`.

| Button | Effect |
|---|---|
| **Start episode** | Begin capturing. Takes a moment: the dataset/video encoder is created on the first start. |
| **Stop & keep** | Save the episode and increment the count. |
| **Discard** | Stop and throw the episode away (a fumbled attempt, a bad grasp). |

While recording, the panel shows a live frame count. Terminal `ENTER`
control still works exactly as before — the two are interchangeable.

Frames are skipped (with a warning in the recorder's terminal) whenever any
stream is missing or stale, so a dead camera yields a short episode rather
than silently corrupt data. If `action` (i.e. `/joint_command`) is the stale
one, nothing is commanding the arm — start the driver and a commander.

Episodes accumulate in one dataset across runs; the recorder resumes an
existing `--root` rather than starting over.

### Replaying an episode

Below the record controls: pick an episode from the dropdown to watch it back.
**Play/Pause**, a scrubber, and **Stop** (which returns to live). While
replaying, the 3D view follows the recorded joint positions and both camera
panels show the recorded frames instead of the live feeds — **Refresh**
re-reads the dataset to pick up episodes recorded since the page loaded.

This is **visual only**: replay never publishes to `/joint_command`, so the
arm does not move, and it is safe with or without the driver running. It
answers "does this episode actually contain what I think it does?" — the
dataset-QA step before training on it.

### Replaying on the real arm

> **The arm moves on its own, with nobody holding a clutch.** Stand clear,
> keep the driver's terminal within reach, and treat this like the autonomous
> rollout it effectively is.

Under **On the real arm** in the same panel. It re-publishes the episode's
recorded *action* stream to `/joint_command`, so every driver gate — velocity
clamp, workspace box, singularity floor, joint limits, grasp latch — applies
exactly as it does to the phone.

1. Stop the phone/slider commander. Replay refuses to arm while **Web
   control** is on, and `/api/control` refuses while replay is armed: two
   automatic publishers on `/joint_command` have no arbitration between them.
2. **Arm** — a separate deliberate act. Refuses if no driver is running,
   since that is where every safety gate lives.
3. Pick the episode in the dropdown above (the same one the visual player
   uses), set **speed** (capped at 1.0 — never faster than recorded; 0.5 is
   a sane first try).
4. **Run on arm.** It first *seeks*: it holds the episode's opening pose and
   lets the driver's own slew limiter walk the arm there at its configured
   velocity, rather than jumping into the middle of a trajectory. Once every
   joint is within 0.05 rad it streams the episode. If it cannot get there
   within 45 s it gives up and says which joint is still off — usually
   something blocked, or a safety gate holding.
5. **STOP** at any moment. Publishing ceases immediately and the driver's
   deadman freezes the arm within 200 ms — the same stop semantics as
   releasing the phone clutch.

Replay also stops itself if the driver disappears mid-run.

**A caveat worth respecting:** an episode reproduces only if the world is set
up as it was when recorded. The recorded commands are replayed open-loop —
nothing is watching the cameras — so an object in a different place will
simply be missed, and the arm will confidently execute the old trajectory
anyway.

---

## 7. Calibration wizard

> **Read this before touching it.** Step 3 writes homing offsets to servo
> EEPROM. That **invalidates the existing tick↔radian table** and there is no
> partial redo: doing M1a means doing M1b and the verification gate again.
> `calibration/*.json` is hand-measured physical truth. Treat a full
> recalibration as a deliberate session, not something to click through.

**Your current calibration is backed up automatically.** Preflight snapshots
every servo's `Homing_Offset` and min/max position limits *plus* the current
`calibration/*.json` into one file under `calibration/backups/`, before
anything can write. The path is shown under the wizard status and again in
the EEPROM confirm dialog. If a run goes wrong:

```bash
# driver stopped, as always
pixi run calib-restore              # newest snapshot
pixi run calib-backup --list        # see them all
pixi run calib-restore --file calibration/backups/calib-20260726T2140.json
```

> **Restore makes the arm go limp — support it first.** Feetech gates EEPROM
> writes on a `Lock` register that lerobot ties to torque, so the values
> cannot go back while torque is on. Backup is read-only and leaves torque
> alone; only restore has this cost. Restore then reads every register back
> and refuses (leaving your files untouched) if the servos did not take the
> write — a locked EEPROM discards it *and still answers OK*, so a status
> byte is not evidence.

The wizard **refuses to start if it cannot write that snapshot** — no backup,
no destructive write. Restore puts the EEPROM and the json back *together*;
they are not independent, and restoring one alone leaves a table describing
servos that no longer match it. You can also snapshot any time with
`pixi run calib-backup` (driver stopped) — worth doing right now, before you
ever open the wizard.

**The driver must be stopped.** The wizard needs exclusive serial access, so
`Start preflight` returns a `409` (shown inline in red) while the driver is
alive. That refusal is the single-owner rule protecting you, not a fault.

Torque is disabled throughout: **the arm goes limp and will sag.** Rest it
low or support it before starting.

> **Not yet walked on real hardware.** The wizard's logic mirrors
> `m1a_calibrate` / `m1b_reconcile` and is covered by offline tests against a
> stub bus, but nobody has driven it through a real EEPROM write yet — that
> is inherently a deliberate human session. Until someone has, the CLI
> scripts remain the authoritative path, and this is the riskier one.

The flow — note the **EEPROM write comes before the sweep**:

1. **Start preflight** — opens the bus, disables torque. Pass `PORT=` if the
   arm is not on `/dev/ttyACM0`: `PORT=/dev/ttyACM1 pixi run web`.
2. **Park the arm** in a relaxed posture with no joint near a hard stop
   (~20° clearance is plenty; do not measure). Whatever pose it is in becomes
   tick 2047 on every motor.
3. **Write EEPROM…** — the destructive step. Type `ERASE` exactly to enable
   the confirm button. **This must happen before any sweep**:
   `set_half_turn_homings()` redefines what `Present_Position` returns, so
   ranges recorded beforehand would be in a coordinate frame this write
   invalidates, and the derived offsets would be wrong by the homing delta.
4. **Begin sweep** → move every joint through its full range by hand →
   **End sweep**. The gate then lists each joint's span vs. its URDF
   expectation; anything outside ±20% is flagged `SUSPECT` (usually means
   you did not sweep far enough). A joint that never answers now aborts the
   sweep loudly instead of silently recording nothing.
5. **Begin/End sweep** twice more — joints 5 and 7 are near-full-turn, so
   they are swept individually with encoder unwrapping (a plain min/max
   would straddle the 0/4095 wrap and record garbage).
6. **Sign check** — for each joint, move the slider, then move the real joint
   the same way. If the model moved *opposite* the real arm, click that
   joint's chip to flip its sign. Signs are pose-dependent and 2 of 7 were
   recorded backwards on this arm once — this step is the real authority.
7. **Finish** — derives the offsets, writes `calibration/urdf_ticks.json`,
   and reports predicted TCP height/reach plus which joints the pose could
   actually discriminate. Refuses to write a partial table if any joint's
   range is missing.

**Verify with a tape measure.** The FK report is only meaningful in a clearly
bent pose; near neutral, a wrong sign is invisible and the check passes
vacuously.

---

## 8. Troubleshooting

| Symptom | Cause |
|---|---|
| `driver down`, `---- ms`, sliders inert | No driver running. Start the driver (or `m3-arm`). |
| Start episode does nothing / count `--` | `pixi run record` not running. |
| Camera tile: "no signal" | Cam node not running for that topic. |
| Camera tile: black, no text | Wrong `/dev/video*`. Use `WRIST_DEV`/`EXT_DEV`. |
| `Start preflight` → red 409 | Driver still running. Stop it; only one process may own the port. |
| Layout collapses to one column | Viewport under 1200px (often DevTools docked open). Expected. |
| 3D model missing after a URDF change | Hard refresh (Ctrl+Shift+R). `/` and `/urdf` send `no-store`, but `style.css` and the JS come through the static mount and can cache. |
| Console: `ColladaLoader … Z-UP` ×20 | Harmless. One per mesh; the loader is correctly handling Z-up assets. |

**Leaving the arm:** stopping the driver leaves torque **ON**, holding
position. `pixi run ticks` releases it — the arm goes limp, so support it
first.
