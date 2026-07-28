# Web Cockpit — Operator Guide

A browser cockpit for the Elrobot arm: live 3D model, both camera feeds,
joint sliders that drive the real arm, a guided calibration wizard, and a
task-labelled collection → curation → LeRobot v3 export workflow.
`pixi run web` prints the URL when it starts (normally
`http://localhost:8080`).

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

The cockpit hosts its managed dataset recorder, but the driver and cameras
remain separate processes.

| You want to… | Run these (each in its own terminal) |
|---|---|
| Just look at the 3D model / cameras | `pixi run web` (+ `pixi run cams`) |
| **Drive and collect from the browser** | `pixi run python -m elrobot.nodes.elrobot_driver` + `pixi run cams` + `pixi run web` |
| Drive from the phone, watch in browser | `pixi run m3-arm` + `pixi run web` |
| See camera feeds | add `pixi run cams` (see device note below) |
| Calibrate | `pixi run web` **only** — driver must be stopped |

Do not also run `pixi run record` while using managed collection. The cockpit
detects the independent recorder and refuses to start a session rather than
letting two recorder nodes compete for the same topics and dataset intent.
The standalone command remains available for CLI-only capture.

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

The header switches between two workspaces without creating a second 3D
renderer or another pair of camera pollers:

- **Teleop** — both camera feeds and the 3D model on the left, **Calibrate /
  Collect** tabs in the middle, and joint sliders on the right.
- **Curate** — task groups and episode list on the left, the same camera/3D
  stage in the middle, and reversible review/replay/export controls on the
  right.

The camera/3D stage is always visible. Drag in the 3D view to orbit and scroll
to zoom. The model mirrors the real arm from `/joint_states` whether or not
you are commanding it.

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

## 6. Collection and curation

### Storage and saved tasks

Managed data lives under `data/collections` by default:

```text
data/collections/
├── catalog.json
├── raw/<session-id>/             # immutable after session finish
└── exports/<name>-vNNN/          # immutable curated LeRobot v3 output
```

Set another root for a campaign with:

```bash
COLLECTION_ROOT=/data/elrobot-campaign pixi run web
```

The atomic `catalog.json` stores stable task IDs, session lifecycle, and
reversible curation overlays. It never rewrites finalized raw datasets.

In **Teleop → Collect**, create a saved task with a concise display name and
the exact training instruction that should be written into LeRobot frames.
Tasks may be edited later. Archive hides a task from new capture without
breaking historical sessions or completed export manifests; restore makes it
available again.

### Collect a session

1. Choose a saved task and optionally name the session.
2. Press **Start collection**. One session may hold episodes from several
   tasks.
3. Press **Record episode** only when the driver/commander and both camera
   streams are fresh.
4. Press **Stop & keep** to save the take, or **Discard** to clear the
   in-memory take without adding an episode.
5. Between episodes, change **Episode task** if the next demonstration has a
   different instruction.
6. Press **Finish session**. Finalization validates episode counts and only
   then exposes the session in Curate.

Recording encodes both camera streams continuously with two encoder threads,
so **Stop & keep** drains a small remainder rather than launching a large
post-take encoding burst. Frames are skipped when any required stream is
missing or stale; this produces an explicitly shorter take instead of silent
corruption.

If the cockpit or machine stops unexpectedly, open **Interrupted sessions**:

- **Recover & finish** reconciles already-saved episodes against the
  catalog's pending marker, finalizes, validates, and exposes the session.
- **Archive incomplete** hides it without deleting any raw file.

Frames that existed only in the recorder's unfinished memory buffer at the
instant of a hard crash cannot be recovered.

### Curate reversibly

Open **Curate**, choose a task group, then an episode. Review decisions mean:

- **Unreviewed** — not decided; never exported.
- **Keep** — eligible for export if its effective task group is selected.
- **Reject** — retained in raw storage and excluded from export.

The review sidebar can reassign the effective task, add notes, and set one
continuous trim. **Trim end is exclusive**: start `10`, end `80` keeps frames
`10..79`. **Use full take** clears the trim. **View untouched raw episode**
temporarily bypasses the overlay for inspection; it does not undo edits.

Every change updates only `catalog.json`, so keep/reject/unreview, task
assignment, trim, and notes remain reversible after restarts.

### Visual replay

The Curate player drives the existing 3D view and camera panels from the
selected effective range. It never publishes `/joint_command`. Scrubbing,
raw/curated switching, and reaching the end all remain visual-only; after the
last frame, **Play** resets to frame zero so it can immediately be replayed.

### Replaying on the real arm

> **The arm moves on its own, with nobody holding a clutch.** Stand clear,
> keep the driver's terminal within reach, and treat this like the autonomous
> rollout it effectively is.

The player re-publishes the selected range's recorded *action* stream to
`/joint_command`, so every driver gate — velocity clamp, workspace box,
singularity floor, joint limits, grasp latch — applies exactly as it does to
the phone.

1. Stop the phone/slider commander. Replay refuses to arm while **Web
   control** is on, and `/api/control` refuses while replay is armed.
2. **Arm** — a separate deliberate act. It refuses if no driver is running,
   since that is where every safety gate lives.
3. Select the same curated or raw episode used by visual replay. The default
   armed replay speed is **0.6×**, capped at 1.0.
4. **Run on arm.** It first *seeks*: it holds the selected range's opening
   pose and lets the driver's slew limiter walk the arm there rather than
   jumping into a trajectory. Once every joint is within 0.05 rad it streams
   the episode. A 45 s seek timeout reports which joint remains off.
5. **STOP** at any moment. Publishing ceases immediately and the driver's
   deadman freezes the arm within 200 ms.

Replay also stops if the driver disappears. Changing workspace mode, episode,
trim, task assignment, or raw/curated view stops and disarms it before
changing what the selection means.

An episode reproduces only if the world is set up as it was when recorded.
Physical replay is open-loop; nothing watches the cameras to correct a
misplaced object.

### Export LeRobot v3

Only explicitly kept episodes are exportable. Press **Export kept episodes…**,
choose one or more task groups, name the dataset, and **Validate selection**.
The preview reports kept episode count, effective frame/duration total, and
the next immutable version. Starting returns immediately while one supervised
background worker copies and encodes the data.

Completed exports are local and versioned:

```text
data/collections/exports/training-v001/
data/collections/exports/training-v002/
```

Each source episode stays a distinct output episode; trimmed frames are
omitted, indices are regenerated, and the current task instruction is written
into every output frame. `curation-manifest.json` snapshots the catalog
revision, task definitions, source session/index/range, and raw-tree SHA-256.
Output is built under `.inprogress`, reload-validated as LeRobot v3, then
atomically promoted. Existing versions are never overwritten.

Training, policy evaluation, and inference controls are intentionally **not
part of this milestone**. This cockpit now produces the traceable LeRobot v3
dataset those later Python phases will consume.

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
| Start collection reports a recorder conflict | An independent `pixi run record` is active. Stop it before managed collection. |
| Record episode is refused | Driver/commander or one of the camera streams is missing or stale. |
| Session appears under Interrupted sessions | Finalization was interrupted or validation failed; recover and finish it, or archive it without deleting raw files. |
| Export is disabled or refused | No selected task group contains kept episodes, or source schemas/FPS differ. |
| Camera tile: "no signal" | Cam node not running for that topic. |
| Camera tile: black, no text | Wrong `/dev/video*`. Use `WRIST_DEV`/`EXT_DEV`. |
| `Start preflight` → red 409 | Driver still running. Stop it; only one process may own the port. |
| Layout collapses to one column | Viewport under 1200px (often DevTools docked open). Expected. |
| 3D model missing after a URDF change | Reload the page. `/`, `/urdf`, and cockpit static assets send `no-store`. |
| Console: `ColladaLoader … Z-UP` ×20 | Harmless. One per mesh; the loader is correctly handling Z-up assets. |

**Leaving the arm:** stopping the driver leaves torque **ON**, holding
position. `pixi run ticks` releases it — the arm goes limp, so support it
first.
