# Standalone Episode Record, Manage, and Replay TUI — Design

**Date:** 2026-07-30  
**Status:** Approved design

## Goal

Replace the standalone recorder's bare ENTER interface with one terminal
application for the complete local episode workflow:

1. create or select a standalone LeRobot dataset;
2. record, stop-and-keep, or discard episodes;
3. inspect episodes, override their task text, and keep or exclude them;
4. replay a selected episode on the real arm through the existing ROS driver;
5. create an immutable cleaned LeRobot v3 export.

The command is `pixi run replay`. It is a mode-focused Textual TUI and does
not depend on FastAPI, the cockpit server, the collection catalog, or browser
assets. `pixi run record` remains available for simple legacy capture.

Only datasets produced by the standalone recorder are in scope. Cockpit
collection sessions, visual video replay, trimming, training, policy
inference, Hub upload, and direct serial replay are out of scope.

## Safety invariants

- The TUI never opens the serial port. `elrobot_driver` remains the only
  hardware owner and applies its velocity, workspace, singularity, joint,
  grasp, and deadman gates to replay commands.
- Recording and physical replay are mutually exclusive.
- Dataset management and export never publish robot commands.
- Raw LeRobot datasets are never rewritten, renamed, or deleted.
- Replay must be explicitly armed and never runs faster than the recording.
- Stopping replay means immediately ceasing `/joint_command` publication; the
  driver's 200 ms deadman then freezes the arm.
- Tests that initialize ROS use `ROS_DOMAIN_ID=77`.

## User interface

The top-level TUI has four mode-focused screens:

1. **Record**
2. **Manage**
3. **Replay**
4. **Export**

The header always shows the selected dataset and live health for the driver,
joint state, wrist camera, external camera, and action stream. The footer shows
only keys valid in the current state.

### Record

The Record screen shows the task instruction, saved episode count, next
episode index, accepted frame count, and stream freshness.

- `Space` starts an episode when all required streams are fresh.
- `Space` again stops and keeps the take.
- `d` discards the active take.
- Dataset and mode switching are disabled during an active take.

The screen uses the existing `Recorder`, including its 30 Hz sampling,
500 ms freshness rule, streaming video encoding, two-thread encoder default,
resume behavior, and task locking.

### Manage

The Manage screen lists episode index, frame count, duration, effective task
text, keep/exclude decision, and interrupted status.

- `Enter` edits the selected episode's task override.
- `x` toggles keep/exclude.
- Dataset rename changes only its display name in the sidecar. It never moves
  the raw directory.

Episodes default to kept with their recorded task text. A missing sidecar
entry therefore means "keep with original task."

### Replay

The Replay screen shows the selected episode, task, frame count, duration,
driver state, and one unambiguous state label: `DISARMED`, `ARMED`,
`SEEKING`, or `PLAYING`.

1. The operator types the exact word `arm`.
2. The TUI verifies the real driver, fresh complete joint state, and command
   source exclusivity.
3. `Space` seeks the first recorded action pose and then replays the action
   stream.
4. `Space` or `Esc` stops at any time.

Replay runs at the dataset's recorded 30 Hz rate.
Changing dataset, episode, or mode stops and disarms before changing context.
An excluded episode remains replayable because exclusion affects cleaned
exports, not the immutable raw recording.

### Export

The Export screen previews kept episode count, frame count, duration, task
overrides, and the next versioned destination. Confirmation creates:

```text
data/episodes/exports/<name>-vNNN/
```

Each kept source episode becomes one output episode. Output indices are
regenerated from zero and task overrides are written into every exported
frame. Existing export versions are never overwritten.

## Dataset discovery and creation

The default browser scans immediate child directories under `data/episodes/`
that contain a valid LeRobot v3 dataset and ignores `exports/`. An explicit
root may be opened from the TUI.

Creating a dataset records a sanitized directory name, display name, local
repository ID, and default task. The raw directory is created lazily by
`Recorder` only after all streams are fresh, because the first wrist and
external images define the video feature shapes. The default repository ID is
`local/<directory-name>`.

Existing standalone datasets open without migration. If no sidecar exists,
the TUI derives the display name from the directory and the repository ID from
that name. The user may correct the repository ID in the dataset metadata
before recording or exporting.

## Sidecar metadata

Mutable decisions live next to, not inside, the raw dataset:

```text
data/episodes/elrobot_teleop/
data/episodes/elrobot_teleop.elrobot.json
```

The versioned JSON document contains:

```json
{
  "version": 1,
  "revision": 4,
  "root": "elrobot_teleop",
  "repo_id": "local/elrobot_teleop",
  "display_name": "Elrobot teleop",
  "default_task": "teleop",
  "episodes": {
    "3": {
      "task": "pick up the red cube",
      "decision": "exclude",
      "interrupted": false
    }
  }
}
```

`revision` increments on every successful metadata change and is captured by
cleaned-export provenance. `task` is absent when the recorded task should be
used. `decision` is absent when the episode is kept. `interrupted` is true only
when shutdown saved an active take.

Writes use a temporary file, flush, and `os.replace()` in the sidecar's
directory. The raw dataset metadata remains the authority for episode
existence, indices, frame counts, FPS, and original task text. Sidecar entries
for nonexistent episodes are rejected rather than silently ignored.

## Components

### Neutral replay core

The existing `ReplayLibrary`, `ReplayError`, and `PhysicalReplay` move from the
web package to a neutral episode module. The cockpit may continue importing
that core, but the TUI imports no web module.

`ReplayLibrary` reads only the `action` column for physical replay. Video
decoding is not part of this TUI.

### TUI ROS bridge

A small ROS node owns:

- subscriptions for `/joint_states` and stream-health reporting;
- the `/joint_command` replay publisher;
- a transient-local `/teleop_mode` publisher;
- graph inspection for driver and competing command nodes.

Arming requires a publisher named `elrobot_driver` and a fresh joint state
containing all eight controlled joints. A generic `/joint_states` publisher
does not count as the real driver.

The phone IK publishes continuously even when the clutch is released. To avoid
fighting replay, `ik_node` gains two neutral mode messages:

- `replay`: pause its `/joint_command` timer while preserving the current
  7/6/5-DoF configuration;
- `resume`: re-enable publication with that same configuration and clear the
  stale target.

The TUI publishes `replay` when arming and `resume` when disarming or exiting.
A latched `replay` message also pauses an IK node that starts while replay is
armed. If the TUI crashes before `resume`, leaving phone IK paused is the safe
failure; restarting the TUI or stack restores normal operation.

The TUI permits its own publisher and the paused `ik` node. Any other
`/joint_command` publisher, including jog or a cockpit process, makes arming
fail with the publisher name shown. Replay does not begin at arm time; the
operator must still press `Space`.

### Recorder lifecycle

The TUI constructs the existing `Recorder` for the selected dataset on entering
Record. Leaving Record while idle calls `close()` and removes the node from the
executor before any reader opens the dataset. Returning to Record creates a new
`Recorder`, which resumes and appends.

This writer/reader separation avoids opening LeRobot Parquet/video data while a
writer still owns unfinished metadata or encoder state. Mode switching is
disabled during an active take; the operator must keep or discard it first.

### Dataset metadata and exporter

A standalone metadata component discovers raw roots and owns sidecar validation
and atomic updates. It has no ROS dependency.

A standalone exporter reads raw episodes plus effective sidecar decisions,
builds a fresh LeRobot v3 dataset under `<destination>.inprogress`, finalizes
it, reloads it, validates episode and frame counts, writes a provenance
manifest, and atomically promotes it to the reserved version path.

The provenance manifest records source root, source episode index, effective
task, frame count, raw-tree SHA-256, and sidecar revision. Export never mutates
or removes the source.

### TUI controller

The Textual layer renders state and forwards user intent to the recorder,
metadata, exporter, and replay core. The transition logic is kept independent
of rendering so it can be unit-tested without an interactive terminal.

Textual supplies the data table, key bindings, modal input, responsive layout,
and headless UI test harness.

## Data and control flow

Recording:

```text
phone/jog -> /joint_command ----+
driver -> /joint_states --------+
cameras -> image topics --------+-> Recorder -> raw LeRobot v3 dataset
```

Physical replay:

```text
raw dataset action -> PhysicalReplay -> /joint_command
                                            |
                                            v
                                      elrobot_driver -> serial arm
```

Cleaned export:

```text
raw dataset + sidecar decisions -> new versioned LeRobot v3 dataset
```

## Shutdown and error handling

- `Ctrl-C`, terminal loss, or an unexpected TUI exception during replay stops
  publishing and attempts to publish `resume`.
- The same events during an active recording attempt `stop()` and finalization.
  A successfully saved take is marked interrupted. If saving fails, the error
  is reported and no sidecar episode entry is invented.
- Driver loss or stale/incomplete joint state during seek or playback stops
  and disarms replay.
- Record start failure leaves the recorder idle and reports the missing or
  stale stream.
- A corrupt dataset or sidecar opens read-only with a precise error. There is
  no automatic repair.
- Export errors leave the source untouched and retain no completed destination
  version. An incomplete staging directory is reported on the next run and may
  be replaced only after explicit confirmation.
- Terminal resize redraws the current mode without changing control state.

## Verification

All automated checks are offline-safe.

1. **Metadata tests**
   - discovery and explicit-root opening;
   - lazy dataset target creation;
   - display-only rename;
   - atomic sidecar updates and malformed-sidecar rejection;
   - nonexistent episode overlay rejection.
2. **Recorder integration**
   - synthetic two-camera/state/action streams on `ROS_DOMAIN_ID=77`;
   - start, stop-and-keep, discard, finalize, resume, and interrupted save;
   - no mode switch while a take is active.
3. **Replay integration**
   - real-driver-name and fresh-complete-state gates;
   - exact `arm` confirmation;
   - IK `replay`/`resume` pause behavior;
   - rejection of any other command publisher;
   - start-pose seek, fixed-rate playback, stop, driver loss, and stale state;
   - automatic stop/disarm on context changes.
4. **Export round trip**
   - kept-only episodes;
   - task overrides;
   - regenerated indices and correct frame counts;
   - reload validation and provenance manifest;
   - unchanged raw-tree hash;
   - immutable version allocation and failed-export isolation.
5. **TUI controller**
   - mode navigation and valid keys by state;
   - record/replay mutual exclusion;
   - shutdown cleanup;
   - tests use a fake renderer and require no terminal.

The full `pixi run test` suite remains the final offline gate. Real-hardware
validation uses a short expendable recording in a clear workspace, visual
inspection of its metadata, a guarded replay with STOP exercised, and a
reload of the cleaned export.

## Deliberate omissions

- No camera preview or visual episode playback in the terminal.
- No episode trimming.
- No source dataset deletion or directory rename.
- No LeRobot hardware plugin and no direct use of `lerobot-replay`.
- No custom serial, motion, or safety implementation.
- No new TUI framework.
