# Collection and Curation Cockpit — Design

**Date:** 2026-07-28
**Status:** Approved in brainstorming; ready for implementation planning

## Goal

Add the first data-management milestone for the physical-AI cockpit:

```
teleop → collect task-labelled episodes → curate → export LeRobot v3
```

An operator must be able to create reusable tasks, record multiple episodes into a
collection session, review finalized sessions by task, apply reversible curation,
and export selected kept episodes as an immutable local LeRobot v3 dataset.

This milestone deliberately stops before training and inference. It establishes the
reliable, traceable data foundation those later stages will consume.

## Existing system and constraints

- `episode_recorder.py` already records directly into LeRobot v3 datasets. Cockpit
  collection continues to use that path; MCAP remains the separate `pixi run bag`
  workflow.
- The installed LeRobot version is 0.6.1. Its dataset representation is Parquet,
  MP4, and metadata.
- The cockpit already has visual replay and explicitly armed physical replay.
- Physical replay remains an ordinary DDS command source and retains all existing
  driver-presence, arming, arbitration, speed, stop, seek, and deadman gates.
- Raw recordings are immutable after session finalization. Curation never edits
  them in place.
- Only one collection session and one export may be active at a time.
- All automated ROS tests use `ROS_DOMAIN_ID=77`; tests never publish on the default
  DDS domain.
- This work does not modify calibration artifacts, the hardware driver, or IK.

## Product decisions

1. Raw recordings are preserved; all curation is a reversible sidecar overlay.
2. A merge combines selected kept episodes from one or more task groups into one
   training dataset. It does not concatenate clips into a single episode.
3. Initial editing supports task reassignment and one continuous start/end trim.
4. Tasks are saved records with stable IDs and editable names and instructions.
5. The selected task is locked while an episode is recording but may change between
   episodes in the same session.
6. Each completed collection session is a separate immutable raw LeRobot v3 dataset.
7. A session appears in Curate only after it has been finalized.
8. Export initially targets versioned immutable local datasets only.
9. Export includes only explicitly kept episodes; unreviewed and rejected episodes
   are excluded.
10. Curate supports both visual replay and separately armed physical replay.
11. Replay uses the curated trim by default and provides a `View raw` toggle.
12. One export may include one or more selected task groups while preserving task
    labels.
13. The cockpit owns the collection-session lifecycle. The standalone
    `pixi run record` command remains supported when it is used independently.

## Chosen architecture

Use immutable raw sessions, a JSON sidecar catalog, and a read-only export builder:

```text
data/collections/
├── catalog.json
├── raw/
│   └── <session-id>/          # One finalized LeRobot v3 dataset
└── exports/
    └── <export-name>-v001/    # One immutable curated LeRobot v3 dataset
```

`data/collections` is the default and is configurable as one collection root so
tests and alternate storage volumes do not require changing individual paths.

Stable raw episode identity is:

```
<session-id>:<source-episode-index>
```

The catalog stores task definitions, session metadata, source episode identity,
review decisions, task reassignment, trim bounds, notes, export records, and recovery
state. It does not duplicate frames, images, or robot state.

This approach was selected over chaining `lerobot-edit-dataset` after every edit,
which would turn cheap UI changes into dataset rewrites, and over SQLite plus a
general job system, which is unnecessary for one local operator and one writer.

## Backend components

### `CollectionCatalog`

Owns the in-memory catalog and serializes it to `catalog.json`.

- Includes a top-level schema version and monotonically increasing catalog revision.
- Writes to a temporary file in the same directory, flushes it, and atomically
  replaces the prior catalog.
- Serializes all writes through one process lock.
- Validates referential integrity before committing a change.
- Stores a write-ahead marker containing the selected task identity before an
  episode can be saved, then clears it only after the saved raw episode and catalog
  record agree.
- Retains source task assignment separately from the effective curated assignment.
- Provides reset operations for review, reassignment, trim, and notes; no raw-file
  mutation is needed to undo curation.

### `CollectionManager`

Owns exactly one collection state machine and the optional managed recorder node.

- Creates a unique session record and raw root.
- Starts recording only after all required DDS streams are fresh.
- Locks the task for the duration of an episode.
- Calls the existing recorder's save or discard behavior.
- Finalizes the LeRobot dataset before registering the session as ready.
- Rejects overlapping or invalid commands with the current authoritative state.
- Detects an independently running `episode_recorder` before starting a managed
  recorder and refuses the start rather than allowing duplicate recorders.

The recorder implementation is refactored only enough to expose a clean managed
lifecycle. The CLI remains a wrapper around the same recorder class. The web process
spins the managed recorder alongside its existing ROS bridge; recording logic is not
reimplemented in FastAPI.

### `CuratedReplayLibrary`

Extends the current replay library from a single dataset/index view to a stable
session episode ID.

- Opens finalized raw sessions read-only.
- Resolves source frames and effective curated frames.
- Provides the same resolved range to state playback, camera playback, and physical
  replay.
- Never opens the active session as a replayable dataset.
- Stops and disarms physical replay before the selected episode, mode, or raw/curated
  range changes.

### `ExportBuilder`

Builds one new LeRobot v3 dataset from the selected catalog view.

- Resolves selected task groups to explicitly kept episodes.
- Reads only the effective continuous frame range from every source episode.
- Writes each source selection as a separate output episode.
- Regenerates output episode and frame indices contiguously.
- Uses the current effective task assignment and instruction.
- Finalizes and reload-validates the output before making it visible.
- Writes an export manifest that records the catalog revision, source episode IDs,
  source frame ranges, effective task definitions, and creation time.

Export runs in one supervised in-process worker because video encoding can take long
enough that the HTTP request must not remain open. This is a single-purpose worker,
not a general queue.

## Task model

Each task contains:

```json
{
  "id": "opaque-stable-id",
  "name": "Pick red cube",
  "instruction": "Pick up the red cube and place it in the tray.",
  "archived": false,
  "created_at": "2026-07-28T10:00:00Z",
  "updated_at": "2026-07-28T10:00:00Z"
}
```

- IDs never change.
- Name and instruction are editable.
- Archive replaces destructive deletion.
- Archived tasks are hidden from new collection by default but remain resolvable for
  historical episodes and exports.
- Every saved raw episode records both its source task ID in the catalog and the task
  instruction snapshot in the raw LeRobot data.
- Changing a task affects future exports that use it. Each export manifest snapshots
  the exact definition used, so completed exports remain reproducible.

## Session and episode model

A session contains an ID, optional display name, creation/finalization timestamps,
raw dataset path, lifecycle state, and source episode records. A single session may
contain episodes from multiple tasks.

The lifecycle is:

```text
idle
  → starting
  → ready
  → recording
  → ready
  → finalizing
  → idle
```

`starting` may return to `idle` on failure. Interrupted finalization enters
`recoverable` and may be retried.

An episode curation overlay is equivalent to:

```json
{
  "episode_id": "session-id:12",
  "source_task_id": "task-a",
  "task_id": "task-b",
  "review": "kept",
  "trim": {
    "start_frame": 14,
    "end_frame_exclusive": 486
  },
  "notes": "Clean grasp; remove setup motion"
}
```

- `review` is `unreviewed`, `kept`, or `rejected`.
- `task_id` is the effective assignment; clearing it restores `source_task_id`.
- Trim bounds are frame-based, use an exclusive end, and must describe a non-empty
  range inside the source episode.
- Clearing the trim restores the full source range.
- Rejected episodes remain available and may be returned to kept or unreviewed.

## Collection lifecycle

1. The operator selects or creates a task and optionally names the session.
2. `Start collection` creates the session marker and managed recorder.
3. `Record episode` waits up to the existing readiness timeout for wrist camera,
   external camera, joint state, and joint command streams.
4. While recording, task and session controls are locked.
5. `Stop & keep` stops sampling and completes `save_episode()` before reporting
   success. The write-ahead marker makes a completed save recoverable if the process
   fails before the following catalog update. Here, "keep" means retain the raw take;
   its curation review state is initially `unreviewed`.
6. `Discard` clears the unfinished episode buffer and returns to the ready state.
7. Between episodes the operator may choose another saved task.
8. `Finish session` is disabled while recording. It finalizes the dataset, validates
   it, closes the recorder node, and atomically marks the session ready.
9. Only then does the session appear in Curate.

If a session has no saved episodes, finishing returns to idle and archives an empty
session record without creating a Curate entry.

## Curation behavior

Curate is a full-workspace cockpit mode rather than another card in the existing
middle column.

```text
┌────────────────────┬───────────────────────────┬──────────────────────┐
│ Tasks and sessions │ Camera replay + timeline  │ Episode details      │
│ Episode list       │ Raw/curated range toggle  │ Review/edit controls │
└────────────────────┴───────────────────────────┴──────────────────────┘
```

The left side groups episodes by their effective task and permits session/task
filtering. Compact state markers show unreviewed, kept, rejected, trimmed, and
reassigned episodes.

The center provides playback, frame scrubbing, start/end trim handles, and `View raw`.
The right side provides keep/reject/unreviewed, task reassignment, notes, and physical
replay controls.

Visual and physical replay use the curated range by default. End-of-range behavior
matches normal replay: after reaching the end, the next Play resets to the beginning
of the selected range. The existing default physical replay speed remains `0.6`.

Leaving Curate, selecting another episode, editing its effective range, or enabling
`View raw` stops and disarms physical replay first.

## Export semantics

The export dialog includes:

- One or more selected task groups
- Kept episode count
- Total effective duration
- Output dataset name
- Next immutable version
- Validation errors

Export is unavailable when the selection resolves to zero kept episodes. Unreviewed
and rejected episodes are always excluded.

Every input episode remains a distinct output episode even when multiple tasks or
sessions are selected. Trimmed leading and trailing frames are omitted. Current task
assignments are written into the output LeRobot task field. Output task indices and
episode indices are regenerated consistently by the dataset writer.

An export is built under an `.inprogress` path and receives its final
`<name>-vNNN` path only after `finalize()` and a successful read-back validation.
Existing versions are never overwritten. A failed build remains retryable but is not
listed as a completed dataset.

## HTTP and WebSocket interface

The server owns state. The browser updates controls from command responses and the
existing WebSocket state broadcast rather than optimistically assuming success.

### Tasks

- `GET /api/tasks`
- `POST /api/tasks`
- `PATCH /api/tasks/{task_id}`

### Collection

- `GET /api/collection`
- `POST /api/collection/session/start`
- `POST /api/collection/episode/start`
- `POST /api/collection/episode/stop`
- `POST /api/collection/episode/discard`
- `POST /api/collection/session/finish`

### Recovery

- `GET /api/collection/recovery`
- `POST /api/collection/recovery/{session_id}/finish`
- `POST /api/collection/recovery/{session_id}/archive`

### Curation

- `GET /api/curation/sessions`
- `GET /api/curation/sessions/{session_id}/episodes`
- `PATCH /api/curation/episodes/{session_id}/{episode_index}`

### Export

- `POST /api/exports`
- `GET /api/exports/{export_id}`

Invalid state transitions return `409 Conflict` and include the current collection
state. Invalid IDs, trim bounds, and empty export selections return `422
Unprocessable Entity`.

The existing `/api/record` endpoint temporarily delegates compatible
start/stop/discard commands to `CollectionManager` so the migration can be
incremental. It requires an active collection session and never creates one
implicitly. It is not a second recorder path.

## Failure handling and recovery

Each session writes its lifecycle marker before dataset creation. Starting an episode
atomically records a pending marker containing its source task ID and instruction
snapshot. Each successful episode save is recorded in the catalog only after
`save_episode()` returns; the pending marker is cleared only after that catalog write
succeeds.

On graceful cockpit shutdown:

- Sampling stops.
- A non-empty active episode is saved and marked as interrupted.
- The dataset is finalized when possible.

After an unclean process or machine failure:

- Previously saved episodes are recoverable.
- Frames held only in the unfinished in-memory episode buffer may be lost.
- The session is not shown as finalized.
- Startup exposes `Recover & finish` and `Archive incomplete session`.

Recovery resumes the raw LeRobot dataset and compares its saved episode count with the
catalog. If the dataset contains one completed episode not yet catalogued, the
write-ahead marker supplies its stable task identity. If no new saved episode exists,
the pending marker represents the lost in-memory take and is cleared with an explicit
warning. Recovery then validates the saved episodes, finalizes the dataset, and marks
it ready. Any other count mismatch is treated as a repair-required error rather than
guessed. Archiving hides an incomplete session without deleting its files.

Missing or stale input streams prevent episode start. During recording, stream
health, accepted frame count, and frame-skip warnings remain visible. Disk or encoder
errors stop further collection and retain a recoverable error state.

Catalog and export promotion use atomic same-filesystem replacement. A partial
catalog or partial export is never presented as completed.

## Safety invariants

- Collection observes DDS streams and writes files; it never opens the serial port.
- Curated physical replay remains mutually exclusive with slider control and requires
  a running driver plus explicit arming.
- Physical replay continues publishing densely enough for the driver's deadman and
  stops publishing immediately on stop, disarm, mode change, selection change, or
  replay error.
- Collection and export code do not modify `elrobot_driver.py`,
  `cartesian_ik.py`, calibration JSON, servo EEPROM, or the authoritative URDF.
- A managed recorder is not started when an external recorder node is detected.

## Testing

All tests are offline. Any test creating ROS nodes sets `ROS_DOMAIN_ID=77` before
initializing ROS.

### Unit coverage

- Task create, edit, archive, and stable identity
- Catalog schema, atomic writes, and reload
- Review-state transitions and reset operations
- Task reassignment and source-assignment preservation
- Trim validation and raw/curated range resolution
- Collection state transitions and duplicate-command rejection
- Empty-session completion
- Export selection and immutable version allocation

### Integration coverage

- Managed recorder keep and discard with fake DDS streams
- Multiple tasks in one finalized session
- Graceful shutdown and interrupted-session recovery
- Finalized sessions hidden until ready
- Curated visual replay frame bounds
- Curated physical replay bounds and existing arming gates
- Replay restart at the selected range beginning after reaching its end
- Multi-session, multi-task export
- LeRobot v3 read-back with contiguous episode/frame indices
- Exported task labels and exact trimmed frame counts
- Export failure, cleanup state, and retry
- Raw source files unchanged by curation and export

The existing recorder, Web API, replay, calibration, driver safety, and complete
`pixi run test` suites must continue to pass.

## Delivery sequence

1. Catalog models, atomic persistence, and task API
2. Minimal recorder lifecycle refactor
3. `CollectionManager`, recovery, and collection API
4. Teleop session UI
5. Stable session-episode replay addressing and curated ranges
6. Full-workspace Curate UI
7. Export builder, manifest, and validation
8. Operator guide, browser verification, and full offline regression suite

Each stage preserves a usable cockpit and the existing standalone recording command.

## Acceptance criteria

The milestone is complete when an operator can:

1. Create a saved task and start a collection session.
2. Record, keep, and discard multiple episodes across one or more tasks.
3. Finish the session and find it grouped correctly in Curate.
4. Keep, reject, unreview, reassign, annotate, and trim episodes across restarts.
5. Replay either the effective curated range or the untouched source episode.
6. Use physical replay only through the existing explicit safety gates.
7. Export selected kept task groups as a loadable local LeRobot v3 dataset.
8. Trace every output episode to its raw session, episode, task, and frame range.
9. Recover completed takes after an interrupted collection session.
10. Verify that curation and export did not modify finalized raw source files.

## Out of scope

- Training and inference controls
- Hugging Face Hub upload
- Permanent raw-data deletion
- Multiple trim segments, episode splitting, or clip concatenation
- Parallel collection sessions or exports
- SQLite or a general background-job system
- MCAP migration or conversion
- Advanced annotations, scoring, user accounts, and collaboration
