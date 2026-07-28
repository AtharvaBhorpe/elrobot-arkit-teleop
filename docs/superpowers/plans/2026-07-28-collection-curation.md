# Collection and Curation Cockpit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the cockpit workflow for task-labelled collection, reversible episode curation, safe replay, and immutable local LeRobot v3 export.

**Architecture:** Finalized LeRobot v3 session datasets remain immutable under one collection root. A small atomic JSON catalog stores tasks, lifecycle markers, and reversible curation overlays; focused manager, replay, and export modules own their respective workflows while FastAPI only adapts them to HTTP/WebSocket.

**Tech Stack:** Python 3.12, ROS 2 Jazzy `rclpy`, FastAPI, LeRobot 0.6.1, NumPy, OpenCV/PyAV, vanilla HTML/CSS/JavaScript, existing script-style offline tests.

## Global Constraints

- Complete `docs/superpowers/plans/2026-07-28-recorder-performance.md` first.
- Default collection root is exactly `data/collections`; allow one `COLLECTION_ROOT` override.
- Finalized raw datasets and completed exports are immutable; curation changes only `catalog.json`.
- Review state is exactly `unreviewed`, `kept`, or `rejected`.
- Initial trim is one non-empty continuous range with an exclusive end frame.
- Export includes only explicitly kept episodes from selected effective task groups.
- One collection session and one export may be active at a time.
- A session is invisible in Curate until LeRobot finalization and read-back validation succeed.
- Physical replay retains all existing driver, arm, arbitration, seek, speed, stop, and deadman gates; default speed remains `0.6`.
- No driver, IK, calibration JSON, servo EEPROM, or authoritative URDF changes.
- Any test that creates ROS nodes must set `ROS_DOMAIN_ID=77`.
- Preserve the standalone `pixi run record` command and all unrelated dirty-worktree changes.
- Do not add SQLite, a general job queue, Hub upload, permanent raw deletion, training, or inference.

---

### Task 0: Preserve the completed cockpit usability baseline

**Files:**
- Existing modified: `src/elrobot/web/server.py`
- Existing modified: `src/elrobot/web/static/app.js`
- Existing modified: `src/elrobot/web/static/index.html`
- Existing modified: `src/elrobot/web/static/scene.js`
- Existing modified: `src/elrobot/web/static/style.css`
- Existing modified: `tests/test_web_api.py`

**Interfaces:**
- Consumes: the already-authorized uncommitted Web UI fixes in the current worktree.
- Produces: a clean baseline containing calibration/replay tabs, replay reset, `0.6` physical replay UI default, camera/render throttling, control freshness guards, disconnect disarming, accessibility fixes, and printed Cockpit URL.

- [ ] **Step 1: Review the exact existing diff**

Run:

```bash
git diff -- src/elrobot/web/server.py src/elrobot/web/static/app.js src/elrobot/web/static/index.html src/elrobot/web/static/scene.js src/elrobot/web/static/style.css tests/test_web_api.py
```

Expected: only the previously requested cockpit improvements; no collection,
curation, or export implementation.

- [ ] **Step 2: Verify the baseline**

Run:

```bash
pixi run python tests/test_web_api.py
pixi run lint
```

Expected: both exit zero.

- [ ] **Step 3: Commit only the baseline files**

```bash
git add src/elrobot/web/server.py src/elrobot/web/static/app.js src/elrobot/web/static/index.html src/elrobot/web/static/scene.js src/elrobot/web/static/style.css tests/test_web_api.py
git commit -m "fix: finish cockpit usability improvements"
```

Do not stage `.agents/`, `.codex/`, or `.superpowers/`.

---

### Task 1: Atomic task and curation catalog

**Files:**
- Create: `src/elrobot/web/collection.py`
- Create: `tests/test_collection.py`
- Modify: `pixi.toml:43-47`

**Interfaces:**
- Produces: `CatalogError`, `CollectionCatalog(root: str | Path)`, `create_task(name, instruction)`, `update_task(task_id, *, name=None, instruction=None, archived=None)`, `create_session(name)`, `set_pending(session_id, task_id)`, `clear_pending(session_id)`, `commit_episode(session_id, source_index, frames, interrupted=False)`, `finalize_session(session_id, state)`, `update_episode(session_id, source_index, patch)`, `task_groups()`, `sessions()`, `episode(session_id, source_index)`, `create_export(record)`, `update_export(export_id, patch)`, and `export(export_id)`.
- Storage contract: `catalog.json` with `schema_version`, `revision`, `tasks`, `sessions`, and `exports`.

- [ ] **Step 1: Write failing catalog tests**

Create `tests/test_collection.py` with:

```python
import json
import tempfile
from pathlib import Path

from elrobot.web.collection import CatalogError, CollectionCatalog


def catalog():
    return CollectionCatalog(Path(tempfile.mkdtemp()) / "collections")


def test_tasks_have_stable_identity_and_archive():
    c = catalog()
    task = c.create_task("Pick cube", "Pick up the red cube.")
    changed = c.update_task(task["id"], name="Pick red cube")
    assert changed["id"] == task["id"]
    assert changed["name"] == "Pick red cube"
    assert c.update_task(task["id"], archived=True)["archived"] is True


def test_episode_edits_are_reversible_overlays():
    c = catalog()
    source = c.create_task("Pick", "Pick the object.")
    reassigned = c.create_task("Place", "Place the object.")
    session = c.create_session("morning")
    c.set_pending(session["id"], source["id"])
    c.commit_episode(session["id"], 0, 100)
    edited = c.update_episode(session["id"], 0, {
        "review": "kept",
        "task_id": reassigned["id"],
        "trim": {"start_frame": 10, "end_frame_exclusive": 80},
        "notes": "clean take",
    })
    assert edited["source_task_id"] == source["id"]
    assert edited["task_id"] == reassigned["id"]
    reset = c.update_episode(session["id"], 0, {
        "review": "unreviewed", "task_id": None, "trim": None, "notes": "",
    })
    assert reset["source_task_id"] == source["id"]
    assert reset["trim"] is None


def test_trim_and_review_validation():
    c = catalog()
    task = c.create_task("Pick", "Pick.")
    session = c.create_session("")
    c.set_pending(session["id"], task["id"])
    c.commit_episode(session["id"], 0, 20)
    for patch in (
        {"review": "deleted"},
        {"trim": {"start_frame": 8, "end_frame_exclusive": 8}},
        {"trim": {"start_frame": -1, "end_frame_exclusive": 8}},
        {"trim": {"start_frame": 0, "end_frame_exclusive": 21}},
    ):
        try:
            c.update_episode(session["id"], 0, patch)
        except CatalogError as exc:
            assert exc.code == 422
        else:
            raise AssertionError(f"accepted invalid patch {patch}")


def test_catalog_write_is_reloadable_and_revisioned():
    c = catalog()
    c.create_task("Pick", "Pick.")
    raw = json.loads((c.root / "catalog.json").read_text())
    assert raw["schema_version"] == 1
    assert raw["revision"] == 1
    reopened = CollectionCatalog(c.root)
    assert reopened.snapshot() == raw


def test_export_records_share_atomic_catalog_path():
    c = catalog()
    created = c.create_export({
        "id": "export_test",
        "state": "queued",
        "name": "training",
    })
    assert created["state"] == "queued"
    c.update_export("export_test", {"state": "complete"})
    assert CollectionCatalog(c.root).export("export_test")["state"] == "complete"
```

Add a `main()` that calls every test and prints `COLLECTION TESTS PASSED`.
Append `python tests/test_collection.py` to the `pixi run test` chain.

- [ ] **Step 2: Run the new suite and verify it fails**

Run:

```bash
pixi run python tests/test_collection.py
```

Expected: import failure because `elrobot.web.collection` does not exist.

- [ ] **Step 3: Implement the catalog schema and atomic writer**

Use this catalog shape:

```python
EMPTY_CATALOG = {
    "schema_version": 1,
    "revision": 0,
    "tasks": {},
    "sessions": {},
    "exports": {},
}
```

Use UTC ISO timestamps and opaque IDs:

```python
def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex}"
```

Implement atomic persistence with a same-directory temporary file:

```python
def _persist(self) -> None:
    self.root.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".catalog-", suffix=".json",
                               dir=self.root)
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(self._data, fh, indent=2, sort_keys=True)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, self.path)
    finally:
        Path(tmp).unlink(missing_ok=True)
```

All public mutations acquire one `threading.RLock`, mutate a deep copy, validate
it, increment `revision`, persist it, then publish the copy as current state.

- [ ] **Step 4: Implement tasks, sessions, pending markers, and episode overlays**

Task records contain `id`, `name`, `instruction`, `archived`, `created_at`, and
`updated_at`. Reject blank names and instructions.

Session records contain:

```python
{
    "id": session_id,
    "name": name.strip(),
    "repo_id": f"local/elrobot_{session_id}",
    "root": str(self.root / "raw" / session_id),
    "state": "active",
    "created_at": _now(),
    "finalized_at": None,
    "pending_episode": None,
    "episodes": {},
}
```

`set_pending()` snapshots `source_task_id`, `source_task_instruction`, and the
next source index. `commit_episode()` consumes that marker and creates:

```python
{
    "source_index": source_index,
    "source_task_id": pending["source_task_id"],
    "source_task_instruction": pending["source_task_instruction"],
    "frames": frames,
    "review": "unreviewed",
    "task_id": None,
    "trim": None,
    "notes": "",
    "interrupted": interrupted,
}
```

`update_episode()` validates task existence, review value, string notes, and
trim bounds. `task_groups()` groups by `task_id or source_task_id`.
`create_export()`, `update_export()`, and `export()` use the same locked,
revisioned persistence path as every other catalog mutation.

- [ ] **Step 5: Run catalog tests**

Run:

```bash
pixi run python tests/test_collection.py
pixi run lint
```

Expected: both exit zero.

- [ ] **Step 6: Commit the catalog**

```bash
git add src/elrobot/web/collection.py tests/test_collection.py pixi.toml
git commit -m "feat: add collection catalog"
```

---

### Task 2: Managed recorder lifecycle contract

**Files:**
- Modify: `src/elrobot/nodes/episode_recorder.py`
- Modify: `tests/test_recorder.py`

**Interfaces:**
- Consumes: the streaming writer configuration implemented by the recorder-performance plan.
- Produces: `Recorder.set_task(instruction: str) -> None`, `Recorder.start() -> bool`, `Recorder.stop() -> int`, and the existing `discard()` and `close()` behavior.

- [ ] **Step 1: Add failing managed-lifecycle assertions**

In `test_record_cmd_topic()`, after recording starts:

```python
try:
    node.set_task("changed during recording")
except RuntimeError:
    pass
else:
    raise AssertionError("task changed during an active episode")
```

After the first save:

```python
node.set_task("second task")
assert node.args.task == "second task"
```

Add a direct start-timeout assertion using a recorder with no fresh streams:

```python
def test_start_reports_failure_without_streams():
    args = argparse.Namespace(
        fps=30.0, repo_id="local/missing", root="data/test_missing_streams",
        task="test", encoder_threads=2, start_timeout=0.05)
    rclpy.init()
    try:
        node = Recorder(args)
        assert node.start() is False
        assert node.recording is False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

Call it from `main()`.

- [ ] **Step 2: Run the recorder suite and verify failure**

Run:

```bash
pixi run python tests/test_recorder.py
```

Expected: failure because `set_task()` is missing and `start()` does not return
a success value.

- [ ] **Step 3: Implement the managed contract**

Add:

```python
def set_task(self, instruction: str) -> None:
    if self.recording or self._starting:
        raise RuntimeError("task is locked while an episode is active")
    instruction = instruction.strip()
    if not instruction:
        raise ValueError("task instruction cannot be blank")
    self.args.task = instruction
```

Use `getattr(self.args, "start_timeout", 10.0)` in `start()`. Return `False`
after timeout and `True` after `recording` becomes true.

Capture `frames = self.n_frames` in `stop()`, return `frames` after a successful
save, and return `0` when nothing was saved. Do not change the ROS command or
terminal behavior.

- [ ] **Step 4: Run recorder tests and commit**

Run:

```bash
pixi run python tests/test_recorder.py
pixi run lint
```

Expected: both exit zero.

```bash
git add src/elrobot/nodes/episode_recorder.py tests/test_recorder.py
git commit -m "refactor: expose managed recorder lifecycle"
```

---

### Task 3: Collection manager and recovery

**Files:**
- Create: `src/elrobot/web/collection_manager.py`
- Modify: `tests/test_collection.py`

**Interfaces:**
- Consumes: `CollectionCatalog` and the managed `Recorder` contract.
- Produces: `CollectionError(detail, code=409)`, `CollectionManager`, `snapshot()`, `start_session(task_id, name="")`, `start_episode(task_id)`, `stop_episode()`, `discard_episode()`, `finish_session()`, `recoveries()`, `recover_finish(session_id)`, `recover_archive(session_id)`, and `shutdown()`.
- Dataset validator contract: `dataset_validator(session: dict, finalize: bool) -> {"count": int, "lengths": list[int]}`.

- [ ] **Step 1: Add a fake recorder and failing state-machine tests**

Add to `tests/test_collection.py`:

```python
class FakeRecorder:
    def __init__(self):
        self.recording = False
        self.task = None
        self.episodes = 0
        self.next_frames = 12
        self.closed = False

    def set_task(self, instruction):
        if self.recording:
            raise RuntimeError
        self.task = instruction

    def start(self):
        self.recording = True
        return True

    def stop(self):
        self.recording = False
        self.episodes += 1
        return self.next_frames

    def discard(self):
        self.recording = False

    def close(self):
        self.closed = True


def test_collection_lifecycle_and_task_lock():
    from elrobot.web.collection_manager import CollectionError, CollectionManager
    c = catalog()
    a = c.create_task("Pick", "Pick.")
    b = c.create_task("Place", "Place.")
    made = []
    m = CollectionManager(c, recorder_factory=lambda _: made.append(FakeRecorder()) or made[-1])
    assert m.start_session(a["id"], "morning")["state"] == "ready"
    assert m.start_episode(a["id"])["state"] == "recording"
    try:
        m.start_episode(b["id"])
    except CollectionError as exc:
        assert exc.code == 409
    else:
        raise AssertionError("started a second episode")
    assert m.stop_episode()["state"] == "ready"
    assert m.start_episode(b["id"])["state"] == "recording"
    assert m.discard_episode()["state"] == "ready"
    done = m.finish_session()
    assert done["state"] == "idle"
    assert made[0].closed is True
```

Add these transition tests:

```python
def test_invalid_transitions_and_empty_finish():
    from elrobot.web.collection_manager import CollectionError, CollectionManager
    c = catalog()
    task = c.create_task("Pick", "Pick.")
    m = CollectionManager(c, recorder_factory=lambda _: FakeRecorder())
    m.start_session(task["id"])
    try:
        m.start_session(task["id"])
    except CollectionError as exc:
        assert exc.code == 409
    else:
        raise AssertionError("started overlapping session")
    assert m.finish_session()["state"] == "idle"
    assert c.sessions()[0]["state"] == "archived_empty"


def test_finish_refuses_while_recording():
    from elrobot.web.collection_manager import CollectionError, CollectionManager
    c = catalog()
    task = c.create_task("Pick", "Pick.")
    m = CollectionManager(c, recorder_factory=lambda _: FakeRecorder())
    m.start_session(task["id"])
    m.start_episode(task["id"])
    try:
        m.finish_session()
    except CollectionError as exc:
        assert exc.code == 409
    else:
        raise AssertionError("finished a session while recording")


def test_recovery_reconciles_one_saved_pending_episode():
    from elrobot.web.collection_manager import CollectionManager
    c = catalog()
    task = c.create_task("Pick", "Pick.")
    session = c.create_session("interrupted")
    c.set_pending(session["id"], task["id"])
    c.finalize_session(session["id"], "recoverable")
    validator = lambda record, finalize: {"count": 1, "lengths": [12]}
    m = CollectionManager(
        c, recorder_factory=lambda _: FakeRecorder(),
        dataset_validator=validator)
    m.recover_finish(session["id"])
    repaired = c.episode(session["id"], 0)
    assert repaired["frames"] == 12
    assert repaired["interrupted"] is True


def test_archive_recovery_keeps_raw_files():
    from elrobot.web.collection_manager import CollectionManager
    c = catalog()
    session = c.create_session("broken")
    raw_root = Path(session["root"])
    raw_root.mkdir(parents=True)
    sentinel = raw_root / "do-not-delete"
    sentinel.write_text("raw")
    c.finalize_session(session["id"], "recoverable")
    m = CollectionManager(c, recorder_factory=lambda _: FakeRecorder())
    m.recover_archive(session["id"])
    assert sentinel.read_text() == "raw"
    assert c.sessions()[0]["state"] == "archived_incomplete"
```

- [ ] **Step 2: Run tests and verify import failure**

Run:

```bash
pixi run python tests/test_collection.py
```

Expected: import failure for `collection_manager`.

- [ ] **Step 3: Implement serialized state transitions**

Construct with dependency injection:

```python
class CollectionManager:
    def __init__(self, catalog, recorder_factory,
                 add_node=lambda node: None,
                 remove_node=lambda node: None,
                 external_recorder_alive=lambda: False,
                 dataset_validator=None):
        self.catalog = catalog
        self.recorder_factory = recorder_factory
        self.add_node = add_node
        self.remove_node = remove_node
        self.external_recorder_alive = external_recorder_alive
        self.dataset_validator = dataset_validator or self._validate_dataset
        self._lock = threading.RLock()
        self._state = "idle"
        self._session_id = None
        self._task_id = None
        self._recorder = None
```

Every command holds `_lock`, checks the exact valid source state, performs the
operation, and returns `snapshot()`. `start_session()` rejects archived tasks
and an external recorder, creates `argparse.Namespace` with session root/repo,
FPS 30, selected instruction, `encoder_threads=2`, and `start_timeout=10.0`,
then adds the recorder node. Store the selected task in `_task_id` and include
it in `snapshot()` so the legacy `/api/record` adapter can start an episode for
the active task without inventing a second task-selection state.

`start_episode()` writes the pending marker before calling `Recorder.start()`.
Clear it if start fails. `stop_episode()` saves first, then commits the catalog
episode. `discard_episode()` discards then clears the marker.

- [ ] **Step 4: Implement finalization and recovery**

`finish_session()` refuses while recording. It closes and removes the recorder,
read-back validates `num_episodes`, marks non-empty sessions `ready`, marks empty
sessions `archived_empty`, and returns to idle.

Recovery compares raw saved count with catalog episode count:

```python
if raw_count == catalog_count:
    # pending frames were never saved
    catalog.clear_pending(session_id)
elif raw_count == catalog_count + 1 and pending is not None:
    catalog.commit_episode(
        session_id, pending["source_index"], raw_lengths[-1],
        interrupted=True)
else:
    raise CollectionError(
        f"episode count mismatch: raw={raw_count}, catalog={catalog_count}",
        409)
```

After reconciliation, finalize/read-back validate and mark ready. Archive sets
`archived_incomplete` without unlinking any file.

`shutdown()` saves a non-empty active take as interrupted, attempts finalization,
and marks `recoverable` on any failure.

- [ ] **Step 5: Run manager tests and commit**

Run:

```bash
pixi run python tests/test_collection.py
pixi run lint
```

Expected: both exit zero.

```bash
git add src/elrobot/web/collection_manager.py tests/test_collection.py
git commit -m "feat: manage collection sessions"
```

---

### Task 4: Collection and task APIs

**Files:**
- Modify: `src/elrobot/web/server.py`
- Modify: `tests/test_web_api.py`

**Interfaces:**
- Consumes: `CollectionCatalog`, `CollectionManager`, and `Recorder`.
- Produces: task, collection, and recovery HTTP routes; WebSocket `collection` state; production ROS node add/remove hooks.

- [ ] **Step 1: Add failing API tests**

Add this recorder double and client helper:

```python
class FakeManagedRecorder:
    def __init__(self, args):
        self.args = args
        self.recording = False
        self.episodes = 0

    def set_task(self, instruction):
        self.args.task = instruction

    def start(self):
        self.recording = True
        return True

    def stop(self):
        self.recording = False
        self.episodes += 1
        return 12

    def discard(self):
        self.recording = False

    def close(self):
        pass


def _collection_client():
    root = Path(tempfile.mkdtemp()) / "collections"
    made = []

    def recorder_factory(args):
        recorder = FakeManagedRecorder(args)
        made.append(recorder)
        return recorder

    def validator(session, finalize):
        count = made[0].episodes if made else 0
        return {"count": count, "lengths": [12] * count}

    app = create_app(
        FakeBridge(),
        collection_root=root,
        recorder_factory=recorder_factory,
        dataset_validator=validator,
    )
    return TestClient(app), made
```

Then test:

```python
def test_task_and_collection_api():
    c, made = _collection_client()
    task = c.post("/api/tasks", json={
        "name": "Pick red cube",
        "instruction": "Pick up the red cube.",
    }).json()
    assert c.get("/api/tasks").json()["tasks"][0]["id"] == task["id"]
    assert c.post("/api/collection/session/start", json={
        "task_id": task["id"], "name": "morning",
    }).json()["state"] == "ready"
    assert c.post("/api/collection/episode/start", json={
        "task_id": task["id"],
    }).json()["state"] == "recording"
    assert c.post("/api/collection/episode/stop").json()["state"] == "ready"
    assert c.post("/api/collection/session/finish").json()["state"] == "idle"


def test_legacy_record_requires_active_session():
    c, _ = _collection_client()
    r = c.post("/api/record", json={"cmd": "start"})
    assert r.status_code == 409


def test_collection_api_errors_and_websocket_state():
    c, _ = _collection_client()
    assert c.post("/api/collection/episode/stop").status_code == 409
    assert c.post("/api/tasks", json={
        "name": "", "instruction": "Pick.",
    }).status_code == 422
    with c.websocket_connect("/ws") as ws:
        assert "collection" in ws.receive_json()
```

- [ ] **Step 2: Run Web API tests and verify failure**

Run:

```bash
pixi run python tests/test_web_api.py
```

Expected: new routes return 404.

- [ ] **Step 3: Make `WebBridge` own an explicit executor**

Replace `rclpy.spin(node)` with:

```python
from rclpy.executors import MultiThreadedExecutor

self._executor = MultiThreadedExecutor(num_threads=2)
self._executor.add_node(self.node)
self._spin = threading.Thread(target=self._executor.spin, daemon=True)
self._spin.start()
```

Add:

```python
def add_node(self, node):
    self._executor.add_node(node)

def remove_node(self, node):
    self._executor.remove_node(node)

def external_recorder_alive(self):
    return any(info.node_name == "episode_recorder"
               for info in self.node.get_publishers_info_by_topic("/record/status"))
```

Do not change command arbitration or serial ownership.

- [ ] **Step 4: Wire catalog and manager into `create_app()`**

Extend the factory signature with:

```python
collection_root=os.environ.get("COLLECTION_ROOT", "data/collections"),
recorder_factory=None,
dataset_validator=None,
```

Create a default recorder factory around `Recorder(args)`, instantiate catalog
and manager with the injected `dataset_validator`, and expose them as
`app.state.catalog` and `app.state.collection`.

Use safe bridge fallbacks for offline tests:

```python
add_node = getattr(bridge, "add_node", lambda node: None)
remove_node = getattr(bridge, "remove_node", lambda node: None)
external_alive = getattr(
    bridge, "external_recorder_alive", lambda: False)
```

Register it with
`app.add_event_handler("shutdown", manager.shutdown)`.

- [ ] **Step 5: Add the approved routes**

Implement:

```text
GET    /api/tasks
POST   /api/tasks
PATCH  /api/tasks/{task_id}
GET    /api/collection
POST   /api/collection/session/start
POST   /api/collection/episode/start
POST   /api/collection/episode/stop
POST   /api/collection/episode/discard
POST   /api/collection/session/finish
GET    /api/collection/recovery
POST   /api/collection/recovery/{session_id}/finish
POST   /api/collection/recovery/{session_id}/archive
```

Convert `CatalogError` and `CollectionError` into `HTTPException(exc.code,
exc.detail)`. Make `/api/record` delegate start/stop/discard only when a session
is active. Add `"collection": manager.snapshot()` to WebSocket state.

- [ ] **Step 6: Run Web API and full backend tests**

Run:

```bash
pixi run python tests/test_web_api.py
pixi run python tests/test_collection.py
pixi run python tests/test_recorder.py
pixi run lint
```

Expected: all exit zero.

- [ ] **Step 7: Commit API wiring**

```bash
git add src/elrobot/web/server.py tests/test_web_api.py
git commit -m "feat: add collection APIs"
```

---

### Task 5: Stable curated replay

**Files:**
- Create: `src/elrobot/web/curation.py`
- Modify: `src/elrobot/web/replay.py`
- Modify: `src/elrobot/web/server.py`
- Modify: `tests/test_web_api.py`

**Interfaces:**
- Produces: `EpisodeRef(session_id: str, source_index: int, raw: bool=False)`, `CuratedReplayLibrary`, `list_episodes(task_id=None, session_id=None)`, `states(ref)`, `actions(ref)`, and `frame_jpeg(ref, n, cam)`.
- Compatibility: `CuratedReplayLibrary(catalog, legacy=None)` delegates integer
  selections to the legacy single-dataset library until Task 7 removes those
  calls from the browser.
- Changes: `PhysicalReplay.play(selection, speed=0.6)` treats the selection as opaque and asks its library for actions.

- [ ] **Step 1: Add failing curated-range tests**

Build a finalized raw session with `_tiny_dataset()`, register two catalog
episodes, keep one with trim `[1, 4)`, and assert:

```python
listing = c.get(f"/api/curation/sessions/{session_id}/episodes").json()
assert listing["episodes"][0]["effective_frames"] == 3

states = c.get(
    f"/api/curation/episodes/{session_id}/0/states").json()
assert states["frames"] == 3

raw = c.get(
    f"/api/curation/episodes/{session_id}/0/states?raw=true").json()
assert raw["frames"] == 6
```

Add a physical replay test whose fake library records the received
`EpisodeRef`; assert trimmed actions only are published. Preserve the existing
arming and driver tests.

- [ ] **Step 2: Run Web API tests and verify 404 failures**

Run:

```bash
pixi run python tests/test_web_api.py
```

Expected: curation routes are absent.

- [ ] **Step 3: Implement effective bounds and session caching**

Define:

```python
@dataclass(frozen=True)
class EpisodeRef:
    session_id: str
    source_index: int
    raw: bool = False
```

`CuratedReplayLibrary` accepts a catalog plus an optional legacy
`ReplayLibrary`, caches one `ReplayLibrary` per ready session, and resolves:

```python
start = 0 if ref.raw or episode["trim"] is None else episode["trim"]["start_frame"]
end = episode["frames"] if ref.raw or episode["trim"] is None \
    else episode["trim"]["end_frame_exclusive"]
```

Extend `ReplayLibrary.actions()`, `states()`, and `frame_jpeg()` with optional
episode-local `start_frame` and `end_frame_exclusive` arguments. Clamp only the
requested displayed frame; reject invalid episode ranges. When a selection is
an integer, delegate `actions()`, `states()`, and `frame_jpeg()` to the legacy
library unchanged.

- [ ] **Step 4: Adapt physical replay without weakening gates**

Set `PhysicalReplay.speed = 0.6` and `play(selection, speed=0.6)`. Remove any
`int()` coercion of `selection`; store it opaquely in status and ask the
configured library for actions. Leave arm, driver, mutual-exclusion, max-speed,
seek, publish frequency, stop, and deadman behavior unchanged.

Before a curation update, raw/curated toggle, or selected-episode change, call
`player.stop()` and `player.arm(False, False)`.

- [ ] **Step 5: Add curation routes**

Implement:

```text
GET   /api/curation/sessions
GET   /api/curation/sessions/{session_id}/episodes
PATCH /api/curation/episodes/{session_id}/{source_index}
GET   /api/curation/episodes/{session_id}/{source_index}/states
GET   /api/curation/episodes/{session_id}/{source_index}/frame/{n}
```

Construct the player library as:

```python
legacy_library = ReplayLibrary(replay_root)
library = CuratedReplayLibrary(catalog, legacy=legacy_library)
player = PhysicalReplay(bridge, library)
```

Update `/api/replay/play` to accept `session_id`, `episode`, `raw`, and `speed`.
When `session_id` is present, build `EpisodeRef(session_id, episode, raw)`;
otherwise pass the legacy integer episode through. Keep the legacy
single-dataset endpoints wired to `legacy_library` until Task 7 no longer calls
them.

- [ ] **Step 6: Run replay safety regression and commit**

Run:

```bash
pixi run python tests/test_web_api.py
pixi run lint
```

Expected: all visual and physical replay tests pass.

```bash
git add src/elrobot/web/curation.py src/elrobot/web/replay.py src/elrobot/web/server.py tests/test_web_api.py
git commit -m "feat: replay curated episode ranges"
```

---

### Task 6: Immutable LeRobot v3 export

**Files:**
- Create: `src/elrobot/web/export.py`
- Create: `tests/test_export.py`
- Modify: `src/elrobot/web/server.py`
- Modify: `pixi.toml`

**Interfaces:**
- Produces: `ExportError`, `ExportBuilder(catalog)`, `build(name, task_ids)`,
  `preview(name, task_ids)`, `ExportService.start(name, task_ids)`, and
  `ExportService.status(export_id)`.
- HTTP: `POST /api/exports/preview`, `POST /api/exports`, and
  `GET /api/exports/{export_id}`.

- [ ] **Step 1: Write a failing multi-task export test**

Import `hashlib`, `json`, `tempfile`, `Path`, `numpy as np`, and
`LeRobotDataset`. Add one helper that creates real, tiny LeRobot datasets:

```python
def tiny_raw(root, tasks):
    features = {
        "observation.state": {
            "dtype": "float32", "shape": (8,),
            "names": {"motors": [f"joint_{n}" for n in range(8)]},
        },
        "action": {
            "dtype": "float32", "shape": (8,),
            "names": {"motors": [f"joint_{n}" for n in range(8)]},
        },
        "observation.images.cam_1": {
            "dtype": "video", "shape": (16, 16, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.images.cam_2": {
            "dtype": "video", "shape": (16, 16, 3),
            "names": ["height", "width", "channels"],
        },
    }
    ds = LeRobotDataset.create(
        repo_id=f"local/{root.name}",
        root=root,
        fps=30,
        features=features,
        robot_type="elrobot",
        video_backend="pyav",
        streaming_encoding=True,
        encoder_threads=2,
    )
    for task in tasks:
        for frame in range(6):
            image = np.full((16, 16, 3), frame * 20, dtype=np.uint8)
            vector = np.full(8, frame, dtype=np.float32)
            ds.add_frame({
                "observation.state": vector,
                "action": vector,
                "observation.images.cam_1": image,
                "observation.images.cam_2": image,
                "task": task,
            })
        ds.save_episode()
    ds.finalize()
```

Create two finalized raw session datasets with this helper. Register session A
source index `0` as kept in task A. Register session B source index `0` as
rejected and source index `1` as kept with trim `[1, 4)` and reassigned to task
B. Then:

```python
expected_refs = [
    (session_a["id"], 0),
    (session_b["id"], 1),
]
record = ExportBuilder(catalog).build("training", [task_a["id"], task_b["id"]])
assert record["state"] == "complete"
out = LeRobotDataset(
    repo_id=record["repo_id"],
    root=record["root"],
    video_backend="pyav",
)
assert out.num_episodes == 2
assert out.meta.episodes[0]["length"] == 6
assert out.meta.episodes[1]["length"] == 3
manifest = json.loads((Path(record["root"]) / "curation-manifest.json").read_text())
assert [(s["session_id"], s["source_index"])
        for s in manifest["sources"]] == expected_refs
```

Also test zero kept episodes, unknown task IDs, schema mismatch, one-active-export
guard, immutable `v001`/`v002` allocation, and raw-tree hashes before and after
export:

```python
def tree_hashes(root):
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*")) if path.is_file()
    }

raw_root = Path(session_a["root"])
before = tree_hashes(raw_root)
ExportBuilder(catalog).build("immutable-check", [task_a["id"]])
assert tree_hashes(raw_root) == before
```

This covers the exact finalized source tree used by the export.

- [ ] **Step 2: Run the export suite and verify import failure**

Run:

```bash
pixi run python tests/test_export.py
```

Expected: import failure because `elrobot.web.export` does not exist.

- [ ] **Step 3: Implement frame normalization and selection**

Use:

```python
GENERATED_KEYS = {
    "timestamp", "frame_index", "episode_index", "index", "task_index",
}


def _rgb_u8(value):
    arr = np.asarray(value)
    if arr.ndim == 3 and arr.shape[0] in (1, 3):
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0, 1) * 255).astype(np.uint8)
    return np.ascontiguousarray(arr)
```

Copy recordable features from the first source metadata, excluding generated
keys. Validate all selected sessions have the same FPS, feature names, shapes,
and dtypes.

For every selected kept episode, read only its effective range, convert video
features with `_rgb_u8`, copy state/action arrays, assign the effective current
task instruction, and call `save_episode()` once per source episode.

- [ ] **Step 4: Implement versioning, finalization, and manifest**

Sanitize names to `[a-z0-9][a-z0-9_-]*`. Allocate the first absent
`exports/<name>-vNNN` and build under `<name>-vNNN.inprogress`.

Create the output with streaming encoding and two encoder threads. After
`finalize()`, reload it and verify episode/frame counts. Write:

```json
{
  "catalog_revision": 12,
  "created_at": "2026-07-28T10:00:00+00:00",
  "tasks": {},
  "sources": [
    {
      "session_id": "session_abc123",
      "source_index": 0,
      "start_frame": 1,
      "end_frame_exclusive": 4,
      "task_id": "task_def456"
    }
  ]
}
```

Atomically rename the in-progress directory to the final version only after all
checks pass. Never overwrite an existing final path.

- [ ] **Step 5: Add the single-worker service and API**

`ExportService` owns one lock and one daemon worker thread. `start()` rejects a
second running export with `409`, records `queued/running/complete/failed`, and
returns immediately. `status()` returns the catalog export record.

`ExportBuilder.preview(name, task_ids)` performs the same name, task, selection,
and compatibility validation without writing anything and returns:

```python
{
    "name": sanitized_name,
    "next_version": next_version,
    "kept_episodes": len(selected),
    "frames": sum(item["end"] - item["start"] for item in selected),
    "seconds": total_frames / fps,
}
```

Add all three HTTP routes and map `ExportError` to its status code.

- [ ] **Step 6: Run export and Web API tests**

Run:

```bash
pixi run python tests/test_export.py
pixi run python tests/test_web_api.py
pixi run lint
```

Expected: all exit zero.

- [ ] **Step 7: Commit export support**

```bash
git add src/elrobot/web/export.py tests/test_export.py src/elrobot/web/server.py pixi.toml
git commit -m "feat: export curated lerobot datasets"
```

---

### Task 7: Full-workspace Collect and Curate UI

**Files:**
- Modify: `src/elrobot/web/static/index.html`
- Modify: `src/elrobot/web/static/app.js`
- Modify: `src/elrobot/web/static/style.css`
- Modify: `tests/test_web_api.py`

**Interfaces:**
- Consumes: task, collection, curation, replay, and export APIs from Tasks 4-6.
- Produces: top-level `Teleop` and `Curate` modes; `Calibrate` and `Collect` tabs inside Teleop; full-workspace curation controls.

- [ ] **Step 1: Add a failing static UI contract test**

Add:

```python
def test_collection_and_curate_shell_is_served():
    c = TestClient(create_app(FakeBridge(),
                              collection_root=Path(tempfile.mkdtemp())))
    html = c.get("/").text
    for element_id in (
        "mode-teleop", "mode-curate", "task-select", "session-start",
        "episode-start", "session-finish", "curate-task-list",
        "curate-episode-list", "curate-keep", "curate-reject",
        "curate-trim-start", "curate-trim-end", "curate-view-raw",
        "export-open",
    ):
        assert f'id="{element_id}"' in html
```

Run `pixi run python tests/test_web_api.py`; expect failure on the first missing
ID.

- [ ] **Step 2: Add top-level modes without duplicating cameras or the 3D scene**

Keep one `.stage` and one `#scene3d`. Add Teleop/Curate buttons in the header and
two sidebars inside the existing `main`. Keep the current `.stage`, `.deck`, and
`.card.rail` nodes in their current order and add these exact sidebars around
them:

```html
<aside class="curate-left" aria-label="Curation browser">
  <div id="curate-task-list"></div>
  <div id="curate-episode-list"></div>
</aside>
<aside class="curate-right" aria-label="Episode review">
  <button id="curate-keep" type="button">Keep</button>
  <button id="curate-reject" type="button">Reject</button>
  <label>Trim start <input id="curate-trim-start" type="number" min="0"></label>
  <label>Trim end <input id="curate-trim-end" type="number" min="1"></label>
  <label><input id="curate-view-raw" type="checkbox"> View raw</label>
  <button id="export-open" type="button">Export</button>
</aside>
```

Set `<main id="workspace" data-mode="teleop">` on the existing main element.

CSS uses:

```css
#workspace[data-mode="teleop"] {
  grid-template-columns: minmax(0, 1fr) 340px 300px;
}
#workspace[data-mode="curate"] {
  grid-template-columns: 280px minmax(0, 1fr) 320px;
}
#workspace[data-mode="curate"] .deck,
#workspace[data-mode="curate"] .rail { display: none; }
#workspace[data-mode="teleop"] .curate-left,
#workspace[data-mode="teleop"] .curate-right { display: none; }
```

Do not create a second renderer or duplicate camera polling.

- [ ] **Step 3: Replace the old record card with session collection controls**

Rename the Teleop tab from Replay to Collect. Add saved-task selection/create,
optional session name, Start collection, Record episode, Stop & keep, Discard,
and Finish session controls.

Drive enabled state only from the WebSocket `collection` snapshot:

```javascript
function renderCollection(s) {
  const ready = s.state === "ready";
  const recording = s.state === "recording";
  sessionStart.disabled = s.state !== "idle";
  episodeStart.disabled = !ready;
  episodeStop.disabled = !recording;
  episodeDiscard.disabled = !recording;
  sessionFinish.disabled = !ready;
  taskSelect.disabled = recording;
}
```

Refresh tasks after create/edit/archive and allow task changes only while ready.

- [ ] **Step 4: Build the Curate browser and reversible edit controls**

On mode entry, fetch sessions and effective task groups. Selecting an episode:

1. stops and disarms physical replay;
2. fetches curated states;
3. loads frame zero into the existing cameras and 3D scene;
4. fills review, task, trim, and notes controls.

Every edit sends one `PATCH` with the complete changed field. The raw toggle
reloads the selected episode with `raw=true`. Use number inputs bounded to
`0..frames`, with end exclusive and `start < end`.

Keep visual replay end behavior:

```javascript
if (replay.frame >= replay.states.length - 1) {
  stopReplay();
  showFrame(0);
  return;
}
```

- [ ] **Step 5: Adapt physical replay and export controls**

Send:

```javascript
{
  session_id: selected.session_id,
  episode: selected.source_index,
  raw: viewRaw.checked,
  speed: Number(physSpeed.value),
}
```

Keep speed default `0.6`. Mode exit, selection change, trim change, and raw
toggle must call `/api/replay/stop` then `/api/replay/arm {"on": false}`.

The export dialog lists task groups, kept counts, total curated duration, output
name, and next version. Fetch that validation and version from
`POST /api/exports/preview`, then start the confirmed export. Poll
`/api/exports/{id}` until complete or failed and show the final local path.

- [ ] **Step 6: Preserve accessibility and hidden-tab performance**

Use real buttons, labels, `role=tab`, `aria-selected`, `aria-controls`,
`aria-live` status, keyboard left/right navigation, and visible focus rings.
Continue pausing live camera fetches and 3D rendering while `document.hidden`.

- [ ] **Step 7: Run API tests and browser smoke check**

Run:

```bash
pixi run python tests/test_web_api.py
pixi run lint
```

Then start only one server after checking the port:

```bash
ss -tlnp | grep 8080
pixi run web
```

Verify Teleop/Curate switching, task creation, session button states, curation
scrub/reset, raw toggle, and export confirmation in the browser. Do not start a
second server if port 8080 is already owned.

- [ ] **Step 8: Commit the cockpit workflow**

```bash
git add src/elrobot/web/static/index.html src/elrobot/web/static/app.js src/elrobot/web/static/style.css tests/test_web_api.py
git commit -m "feat: add collection and curation cockpit"
```

---

### Task 8: Documentation and complete verification

**Files:**
- Modify: `docs/web-cockpit-guide.md`
- Modify: `AGENTS.md`
- Modify: `docs/superpowers/specs/2026-07-28-collection-curation-design.md`

**Interfaces:**
- Consumes: all completed collection/curation behavior.
- Produces: operator workflow and final implementation status.

- [ ] **Step 1: Update the operator guide**

Document:

- collection root and `COLLECTION_ROOT`;
- saved task creation/archive;
- session start, episode keep/discard, task changes between episodes, and finish;
- recovery actions;
- unreviewed/kept/rejected semantics;
- task reassignment, exclusive-end trim, notes, and raw toggle;
- visual versus armed physical replay;
- local versioned export paths and source manifest;
- explicit statement that training and inference are not part of this milestone.

- [ ] **Step 2: Update repository guidance**

Update the `web` task description in `AGENTS.md` to mention managed collection,
curation, and export. Add the new files to the architecture notes without
changing any hardware safety rule.

- [ ] **Step 3: Run the complete offline gates**

Run:

```bash
pixi run prove-env
pixi run lint
pixi run test
```

Expected: every command exits zero.

- [ ] **Step 4: Re-run the automated raw-immutability check**

Run:

```bash
pixi run python tests/test_export.py
```

Expected: `EXPORT TESTS PASSED`, including identical raw-tree hashes before and
after curation/export.

Now set the collection/curation spec status to:

```markdown
**Status:** Implemented; offline verification passed; physical operator check pending
```

- [ ] **Step 5: Hand the physical operator check to the user**

Do not move the physical arm automatically. Give the user this checklist:

1. Start the existing teleop/driver and camera stacks.
2. Start `pixi run web`; confirm the printed Cockpit URL.
3. Collect and finalize one short task-labelled session.
4. Curate a shorter range and replay it visually.
5. Arm physical replay, seek, run at `0.6`, and press STOP.
6. Confirm the driver deadman freezes within its existing 200 ms contract.
7. Export and load the local LeRobot v3 output.

Do not alter any measured safety threshold during this check.
After the user completes this checklist, make a later documentation-only update
to mark the spec fully verified.

- [ ] **Step 6: Commit documentation**

```bash
git add docs/web-cockpit-guide.md AGENTS.md docs/superpowers/specs/2026-07-28-collection-curation-design.md
git commit -m "docs: document collection and curation workflow"
```
