"""Collection catalog tests — file-only, no ROS or hardware."""

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


def _validator(recorders):
    def validate(session, finalize):
        count = recorders[0].episodes if recorders else 0
        return {"count": count, "lengths": [12] * count}

    return validate


def test_collection_lifecycle_and_task_lock():
    from elrobot.web.collection_manager import CollectionError, CollectionManager

    c = catalog()
    a = c.create_task("Pick", "Pick.")
    b = c.create_task("Place", "Place.")
    made = []
    m = CollectionManager(
        c,
        recorder_factory=lambda _: made.append(FakeRecorder()) or made[-1],
        dataset_validator=_validator(made),
    )
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


def test_invalid_transitions_and_empty_finish():
    from elrobot.web.collection_manager import CollectionError, CollectionManager

    c = catalog()
    task = c.create_task("Pick", "Pick.")
    made = []
    m = CollectionManager(
        c,
        recorder_factory=lambda _: made.append(FakeRecorder()) or made[-1],
        dataset_validator=_validator(made),
    )
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
    made = []
    m = CollectionManager(
        c,
        recorder_factory=lambda _: made.append(FakeRecorder()) or made[-1],
        dataset_validator=_validator(made),
    )
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

    def validator(record, finalize):
        return {"count": 1, "lengths": [12]}

    m = CollectionManager(
        c,
        recorder_factory=lambda _: FakeRecorder(),
        dataset_validator=validator,
    )
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


def main():
    test_tasks_have_stable_identity_and_archive()
    test_episode_edits_are_reversible_overlays()
    test_trim_and_review_validation()
    test_catalog_write_is_reloadable_and_revisioned()
    test_export_records_share_atomic_catalog_path()
    test_collection_lifecycle_and_task_lock()
    test_invalid_transitions_and_empty_finish()
    test_finish_refuses_while_recording()
    test_recovery_reconciles_one_saved_pending_episode()
    test_archive_recovery_keeps_raw_files()
    print("COLLECTION TESTS PASSED")


if __name__ == "__main__":
    main()
