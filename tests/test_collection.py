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


def main():
    test_tasks_have_stable_identity_and_archive()
    test_episode_edits_are_reversible_overlays()
    test_trim_and_review_validation()
    test_catalog_write_is_reloadable_and_revisioned()
    test_export_records_share_atomic_catalog_path()
    print("COLLECTION TESTS PASSED")


if __name__ == "__main__":
    main()
