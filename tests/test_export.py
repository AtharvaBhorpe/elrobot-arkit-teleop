"""Curated LeRobot export tests — real tiny datasets, no ROS or hardware."""

import hashlib
import json
import tempfile
import threading
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from elrobot.web.collection import CollectionCatalog
from elrobot.web.export import ExportBuilder, ExportError, ExportService


def tiny_raw(root, repo_id, tasks, fps=30):
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
            "dtype": "video", "shape": (48, 64, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.images.cam_2": {
            "dtype": "video", "shape": (48, 64, 3),
            "names": ["height", "width", "channels"],
        },
    }
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        fps=fps,
        features=features,
        robot_type="elrobot",
        video_backend="pyav",
        streaming_encoding=True,
        encoder_threads=2,
    )
    for task in tasks:
        for frame in range(6):
            image = np.full((48, 64, 3), frame * 20, dtype=np.uint8)
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


def tree_hashes(root):
    root = Path(root)
    return {
        str(path.relative_to(root)): hashlib.sha256(
            path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def curated_catalog():
    catalog = CollectionCatalog(
        Path(tempfile.mkdtemp()) / "collections")
    task_a = catalog.create_task("Pick", "Pick up the cube.")
    task_b = catalog.create_task("Place", "Place the cube.")

    session_a = catalog.create_session("morning")
    tiny_raw(
        Path(session_a["root"]), session_a["repo_id"],
        [task_a["instruction"]],
    )
    catalog.set_pending(session_a["id"], task_a["id"])
    catalog.commit_episode(session_a["id"], 0, 6)
    catalog.update_episode(
        session_a["id"], 0, {"review": "kept"})
    catalog.finalize_session(session_a["id"], "ready")

    session_b = catalog.create_session("afternoon")
    tiny_raw(
        Path(session_b["root"]), session_b["repo_id"],
        [task_a["instruction"], task_a["instruction"]],
    )
    for source_index in range(2):
        catalog.set_pending(session_b["id"], task_a["id"])
        catalog.commit_episode(session_b["id"], source_index, 6)
    catalog.update_episode(
        session_b["id"], 0, {"review": "rejected"})
    catalog.update_episode(session_b["id"], 1, {
        "review": "kept",
        "task_id": task_b["id"],
        "trim": {"start_frame": 1, "end_frame_exclusive": 4},
    })
    catalog.finalize_session(session_b["id"], "ready")
    return catalog, task_a, task_b, session_a, session_b


def test_multi_task_export_is_loadable_and_traceable():
    catalog, task_a, task_b, session_a, session_b = curated_catalog()
    expected_refs = [
        (session_a["id"], 0),
        (session_b["id"], 1),
    ]
    preview = ExportBuilder(catalog).preview(
        "Training Set", [task_a["id"], task_b["id"]])
    assert preview["name"] == "training-set"
    assert preview["kept_episodes"] == 2
    assert preview["frames"] == 9
    assert preview["next_version"] == 1

    record = ExportBuilder(catalog).build(
        "Training Set", [task_a["id"], task_b["id"]])
    assert record["state"] == "complete"
    out = LeRobotDataset(
        repo_id=record["repo_id"],
        root=record["root"],
        video_backend="pyav",
    )
    assert out.num_episodes == 2
    assert out.meta.episodes[0]["length"] == 6
    assert out.meta.episodes[1]["length"] == 3
    manifest = json.loads(
        (Path(record["root"]) / "curation-manifest.json").read_text())
    assert [
        (source["session_id"], source["source_index"])
        for source in manifest["sources"]
    ] == expected_refs
    assert manifest["tasks"][task_b["id"]]["instruction"] == (
        task_b["instruction"])


def test_export_rejects_empty_unknown_and_incompatible_selection():
    catalog, task_a, _, _, _ = curated_catalog()
    empty_task = catalog.create_task("Unused", "Do nothing.")
    try:
        ExportBuilder(catalog).preview("empty", [empty_task["id"]])
    except ExportError as exc:
        assert exc.code == 422
    else:
        raise AssertionError("accepted a selection with no kept episodes")

    try:
        ExportBuilder(catalog).preview("unknown", ["task_missing"])
    except ExportError as exc:
        assert exc.code == 422
    else:
        raise AssertionError("accepted an unknown task")

    incompatible = catalog.create_session("incompatible")
    tiny_raw(
        Path(incompatible["root"]), incompatible["repo_id"],
        ["Pick up the cube."], fps=20,
    )
    catalog.set_pending(incompatible["id"], task_a["id"])
    catalog.commit_episode(incompatible["id"], 0, 6)
    catalog.update_episode(
        incompatible["id"], 0, {"review": "kept"})
    catalog.finalize_session(incompatible["id"], "ready")
    try:
        ExportBuilder(catalog).preview("mixed", [task_a["id"]])
    except ExportError as exc:
        assert "compatible" in exc.detail
    else:
        raise AssertionError("accepted incompatible source schemas")


def test_exports_are_versioned_and_leave_raw_unchanged():
    catalog, task_a, _, session_a, _ = curated_catalog()
    before = tree_hashes(session_a["root"])
    first = ExportBuilder(catalog).build(
        "immutable-check", [task_a["id"]])
    second = ExportBuilder(catalog).build(
        "immutable-check", [task_a["id"]])
    assert Path(first["root"]).name == "immutable-check-v001"
    assert Path(second["root"]).name == "immutable-check-v002"
    assert tree_hashes(session_a["root"]) == before


def test_export_service_allows_one_active_worker():
    catalog, task_a, _, _, _ = curated_catalog()
    entered = threading.Event()
    release = threading.Event()

    class SlowBuilder:
        def build(self, name, task_ids, record=None):
            entered.set()
            release.wait(5)
            return catalog.update_export(
                record["id"], {"state": "complete"})

    service = ExportService(catalog, builder=SlowBuilder())
    started = service.start("training", [task_a["id"]])
    assert entered.wait(2)
    try:
        service.start("another", [task_a["id"]])
    except ExportError as exc:
        assert exc.code == 409
    else:
        raise AssertionError("started two exports")
    release.set()
    service.join(5)
    assert service.status(started["id"])["state"] == "complete"


def main():
    test_multi_task_export_is_loadable_and_traceable()
    test_export_rejects_empty_unknown_and_incompatible_selection()
    test_exports_are_versioned_and_leave_raw_unchanged()
    test_export_service_allows_one_active_worker()
    print("EXPORT TESTS PASSED")


if __name__ == "__main__":
    main()
