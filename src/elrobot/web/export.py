"""Immutable LeRobot v3 exports from reversible curation overlays."""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import threading
import uuid
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from elrobot.web.collection import CatalogError

os.environ.setdefault("HF_HUB_OFFLINE", "1")

GENERATED_KEYS = {
    "timestamp", "frame_index", "episode_index", "index", "task_index",
}


class ExportError(RuntimeError):
    def __init__(self, detail: str, code: int = 422):
        super().__init__(detail)
        self.detail = detail
        self.code = code


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _rgb_u8(value):
    arr = np.asarray(value)
    if arr.ndim == 3 and arr.shape[0] in (1, 3):
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0, 1) * 255).astype(np.uint8)
    return np.ascontiguousarray(arr)


def _scalar(value) -> int:
    arr = np.asarray(value).reshape(-1)
    return int(arr[0])


def _source_tree_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        digest.update(str(path.relative_to(root)).encode())
        digest.update(b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


class ExportBuilder:
    def __init__(self, catalog):
        self.catalog = catalog

    @staticmethod
    def _name(name: str) -> str:
        value = re.sub(r"[^a-z0-9_-]+", "-", str(name).strip().lower())
        value = value.strip("-_")
        if not value or not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", value):
            raise ExportError(
                "dataset name must contain letters or numbers")
        return value

    def _version(self, name: str) -> tuple[int, Path, Path]:
        exports = self.catalog.root / "exports"
        version = 1
        while True:
            final = exports / f"{name}-v{version:03d}"
            staging = exports / f"{name}-v{version:03d}.inprogress"
            if not final.exists() and not staging.exists():
                return version, final, staging
            version += 1

    @staticmethod
    def _recordable_features(dataset) -> dict:
        features = {}
        for key, feature in dataset.features.items():
            if key in GENERATED_KEYS:
                continue
            features[key] = {
                field: deepcopy(feature[field])
                for field in ("dtype", "shape", "names")
                if field in feature
            }
        return features

    @staticmethod
    def _signature(dataset) -> tuple:
        features = ExportBuilder._recordable_features(dataset)
        normalized = tuple(sorted(
            (
                key,
                value.get("dtype"),
                tuple(value.get("shape", ())),
                repr(value.get("names")),
            )
            for key, value in features.items()
        ))
        return int(dataset.fps), normalized

    @staticmethod
    def _bounds(dataset, source_index: int) -> tuple[int, int]:
        for row in dataset.meta.episodes:
            if _scalar(row["episode_index"]) == source_index:
                return (
                    _scalar(row["dataset_from_index"]),
                    _scalar(row["dataset_to_index"]),
                )
        raise ExportError(f"raw dataset has no episode {source_index}")

    def _prepare(self, name: str, task_ids) -> dict:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        clean_name = self._name(name)
        if not isinstance(task_ids, list) or not task_ids:
            raise ExportError("select at least one task")
        snapshot = self.catalog.snapshot()
        if snapshot.get("schema_version") != 1:
            raise ExportError("unsupported catalog schema")

        selected_task_ids = list(dict.fromkeys(map(str, task_ids)))
        try:
            tasks = {
                task_id: deepcopy(snapshot["tasks"][task_id])
                for task_id in selected_task_ids
            }
        except KeyError as exc:
            raise ExportError(f"unknown task {exc.args[0]}") from exc

        selections = []
        opened = {}
        signature = None
        features = None
        fps = None
        sessions = sorted(
            snapshot["sessions"].values(),
            key=lambda item: item["created_at"],
        )
        for session in sessions:
            if session["state"] != "ready":
                continue
            for episode in sorted(
                session["episodes"].values(),
                key=lambda item: item["source_index"],
            ):
                effective_task = (
                    episode["task_id"] or episode["source_task_id"])
                if (
                    episode["review"] != "kept"
                    or effective_task not in tasks
                ):
                    continue
                dataset = opened.get(session["id"])
                if dataset is None:
                    try:
                        dataset = LeRobotDataset(
                            repo_id=session["repo_id"],
                            root=session["root"],
                            video_backend="pyav",
                        )
                    except Exception as exc:                    # noqa: BLE001
                        raise ExportError(
                            f"cannot open session {session['id']}: {exc}"
                        ) from exc
                    opened[session["id"]] = dataset
                current = self._signature(dataset)
                if signature is None:
                    signature = current
                    fps = int(dataset.fps)
                    features = self._recordable_features(dataset)
                elif current != signature:
                    raise ExportError(
                        "selected sessions do not have compatible schemas")

                raw_start, raw_end = self._bounds(
                    dataset, episode["source_index"])
                raw_frames = raw_end - raw_start
                if raw_frames != episode["frames"]:
                    raise ExportError(
                        f"catalog/raw frame mismatch for "
                        f"{episode['episode_id']}")
                trim = episode["trim"]
                start = 0 if trim is None else trim["start_frame"]
                end = (
                    raw_frames
                    if trim is None else trim["end_frame_exclusive"]
                )
                selections.append({
                    "session": session,
                    "episode": episode,
                    "task_id": effective_task,
                    "dataset": dataset,
                    "raw_start": raw_start,
                    "start": start,
                    "end": end,
                })

        if not selections:
            raise ExportError(
                "selection contains no explicitly kept episodes")
        version, final, staging = self._version(clean_name)
        return {
            "name": clean_name,
            "version": version,
            "final": final,
            "staging": staging,
            "repo_id": f"local/{clean_name}-v{version:03d}",
            "catalog_revision": snapshot["revision"],
            "tasks": tasks,
            "selections": selections,
            "features": features,
            "fps": fps,
        }

    def preview(self, name: str, task_ids) -> dict:
        prepared = self._prepare(name, task_ids)
        frames = sum(
            item["end"] - item["start"]
            for item in prepared["selections"]
        )
        return {
            "name": prepared["name"],
            "next_version": prepared["version"],
            "kept_episodes": len(prepared["selections"]),
            "frames": frames,
            "seconds": frames / prepared["fps"],
        }

    def build(self, name: str, task_ids, record=None) -> dict:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        prepared = self._prepare(name, task_ids)
        export_id = (
            record["id"] if record is not None
            else f"export_{uuid.uuid4().hex}"
        )
        base_record = {
            "id": export_id,
            "name": prepared["name"],
            "version": prepared["version"],
            "repo_id": prepared["repo_id"],
            "root": str(prepared["final"]),
            "task_ids": list(prepared["tasks"]),
            "state": "running",
            "created_at": (
                record.get("created_at", _now()) if record else _now()),
            "completed_at": None,
            "error": None,
        }
        try:
            if record is None:
                self.catalog.create_export(base_record)
            else:
                self.catalog.update_export(export_id, base_record)

            prepared["staging"].parent.mkdir(parents=True, exist_ok=True)
            output = LeRobotDataset.create(
                repo_id=prepared["repo_id"],
                root=prepared["staging"],
                fps=prepared["fps"],
                features=prepared["features"],
                robot_type="elrobot",
                video_backend="pyav",
                streaming_encoding=True,
                encoder_threads=2,
            )
            sources = []
            source_hashes = {}
            total_frames = 0
            for item in prepared["selections"]:
                dataset = item["dataset"]
                instruction = prepared["tasks"][
                    item["task_id"]]["instruction"]
                lo = item["raw_start"] + item["start"]
                hi = item["raw_start"] + item["end"]
                for index in range(lo, hi):
                    source = dataset[index]
                    frame = {}
                    for key, feature in prepared["features"].items():
                        value = source[key]
                        frame[key] = (
                            _rgb_u8(value)
                            if feature["dtype"] in {"video", "image"}
                            else np.asarray(value)
                        )
                    frame["task"] = instruction
                    output.add_frame(frame)
                output.save_episode()
                total_frames += hi - lo
                session = item["session"]
                if session["id"] not in source_hashes:
                    source_hashes[session["id"]] = _source_tree_sha256(
                        Path(session["root"]))
                sources.append({
                    "session_id": session["id"],
                    "source_index": item["episode"]["source_index"],
                    "start_frame": item["start"],
                    "end_frame_exclusive": item["end"],
                    "task_id": item["task_id"],
                    "source_tree_sha256": source_hashes[session["id"]],
                })
            output.finalize()

            verified = LeRobotDataset(
                repo_id=prepared["repo_id"],
                root=prepared["staging"],
                video_backend="pyav",
            )
            if (
                verified.num_episodes != len(sources)
                or len(verified) != total_frames
            ):
                raise ExportError(
                    "export read-back count validation failed", 500)

            manifest = {
                "catalog_revision": prepared["catalog_revision"],
                "created_at": _now(),
                "tasks": prepared["tasks"],
                "sources": sources,
            }
            manifest_path = (
                prepared["staging"] / "curation-manifest.json")
            manifest_path.write_text(
                json.dumps(manifest, indent=2, sort_keys=True) + "\n")
            os.replace(prepared["staging"], prepared["final"])
            return self.catalog.update_export(export_id, {
                "state": "complete",
                "completed_at": _now(),
                "episodes": len(sources),
                "frames": total_frames,
            })
        except Exception as exc:
            if prepared["staging"].exists():
                shutil.rmtree(prepared["staging"])
            detail = exc.detail if isinstance(exc, ExportError) else str(exc)
            try:
                self.catalog.update_export(export_id, {
                    "state": "failed",
                    "completed_at": _now(),
                    "error": detail,
                })
            except CatalogError:
                pass
            if isinstance(exc, ExportError):
                raise
            raise ExportError(f"export failed: {detail}", 500) from exc


class ExportService:
    def __init__(self, catalog, builder=None):
        self.catalog = catalog
        self.builder = builder or ExportBuilder(catalog)
        self._lock = threading.Lock()
        self._thread = None

    def preview(self, name: str, task_ids) -> dict:
        return self.builder.preview(name, task_ids)

    def start(self, name: str, task_ids) -> dict:
        # Preview before allocating an ID so invalid requests do not create
        # failed jobs in the catalog.
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise ExportError("an export is already running", 409)
        preview = (
            self.builder.preview(name, task_ids)
            if hasattr(self.builder, "preview")
            else {"name": ExportBuilder._name(name)}
        )
        with self._lock:
            # A caller may have entered while this request was validating
            # source metadata outside the lock.
            if self._thread is not None and self._thread.is_alive():
                raise ExportError("an export is already running", 409)
            record = self.catalog.create_export({
                "id": f"export_{uuid.uuid4().hex}",
                "name": preview["name"],
                "task_ids": list(task_ids),
                "state": "queued",
                "created_at": _now(),
                "completed_at": None,
                "error": None,
            })

            def run():
                try:
                    self.catalog.update_export(
                        record["id"], {"state": "running"})
                    self.builder.build(name, task_ids, record=record)
                except Exception as exc:                       # noqa: BLE001
                    current = self.catalog.export(record["id"])
                    if current["state"] != "failed":
                        self.catalog.update_export(record["id"], {
                            "state": "failed",
                            "completed_at": _now(),
                            "error": str(exc),
                        })

            self._thread = threading.Thread(target=run, daemon=True)
            self._thread.start()
            return record

    def status(self, export_id: str) -> dict:
        try:
            return self.catalog.export(export_id)
        except CatalogError as exc:
            raise ExportError(exc.detail, exc.code) from exc

    def join(self, timeout=None) -> None:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
