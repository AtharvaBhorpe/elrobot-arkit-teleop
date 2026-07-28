"""Atomic task, session, curation, and export catalog."""

from __future__ import annotations

import copy
import json
import os
import tempfile
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

EMPTY_CATALOG = {
    "schema_version": 1,
    "revision": 0,
    "tasks": {},
    "sessions": {},
    "exports": {},
}
REVIEWS = {"unreviewed", "kept", "rejected"}
SESSION_STATES = {
    "active", "ready", "recoverable", "archived_empty",
    "archived_incomplete",
}


class CatalogError(RuntimeError):
    def __init__(self, detail: str, code: int = 422):
        super().__init__(detail)
        self.detail = detail
        self.code = code


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex}"


class CollectionCatalog:
    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.path = self.root / "catalog.json"
        self._lock = threading.RLock()
        if self.path.exists():
            try:
                self._data = json.loads(self.path.read_text())
            except (OSError, json.JSONDecodeError) as exc:
                raise CatalogError(f"cannot read catalog: {exc}") from exc
            self._validate(self._data)
        else:
            self._data = copy.deepcopy(EMPTY_CATALOG)

    def snapshot(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._data)

    def _persist(self, data: dict) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(
            prefix=".catalog-", suffix=".json", dir=self.root)
        try:
            with os.fdopen(fd, "w") as fh:
                json.dump(data, fh, indent=2, sort_keys=True)
                fh.write("\n")
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self.path)
        finally:
            Path(tmp).unlink(missing_ok=True)

    def _mutate(self, change):
        with self._lock:
            data = copy.deepcopy(self._data)
            result = change(data)
            self._validate(data)
            data["revision"] += 1
            self._persist(data)
            self._data = data
            return copy.deepcopy(result)

    @staticmethod
    def _validate(data: dict) -> None:
        if data.get("schema_version") != 1:
            raise CatalogError("unsupported catalog schema")
        if not isinstance(data.get("revision"), int):
            raise CatalogError("invalid catalog revision")
        for key in ("tasks", "sessions", "exports"):
            if not isinstance(data.get(key), dict):
                raise CatalogError(f"invalid catalog {key}")
        tasks = data["tasks"]
        for session in data["sessions"].values():
            pending = session.get("pending_episode")
            if pending and pending.get("source_task_id") not in tasks:
                raise CatalogError("pending episode references an unknown task")
            for episode in session.get("episodes", {}).values():
                if episode.get("source_task_id") not in tasks:
                    raise CatalogError("episode references an unknown source task")
                effective = episode.get("task_id")
                if effective is not None and effective not in tasks:
                    raise CatalogError("episode references an unknown task")

    @staticmethod
    def _task(data: dict, task_id: str) -> dict:
        try:
            return data["tasks"][task_id]
        except KeyError as exc:
            raise CatalogError(f"unknown task {task_id}") from exc

    @staticmethod
    def _session(data: dict, session_id: str) -> dict:
        try:
            return data["sessions"][session_id]
        except KeyError as exc:
            raise CatalogError(f"unknown session {session_id}") from exc

    @staticmethod
    def _episode(session: dict, source_index: int) -> dict:
        try:
            return session["episodes"][str(int(source_index))]
        except (KeyError, TypeError, ValueError) as exc:
            raise CatalogError(
                f"unknown episode {session['id']}:{source_index}") from exc

    def create_task(self, name: str, instruction: str) -> dict:
        name, instruction = name.strip(), instruction.strip()
        if not name or not instruction:
            raise CatalogError("task name and instruction are required")

        def change(data):
            now = _now()
            task = {
                "id": _id("task"),
                "name": name,
                "instruction": instruction,
                "archived": False,
                "created_at": now,
                "updated_at": now,
            }
            data["tasks"][task["id"]] = task
            return task

        return self._mutate(change)

    def update_task(
        self, task_id: str, *, name=None, instruction=None, archived=None,
    ) -> dict:
        def change(data):
            task = self._task(data, task_id)
            if name is not None:
                value = str(name).strip()
                if not value:
                    raise CatalogError("task name is required")
                task["name"] = value
            if instruction is not None:
                value = str(instruction).strip()
                if not value:
                    raise CatalogError("task instruction is required")
                task["instruction"] = value
            if archived is not None:
                if not isinstance(archived, bool):
                    raise CatalogError("archived must be a boolean")
                task["archived"] = archived
            task["updated_at"] = _now()
            return task

        return self._mutate(change)

    def create_session(self, name: str) -> dict:
        def change(data):
            session_id = _id("session")
            session = {
                "id": session_id,
                "name": str(name).strip(),
                "repo_id": f"local/elrobot_{session_id}",
                "root": str(self.root / "raw" / session_id),
                "state": "active",
                "created_at": _now(),
                "finalized_at": None,
                "pending_episode": None,
                "episodes": {},
            }
            data["sessions"][session_id] = session
            return session

        return self._mutate(change)

    def set_pending(self, session_id: str, task_id: str) -> dict:
        def change(data):
            session = self._session(data, session_id)
            task = self._task(data, task_id)
            if session["pending_episode"] is not None:
                raise CatalogError("session already has a pending episode", 409)
            indices = [int(index) for index in session["episodes"]]
            pending = {
                "source_index": max(indices, default=-1) + 1,
                "source_task_id": task_id,
                "source_task_instruction": task["instruction"],
                "created_at": _now(),
            }
            session["pending_episode"] = pending
            return pending

        return self._mutate(change)

    def clear_pending(self, session_id: str) -> dict:
        def change(data):
            session = self._session(data, session_id)
            session["pending_episode"] = None
            return session

        return self._mutate(change)

    def commit_episode(
        self, session_id: str, source_index: int, frames: int,
        interrupted: bool = False,
    ) -> dict:
        if isinstance(frames, bool) or not isinstance(frames, int) or frames < 1:
            raise CatalogError("episode frames must be a positive integer")

        def change(data):
            session = self._session(data, session_id)
            pending = session["pending_episode"]
            if pending is None:
                raise CatalogError("session has no pending episode", 409)
            if pending["source_index"] != source_index:
                raise CatalogError("pending episode index mismatch", 409)
            key = str(source_index)
            if key in session["episodes"]:
                raise CatalogError("episode already exists", 409)
            episode = {
                "episode_id": f"{session_id}:{source_index}",
                "source_index": source_index,
                "source_task_id": pending["source_task_id"],
                "source_task_instruction": pending[
                    "source_task_instruction"],
                "frames": frames,
                "review": "unreviewed",
                "task_id": None,
                "trim": None,
                "notes": "",
                "interrupted": bool(interrupted),
            }
            session["episodes"][key] = episode
            session["pending_episode"] = None
            return episode

        return self._mutate(change)

    def finalize_session(self, session_id: str, state: str) -> dict:
        if state not in SESSION_STATES:
            raise CatalogError(f"invalid session state {state}")

        def change(data):
            session = self._session(data, session_id)
            session["state"] = state
            if state in {"ready", "archived_empty", "archived_incomplete"}:
                session["finalized_at"] = _now()
            return session

        return self._mutate(change)

    def update_episode(
        self, session_id: str, source_index: int, patch: dict,
    ) -> dict:
        if not isinstance(patch, dict):
            raise CatalogError("episode patch must be an object")
        unknown = set(patch) - {"review", "task_id", "trim", "notes"}
        if unknown:
            raise CatalogError(
                f"unknown episode fields: {', '.join(sorted(unknown))}")

        def change(data):
            session = self._session(data, session_id)
            episode = self._episode(session, source_index)
            if "review" in patch:
                if patch["review"] not in REVIEWS:
                    raise CatalogError("invalid review state")
                episode["review"] = patch["review"]
            if "task_id" in patch:
                task_id = patch["task_id"]
                if task_id is not None:
                    self._task(data, task_id)
                episode["task_id"] = task_id
            if "trim" in patch:
                trim = patch["trim"]
                if trim is not None:
                    if not isinstance(trim, dict) or set(trim) != {
                        "start_frame", "end_frame_exclusive",
                    }:
                        raise CatalogError("invalid trim")
                    start = trim["start_frame"]
                    end = trim["end_frame_exclusive"]
                    if (
                        isinstance(start, bool) or isinstance(end, bool)
                        or not isinstance(start, int)
                        or not isinstance(end, int)
                        or not 0 <= start < end <= episode["frames"]
                    ):
                        raise CatalogError("invalid trim bounds")
                    trim = {
                        "start_frame": start,
                        "end_frame_exclusive": end,
                    }
                episode["trim"] = trim
            if "notes" in patch:
                if not isinstance(patch["notes"], str):
                    raise CatalogError("notes must be a string")
                episode["notes"] = patch["notes"]
            return episode

        return self._mutate(change)

    def sessions(self) -> list[dict]:
        with self._lock:
            sessions = sorted(
                self._data["sessions"].values(),
                key=lambda item: item["created_at"],
            )
            return copy.deepcopy(sessions)

    def session(self, session_id: str) -> dict:
        with self._lock:
            return copy.deepcopy(self._session(self._data, session_id))

    def tasks(self, include_archived: bool = True) -> list[dict]:
        with self._lock:
            tasks = self._data["tasks"].values()
            if not include_archived:
                tasks = (task for task in tasks if not task["archived"])
            return copy.deepcopy(sorted(tasks, key=lambda item: item["name"]))

    def task(self, task_id: str) -> dict:
        with self._lock:
            return copy.deepcopy(self._task(self._data, task_id))

    def episode(self, session_id: str, source_index: int) -> dict:
        with self._lock:
            session = self._session(self._data, session_id)
            return copy.deepcopy(self._episode(session, source_index))

    def task_groups(self) -> list[dict]:
        with self._lock:
            groups = {}
            for session in self._data["sessions"].values():
                if session["state"] != "ready":
                    continue
                for episode in session["episodes"].values():
                    task_id = episode["task_id"] or episode["source_task_id"]
                    group = groups.setdefault(task_id, {
                        "task": copy.deepcopy(self._data["tasks"][task_id]),
                        "episodes": 0,
                        "kept": 0,
                        "frames": 0,
                    })
                    trim = episode["trim"]
                    frames = (
                        trim["end_frame_exclusive"] - trim["start_frame"]
                        if trim else episode["frames"]
                    )
                    group["episodes"] += 1
                    group["frames"] += frames
                    group["kept"] += episode["review"] == "kept"
            return list(groups.values())

    def create_export(self, record: dict) -> dict:
        if not isinstance(record, dict) or not str(record.get("id", "")).strip():
            raise CatalogError("export id is required")

        def change(data):
            export_id = record["id"]
            if export_id in data["exports"]:
                raise CatalogError(f"export {export_id} already exists", 409)
            data["exports"][export_id] = copy.deepcopy(record)
            return data["exports"][export_id]

        return self._mutate(change)

    def update_export(self, export_id: str, patch: dict) -> dict:
        if not isinstance(patch, dict):
            raise CatalogError("export patch must be an object")

        def change(data):
            try:
                record = data["exports"][export_id]
            except KeyError as exc:
                raise CatalogError(f"unknown export {export_id}") from exc
            record.update(copy.deepcopy(patch))
            return record

        return self._mutate(change)

    def export(self, export_id: str) -> dict:
        with self._lock:
            try:
                return copy.deepcopy(self._data["exports"][export_id])
            except KeyError as exc:
                raise CatalogError(f"unknown export {export_id}") from exc
