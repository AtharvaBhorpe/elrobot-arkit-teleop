"""Serialized cockpit-owned collection lifecycle and recovery."""

from __future__ import annotations

import argparse
import threading

from elrobot.web.collection import CatalogError


class CollectionError(RuntimeError):
    def __init__(self, detail: str, code: int = 409):
        super().__init__(detail)
        self.detail = detail
        self.code = code


class CollectionManager:
    def __init__(
        self,
        catalog,
        recorder_factory,
        add_node=lambda node: None,
        remove_node=lambda node: None,
        external_recorder_alive=lambda: False,
        dataset_validator=None,
    ):
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
        self._error = None

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "session_id": self._session_id,
                "task_id": self._task_id,
                "error": self._error,
            }

    def _require(self, state: str) -> None:
        if self._state != state:
            raise CollectionError(
                f"collection is {self._state}; expected {state}")

    def _task(self, task_id: str) -> dict:
        try:
            task = self.catalog.task(task_id)
        except CatalogError as exc:
            raise CollectionError(exc.detail, exc.code) from exc
        if task["archived"]:
            raise CollectionError("archived tasks cannot record new episodes", 422)
        return task

    def _detach_recorder(self) -> None:
        recorder, self._recorder = self._recorder, None
        if recorder is not None:
            try:
                self.remove_node(recorder)
            except Exception:  # noqa: BLE001
                pass

    def _idle(self) -> dict:
        self._state = "idle"
        self._session_id = None
        self._task_id = None
        self._recorder = None
        return self.snapshot()

    def start_session(self, task_id: str, name: str = "") -> dict:
        with self._lock:
            self._require("idle")
            task = self._task(task_id)
            if self.external_recorder_alive():
                raise CollectionError(
                    "an independent episode_recorder is already running")
            self._state = "starting"
            self._error = None
            session = self.catalog.create_session(name)
            self._session_id = session["id"]
            self._task_id = task_id
            args = argparse.Namespace(
                fps=30.0,
                repo_id=session["repo_id"],
                root=session["root"],
                task=task["instruction"],
                encoder_threads=2,
                start_timeout=10.0,
            )
            try:
                self._recorder = self.recorder_factory(args)
                self.add_node(self._recorder)
            except Exception as exc:  # noqa: BLE001
                self._detach_recorder()
                self.catalog.finalize_session(
                    session["id"], "archived_incomplete")
                self._idle()
                raise CollectionError(
                    f"cannot start collection: {exc}") from exc
            self._state = "ready"
            return self.snapshot()

    def start_episode(self, task_id: str) -> dict:
        with self._lock:
            self._require("ready")
            task = self._task(task_id)
            self._recorder.set_task(task["instruction"])
            pending = self.catalog.set_pending(self._session_id, task_id)
            try:
                started = self._recorder.start()
            except Exception as exc:  # noqa: BLE001
                self.catalog.clear_pending(self._session_id)
                raise CollectionError(f"cannot start episode: {exc}") from exc
            if not started:
                self.catalog.clear_pending(self._session_id)
                raise CollectionError("required recording streams are not fresh")
            self._task_id = pending["source_task_id"]
            self._state = "recording"
            return self.snapshot()

    def stop_episode(self) -> dict:
        with self._lock:
            self._require("recording")
            frames = self._recorder.stop()
            if frames < 1:
                self.catalog.clear_pending(self._session_id)
                self._state = "ready"
                raise CollectionError("episode contained no frames")
            pending = self.catalog.session(
                self._session_id)["pending_episode"]
            self.catalog.commit_episode(
                self._session_id, pending["source_index"], frames)
            self._state = "ready"
            return self.snapshot()

    def discard_episode(self) -> dict:
        with self._lock:
            self._require("recording")
            self._recorder.discard()
            self.catalog.clear_pending(self._session_id)
            self._state = "ready"
            return self.snapshot()

    def finish_session(self) -> dict:
        with self._lock:
            self._require("ready")
            self._state = "finalizing"
            session_id = self._session_id
            session = self.catalog.session(session_id)
            try:
                self._recorder.close()
                self._detach_recorder()
                if not session["episodes"]:
                    self.catalog.finalize_session(
                        session_id, "archived_empty")
                    return self._idle()
                raw = self.dataset_validator(session, False)
                self._check_saved_episodes(session, raw)
                self.catalog.finalize_session(session_id, "ready")
                return self._idle()
            except Exception as exc:  # noqa: BLE001
                self._detach_recorder()
                self.catalog.finalize_session(session_id, "recoverable")
                self._error = str(exc)
                self._idle()
                raise CollectionError(
                    f"cannot finalize collection: {exc}") from exc

    @staticmethod
    def _check_saved_episodes(session: dict, raw: dict) -> None:
        episodes = sorted(
            session["episodes"].values(),
            key=lambda episode: episode["source_index"],
        )
        expected = [episode["frames"] for episode in episodes]
        if raw["count"] != len(expected) or raw["lengths"] != expected:
            raise CollectionError(
                "saved dataset does not match the collection catalog")

    def recoveries(self) -> list[dict]:
        return [
            session for session in self.catalog.sessions()
            if session["state"] in {"active", "recoverable"}
        ]

    def recover_finish(self, session_id: str) -> dict:
        with self._lock:
            self._require("idle")
            session = self.catalog.session(session_id)
            if session["state"] not in {"active", "recoverable"}:
                raise CollectionError("session is not recoverable")
            raw = self.dataset_validator(session, True)
            catalog_count = len(session["episodes"])
            pending = session["pending_episode"]
            if raw["count"] == catalog_count:
                if pending is not None:
                    self.catalog.clear_pending(session_id)
            elif raw["count"] == catalog_count + 1 and pending is not None:
                self.catalog.commit_episode(
                    session_id,
                    pending["source_index"],
                    raw["lengths"][-1],
                    interrupted=True,
                )
            else:
                raise CollectionError(
                    f"episode count mismatch: raw={raw['count']}, "
                    f"catalog={catalog_count}")
            repaired = self.catalog.session(session_id)
            self._check_saved_episodes(repaired, raw)
            return self.catalog.finalize_session(session_id, "ready")

    def recover_archive(self, session_id: str) -> dict:
        with self._lock:
            self._require("idle")
            session = self.catalog.session(session_id)
            if session["state"] not in {"active", "recoverable"}:
                raise CollectionError("session is not recoverable")
            return self.catalog.finalize_session(
                session_id, "archived_incomplete")

    def shutdown(self) -> None:
        with self._lock:
            if self._state == "recording":
                try:
                    frames = self._recorder.stop()
                    pending = self.catalog.session(
                        self._session_id)["pending_episode"]
                    if frames > 0:
                        self.catalog.commit_episode(
                            self._session_id,
                            pending["source_index"],
                            frames,
                            interrupted=True,
                        )
                    else:
                        self.catalog.clear_pending(self._session_id)
                    self._state = "ready"
                except Exception as exc:  # noqa: BLE001
                    self._error = str(exc)
                    self.catalog.finalize_session(
                        self._session_id, "recoverable")
                    self._detach_recorder()
                    self._idle()
                    return
            if self._state == "ready":
                try:
                    self.finish_session()
                except CollectionError:
                    pass

    @staticmethod
    def _validate_dataset(session: dict, finalize: bool) -> dict:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        if finalize:
            writer = LeRobotDataset.resume(
                repo_id=session["repo_id"],
                root=session["root"],
                video_backend="pyav",
                streaming_encoding=True,
                encoder_threads=2,
            )
            writer.finalize()
        dataset = LeRobotDataset(
            repo_id=session["repo_id"],
            root=session["root"],
            video_backend="pyav",
        )
        lengths = [
            int(episode["length"]) for episode in dataset.meta.episodes
        ]
        return {"count": dataset.num_episodes, "lengths": lengths}
