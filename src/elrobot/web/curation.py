"""Read-only curated views over immutable collection sessions."""

from __future__ import annotations

from dataclasses import dataclass

from elrobot.web.collection import CatalogError
from elrobot.web.replay import ReplayLibrary


@dataclass(frozen=True)
class EpisodeRef:
    session_id: str
    source_index: int
    raw: bool = False


class CuratedReplayLibrary:
    def __init__(self, catalog, legacy=None):
        self.catalog = catalog
        self.legacy = legacy
        self._sessions = {}

    def _resolve(self, ref: EpisodeRef):
        try:
            session = self.catalog.session(ref.session_id)
            episode = self.catalog.episode(
                ref.session_id, ref.source_index)
        except CatalogError as exc:
            raise KeyError(exc.detail) from exc
        if session["state"] != "ready":
            raise KeyError(f"session {ref.session_id} is not ready")
        trim = episode["trim"]
        start = 0 if ref.raw or trim is None else trim["start_frame"]
        end = (
            episode["frames"]
            if ref.raw or trim is None else trim["end_frame_exclusive"]
        )
        library = self._sessions.get(ref.session_id)
        if library is None:
            library = ReplayLibrary(
                root=session["root"], repo_id=session["repo_id"])
            self._sessions[ref.session_id] = library
        return library, episode, start, end

    def list_episodes(self, task_id=None, session_id=None) -> list[dict]:
        episodes = []
        for session in self.catalog.sessions():
            if session["state"] != "ready":
                continue
            if session_id is not None and session["id"] != session_id:
                continue
            for episode in sorted(
                session["episodes"].values(),
                key=lambda item: item["source_index"],
            ):
                effective_task_id = (
                    episode["task_id"] or episode["source_task_id"])
                if task_id is not None and effective_task_id != task_id:
                    continue
                trim = episode["trim"]
                start = 0 if trim is None else trim["start_frame"]
                end = (
                    episode["frames"]
                    if trim is None else trim["end_frame_exclusive"]
                )
                episodes.append({
                    **episode,
                    "session_id": session["id"],
                    "session_name": session["name"],
                    "effective_task_id": effective_task_id,
                    "effective_task": self.catalog.task(effective_task_id),
                    "start_frame": start,
                    "end_frame_exclusive": end,
                    "effective_frames": end - start,
                })
        return episodes

    def actions(self, selection) -> list:
        if isinstance(selection, int):
            if self.legacy is None:
                raise KeyError("legacy replay is unavailable")
            return self.legacy.actions(selection)
        library, _, start, end = self._resolve(selection)
        return library.actions(
            selection.source_index, start, end)

    def states(self, selection) -> dict:
        if isinstance(selection, int):
            if self.legacy is None:
                raise KeyError("legacy replay is unavailable")
            return self.legacy.states(selection)
        library, _, start, end = self._resolve(selection)
        return library.states(
            selection.source_index, start, end)

    def frame_jpeg(self, selection, n: int, cam: str) -> bytes:
        if isinstance(selection, int):
            if self.legacy is None:
                raise KeyError("legacy replay is unavailable")
            return self.legacy.frame_jpeg(selection, n, cam)
        library, _, start, end = self._resolve(selection)
        return library.frame_jpeg(
            selection.source_index, n, cam, start, end)
