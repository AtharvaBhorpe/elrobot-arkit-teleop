"""Read recorded LeRobotDataset episodes back for the cockpit's replay panel.

Visual replay only: this module never publishes to /joint_command and never
touches the bus. It reads what was recorded so an episode can be watched back
- the URDF driven by the recorded observation.state, the camera panels showing
the recorded frames. That is dataset QA (does the episode contain what you
think it does?), not robot motion.

Physical replay - re-executing an episode's action stream on the real arm - is
deliberately NOT here. It is the same risk class as an autonomous policy
rollout: the arm moves with nobody holding a clutch.
"""

import os

# Same as episode_recorder: these datasets are local and must never trigger a
# hub round-trip. Without it, opening one can fall through to
# huggingface.co and fail with a 401 for a repo_id that only exists on disk.
os.environ.setdefault("HF_HUB_OFFLINE", "1")

import threading  # noqa: E402
from pathlib import Path  # noqa: E402

import cv2
import numpy as np

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]


def _to_bgr_u8(arr) -> np.ndarray:
    """Dataset frames come back as CHW float32 in [0,1] (torch convention);
    cv2 wants HWC uint8 BGR."""
    a = np.asarray(arr)
    if a.ndim == 3 and a.shape[0] in (1, 3) and a.shape[0] < a.shape[-1]:
        a = np.transpose(a, (1, 2, 0))          # CHW -> HWC
    if a.dtype != np.uint8:
        a = (np.clip(a, 0.0, 1.0) * 255).astype(np.uint8) if a.max() <= 1.5 \
            else a.astype(np.uint8)
    return a[:, :, ::-1] if a.shape[-1] == 3 else a      # RGB -> BGR


class ReplayLibrary:
    """Lazily opens the dataset and caches it. Opening costs seconds (video
    backend setup), so it must not happen per request."""

    def __init__(self, root="data/episodes/elrobot_teleop",
                 repo_id="local/elrobot_teleop"):
        self.root, self.repo_id = root, repo_id
        self._ds = None
        self._lock = threading.Lock()

    def available(self) -> bool:
        return Path(self.root).exists()

    def _dataset(self):
        if self._ds is None:
            with self._lock:
                if self._ds is None:            # re-check under the lock
                    from lerobot.datasets.lerobot_dataset import LeRobotDataset
                    self._ds = LeRobotDataset(
                        repo_id=self.repo_id, root=self.root,
                        video_backend="pyav")
        return self._ds

    def reload(self):
        """Drop the cache so episodes recorded since startup appear."""
        with self._lock:
            self._ds = None

    def writing_in_progress(self) -> bool:
        """True while a recorder still holds this dataset open.

        LeRobotDataset buffers episodes and only writes the parquet footer
        and meta/episodes on finalize(), i.e. when the recorder EXITS. So a
        dataset being actively recorded into has a last data file with no
        PAR1 footer, and reading it raises a bare ArrowInvalid about
        "Parquet magic bytes not found" - which looks like corruption but is
        just an unfinished write. Detect it so the UI can say "quit the
        recorder to replay" instead of "no episodes".
        """
        files = sorted((Path(self.root) / "data").rglob("*.parquet"))
        if not files:
            return False
        try:
            with open(files[-1], "rb") as fh:
                fh.seek(-4, 2)
                return fh.read(4) != b"PAR1"
        except OSError:
            return False

    def episodes(self) -> list:
        if not self.available():
            return []
        if self.writing_in_progress():
            raise RuntimeError(
                "a recorder is still running - its episodes are only written "
                "out when it exits (q + ENTER, or Ctrl-C). Quit it, then "
                "press Refresh.")
        ds = self._dataset()
        out = []
        for row in ds.meta.episodes:
            lo, hi = row["dataset_from_index"], row["dataset_to_index"]
            tasks = row.get("tasks")
            out.append({
                "index": int(row["episode_index"]),
                "frames": int(row["length"]),
                "seconds": round(int(row["length"]) / ds.fps, 1),
                "task": (tasks[0] if isinstance(tasks, list) and tasks
                         else str(tasks or "")),
                "from": int(lo), "to": int(hi),
            })
        return out

    def _bounds(self, episode: int):
        for e in self.episodes():
            if e["index"] == episode:
                return e["from"], e["to"]
        raise KeyError(f"no episode {episode}")

    def states(self, episode: int) -> dict:
        """Whole joint trajectory in one response: 8 floats x N frames is
        small, and it lets the browser animate the URDF smoothly without a
        request per frame."""
        ds = self._dataset()
        lo, hi = self._bounds(episode)
        rows = [np.asarray(ds[i]["observation.state"]).tolist()
                for i in range(lo, hi)]
        return {"fps": float(ds.fps), "names": JOINTS,
                "frames": len(rows), "states": rows}

    def frame_jpeg(self, episode: int, n: int, cam: str) -> bytes:
        key = {"wrist": "observation.images.wrist",
               "ext": "observation.images.external"}[cam]
        ds = self._dataset()
        lo, hi = self._bounds(episode)
        idx = lo + max(0, min(n, hi - lo - 1))
        img = _to_bgr_u8(ds[idx][key])
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        return jpg.tobytes()
