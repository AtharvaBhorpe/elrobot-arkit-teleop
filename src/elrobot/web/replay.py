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
import time  # noqa: E402
from dataclasses import asdict, is_dataclass  # noqa: E402

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

    def _dataset(self):
        if self._ds is None:
            with self._lock:
                if self._ds is None:            # re-check under the lock
                    from lerobot.datasets.lerobot_dataset import LeRobotDataset
                    self._ds = LeRobotDataset(
                        repo_id=self.repo_id, root=self.root,
                        video_backend="pyav")
        return self._ds

    def _bounds(
        self, episode: int, start_frame=None, end_frame_exclusive=None,
    ):
        ds = self._dataset()
        for row in ds.meta.episodes:
            if int(row["episode_index"]) == episode:
                lo = int(row["dataset_from_index"])
                hi = int(row["dataset_to_index"])
                length = hi - lo
                start = 0 if start_frame is None else int(start_frame)
                end = (
                    length if end_frame_exclusive is None
                    else int(end_frame_exclusive)
                )
                if not 0 <= start < end <= length:
                    raise KeyError(
                        f"invalid frame range [{start}, {end}) "
                        f"for {length}-frame episode {episode}")
                return lo + start, lo + end
        raise KeyError(f"no episode {episode}")

    def actions(
        self, episode: int, start_frame=None, end_frame_exclusive=None,
    ) -> list:
        """The recorded ACTION stream - what the operator commanded, which is
        what reproduces the demonstration. (states() returns what the arm
        actually did, which is for looking at, not for re-commanding.)"""
        ds = self._dataset()
        lo, hi = self._bounds(
            episode, start_frame, end_frame_exclusive)
        actions = ds.select_columns("action")
        return [np.asarray(actions[i]["action"]).tolist()
                for i in range(lo, hi)]

    def states(
        self, episode: int, start_frame=None, end_frame_exclusive=None,
    ) -> dict:
        """Whole joint trajectory in one response: 8 floats x N frames is
        small, and it lets the browser animate the URDF smoothly without a
        request per frame."""
        ds = self._dataset()
        lo, hi = self._bounds(
            episode, start_frame, end_frame_exclusive)
        states = ds.select_columns("observation.state")
        rows = [np.asarray(states[i]["observation.state"]).tolist()
                for i in range(lo, hi)]
        return {"fps": float(ds.fps), "names": JOINTS,
                "frames": len(rows), "states": rows}

    def frame_jpeg(
        self, episode: int, n: int, cam: str,
        start_frame=None, end_frame_exclusive=None,
    ) -> bytes:
        key = {"wrist": "observation.images.wrist",
               "ext": "observation.images.external"}[cam]
        ds = self._dataset()
        lo, hi = self._bounds(
            episode, start_frame, end_frame_exclusive)
        idx = lo + max(0, min(n, hi - lo - 1))
        img = _to_bgr_u8(ds[idx][key])
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        return jpg.tobytes()


class ReplayError(Exception):
    def __init__(self, detail, code=409):
        super().__init__(detail)
        self.detail, self.code = detail, code


# Rate the recorded actions are streamed back at; also the rate used while
# seeking the start pose, so the driver's deadman (200 ms) stays fed.
PUBLISH_HZ = 30.0
# Every joint must be within this of the episode's first pose before playback
# begins. The seek phase just publishes that first pose and lets the DRIVER's
# own slew limiter (max_vel, 0.6 rad/s by default) walk the arm there - the
# existing, measured safety mechanism, rather than a second motion primitive
# invented here.
SEEK_TOL_RAD = 0.05
SEEK_TIMEOUT_S = 45.0
MAX_SPEED = 1.0          # never faster than it was recorded


class PhysicalReplay:
    """Re-executes a recorded episode ON THE REAL ARM.

    The arm moves with nobody holding a clutch, so this is gated far more
    heavily than the visual player:

      * must be ARMED explicitly, as a separate deliberate act;
      * arming is mutually exclusive with slider control - two publishers on
        /joint_command with no arbitration is exactly the fight the cockpit
        already warns about;
      * refuses to run without a live driver, since every safety gate
        (velocity clamp, workspace box, sigma floor, joint limits, grasp
        latch) lives there and applies to these commands unchanged;
      * speed is capped at the recorded speed, never faster;
      * seeks the episode's start pose first, at driver-limited velocity,
        instead of jumping into the middle of a trajectory;
      * stop() halts publishing immediately, and the driver's deadman then
        freezes the arm within 200 ms - the same stop semantics as releasing
        the phone clutch or closing the browser tab.
    """

    def __init__(self, library, publish, current_pose, driver_alive):
        self._library = library
        self._publish = publish            # dict[name, rad] -> /joint_command
        self._current_pose = current_pose  # () -> dict[name, rad] from /joint_states
        self._driver_alive = driver_alive
        self.armed = False
        self.phase = "idle"                # idle | seeking | playing | done
        self.episode = None
        self.frame = 0
        self.total = 0
        self.speed = 0.6
        self.error = None
        self._stop = threading.Event()
        self._thread = None

    # ---- control -----------------------------------------------------
    def arm(self, on: bool, slider_control_on: bool):
        if on:
            if slider_control_on:
                raise ReplayError(
                    "turn Web control off first - slider commands and replay "
                    "would both publish /joint_command with no arbitration")
            if not self._driver_alive():
                raise ReplayError("no driver running - it holds every safety "
                                  "gate these commands rely on")
        else:
            self.stop()
        self.armed = bool(on)
        self.error = None
        return self.status()

    def play(self, selection, speed: float = 0.6):
        if not self.armed:
            raise ReplayError("arm replay first (this moves the real arm)")
        if not self._driver_alive():
            raise ReplayError("no driver running")
        if self.phase in ("seeking", "playing"):
            raise ReplayError("already running")
        try:
            speed = float(speed)
        except (TypeError, ValueError) as e:
            raise ReplayError("speed must be a number", code=400) from e
        if not 0 < speed <= MAX_SPEED:
            raise ReplayError(f"speed must be in (0, {MAX_SPEED}]", code=400)

        actions = self._library.actions(selection)
        if not actions:
            raise ReplayError(f"episode {selection} has no frames", code=404)

        self.episode, self.speed = selection, speed
        self.total, self.frame = len(actions), 0
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, args=(actions,), daemon=True)
        self._thread.start()
        return self.status()

    def stop(self):
        """Halt publishing at once. The driver's deadman does the rest."""
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3)
        self.phase = "idle"
        return self.status()

    def status(self) -> dict:
        episode = (
            asdict(self.episode)
            if is_dataclass(self.episode) else self.episode
        )
        return {"armed": self.armed, "phase": self.phase,
                "episode": episode, "frame": self.frame,
                "total": self.total, "speed": self.speed,
                "error": self.error}

    # ---- the player thread -------------------------------------------
    def _send(self, action):
        self._publish({n: float(v) for n, v in zip(JOINTS, action)})

    def _run(self, actions):
        try:
            self.phase = "seeking"
            if not self._seek(actions[0]):
                return                      # _seek set phase/error already

            self.phase = "playing"
            dt = 1.0 / (PUBLISH_HZ * self.speed)
            for i, action in enumerate(actions):
                if self._stop.is_set():
                    self.phase = "idle"
                    return
                if not self._driver_alive():
                    self.error = "driver disappeared mid-replay; stopped"
                    self.phase = "idle"
                    return
                self.frame = i
                self._send(action)
                time.sleep(dt)
            self.phase = "done"
        except Exception as e:                              # noqa: BLE001
            # A daemon thread's traceback goes nowhere useful; surface it.
            self.error = f"{type(e).__name__}: {e}"
            self.phase = "idle"
        finally:
            # Publishing has ceased either way; the deadman freezes the arm.
            self._stop.set()

    def _seek(self, first_action) -> bool:
        """Hold the episode's first pose until the arm has actually reached
        it. Publishing (not jumping) lets the driver's slew limiter walk it
        there at its own configured max velocity."""
        deadline = time.monotonic() + SEEK_TIMEOUT_S
        while not self._stop.is_set():
            self._send(first_action)
            pose = self._current_pose() or {}
            gap = max((abs(pose.get(n, 0.0) - v)
                       for n, v in zip(JOINTS, first_action)), default=0.0)
            if pose and gap <= SEEK_TOL_RAD:
                return True
            if time.monotonic() > deadline:
                self.error = (f"gave up seeking the start pose after "
                              f"{SEEK_TIMEOUT_S:.0f}s (worst joint still "
                              f"{gap:.2f} rad off) - is the arm blocked, or "
                              f"is a safety gate holding it?")
                self.phase = "idle"
                return False
            time.sleep(1.0 / PUBLISH_HZ)
        self.phase = "idle"
        return False
