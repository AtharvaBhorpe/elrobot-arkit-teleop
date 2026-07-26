"""Episode recorder -> LeRobotDataset (the format training consumes).

Samples the LATEST of each stream at --fps (default 30):
  observation.images.wrist     /wrist_cam/image
  observation.images.external  /ext_cam/image
  observation.state            /joint_states   (real arm, 8 joints, rad)
  action                       /joint_command  (commanded, 8 joints, rad)

Run in its OWN terminal (keyboard control) with the m3-arm stack and both
cam nodes up:

    pixi run record            # = python -m elrobot.nodes.episode_recorder

ENTER starts/stops an episode; q + ENTER saves everything and exits.
--auto N records one N-second episode with no keyboard (used by the test).
Also controllable over ROS: /record/cmd (std_msgs/String, "start"|"stop"|
"discard") in, /record/status (JSON: recording/episodes/frames) out at 1 Hz
- the web cockpit's Record panel uses this, terminal ENTER keeps working.

Frames are skipped (with a warn) while any stream is missing or stale, so a
dead camera yields a short episode, not silently corrupt data.
"""

import os

os.environ.setdefault("HF_HUB_OFFLINE", "1")  # local datasets, never the network

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT  # noqa: E402

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
STALE_S = 0.5


class Recorder(Node):
    def __init__(self, args):
        super().__init__("episode_recorder")
        self.args = args
        self.latest = {}   # key -> (data, monotonic time)
        self.dataset = None
        self.recording = False
        self.n_frames = 0
        self.episodes = 0
        self._starting = False   # guards start() against overlapping calls

        self.create_subscription(Image, "/wrist_cam/image",
                                 lambda m: self._img(m, "wrist"), 1)
        self.create_subscription(Image, "/ext_cam/image",
                                 lambda m: self._img(m, "external"), 1)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._joints(m, "state"), 1)
        self.create_subscription(JointState, "/joint_command",
                                 lambda m: self._joints(m, "action"), 1)
        self.create_subscription(String, "/record/cmd", self._on_cmd, 1)
        self._status_pub = self.create_publisher(String, "/record/status", 1)
        self.create_timer(1.0 / args.fps, self._tick)
        self.create_timer(1.0, self._status_tick)

    def _img(self, msg, key):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        self.latest[key] = (a[:, :, ::-1].copy(), time.monotonic())  # bgr->rgb

    def _joints(self, msg, key):
        d = dict(zip(msg.name, msg.position))
        if all(n in d for n in JOINTS):
            v = np.array([d[n] for n in JOINTS], dtype=np.float32)
            self.latest[key] = (v, time.monotonic())

    def _sample(self):
        now = time.monotonic()
        out = {}
        for key in ("wrist", "external", "state", "action"):
            if key not in self.latest:
                return None, f"no {key} yet"
            data, t = self.latest[key]
            if now - t > STALE_S:
                return None, f"{key} stale ({now - t:.1f} s)"
            out[key] = data
        return out, ""

    def _ensure_dataset(self, s):
        if self.dataset is not None:
            return
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        feats = {
            "observation.state": {"dtype": "float32", "shape": (len(JOINTS),),
                                  "names": JOINTS},
            "action": {"dtype": "float32", "shape": (len(JOINTS),),
                       "names": JOINTS},
        }
        for key in ("wrist", "external"):
            h, w, c = s[key].shape
            feats[f"observation.images.{key}"] = {
                "dtype": "video", "shape": (h, w, c),
                "names": ["height", "width", "channels"]}

        # RESUME an existing dataset rather than always create()ing. create()
        # does mkdir(exist_ok=False), so the second RUN of the recorder against
        # the same --root died with FileExistsError the moment you started an
        # episode - fatal for a multi-session collection campaign, where the
        # whole point is to keep adding episodes to one dataset.
        if Path(self.args.root).exists():
            self.dataset = LeRobotDataset.resume(
                repo_id=self.args.repo_id, root=self.args.root,
                video_backend="pyav")
            self.get_logger().info(
                f"resumed {self.args.root} "
                f"({self.dataset.num_episodes} episode(s) already recorded)")
        else:
            self.dataset = LeRobotDataset.create(
                repo_id=self.args.repo_id, fps=int(self.args.fps),
                features=feats, root=self.args.root, robot_type="elrobot",
                video_backend="pyav")  # torchcodec can't bind this env's ffmpeg
            self.get_logger().info(f"dataset created at {self.args.root}")
        self.episodes = self.dataset.num_episodes

    def _tick(self):
        if not self.recording:
            return
        s, why = self._sample()
        if s is None:
            self.get_logger().warning(f"frame skipped: {why}",
                                      throttle_duration_sec=1.0)
            return
        self.dataset.add_frame({
            "observation.state": s["state"],
            "action": s["action"],
            "observation.images.wrist": s["wrist"],
            "observation.images.external": s["external"],
            "task": self.args.task,
        })
        self.n_frames += 1

    def start(self):
        # Wait for EVERY stream to be live and fresh before arming, on every
        # episode - not just the first. The old loop was `while self.dataset
        # is None`, so once a dataset existed it never sampled at all: episode
        # 2 onward armed blind. Observed exactly that - "RECORDING" logged
        # while /joint_command had been silent for 141 s, then every frame
        # skipped and "nothing recorded, nothing saved".
        deadline = time.monotonic() + 10.0
        while True:
            s, why = self._sample()
            if s is not None:
                break
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"cannot start: {why}. Is the driver/teleop running "
                    f"(action = /joint_command) and are both cameras up?")
                return
            time.sleep(0.1)

        # Dataset creation takes seconds (video encoder setup) - do it HERE,
        # in the caller's thread, never inside the 30 Hz timer callback
        # (measured: create() inside _tick froze the executor and episodes
        # recorded zero frames).
        self._ensure_dataset(s)
        self.recording = True
        self.n_frames = 0
        self.get_logger().info("RECORDING - ENTER to stop")

    def stop(self):
        self.recording = False
        if self.dataset is not None and self.n_frames > 0:
            self.dataset.save_episode()
            self.episodes += 1
            self.get_logger().info(f"episode saved ({self.n_frames} frames)")
        else:
            self.get_logger().warning("nothing recorded, nothing saved")

    def discard(self):
        self.recording = False
        if self.dataset is not None:
            self.dataset.clear_episode_buffer()
        self.get_logger().info("episode discarded")

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "start" and not self.recording and not self._starting:
            # start() blocks up to 10s on dataset creation - NEVER in the
            # executor thread (the zero-frame incident), so thread it. The
            # keyboard path already runs start()/stop() from a thread other
            # than rclpy.spin()'s (see main()), so this isn't a new pattern.
            # _starting guards against a second "start" arriving (e.g. a
            # laggy double-click, or a caller retrying) while the first is
            # still mid-wait: self.recording only flips True at the very
            # end of start(), so without this guard a second start() could
            # run concurrently and reset self.n_frames = 0 after real
            # frames from the first call had already begun accumulating.
            self._starting = True

            def run():
                try:
                    self.start()
                except Exception as e:                   # noqa: BLE001
                    # A raw traceback from a daemon thread is not something
                    # the web Start button can show; log it as an error so it
                    # lands in this terminal like every other failure.
                    self.get_logger().error(f"start failed: {e}")
                finally:
                    self._starting = False
            threading.Thread(target=run, daemon=True).start()
        elif cmd == "stop" and self.recording:
            self.stop()
        elif cmd == "discard" and self.recording:
            self.discard()

    def _status_tick(self):
        self._status_pub.publish(String(data=json.dumps({
            "recording": self.recording, "episodes": self.episodes,
            "frames": self.n_frames})))

    def close(self):
        if self.dataset is not None:
            self.dataset.finalize()
            self.get_logger().info(f"dataset finalized: {self.args.root}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--repo-id", default="local/elrobot_teleop")
    p.add_argument("--root", default="data/episodes/elrobot_teleop")
    p.add_argument("--task", default="teleop")
    p.add_argument("--auto", type=float, default=None,
                   help="record ONE episode of N seconds, no keyboard (tests)")
    args, _ = p.parse_known_args()

    rclpy.init()
    node = Recorder(args)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        if args.auto is not None:
            time.sleep(1.0)          # let streams arrive
            node.start()
            time.sleep(args.auto)
            node.stop()
        else:
            print("ENTER = start/stop episode, q+ENTER = save all and quit")
            while True:
                line = input()
                if line.strip().lower() == "q":
                    if node.recording:
                        node.stop()
                    break
                node.stop() if node.recording else node.start()
    except (KeyboardInterrupt, EOFError):
        if node.recording:
            node.stop()
    finally:
        node.close()
        rclpy.try_shutdown()
        # Hard exit once the dataset is finalised and rclpy is down.
        # Interpreter finalisation here intermittently aborts with
        # "terminate called without an active exception" (SIGABRT, core
        # dumped) AFTER a completely successful recording - a teardown race
        # in the native stack (SVT-AV1 encoder threads / pyarrow) as their
        # objects are collected during shutdown. It only shows up under the
        # load of a full test run, never in isolation, and it made the suite
        # and CI flaky by turning a good recording into a nonzero exit.
        # Nothing here relies on atexit; flush explicitly, then skip the
        # teardown that races instead of trying to win the race.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
