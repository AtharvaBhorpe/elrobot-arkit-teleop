"""Episode recorder -> LeRobotDataset (the format training consumes).

Samples the LATEST of each stream at --fps (default 30):
  observation.images.wrist     /wrist_cam/image
  observation.images.external  /ext_cam/image
  observation.state            /joint_states   (real arm, 8 joints, rad)
  action                       /joint_command  (commanded, 8 joints, rad)

Run in its OWN terminal (keyboard control) with the m3-arm stack and both
cam nodes up:

    pixi run record            # = python scripts/episode_recorder.py

ENTER starts/stops an episode; q + ENTER saves everything and exits.
--auto N records one N-second episode with no keyboard (used by the test).

Frames are skipped (with a warn) while any stream is missing or stale, so a
dead camera yields a short episode, not silently corrupt data.
"""

import os

os.environ.setdefault("HF_HUB_OFFLINE", "1")  # local datasets, never the network

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

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

        self.create_subscription(Image, "/wrist_cam/image",
                                 lambda m: self._img(m, "wrist"), 1)
        self.create_subscription(Image, "/ext_cam/image",
                                 lambda m: self._img(m, "external"), 1)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._joints(m, "state"), 1)
        self.create_subscription(JointState, "/joint_command",
                                 lambda m: self._joints(m, "action"), 1)
        self.create_timer(1.0 / args.fps, self._tick)

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
        self.dataset = LeRobotDataset.create(
            repo_id=self.args.repo_id, fps=int(self.args.fps), features=feats,
            root=self.args.root, robot_type="elrobot",
            video_backend="pyav")  # torchcodec cannot bind this env's ffmpeg
        self.get_logger().info(f"dataset created at {self.args.root}")

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
        # Dataset creation takes seconds (video encoder setup) - do it HERE,
        # in the caller's thread, never inside the 30 Hz timer callback
        # (measured: create() inside _tick froze the executor and episodes
        # recorded zero frames).
        deadline = time.monotonic() + 10.0
        while self.dataset is None:
            s, why = self._sample()
            if s is not None:
                self._ensure_dataset(s)
                break
            if time.monotonic() > deadline:
                self.get_logger().error(f"cannot start: {why}")
                return
            time.sleep(0.1)
        self.recording = True
        self.n_frames = 0
        self.get_logger().info("RECORDING - ENTER to stop")

    def stop(self):
        self.recording = False
        if self.dataset is not None and self.n_frames > 0:
            self.dataset.save_episode()
            self.get_logger().info(f"episode saved ({self.n_frames} frames)")
        else:
            self.get_logger().warning("nothing recorded, nothing saved")

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


if __name__ == "__main__":
    main()
