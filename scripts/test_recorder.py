"""End-to-end recorder test: fake cameras + fake joints -> valid dataset.

Publishes synthetic streams (distinct image patterns per camera, known joint
values), runs episode_recorder --auto 2 as a subprocess, then loads the
dataset back and checks frame count, shapes, and that the values round-trip.

    pixi run python scripts/test_recorder.py
"""

import os
os.environ.setdefault("HF_HUB_OFFLINE", "1")  # local datasets, never the network

import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import Image, JointState

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cartesian_ik import ARM_JOINTS, GRIPPER_JOINT, JAW_MIMIC  # noqa: E402

ROOT = Path("data/test_episodes")
JOINTS = ARM_JOINTS + [GRIPPER_JOINT]


def img_msg(node, value, w=64, h=48):
    a = np.full((h, w, 3), value, dtype=np.uint8)
    m = Image()
    m.header.stamp = node.get_clock().now().to_msg()
    m.height, m.width = h, w
    m.encoding = "bgr8"
    m.step = w * 3
    m.data = a.tobytes()
    return m


def js_msg(node, values):
    m = JointState()
    m.header.stamp = node.get_clock().now().to_msg()
    m.name = JOINTS + list(JAW_MIMIC)
    m.position = list(values) + [0.0] * len(JAW_MIMIC)
    return m


def main():
    if ROOT.exists():
        shutil.rmtree(ROOT)
    rclpy.init()
    node = rclpy.create_node("fake_streams")
    pubs = {
        "wrist": node.create_publisher(Image, "/wrist_cam/image", 1),
        "ext": node.create_publisher(Image, "/ext_cam/image", 1),
        "state": node.create_publisher(JointState, "/joint_states", 1),
        "action": node.create_publisher(JointState, "/joint_command", 1),
    }
    state_v = [0.1] * 8
    action_v = [0.2] * 8

    def feed():
        pubs["wrist"].publish(img_msg(node, 50))    # wrist gray = 50
        pubs["ext"].publish(img_msg(node, 200))     # external gray = 200
        pubs["state"].publish(js_msg(node, state_v))
        pubs["action"].publish(js_msg(node, action_v))

    node.create_timer(1 / 30, feed)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    proc = subprocess.run(
        [sys.executable, str(HERE / "episode_recorder.py"),
         "--auto", "2", "--root", str(ROOT), "--task", "test"],
        capture_output=True, text=True, timeout=180)
    print(proc.stdout[-800:] or "", proc.stderr[-800:] or "")
    assert proc.returncode == 0, "recorder exited nonzero"

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id="local/elrobot_teleop", root=str(ROOT),
                        video_backend="pyav")
    n = ds.num_frames
    print(f"dataset loads: {n} frames, {ds.num_episodes} episode(s)")
    assert ds.num_episodes == 1
    assert 30 <= n <= 75, f"expected ~60 frames of 2 s @30 fps, got {n}"
    f = ds[0]
    st = np.asarray(f["observation.state"])
    ac = np.asarray(f["action"])
    assert st.shape == (8,) and np.allclose(st, 0.1, atol=1e-6), st
    assert ac.shape == (8,) and np.allclose(ac, 0.2, atol=1e-6), ac
    wr = np.asarray(f["observation.images.wrist"])
    ex = np.asarray(f["observation.images.external"])
    # dataset returns CHW float or HWC uint8 depending on version - normalize
    def mean255(a):
        a = a.astype(np.float32)
        return a.mean() * (255.0 if a.max() <= 1.5 else 1.0)
    assert abs(mean255(wr) - 50) < 12, f"wrist pixel mean {mean255(wr)}"
    assert abs(mean255(ex) - 200) < 12, f"external pixel mean {mean255(ex)}"
    print("images round-trip with correct per-camera content")
    print("\nRECORDER TEST PASSED")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
