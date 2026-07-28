"""End-to-end recorder test: fake cameras + fake joints -> valid dataset.

Publishes synthetic streams (distinct image patterns per camera, known joint
values), runs episode_recorder --auto 2 as a subprocess, then loads the
dataset back and checks frame count, shapes, and that the values round-trip.

    pixi run python tests/test_recorder.py
"""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session's DDS graph
os.environ.setdefault("HF_HUB_OFFLINE", "1")  # local datasets, never the network

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import Image, JointState

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT, JAW_MIMIC  # noqa: E402
from elrobot.nodes.episode_recorder import Recorder, positive_int  # noqa: E402

ROOT = Path("data/test_episodes")
JOINTS = ARM_JOINTS + [GRIPPER_JOINT]


def test_encoder_configuration():
    assert positive_int("1") == 1
    assert positive_int("2") == 2
    for bad in ("0", "-1"):
        try:
            positive_int(bad)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError(
                f"accepted invalid encoder thread count {bad}")


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


def test_subprocess_auto_episode():
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
        [sys.executable, "-m", "elrobot.nodes.episode_recorder",
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
    print("subprocess --auto scenario PASSED")
    rclpy.try_shutdown()


def test_record_cmd_topic():
    """/record/cmd start/stop drives the recorder the same as ENTER does.

    Runs the Recorder in-process (not a subprocess): racing a subprocess's
    stdin against a ROS command is fragile - without --auto the recorder's
    main() blocks on input(), and a piped/closed stdin on a subprocess
    hits EOF almost immediately, likely before the fake /record/cmd message
    even has time to arrive. In-process sidesteps that timing race entirely
    and exercises the exact same Recorder._on_cmd/_status_tick code.
    """
    from std_msgs.msg import String

    root = Path("data/test_episodes_cmd")
    if root.exists():
        shutil.rmtree(root)
    rclpy.init()
    try:
        fake = rclpy.create_node("fake_streams_cmd")
        pubs = {
            "wrist": fake.create_publisher(Image, "/wrist_cam/image", 1),
            "ext": fake.create_publisher(Image, "/ext_cam/image", 1),
            "state": fake.create_publisher(JointState, "/joint_states", 1),
            "action": fake.create_publisher(JointState, "/joint_command", 1),
            "cmd": fake.create_publisher(String, "/record/cmd", 1),
        }
        fake.create_timer(1 / 30, lambda: (
            pubs["wrist"].publish(img_msg(fake, 50)),
            pubs["ext"].publish(img_msg(fake, 200)),
            pubs["state"].publish(js_msg(fake, [0.1] * 8)),
            pubs["action"].publish(js_msg(fake, [0.2] * 8))))

        args = argparse.Namespace(fps=30.0, repo_id="local/elrobot_teleop",
                                  root=str(root), task="test",
                                  encoder_threads=2)
        node = Recorder(args)

        statuses = []
        fake.create_subscription(String, "/record/status",
                                 lambda m: statuses.append(json.loads(m.data)), 5)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(fake)
        executor.add_node(node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()

        deadline = time.monotonic() + 10.0
        while not node.recording and time.monotonic() < deadline:
            pubs["cmd"].publish(String(data="start"))
            time.sleep(0.1)
        assert node.recording, "recorder never started via /record/cmd"
        assert node.dataset._encoder_threads == 2
        assert node.dataset.writer._streaming_encoder is not None
        try:
            node.set_task("changed during recording")
        except RuntimeError:
            pass
        else:
            raise AssertionError("task changed during an active episode")

        # _status_tick fires once per second - hold recording comfortably
        # past that so at least one /record/status message is captured
        # before we stop, not just enough frames
        time.sleep(1.5)

        deadline = time.monotonic() + 10.0
        while node.recording and time.monotonic() < deadline:
            pubs["cmd"].publish(String(data="stop"))
            time.sleep(0.1)
        assert not node.recording, "recorder never stopped via /record/cmd"

        # recording flips False at the START of stop(), before the
        # (possibly slow) dataset.save_episode() call that increments
        # episodes - wait for the actual completion signal, not just for
        # recording to flip, or this races the encoder flush
        deadline = time.monotonic() + 10.0
        while node.episodes < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert node.episodes == 1, "save_episode() never completed in time"
        node.set_task("second task")
        assert node.args.task == "second task"

        time.sleep(1.2)   # let one more /record/status tick land
        assert any(s["recording"] for s in statuses), (
            "no /record/status message showed recording=true")
        assert statuses[-1]["episodes"] == 1

        node.close()
        executor.shutdown()
    finally:
        rclpy.try_shutdown()
    print("/record/cmd + /record/status PASSED")


def test_start_reports_failure_without_streams():
    root = Path(tempfile.mkdtemp()) / "missing"
    args = argparse.Namespace(
        fps=30.0,
        repo_id="local/missing",
        root=str(root),
        task="test",
        encoder_threads=2,
        start_timeout=0.05,
    )
    rclpy.init()
    node = Recorder(args)
    try:
        assert node.start() is False
        assert node.recording is False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_second_run_resumes_existing_dataset():
    """A second RUN of the recorder against the same --root used to die with
    FileExistsError the moment an episode started, because _ensure_dataset
    always called LeRobotDataset.create() (mkdir exist_ok=False). That makes a
    multi-session collection campaign impossible - the whole point is to keep
    adding episodes to one dataset. ROOT already holds one episode from
    test_subprocess_auto_episode; run the recorder again over it."""
    assert ROOT.exists(), "expected the earlier episode's dataset to be there"

    rclpy.init()
    node = rclpy.create_node("fake_streams_resume")
    pubs = {
        "wrist": node.create_publisher(Image, "/wrist_cam/image", 1),
        "ext": node.create_publisher(Image, "/ext_cam/image", 1),
        "state": node.create_publisher(JointState, "/joint_states", 1),
        "action": node.create_publisher(JointState, "/joint_command", 1),
    }
    node.create_timer(1 / 30, lambda: (
        pubs["wrist"].publish(img_msg(node, 50)),
        pubs["ext"].publish(img_msg(node, 200)),
        pubs["state"].publish(js_msg(node, [0.1] * 8)),
        pubs["action"].publish(js_msg(node, [0.2] * 8))))
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    proc = subprocess.run(
        [sys.executable, "-m", "elrobot.nodes.episode_recorder",
         "--auto", "2", "--root", str(ROOT), "--task", "test"],
        capture_output=True, text=True, timeout=180)
    print(proc.stdout[-400:] or "", proc.stderr[-400:] or "")
    assert "FileExistsError" not in proc.stderr, "second run must not crash"
    assert proc.returncode == 0, "recorder exited nonzero on resume"

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id="local/elrobot_teleop", root=str(ROOT),
                        video_backend="pyav")
    assert ds.num_episodes == 2, f"expected 2 episodes, got {ds.num_episodes}"
    print(f"resumed dataset now has {ds.num_episodes} episodes")
    rclpy.try_shutdown()
    print("resume-existing-dataset PASSED")


def main():
    test_encoder_configuration()
    test_start_reports_failure_without_streams()
    test_subprocess_auto_episode()
    test_second_run_resumes_existing_dataset()
    test_record_cmd_topic()
    print("\nRECORDER TEST PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
