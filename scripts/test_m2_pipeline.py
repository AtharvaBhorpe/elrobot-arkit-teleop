"""End-to-end M2 pipeline test, no GUI, no phone.

Spawns ik_node.py and arkit_receiver.py as separate processes (same topology
as m2.launch.py) and fakes ZIG SIM PRO over UDP: engage clutch, move the
phone "up" 25 cm, toggle the gripper, go silent to trip the stream deadman,
then re-engage far away. All assertions are made from the outside, via
/joint_states + Pinocchio FK — exactly what rviz sees.

    pixi run python scripts/test_m2_pipeline.py
"""

import json
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cartesian_ik import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_JOINT, CartesianServoIK  # noqa: E402

PORT = 50123
SCALE = 0.4


def packet(pos, n_touch, rot=(0.0, 0.0, 0.0, 1.0)):
    return json.dumps({"sensordata": {
        "arkit": {"position": list(pos), "rotation": list(rot)},
        "touch": [{"x": 0, "y": 0}] * n_touch,
    }}).encode()


class Probe(Node):
    """Watches /joint_states and answers 'where is the TCP?' via FK."""

    def __init__(self):
        super().__init__("m2_test_probe")
        self.fk = CartesianServoIK()
        self.qidx = {n: self.fk.model.joints[self.fk.model.getJointId(n)].idx_q
                     for n in ARM_JOINTS}
        self.gripper = None
        self.seen = False
        self.create_subscription(JointState, "/joint_states", self._on_js, 1)

    def _on_js(self, msg):
        q = self.fk.q.copy()
        for name, pos in zip(msg.name, msg.position):
            if name in self.qidx:
                q[self.qidx[name]] = pos
            elif name == GRIPPER_JOINT:
                self.gripper = pos
        self.fk.set_q(q)
        self.seen = True

    def tcp(self):
        return self.fk.ee_pose().translation.copy()


def main():
    procs = [subprocess.Popen([sys.executable, str(HERE / s), *a])
             for s, a in ((("ik_node.py"), []),
                          (("arkit_receiver.py"), ["--port", str(PORT)]))]
    try:
        rclpy.init()
        probe = Probe()
        spin = threading.Thread(target=rclpy.spin, args=(probe,), daemon=True)
        spin.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def send(pos, n):
            sock.sendto(packet(pos, n), ("127.0.0.1", PORT))

        t0 = time.time()
        while not probe.seen:
            assert time.time() - t0 < 10, "no /joint_states within 10 s"
            time.sleep(0.05)
        time.sleep(0.5)  # receiver needs a /joint_states sample too
        start = probe.tcp()
        print(f"TCP at start: {np.round(start, 4)}")

        # 1) engage clutch, ramp phone +Y (ARKit up) to 25 cm over 2 s
        send([0.0, 0.0, 0.0], 1)
        time.sleep(0.1)
        for i in range(1, 101):
            send([0.0, 0.25 * i / 100, 0.0], 1)
            time.sleep(0.02)
        # keep streaming the final pose while the servo settles (kp=2 ->
        # ~0.5 s time constant; 3 s leaves <0.3% of the transient)
        for _ in range(150):
            send([0.0, 0.25, 0.0], 1)
            time.sleep(0.02)

        tcp = probe.tcp()
        dz = tcp[2] - start[2]
        dxy = np.linalg.norm(tcp[:2] - start[:2])
        want = 0.25 * SCALE
        print(f"TCP after +25 cm phone-up: {np.round(tcp, 4)} "
              f"(dz={dz*100:+.1f} cm, want {want*100:.0f}; drift {dxy*100:.1f} cm)")
        assert abs(dz - want) < 0.02, f"expected dz ~ {want:.2f}, got {dz:.3f}"
        assert dxy < 0.02, f"lateral drift {dxy:.3f} m"

        # 2) 2-finger tap latches the gripper closed
        send([0.0, 0.25, 0.0], 2)
        time.sleep(0.3)
        assert probe.gripper is not None and \
            abs(probe.gripper - GRIPPER_CLOSED) < 1e-6, probe.gripper
        print("gripper latched closed")

        # 3) silence > deadman, then re-engage at a far phone pose: no jump.
        #    (If the deadman failed, the clutch would still be engaged and the
        #    far pose would command a huge move - caught by the same assert.)
        time.sleep(0.5)
        before = probe.tcp()
        send([5.0, -3.0, 2.0], 1)
        for _ in range(25):
            send([5.0, -3.0, 2.0], 1)
            time.sleep(0.02)
        jump = np.linalg.norm(probe.tcp() - before)
        assert jump < 0.01, f"re-engage after deadman jumped {jump:.3f} m"
        print(f"deadman + far re-engage: no jump ({jump*1000:.1f} mm)")

        print("\nM2 PIPELINE TEST PASSED")
        return 0
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            p.wait(timeout=5)
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
