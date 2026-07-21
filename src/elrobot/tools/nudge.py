"""Nudge one joint by a small delta through the driver - the first-motion test.

Reads the real pose from /joint_states, adds delta to ONE joint, publishes
that as /joint_command repeatedly for `hold` seconds (the driver's deadman
freezes on silence, so a single message would move ~0.2 s worth and stop).

    pixi run python scripts/nudge.py rev_motor_04 --delta 0.09 --hold 2
"""

import argparse
import time

import rclpy
from sensor_msgs.msg import JointState

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT  # noqa: E402

ALL = ARM_JOINTS + [GRIPPER_JOINT]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("joint", choices=ALL)
    ap.add_argument("--delta", type=float, default=0.09, help="rad (~5 deg)")
    ap.add_argument("--hold", type=float, default=2.0, help="seconds to stream")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("nudge")
    state = {}

    def on_js(msg):
        state.update(zip(msg.name, msg.position))

    node.create_subscription(JointState, "/joint_states", on_js, 1)
    pub = node.create_publisher(JointState, "/joint_command", 1)

    t0 = time.time()
    while len(state) < len(ALL):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 5:
            raise SystemExit("no /joint_states - is the driver running?")

    target = {n: state[n] for n in ALL}
    target[a.joint] += a.delta
    print(f"nudging {a.joint}: {state[a.joint]:+.3f} -> {target[a.joint]:+.3f} rad")

    end = time.time() + a.hold
    while time.time() < end:
        m = JointState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.name = ALL
        m.position = [target[n] for n in ALL]
        pub.publish(m)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)
    print(f"done: {a.joint} now at {state.get(a.joint, float('nan')):+.3f} rad "
          "(stream stopped -> driver deadman freezes)")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
