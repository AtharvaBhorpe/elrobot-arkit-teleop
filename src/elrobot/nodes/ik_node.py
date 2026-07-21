"""ik — Cartesian servo node: /target_pose -> /joint_command (+ /joint_states).

Servos the 7 arm joints toward the latest target pose at a fixed rate;
rev_motor_08 passes through from /gripper_command untouched by IK.

Publishes:
  /joint_command  JointState, 8 positions — consumed by elrobot_driver (M3+)
  /joint_states   same content — drives robot_state_publisher/rviz in M2,
                  where there is no hardware to report real state. In M3 the
                  driver owns /joint_states; run with --no-sim-state then.

    pixi run python scripts/ik_node.py
"""

import argparse
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from elrobot.control.cartesian_ik import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_JOINT,
    GRIPPER_OPEN,
    JAW_MIMIC,
    CartesianServoIK,
)


class IKNode(Node):
    def __init__(self, args):
        super().__init__("ik")
        frozen = tuple(f for f in args.freeze.split(",") if f)
        self.ik = CartesianServoIK(frozen=frozen)
        if frozen:
            self.get_logger().info(f"SO-101 mode: frozen {list(frozen)}")
        self.ik.q_ref = self.ik.arm_q()  # null-space anchor; re-set on seed
        self.target: pin.SE3 | None = None
        self.gripper = GRIPPER_OPEN
        self.dt = 1.0 / args.rate
        self.smooth = args.smooth
        self.last_target_time = None

        self.create_subscription(PoseStamped, "/target_pose", self._on_target, 1)
        self.create_subscription(Float64, "/gripper_command", self._on_gripper, 1)
        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 1)
        self.state_pub = (self.create_publisher(JointState, "/joint_states", 1)
                          if args.sim_state else None)
        # M3 (driver present): seed our q from the REAL arm once, or the first
        # command would order a move to URDF neutral from wherever the arm is.
        self.seeded = args.sim_state  # sim mode needs no seed
        if not self.seeded:
            self.qidx = {n: self.ik.model.joints[self.ik.model.getJointId(n)].idx_q
                         for n in ARM_JOINTS}
            self.create_subscription(JointState, "/joint_states", self._on_state, 1)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"ik up: {args.rate:.0f} Hz, sim_state={'on' if args.sim_state else 'off'}")

    def _on_state(self, msg: JointState):
        if self.seeded:
            return
        q = self.ik.q.copy()
        for name, pos in zip(msg.name, msg.position):
            if name in self.qidx:
                q[self.qidx[name]] = pos
        self.ik.set_q(q)
        self.ik.q_ref = self.ik.arm_q()  # anchor the real starting posture
        self.seeded = True
        self.get_logger().info("ik seeded from real /joint_states")

    def _on_target(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        raw = pin.SE3(
            pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(),
            np.array([p.x, p.y, p.z]))
        # EMA on the target: ARKit pose noise + hand tremor otherwise pass
        # straight into the servo. alpha=1 disables.
        a = self.smooth
        if self.target is None or a >= 1.0:
            self.target = raw
        else:
            t = self.target.translation * (1 - a) + raw.translation * a
            q0 = pin.Quaternion(self.target.rotation)
            self.target = pin.SE3(
                q0.slerp(a, pin.Quaternion(raw.rotation)).matrix(), t)
        self.last_target_time = time.monotonic()

    def _on_gripper(self, msg: Float64):
        self.gripper = msg.data

    def _tick(self):
        # Target-stream timeout: the receiver only publishes while the clutch
        # is held. Without this, releasing the clutch left the last target
        # live and the arm kept moving for seconds ("release = freeze" broken).
        # Backstop only - the receiver also publishes an explicit stop-here
        # target on release; this catches a crashed/wedged receiver too.
        if (self.target is not None and self.last_target_time is not None
                and time.monotonic() - self.last_target_time > 0.3):
            self.target = None
            self.get_logger().info("target stream silent: holding")
        if self.target is not None:
            self.ik.servo(self.target, self.dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # jaw mimic states ride along for robot_state_publisher/rviz;
        # the driver ignores the jaw names on /joint_command
        msg.name = ARM_JOINTS + [GRIPPER_JOINT] + list(JAW_MIMIC)
        msg.position = [*map(float, self.ik.arm_q()), float(self.gripper)] \
            + [r * float(self.gripper) for r in JAW_MIMIC.values()]
        self.cmd_pub.publish(msg)
        if self.state_pub:
            self.state_pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rate", type=float, default=100.0, help="servo rate Hz")
    p.add_argument("--smooth", type=float, default=0.35,
                   help="EMA alpha on the target pose (1 = no smoothing)")
    p.add_argument("--freeze", default="",
                   help="comma-separated joint names held at their seed pose")
    p.add_argument("--no-sim-state", dest="sim_state", action="store_false",
                   help="do not publish /joint_states (M3+: the driver owns it)")
    p.set_defaults(sim_state=True)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = IKNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
