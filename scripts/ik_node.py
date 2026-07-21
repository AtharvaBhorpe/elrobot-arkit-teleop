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
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cartesian_ik import ARM_JOINTS, GRIPPER_JOINT, GRIPPER_OPEN, CartesianServoIK  # noqa: E402


class IKNode(Node):
    def __init__(self, args):
        super().__init__("ik")
        self.ik = CartesianServoIK()
        self.target: pin.SE3 | None = None
        self.gripper = GRIPPER_OPEN
        self.dt = 1.0 / args.rate

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
        self.seeded = True
        self.get_logger().info("ik seeded from real /joint_states")

    def _on_target(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.target = pin.SE3(
            pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(),
            np.array([p.x, p.y, p.z]))

    def _on_gripper(self, msg: Float64):
        self.gripper = msg.data

    def _tick(self):
        if self.target is not None:
            self.ik.servo(self.target, self.dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + [GRIPPER_JOINT]
        msg.position = [*map(float, self.ik.arm_q()), float(self.gripper)]
        self.cmd_pub.publish(msg)
        if self.state_pub:
            self.state_pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rate", type=float, default=100.0, help="servo rate Hz")
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
