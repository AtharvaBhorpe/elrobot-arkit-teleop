"""ik — Cartesian servo node: /target_pose -> /joint_command (+ /joint_states).

Servos the 7 arm joints toward the latest target pose at a fixed rate;
rev_motor_08 passes through from /gripper_command untouched by IK.

Publishes:
  /joint_command  JointState, 8 positions — consumed by elrobot_driver (M3+)
  /joint_states   same content — drives robot_state_publisher/rviz in M2,
                  where there is no hardware to report real state. In M3 the
                  driver owns /joint_states; run with --no-sim-state then.

    pixi run python -m elrobot.nodes.ik_node
"""

import argparse
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from elrobot.control.cartesian_ik import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_JOINT,
    GRIPPER_OPEN,
    JAW_MIMIC,
    CartesianServoIK,
)

MODE_FROZEN = {
    "7dof": (),
    "6dof": ("rev_motor_03",),
    "5dof": ("rev_motor_03", "rev_motor_05"),
}
STATE_FRESH_S = 0.5


class IKNode(Node):
    def __init__(self, args):
        super().__init__("ik")
        frozen = tuple(f for f in args.freeze.split(",") if f)
        self.ik = CartesianServoIK(frozen=frozen)
        self.mode = next((m for m, f in MODE_FROZEN.items() if f == frozen), "custom")
        self.enabled = True
        self.latest_state = None
        self.latest_state_time = None
        if frozen:
            self.get_logger().info(f"SO-101 mode: frozen {list(frozen)}")
        self.ik.set_q_ref()  # null-space anchor; re-set on seed
        self.target: pin.SE3 | None = None
        self.gripper = GRIPPER_OPEN
        self.dt = 1.0 / args.rate
        self.smooth = args.smooth
        self.last_target_time = None

        mode_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, "/target_pose", self._on_target, 1)
        self.create_subscription(Float64, "/gripper_command", self._on_gripper, 1)
        self.create_subscription(String, "/teleop_mode", self._on_mode, mode_qos)
        self.mode_status_pub = self.create_publisher(
            String, "/teleop_mode/status", mode_qos
        )
        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 1)
        self.state_pub = (
            self.create_publisher(JointState, "/joint_states", 1) if args.sim_state else None
        )
        # M3 (driver present): seed our q from the REAL arm once, or the first
        # command would order a move to URDF neutral from wherever the arm is.
        self.seeded = args.sim_state  # sim mode needs no seed
        if not self.seeded:
            self.qidx = {
                n: self.ik.model.joints[self.ik.model.getJointId(n)].idx_q for n in ARM_JOINTS
            }
            self.create_subscription(JointState, "/joint_states", self._on_state, 1)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"ik up: {args.rate:.0f} Hz, sim_state={'on' if args.sim_state else 'off'}"
        )

    def _on_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in ARM_JOINTS + [GRIPPER_JOINT]):
            return
        self.latest_state = {
            name: float(values[name]) for name in ARM_JOINTS + [GRIPPER_JOINT]
        }
        self.latest_state_time = time.monotonic()
        if self.seeded:
            return
        self._sync_real_state()
        self.seeded = True
        self.get_logger().info("ik seeded from real /joint_states")

    def _sync_real_state(self) -> bool:
        if (
            self.latest_state is None
            or self.latest_state_time is None
            or time.monotonic() - self.latest_state_time > STATE_FRESH_S
        ):
            return False
        q = self.ik.q.copy()
        for name in ARM_JOINTS:
            q[self.qidx[name]] = self.latest_state[name]
        self.ik.set_q(q)
        self.gripper = self.latest_state[GRIPPER_JOINT]
        self.ik.set_q_ref()  # anchor the real starting posture
        return True

    def _on_target(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        raw = pin.SE3(
            pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(), np.array([p.x, p.y, p.z])
        )
        # EMA on the target: ARKit pose noise + hand tremor otherwise pass
        # straight into the servo. alpha=1 disables.
        a = self.smooth
        if self.target is None or a >= 1.0:
            self.target = raw
        else:
            t = self.target.translation * (1 - a) + raw.translation * a
            q0 = pin.Quaternion(self.target.rotation)
            self.target = pin.SE3(q0.slerp(a, pin.Quaternion(raw.rotation)).matrix(), t)
        self.last_target_time = time.monotonic()

    def _on_gripper(self, msg: Float64):
        self.gripper = msg.data

    def _on_mode(self, msg: String):
        mode = msg.data.strip().lower()
        if mode == "replay":
            self.enabled = False
            self.target = None
            self.mode_status_pub.publish(String(data="replay"))
            self.get_logger().info("physical replay: IK publisher paused")
            return
        if mode == "resume":
            self.enabled = False
            self.target = None
            if not self._sync_real_state():
                self.mode_status_pub.publish(String(data="resume-blocked"))
                self.get_logger().warning(
                    "phone teleop remains paused: no fresh complete real joint state"
                )
                return
            self.enabled = True
            self.mode_status_pub.publish(String(data="resume"))
            self.get_logger().info(f"phone teleop resumed in {self.mode}")
            return
        if mode == "web":
            self.enabled = False
            self.target = None
            self.mode_status_pub.publish(String(data="web"))
            self.get_logger().info("web control mode: IK publisher paused")
            return
        if mode not in MODE_FROZEN:
            self.get_logger().warning(f"ignoring unknown teleop mode: {mode!r}")
            return
        self.enabled = True
        self.mode = mode
        self.target = None
        self.ik.set_frozen(MODE_FROZEN[mode])
        self.ik.set_q_ref()
        self.mode_status_pub.publish(String(data=mode))
        self.get_logger().info(f"phone teleop mode: {mode}")

    def _tick(self):
        # Target-stream timeout: the receiver only publishes while the clutch
        # is held. Without this, releasing the clutch left the last target
        # live and the arm kept moving for seconds ("release = freeze" broken).
        # Backstop only - the receiver also publishes an explicit stop-here
        # target on release; this catches a crashed/wedged receiver too.
        if (
            self.target is not None
            and self.last_target_time is not None
            and time.monotonic() - self.last_target_time > 0.3
        ):
            self.target = None
            self.get_logger().info("target stream silent: holding")
        if not self.enabled:
            return
        if self.target is not None:
            self.ik.servo(self.target, self.dt)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # jaw mimic states ride along for robot_state_publisher/rviz;
        # the driver ignores the jaw names on /joint_command
        msg.name = ARM_JOINTS + [GRIPPER_JOINT] + list(JAW_MIMIC)
        msg.position = [*map(float, self.ik.arm_q()), float(self.gripper)] + [
            r * float(self.gripper) for r in JAW_MIMIC.values()
        ]
        self.cmd_pub.publish(msg)
        if self.state_pub:
            self.state_pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--rate", type=float, default=100.0, help="servo rate Hz")
    p.add_argument(
        "--smooth", type=float, default=0.35, help="EMA alpha on the target pose (1 = no smoothing)"
    )
    p.add_argument(
        "--freeze", default="", help="comma-separated joint names held at their seed pose"
    )
    p.add_argument(
        "--no-sim-state",
        dest="sim_state",
        action="store_false",
        help="do not publish /joint_states (M3+: the driver owns it)",
    )
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
