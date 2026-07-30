"""arkit_receiver — ZIG SIM PRO iPhone pose + touch -> /target_pose.

Ported from franka-isaac-arkit-teleop with the robot swapped (Elrobot URDF,
TCP frame Gripper_Base_v1_1) and the motion scale dropped to 0.4 for the
0.42 m reach. UDP only — ZIG SIM's default and the lowest-latency transport.

Interaction (finger count from ZIG SIM's touch array):
  1 finger held  clutch engaged; phone motion drives the TCP. Reference pose
                 re-zeroed on engage, so re-engaging never jumps.
  release        freeze immediately.
  2-finger tap   toggle gripper open/closed (latched).
  200 ms silence clutch force-released (stream deadman; the driver has its
                 own freeze in M3+ — this one just guarantees a re-zero).

    pixi run python -m elrobot.nodes.arkit_receiver
"""

import argparse
import json
import socket
import threading
import time

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from elrobot.control.cartesian_ik import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    CartesianServoIK,
)

# ARKit world (RH, +Y up, camera looks -Z) -> robot base (+Z up, arm along +Y).
# Operator stance: standing at the base, looking out along the arm (+Y),
# phone camera pointing at the robot:
#   phone right   (+X arkit) -> robot right   (+X)
#   phone forward (-Z arkit) -> arm forward   (+Y)
#   phone up      (+Y arkit) -> robot up      (+Z)
# Rotations go through C.R.C^T, so the phone's screwdriver axis (camera, -Z)
# lands on the arm's length axis (+Y): roll the phone while pointing it at the
# robot and the gripper rolls the same way, clockwise-for-clockwise as seen
# from the base. (The Franka project's map had the operator 90 deg around --
# phone-right came out as robot-backward here.)
ARKIT_TO_ROS = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]
)

DEADMAN_S = 0.2  # tolerates 1 dropped packet at ZIG SIM's 10 Hz, fires on the 2nd


class ARKitReceiver(Node):
    def __init__(self, args):
        super().__init__("arkit_receiver")
        self.scale = args.scale
        self.orient = args.orient
        self.quat_order = args.quat_order
        self.C = ARKIT_TO_ROS

        # FK only (reuses the IK class for model + ee_pose)
        self.fk = CartesianServoIK()
        self.qidx = {n: self.fk.model.joints[self.fk.model.getJointId(n)].idx_q for n in ARM_JOINTS}
        self._latest_q = None
        self.robot_ref = None
        self.phone_ref = None
        self.phone_rot_ref = None
        self.moving = False
        self.gripper_closed = False
        self.prev_n = 0
        self._last_rx = None
        self._last_rot_log = 0.0

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose", 1)
        self.grip_pub = self.create_publisher(Float64, "/gripper_command", 1)

        self.host, self.port = args.host, args.port
        self._rx_arrived = self._rx_handled = 0
        threading.Thread(target=self._udp_serve, daemon=True).start()
        self.create_timer(0.05, self._check_deadman)
        self.create_timer(2.0, self._log_rx)
        self.get_logger().info(
            f"arkit_receiver up: UDP :{self.port}  scale={self.scale}\n"
            "   1 finger = move, 0 = freeze, 2-finger tap = toggle gripper"
        )

    def _on_joint_states(self, msg: JointState):
        q = self.fk.q.copy()
        for name, pos in zip(msg.name, msg.position):
            if name in self.qidx:
                q[self.qidx[name]] = pos
        self._latest_q = q

    def _check_deadman(self):
        if (
            self.moving
            and self._last_rx is not None
            and time.monotonic() - self._last_rx > DEADMAN_S
        ):
            self.moving = False
            self._publish_stop()
            self.get_logger().warning("stream deadman: clutch released")

    def _publish_stop(self):
        """Release = freeze IMMEDIATELY: final target := current robot pose.

        Without this the ik node kept chasing the last clutched target for
        seconds after release. Its 0.3 s stream timeout remains the backstop.
        """
        if self._latest_q is None:
            return
        self.fk.set_q(self._latest_q)
        self._publish_pose(self.fk.ee_pose())

    def _publish_pose(self, M):
        quat = pin.Quaternion(M.rotation)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, M.translation)
        msg.pose.orientation.w = float(quat.w)
        msg.pose.orientation.x = float(quat.x)
        msg.pose.orientation.y = float(quat.y)
        msg.pose.orientation.z = float(quat.z)
        self.pose_pub.publish(msg)

    def _log_rot_axis(self, d_robot):
        """1 Hz: dominant rotation axis in the base frame, for axis debugging.
        X=right, Y=arm-forward (roll axis), Z=up (yaw axis)."""
        now = time.monotonic()
        if now - self._last_rot_log < 1.0:
            return
        aa = pin.AngleAxis(d_robot)
        deg = np.degrees(aa.angle)
        if deg < 15.0:
            return
        self._last_rot_log = now
        ax = aa.axis
        dom = "XYZ"[int(np.argmax(np.abs(ax)))]
        self.get_logger().info(
            f"phone rotation: {deg:.0f} deg about "
            f"[{ax[0]:+.2f} {ax[1]:+.2f} {ax[2]:+.2f}] (mostly {dom})"
        )

    def _quat_to_R(self, q):
        if self.quat_order == "xyzw":
            x, y, z, w = q
        else:
            w, x, y, z = q
        return pin.Quaternion(float(w), float(x), float(y), float(z)).normalized().matrix()

    def _udp_serve(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        sock.bind((self.host, self.port))
        while rclpy.ok():
            data, _ = sock.recvfrom(65535)
            self._rx_arrived += 1
            sock.setblocking(False)
            while True:  # drain backlog -> act on the freshest frame only
                try:
                    data, _ = sock.recvfrom(65535)
                    self._rx_arrived += 1
                except BlockingIOError:
                    break
            sock.setblocking(True)
            self._feed(data)

    def _feed(self, data: bytes):
        try:
            obj = json.loads(data.decode("utf-8"))
            sensors = obj["sensordata"]
            arkit = sensors["arkit"]
            pos = np.asarray(arkit["position"], dtype=float)
            rot = arkit["rotation"]
        except (KeyError, ValueError, TypeError, UnicodeDecodeError):
            return
        self._rx_handled += 1
        self._last_rx = time.monotonic()
        touch = sensors.get("touch") or []
        try:
            n = len(touch) if isinstance(touch, list) else int(touch)
        except (TypeError, ValueError):
            return
        self._process(pos, rot, n)

    def _log_rx(self):
        if self._rx_arrived or self._rx_handled:
            self.get_logger().info(
                f"rx: {self._rx_arrived / 2:.0f}/s arrived, "
                f"{self._rx_handled / 2:.0f}/s handled (latest-only)"
            )
        self._rx_arrived = self._rx_handled = 0

    def _process(self, pos: np.ndarray, rot, n: int):
        # Gripper: toggle on rising edge into >=2 fingers; latched.
        if n >= 2 and self.prev_n < 2:
            self.gripper_closed = not self.gripper_closed
            self.get_logger().info(f"gripper -> {'CLOSED' if self.gripper_closed else 'OPEN'}")
        self.grip_pub.publish(Float64(data=GRIPPER_CLOSED if self.gripper_closed else GRIPPER_OPEN))

        # Clutch: exactly 1 finger = moving; re-zero both refs on engage.
        moving = n == 1
        if moving and not self.moving:
            if self._latest_q is not None:
                self.fk.set_q(self._latest_q)
                self.robot_ref = self.fk.ee_pose()
                self.phone_ref = pos
                self.phone_rot_ref = self._quat_to_R(rot)
                self.moving = True
                self.get_logger().info("move engaged")
        elif not moving and self.moving:
            self.moving = False
            self._publish_stop()
            self.get_logger().info("move released (stop-here target sent)")

        robot_ref = self.robot_ref
        phone_ref = self.phone_ref
        phone_rot_ref = self.phone_rot_ref
        if (
            self.moving
            and robot_ref is not None
            and phone_ref is not None
            and phone_rot_ref is not None
        ):
            delta = self.C @ (pos - phone_ref) * self.scale
            target_pos = robot_ref.translation + delta

            if self.orient:
                # dR_robot = C (R_now R_ref^T) C^T, applied to the engage pose.
                d_arkit = self._quat_to_R(rot) @ phone_rot_ref.T
                d_robot = self.C @ d_arkit @ self.C.T
                target_R = d_robot @ robot_ref.rotation
                self._log_rot_axis(d_robot)
            else:
                target_R = robot_ref.rotation
            self._publish_pose(pin.SE3(target_R, target_pos))

        self.prev_n = n


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=50000)
    p.add_argument(
        "--scale",
        type=float,
        default=0.4,
        help="phone->TCP translation gain (spec default, tune in M4)",
    )
    p.add_argument(
        "--no-orient",
        dest="orient",
        action="store_false",
        help="position-only (TCP keeps engage orientation)",
    )
    p.set_defaults(orient=True)
    p.add_argument("--quat-order", choices=["xyzw", "wxyz"], default="xyzw")
    args, _ = p.parse_known_args()

    rclpy.init()
    node = ARKitReceiver(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
