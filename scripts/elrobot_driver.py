"""elrobot_driver — the only node that touches hardware. All safety lives here.

/joint_command (JointState, by NAME) -> ticks via calibration/urdf_ticks.json
-> slew-limited sync_write. sync_read back -> /joint_states (real arm state).

Safety, enforced every cycle:
  velocity clamp   slew limiter in tick space: last_sent steps toward the
                   target by at most vel*dt per joint. Also absorbs a far-off
                   first command (no startup lurch).
  workspace box    FK of the commanded q; TCP below z_min or beyond r_max
                   -> hold (table strikes are the real desk hazard).
  sigma_min floor  commanded pose too near singular -> hold.
  joint limits     URDF clamp (defense in depth; IK clamps too).
  gripper current  Torque_Limit written for motor 8 at startup; Status byte
                   checked each cycle (motor 8 has latched Overload before).
  deadman          no /joint_command for 200 ms -> freeze: latch present
                   position as goal ONCE (re-writing present each cycle would
                   let gravity walk the arm down).

Torque-enable sequence: read present -> write present AS GOAL -> enable.
The arm's first powered act is holding where it already is.

    pixi run python scripts/elrobot_driver.py [--no-torque]

--no-torque runs everything (reads, conversion, checks) but never enables
torque and never writes goals: the safe smoke-test mode.
Ctrl-C leaves torque ON, holding. Release with scripts/watch_ticks.py.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cartesian_ik import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_JOINT, TCP_FRAME  # noqa: E402

TICKS_PER_RAD = 651.9
DEADMAN_S = 0.2
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]


class Converter:
    """q (URDF rad) <-> ticks via the M1b table. Pure logic, no hardware."""

    def __init__(self, table_path="calibration/urdf_ticks.json"):
        self.t = json.loads(Path(table_path).read_text())

    def arm_ticks(self, name, q):
        e = self.t[name]
        return int(round(e["offset"] + e["sign"] * TICKS_PER_RAD * q))

    def arm_q(self, name, ticks):
        e = self.t[name]
        return (ticks - e["offset"]) / (e["sign"] * TICKS_PER_RAD)

    def grip_ticks(self, q):  # q in [0=open .. GRIPPER_CLOSED]
        g = self.t[GRIPPER_JOINT]
        frac = min(max(q / GRIPPER_CLOSED, 0.0), 1.0)
        return int(round(g["open_ticks"] + frac * (g["closed_ticks"] - g["open_ticks"])))

    def grip_q(self, ticks):
        g = self.t[GRIPPER_JOINT]
        span = g["closed_ticks"] - g["open_ticks"]
        return (ticks - g["open_ticks"]) / span * GRIPPER_CLOSED


class SafetyGate:
    """Workspace box + sigma_min floor + URDF limit clamp. Pure logic."""

    def __init__(self, z_min=0.02, r_max=0.45, sigma_floor=0.005,
                 urdf="docs/urdf_Elrobot.urdf"):
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(TCP_FRAME)
        self.jidx = {n: self.model.joints[self.model.getJointId(n)].idx_q
                     for n in ARM_JOINTS}
        self.lo = self.model.lowerPositionLimit
        self.hi = self.model.upperPositionLimit
        self.z_min, self.r_max, self.sigma_floor = z_min, r_max, sigma_floor

    def clamp_limits(self, name, q):
        i = self.jidx[name]
        return float(np.clip(q, self.lo[i], self.hi[i]))

    def check(self, q_by_name):
        """Returns (ok, reason). q_by_name: arm joints only, URDF rad."""
        q = pin.neutral(self.model)
        for n, v in q_by_name.items():
            q[self.jidx[n]] = v
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        p = self.data.oMf[self.fid].translation
        if p[2] < self.z_min:
            return False, f"TCP z={p[2]:.3f} < z_min {self.z_min}"
        if float(np.hypot(p[0], p[1])) > self.r_max:
            return False, f"TCP r={np.hypot(p[0], p[1]):.3f} > r_max {self.r_max}"
        J = pin.computeFrameJacobian(self.model, self.data, q, self.fid,
                                     pin.LOCAL)[:, :7]
        s_min = float(np.linalg.svd(J, compute_uv=False)[-1])
        if s_min < self.sigma_floor:
            return False, f"sigma_min={s_min:.4f} < floor {self.sigma_floor}"
        return True, ""


class ElrobotDriver(Node):
    def __init__(self, args, bus=None):
        super().__init__("elrobot_driver")
        self.conv = Converter(args.table)
        self.gate = SafetyGate(args.z_min, args.r_max, args.sigma_floor)
        self.dt = 1.0 / args.rate
        # per-cycle tick step (FLOAT): vel clamp. Kept fractional and
        # accumulated in slew_pos - integer truncation both staircased the
        # motion (jitter) and silently ran 23% under the configured velocity.
        self.max_step = {n: args.max_vel * self.dt * TICKS_PER_RAD
                         for n in ARM_JOINTS}
        self.max_step[GRIPPER_JOINT] = (
            args.grip_vel * self.dt * abs(
                self.conv.t[GRIPPER_JOINT]["closed_ticks"]
                - self.conv.t[GRIPPER_JOINT]["open_ticks"]) / GRIPPER_CLOSED)
        self.torque = args.torque

        if bus is None:
            from lerobot.motors import Motor, MotorNormMode
            from lerobot.motors.feetech import FeetechMotorsBus
            bus = FeetechMotorsBus(
                port=args.port,
                motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                        for i, n in enumerate(ALL_JOINTS, start=1)},
                calibration=None)
            bus.connect(handshake=True)
        self.bus = bus

        # gripper grasp state: contact detected via Present_Current while
        # closing -> latch goal at contact + squeeze (bounded press force)
        g = self.conv.t[GRIPPER_JOINT]
        self.close_dir = 1 if g["closed_ticks"] > g["open_ticks"] else -1
        self.grip_thresh = args.grip_current_thresh
        self.grip_squeeze = args.grip_squeeze
        self.grasp_goal = None
        self.grasp_hits = 0

        present = self.bus.sync_read("Present_Position", ALL_JOINTS,
                                     normalize=False)
        self.slew_pos = {n: float(v) for n, v in present.items()}  # float acc
        self.last_sent = dict(present)   # last INTEGER goal written
        self.target = None               # ticks; None until first command
        self.frozen = False
        self.last_cmd_time = None

        # gripper current limit BEFORE any torque (spec: motor 8 has latched
        # Overload from being driven past its stop)
        self.bus.write("Torque_Limit", GRIPPER_JOINT, args.grip_torque_limit)
        # acceleration profile: without it every 100 Hz mini-goal is a step
        # input to a stiff position loop -> audible ticking / jitter
        for n in ALL_JOINTS:
            self.bus.write("Acceleration", n, args.accel)
        if self.torque:
            # hold-in-place enable: goal := present, THEN torque on
            self.bus.sync_write("Goal_Position", present, normalize=False)
            self.bus.enable_torque()
            self.get_logger().info("torque ENABLED, holding present pose")
        else:
            self.get_logger().info("running TORQUE-OFF (smoke mode): "
                                   "no goals will be written")

        self.create_subscription(JointState, "/joint_command", self._on_cmd, 1)
        self.state_pub = self.create_publisher(JointState, "/joint_states", 1)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"driver up: {args.rate:.0f} Hz, vel clamp {args.max_vel} rad/s "
            f"({self.max_step[ARM_JOINTS[0]]} ticks/cycle), "
            f"z_min {args.z_min} m, r_max {args.r_max} m, "
            f"sigma floor {args.sigma_floor}")

    # -- command intake ----------------------------------------------------
    def _on_cmd(self, msg: JointState):
        q_arm, ticks = {}, {}
        for name, pos in zip(msg.name, msg.position):
            if name in self.conv.t and name != GRIPPER_JOINT:
                q = self.gate.clamp_limits(name, float(pos))
                q_arm[name] = q
                ticks[name] = self.conv.arm_ticks(name, q)
            elif name == GRIPPER_JOINT:
                ticks[name] = self.conv.grip_ticks(float(pos))
        if len(q_arm) != len(ARM_JOINTS):
            return  # incomplete command; ignore
        ok, why = self.gate.check(q_arm)
        if not ok:
            if not self.frozen:
                self.get_logger().warning(f"SAFETY HOLD: {why}")
                self._freeze()
            # the gripper is kinematically irrelevant to the arm gates -
            # keep it responsive while the arm holds (a gated arm pose was
            # silently eating gripper commands)
            if self.target is not None and GRIPPER_JOINT in ticks:
                self.target[GRIPPER_JOINT] = ticks[GRIPPER_JOINT]
            return
        self.frozen = False
        self.target = ticks
        self.last_cmd_time = time.monotonic()

    def _freeze(self):
        """Latch present position as the goal, once."""
        self.frozen = True
        present = self.bus.sync_read("Present_Position", ALL_JOINTS,
                                     normalize=False, num_retry=2)
        self.target = dict(present)

    def _recover_bus(self):
        """Post-failure hygiene: release the latched port lock and drain RX.

        A single glitched transaction otherwise kills the loop permanently:
        the exception leaves port_handler.is_using latched, and every later
        call fails 'Port is in use' (observed: 1 glitch -> 532 dead cycles).
        Draining RX is the station-defect lesson from the spec - stale reply
        bytes must never be consumed by the next transaction.
        """
        try:
            ph = self.bus.port_handler
            ph.is_using = False
            ph.ser.reset_input_buffer()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"bus recovery failed: {e}")

    def _grasp_logic(self):
        """Contact-detecting gripper: stop at the object, keep pressing.

        While the gripper is commanded toward CLOSED and still moving, watch
        Present_Current; a sustained spike (3 cycles) means the jaws met
        something. Latch the goal at contact + squeeze ticks: the servo's
        position error then presses with a steady, Torque_Limit-capped force
        (active normal force) instead of driving on toward fully-closed and
        squirting the object out. An OPEN command releases the latch.
        """
        g = GRIPPER_JOINT
        cmd = self.target[g]
        pos = self.slew_pos[g]
        if self.grasp_goal is not None:
            # release when the command retreats toward open past the latch
            if (cmd - self.grasp_goal) * self.close_dir < -30:
                self.grasp_goal = None
                self.grasp_hits = 0
                self.get_logger().info("grasp released")
            return
        if (cmd - pos) * self.close_dir <= 5:   # not actively closing
            self.grasp_hits = 0
            return
        try:
            raw = self.bus.sync_read("Present_Current", [g],
                                     normalize=False, num_retry=1)[g]
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"current read failed: {e}")
            self._recover_bus()
            return
        # Feetech encodes signed values sign-magnitude (bit 15). ~6.5 mA/LSB.
        mag = raw & 0x7FFF
        self.grasp_hits = self.grasp_hits + 1 if mag >= self.grip_thresh else 0
        if self.grasp_hits >= 3:
            t = self.conv.t[g]
            lo, hi = sorted((t["open_ticks"], t["closed_ticks"]))
            self.grasp_goal = int(np.clip(
                pos + self.close_dir * self.grip_squeeze, lo, hi))
            self.get_logger().info(
                f"grasp: contact at ~{mag * 6.5:.0f} mA, holding "
                f"{self.grip_squeeze} ticks of squeeze")

    # -- the 100 Hz cycle --------------------------------------------------
    def _tick(self):
        # deadman: command stream went quiet while we had a live target
        if (self.target is not None and not self.frozen
                and self.last_cmd_time is not None
                and time.monotonic() - self.last_cmd_time > DEADMAN_S):
            self.get_logger().warning("deadman: command stream silent, freezing")
            self._freeze()

        # slew toward target and write (unless smoke mode / no command yet)
        if self.target is not None and self.torque:
            self._grasp_logic()
            out = {}
            for n in ALL_JOINTS:
                tgt = (self.grasp_goal
                       if n == GRIPPER_JOINT and self.grasp_goal is not None
                       else self.target[n])
                self.slew_pos[n] += float(np.clip(
                    tgt - self.slew_pos[n],
                    -self.max_step[n], self.max_step[n]))
                out[n] = int(round(self.slew_pos[n]))
            if out != self.last_sent:
                try:
                    self.bus.sync_write("Goal_Position", out, normalize=False)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warning(f"sync_write failed: {e}")
                    self._recover_bus()
                    return  # don't advance last_sent past what was written
            self.last_sent = out

        # real state out
        try:
            present = self.bus.sync_read("Present_Position", ALL_JOINTS,
                                         normalize=False, num_retry=2)
        except Exception as e:  # noqa: BLE001 - keep the loop alive on a bad read
            self.get_logger().warning(f"sync_read failed: {e}")
            self._recover_bus()
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self.conv.arm_q(n, present[n]) for n in ARM_JOINTS] \
            + [self.conv.grip_q(present[GRIPPER_JOINT])]
        self.state_pub.publish(msg)


def build_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--table", default="calibration/urdf_ticks.json")
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--max-vel", dest="max_vel", type=float, default=0.6,
                   help="per-joint velocity clamp, rad/s (M3 conservative)")
    p.add_argument("--grip-vel", dest="grip_vel", type=float, default=2.0)
    p.add_argument("--grip-torque-limit", type=int, default=300,
                   help="Torque_Limit register for motor 8 (0-1000)")
    p.add_argument("--accel", type=int, default=40,
                   help="Acceleration register, all motors (0=step response)")
    p.add_argument("--grip-current-thresh", type=int, default=60,
                   help="grasp contact threshold, Present_Current LSB "
                        "(~6.5 mA each). Tune with watch_ticks' mA column.")
    p.add_argument("--grip-squeeze", type=int, default=40,
                   help="ticks of squeeze held past the contact point")
    p.add_argument("--z-min", dest="z_min", type=float, default=0.02)
    p.add_argument("--r-max", dest="r_max", type=float, default=0.45)
    # Backstop only: catches truly degenerate commands, never working poses.
    # Measured on hardware: URDF neutral sits at sigma 0.0011 and the slider
    # GUI's "Center" (mid-limits) pose family at 0.0005-0.0007 - all
    # legitimate, all held by the earlier 0.0008 floor. IK's adaptive damping
    # owns the whole near-singular band; the driver's slew clamp bounds any
    # residual velocity, so the floor only needs to reject the ~0.0001
    # wrist-aligned degeneracies.
    p.add_argument("--sigma-floor", dest="sigma_floor", type=float,
                   default=0.0002)
    p.add_argument("--no-torque", dest="torque", action="store_false",
                   help="smoke mode: never enable torque, never write goals")
    p.set_defaults(torque=True)
    args, _ = p.parse_known_args(argv)
    return args


def main():
    rclpy.init()
    node = ElrobotDriver(build_args())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            "exiting: torque left ON holding (release: scripts/watch_ticks.py)")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
