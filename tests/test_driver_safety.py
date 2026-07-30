"""Driver safety unit tests - stub bus, no hardware, no torque, no risk.

Exercises every safety mechanism in elrobot_driver against a recorded-call
stub: conversion round-trip, limit clamp, workspace hold, sigma floor,
slew/velocity clamp, deadman, freeze latching, torque-enable ordering,
smoke-mode write suppression, gripper Torque_Limit.

    pixi run python tests/test_driver_safety.py
"""

import os
import time

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session

import rclpy
from sensor_msgs.msg import JointState

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_CLOSED, GRIPPER_JOINT  # noqa: E402
from elrobot.nodes.elrobot_driver import (  # noqa: E402
    TICKS_PER_RAD,
    Converter,
    ElrobotDriver,
    SafetyGate,
    build_args,
)


class StubBus:
    def __init__(self, present):
        self.present = dict(present)
        self.load = {n: 0 for n in present}  # Present_Load raw LSB (signed)
        self.calls = []
        self.fail_goal_writes = 0
        self.fail_present_reads = 0
        self.fail_connects = 0
        self.connected = True

    def sync_read(self, reg, motors=None, normalize=True, num_retry=0):
        self.calls.append(("sync_read", reg))
        if reg == "Present_Position" and self.fail_present_reads:
            self.fail_present_reads -= 1
            raise RuntimeError("stub read failure")
        src = self.load if reg == "Present_Load" else self.present
        return {n: src[n] for n in (motors or src)}

    def sync_write(self, reg, values, normalize=True, num_retry=0):
        self.calls.append(("sync_write", reg, dict(values)))
        if reg == "Goal_Position" and self.fail_goal_writes:
            self.fail_goal_writes -= 1
            raise RuntimeError("stub write failure")

    def write(self, reg, motor, value):
        self.calls.append(("write", reg, motor, value))

    def enable_torque(self, *a, **k):
        self.calls.append(("enable_torque",))

    def disconnect(self, disable_torque=True):
        if not self.connected:
            raise RuntimeError("bus is already disconnected")
        self.calls.append(("disconnect", disable_torque))
        self.connected = False

    def connect(self, handshake=True):
        self.calls.append(("connect", handshake))
        if self.fail_connects:
            self.fail_connects -= 1
            raise RuntimeError("stub reconnect failure")
        self.connected = True

    @property
    def is_connected(self):
        return self.connected

    def goal_writes(self):
        return [c for c in self.calls if c[0] == "sync_write" and c[1] == "Goal_Position"]


class StubPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def cmd(q_arm, grip=0.0):
    m = JointState()
    m.name = list(ARM_JOINTS) + [GRIPPER_JOINT]
    m.position = [q_arm.get(n, 0.0) for n in ARM_JOINTS] + [grip]
    return m


def make_driver(present, **overrides):
    a = build_args([])
    for k, v in overrides.items():
        setattr(a, k, v)
    return ElrobotDriver(a, bus=StubBus(present))


def main():
    rclpy.init()
    conv = Converter()
    # present = the arm sitting at URDF zero on every joint, gripper mid
    present = {n: conv.arm_ticks(n, 0.0) for n in ARM_JOINTS}
    present[GRIPPER_JOINT] = conv.grip_ticks(GRIPPER_CLOSED / 2)

    # 1) conversion round-trips
    for n in ARM_JOINTS:
        for q in (-1.0, -0.3, 0.0, 0.7, 1.2):
            assert abs(conv.arm_q(n, conv.arm_ticks(n, q)) - q) < 2 / TICKS_PER_RAD
    g = conv.t[GRIPPER_JOINT]
    assert conv.grip_ticks(0.0) == g["open_ticks"]
    assert conv.grip_ticks(GRIPPER_CLOSED) == g["closed_ticks"]
    assert abs(conv.grip_q(g["closed_ticks"]) - GRIPPER_CLOSED) < 1e-9
    print("1. conversion round-trips OK")

    # 2) torque-enable ordering: goal := present BEFORE enable_torque
    d = make_driver(present)
    idx = {c[0]: i for i, c in enumerate(d.bus.calls)}
    gw = d.bus.goal_writes()
    assert gw and gw[0][2] == present, "first goal write must be present pose"
    assert d.bus.calls.index(gw[0]) < idx["enable_torque"]
    tl = [c for c in d.bus.calls if c[0] == "write" and c[1] == "Torque_Limit"]
    assert tl and tl[0][2] == GRIPPER_JOINT and tl[0][3] == 300
    assert d.bus.calls.index(tl[0]) < idx["enable_torque"]
    print("2. torque-enable ordering + gripper Torque_Limit OK")

    # 3) smoke mode writes nothing, ever
    d = make_driver(present, torque=False)
    d._on_cmd(cmd({n: 0.3 for n in ARM_JOINTS}))
    for _ in range(20):
        d._tick()
    assert not d.bus.goal_writes(), "smoke mode must never write goals"
    assert not any(c[0] == "enable_torque" for c in d.bus.calls)
    print("3. smoke mode (--no-torque) writes no goals OK")

    # 4) slew limiter: far command -> acceleration-limited float steps,
    #    bounded velocity, braking, and no overshoot
    d = make_driver(present)
    n0 = ARM_JOINTS[0]
    q_t = 1.0  # ~652 ticks away from present (q=0)
    d._on_cmd(cmd({n0: q_t}))
    prev = dict(d.last_sent)
    prev_vel = d.slew_vel[n0]
    prev_step = 0
    goal_ticks = conv.arm_ticks(n0, q_t)
    import math
    for i in range(3000):
        d._on_cmd(cmd({n0: q_t}))  # keep deadman fed
        d._tick()
        step = d.last_sent[n0] - prev[n0]
        assert abs(step) <= math.ceil(d.max_step[n0]), f"step {step} too big"
        assert abs(step - prev_step) <= math.ceil(d.max_dv) + 1, \
            "written goal acceleration limit exceeded"
        assert abs(d.slew_vel[n0] - prev_vel) <= d.max_dv + 1e-9, \
            "acceleration limit exceeded"
        assert (goal_ticks - d.last_sent[n0]) * (goal_ticks - prev[n0]) >= 0, \
            "overshot the target"
        prev = dict(d.last_sent)
        prev_vel = d.slew_vel[n0]
        prev_step = step
        if d.last_sent[n0] == goal_ticks:
            break
    assert d.last_sent[n0] == goal_ticks, "must converge to target"
    assert i < 200, f"took {i} cycles - float accumulator not accumulating?"

    # A streaming target reversal decelerates toward the old safe target
    # before moving toward the new one; the written integer goals stay
    # acceleration bounded.
    d = make_driver(present)
    d._on_cmd(cmd({n0: 1.0}))
    last = d.last_sent[n0]
    last_step = 0
    for _ in range(40):
        d._on_cmd(cmd({n0: 1.0}))
        d._tick()
        step = d.last_sent[n0] - last
        last, last_step = d.last_sent[n0], step
    d._on_cmd(cmd({n0: -0.5}))
    for _ in range(40):
        d._on_cmd(cmd({n0: -0.5}))
        d._tick()
        step = d.last_sent[n0] - last
        assert abs(step - last_step) <= math.ceil(d.max_dv) + 1, \
            "reversal jerk exceeded acceleration limit"
        last, last_step = d.last_sent[n0], step

    # A failed bus write rolls every trajectory accumulator back. The next
    # successful write cannot jump ahead through the unwritten motion.
    d = make_driver(present)
    d._on_cmd(cmd({n0: 1.0}))
    d.bus.fail_goal_writes = 1
    for _ in range(20):
        snapshot = (dict(d.slew_pos), dict(d.slew_vel),
                    dict(d.slew_target), dict(d.brake_bound))
        failures = d.bus.fail_goal_writes
        d._on_cmd(cmd({n0: 1.0}))
        d._tick()
        if failures and not d.bus.fail_goal_writes:
            assert (d.slew_pos, d.slew_vel,
                    d.slew_target, d.brake_bound) == snapshot
            break
    else:
        raise AssertionError("stub write failure was not exercised")
    before = d.last_sent[n0]
    d._on_cmd(cmd({n0: 1.0}))
    d._tick()
    assert abs(d.last_sent[n0] - before) <= math.ceil(d.max_step[n0])

    # Goal telemetry is emitted immediately after a successful write, even
    # when the physical-state read later in the same cycle fails.
    d = make_driver(present, max_accel=60.0)
    d.goal_pub = StubPublisher()
    d.bus.fail_present_reads = 1
    d._on_cmd(cmd({n0: 1.0}))
    d._tick()
    assert len(d.goal_pub.messages) == 1
    assert d.goal_pub.messages[0].position[0] == conv.arm_q(
        n0, d.last_sent[n0])

    print(f"4. acceleration-limited slew OK "
          f"({d.max_step[n0]:.1f} ticks/cycle max, "
          f"converged in {i} cycles, reversal + rollback safe)")

    # 5) joint-limit clamp: command beyond URDF limit is clamped
    d = make_driver(present)
    d._on_cmd(cmd({n0: 99.0}))
    hi = d.gate.hi[d.gate.jidx[n0]]
    assert d.target[n0] == conv.arm_ticks(n0, hi)
    print("5. URDF joint-limit clamp OK")

    # 6) workspace hold: gripper driven at the table -> SAFETY HOLD + freeze
    gate = SafetyGate()
    q_bad = {n: 0.0 for n in ARM_JOINTS}
    q_bad["rev_motor_02"] = 1.6  # shoulder full forward-down
    ok, why = gate.check(q_bad)
    assert not ok and "z" in why, (ok, why)
    d = make_driver(present)
    d._on_cmd(cmd(q_bad))
    assert d.frozen and \
        {n: d.target[n] for n in ARM_JOINTS} == \
        {n: present[n] for n in ARM_JOINTS}, "arm must freeze at present"
    # the gripper stays responsive while the arm is held
    d._on_cmd(cmd(q_bad, grip=GRIPPER_CLOSED))
    assert d.target[GRIPPER_JOINT] == conv.grip_ticks(GRIPPER_CLOSED), \
        "gripper command must survive an arm safety hold"
    assert d.frozen, "gripper update must not unfreeze the arm"
    # and recovery: a good command un-freezes
    d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}))
    assert not d.frozen
    print(f"6. workspace hold + gripper-passthrough + recovery OK ({why})")

    # 7) sigma floor mechanism (floor raised so neutral trips it)
    d = make_driver(present, sigma_floor=0.02)
    d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}))
    assert d.frozen, "raised floor must hold near-singular neutral"
    print("7. sigma-floor hold OK")

    # 8) deadman: silence > 200 ms freezes at present
    d = make_driver(present)
    d._on_cmd(cmd({n0: 0.5}))
    d._tick()
    moved = dict(d.last_sent)
    time.sleep(0.25)
    d._tick()
    assert d.frozen and d.target == d.bus.present, "deadman must latch present"
    # frozen target stays latched across further ticks (no gravity-follow)
    latched = dict(d.target)
    for _ in range(10):
        d._tick()
    assert d.target == latched
    assert moved  # silence the linter: we did move first
    print("8. deadman freeze + latch OK")

    # 9) grasp: goal pushing past the PHYSICAL position + sustained load ->
    #    latch at physical contact + squeeze; deeper-close commands ignored;
    #    open command releases. The stub's physical position never moves,
    #    which models a fully stalled jaw (the case the old slew-based
    #    window missed entirely).
    d = make_driver(present)
    g_open = conv.grip_ticks(0.0)
    d.bus.present[GRIPPER_JOINT] = g_open
    d.last_present[GRIPPER_JOINT] = g_open
    d.slew_pos[GRIPPER_JOINT] = float(g_open)
    d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}, grip=GRIPPER_CLOSED))
    for _ in range(30):    # pushing, but NO load: must not latch
        d._tick()
    assert d.grasp_goal is None, "must not latch without load"
    # MOVING jaw under high load = acceleration transient, NOT contact
    # (field-observed: fast slider drags cost ~17% load in free air and
    # false-latched). High load only counts when the jaw has stopped.
    d.bus.load[GRIPPER_JOINT] = -280
    for _ in range(30):
        d.bus.present[GRIPPER_JOINT] += d.close_dir * 10   # still moving
        d.last_present[GRIPPER_JOINT] = d.bus.present[GRIPPER_JOINT]
        d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}, grip=GRIPPER_CLOSED))
        d._tick()
    assert d.grasp_goal is None, "must not latch while the jaw still moves"
    g_open = d.bus.present[GRIPPER_JOINT]       # jaws now stalled HERE
    for _ in range(5):
        d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}, grip=GRIPPER_CLOSED))
        d._tick()
    assert d.grasp_goal is not None, "grasp not detected"
    expect = g_open + d.close_dir * d.grip_squeeze
    assert d.grasp_goal == expect, (d.grasp_goal, expect)
    latched = d.grasp_goal
    for _ in range(200):   # keep commanding fully closed: latch must hold
        d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}, grip=GRIPPER_CLOSED))
        d._tick()
    assert d.grasp_goal == latched
    assert abs(d.last_sent[GRIPPER_JOINT] - latched) <= 1, \
        "goal must stop at the latch"
    d._on_cmd(cmd({n: 0.0 for n in ARM_JOINTS}, grip=0.0))  # open
    d._tick()
    assert d.grasp_goal is None, "open command must release the grasp"
    print(f"9. grasp detect/hold/release OK (stalled jaw, signed load, "
          f"latched at contact{d.close_dir * d.grip_squeeze:+d})")

    # 10) freeze during a BUS OUTAGE must not raise (field crash: cameras
    #     renegotiating shared USB disrupted the serial adapter exactly when
    #     the deadman tried to freeze; the unguarded read killed the driver).
    d = make_driver(present)
    d._on_cmd(cmd({n0: 0.5}))
    d._tick()

    def dead_read(reg, motors=None, normalize=True, num_retry=0):
        raise RuntimeError("device disconnected")
    d.bus.sync_read = dead_read
    time.sleep(0.25)
    d._tick()                      # deadman fires -> _freeze on a dead bus
    assert d.frozen, "must still freeze"
    assert d.target == {n: present[n] for n in d.target}, \
        "must latch last known pose"
    for _ in range(5):
        d._tick()                  # loop must stay alive on the dead bus
    print("10. freeze survives a bus outage (latches last known pose) OK")

    # 11) several failed state reads take the transport offline. A command
    # stamped before recovery is rejected even if DDS dispatches it after the
    # reconnect; only a later timestamp may re-arm motion.
    d = make_driver(present)
    d._on_cmd(cmd({n0: 0.5}))
    d._tick()
    d.bus.fail_present_reads = 3
    for _ in range(3):
        d._tick()
    assert d.transport_offline, "three failed state reads must take transport offline"
    recovered = dict(present)
    recovered[n0] += 123
    d.bus.present = recovered
    d._tick()
    assert not d.transport_offline, "healthy reconnect must leave offline mode"
    assert d.frozen and d.target == recovered, \
        "reconnect must hold the newly measured pose"
    assert d.bus.goal_writes()[-1][2] == recovered, \
        "reconnect hold must write the just-read physical pose"
    assert d.last_cmd_time is None, "outage command must not arm motion"
    assert ("disconnect", False) in d.bus.calls and ("connect", True) in d.bus.calls
    queued = cmd({n0: -0.5})
    queued.header.stamp = d.rearm_after_stamp
    d._on_cmd(queued)
    assert d.frozen and d.target == recovered, \
        "a command queued before recovery must not re-arm motion"
    fresh = cmd({n0: 0.2})
    fresh.header.stamp.sec = d.rearm_after_stamp.sec
    fresh.header.stamp.nanosec = d.rearm_after_stamp.nanosec + 1
    d._on_cmd(fresh)
    assert not d.frozen and d.target[n0] == conv.arm_ticks(n0, 0.2), \
        "only a post-recovery command may re-arm motion"
    print("11. transport recovery re-seeds and requires a fresh command OK")

    # 12) a failed reconnect leaves the port closed. The next retry must call
    # connect directly rather than failing on disconnect-before-connect.
    d = make_driver(present)
    d._on_cmd(cmd({n0: 0.5}))
    d.bus.fail_present_reads = 3
    for _ in range(3):
        d._tick()
    d.bus.fail_connects = 1
    d._tick()
    assert d.transport_offline and not d.bus.connected
    d.next_reconnect_at = 0.0
    d._tick()
    assert not d.transport_offline and d.bus.connected, \
        "reconnect retry must work with an already-closed port"
    print("12. reconnect retries after a closed-port failure OK")

    print("\nALL DRIVER SAFETY TESTS PASSED")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
