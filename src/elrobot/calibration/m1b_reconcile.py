"""M1b — URDF <-> tick reconciliation.

Derives the per-joint table the driver needs:

    ticks_i = offset_i + sign_i * 651.9 * q_urdf_i

Scale is fixed (STS3215 direct drive, 4096 ticks/rev), so only offset and sign
are unknown. Offsets come from the midpoint of each joint's tick range paired
with the midpoint of its URDF range - symmetric sweep error cancels.

Joints 5 and 7 were excluded from the M1a sweep as full-turn, so they have no
recorded range; phase A captures it here (with encoder-wrap unwrapping).

INTERACTIVE, torque off, arm moved by hand. Run from a real terminal:

    pixi run python -m elrobot.calibration.m1b_reconcile

SAFETY: torque disabled throughout - the arm is limp and will sag. Nothing here
commands motion. The verification gate at the end is measure-only.
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from elrobot.calibration.steps import derive_table
from elrobot.calibration.steps import unwrap as _unwrap  # re-exported below

MODEL = "sts3215"
TICKS_PER_RAD = 651.9
ENC = 4096
ARM = [f"rev_motor_{i:02d}" for i in range(1, 8)]
NO_RANGE = ["rev_motor_05", "rev_motor_07"]  # excluded from M1a sweep
# Observe the gripper base (the TCP). NOT the jaw frames: rev_motor_08_1/_2 have
# URDF origins ~38 cm off (they kept CAD world coords - extraction artifact), so
# jaw geometry is unusable. Leaf joints, so the arm chain and IK are unaffected.
TCP_FRAME = "Gripper_Base_v1_1"
PROBE = 0.3  # rad, for deriving the "which way does it move" description
TRANSLATE_MIN = 0.005  # m; below this the joint is a roll -> describe the spin


def build_bus(port):
    motors = {n: Motor(i, MODEL, MotorNormMode.RANGE_M100_100)
              for i, n in enumerate(ARM + ["rev_motor_08"], start=1)}
    return FeetechMotorsBus(port=port, motors=motors, calibration=None)


def read(bus, name):
    return bus.sync_read("Present_Position", [name], normalize=False)[name]


unwrap = _unwrap  # steps.unwrap - identical logic, extracted for the web wizard


def joint_axes(path="docs/urdf_Elrobot.urdf"):
    import xml.etree.ElementTree as ET
    out = {}
    for j in ET.parse(path).getroot().findall("joint"):
        ax = j.find("axis")
        if ax is not None:
            out[j.get("name")] = np.array([float(x) for x in ax.get("xyz").split()])
    return out


def describe(model, data, jid, axes):
    """Plain-English motion of the GRIPPER for +q on this joint.

    Joints 5 and 7 are rolls whose axes run through the TCP, so they translate
    it by ~0 - those fall back to a spin description. The off-axis jaw frames
    would translate, but their URDF origins are ~38 cm wrong, so they are not
    usable as an observable.
    """
    fid = model.getFrameId(TCP_FRAME)
    q0 = pin.neutral(model)

    def at(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    p0 = at(q0)
    q = q0.copy()
    q[model.joints[jid].idx_q] += PROBE
    dv = at(q) - p0
    mag = float(np.linalg.norm(dv))

    if mag < TRANSLATE_MIN:
        # roll joint: the gripper spins in place. Right-hand rule about the
        # world axis; an axis pointing back at a viewer standing at the base
        # (looking outward along the arm) makes +q read counter-clockwise.
        pin.forwardKinematics(model, data, q0)
        axis_w = data.oMi[jid].rotation @ axes[model.names[jid]]
        outward = p0 / np.linalg.norm(p0)
        sense = "COUNTER-CLOCKWISE" if float(axis_w @ outward) < 0 else "CLOCKWISE"
        return (f"SPIN {sense} (stand at the BASE, look out along the arm)", mag)

    if abs(dv[2]) >= max(abs(dv[0]), abs(dv[1])):
        return ("UP" if dv[2] > 0 else "DOWN"), mag
    # horizontal: express as rotation sense about vertical, seen from above
    cross_z = p0[0] * dv[1] - p0[1] * dv[0]
    sense = "COUNTER-CLOCKWISE" if cross_z > 0 else "CLOCKWISE"
    return (f"swing {sense} seen from ABOVE", mag)


def sweep_range(bus, name):
    """Phase A: drive to both hard stops, unwrapping the encoder."""
    print(f"\n--- {name}: move slowly to ONE hard stop, then the OTHER ---")
    input("    ENTER to start recording...")
    prev = read(bus, name)
    start_raw, acc = prev, 0
    lo = hi = 0
    print("    recording... press ENTER when both stops have been reached")
    import select
    import sys
    while True:
        raw = read(bus, name)
        acc = unwrap(prev, raw, acc)
        prev = raw
        lo, hi = min(lo, acc), max(hi, acc)
        print(f"\r    travel {acc:+6d}  min {lo:+6d}  max {hi:+6d}  "
              f"span {hi-lo:5d} tk ({(hi-lo)/TICKS_PER_RAD*180/math.pi:6.1f} deg)",
              end="", flush=True)
        if select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.readline()
            break
        time.sleep(0.02)
    print()
    return start_raw + lo, start_raw + hi


def ask_sign(bus, name, direction, mag_mm):
    """Phase B: user moves the joint the described way; tick delta gives sign."""
    print(f"\n--- {name}: sign ---")
    if direction.startswith("SPIN"):
        print(f"    Move this joint so the GRIPPER does: {direction}")
        print("    (this joint spins the gripper in place - it does not move it)")
    else:
        print(f"    Move this joint so the GRIPPER travels {direction}.")
        print(f"    (a 0.3 rad move shifts it ~{mag_mm:.0f} mm)")
    while True:
        input("    ENTER when the joint is parked, ready to move...")
        a = read(bus, name)
        input(f"    now MOVE it so the jaw goes {direction}, hold, then ENTER...")
        b = read(bus, name)
        d = b - a
        if d > ENC // 2:
            d -= ENC
        elif d < -ENC // 2:
            d += ENC
        if abs(d) < 40:
            print(f"    only {d:+d} ticks - too small to be sure. Move further.")
            continue
        sign = 1 if d > 0 else -1
        print(f"    ticks {a} -> {b} ({d:+d}) => sign = {sign:+d}")
        return sign


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--calib", default="calibration/elrobot.json")
    ap.add_argument("--out", default="calibration/urdf_ticks.json")
    args = ap.parse_args()

    model = pin.buildModelFromUrdf("docs/urdf_Elrobot.urdf")
    data = model.createData()
    jid = {model.names[j]: j for j in range(1, model.njoints)}
    limits = {n: (model.lowerPositionLimit[model.joints[jid[n]].idx_q],
                  model.upperPositionLimit[model.joints[jid[n]].idx_q])
              for n in ARM}
    calib = json.loads(Path(args.calib).read_text())

    print(__doc__)
    input("Arm resting low / supported? Torque will be disabled. ENTER...")

    bus = build_bus(args.port)
    bus.connect(handshake=True)
    try:
        bus.disable_torque()
        print("torque DISABLED - arm is limp\n")

        # Phase A - ranges for the two joints M1a skipped
        ranges = {}
        for n in ARM:
            if n in NO_RANGE:
                ranges[n] = sweep_range(bus, n)
            else:
                ranges[n] = (calib[n]["range_min"], calib[n]["range_max"])

        # Phase B - signs
        axes = joint_axes()
        signs = {}
        for n in ARM:
            direction, mag = describe(model, data, jid[n], axes)
            signs[n] = ask_sign(bus, n, direction, mag * 1000)

        # Derive offsets (steps.derive_table has the actual math; this loop
        # is only the CLI's live table printing)
        norm_ranges = {n: (min(ranges[n]), max(ranges[n])) for n in ARM}
        table = derive_table(
            norm_ranges, signs,
            gripper={"closed_ticks": calib["rev_motor_08"]["range_min"],
                    "open_ticks": calib["rev_motor_08"]["range_max"]},
            limits=limits)
        print(f"\n{'JOINT':<14}{'SPAN':>6}{'EXPECT':>8}{'SIGN':>6}{'OFFSET':>10}")
        for n in ARM:
            lo_t, hi_t = norm_ranges[n]
            q_lo, q_hi = limits[n]
            span, expect = hi_t - lo_t, (q_hi - q_lo) * TICKS_PER_RAD
            print(f"{n:<14}{span:>6}{expect:>8.0f}{signs[n]:>+6d}"
                  f"{table[n]['offset']:>10.1f}")

        # Phase C - verification gate: FK against a physical measurement
        print("\n=== verification gate ===")
        print("Hold the arm still in any pose you can measure.")
        input("ENTER to read the current pose...")
        ticks = bus.sync_read("Present_Position", ARM, normalize=False)
        q = pin.neutral(model)
        for n in ARM:
            q[model.joints[jid[n]].idx_q] = (
                (ticks[n] - table[n]["offset"]) / (table[n]["sign"] * TICKS_PER_RAD))
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        tcp = data.oMf[model.getFrameId("Gripper_Base_v1_1")].translation

        print(f"\n{'JOINT':<14}{'TICKS':>7}{'q (rad)':>10}{'q (deg)':>9}   LIMITS OK?")
        sane = True
        for n in ARM:
            qi = q[model.joints[jid[n]].idx_q]
            lo, hi = limits[n]
            ok = lo - 0.05 <= qi <= hi + 0.05
            sane &= ok
            print(f"{n:<14}{ticks[n]:>7}{qi:>10.4f}{math.degrees(qi):>9.1f}   "
                  f"{'ok' if ok else 'OUT OF RANGE - sign wrong?'}")
        print(f"\nPredicted TCP (base frame): x={tcp[0]:+.3f} y={tcp[1]:+.3f} "
              f"z={tcp[2]:+.3f} m")
        print(f"  -> gripper should be ~{tcp[2]*100:.1f} cm above the base plane")
        print("  Measure it. Agreement within a few cm = table is good.")
        print("  A sign error shows up as a large, obvious mismatch.")

        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(table, indent=2))
        print(f"\ntable written to {args.out}")
        if not sane:
            print("GATE FAIL: a joint decoded outside its URDF limits.")
            return 1
        print("GATE PASS (pending your tape-measure check above)")
        return 0
    finally:
        bus.disconnect()
        print("disconnected")


if __name__ == "__main__":
    raise SystemExit(main())
