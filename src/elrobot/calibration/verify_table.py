"""Verify the M1b URDF<->tick table against a physical measurement.

Reads the arm's current pose through the table, runs Pinocchio FK, and prints
where the gripper should be so you can measure it.

Also reports how much each joint's sign actually MATTERS at the pose you are
holding. Near the URDF neutral pose every joint decodes to q~0 for either sign,
so a wrong sign is invisible and the check passes vacuously. Hold a clearly
BENT pose - the sensitivity column tells you whether the pose has leverage.

    pixi run python -m elrobot.calibration.verify_table

Read-only: torque is disabled, nothing is ever commanded.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pinocchio as pin
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

TICKS_PER_RAD = 651.9
ARM = [f"rev_motor_{i:02d}" for i in range(1, 8)]
TCP_FRAME = "Gripper_Base_v1_1"
DISCRIMINATING = 5.0  # cm of TCP shift needed to call a sign "testable" here


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--table", default="calibration/urdf_ticks.json")
    args = ap.parse_args()

    model = pin.buildModelFromUrdf("docs/urdf_Elrobot.urdf")
    data = model.createData()
    jid = {model.names[j]: j for j in range(1, model.njoints)}
    fid = model.getFrameId(TCP_FRAME)
    table = json.loads(Path(args.table).read_text())
    qmid = {n: (model.lowerPositionLimit[model.joints[jid[n]].idx_q]
                + model.upperPositionLimit[model.joints[jid[n]].idx_q]) / 2
            for n in ARM}

    def decode(ticks, tab):
        q = pin.neutral(model)
        for n in ARM:
            q[model.joints[jid[n]].idx_q] = (
                (ticks[n] - tab[n]["offset"]) / (tab[n]["sign"] * TICKS_PER_RAD))
        return q

    def tcp(q):
        """Returns a COPY - oMf is a live buffer that later calls overwrite."""
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    print(__doc__)
    bus = FeetechMotorsBus(
        port=args.port,
        motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                for i, n in enumerate(ARM + ["rev_motor_08"], start=1)},
        calibration=None)
    bus.connect(handshake=True)
    try:
        bus.disable_torque()
        input("Hold the arm in a clearly BENT pose you can measure, then ENTER...")
        ticks = bus.sync_read("Present_Position", ARM, normalize=False)
        q = decode(ticks, table)
        p = tcp(q)

        print(f"\n{'JOINT':<14}{'TICKS':>7}{'q(deg)':>9}   LIMITS   SIGN TESTABLE HERE?")
        ok = True
        for n in ARM:
            qi = q[model.joints[jid[n]].idx_q]
            lo = model.lowerPositionLimit[model.joints[jid[n]].idx_q]
            hi = model.upperPositionLimit[model.joints[jid[n]].idx_q]
            inlim = lo - 0.05 <= qi <= hi + 0.05
            ok &= inlim
            # how far would the TCP move if this joint's sign were flipped?
            alt = {k: dict(v) for k, v in table.items() if k in ARM}
            s = -table[n]["sign"]
            tm = table[n]["offset"] + table[n]["sign"] * TICKS_PER_RAD * qmid[n]
            alt[n] = {"sign": s, "offset": tm - s * TICKS_PER_RAD * qmid[n]}
            shift = np.linalg.norm(tcp(decode(ticks, alt)) - p) * 100
            verdict = ("yes" if shift >= DISCRIMINATING
                       else "NO - bend more" if shift > 0.5 else "NEVER (roll joint)")
            print(f"{n:<14}{ticks[n]:>7}{math.degrees(qi):>9.1f}   "
                  f"{'ok' if inlim else 'OUT':<8} {shift:5.1f} cm  {verdict}")

        print(f"\nPredicted TCP: x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} m")
        print(f"  gripper height above base plane : {p[2]*100:6.1f} cm")
        print(f"  horizontal distance from base   : {math.hypot(p[0],p[1])*100:6.1f} cm")
        print("\nMeasure both. Agreement within a few cm => table is good.")
        if not ok:
            print("GATE FAIL: a joint decoded outside its URDF limits.")
            return 1
        return 0
    finally:
        bus.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
