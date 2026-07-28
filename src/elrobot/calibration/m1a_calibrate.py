"""M1a — LeRobot motor calibration for the Elrobot arm.

Follows lerobot's SOFollower.calibrate() flow, adapted to 8 Feetech STS3215:
torque off -> set_half_turn_homings() -> record_ranges_of_motion() -> write to
servo EEPROM + a JSON alongside.

INTERACTIVE: you move the arm by hand. Must be run from a real terminal
(record_ranges_of_motion streams a live table and waits on ENTER).

    pixi run python -m elrobot.calibration.m1a_calibrate

WARNING - this writes homing offsets to servo EEPROM, which changes what
Present_Position returns. The M1b offset/sign table is derived from those raw
ticks, so re-running M1a INVALIDATES M1b. Redo M1b after any recalibration.

SAFETY: torque is disabled throughout; the arm is limp and will sag under
gravity. Support it or rest it low before starting.
"""

import argparse
import json
import time
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode

from elrobot.calibration import steps
from elrobot.calibration.steps import gate_ranges, write_homing

TICKS_PER_RAD = 651.9  # STS3215 direct drive, 4096 ticks/rev

# Joints 5 and 7 sweep 336 deg / 340 deg — near a full revolution, so a
# hand-swept min/max can straddle the encoder wrap and record garbage. lerobot
# handles this by excluding such joints and assigning the full 0..4095 range
# (see SOFollower: wrist_roll). Safe here because the control path does NOT use
# lerobot normalization — the driver works in raw ticks (M1b) and clamps joint
# limits from the URDF. Only normalization sees these bounds.
FULL_TURN_MOTORS = ["rev_motor_05", "rev_motor_07"]

# URDF joint ranges (rad) -> expected tick span, used for the "ranges sane" gate.
# Gripper (08) has no URDF correspondence; excluded from the check.
URDF_RANGE_RAD = {
    "rev_motor_01": 3.1018,
    "rev_motor_02": 3.2244,
    "rev_motor_03": 3.5220,
    "rev_motor_04": 3.5066,
    "rev_motor_05": 5.8720,
    "rev_motor_06": 3.1416,
    "rev_motor_07": 5.9350,
}
SANE_TOLERANCE = 0.20  # recorded span within +/-20% of URDF expectation


def check_ranges(mins: dict, maxes: dict) -> bool:
    """M1a gate: 'all 8 motors calibrated, ranges sane'.

    Catches the common failure — a joint not swept far enough, which yields a
    plausible-looking calibration that quietly breaks IK later. The actual
    +/-20% span-vs-URDF math is steps.gate_ranges; this wraps it with the
    full-turn/gripper special cases and the CLI's live table printing.
    """
    print("\n=== range check ===")
    print(f"{'MOTOR':<14} {'SPAN':>6} {'EXPECTED':>9} {'DIFF':>7}  STATUS")
    ok = True
    urdf_motors = {m: mins[m] for m in mins
                   if m not in FULL_TURN_MOTORS and m in URDF_RANGE_RAD}
    gated = {g["name"]: g for g in gate_ranges(
        {m: (mins[m], maxes[m]) for m in urdf_motors}, URDF_RANGE_RAD)}
    for motor in sorted(mins):
        span = maxes[motor] - mins[motor]
        if motor in FULL_TURN_MOTORS:
            print(f"{motor:<14} {span:>6} {'full-turn':>9} {'-':>7}  assigned 0..4095")
            continue
        if motor not in URDF_RANGE_RAD:  # gripper
            print(f"{motor:<14} {span:>6} {'n/a':>9} {'-':>7}  gripper, no URDF ref")
            if span < 100:
                print("    WARNING: gripper span very small — did the jaws move?")
                ok = False
            continue
        expected = URDF_RANGE_RAD[motor] * TICKS_PER_RAD
        g = gated[motor]
        ok = ok and g["ok"]
        print(f"{motor:<14} {span:>6} {expected:>9.0f} {g['span_pct']:>+6.0%}  "
              f"{'ok' if g['ok'] else 'SUSPECT — sweep further?'}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--out", default="calibration/elrobot.json")
    args = ap.parse_args()

    print(__doc__)
    input("Arm resting low / supported? Torque will be disabled. ENTER to start...")

    bus = steps.build_bus(args.port)
    bus.connect(handshake=True)
    print(f"connected to {args.port}")

    try:
        bus.disable_torque()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        print("torque DISABLED — arm is limp\n")

        # Whatever position this is becomes tick 2047 for every joint. Precision
        # is not needed: the swept joints tolerate +/-79 deg of centring error
        # before the sweep could reach the 0/4095 encoder wrap. Joints 5 and 7
        # tolerate only ~10 deg — which is why they are excluded as full-turn.
        print("Park the arm in a relaxed posture with NO joint near a hard stop")
        print("(~20 deg of clearance at each end is plenty — do not measure).")
        input("ENTER when parked...")
        homing_offsets = write_homing(bus)   # THE EEPROM write
        print("homing offsets written to EEPROM\n")

        sweep = [m for m in bus.motors if m not in FULL_TURN_MOTORS]
        print(f"Now sweep these joints through their FULL range: {sweep}")
        print(f"(skip {FULL_TURN_MOTORS} — assigned full turn automatically)")
        print("Recording... press ENTER when every joint above has been swept.\n")
        time.sleep(1)
        range_mins, range_maxes = bus.record_ranges_of_motion(sweep)

        for motor in FULL_TURN_MOTORS:
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        sane = check_ranges(range_mins, range_maxes)

        calibration = {
            motor: MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
            for motor, m in bus.motors.items()
        }
        bus.write_calibration(calibration)

        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(
            {m: vars(c) for m, c in calibration.items()}, indent=2))
        print(f"\ncalibration written to servo EEPROM and {out}")

        if sane:
            print("GATE PASS: all 8 motors calibrated, ranges sane -> proceed to M1b")
            return 0
        print("GATE FAIL: some ranges look wrong (see SUSPECT above). Re-run.")
        return 1
    finally:
        bus.disconnect()
        print("disconnected")


if __name__ == "__main__":
    raise SystemExit(main())
