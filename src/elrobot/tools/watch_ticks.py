"""Live raw ticks + decoded URDF angles for all 8 motors, 5 Hz.

The hand tool for sign checks and M3 debugging: rotate a joint by hand and
watch its tick count and decoded q move in real time.

    pixi run python scripts/watch_ticks.py

Torque is disabled on start (so joints can be moved by hand) and left
disabled. Read-only otherwise. Ctrl-C to quit.
"""

import argparse
import json
import math
import time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

TICKS_PER_RAD = 651.9
ARM = [f"rev_motor_{i:02d}" for i in range(1, 8)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--table", default="calibration/urdf_ticks.json")
    args = ap.parse_args()

    table = json.loads(Path(args.table).read_text())
    bus = FeetechMotorsBus(
        port=args.port,
        motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                for i, n in enumerate(ARM + ["rev_motor_08"], start=1)},
        calibration=None)
    bus.connect(handshake=True)
    bus.disable_torque()
    print("torque DISABLED - joints move by hand. Ctrl-C to quit.\n")
    # Load = signed motor effort, 0.1%/LSB (Present_Current is ~0 on this
    # firmware even in motion - measured; load is the usable signal)
    print(f"{'JOINT':<14}{'TICKS':>7}{'q(rad)':>9}{'q(deg)':>9}{'load%':>7}")
    try:
        first = True
        while True:
            names = ARM + ["rev_motor_08"]
            ticks = bus.sync_read("Present_Position", names, normalize=False)
            load = bus.sync_read("Present_Load", names, normalize=False)
            ma = {n: v / 10 for n, v in load.items()}  # signed percent
            if not first:
                print(f"\x1b[{len(ticks)}A", end="")  # cursor up, overwrite
            first = False
            for n in ARM:
                q = (ticks[n] - table[n]["offset"]) / (
                    table[n]["sign"] * TICKS_PER_RAD)
                print(f"{n:<14}{ticks[n]:>7}{q:>9.3f}{math.degrees(q):>9.1f}"
                      f"{ma[n]:>7.0f}")
            g = table["rev_motor_08"]
            span = g["open_ticks"] - g["closed_ticks"]
            frac = (ticks["rev_motor_08"] - g["closed_ticks"]) / span
            print(f"{'rev_motor_08':<14}{ticks['rev_motor_08']:>7}"
                  f"{'':>9}{frac:>8.0%} open{ma['rev_motor_08']:>6.0f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect()
        print("\ndisconnected (torque still off)")


if __name__ == "__main__":
    main()
