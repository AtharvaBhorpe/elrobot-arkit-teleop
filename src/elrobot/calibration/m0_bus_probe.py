"""M0 — Feetech bus probe: sync_read/sync_write latency + desync check.

Gate (from the design spec): p50 < 5 ms -> proceed; ~20 ms -> fix USB transport
first. Station measured 20-21 ms per *individual* motor transaction; the whole
premise of using lerobot is that sync_* does all 8 in one.

SAFETY: torque is disabled immediately after connect and never re-enabled, so
no command here can move the arm. The optional --write test writes each motor's
*current* position back to itself, which is a no-op even if torque were on.

All reads use normalize=False: the arm is uncalibrated at M0, and normalization
requires calibration. Raw ticks are also what M1b needs for the offset table.

Run: pixi run python -m elrobot.calibration.m0_bus_probe [--port /dev/ttyACM0] [-n 200] [--write]
"""

import argparse
import statistics
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# 7 arm joints + gripper, per the URDF (rev_motor_01..08) -> servo IDs 1..8
MOTOR_IDS = range(1, 9)
MODEL = "sts3215"


def build_bus(port: str) -> FeetechMotorsBus:
    motors = {
        f"rev_motor_{i:02d}": Motor(i, MODEL, MotorNormMode.RANGE_M100_100)
        for i in MOTOR_IDS
    }
    # calibration=None: M0 runs before M1a, reads stay raw
    return FeetechMotorsBus(port=port, motors=motors, calibration=None)


def summarize(label: str, samples: list[float], failures: int, n: int) -> float:
    if not samples:
        print(f"{label}: ALL {n} FAILED")
        return float("inf")
    s = sorted(samples)
    p50 = statistics.median(s)
    p95 = s[min(int(0.95 * len(s)), len(s) - 1)]
    print(
        f"{label}: n={len(s)} fail={failures} "
        f"p50={p50*1e3:.2f}ms p95={p95*1e3:.2f}ms "
        f"min={s[0]*1e3:.2f}ms max={s[-1]*1e3:.2f}ms"
    )
    return p50


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("-n", type=int, default=200, help="samples per test")
    ap.add_argument("--write", action="store_true",
                    help="also time sync_write (no-op writes, torque off)")
    args = ap.parse_args()

    bus = build_bus(args.port)
    print(f"connecting to {args.port} ...")
    # handshake=True pings every motor: catches a missing/misnumbered servo up front
    bus.connect(handshake=True)
    print("connected")

    try:
        bus.disable_torque()
        print("torque DISABLED (arm cannot move during probe)\n")

        expected = set(bus.motors)
        read_samples, read_fail, desync = [], 0, 0
        for _ in range(args.n):
            t0 = time.perf_counter()
            try:
                pos = bus.sync_read("Present_Position", normalize=False)
                dt = time.perf_counter() - t0
                # desync check: station's defect returned another motor's reply.
                # A short/mismatched dict means the bus handed back the wrong data.
                if set(pos) != expected:
                    desync += 1
                else:
                    read_samples.append(dt)
            except Exception:  # noqa: BLE001 - a timeout is a data point, not a crash
                read_fail += 1

        p50 = summarize("sync_read(8 motors)", read_samples, read_fail, args.n)
        print(f"desync (wrong/missing motors in reply): {desync}")

        if args.write:
            # write current position back -> zero motion even if torque returns
            current = bus.sync_read("Present_Position", normalize=False)
            write_samples, write_fail = [], 0
            for _ in range(args.n):
                t0 = time.perf_counter()
                try:
                    bus.sync_write("Goal_Position", current, normalize=False)
                    write_samples.append(time.perf_counter() - t0)
                except Exception:  # noqa: BLE001
                    write_fail += 1
            summarize("sync_write(8 motors)", write_samples, write_fail, args.n)

        print()
        if p50 < 0.005:
            print(f"GATE PASS: p50 {p50*1e3:.2f}ms < 5ms -> proceed to M1")
            return 0
        print(f"GATE FAIL: p50 {p50*1e3:.2f}ms >= 5ms -> fix USB transport first")
        return 1
    finally:
        bus.disconnect()
        print("disconnected")


if __name__ == "__main__":
    raise SystemExit(main())
