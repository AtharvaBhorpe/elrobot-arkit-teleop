"""Prove RoboStack ROS 2 Jazzy + lerobot + Pinocchio coexist in one env.

The spec marks this env as "unproven" and gates M0/M2 on it. This is that gate:
every stack must import in one process, and we surface the Feetech bus API so the
"does connect() require calibration to read?" question can be answered offline.

Run: pixi run prove-env
"""

import sys


def _ok(label, thunk):
    try:
        print(f"  {label:<24} {thunk()}")
        return True
    except Exception as e:  # noqa: BLE001 - we want every failure, not the first
        print(f"  {label:<24} FAIL: {type(e).__name__}: {e}")
        return False


def main() -> int:
    print(f"python {sys.version.split()[0]}")

    print("imports:")
    results = [
        _ok("rclpy (ROS 2)", lambda: __import__("rclpy").__name__),
        _ok("numpy", lambda: __import__("numpy").__version__),
        _ok("pinocchio", lambda: __import__("pinocchio").__version__),
        _ok("torch", lambda: __import__("torch").__version__),
        _ok("lerobot", lambda: __import__("importlib.metadata", fromlist=["version"]).version("lerobot")),
    ]

    # Surface the Feetech bus API without touching hardware: does connect()
    # take an arg to skip calibration? (spec risk: M0 vs M1 ordering)
    print("feetech bus API:")
    try:
        import inspect

        from lerobot.motors.feetech import FeetechMotorsBus

        sig = inspect.signature(FeetechMotorsBus.connect)
        print(f"  connect{sig}")
        methods = [m for m in ("sync_read", "sync_write") if hasattr(FeetechMotorsBus, m)]
        print(f"  has: {', '.join(methods) or 'NONE — wrong class?'}")
        results.append(bool(methods))
    except Exception as e:  # noqa: BLE001
        # import path shifts between lerobot versions; report, don't crash
        print(f"  FeetechMotorsBus lookup FAIL: {type(e).__name__}: {e}")
        results.append(False)

    ok = all(results)
    print("\nENV PROVEN" if ok else "\nENV NOT PROVEN — see FAILs above")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
