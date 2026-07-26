"""Snapshot / restore of everything a recalibration destroys.

Two artifacts encode this arm's calibration, and only ONE of them is
recoverable on its own:

  * `calibration/*.json` - hand-measured, and git-tracked, so `git checkout`
    already brings them back.
  * **servo EEPROM** - `Homing_Offset` + `Min_Position_Limit` +
    `Max_Position_Limit`, per motor. `set_half_turn_homings()` (via
    `reset_calibration()`) overwrites exactly these three registers and
    nothing tracks them. Once written, the old values are gone.

They are also not independent. `urdf_ticks.json`'s offsets are expressed in
the tick frame that the homing offsets define, so restoring the file WITHOUT
the EEPROM - or the EEPROM without the file - yields a table describing
servos that no longer match it: an arm that moves wrong while every file on
disk looks right. That is worse than no backup, because it looks fine.

So a snapshot is ONE json holding both halves, and restore puts back both.

Note `read_calibration()` only reads `Homing_Offset` when
`protocol_version == 0` (it returns 0 otherwise). `steps.build_bus()` uses
the default, which is 0 - `capture()` asserts it rather than trusting it,
because a silently-zeroed backup is the one failure mode that would not
surface until the day it is needed.

    pixi run calib-backup                 # snapshot now
    pixi run calib-backup --list
    pixi run calib-restore                # restore the newest snapshot
    pixi run calib-restore --file <path>

The driver must be stopped for either (hard rule 3) - the serial open is
what enforces it.
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

from lerobot.motors.motors_bus import MotorCalibration

DEFAULT_ROOT = Path("calibration/backups")
# The files that must travel WITH the EEPROM values to stay coherent.
TRACKED_FILES = ("calibration/urdf_ticks.json", "calibration/elrobot.json")


def capture(bus, port="", note="") -> dict:
    """Read the three destructible registers off every motor, plus the
    current calibration files, into one self-contained dict."""
    if getattr(bus, "protocol_version", 0) != 0:
        raise RuntimeError(
            "bus protocol_version != 0: read_calibration() would report "
            "homing_offset=0 for every motor and the backup would be a lie")
    motors = {n: vars(c) for n, c in bus.read_calibration().items()}
    if not motors:
        raise RuntimeError("read_calibration() returned nothing")
    files = {p: Path(p).read_text() for p in TRACKED_FILES if Path(p).exists()}
    return {"created": datetime.now().isoformat(timespec="seconds"),
            "port": port, "note": note, "motors": motors, "files": files}


def save(snap: dict, root=DEFAULT_ROOT) -> Path:
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    stamp = snap["created"].replace(":", "").replace("-", "")
    path = root / f"calib-{stamp}.json"
    # Never clobber an existing snapshot: two runs in the same second would
    # otherwise silently leave one backup where the operator expects two.
    n = 1
    while path.exists():
        n += 1
        path = root / f"calib-{stamp}-{n}.json"
    path.write_text(json.dumps(snap, indent=2))
    return path


def snapshot(bus, port="", note="", root=DEFAULT_ROOT) -> Path:
    return save(capture(bus, port, note), root)


def latest(root=DEFAULT_ROOT) -> Path | None:
    snaps = sorted(Path(root).glob("calib-*.json"))
    return snaps[-1] if snaps else None


def load(path) -> dict:
    snap = json.loads(Path(path).read_text())
    if "motors" not in snap or not snap["motors"]:
        raise RuntimeError(f"{path} has no motor data - not a usable snapshot")
    return snap


def restore(bus, snap: dict, files=True) -> dict:
    """Write the snapshot's EEPROM values back, then the files.

    EEPROM first: it is the half that can fail (a motor not answering), and
    a file restored against un-restored servos is the incoherent state this
    module exists to prevent. If the bus write throws, the files on disk are
    still the ones that match the servos' CURRENT state.
    """
    calib = {n: MotorCalibration(**c) for n, c in snap["motors"].items()}
    bus.write_calibration(calib)
    written = []
    if files:
        for p, text in snap.get("files", {}).items():
            Path(p).parent.mkdir(parents=True, exist_ok=True)
            Path(p).write_text(text)
            written.append(p)
    return {"motors": sorted(calib), "files": written}


def _cli():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--root", default=str(DEFAULT_ROOT))
    ap.add_argument("--restore", action="store_true")
    ap.add_argument("--file", help="snapshot to restore (default: newest)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-files", action="store_true",
                    help="restore EEPROM only, leave calibration/*.json alone")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation")
    a = ap.parse_args()

    if a.list:
        for p in sorted(Path(a.root).glob("calib-*.json")):
            s = json.loads(p.read_text())
            print(f"{p}  {s.get('created','?')}  {s.get('note','')}")
        return
    if a.file and not a.restore:
        # Silently ignoring --file here would take a NEW snapshot while the
        # operator believed they were restoring the named one.
        raise SystemExit("--file only applies to --restore (try --list)")

    from elrobot.calibration import steps
    bus = steps.build_bus(a.port)
    bus.connect(handshake=True)
    try:
        if not a.restore:
            p = snapshot(bus, a.port, note="manual", root=a.root)
            print(f"saved {p}")
            return

        path = Path(a.file) if a.file else latest(a.root)
        if path is None:
            raise SystemExit(f"no snapshots in {a.root}")
        snap = load(path)
        print(f"restore {path} (taken {snap['created']}, port {snap['port']})")
        print(f"  {len(snap['motors'])} motors -> Homing_Offset, "
              f"Min/Max_Position_Limit")
        for p in (snap.get("files") or {}) if not a.no_files else []:
            print(f"  overwrite {p}")
        if not a.yes and input("proceed? type yes: ").strip() != "yes":
            raise SystemExit("aborted")
        done = restore(bus, snap, files=not a.no_files)
        print(f"restored {len(done['motors'])} motors, "
              f"{len(done['files'])} files")
        print("Torque was never enabled; power-cycle not required.")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    _cli()
