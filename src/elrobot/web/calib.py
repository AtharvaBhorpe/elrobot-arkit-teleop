"""Web-wizard calibration session: the M1a -> M1b -> signs -> FK guided flow
as a small state machine driven by HTTP calls instead of terminal prompts.

States:

    idle -> preflight -> homed -> sweeping -> gate
         -> fullturn (x2, joints 05 and 07) -> signs -> done

**The EEPROM write comes BEFORE the range sweep, not after.** This mirrors
m1a_calibrate exactly, and the order is load-bearing:
set_half_turn_homings() redefines what Present_Position returns (the current
pose becomes tick 2047 for every motor), so any range recorded BEFORE that
write is expressed in a coordinate frame that the write then invalidates.
Deriving offsets from pre-homing ranges yields a table wrong by the homing
delta - i.e. an arm that moves wrong. An earlier version of this wizard had
sweep-then-write and would have produced exactly that.

`sweep_begin`/`sweep_end` serve two phases - the M1a full-arm range sweep
(from "homed") and the M1b encoder-unwrapped sweep of the two full-turn
joints (from "gate") - because which joints are swept, and how, depends on
the state rather than on a separate endpoint.

Preflight enforces single-owner serial access (hard rule 3): the wizard can
only open the bus while the driver is not running (checked by the caller via
bridge.driver_alive() BEFORE start() opens the port).
"""

import threading

from elrobot.calibration import backup, steps
from elrobot.calibration.m1a_calibrate import FULL_TURN_MOTORS, URDF_RANGE_RAD

ARM_JOINTS = [f"rev_motor_{i:02d}" for i in range(1, 8)]
ALL_JOINTS = ARM_JOINTS + ["rev_motor_08"]
SWEEP_JOINTS = [j for j in ALL_JOINTS if j not in FULL_TURN_MOTORS]


class CalibError(Exception):
    def __init__(self, detail, code=409):
        super().__init__(detail)
        self.detail, self.code = detail, code


class CalibSession:
    def __init__(self, bus_factory=None, port="/dev/ttyACM0",
                 backup_root=backup.DEFAULT_ROOT):
        self._bus_factory = bus_factory or (lambda: steps.build_bus(port))
        self.port = port
        self.backup_root = backup_root
        self.state = "idle"
        self.bus = None
        self.ranges = {}      # name -> (min, max) raw ticks
        self.gate = []        # steps.gate_ranges() output
        self.signs = {}       # name -> +1/-1, default +1 once touched
        self.table = None
        self.fk = None
        self.backup = None    # path of the pre-calibration snapshot
        self.error = None     # last sweep-thread failure, surfaced to the UI
        self._stop = None
        self._thread = None
        self._sweep_result = {}
        self._sweep_error = None

    def _require(self, *legal):
        if self.state not in legal:
            raise CalibError(f"cannot do this from state {self.state}")

    def start(self, driver_alive):
        # NB "driver_alive" really means "something is publishing
        # /joint_states" - that is also true of view/m2/jog, whose slider GUI
        # publishes it with no hardware attached. Fails safe (refuses when it
        # need not) and the real guarantee is the port open below, which
        # cannot succeed while another process owns the bus.
        if driver_alive:
            raise CalibError(
                "something is publishing /joint_states (driver, jog or view) "
                "- stop it before calibrating", code=409)
        self._require("idle", "done")
        try:
            self.bus = self._bus_factory()
            self.bus.connect(handshake=True)
            self.bus.disable_torque()
        except Exception as e:
            # _close_bus, not `self.bus = None`: if connect() succeeded and
            # disable_torque() then threw, the old code dropped a live,
            # connected bus on the floor with no reference left to close it -
            # the port stayed held, and the operator could hit Start again
            # and open a SECOND connection to the same UART.
            self._close_bus()
            # Most likely another process owns the port, or the arm is on a
            # different ttyACM than this session was told about.
            raise CalibError(
                f"cannot open {self.port}: {e}. Is another process using it?",
                code=409) from e
        # Snapshot BEFORE anything can write: this is the only moment the
        # pre-calibration EEPROM still exists. A backup the operator has to
        # remember to take is a backup that is not there on the day it is
        # needed, so it is taken automatically and the wizard REFUSES to
        # continue without one - no backup, no destructive write. (To
        # calibrate anyway, the CLI scripts remain the unguarded path.)
        try:
            self.backup = str(backup.snapshot(
                self.bus, self.port, note="web wizard preflight",
                root=self.backup_root))
        except Exception as e:
            self._close_bus()
            raise CalibError(
                f"could not back up the current calibration ({e}) - refusing "
                f"to start, since the EEPROM write cannot be undone", code=500
            ) from e
        self.error = None
        self.state = "preflight"
        return self.snapshot()

    def eeprom(self, confirm):
        # Comes BEFORE any sweep - see the module docstring. Park the arm
        # first; whatever pose it is in becomes tick 2047 on every motor.
        self._require("preflight")
        if confirm != "ERASE":
            raise CalibError("type ERASE to confirm the EEPROM write", code=400)
        steps.write_homing(self.bus)
        self.state = "homed"
        return self.snapshot()

    def sweep_begin(self):
        self._require("homed", "gate")
        self._stop = threading.Event()
        self._sweep_result = {}
        self._sweep_error = None
        self.error = None

        if self.state == "homed":
            joints, phase = SWEEP_JOINTS, "sweeping"

            def work():
                return steps.read_ranges(self.bus, joints, poll_s=0.05,
                                         stop=self._stop)
        else:
            # M1b phase A: full-turn joints need unwrapping, one at a time -
            # pick the first not already recorded, not always index 0
            remaining = [n for n in FULL_TURN_MOTORS if n not in self.ranges]
            if not remaining:
                raise CalibError("both full-turn joints already swept")
            joint = remaining[0]
            phase = "fullturn"

            def work():
                return {joint: steps.read_range_unwrapped(
                    self.bus, joint, poll_s=0.05, stop=self._stop)}

        def run():
            # A raw thread swallows exceptions silently, which previously let
            # a dead servo produce an EMPTY range set that still advanced the
            # state machine - and the destructive EEPROM write sat right
            # after it. Capture the failure and let sweep_end raise it.
            try:
                self._sweep_result = work()
            except Exception as e:                       # noqa: BLE001
                self._sweep_error = f"{type(e).__name__}: {e}"

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        self.state = phase
        return self.snapshot()

    def sweep_end(self):
        self._require("sweeping", "fullturn")
        was = self.state
        self._stop.set()
        self._thread.join(timeout=10)

        if self._thread.is_alive():
            self.error = "sweep thread did not stop; bus may be wedged"
            raise CalibError(self.error, code=500)
        if self._sweep_error:
            self.error = f"sweep failed: {self._sweep_error}"
            raise CalibError(self.error, code=500)

        if was == "sweeping":
            missing = [j for j in SWEEP_JOINTS if j not in self._sweep_result]
            if missing:
                self.error = f"no data from: {', '.join(missing)}"
                raise CalibError(self.error, code=500)
            self.ranges.update(self._sweep_result)
            gate_in = {n: self.ranges[n] for n in self.ranges
                       if n in URDF_RANGE_RAD}
            self.gate = steps.gate_ranges(gate_in, URDF_RANGE_RAD)
            self.state = "gate"
        else:
            self.ranges.update(self._sweep_result)
            self.state = ("signs"
                          if all(n in self.ranges for n in FULL_TURN_MOTORS)
                          else "gate")
        return self.snapshot()

    def sign(self, joint, flip):
        self._require("signs")
        if joint not in ARM_JOINTS:
            raise CalibError(f"{joint} is not an arm joint", code=400)
        cur = self.signs.get(joint, 1)
        self.signs[joint] = -cur if flip else cur
        return self.snapshot()

    def finish(self, out="calibration/urdf_ticks.json"):
        self._require("signs")
        # Refuse to write a partial table: this file is hand-measured
        # physical truth (hard rule 1) and a missing joint would silently
        # become a wrong offset.
        missing = [j for j in ALL_JOINTS if j not in self.ranges]
        if missing:
            raise CalibError(
                f"refusing to write a partial table; no ranges for: "
                f"{', '.join(missing)}", code=409)
        for n in ARM_JOINTS:
            self.signs.setdefault(n, 1)
        limits = steps.read_urdf_limits(ARM_JOINTS)
        gripper = {"closed_ticks": self.ranges["rev_motor_08"][0],
                   "open_ticks": self.ranges["rev_motor_08"][1]}
        arm_ranges = {n: self.ranges[n] for n in ARM_JOINTS}
        table = steps.derive_table(arm_ranges, self.signs, gripper, limits)

        # EVERY fallible step happens BEFORE the file is written. save_table
        # overwrites calibration/urdf_ticks.json - hand-measured physical
        # truth - so a bus read that throws after the write would leave the
        # file replaced while the UI reported a failure and re-enabled
        # Finish, i.e. the operator believing nothing happened.
        try:
            current = self.bus.sync_read("Present_Position", ARM_JOINTS,
                                         normalize=False)
            fk = steps.fk_report(table, current)
        except Exception as e:
            self.error = (f"final pose read failed, table NOT written: {e}")
            raise CalibError(self.error, code=500) from e

        steps.save_table(table, out)      # last, and cannot fail after this
        self.table, self.fk = table, fk
        self._close_bus()
        self.state = "done"
        return self.snapshot()

    def _close_bus(self):
        if self.bus is not None:
            try:
                self.bus.disconnect()
            except Exception:                            # noqa: BLE001
                pass          # already gone; nothing useful to do about it
            self.bus = None

    def abort(self):
        """Release the serial port and reset. Without this the port stayed
        open until the whole web backend was restarted, since the only
        disconnect was on a fully successful finish() - which blocks the
        driver's single-owner access (hard rule 3) after any mid-session
        change of mind or error."""
        if self._stop is not None:
            self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5)
        self._close_bus()
        self.state = "idle"
        self.ranges, self.gate, self.signs = {}, [], {}
        self.table = self.fk = None
        self._sweep_result, self._sweep_error = {}, None
        return self.snapshot()

    def snapshot(self):
        return {
            "state": self.state,
            "ranges": {k: list(v) for k, v in self.ranges.items()},
            "gate": self.gate,
            "signs": dict(self.signs),
            "table": self.table,
            "fk": self.fk,
            "error": self.error,
            "backup": self.backup,
        }
