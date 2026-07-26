"""Web-wizard calibration session: the M1a -> M1b -> signs -> FK guided flow
as a small state machine driven by HTTP calls instead of terminal prompts.

States: idle -> preflight -> sweeping -> gate -> eeprom_done -> fullturn ->
signs -> done. `sweep_begin`/`sweep_end` are reused across two phases (the
M1a-style full-arm range sweep from "preflight", and the M1b-style
encoder-unwrapped sweep of the two full-turn joints from "eeprom_done") -
which joints get swept and how depends on the state, not a new endpoint.

Preflight enforces single-owner serial access (hard rule 3): the wizard can
only open the bus while the driver is not running (checked by the caller
via bridge.driver_alive() BEFORE start() opens the port).
"""

import threading

from elrobot.calibration import steps
from elrobot.calibration.m1a_calibrate import FULL_TURN_MOTORS, URDF_RANGE_RAD

ARM_JOINTS = [f"rev_motor_{i:02d}" for i in range(1, 8)]
ALL_JOINTS = ARM_JOINTS + ["rev_motor_08"]
SWEEP_JOINTS = [j for j in ALL_JOINTS if j not in FULL_TURN_MOTORS]


class CalibError(Exception):
    def __init__(self, detail, code=409):
        super().__init__(detail)
        self.detail, self.code = detail, code


class CalibSession:
    def __init__(self, bus_factory=None, port="/dev/ttyACM0"):
        self._bus_factory = bus_factory or (lambda: steps.build_bus(port))
        self.state = "idle"
        self.bus = None
        self.ranges = {}      # name -> (min, max) raw ticks
        self.gate = []        # steps.gate_ranges() output
        self.signs = {}       # name -> +1/-1, default +1 once touched
        self.table = None
        self.fk = None
        self._stop = None
        self._thread = None
        self._sweep_result = {}

    def _require(self, *legal):
        if self.state not in legal:
            raise CalibError(f"cannot do this from state {self.state}")

    def start(self, driver_alive):
        if driver_alive:
            raise CalibError("driver alive - stop it before calibrating", code=409)
        self._require("idle", "done")
        self.bus = self._bus_factory()
        self.bus.connect(handshake=True)
        self.bus.disable_torque()
        self.state = "preflight"
        return self.snapshot()

    def sweep_begin(self):
        self._require("preflight", "eeprom_done")
        self._stop = threading.Event()
        self._sweep_result = {}
        if self.state == "preflight":
            joints, phase = SWEEP_JOINTS, "sweeping"

            def run():
                self._sweep_result.update(
                    steps.read_ranges(self.bus, joints, poll_s=0.05, stop=self._stop))
        else:
            # M1b phase A: full-turn joints need unwrap, one at a time -
            # pick the first one NOT already recorded, not always index 0
            joint = next(n for n in FULL_TURN_MOTORS if n not in self.ranges)

            def run():
                self._sweep_result[joint] = steps.read_range_unwrapped(
                    self.bus, joint, poll_s=0.05, stop=self._stop)
            phase = "fullturn"
        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        self.state = phase
        return self.snapshot()

    def sweep_end(self):
        self._require("sweeping", "fullturn")
        was = self.state
        self._stop.set()
        self._thread.join(timeout=5)
        self.ranges.update(self._sweep_result)
        if was == "sweeping":
            gate_in = {n: self.ranges[n] for n in self.ranges if n in URDF_RANGE_RAD}
            self.gate = steps.gate_ranges(gate_in, URDF_RANGE_RAD)
            self.state = "gate"
        else:
            # one full-turn joint down; if both 05 and 07 are now recorded,
            # the M1b sweep phase is complete
            self.state = ("signs" if all(n in self.ranges for n in FULL_TURN_MOTORS)
                         else "eeprom_done")
        return self.snapshot()

    def eeprom(self, confirm):
        self._require("gate")
        if confirm != "ERASE":
            raise CalibError("type ERASE to confirm the EEPROM write", code=400)
        steps.write_homing(self.bus)
        self.state = "eeprom_done"
        return self.snapshot()

    def sign(self, joint, flip):
        self._require("eeprom_done", "signs")
        self.state = "signs"
        cur = self.signs.get(joint, 1)
        self.signs[joint] = -cur if flip else cur
        return self.snapshot()

    def finish(self, out="calibration/urdf_ticks.json"):
        self._require("signs")
        for n in ARM_JOINTS:
            self.signs.setdefault(n, 1)
        limits = steps.read_urdf_limits(ARM_JOINTS)
        gripper = {"closed_ticks": self.ranges["rev_motor_08"][0],
                  "open_ticks": self.ranges["rev_motor_08"][1]}
        arm_ranges = {n: self.ranges[n] for n in ARM_JOINTS}
        self.table = steps.derive_table(arm_ranges, self.signs, gripper, limits)
        steps.save_table(self.table, out)
        current = self.bus.sync_read("Present_Position", ARM_JOINTS, normalize=False)
        self.fk = steps.fk_report(self.table, current)
        self.bus.disconnect()
        self.state = "done"
        return self.snapshot()

    def snapshot(self):
        return {
            "state": self.state,
            "ranges": {k: list(v) for k, v in self.ranges.items()},
            "gate": self.gate,
            "signs": dict(self.signs),
            "table": self.table,
            "fk": self.fk,
        }
