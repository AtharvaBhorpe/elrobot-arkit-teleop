"""Shared test doubles for the offline suites - no hardware, ever."""

from dataclasses import dataclass

HALF_TURN = 2047        # what set_half_turn_homings() makes the current pose
MAX_TICKS = 4095


@dataclass
class _Calib:
    """Mirrors lerobot's MotorCalibration field-for-field, without importing
    it - backup.capture() only needs vars() to work."""
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


class StubBus:
    """Stands in for a real lerobot FeetechMotorsBus: dict-backed
    sync_read/sync_write, plus the no-op lifecycle methods
    (connect/disable_torque/set_half_turn_homings/disconnect) and the
    read_calibration/write_calibration pair that elrobot.calibration.backup
    snapshots and restores."""

    def __init__(self, positions):
        self.pos = dict(positions)          # name -> ticks
        self.writes = []
        self.connected = False
        self.torque_disabled = False
        # Feetech `Lock` (addr 55) gates EEPROM writes and lerobot ties it to
        # torque. Start LOCKED: that is the state a driver exit leaves the
        # arm in, and the one where a naive restore silently does nothing.
        self.locked = True
        self.set_half_turn_homings_calls = 0   # EEPROM write order matters
        self.calib = {n: {"id": i + 1, "drive_mode": 0, "homing_offset": 0,
                          "range_min": 0, "range_max": MAX_TICKS}
                      for i, n in enumerate(self.pos)}

    def sync_read(self, item, names=None, normalize=False):
        return {n: self.pos[n] for n in (names or self.pos)}

    def sync_write(self, item, values, normalize=False):
        self.writes.append((item, dict(values)))

    def connect(self, handshake=True):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def disable_torque(self):
        self.torque_disabled = True
        self.locked = False          # real disable_torque() writes Lock=0

    def set_half_turn_homings(self):
        self.set_half_turn_homings_calls += 1
        # Mutate the stub's EEPROM the way the real call does, so a test can
        # prove a restore actually puts the OLD offsets back rather than just
        # round-tripping a dict that nothing ever changed.
        for n, c in self.calib.items():
            c["homing_offset"] = self.pos[n] - HALF_TURN
        return {n: c["homing_offset"] for n, c in self.calib.items()}

    # --- the three destructible EEPROM registers (elrobot.calibration.backup)
    def read_calibration(self):
        return {n: _Calib(**c) for n, c in self.calib.items()}

    def write_calibration(self, calibration_dict, cache=True):
        if self.locked:
            return       # servo discards the write and still answers OK
        self.calib = {n: dict(vars(c)) for n, c in calibration_dict.items()}
