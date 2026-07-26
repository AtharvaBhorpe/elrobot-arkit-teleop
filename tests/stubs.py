"""Shared test doubles for the offline suites - no hardware, ever."""


class StubBus:
    """Stands in for a real lerobot FeetechMotorsBus: dict-backed
    sync_read/sync_write, plus the no-op lifecycle methods
    (connect/disable_torque/set_half_turn_homings/disconnect) the web
    wizard's session (elrobot.web.calib.CalibSession) calls through
    start()/eeprom()/finish()."""

    def __init__(self, positions):
        self.pos = dict(positions)          # name -> ticks
        self.writes = []
        self.connected = False
        self.torque_disabled = False
        self.set_half_turn_homings_calls = 0   # order of the EEPROM write matters

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

    def set_half_turn_homings(self):
        self.set_half_turn_homings_calls += 1
        return {n: 0 for n in self.pos}
