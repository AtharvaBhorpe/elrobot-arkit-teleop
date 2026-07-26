"""Calibration step functions against a stub bus - no hardware, no EEPROM."""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session

import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for `stubs` below

from stubs import StubBus  # noqa: E402 - shared with tests/test_web_api.py

from elrobot.calibration import backup
from elrobot.calibration.steps import derive_table, gate_ranges, read_ranges


def test_read_ranges_tracks_min_max():
    bus = StubBus({"rev_motor_01": 2000})
    stop = threading.Event()
    out = {}
    t = threading.Thread(target=lambda: out.update(
        read_ranges(bus, ["rev_motor_01"], poll_s=0.01, stop=stop)))
    t.start()
    time.sleep(0.05)
    bus.pos["rev_motor_01"] = 1500
    time.sleep(0.05)
    bus.pos["rev_motor_01"] = 2600
    time.sleep(0.05)
    stop.set()
    t.join(timeout=2)
    assert out["rev_motor_01"] == (1500, 2600)


def test_gate_flags_short_spans():
    # 3.4 rad expected (URDF) -> 3.4 * 651.9 = 2216.5 ticks expected span.
    # Same +/-20% SANE_TOLERANCE as the real, field-verified check_ranges.
    spans = {"rev_motor_01": 3.4}
    ok = gate_ranges({"rev_motor_01": (0, 2200)}, spans)       # -0.7% off
    bad = gate_ranges({"rev_motor_01": (1900, 2100)}, spans)   # 200 tk, -91% off
    assert ok[0]["ok"] and not bad[0]["ok"]


def test_derive_table_midpoint_offsets():
    # symmetric URDF limits (q_lo=-q_hi) so the urdf-midpoint term is 0 and
    # offset reduces to the tick midpoint - matches rev_motor_01 on the real
    # arm (URDF limit is +/-1.5509 rad, see docs/urdf_Elrobot.urdf)
    tbl = derive_table({"rev_motor_01": (1000, 3000)},
                       signs={"rev_motor_01": 1},
                       gripper={"closed_ticks": 2047, "open_ticks": 3586},
                       limits={"rev_motor_01": (-1.5509, 1.5509)})
    j = tbl["rev_motor_01"]
    assert j["offset"] == 2000.0 and j["sign"] == 1
    assert tbl["rev_motor_08"]["closed_ticks"] == 2047


def test_derive_table_asymmetric_urdf_range():
    # rev_motor_05/06/07 on the real arm are NOT centered on 0 - offset must
    # account for the urdf midpoint, not just the tick midpoint
    tbl = derive_table({"rev_motor_06": (1000, 3000)},
                       signs={"rev_motor_06": 1},
                       gripper={"closed_ticks": 2047, "open_ticks": 3586},
                       limits={"rev_motor_06": (-1.3775, 1.7641)})
    urdf_mid = (-1.3775 + 1.7641) / 2
    expected_offset = round(2000 - 651.9 * urdf_mid, 1)
    assert tbl["rev_motor_06"]["offset"] == expected_offset
    assert expected_offset != 2000.0   # would silently be wrong if ignored


def test_backup_round_trips_the_eeprom_the_wizard_destroys():
    """The whole point: after set_half_turn_homings() has overwritten every
    homing offset, restoring the snapshot must put the ORIGINAL values back.
    A backup that only round-trips an unchanged dict proves nothing."""
    bus = StubBus({f"rev_motor_{i:02d}": 1500 + 100 * i for i in range(1, 9)})
    for i, n in enumerate(bus.calib):          # give it a distinctive prior
        bus.calib[n]["homing_offset"] = 300 + i
        bus.calib[n]["range_min"] = 10 + i
    before = {n: dict(c) for n, c in bus.calib.items()}

    with tempfile.TemporaryDirectory() as d:
        snap_path = backup.snapshot(bus, "/dev/null", root=d)

        bus.set_half_turn_homings()            # the destructive write
        assert bus.calib != before, "stub did not actually change the EEPROM"

        # files=False: this test must never write the real calibration/*.json
        backup.restore(bus, backup.load(snap_path), files=False)

    assert bus.calib == before, "restore did not put the old EEPROM back"


def test_restore_unlocks_eeprom_before_writing():
    """Feetech gates EEPROM writes on `Lock`, which lerobot ties to torque.
    A driver exit leaves torque ON, so a restore that does not disable it
    first writes into locked EEPROM: the servo discards the value and still
    answers OK. StubBus starts locked precisely to catch that."""
    bus = StubBus({f"rev_motor_{i:02d}": 2000 for i in range(1, 9)})
    for n in bus.calib:
        bus.calib[n]["homing_offset"] = 111
    assert bus.locked, "stub must start locked - that is a driver's exit state"

    with tempfile.TemporaryDirectory() as d:
        snap = backup.load(backup.snapshot(bus, root=d))
    bus.set_half_turn_homings()
    backup.restore(bus, snap, files=False)

    assert bus.torque_disabled and not bus.locked
    assert all(c["homing_offset"] == 111 for c in bus.calib.values())


def test_restore_catches_a_write_that_was_silently_ignored():
    """The read-back backstop: if the EEPROM never took the value, restore
    must raise rather than report success - and must NOT go on to overwrite
    the calibration files, which still match the servos' real state."""
    class StubbornBus(StubBus):
        def disable_torque(self):        # a motor that stays locked
            self.torque_disabled = True

    bus = StubbornBus({f"rev_motor_{i:02d}": 2000 for i in range(1, 9)})
    for n in bus.calib:
        bus.calib[n]["homing_offset"] = 222
    with tempfile.TemporaryDirectory() as d:
        snap = backup.load(backup.snapshot(bus, root=d))
    bus.set_half_turn_homings()

    try:
        backup.restore(bus, snap, files=True)   # files=True: must not reach it
    except RuntimeError as e:
        assert "read-back" in str(e)
        assert all(c["homing_offset"] != 222 for c in bus.calib.values())
    else:
        raise AssertionError("reported success on an EEPROM that never changed")


def test_backup_refuses_a_protocol_that_cannot_read_offsets():
    """read_calibration() silently reports homing_offset=0 on protocol 1.
    A backup full of zeros looks fine and is worthless, so capture() must
    refuse rather than write one."""
    bus = StubBus({"rev_motor_01": 2047})
    bus.protocol_version = 1
    try:
        backup.capture(bus)
    except RuntimeError as e:
        assert "protocol_version" in str(e)
    else:
        raise AssertionError("captured a backup that would have been zeros")


def test_backup_never_clobbers_an_existing_snapshot():
    bus = StubBus({"rev_motor_01": 2047})
    with tempfile.TemporaryDirectory() as d:
        a = backup.snapshot(bus, root=d)
        b = backup.snapshot(bus, root=d)       # same second -> same stamp
        assert a != b and a.exists() and b.exists()
        assert backup.latest(d) == max(a, b)


def test_wizard_snapshots_before_it_can_write_and_refuses_without_one():
    """Preflight must produce a backup, and must NOT enter a state from
    which the EEPROM write is reachable if it could not."""
    from elrobot.web.calib import CalibError, CalibSession
    bus = StubBus({f"rev_motor_{i:02d}": 2047 for i in range(1, 9)})
    with tempfile.TemporaryDirectory() as d:
        s = CalibSession(bus_factory=lambda: bus, backup_root=d)
        snap = s.start(driver_alive=False)
        assert snap["state"] == "preflight"
        assert snap["backup"] and Path(snap["backup"]).exists()
        assert bus.set_half_turn_homings_calls == 0, "wrote before backing up"
        s.abort()

    # unwritable backup root -> refuse to start at all
    s2 = CalibSession(bus_factory=lambda: bus, backup_root="/proc/nope")
    try:
        s2.start(driver_alive=False)
    except CalibError as e:
        assert s2.state == "idle" and "back up" in e.detail
        assert s2.bus is None, "left the serial port held after refusing"
    else:
        raise AssertionError("started a calibration with no backup")


if __name__ == "__main__":
    test_read_ranges_tracks_min_max()
    test_gate_flags_short_spans()
    test_derive_table_midpoint_offsets()
    test_derive_table_asymmetric_urdf_range()
    test_backup_round_trips_the_eeprom_the_wizard_destroys()
    test_restore_unlocks_eeprom_before_writing()
    test_restore_catches_a_write_that_was_silently_ignored()
    test_backup_refuses_a_protocol_that_cannot_read_offsets()
    test_backup_never_clobbers_an_existing_snapshot()
    test_wizard_snapshots_before_it_can_write_and_refuses_without_one()
    print("CALIB STEPS TESTS PASSED")
