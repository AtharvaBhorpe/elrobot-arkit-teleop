"""Calibration step functions against a stub bus - no hardware, no EEPROM."""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for `stubs` below

from stubs import StubBus  # noqa: E402 - shared with tests/test_web_api.py

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


if __name__ == "__main__":
    test_read_ranges_tracks_min_max()
    test_gate_flags_short_spans()
    test_derive_table_midpoint_offsets()
    test_derive_table_asymmetric_urdf_range()
    print("CALIB STEPS TESTS PASSED")
