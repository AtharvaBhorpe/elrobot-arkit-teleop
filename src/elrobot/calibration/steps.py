"""Pure calibration computations, shared by the CLI scripts (m1a/m1b/verify)
and the web wizard. No input()/print() here - callers own all interaction
and output framing; these functions only compute or touch the bus/disk.

Extracted from m1a_calibrate.py, m1b_reconcile.py and verify_table.py -
the math below is a straight move, not a reimplementation. urdf_ticks.json's
schema is FLAT ({"rev_motor_01": {"offset", "sign"}, ..., "rev_motor_08":
{"closed_ticks", "open_ticks"}}) because that's what elrobot_driver.Converter
reads - see its __init__/arm_ticks/grip_ticks.
"""

import json
import math
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

TICKS_PER_RAD = 651.9
ENC = 4096
SANE_TOLERANCE = 0.20  # same rule as m1a's check_ranges
TCP_FRAME = "Gripper_Base_v1_1"
DISCRIMINATING_CM = 5.0  # verify_table's threshold for "sign is testable here"


def read_ranges(bus, joints, poll_s=0.05, stop=None):
    """Track min/max raw ticks for each joint until `stop` is set.

    Web-wizard equivalent of m1a's interactive sweep (ENTER-to-stop) and
    m1b's sweep_range (minus its encoder-unwrap - callers needing that use
    unwrap() directly, same as m1b's CLI sweep for joints 5/7).
    """
    mins, maxs = {}, {}

    def sample():
        vals = bus.sync_read("Present_Position", joints, normalize=False)
        for n, v in vals.items():
            mins[n] = v if n not in mins else min(mins[n], v)
            maxs[n] = v if n not in maxs else max(maxs[n], v)

    sample()
    while not (stop is not None and stop.is_set()):
        time.sleep(poll_s)
        sample()
    return {n: (mins[n], maxs[n]) for n in joints}


def unwrap(prev_raw, raw, acc):
    """Track absolute travel across the 0/4095 encoder boundary (m1b)."""
    d = raw - prev_raw
    if d > ENC // 2:
        d -= ENC
    elif d < -ENC // 2:
        d += ENC
    return acc + d


def gate_ranges(ranges: dict, urdf_spans_rad: dict) -> list:
    """M1a's 'ranges sane' gate: recorded tick span within +/-20% of the
    URDF-expected span (same SANE_TOLERANCE as check_ranges - the tolerance
    that actually shipped and was field-verified, not a looser guess)."""
    out = []
    for name, (lo, hi) in ranges.items():
        span = hi - lo
        expected = urdf_spans_rad[name] * TICKS_PER_RAD
        span_pct = (span - expected) / expected
        out.append({"name": name, "span_pct": span_pct,
                   "ok": abs(span_pct) <= SANE_TOLERANCE})
    return out


def write_homing(bus):
    """THE EEPROM write - nothing else in this module writes to the bus.
    lerobot's set_half_turn_homings() reads the bus's CURRENT position and
    assigns it tick 2047 (half turn) for every motor; it takes no arguments
    of its own (the original plan's `park_ticks` param doesn't exist on the
    real API - the bus supplies its own current position)."""
    return bus.set_half_turn_homings()


def derive_table(ranges: dict, signs: dict, gripper: dict, limits: dict) -> dict:
    """m1b's offset math: ticks = offset + sign * TICKS_PER_RAD * q_urdf,
    offset = tick_midpoint - sign * TICKS_PER_RAD * urdf_midpoint.

    `limits` (URDF q_lo/q_hi per joint) is REQUIRED, not optional: joints
    5/6/7 on this arm have URDF ranges NOT centered on 0 (midpoints
    +0.32/+0.19/-0.23 rad), so dropping the urdf-midpoint term would silently
    mis-derive their offsets. Returns the FLAT schema the driver reads
    (no "joints"/"gripper" nesting).
    """
    table = {}
    for n, (lo_t, hi_t) in ranges.items():
        q_lo, q_hi = limits[n]
        offset = (lo_t + hi_t) / 2 - signs[n] * TICKS_PER_RAD * (q_lo + q_hi) / 2
        table[n] = {"offset": round(offset, 1), "sign": signs[n]}
    table["rev_motor_08"] = dict(gripper)
    return table


def save_table(table: dict, path="calibration/urdf_ticks.json"):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(table, indent=2))


def fk_report(table_path, q_ticks: dict, urdf="docs/urdf_Elrobot.urdf") -> dict:
    """verify_table's FK check, minus the printing: decode ticks -> q via the
    table, run FK, report TCP height/radius and per-joint sign sensitivity
    (how far the TCP would move if that joint's sign were flipped - near the
    URDF neutral pose every sign decodes ~alike, so sensitivity tells you
    whether the held pose can actually discriminate a wrong sign)."""
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    jid = {model.names[j]: j for j in range(1, model.njoints)}
    fid = model.getFrameId(TCP_FRAME)
    table = (json.loads(Path(table_path).read_text())
             if isinstance(table_path, (str, Path)) else table_path)
    arm = [n for n in table if n != "rev_motor_08"]
    qmid = {n: (model.lowerPositionLimit[model.joints[jid[n]].idx_q]
                + model.upperPositionLimit[model.joints[jid[n]].idx_q]) / 2
            for n in arm}

    def decode(ticks, tab):
        q = pin.neutral(model)
        for n in arm:
            q[model.joints[jid[n]].idx_q] = (
                (ticks[n] - tab[n]["offset"]) / (tab[n]["sign"] * TICKS_PER_RAD))
        return q

    def tcp(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation.copy()

    q = decode(q_ticks, table)
    p = tcp(q)

    sensitive = {}
    for n in arm:
        alt = {k: dict(v) for k, v in table.items() if k in arm}
        s = -table[n]["sign"]
        tm = table[n]["offset"] + table[n]["sign"] * TICKS_PER_RAD * qmid[n]
        alt[n] = {"sign": s, "offset": tm - s * TICKS_PER_RAD * qmid[n]}
        shift_cm = float(np.linalg.norm(tcp(decode(q_ticks, alt)) - p)) * 100
        sensitive[n] = shift_cm >= DISCRIMINATING_CM

    return {"height_m": float(p[2]), "radius_m": float(math.hypot(p[0], p[1])),
           "sensitive": sensitive}
