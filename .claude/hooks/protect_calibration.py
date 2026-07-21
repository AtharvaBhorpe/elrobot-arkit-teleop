#!/usr/bin/env python3
"""PreToolUse hook: block edits to hand-measured calibration artifacts.

calibration/*.json and docs/urdf_Elrobot.urdf encode PHYSICAL truth
(servo tick offsets, joint signs, vendor kinematics). An accidental edit
does not break a build - it makes the real arm move wrong. Exit 2 blocks
the tool call; the user can still edit these deliberately by hand.
"""

import json
import sys

PROTECTED = ("calibration/urdf_ticks.json", "calibration/elrobot.json",
             "docs/urdf_Elrobot.urdf")

data = json.load(sys.stdin)
path = data.get("tool_input", {}).get("file_path", "")
if any(path.endswith(p) for p in PROTECTED):
    print(f"BLOCKED: {path} is a hand-measured calibration artifact "
          "(see AGENTS.md rule 1). If this change is truly intended, the "
          "human edits it directly or re-runs the calibration procedure "
          "(/recalibrate).", file=sys.stderr)
    sys.exit(2)
sys.exit(0)
