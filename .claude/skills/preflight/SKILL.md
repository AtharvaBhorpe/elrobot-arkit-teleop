---
name: preflight
description: Pre-session hardware checklist for the Elrobot teleop stack - run before any launch (m3-arm, jog, cams, record, web). Checks serial port, cameras, orphan processes, ports 8765/8080, and calibration presence; reports fixes for anything wrong.
---

Run every check below, then report a table of PASS/FAIL with the fix for
each failure. Do not launch anything; this skill only inspects and cleans.

## Checks

1. **Serial port**: `ls /dev/ttyACM*`. Missing → arm unplugged or held by
   another host. Present but not ACM0 → remind: `PORT=/dev/ttyACMx pixi run …`.
   If ACM1 appears after a replug, something held ACM0 during re-enumeration.

2. **Cameras**: `ls /dev/video*`. Expect the wrist + external cams (even
   numbers are capture nodes). Unsure which is which → `pixi run campick`
   (close it before starting `cams` — devices are single-owner).

3. **Orphan processes** (the top field-failure cause):
   `ps aux | grep -E "ik_node|arkit_receiver|cam_node|elrobot_driver|foxglove_bridge|robot_state_pub|episode_recorder|elrobot.web" | grep -v grep`
   Anything running that the user did not knowingly start → list pid + name
   and offer to kill. NOTE: killing a `pixi run` wrapper leaves the child
   alive — kill the child pid itself. Kill by the pid you identified, never
   by pattern: a broad `pkill -f` here has killed a live session's server.

4. **Port 8765** (Foxglove bridge): `ss -tlnp | grep 8765`. Held → the
   bridge will die with "Bind Error"; kill the holder.

4b. **Port 8080** (web cockpit): `ss -tlnp | grep 8080`. Held → a second
   `pixi run web` fails to bind and dies quietly, and every request you then
   make to localhost:8080 is answered by the FIRST server — which has once
   meant enabling web control on someone's live session. Never assume a
   cockpit you started is the one answering.

5. **Calibration present**: `calibration/urdf_ticks.json` and
   `calibration/elrobot.json` exist. Missing → STOP; the driver must not run
   uncalibrated. Point to /recalibrate.

6. **Torque state warning**: if the previous session ended with the driver,
   torque is ON and the arm is holding. `pixi run ticks` releases it
   (arm goes limp — it must be supported or resting).

## Output

A short table: check | status | fix. End with the exact launch commands for
the session the user described (default: `pixi run m3-arm`, `pixi run cams`,
`pixi run record`, optional `pixi run web` / `pixi run bag` / `pixi run bridge`).
