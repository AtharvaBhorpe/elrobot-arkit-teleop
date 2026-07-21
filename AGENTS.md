# Elrobot ARKit Teleop — Agent Conventions

Phone (ZIG SIM PRO / ARKit) teleoperates a real 7-DoF + gripper Elrobot arm
over Feetech STS3215 serial servos. Everything runs inside the pixi env:
`pixi run <task>`. The design spec + full decision history:
`docs/superpowers/specs/2026-07-20-elrobot-arkit-teleop-design.md`.

## Architecture (three ROS 2 nodes over DDS)

```
phone -UDP-> arkit_receiver -/target_pose-> ik -/joint_command-> elrobot_driver -serial-> arm
                                                     ^                  |
                                                     +--/joint_states---+
```

- `scripts/elrobot_driver.py` — the ONLY code touching hardware; ALL safety
  lives here (slew/velocity clamp, workspace box, sigma floor, deadman,
  grasp latch). Its module docstring is the safety contract.
- `scripts/cartesian_ik.py` — DLS servo IK; task-priority when joints are
  frozen (SO-101 modes). Self-test: `pixi run python scripts/cartesian_ik.py`.
- `scripts/make_viz_urdf.py` — derives the display model (DAE meshes,
  relocated jaw frames, camera). Kinematics ALWAYS from docs/urdf_Elrobot.urdf.

## Tasks

| task | what |
|---|---|
| `m3-arm` / `m3-arm6` / `m3-arm5` | phone drives real arm (7 / 6+1 / 5+1 DoF) |
| `jog` | slider GUI drives real arm (sliders seed from real pose) |
| `cams`, `campick`, `rqt-cam` | camera nodes / identifier GUI / feed viewer |
| `record`, `bag` | LeRobotDataset episodes / mcap rosbag |
| `bridge` | Foxglove websocket :8765 (`rviz:=false` to drop rviz) |
| `view`, `m2` | viz-only modes, no hardware |

Env knobs (prefix any launch): `PORT= SCALE= ORIENT=0 SMOOTH= MAX_VEL=
FREEZE= GRIP_LOAD_THRESH= GRIP_SQUEEZE= Z_MIN= R_MAX= RVIZ=0`.

## Hard rules (each earned by an incident)

1. **Calibration artifacts are sacred**: `calibration/*.json` and
   `docs/urdf_Elrobot.urdf` encode hand-measured physical truth. Never edit
   without an explicit human decision. Re-running M1a rewrites servo EEPROM
   and invalidates the M1b table.
2. **Tests never touch the default DDS domain**: integration tests pin
   `ROS_DOMAIN_ID=77`. A test once published /joint_command at the LIVE
   driver. Any new test that creates ROS nodes must set a domain.
3. **One process per device**: serial port and each /dev/video* are
   single-owner. Don't run campick while cams holds a device; don't start a
   second driver. Orphaned children survive `kill` of a `pixi run` wrapper —
   kill the child pid.
4. **Never use rviz "Add Display"** in this env (conda rviz2 heap-crashes
   creating render panels at runtime). Add displays by editing
   `scripts/view.rviz`. Foxglove is the crash-free GUI.
5. **Driver exit leaves torque ON holding**; `scripts/watch_ticks.py`
   disables torque (arm goes limp — support it).
6. **Hand-observed joint signs are pose-dependent** (2 of 7 were recorded
   flipped). Only the slider-vs-hand direction test or a bent-pose FK check
   with a real tape measure is authoritative.
7. **Safety thresholds come from measurement, not theory** — the sigma floor
   was re-placed twice after biting legitimate poses. Measure the operating
   distribution before moving any threshold.

## Test suites (all offline-safe, run before committing driver/IK changes)

- `pixi run python scripts/test_driver_safety.py` — 10 checks, stub bus
- `pixi run python scripts/test_m2_pipeline.py` — end-to-end, fake phone
- `pixi run python scripts/cartesian_ik.py` — IK self-test incl. frozen modes
- `pixi run python scripts/test_recorder.py` — dataset round-trip
