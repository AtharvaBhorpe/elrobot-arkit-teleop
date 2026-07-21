# Elrobot ARKit Teleop

Teleoperate a real 7-DoF + gripper Elrobot arm with an iPhone. ARKit pose
(via [ZIG SIM PRO](https://1-10.github.io/zigsim/)) drives the end-effector
through damped-least-squares servo IK; screen touches work the clutch and
gripper. Includes contact-sensing grasping, three DoF modes, dual-camera
recording straight into LeRobotDataset for imitation learning, and mcap
session capture for Foxglove replay.

```
iPhone ──UDP──▶ arkit_receiver ──/target_pose──▶ ik ──/joint_command──▶ elrobot_driver ──serial──▶ arm
                                                  ▲                          │
                                                  └───────/joint_states──────┘
```

Three ROS 2 nodes over DDS. `elrobot_driver` is the **only** code that
touches hardware — every safety rule (velocity clamp, workspace box,
singularity floor, deadman freeze, grasp latch) lives there and is unit
tested against a stub bus.

## Hardware

| | |
|---|---|
| Arm | Elrobot 7R + gripper, Feetech STS3215 servos, CH343 serial (`/dev/ttyACM*`) |
| Cameras | Innomaker U20CAM-1080P on the gripper + external webcam |
| Phone | iPhone running ZIG SIM PRO (ARKit + touch, UDP JSON, :50000) |

## Quickstart

```bash
# one-time: install pixi (pixi.sh), then
pixi install
pixi run prove-env          # all stacks import in one process
pixi run test               # every offline test suite (no hardware needed)

# a teleop session (arm plugged in, calibrated):
pixi run m3-arm             # phone drives the real arm (+ rviz)
pixi run cams               # WRIST_DEV=/dev/videoX EXT_DEV=/dev/videoY
pixi run record             # LeRobotDataset episodes (ENTER = start/stop)
pixi run bag                # optional: everything to mcap
```

First time on new hardware: calibrate first — the guided procedure is the
`/recalibrate` Claude skill, or follow the spec's Calibration section
(M1a → M1b → sign verification). **Never** run the driver uncalibrated.

## Tasks

| task | what |
|---|---|
| `m3-arm` / `m3-arm6` / `m3-arm5` | phone → real arm (7 / 6+1 / 5+1 DoF) |
| `jog` | slider GUI → real arm (sliders seed from the real pose) |
| `view`, `m2` | visualization only, no hardware |
| `cams` / `campick` / `rqt-cam` | camera nodes / identify devices / view feeds |
| `record` / `bag` | LeRobotDataset episodes / mcap rosbag |
| `bridge` | Foxglove WebSocket on :8765 (`rviz:=false` to drop rviz) |
| `test` / `lint` | all offline suites / ruff |

Tuning knobs as env prefixes: `PORT= SCALE= ORIENT=0 SMOOTH= MAX_VEL=
FREEZE= GRIP_LOAD_THRESH= GRIP_SQUEEZE= Z_MIN= R_MAX= RVIZ=0`.

## Safety model

- The driver's module docstring is the safety contract; `pixi run python
  tests/test_driver_safety.py` proves all 10 mechanisms on a stub bus.
- `calibration/*.json` and `docs/urdf_Elrobot.urdf` are hand-measured
  physical truth — never edited casually (a repo hook blocks agent edits).
- Driver exit leaves torque ON holding; `pixi run ticks` releases
  it (the arm goes limp — support it).
- Integration tests pin `ROS_DOMAIN_ID=77` so they can never touch a live
  session.

## Repository map

```
src/elrobot/
├── nodes/        the three ROS 2 nodes (driver, ik, receiver) + cams + recorder
├── control/      cartesian_ik — DLS servo IK, task-priority frozen-joint modes
├── calibration/  guided procedures: bus probe, M1a/M1b, FK verification
└── tools/        watch_ticks, cam_picker, nudge, viz-URDF generator
launch/           ros2 launch files (m3 / jog / cams / view / m2)
config/           view.rviz — edit this file to add displays, never rviz "Add Display"
tests/            offline suites, no hardware needed (pixi run test)
calibration/      servo tick ↔ URDF radian tables — sacred, hook-protected
docs/             real URDF (kinematic truth), vendored meshes, design spec
data/             episodes, bags, derived viz meshes (gitignored)
tasks/            implementation plan + todo
AGENTS.md         conventions + hard rules, each earned by an incident
```

The full design rationale and decision history:
[`docs/superpowers/specs/2026-07-20-elrobot-arkit-teleop-design.md`](docs/superpowers/specs/2026-07-20-elrobot-arkit-teleop-design.md).

## License

MIT. Arm meshes vendored from
[norma-core](https://github.com/norma-core/norma-core) (MIT), see
`docs/assets/LICENSE.norma-core`.
