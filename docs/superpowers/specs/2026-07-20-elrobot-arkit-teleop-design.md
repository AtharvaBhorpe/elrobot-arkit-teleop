# ARKit Phone Teleoperation of the Elrobot Arm — Design

**Date:** 2026-07-20 (updated 2026-07-21)
**Status:** M0, M1a, M1b done (joints 1/6 pending one bent-pose check; 5/7 roll signs
confirmed in M2). Next: M2 — receiver + IK + viewer, no hardware.

## Goal

Teleoperate a physical 7-DOF + gripper Elrobot arm from an iPhone running ZIG SIM PRO.
Phone pose (ARKit) drives the end-effector through Pinocchio servo-IK; screen touches
provide clutch and gripper control.

This ports the architecture of the existing
[franka-isaac-arkit-teleop](https://github.com/AtharvaBhorpe/franka-isaac-arkit-teleop)
project from Isaac Sim to real hardware. The receiver and IK layers carry over nearly
unchanged; all new work sits below the joint-command boundary.

## Hardware context

| | |
|---|---|
| Arm | Elrobot, 7 revolute joints (`rev_motor_01..07`) + gripper (`rev_motor_08`) |
| Gripper | one servo driving two prismatic jaws as URDF `<mimic>` joints |
| Servos | Feetech STS3215, 4096 ticks/rev, ~2.94 N·m stall |
| Bus | single half-duplex serial via CH343 USB adapter (`1a86:55d3`) |
| Port | `/dev/ttyACM0` — CH343 binds `cdc_acm`, **not** `ttyUSB`. Needs `dialout`. |
| URDF | extracted from the NormaCore `station` binary; 18 joints, 19 links |

### Why not NormaCore station

Station is the vendor platform that ships with the arm. It is not used as the control
path, for two evidenced reasons:

1. **Throughput.** Station reads motors individually at a measured 20–21 ms per
   transaction (1786 samples, evenly spread across all 8 motors). Eight sequential
   reads ≈ 160 ms ≈ 6 Hz — far below what an IK servo loop needs.
2. **A driver defect.** After a read times out, station does not drain the RX buffer,
   so the next motor's transaction consumes the stale reply. Observed cascade:
   `motor 1: Timeout` → `motor 3: expected 3, got 1` → `motor 6: expected 6, got 3`.
   This silently corrupts calibration by leaving motors half-reset.

LeRobot's `FeetechMotorsBus` is used instead: it offers `sync_read`/`sync_write`
(all motors in one transaction) and performs its own RX handling.

## Architecture

Three ROS 2 nodes over localhost DDS. Only `elrobot_driver` is new.

```
iPhone ──UDP/JSON──▶ arkit_receiver ──/target_pose──▶ ik ──/joint_command──▶ elrobot_driver ──serial──▶ arm
                                                       ▲                          │
                                                       └──────/joint_states───────┘
```

All ARKit-specific logic stays inside `arkit_receiver`; everything downstream is
input-agnostic, so the phone can later be swapped for another pose source without
touching the control path.

### `arkit_receiver`

Ports from the Franka project unchanged. Listens on a UDP socket, parses ZIG SIM JSON,
emits a pose target per received packet (event-driven, not polled).

ZIG SIM payload fields:

| Field | Type |
|---|---|
| `sensordata.arkit.position` | `[x, y, z]` metres |
| `sensordata.arkit.rotation` | `[x, y, z, w]` quaternion |
| `touch` | int count (0, 1, 2) |

ZIG SIM supports 1/10/30/60 FPS; the Franka project ran at ~10 Hz.

### `ik`

Ports from the Franka project with the URDF and TCP frame swapped. Damped least-squares
Cartesian servo:

```
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

with singularity-adaptive λ, joint-velocity clamping, and joint-limit clamping from the
URDF. Runs over `rev_motor_01..07` only. TCP frame is defined on `Gripper_Base_v1_1`.

`rev_motor_08` is excluded from IK and driven directly by the gripper toggle. The two
prismatic jaws are mechanically coupled and are never commanded.

### `elrobot_driver` (new)

The only component touching hardware. Therefore all safety lives here.

- Subscribes `/joint_command`, converts 7 joint angles + 1 gripper state to servo ticks
- `sync_write`s ticks; `sync_read`s state back to `/joint_states`
- Enforces velocity clamp, workspace bounds, manipulability floor, gripper current limit
- Freezes on packet-loss deadman

## Interaction model

| Gesture | Effect |
|---|---|
| 1 finger held | Clutch engaged. Reference pose re-zeroed on engage; phone motion drives the TCP. |
| Release | Motion freezes immediately. |
| 2-finger tap | Toggles gripper open/closed. Latched — persists until next toggle. |
| No packet for 200 ms | Deadman: freeze in place. |

The 200 ms deadman is a default, not a tuned value: at ZIG SIM's 10 Hz stream rate it
tolerates one dropped packet and fires on the second. Retune if the stream rate changes.

Hold-to-move was chosen over a latched clutch because on real hardware "release = stop"
is the safest available failure mode. The latched gripper composes correctly with it:
tap to grip, then move with one finger while carrying the object.

## Coordinate frames

Axis remap (`ARKIT_TO_ROS`), **retuned 2026-07-21 on the live phone run** for the
actual operator stance — standing at the base, looking out along the arm (+Y),
phone camera pointed at the robot:

- device +X (phone right) → robot +X (right)
- device −Z (phone forward) → robot +Y (arm forward)
- device +Y (phone up) → robot +Z (up)

The Franka project's map had the operator rotated 90°: phone-right came out as
robot-backward here. Rotations go through `C·R·Cᵀ`, so this same matrix puts the
phone's screwdriver axis (camera direction) on the arm's length axis: roll the
phone while pointing it at the robot and the gripper rolls clockwise-for-
clockwise as seen from the base. The receiver logs the dominant rotation axis
(1 Hz, ≥15°) for future axis debugging.

Orientation is applied relative to the clutch-engagement pose and remapped to the base
frame as `C · (R_now · R_refᵀ) · Cᵀ`.

**Motion scale must be retuned.** The Elrobot's reach is 0.424 m against the Franka's
~0.85 m, so the Franka project's scaling does not port. Default to **0.4** (40 cm of
phone travel maps to 16 cm of TCP travel), exposed as config and tuned in M4.

## Calibration

Two distinct steps. LeRobot's calibration alone is **not sufficient** for IK.

### M1a — LeRobot motor calibration

Standard `FeetechMotorsBus` procedure: torque off → `set_half_turn_homings()` →
`record_ranges_of_motion()` → writes homing offsets to servo EEPROM plus a JSON.

Joints 5 and 7 have near-full-revolution ranges (336° and 340°) and will likely need
LeRobot's `full_turn_motors` treatment rather than swept-range recording — the same
special-casing the SO-101 applies to `shoulder_pan` and `wrist_roll`.

Calibration is interactive and hand-paced, so it is insensitive to bus latency.

**DONE 2026-07-21** (`scripts/m1a_calibrate.py`, output `calibration/elrobot.json`).
Notes for a re-run:

- lerobot 0.6.1 has no `full_turn_motors` *parameter* — the treatment is done at the
  robot-class level: exclude those joints from the sweep and assign `0..4095`
  (see `SOFollower.calibrate()`). Safe here because the control path never uses
  lerobot normalization; real limits come from the URDF.
- The park pose needs no precision: swept joints tolerate ±79° of centring error.
  Any relaxed pose with no joint within ~20° of a hard stop works.
- Every swept joint recorded −1 to −3% of its URDF-expected span (a hand sweep
  stops just shy of the stops) — physical corroboration of the URDF limits.
- Gripper recorded 2047→3586 (closed→open), parked fully closed.

### M1b — URDF ↔ tick reconciliation

LeRobot places joint zero at the **midpoint of the recorded range**. The URDF's zero is
the CAD neutral pose. These do not coincide, and nothing reconciles them automatically.
Feeding IK output straight to LeRobot-normalized commands yields a smooth, confident
move to the wrong pose.

Scale is fixed — STS3215 is direct-drive at 4096 ticks/rev = **651.9 ticks/rad** — so
only a per-joint offset and sign are needed:

```
ticks_i = offset_i + sign_i * 651.9 * q_urdf_i
```

Procedure, once:

1. Place the arm in the **URDF neutral pose** (`pin.neutral(model)`, all joints at 0),
   matched physically against the model rendered in the viewer. Read ticks on each
   joint → `offset_i`.
2. Move each joint in its **+URDF** direction; if ticks decrease, `sign_i = -1`.

Reproducibility matters more than accuracy here: any consistently identifiable pose
works, provided the same pose is used when re-deriving the table after a recalibration.

The gripper needs no URDF correspondence — record ticks at fully-open and fully-closed
(with a current limit) and map the toggle to those two values.

**Verification gate:** command a known joint vector, read back, run Pinocchio FK, and
compare predicted TCP against the arm's actual pose in the URDF viewer. This catches a
sign error before the arm swings into the table.

**DONE 2026-07-21** (`scripts/m1b_reconcile.py` derives the table, `scripts/verify_table.py`
re-checks it; output `calibration/urdf_ticks.json`). What was learned:

- **Offsets came from range midpoints, not a posed neutral.** The URDF has no meshes,
  so matching `pin.neutral` by eye is impractical. Midpoint-of-recorded-range paired
  with midpoint-of-URDF-range is self-correcting: the uniform −1 to −3% sweep
  undershoot cancels. Joints 1–4 have symmetric URDF limits, so their offsets are
  sign-independent for free.
- Joints 5/7 got their ranges here instead (hand sweep with software encoder-wrap
  unwrapping): spans −2.5% and −1.0% vs URDF — same band as the other five.
- **Signs were observed via FK-derived plain-English prompts** ("move so the gripper
  goes DOWN"), computed from TCP displacement — not the raw `+URDF direction`, which
  is unactionable by hand.
- **Verify at a clearly BENT pose.** Near neutral every joint decodes to q≈0 for
  either sign, so the FK gate passes vacuously. At a bent pose flipped signs shift
  the TCP 6–16 cm (joints 2/3/4) — verified against tape measurements agreeing to
  ~1.5 cm once the ~3 cm base-pedestal height (URDF base frame sits at the joint-1
  rotation plane, not the table) is accounted for.
- **Joints 5 and 7 cannot be position-verified at any pose** — they are rolls whose
  axes pass through the TCP; flipping them moves it ≤3.6 cm / 0.0 cm. Their signs
  are confirmed in M2 by comparing model gripper roll to the real arm (fix = one
  character in the JSON, no recalibration). Joints 1/6 signs are FK-consistent but
  weakly tested so far; one bent-pose `verify_table.py` run clears them before M3.
- Re-running M1a rewrites EEPROM homing offsets and **invalidates this table**.

**Incident (2026-07-21): joint 2's sign was recorded flipped in M1b.** Root cause:
the sign prompts ("move so the gripper goes DOWN") were derived from FK **at
neutral**, but the arm was in an arbitrary folded pose during observation — for
shoulder-class joints the TCP can sit on the other side of the joint axis, where
the same +q moves the gripper the opposite way. Caught by the bent-pose FK check
(prediction 35 cm up vs reality 2 cm up), pinned by an exhaustive flip-fit against
two measured poses, and settled by the measurement-free direction test: raise the
gripper by hand and watch decoded q2 in `watch_ticks.py` (+q2 is FK-verified to
move the TCP down, so q2 must fall as the gripper rises). Fix: one sign flip in
`calibration/urdf_ticks.json` (offset unchanged — midpoint-derived, symmetric
limits). **Lesson: hand-observed signs are pose-dependent; only the bent-pose FK
gate or the watch_ticks direction test is authoritative.** A crude first
measurement ("iPhone units") had wrongly passed joint 2 — measure verification
poses with a real tape.

## Verified findings

Established by execution against the extracted URDF (Pinocchio 4.1.0), not assumed.

| Finding | Value | Consequence |
|---|---|---|
| URDF loads without meshes | `nq=10, nv=10` | kinematics-only build is sufficient |
| **Pinocchio ignores `<mimic>`** | both jaws are independent DOFs | driver **must index q by joint name**, never 1:1 to motors |
| **Jaw origins are wrong in the URDF** | `rev_motor_08_1/_2` origins kept CAD world coords — jaws sit ~38 cm from `Gripper_Base_v1_1` | **upstream vendor bug**, not an extraction artifact: norma-core's own `elrobot_follower.urdf` is joint-for-joint identical to ours (all 18 joints). Leaf joints, so arm chain/IK/TCP unaffected. Never trust jaw geometry (collision checks, viz, grasp points) |
| Arm Jacobian | 6×7, rank 6 in 100% of 4000 poses | full 6-DOF control, 1 redundant DOF |
| Workspace | 0.628 × 0.621 × 0.540 m | — |
| Max reach | 0.424 m | motion scale must drop to ~0.4 |
| Near-singular volume | 13.9% at σ_min < 0.01 | adaptive damping is load-bearing; add manipulability floor |
| Worst gravity torque | 0.973 N·m on `rev_motor_02` = 33.1% of stall | arm holds its own poses; 0 of 4000 poses exceed stall |
| Model mass | 0.515 kg | light for 8 servos — treat torque margin as ~2× derated |

Joint limits (rad), from the URDF:

```
rev_motor_01  [-1.5509, +1.5509]
rev_motor_02  [-1.6122, +1.6122]
rev_motor_03  [-1.7610, +1.7610]
rev_motor_04  [-1.7533, +1.7533]
rev_motor_05  [-2.6200, +3.2520]
rev_motor_06  [-1.3775, +1.7641]
rev_motor_07  [-3.2014, +2.7336]
```

### Bus measurements (M0, 2026-07-21)

Measured on hardware, 200 samples per transaction, all 8 motors per call, torque
disabled. lerobot 0.6.1 `FeetechMotorsBus`, `normalize=False`, protocol 0.

| Transaction | p50 | p95 | max | Failures |
|---|---|---|---|---|
| `sync_read` (8 motors) | **1.34 ms** | 1.39 ms | 1.79 ms | 0/200 |
| `sync_write` (8 motors) | **0.32 ms** | 0.37 ms | 0.59 ms | 0/200 |

- **Gate passed** with ~3.7× margin (1.34 ms against the 5 ms threshold).
- **~122× faster than station.** Station's 20–21 ms per *individual* motor gives
  ≈164 ms ≈ 6 Hz for 8 reads. One `sync_read` does all 8 in 1.34 ms (~746 Hz).
- **Full read+write cycle ≈ 1.66 ms → ~600 Hz ceiling.** The bus is not the
  bottleneck; IK solve time will dominate the servo loop.
- **Jitter is negligible** — p95 within 0.05 ms of p50. No CDC-ACM latency tuning
  needed. Tight jitter matters more than raw speed for a servo loop.
- **Zero desync in 200 reads.** Station's stale-RX defect did not reproduce;
  lerobot's own RX handling holds. `connect(handshake=True)` confirmed all 8
  servos answer at their expected IDs.
- Writes are ~4× faster than reads because Feetech SYNC WRITE is a broadcast with
  no status reply — the host never waits on the half-duplex bus.

## Safety requirements

Non-negotiable, all enforced in `elrobot_driver`:

- **Velocity clamp** per joint on every command
- **Workspace bounding box** — targets outside it rejected before IK runs
- **Manipulability floor** — refuse or heavily damp when σ_min falls below threshold
- **Gripper current limit** — motor 8 has already latched an Overload (`status=0x20`)
  from being commanded past its mechanical stop
- **Deadman freeze** on packet loss or clutch release

In simulation a bad IK solve is a visual glitch. Here it is a collision.

## Risks and open questions

| Risk | Status |
|---|---|
| Bus rate under LeRobot `sync_read` | **RESOLVED 2026-07-21.** Measured p50 1.34 ms for all 8 motors. See Bus measurements. |
| Does `FeetechMotorsBus.connect()` require calibration to read? | **RESOLVED 2026-07-21: no.** `connect(handshake=True)` takes no calibration argument, and `sync_read(..., normalize=False)` returns raw ticks uncalibrated. M0 and M1 are independent. |
| RoboStack ROS 2 Jazzy + lerobot in one env | **RESOLVED 2026-07-21: proven.** One pixi env, Python 3.12. See Environment. |
| URDF inertias understated | likely; derate torque margin ~2× |
| Motor 8 mechanical state | overload was commanded-past-stop, not gravity; confirm jaws move freely by hand |

## Environment (proven 2026-07-21)

One `pixi` env holds the whole stack — no split env or per-node isolation needed.
`pixi.toml` / `pixi.lock` at repo root; `pixi run prove-env` re-verifies.

| | |
|---|---|
| Python | 3.12 |
| ROS 2 | Jazzy via RoboStack (`ros-jazzy-ros-base` — no rviz/gazebo) |
| Pinocchio | 4.1.0 (conda-forge) |
| lerobot | 0.6.1 |
| numpy | 2.2.6 |

Four non-obvious constraints, each found by a failed solve:

1. **PyPI `lerobot` is a stale placeholder.** `lerobot==0.1.0` on PyPI ships only
   `datasets/envs/policies` — **no motor or Feetech code at all**. The maintained
   bus exists only on git. Install from the GitHub source with the `feetech` extra.
2. **Python must be 3.12.** git lerobot requires `>=3.12`; Jazzy targets 3.12
   upstream anyway. 3.11 fails to resolve.
3. **numpy must be 2.x** (lerobot pins `>=2.0,<2.3`), while conda would otherwise
   serve numpy 1.26 for ROS/Pinocchio. RoboStack Jazzy has numpy-2 builds, so
   rclpy and Pinocchio import cleanly under numpy 2 — verified, not assumed.
4. **`setuptools` and `packaging` need conda-side pins** (`setuptools>=71,<81`,
   `packaging>=24.2,<26`) or the conda solve pins versions lerobot rejects.

`FeetechMotorsBus` lives at `lerobot.motors.feetech`. Read raw ticks with
`sync_read(..., normalize=False)` — normalization requires calibration, so
uncalibrated work (M0, M1b) must pass `normalize=False`.

## Milestones

| | Milestone | Gate |
|---|---|---|
| M0 | Bus probe — `sync_read`/`sync_write` rate, desync check | ✅ **PASSED 2026-07-21.** p50 1.34 ms read / 0.32 ms write, 0 desync. `scripts/m0_bus_probe.py` |
| M1a | LeRobot calibration | ✅ **PASSED 2026-07-21.** All 8 within −3% of URDF spans. `scripts/m1a_calibrate.py` |
| M1b | URDF↔tick offset/sign table | ✅ **PASSED 2026-07-21** for joints 2/3/4 (FK vs tape, ~1.5 cm). Joints 1/6: one bent-pose `verify_table.py` run pending; 5/7: confirmed visually in M2 |
| M2 | Receiver + IK + viewer, no hardware | ✅ **PASSED 2026-07-21, live.** Phone drives the model accurately (translation + rotation) after the axis-map retune; 60 pkt/s handled; gripper latch confirmed in logs (gear rotation is the rviz indicator — jaws are stripped from the viz model). Synthetic-phone regression: `scripts/test_m2_pipeline.py` |
| M3 | Driver node, position-only, hard velocity clamp | arm tracks phone safely |
| M4 | Full 6-DOF pose + gripper | pick-and-place |
| M5 | Safety hardening and tuning | all safety rules enforced and tested |

M2 carries zero hardware risk and validates the entire upper pipeline — the role Isaac
played in the Franka project.

## Out of scope

Deliberately excluded from this design: imitation-learning data collection, Rerun
recording, VLA policy integration, and any NormaCore station interoperability. Those
are later phases and none of them change the decisions above.
