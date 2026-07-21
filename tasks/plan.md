# Implementation Plan: Milestone Closure → First Trained Policy

## Overview

The teleop stack is functionally complete (M0–M2 passed; M3 built, field-run,
and stable; grasp sensing, three DoF modes, two cameras, LeRobotDataset +
mcap recording, Foxglove). What remains is: formally close the spec's
milestones (M3/M4/M5), then walk the now-unblocked path the spec deferred —
collect a real dataset and train + deploy a first imitation-learning policy
through the existing safety-gated driver. Every task below rides on
infrastructure that already exists and is tested.

## Architecture Decisions (already made; tasks inherit them)

- The driver is the ONLY hardware writer; a future policy node publishes
  /joint_command exactly like the ik node does — every safety gate applies
  to a policy unchanged. No new safety surface is created by Phase 3.
- Datasets are LeRobotDataset (training-native); mcap bags are the debug/
  replay channel. Both already record.
- Tests pin ROS_DOMAIN_ID=77; nothing in this plan touches a live session.

## Task List

### Phase 1: Close the spec milestones

- [ ] **Task 1: Formal M3 gate session** (S, no code)
  Run `/preflight`, then a pick-carry-place cycle per DoF mode
  (`m3-arm`, `m3-arm6`, `m3-arm5`).
  *Accept:* each mode completes a pick-carry-place without safety holds
  misfiring or manual rescue; user declares which mode feels best.
  *Verify:* driver log shows grasp latch + release; no SAFETY HOLD during
  legitimate motion. Spec M3 row marked PASSED with the chosen defaults.
  *Depends:* none.

- [ ] **Task 2: M4 defaults from field tuning** (S, 2 files)
  Freeze the tuned knob values (SCALE / SMOOTH / ACCEL / MAX_VEL /
  GRIP_*) as the argparse defaults instead of env-only overrides.
  *Accept:* `pixi run m3-arm` with NO env vars reproduces the tuned feel;
  spec M4 row records the values and why.
  *Verify:* `pixi run python scripts/test_driver_safety.py` and
  `test_m2_pipeline.py` pass with new defaults.
  *Depends:* Task 1 (values come from that session).

- [ ] **Task 3: M5 soak + hardening review** (M, 1–3 files)
  30-minute continuous teleop soak: watch for bus warnings, servo
  temperature (`Present_Temperature` via a watch_ticks column), memory
  growth, missed deadman events. Dispatch the `safety-reviewer` agent over
  the final driver/IK state as the M5 audit.
  *Accept:* zero unexplained warnings in soak; reviewer verdict SAFE;
  any FIX-FIRST finding resolved. Spec M5 row closed.
  *Verify:* soak log attached to the spec; safety suite green.
  *Depends:* Task 2.

### Checkpoint A — spec complete
- [ ] All milestone rows M0–M5 marked passed with evidence
- [ ] All four test suites green
- [ ] Human sign-off before data collection begins

### Phase 2: Dataset readiness

- [ ] **Task 4: Camera QA at recording rate** (S, 0–1 files)
  With both cams + recorder live, verify sustained fps (target ≥25 each),
  exposure sanity in the actual task lighting, and wrist-cam framing of
  the workspace. Split USB controllers if fps sags (lsusb -t).
  *Accept:* 60 s recording with no "frame skipped" warnings; both streams
  ≥25 fps sustained.
  *Verify:* `ros2 topic hz` on both topics; recorder log clean.
  *Depends:* Checkpoint A.

- [ ] **Task 5: Episode QA loop** (S, no code expected)
  Record 5 throwaway episodes of the chosen task; load the dataset, check
  frame counts, spot-check video content, replay one session mcap in
  Foxglove alongside.
  *Accept:* dataset loads; frames ≈ duration×fps; images show the task;
  actions/states plausible (commanded-vs-actual plot in Foxglove).
  *Verify:* small load script or `test_recorder.py`-style check against
  the real dataset root.
  *Depends:* Task 4.

- [ ] **Task 6: Collection protocol note** (XS, 1 file)
  One page in docs/: task definition (object, start/goal), episode length,
  reset procedure between episodes, `--task` label convention, target
  episode count for the first training run.
  *Accept:* a second person could collect consistent episodes from it.
  *Depends:* Task 5.

### Checkpoint B — ready to collect
- [ ] One full dress-rehearsal episode collected per protocol and QA'd

- [ ] **Task 7: Collect the first real dataset** (M, no code)
  Per protocol: target ~50 episodes of the single chosen task (industry
  floor for a first ACT run; adjust per Open Questions).
  *Accept:* ≥50 clean episodes in one dataset; failure/reset episodes
  discarded per protocol.
  *Verify:* dataset load reports the count; random spot-checks pass.
  *Depends:* Checkpoint B.

### Phase 3: First policy

- [ ] **Task 8: Train ACT baseline** (M, 1–2 files)
  lerobot's ACT trainer on the Task 7 dataset (the `dataset` extra is
  installed; training likely wants the GPU — see Open Questions).
  *Accept:* training completes; loss curve sane; checkpoint saved under
  data/ (gitignored).
  *Verify:* offline rollout on held-out episodes: predicted vs recorded
  actions plot is not degenerate.
  *Depends:* Task 7.

- [ ] **Task 9: Policy node** (M, 2 files + test)
  `scripts/policy_node.py`: loads the checkpoint, subscribes both image
  topics + /joint_states, publishes /joint_command at the policy rate —
  a drop-in replacement for the ik node; driver and all gates unchanged.
  Synthetic test in the test_m2_pipeline style (fake cameras/states →
  commands within limits), ROS_DOMAIN_ID pinned.
  *Accept:* node runs against fakes; commands bounded and smooth.
  *Verify:* new `scripts/test_policy_node.py` green; safety suite green.
  *Depends:* Task 8 (checkpoint exists); can start scaffolding after
  Checkpoint B using a random-weights checkpoint.

- [ ] **Task 10: First autonomous rollout** (S, no code)
  `/preflight`, clear workspace, policy node instead of receiver+ik, low
  MAX_VEL, human on the kill switch. Score N attempts.
  *Accept:* arm attempts the task autonomously without safety incidents;
  success rate recorded (any nonzero rate = milestone; refinement is a
  later plan).
  *Verify:* session mcap recorded for replay; results noted in the spec.
  *Depends:* Tasks 8, 9.

### Checkpoint C — plan complete
- [ ] Autonomous attempt on hardware, recorded and replayable
- [ ] Spec updated with the imitation-learning phase results

## Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Policy commands are jerky/unsafe | High | Driver gates already bound everything (slew, workspace, sigma, deadman); start MAX_VEL low; Task 9's synthetic test runs before hardware |
| Camera fps collapses during collection (USB) | Med | Task 4 QA before any real collection; controller split is the known fix |
| Calibration drift mid-dataset (servo slip) | Med | `/preflight` + a 30 s slider-vs-hand spot check before each collection day; recalibrate = dataset restart, so check first |
| 50 episodes insufficient for ACT | Med | Accept low first-run success; the pipeline is the deliverable — more episodes is a rerun, not new work |
| GPU/VRAM insufficient for training | Med | Resolve in Open Questions before Task 8; lerobot supports smaller ACT configs |

## Open Questions (need the human)

1. **The task**: what does the arm pick, from where, to where? (Defines
   Tasks 6–10; a light rigid object with a clear goal zone suits the
   gripper's 30% torque cap.)
2. **Training hardware**: local GPU (torch cu128 is installed — what VRAM?)
   or cloud? Decides ACT config size in Task 8.
3. **Episode budget**: 50 is the planning default; willing to do 100+ in
   one sitting, or split across days (accepting drift-check overhead)?
