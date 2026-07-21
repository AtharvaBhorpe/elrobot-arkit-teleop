# TODO — Milestone Closure → First Trained Policy

See tasks/plan.md for acceptance criteria, verification, and risks.

## Phase 1: Close the spec milestones
- [ ] 1. Formal M3 gate session (pick-carry-place in all 3 DoF modes)   [S]
- [ ] 2. Freeze M4 defaults from field tuning                            [S]
- [ ] 3. M5 soak test + safety-reviewer audit                            [M]
- [ ] **Checkpoint A**: spec M0–M5 all passed, suites green, human sign-off

## Phase 2: Dataset readiness
- [ ] 4. Camera QA at recording rate (≥25 fps sustained, framing)        [S]
- [ ] 5. Episode QA loop (5 throwaway episodes, load + replay checks)    [S]
- [ ] 6. Collection protocol note in docs/                               [XS]
- [ ] **Checkpoint B**: dress-rehearsal episode collected and QA'd
- [ ] 7. Collect first real dataset (~50 episodes, one task)             [M]

## Phase 3: First policy
- [ ] 8. Train ACT baseline on the dataset                               [M]
- [ ] 9. policy_node.py + synthetic test (drop-in for receiver+ik)       [M]
- [ ] 10. First autonomous rollout (low MAX_VEL, kill switch, mcap on)   [S]
- [ ] **Checkpoint C**: autonomous attempt recorded; spec updated

## Open questions blocking start
- [ ] Pick task definition (object / start / goal)      → gates Tasks 6–10
- [ ] Training hardware (local GPU VRAM? cloud?)        → gates Task 8
- [ ] Episode budget (50 default / 100+ / multi-day)    → gates Task 7
