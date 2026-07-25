---
name: recalibrate
description: Guided full recalibration of the Elrobot arm (M1a servo homing + M1b URDF-tick table + sign verification). Use after a servo swap, mechanical work, or when FK stops matching reality. Writes servo EEPROM - user runs each step at the arm.
disable-model-invocation: true
---

Guide the user through recalibration. Every lesson below was earned by a
real incident (see the spec's Calibration section + Incident notes). The
user runs the interactive scripts in their own terminal; you interpret
outputs and gate progression.

## Before starting — say this plainly

- M1a writes homing offsets to servo EEPROM and **invalidates the existing
  M1b table**. There is no partial redo: M1a means M1b + verification again.
- Torque will be off throughout: the arm is limp and will sag. Rest it low.
- Have a REAL tape measure. "iPhone units" wrongly passed a flipped sign once.

## Step 1 — M1a: `pixi run python -m elrobot.calibration.m1a_calibrate`

- Park pose needs no precision (±79° tolerance on swept joints): any relaxed
  pose with no joint within ~20° of a hard stop.
- Joints 5 and 7 are excluded from the sweep (near-full-turn) and assigned
  0..4095 — correct, the control path never uses lerobot normalization.
- Gate: every swept joint within ±20% of its URDF span. Typical good result
  is −1..−3% (hand sweeps stop shy of the stops).
- Gripper: park it FULLY CLOSED first, then sweep to fully open.

## Step 2 — M1b: `pixi run python -m elrobot.calibration.m1b_reconcile`

- Offsets come from range midpoints; joints 5/7 get their ranges here via
  the encoder-unwrapping sweep (expect ~336°/340°).
- The sign prompts are FK-derived from the NEUTRAL pose. **They are
  pose-dependent** — this recorded 2 of 7 signs flipped once. Treat Step 3
  as the real authority, not this step.

## Step 3 — Sign verification (authoritative, ~30 s per joint)

Two terminals: `pixi run view` (sliders) + `pixi run ticks`
(live joint monitor). For EVERY joint 1–7:
- Move the slider positive → note the model's motion direction.
- Move the real joint by hand so its q INCREASES on the monitor → the real
  motion must match the model's. Opposite = flip that joint's `sign` in
  `calibration/urdf_ticks.json` (offset unchanged for symmetric-limit
  joints; recompute via the m1b script otherwise).
- Rolls (5/7): compare spin direction viewed from the base along the arm.
  Gentle — near-full-turn joints, and motor 8 has an Overload history.

## Step 4 — FK gate: `pixi run python -m elrobot.calibration.verify_table`

- Pose must be CLEARLY BENT (near neutral every sign error is invisible —
  the script's sensitivity column must say "yes" for the joints you care
  about). Measure height AND horizontal distance with the tape; remember the
  ~3 cm base pedestal. Agreement within a few cm closes calibration.

## After

Commit `calibration/*.json` with a message recording what changed and why.
The .claude hook blocks Claude from editing these files - the user commits.
