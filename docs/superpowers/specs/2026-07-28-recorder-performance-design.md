# Recorder Save-Path Performance — Design

**Date:** 2026-07-28
**Status:** Implemented; offline verification passed; physical operator check pending

## Goal

Remove the teleoperation stutter observed when an episode is stopped and saved,
without porting the control or data pipeline away from Python.

## Evidence

The operator reports that:

- ZIG SIM PRO is sending at 60 FPS.
- Delay and stutter occur mainly while recording.
- The visible disturbance occurs during `Stop & keep`, not during capture.

Safe measurements on the current Ryzen AI 7 350 machine show:

- Cartesian IK takes about 25 microseconds per tick, or about 0.25% of one CPU
  core at 100 Hz.
- Driver safety kinematics take about 8 microseconds per check, or about 0.08%
  of one core at 100 Hz.
- Two 640x480 JPEG previews at 30 FPS would use about 5% of one core.
- The current non-streaming LeRobot save launches two SVT-AV1 encoders. Each
  selected encoder parallelism 5 in the installed environment, creating a
  short high-contention burst when `save_episode()` runs.

The installed LeRobot 0.6.1 API already supports streaming video encoding and
an encoder thread cap on both `LeRobotDataset.create()` and
`LeRobotDataset.resume()`. A synthetic two-camera episode confirmed that
streaming encoding with `encoder_threads=2` selects parallelism 2 for each
camera encoder and shifts encoding work out of the stop-only burst.

## Design

Keep the existing Python, ROS 2, LeRobot, and AV1 pipeline.

Change the episode recorder so every newly created or resumed dataset uses:

```python
streaming_encoding=True
encoder_threads=2
```

`encoder_threads` is exposed as a recorder CLI option with default `2`, because
encoder contention depends on the deployment CPU. The cockpit-managed recorder
uses the same default.

The codec, pixel format, CRF, preset, frame rate, image size, LeRobot v3 layout,
and episode semantics remain unchanged. Existing raw datasets remain readable
and resumable.

Measure and log elapsed `save_episode()` time with the episode frame count. This
provides a direct check that the stop path remains healthy without adding a
tracing subsystem or changing control messages.

## Data and control flow

Before:

```text
capture frames → buffer images → Stop → burst-encode both cameras → save episode
```

After:

```text
capture frames → bounded streaming encoders → Stop → drain short remainder → save episode
```

The standalone recorder remains a separate ROS node. The cockpit collection
manager owns the same reusable recorder class in-process for managed sessions.
Teleop, IK, driver, camera, replay, and export boundaries do not change.

## Error handling

- Dataset creation and resume continue to fail visibly if LeRobot rejects the
  encoder configuration.
- A streaming encoder error prevents the episode from being reported as saved.
- The recorder logs the failure and retains its existing process-level failure
  behavior; it does not silently mark a partial episode complete.
- The CLI thread count must be a positive integer.
- No automatic codec fallback is added. A fallback could silently change
  storage and decoding characteristics.

## Verification

Automated verification remains offline and uses `ROS_DOMAIN_ID=77` for every
test that initializes ROS.

The existing recorder integration suite must prove:

- A new dataset records and reloads a two-camera episode.
- A second recorder run resumes the dataset and adds another episode.
- ROS start, stop, discard, and status behavior remain intact.
- Frame counts and recorded state/action values remain correct.
- The complete `pixi run test` suite passes.

The save-duration log is observational rather than a flaky timing assertion.
Final validation on the real workflow is:

1. Run teleop, cameras, cockpit, and recording normally.
2. Record an episode.
3. Continue moving under clutch while selecting `Stop & keep`.
4. Confirm the arm no longer pauses or stutters during the save.
5. Reload and replay the saved episode.

## Deferred escalation

Do not add CPU affinity, real-time scheduling, a separate encoding service,
C++, Rust, a custom LeRobot writer, or a codec change in this work.

If the tuned recorder still disturbs teleoperation, first try
`--encoder-threads 1` and capture process/ROS timing evidence. A C++ control
plane is considered only if that evidence shows missed control deadlines
outside the encoder save path.
