# Recorder Save-Path Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the episode stop/save CPU burst by using LeRobot streaming encoding with bounded encoder parallelism.

**Architecture:** Keep the existing Python recorder and LeRobot v3 writer. Pass one shared set of writer options to both dataset creation and resume, expose only the encoder thread count as a tuning knob, and time the synchronous save for operational evidence.

**Tech Stack:** Python 3.12, ROS 2 Jazzy `rclpy`, LeRobot 0.6.1, PyAV/SVT-AV1, existing script-style offline tests.

## Global Constraints

- Keep Python, ROS 2, the default AV1 codec, pixel format, CRF, preset, FPS, image size, and LeRobot v3 layout unchanged.
- Set `streaming_encoding=True` for both `LeRobotDataset.create()` and `LeRobotDataset.resume()`.
- Default `encoder_threads` to exactly `2`; accept only positive integers.
- Do not add CPU affinity, real-time scheduling, a separate encoding service, C++, Rust, or a custom writer.
- Any test that initializes ROS must set `ROS_DOMAIN_ID=77` before importing `rclpy`.
- Do not modify the driver, IK, calibration artifacts, or authoritative URDF.
- Preserve all unrelated dirty-worktree changes.

---

### Task 1: Stream and bound episode encoding

**Files:**
- Modify: `src/elrobot/nodes/episode_recorder.py:47-268`
- Modify: `tests/test_recorder.py:1-245`
- Modify: `docs/web-cockpit-guide.md:139-164`

**Interfaces:**
- Consumes: the `streaming_encoding` and `encoder_threads` keyword arguments
  accepted by `LeRobotDataset.create()` and `LeRobotDataset.resume()`.
- Produces: `positive_int(value: str) -> int`, `Recorder._writer_options() -> dict`, CLI option `--encoder-threads`, and an `episode saved (<frames> frames, <seconds>s)` log line.

- [ ] **Step 1: Add a failing writer-configuration test**

Add this import and test to `tests/test_recorder.py`:

```python
from elrobot.nodes.episode_recorder import positive_int


def test_encoder_configuration():
    assert positive_int("1") == 1
    assert positive_int("2") == 2
    for bad in ("0", "-1"):
        try:
            positive_int(bad)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError(f"accepted invalid encoder thread count {bad}")
```

In `test_record_cmd_topic()`, extend the in-process namespace and assert the real LeRobot writer was created in streaming mode:

```python
args = argparse.Namespace(
    fps=30.0,
    repo_id="local/elrobot_teleop",
    root=str(root),
    task="test",
    encoder_threads=2,
)
node = Recorder(args)

# After `assert node.recording`:
assert node.dataset._encoder_threads == 2
assert node.dataset.writer._streaming_encoder is not None
```

Add `test_encoder_configuration()` to `main()` before the ROS scenarios.

- [ ] **Step 2: Run the recorder test and verify the new test fails**

Run:

```bash
pixi run python tests/test_recorder.py
```

Expected: import failure because `positive_int` does not exist, or the streaming-writer assertions fail because the current recorder uses non-streaming defaults.

- [ ] **Step 3: Add the shared writer options and CLI validation**

Add this function near the recorder constants:

```python
def positive_int(value: str) -> int:
    n = int(value)
    if n < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return n
```

Add this method to `Recorder`:

```python
def _writer_options(self) -> dict:
    return {
        "video_backend": "pyav",
        "streaming_encoding": True,
        "encoder_threads": self.args.encoder_threads,
    }
```

Use the same options in both branches of `_ensure_dataset()`:

```python
writer_options = self._writer_options()
if Path(self.args.root).exists():
    self.dataset = LeRobotDataset.resume(
        repo_id=self.args.repo_id,
        root=self.args.root,
        **writer_options,
    )
else:
    self.dataset = LeRobotDataset.create(
        repo_id=self.args.repo_id,
        fps=int(self.args.fps),
        features=feats,
        root=self.args.root,
        robot_type="elrobot",
        **writer_options,
    )
```

Add the CLI option:

```python
p.add_argument(
    "--encoder-threads",
    type=positive_int,
    default=2,
    help="LeRobot video encoder threads (default: 2; lower if save disturbs teleop)",
)
```

Do not add a codec option or override `rgb_encoder`.

- [ ] **Step 4: Log save duration without adding a timing assertion**

Replace the save portion of `Recorder.stop()` with:

```python
frames = self.n_frames
started = time.perf_counter()
self.dataset.save_episode()
elapsed = time.perf_counter() - started
self.episodes += 1
self.get_logger().info(
    f"episode saved ({frames} frames, {elapsed:.2f}s)")
```

Keep `recording = False` before the save and keep the zero-frame behavior unchanged.

- [ ] **Step 5: Run recorder and resume verification**

Run:

```bash
pixi run python tests/test_recorder.py
```

Expected: `RECORDER TEST PASSED`; the subprocess creates a dataset, the second run resumes it, the in-process writer exposes `_encoder_threads == 2`, and `_streaming_encoder` is non-null.

- [ ] **Step 6: Update the operator guide**

In `docs/web-cockpit-guide.md`, explain:

```markdown
Recording encodes both camera streams continuously with two encoder threads by
default, so Stop & keep only drains the remaining frames instead of launching a
large encoding burst. If saving still disturbs teleoperation on a smaller
computer, run the recorder with `--encoder-threads 1`.
```

Keep the current command and LeRobot v3 description unchanged.

- [ ] **Step 7: Run lint and the complete offline suite**

Run:

```bash
pixi run lint
pixi run test
```

Expected: both commands exit zero. No test may touch the default DDS domain.

- [ ] **Step 8: Commit the prerequisite**

```bash
git add src/elrobot/nodes/episode_recorder.py tests/test_recorder.py docs/web-cockpit-guide.md
git commit -m "perf: stream recorder video encoding"
```

Do not stage the existing Web UI changes or untracked `.agents/`, `.codex/`, or `.superpowers/` directories.
