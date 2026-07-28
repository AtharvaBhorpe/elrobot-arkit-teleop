# Cockpit Joint-Angle Plot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an eight-series joint-angle plot to the cockpit's shared visual
stage for rolling live state and synchronized episode replay.

**Architecture:** A dependency-free ES module owns a bounded live buffer,
replay trajectory, Canvas 2D rendering, legend visibility, resize handling,
and pointer-to-frame mapping. `app.js` feeds it from the existing 30 Hz
WebSocket and already-loaded `replay.states`; no server or ROS data path
changes.

**Tech Stack:** Vanilla browser ES modules, Canvas 2D, native HTML/CSS,
FastAPI static serving, Node's built-in assertion module for the pure mapping
check, existing Python offline test suite.

## Global Constraints

- Show all eight joints overlaid by default with clickable J1–J8 legend
  buttons.
- Plot radians against seconds.
- Live mode retains the last 15 seconds; replay mode shows the selected
  effective trajectory and current frame.
- Clicking or dragging the replay plot must use the existing visual replay
  scrub path.
- Do not add a backend endpoint, ROS subscription, chart dependency, frontend
  build step, or runtime CDN.
- Missing joint values render as gaps, not zeroes.
- Keep the Y-axis stable at approximately -2 to +2 radians.
- Below the existing 1200 px breakpoint, stack the plot under the URDF.
- Tests that initialize ROS must not use the default DDS domain.
- Do not edit calibration artifacts or `docs/urdf_Elrobot.urdf`.

---

## File map

- Create `src/elrobot/web/static/joint-plot.mjs`: pure frame mapping plus the
  complete Canvas plot component.
- Create `tests/test_joint_plot.mjs`: dependency-free check for pointer
  clamping and frame mapping.
- Modify `src/elrobot/web/static/index.html`: add the lower visual row, plot
  card, canvas, status, and legend containers.
- Modify `src/elrobot/web/static/style.css`: split the lower stage and make it
  responsive.
- Modify `src/elrobot/web/static/app.js`: feed live states, replay data, and
  playhead changes into the plot.
- Modify `tests/test_web_api.py`: verify plot markup and its static module are
  served without caching.
- Modify `docs/web-cockpit-guide.md`: describe live and replay plot behavior.
- Replace `docs/assets/cockpit-teleop-collect.jpg` and
  `docs/assets/cockpit-curate.jpg`: keep the operator guide screenshots
  aligned with the visible cockpit.

---

### Task 1: Dependency-free joint plot component

**Files:**

- Create: `tests/test_joint_plot.mjs`
- Create: `src/elrobot/web/static/joint-plot.mjs`

**Interfaces:**

- Produces:
  `frameFromX(clientX: number, plotLeft: number, plotWidth: number,
  frameCount: number) -> number`
- Produces:
  `makeJointPlot({canvas, legend, status, names, onScrub}) -> {
  pushLive, showLive, showReplay, setReplayFrame, showEmpty}`
- `showReplay(states, stateNames, fps, frame, label)` consumes the trajectory
  shape already returned by the curation states endpoint.

- [ ] **Step 1: Write the failing pure mapping check**

```javascript
// tests/test_joint_plot.mjs
import assert from "node:assert/strict";
import { frameFromX } from "../src/elrobot/web/static/joint-plot.mjs";

assert.equal(frameFromX(50, 50, 100, 11), 0);
assert.equal(frameFromX(100, 50, 100, 11), 5);
assert.equal(frameFromX(150, 50, 100, 11), 10);
assert.equal(frameFromX(-20, 50, 100, 11), 0);
assert.equal(frameFromX(900, 50, 100, 11), 10);
assert.equal(frameFromX(100, 50, 0, 11), 0);
assert.equal(frameFromX(100, 50, 100, 0), 0);
console.log("joint plot mapping: ok");
```

- [ ] **Step 2: Run the check and verify it fails**

Run:

```bash
node tests/test_joint_plot.mjs
```

Expected: FAIL with `ERR_MODULE_NOT_FOUND` for `joint-plot.mjs`.

- [ ] **Step 3: Implement the pure mapping and minimal plot component**

Create `src/elrobot/web/static/joint-plot.mjs` with these constants and public
surface:

```javascript
const COLORS = [
  "#2563eb", "#dc2626", "#16a34a", "#9333ea",
  "#ea580c", "#0891b2", "#ca8a04", "#db2777",
];
const HISTORY_SECONDS = 15;
const Y_MIN = -2;
const Y_MAX = 2.05;
const PAD = { left: 38, right: 12, top: 14, bottom: 24 };

export function frameFromX(clientX, plotLeft, plotWidth, frameCount) {
  if (plotWidth <= 0 || frameCount <= 1) return 0;
  const fraction = Math.max(
    0, Math.min(1, (clientX - plotLeft) / plotWidth));
  return Math.round(fraction * (frameCount - 1));
}

export function makeJointPlot({
  canvas, legend, status, names, onScrub,
}) {
  const ctx = canvas.getContext("2d");
  const shown = names.map(() => true);
  let mode = "empty";
  let emptyText = "Waiting for joint states";
  let live = [];
  let states = [];
  let stateNames = [];
  let fps = 30;
  let frame = 0;
  let queued = false;
  let dragging = false;

  function queueDraw() {
    if (queued) return;
    queued = true;
    requestAnimationFrame(draw);
  }

  function color(token, fallback) {
    return getComputedStyle(document.documentElement)
      .getPropertyValue(token).trim() || fallback;
  }

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    const width = Math.round(rect.width * ratio);
    const height = Math.round(rect.height * ratio);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    return { width: rect.width, height: rect.height };
  }

  function source() {
    if (mode === "empty") {
      return { count: 0, seconds: 0, x() { return 0; },
        value() { return NaN; } };
    }
    if (mode === "replay") {
      return {
        count: states.length,
        seconds: states.length > 1 ? (states.length - 1) / fps : 0,
        x(i) {
          return states.length > 1 ? i / (states.length - 1) : 0;
        },
        value(i, joint) {
          const column = stateNames.indexOf(joint);
          return column < 0 ? NaN : Number(states[i]?.[column]);
        },
      };
    }
    const end = live.at(-1)?.time ?? 0;
    const start = Math.max(live[0]?.time ?? 0, end - HISTORY_SECONDS);
    const visible = live.filter((sample) => sample.time >= start);
    return {
      count: visible.length,
      seconds: Math.max(0, end - start),
      x(i) {
        return end > start ? (visible[i].time - start) / (end - start) : 0;
      },
      value(i, joint) {
        return Number(visible[i]?.joints?.[joint]);
      },
    };
  }
```

Complete `draw()` using this exact rendering policy:

1. Call `resize()` and `clearRect`; return if either CSS dimension is zero.
2. Use `PAD` to define the plotting rectangle.
3. Draw five horizontal grid lines labelled `-2`, `-1`, `0`, `1`, `2`
   using `--border` and `--muted-foreground`.
4. Draw X labels at `0`, 50%, and 100% using the source duration in seconds.
5. For each visible joint, draw a two-pixel line using `COLORS[index]`.
   Map X with `source.x(i)` and Y with
   `(Y_MAX - value) / (Y_MAX - Y_MIN)`. Use `moveTo` after every non-finite
   value so missing values create gaps.
6. Set the sampling step to
   `Math.max(1, Math.ceil(source.count / Math.max(1, plotWidth * 2)))`
   so long episodes do not draw more than roughly two samples per pixel.
7. In replay mode, draw a one-pixel foreground-colored vertical playhead at
   `frame / Math.max(1, states.length - 1)`.
8. If there is no source data, center `emptyText` in the plotting rectangle.

Build one native legend button per joint:

```javascript
names.forEach((name, index) => {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "plot-legend-item";
  button.textContent = `J${index + 1}`;
  button.title = name;
  button.setAttribute("aria-label", `${name}; toggle plotted series`);
  button.setAttribute("aria-pressed", "true");
  button.style.setProperty("--joint-color", COLORS[index]);
  button.addEventListener("click", () => {
    shown[index] = !shown[index];
    button.setAttribute("aria-pressed", String(shown[index]));
    queueDraw();
  });
  legend.appendChild(button);
});
```

Map pointer input only while replay is visible:

```javascript
function scrub(event) {
  if (mode !== "replay" || !states.length) return;
  const rect = canvas.getBoundingClientRect();
  onScrub(frameFromX(
    event.clientX, rect.left + PAD.left,
    rect.width - PAD.left - PAD.right, states.length));
}
canvas.addEventListener("pointerdown", (event) => {
  if (mode !== "replay") return;
  dragging = true;
  canvas.setPointerCapture(event.pointerId);
  scrub(event);
});
canvas.addEventListener("pointermove", (event) => {
  if (dragging) scrub(event);
});
canvas.addEventListener("pointerup", (event) => {
  dragging = false;
  canvas.releasePointerCapture(event.pointerId);
});
```

Return these methods:

```javascript
new ResizeObserver(queueDraw).observe(canvas);

return {
  pushLive(joints, time = performance.now() / 1000) {
    live.push({ time, joints: { ...joints } });
    const cutoff = time - HISTORY_SECONDS;
    while (live.length && live[0].time < cutoff) live.shift();
    if (mode === "live") queueDraw();
  },
  showLive() {
    mode = "live";
    emptyText = "Waiting for joint states";
    status.textContent = "Live · last 15 s";
    canvas.setAttribute("aria-label",
      "Live joint angles for the last 15 seconds, in radians");
    queueDraw();
  },
  showReplay(nextStates, nextNames, nextFps, nextFrame = 0,
             label = "Replay") {
    mode = "replay";
    states = Array.isArray(nextStates) ? nextStates : [];
    stateNames = Array.isArray(nextNames) ? nextNames : [];
    fps = Number(nextFps) > 0 ? Number(nextFps) : 30;
    frame = nextFrame;
    emptyText = "No joint trajectory";
    status.textContent =
      `${label} · ${(states.length / fps).toFixed(1)} s`;
    canvas.setAttribute("aria-label",
      `${label} joint-angle trajectory in radians`);
    queueDraw();
  },
  setReplayFrame(nextFrame) {
    frame = Math.max(0, Math.min(
      Number(nextFrame) || 0, Math.max(0, states.length - 1)));
    if (mode === "replay") queueDraw();
  },
  showEmpty(text) {
    mode = "empty";
    emptyText = text;
    status.textContent = text;
    canvas.setAttribute("aria-label", text);
    queueDraw();
  },
};
}
```

- [ ] **Step 4: Run the focused check**

Run:

```bash
node tests/test_joint_plot.mjs
```

Expected: `joint plot mapping: ok`.

- [ ] **Step 5: Commit the component**

```bash
git add src/elrobot/web/static/joint-plot.mjs tests/test_joint_plot.mjs
git commit -m "feat: add joint angle plot component"
```

---

### Task 2: Integrate live and replay visualization

**Files:**

- Modify: `tests/test_web_api.py:243-270`
- Modify: `src/elrobot/web/static/index.html:55-74`
- Modify: `src/elrobot/web/static/style.css:205-260`
- Modify: `src/elrobot/web/static/app.js:1-25,79-120,365-380,797-860`

**Interfaces:**

- Consumes `makeJointPlot()` and its five returned methods from Task 1.
- Consumes existing `m.joints`, `replay.states`, `replay.names`, `replay.fps`,
  `stopReplay()`, and `showFrame(frame)`.
- Produces no new server API or ROS behavior.

- [ ] **Step 1: Extend the static UI test first**

In `test_static_urdf_and_meshes_served()`, extend the existing `element_id`
tuple:

```python
        "joint-plot", "joint-plot-status", "joint-plot-legend",
```

Then add:

```python
    plot_module = c.get("/static/joint-plot.mjs")
    assert plot_module.status_code == 200
    assert plot_module.headers["cache-control"] == "no-store"
    assert "export function makeJointPlot" in plot_module.text
```

- [ ] **Step 2: Run the web test and verify the markup assertion fails**

Run:

```bash
ROS_DOMAIN_ID=77 ROS_LOG_DIR=/tmp/elrobot-test-ros-log \
HF_DATASETS_CACHE=/tmp/elrobot-tests-hf-cache \
pixi run python tests/test_web_api.py
```

Expected: FAIL because `id="joint-plot"` is absent.

- [ ] **Step 3: Add the lower visual row and plot card**

Replace the standalone viewer card in `index.html` with:

```html
      <div class="visual-row">
        <div class="card viewer">
          <div id="scene3d"></div>
        </div>
        <div class="card joint-plot-card">
          <div class="plot-heading">
            <strong>Joint angles</strong>
            <span id="joint-plot-status" class="muted">Waiting for joint states</span>
          </div>
          <canvas id="joint-plot"
                  aria-label="Waiting for joint states"></canvas>
          <div id="joint-plot-legend" class="plot-legend"
               aria-label="Visible joints"></div>
        </div>
      </div>
```

- [ ] **Step 4: Add the minimal responsive CSS**

Add:

```css
.visual-row {
  flex: 1;
  min-height: 0;
  display: grid;
  grid-template-columns: minmax(0, 3fr) minmax(300px, 2fr);
  gap: 10px;
}

.joint-plot-card {
  display: flex;
  flex-direction: column;
  min-height: 0;
  padding: 12px;
}

.plot-heading {
  display: flex;
  justify-content: space-between;
  gap: 12px;
  align-items: baseline;
}

#joint-plot {
  flex: 1;
  min-height: 180px;
  height: auto !important;
  touch-action: none;
}

.plot-legend {
  display: flex;
  flex-wrap: wrap;
  gap: 4px;
  justify-content: center;
}

.plot-legend-item {
  padding: 3px 7px;
  color: var(--muted-foreground);
  border-color: var(--joint-color);
  box-shadow: none;
  font-size: 11px;
}

.plot-legend-item[aria-pressed="true"] {
  color: var(--foreground);
  border-bottom: 3px solid var(--joint-color);
}

.plot-legend-item[aria-pressed="false"] {
  opacity: 0.4;
}
```

Inside the existing `@media (max-width: 1200px)` block add:

```css
  .visual-row {
    grid-template-columns: 1fr;
  }
  .joint-plot-card {
    min-height: 300px;
  }
```

Keep the existing `.viewer.card { min-height: 380px; }` rule.

- [ ] **Step 5: Connect the component to existing state**

At the top of `app.js`:

```javascript
import { makeJointPlot } from "/static/joint-plot.mjs";
```

After `NAMES`, construct it:

```javascript
const jointPlot = makeJointPlot({
  canvas: document.getElementById("joint-plot"),
  legend: document.getElementById("joint-plot-legend"),
  status: document.getElementById("joint-plot-status"),
  names: NAMES,
  onScrub(frame) {
    if (!replay.states || !selectedEpisode) return;
    stopReplay();
    showFrame(frame);
  },
});
jointPlot.showLive();
```

In the WebSocket state handler, before the replay check:

```javascript
    jointPlot.pushLive(m.joints);
```

In `selectMode(mode)`, preserve its safety calls, then set the visualization:

```javascript
  if (mode === "teleop") {
    jointPlot.showLive();
  } else {
    jointPlot.showEmpty("Select an episode");
  }
```

After `await refreshCurate()` in the Curate branch, restore a previously
selected episode:

```javascript
    if (selectedEpisode) await loadCuratedReplay();
```

After assigning replay state in `loadCuratedReplay()`:

```javascript
    jointPlot.showReplay(
      replay.states, replay.names, replay.fps, 0,
      `${selectedEpisode.session_name || "Session"} · #${selectedEpisode.source_index}`,
    );
```

In `showFrame()`, immediately after setting `replay.frame`:

```javascript
  jointPlot.setReplayFrame(i);
```

In the replay load error path:

```javascript
    jointPlot.showEmpty(error.message);
```

- [ ] **Step 6: Run focused checks**

Run:

```bash
node tests/test_joint_plot.mjs
```

Expected: PASS.

Run:

```bash
ROS_DOMAIN_ID=77 ROS_LOG_DIR=/tmp/elrobot-test-ros-log \
HF_DATASETS_CACHE=/tmp/elrobot-tests-hf-cache \
pixi run python tests/test_web_api.py
```

Expected: all web API checks pass.

- [ ] **Step 7: Commit the integration**

```bash
git add src/elrobot/web/static/index.html \
  src/elrobot/web/static/style.css \
  src/elrobot/web/static/app.js \
  tests/test_web_api.py
git commit -m "feat: show live and replay joint trajectories"
```

---

### Task 3: Operator documentation and final verification

**Files:**

- Modify: `docs/web-cockpit-guide.md:103-130,250-263`
- Replace: `docs/assets/cockpit-teleop-collect.jpg`
- Replace: `docs/assets/cockpit-curate.jpg`

**Interfaces:**

- Documents the behavior completed by Tasks 1 and 2.
- Changes no runtime interface.

- [ ] **Step 1: Update the layout and replay guide**

Change the layout bullets and shared-stage paragraph to state that the lower
stage includes the 3D model and joint plot. Add:

```markdown
The joint plot overlays J1–J8 in radians. In Teleop it retains the latest
15 seconds of `/joint_states`; click a J1–J8 legend button to hide or restore
that line.
```

Append to **Visual replay**:

```markdown
For a selected episode, the same plot shows the complete effective
joint-angle trajectory and a moving playhead. Clicking or dragging in the
plot scrubs the plot, 3D model, cameras, joint readouts, and replay timeline
together.
```

- [ ] **Step 2: Capture updated cockpit screenshots**

With the existing cockpit server on `http://localhost:8080/`, use the
synthetic/offline documentation state, not live hardware. Capture:

1. Teleop → Collect with the live plot and legend visible.
2. Curate with one episode selected and the replay trajectory/playhead
   visible.

Replace the two existing JPEGs at their current paths so no guide links need
to change. Do not start a second server on port 8080.

- [ ] **Step 3: Verify documentation assets**

Run:

```bash
test -s docs/assets/cockpit-teleop-collect.jpg
test -s docs/assets/cockpit-curate.jpg
rg -n "latest 15 seconds|complete effective" docs/web-cockpit-guide.md
```

Expected: both images are non-empty and both new guide passages are found.

- [ ] **Step 4: Run the full offline verification**

Run:

```bash
ROS_DOMAIN_ID=77 ROS_LOG_DIR=/tmp/elrobot-test-ros-log \
HF_DATASETS_CACHE=/tmp/elrobot-tests-hf-cache pixi run test
```

Expected: every offline suite passes.

Run:

```bash
pixi run lint
```

Expected: Ruff exits successfully.

Run:

```bash
git diff --check
```

Expected: no whitespace errors.

- [ ] **Step 5: Perform browser acceptance checks**

At `http://localhost:8080/` verify:

1. Teleop renders eight colored lines and a stable radian axis.
2. Clicking J1 and J8 independently hides and restores their lines.
3. Live history continues while Curate is open and is present on return.
4. Selecting an episode displays its complete effective trajectory.
5. Play, Reset, the range slider, and plot click/drag keep the playhead,
   URDF, camera frames, joint readouts, and replay status synchronized.
6. Raw/effective switching and trim application replace the trajectory.
7. At a viewport below 1200 px, the plot stacks below the URDF.
8. Browser console contains no new errors.

- [ ] **Step 6: Commit docs and verification artifacts**

```bash
git add docs/web-cockpit-guide.md \
  docs/assets/cockpit-teleop-collect.jpg \
  docs/assets/cockpit-curate.jpg
git commit -m "docs: show cockpit joint trajectory plot"
```
