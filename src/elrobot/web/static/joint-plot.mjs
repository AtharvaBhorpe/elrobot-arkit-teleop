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

function number(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : NaN;
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
      return {
        count: 0, seconds: 0,
        x() { return 0; },
        value() { return NaN; },
      };
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
          return column < 0 ? NaN : number(states[i]?.[column]);
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
        return number(visible[i]?.joints?.[joint]);
      },
    };
  }

  function drawEmpty(text, left, top, width, height, foreground) {
    ctx.fillStyle = foreground;
    ctx.font = "12px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, left + width / 2, top + height / 2);
  }

  function draw() {
    queued = false;
    const { width, height } = resize();
    if (width <= 0 || height <= 0) return;
    ctx.clearRect(0, 0, width, height);

    const left = PAD.left;
    const top = PAD.top;
    const plotWidth = Math.max(1, width - PAD.left - PAD.right);
    const plotHeight = Math.max(1, height - PAD.top - PAD.bottom);
    const grid = color("--border", "#d4d4d8");
    const muted = color("--muted-foreground", "#71717a");
    const foreground = color("--foreground", "#18181b");
    const data = source();
    const y = (value) =>
      top + ((Y_MAX - value) / (Y_MAX - Y_MIN)) * plotHeight;

    ctx.font = "10px system-ui, sans-serif";
    ctx.lineWidth = 1;
    ctx.textBaseline = "middle";
    for (const value of [-2, -1, 0, 1, 2]) {
      const py = y(value);
      ctx.strokeStyle = grid;
      ctx.beginPath();
      ctx.moveTo(left, py);
      ctx.lineTo(left + plotWidth, py);
      ctx.stroke();
      ctx.fillStyle = muted;
      ctx.textAlign = "right";
      ctx.fillText(String(value), left - 6, py);
    }

    ctx.textBaseline = "top";
    for (const fraction of [0, 0.5, 1]) {
      const seconds = data.seconds * fraction;
      ctx.fillStyle = muted;
      ctx.textAlign = fraction === 0 ? "left"
        : fraction === 1 ? "right" : "center";
      ctx.fillText(
        `${seconds.toFixed(data.seconds >= 10 ? 0 : 1)}s`,
        left + plotWidth * fraction, top + plotHeight + 7);
    }

    let drewValue = false;
    const step = Math.max(
      1, Math.ceil(data.count / Math.max(1, plotWidth * 2)));
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, top, plotWidth, plotHeight);
    ctx.clip();
    names.forEach((joint, jointIndex) => {
      if (!shown[jointIndex]) return;
      let drawing = false;
      ctx.beginPath();
      const point = (i) => {
        const value = data.value(i, joint);
        if (!Number.isFinite(value)) {
          drawing = false;
          return;
        }
        const px = left + data.x(i) * plotWidth;
        const py = y(value);
        if (drawing) ctx.lineTo(px, py);
        else ctx.moveTo(px, py);
        drawing = true;
        drewValue = true;
      };
      for (let i = 0; i < data.count; i += step) point(i);
      if (data.count > 1 && (data.count - 1) % step) point(data.count - 1);
      ctx.strokeStyle = COLORS[jointIndex];
      ctx.lineWidth = 2;
      ctx.lineJoin = "round";
      ctx.stroke();
    });

    if (mode === "replay" && states.length) {
      const fraction = frame / Math.max(1, states.length - 1);
      const px = left + fraction * plotWidth;
      ctx.strokeStyle = foreground;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(px, top);
      ctx.lineTo(px, top + plotHeight);
      ctx.stroke();
    }
    ctx.restore();

    if (!drewValue) {
      drawEmpty(emptyText, left, top, plotWidth, plotHeight, muted);
    }
  }

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
  const endDrag = (event) => {
    dragging = false;
    if (canvas.hasPointerCapture(event.pointerId))
      canvas.releasePointerCapture(event.pointerId);
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("lostpointercapture", () => { dragging = false; });
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
      canvas.setAttribute(
        "aria-label",
        "Live joint angles for the last 15 seconds, in radians");
      queueDraw();
    },
    showReplay(nextStates, nextNames, nextFps, nextFrame = 0,
               label = "Replay") {
      mode = "replay";
      states = Array.isArray(nextStates) ? nextStates : [];
      stateNames = Array.isArray(nextNames) ? nextNames : [];
      fps = Number(nextFps) > 0 ? Number(nextFps) : 30;
      frame = Math.max(0, Math.min(
        Number(nextFrame) || 0, Math.max(0, states.length - 1)));
      emptyText = "No joint trajectory";
      status.textContent =
        `${label} · ${(states.length / fps).toFixed(1)} s`;
      canvas.setAttribute(
        "aria-label", `${label} joint-angle trajectory in radians`);
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
