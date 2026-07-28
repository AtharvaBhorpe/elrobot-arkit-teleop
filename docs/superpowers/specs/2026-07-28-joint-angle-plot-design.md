# Cockpit Joint-Angle Plot — Design

**Date:** 2026-07-28
**Status:** Approved; implementation pending

## Goal

Add a joint-angle time-series beside the cockpit's cameras and 3D URDF so an
operator can see motion across all eight joints during live teleoperation and
episode review.

## Decisions

- Show all eight joints overlaid by default.
- Provide a compact J1–J8 legend; clicking a legend item hides or shows that
  series.
- Use the joint data already present in the browser. Do not add a backend
  endpoint, ROS subscription, chart dependency, or build step.
- Plot radians against seconds.

## Layout

The two camera cards remain side by side across the top of the shared visual
stage. The lower region becomes a two-column row containing the existing 3D
URDF and the joint plot. The URDF receives slightly more width than the plot.

Below the existing narrow-screen breakpoint, the lower row stacks the plot
under the URDF so neither visualization is crushed.

The plot card contains:

- a native `<canvas>` for axes, grid, trajectories, and replay playhead;
- a compact row of eight native buttons used as the legend; and
- a short mode label indicating `Live · last 15 s` or the selected replay.

## Data flow and behavior

### Live teleoperation

Every WebSocket state message already contains the latest `/joint_states`
values. The plot retains a rolling 15-second browser-side history while the
page is connected. Missing joints remain gaps rather than invented zeroes.

The live history continues to update while curation replay owns the visible
stage, so returning to Teleop restores recent context immediately.

### Curation replay

The existing states request loads the selected episode's complete effective
trajectory into `replay.states`, with joint names and FPS. The plot renders
that array directly and draws a vertical playhead at `replay.frame`.

Playing, resetting, or using the existing range slider updates the playhead.
Clicking or dragging horizontally in the plot maps time to a frame and routes
through the existing replay scrub path, keeping the plot, URDF, cameras,
joint readouts, and timeline synchronized.

Changing episode, trim, or raw/effective view replaces the trajectory through
the existing replay reload. Leaving Curate returns the plot to the rolling
live history.

## Rendering

Use the browser Canvas 2D API in one small ES module. Eight stable,
high-contrast colors map to J1–J8. The shared Y-axis is fixed at -2 to +2
radians, with a small allowance for the gripper's 2-radian endpoint, so scale
does not jump as the arm moves. The X-axis is elapsed seconds.

Canvas resolution follows its CSS size and `devicePixelRatio`. Rendering is
requested when data, selection, playhead, visibility, or element size changes;
multiple requests within one animation frame are coalesced. The live buffer is
bounded, and replay uses the already-loaded trajectory, so memory and CPU work
remain small.

Legend buttons expose pressed state and do not rely on color alone: each is
labelled J1 through J8 and has the full ROS joint name as its accessible name
and tooltip. The canvas has an accessible text label describing its current
mode.

## Error and empty states

- Before live states arrive, show `Waiting for joint states`.
- In Curate with no episode selected, show `Select an episode`.
- An empty or malformed trajectory leaves the plot empty without affecting
  the cameras, URDF, replay controls, or WebSocket.
- A zero-sized canvas is not rendered until layout supplies dimensions.

## Verification

Keep the check proportional to this client-only feature:

- Extend the web static-asset test to prove the plot markup and module are
  served.
- Test the small pure frame-from-X mapping used for plot scrubbing.
- Run the existing web API suite, complete offline test suite, and lint.
- In the cockpit, verify live history, legend toggles, replay playhead,
  click/drag scrubbing, episode/trim changes, and the narrow layout.

No automated test may use the default DDS domain.

## Deferred

Do not add zooming, panning, exports, per-joint axes, configurable history
length, persistent legend preferences, velocity/effort series, or a charting
library. Add one only when an operator workflow demonstrates the need.
