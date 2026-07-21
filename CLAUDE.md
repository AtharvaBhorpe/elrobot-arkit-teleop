@AGENTS.md

## Claude-specific notes

- Commits: plain messages, no attribution trailers.
- Hooks in `.claude/settings.json` block edits to calibration artifacts and
  auto-run the safety suite after driver/IK edits — a blocked edit or a
  failing hook is the guardrail working, not an obstacle to route around.
- `/preflight` before any hardware session; `/recalibrate` is the guided
  servo recalibration procedure (user-invoked only — it writes EEPROM).
- The `safety-reviewer` agent reviews any diff touching
  `scripts/elrobot_driver.py` or `scripts/cartesian_ik.py`.
- The arm is on a desk. When in doubt about a powered action, ask; the cost
  asymmetry is a real collision vs. one question.
