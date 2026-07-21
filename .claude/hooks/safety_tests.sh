#!/usr/bin/env bash
# PostToolUse hook: any edit to the safety-critical files re-runs the
# driver safety suite (stub bus, ~5 s, no hardware). Exit 2 feeds the
# failure back to Claude so the regression is fixed before anything else.
set -u
path=$(python3 -c "import json,sys; print(json.load(sys.stdin).get('tool_input',{}).get('file_path',''))")
case "$path" in
  */elrobot_driver.py|*/cartesian_ik.py) ;;
  *) exit 0 ;;
esac
cd "$(dirname "$0")/../.." || exit 0
out=$(pixi run python tests/test_driver_safety.py 2>&1)
if ! grep -q "ALL DRIVER SAFETY TESTS PASSED" <<<"$out"; then
  echo "SAFETY SUITE FAILED after editing $path:" >&2
  tail -15 <<<"$out" >&2
  exit 2
fi
exit 0
