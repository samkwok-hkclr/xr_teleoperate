#!/usr/bin/env bash
#
# Simulate pressing A button(s) continuously for HOLD_SEC seconds,
# then release. The arm(s) will be ENABLED while held (if HOLD_SEC
# >= a_button_hold_duration, default 0.5s) and DISABLED on release.
#
# Usage:
#   ./script/sim_a_button.sh                    # both, 0.7s @ 20Hz
#   ./script/sim_a_button.sh left               # left only, 0.7s
#   ./script/sim_a_button.sh right 0.3          # right only, 0.3s (below threshold)
#   ./script/sim_a_button.sh left 1.5           # left, 1.5s (stays enabled for 1.5s)
#   RATE_HZ=50 ./script/sim_a_button.sh both    # faster publish rate
#
set -euo pipefail

SIDE="${1:-both}"
HOLD_SEC="${2:-0.7}"          # default just above a_button_hold_duration (0.5)
RATE_HZ="${RATE_HZ:-20}"
MSG_TYPE="xr_teleop_msgs/msg/Button"

case "${SIDE}" in
  left)  TOPICS=(/xr/left_a_button) ;;
  right) TOPICS=(/xr/right_a_button) ;;
  both)  TOPICS=(/xr/left_a_button /xr/right_a_button) ;;
  *)
    echo "Unknown side '${SIDE}'. Use: left | right | both" >&2
    exit 1
    ;;
esac

if ! ros2 interface show "${MSG_TYPE}" >/dev/null 2>&1; then
  echo "ERROR: message type '${MSG_TYPE}' not found. Is xr_teleop_msgs sourced?" >&2
  echo "       Try: source install/setup.bash" >&2
  exit 1
fi

# Number of messages = rate * duration (rounded, min 1)
N_MSGS=$(awk -v r="${RATE_HZ}" -v d="${HOLD_SEC}" \
  'BEGIN { n = int(r*d + 0.5); if (n < 1) n = 1; print n }')

echo "[sim_a_button] Pressing ${SIDE} for ${HOLD_SEC}s @ ${RATE_HZ} Hz" \
     "(${N_MSGS} msgs per topic)"

# --- press phase: one long-lived publisher per topic ---------------------
PIDS=()
for topic in "${TOPICS[@]}"; do
  ros2 topic pub -r "${RATE_HZ}" -t "${N_MSGS}" \
    "${topic}" "${MSG_TYPE}" "{pressed: true}" \
    >/dev/null 2>&1 &
  PIDS+=($!)
done
for pid in "${PIDS[@]}"; do
  wait "${pid}" || true
done

# --- release phase: one message per topic --------------------------------
# Release disables the arm (enabled_[arm] = false). If you want to keep
# teleop ON, comment out this block.
for topic in "${TOPICS[@]}"; do
  ros2 topic pub --once "${topic}" "${MSG_TYPE}" "{pressed: false}" \
    >/dev/null 2>&1 &
done
wait || true

echo "[sim_a_button] Done."