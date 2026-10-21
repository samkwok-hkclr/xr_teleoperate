#!/usr/bin/env bash
#
# Simulate pressing B button(s) continuously for HOLD_SEC seconds,
# then release. Triggers /start_robot <-> /shutdown_robot in xr_teleop.
#
# Uses ONE ros2 topic pub process per topic, publishing N messages at
# RATE_HZ (via -r RATE -t N). This avoids the per-iteration process-spawn
# overhead that makes the naive --once loop run at ~3-10 Hz regardless
# of RATE_HZ.
#
# Usage:
#   ./script/sim_b_button.sh                     # both, 3.5 s, 20 Hz
#   RATE_HZ=50 ./script/sim_b_button.sh          # both, 3.5 s, 50 Hz
#   ./script/sim_b_button.sh left 5.0            # left only, 5 s
#   ./script/sim_b_button.sh right 2.0           # right only, 2 s (below threshold)
#
set -euo pipefail

SIDE="${1:-both}"
HOLD_SEC="${2:-3.5}"
RATE_HZ="${RATE_HZ:-20}"
MSG_TYPE="xr_teleop_msgs/msg/Button"

case "${SIDE}" in
  left)  TOPICS=(/xr/left_b_button) ;;
  right) TOPICS=(/xr/right_b_button) ;;
  both)  TOPICS=(/xr/left_b_button /xr/right_b_button) ;;
  *)
    echo "Unknown side '${SIDE}'. Use: left | right | both" >&2
    exit 1
    ;;
esac

# Message type must be discoverable
if ! ros2 interface show "${MSG_TYPE}" >/dev/null 2>&1; then
  echo "ERROR: message type '${MSG_TYPE}' not found. Is xr_teleop_msgs sourced?" >&2
  echo "       Try: source install/setup.bash" >&2
  exit 1
fi

# Number of messages to publish = rate * duration (rounded).
N_MSGS=$(awk -v r="${RATE_HZ}" -v d="${HOLD_SEC}" \
  'BEGIN { n = int(r*d + 0.5); if (n < 1) n = 1; print n }')

echo "[sim_b_button] Pressing ${SIDE} for ${HOLD_SEC}s @ ${RATE_HZ} Hz" \
     "(${N_MSGS} msgs per topic)"

# --- press phase: one long-lived publisher per topic ---------------------
PIDS=()
for topic in "${TOPICS[@]}"; do
  # -r: rate in Hz; -t: stop after N messages; then the process exits.
  # The default --wait-matching-subscriptions=1 makes the publisher block
  # until xr_teleop has subscribed, so timing starts after discovery.
  ros2 topic pub -r "${RATE_HZ}" -t "${N_MSGS}" \
    "${topic}" "${MSG_TYPE}" "{pressed: true}" \
    >/dev/null 2>&1 &
  PIDS+=($!)
done

# Wait for every publisher to finish its N messages
for pid in "${PIDS[@]}"; do
  wait "${pid}" || true
done

# --- release phase: one message per topic --------------------------------
for topic in "${TOPICS[@]}"; do
  ros2 topic pub --once "${topic}" "${MSG_TYPE}" "{pressed: false}" \
    >/dev/null 2>&1 &
done
wait || true

echo "[sim_b_button] Done."