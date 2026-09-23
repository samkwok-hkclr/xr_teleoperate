#!/usr/bin/env bash
# ============================================================
# tune_trajectory.sh —— 发布 [mode, radio] 到单个 topic
#
# Usage:
#   ./tune_trajectory.sh <mode> <radio>
#   ./tune_trajectory.sh 1 15
#
# Mode:
#   0 = Direct   (low latency, may jitter)
#   1 = Quintic  (recommended; radio 0..100 -> 0.01..1.01 s curve)
#   2 = Filter   (radio 0..999 -> 20..1 Hz cutoff)
# ============================================================
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <mode> <radio>"
  echo "  mode : 0|1|2"
  echo "  radio: 0..100 (mode=1) | 0..999 (mode=2)"
  exit 1
fi

MODE="$1"
RADIO="$2"

if ! [[ "$MODE" =~ ^[0-9]+$ ]] || (( MODE < 0 || MODE > 2 )); then
  echo "[tune] ERROR: mode must be 0, 1, or 2 (got '$MODE')"
  exit 1
fi
if ! [[ "$RADIO" =~ ^[0-9]+$ ]]; then
  echo "[tune] ERROR: radio must be non-negative (got '$RADIO')"
  exit 1
fi

TOPIC="/dual_arms/trajectory_tuner"

echo "[tune] publishing to ${TOPIC}: [${MODE}, ${RADIO}]"
ros2 topic pub --once "${TOPIC}" \
  std_msgs/msg/Int16MultiArray \
  "{data: [${MODE}, ${RADIO}]}" >/dev/null

echo "[tune] done. Watch ros2_control_node log for:"
echo "        [dual_arms] trajectory_tuner -> mode=${MODE}, radio=${RADIO}"