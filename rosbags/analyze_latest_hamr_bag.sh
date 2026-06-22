#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANALYZE_SCRIPT="${SCRIPT_DIR}/analyze_hamr_vicon_straight.py"
BASE_TOPIC="${BASE_TOPIC:-/HAMR_base/odom}"
ONBOARD_TOPIC="${ONBOARD_TOPIC:-/local_HAMR/odom}"
WHEEL_ODOM_TOPIC="${WHEEL_ODOM_TOPIC:-/wheel_odom}"

if [[ $# -gt 0 ]]; then
  BAG_DIR="$1"
else
  BAG_DIR="$(find "${SCRIPT_DIR}" -maxdepth 1 -type d -name 'hamr_hw_*' | sort | tail -n 1)"
fi

if [[ -z "${BAG_DIR}" ]]; then
  echo "No hamr_hw_* rosbag directories found in ${SCRIPT_DIR}" >&2
  exit 1
fi

if [[ ! -d "${BAG_DIR}" ]]; then
  echo "Bag directory does not exist: ${BAG_DIR}" >&2
  exit 1
fi

OUT_DIR="${BAG_DIR}/analysis"
mkdir -p "${OUT_DIR}"

echo "Analyzing rosbag:"
echo "  bag:     ${BAG_DIR}"
echo "  output:  ${OUT_DIR}"
echo "  vicon:   ${BASE_TOPIC}"
echo "  onboard: ${ONBOARD_TOPIC}"
echo "  wheel:   ${WHEEL_ODOM_TOPIC}"

python3 "${ANALYZE_SCRIPT}" "${BAG_DIR}" \
  --base-topic "${BASE_TOPIC}" \
  --onboard-topic "${ONBOARD_TOPIC}" \
  --wheel-odom-topic "${WHEEL_ODOM_TOPIC}" \
  --plot "${OUT_DIR}/path_reference.png" \
  --localization-plot "${OUT_DIR}/vicon_onboard_path.png" \
  --localization-error-plot "${OUT_DIR}/vicon_onboard_error.png" \
  --wheel-plot "${OUT_DIR}/wheel_cmd_vel.png" \
  --turret-yaw-plot "${OUT_DIR}/path_reference_turret_yaw.png" \
  --json "${OUT_DIR}/metrics.json" \
  --csv "${OUT_DIR}/base_trace.csv" \
  --localization-csv "${OUT_DIR}/localization_trace.csv"

echo
echo "Generated:"
echo "  ${OUT_DIR}/path_reference.png"
echo "  ${OUT_DIR}/vicon_onboard_path.png"
echo "  ${OUT_DIR}/vicon_onboard_error.png"
if [[ -f "${OUT_DIR}/wheel_cmd_vel.png" ]]; then
  echo "  ${OUT_DIR}/wheel_cmd_vel.png"
else
  echo "  wheel_cmd_vel.png skipped: no wheel cmd_vel samples"
fi
echo "  ${OUT_DIR}/path_reference_turret_yaw.png"
echo "  ${OUT_DIR}/metrics.json"
echo "  ${OUT_DIR}/base_trace.csv"
echo "  ${OUT_DIR}/localization_trace.csv"
