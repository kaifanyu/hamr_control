#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANALYZE_SCRIPT="${SCRIPT_DIR}/analyze_hamr_vicon_straight.py"
BASE_TOPIC="${BASE_TOPIC:-/HAMR_base/odom}"
ONBOARD_TOPIC="${ONBOARD_TOPIC:-/local_HAMR/odom}"
WHEEL_ODOM_TOPIC="${WHEEL_ODOM_TOPIC:-/wheel_odom}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"

if [[ $# -gt 0 ]]; then
  BAG_DIR="$1"
else
  BAG_DIR=""
  while IFS= read -r candidate; do
    metadata="${candidate}/metadata.yaml"
    if [[ ! -f "${metadata}" ]]; then
      continue
    fi

    # A launch that is stopped during startup can create a valid-looking bag
    # whose topics were discovered but never received data. Pick the newest
    # bag that actually contains onboard odometry.
    if awk '
      $1 == "name:" && $2 == "/local_HAMR/odom" { in_topic = 1; next }
      in_topic && $1 == "message_count:" {
        found = ($2 + 0 > 0)
        exit
      }
      END { exit(found ? 0 : 1) }
    ' "${metadata}"; then
      BAG_DIR="${candidate}"
      break
    fi
  done < <(find "${SCRIPT_DIR}" -maxdepth 1 -type d -name 'hamr_hw_*' | sort -r)
fi

if [[ -z "${BAG_DIR}" ]]; then
  echo "No hamr_hw_* rosbag with onboard odometry found in ${SCRIPT_DIR}" >&2
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
echo "  imu:     ${IMU_TOPIC}"

python3 "${ANALYZE_SCRIPT}" "${BAG_DIR}" \
  --base-topic "${BASE_TOPIC}" \
  --onboard-topic "${ONBOARD_TOPIC}" \
  --wheel-odom-topic "${WHEEL_ODOM_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --plot "${OUT_DIR}/path_reference.png" \
  --localization-plot "${OUT_DIR}/vicon_onboard_path.png" \
  --imu-odom-plot "${OUT_DIR}/imu_odom_path.png" \
  --localization-error-plot "${OUT_DIR}/vicon_onboard_error.png" \
  --wheel-plot "${OUT_DIR}/wheel_cmd_vel.png" \
  --turret-yaw-plot "${OUT_DIR}/path_reference_turret_yaw.png" \
  --json "${OUT_DIR}/metrics.json" \
  --csv "${OUT_DIR}/base_trace.csv" \
  --localization-csv "${OUT_DIR}/localization_trace.csv" \
  --imu-odom-csv "${OUT_DIR}/imu_odom_trace.csv"

echo
echo "Generated:"
echo "  ${OUT_DIR}/path_reference.png"
echo "  ${OUT_DIR}/vicon_onboard_path.png"
echo "  ${OUT_DIR}/vicon_onboard_error.png"
if [[ -f "${OUT_DIR}/imu_odom_path.png" ]]; then
  echo "  ${OUT_DIR}/imu_odom_path.png"
fi
if [[ -f "${OUT_DIR}/wheel_cmd_vel.png" ]]; then
  echo "  ${OUT_DIR}/wheel_cmd_vel.png"
else
  echo "  wheel_cmd_vel.png skipped: no wheel cmd_vel samples"
fi
echo "  ${OUT_DIR}/path_reference_turret_yaw.png"
echo "  ${OUT_DIR}/metrics.json"
echo "  ${OUT_DIR}/base_trace.csv"
echo "  ${OUT_DIR}/localization_trace.csv"
if [[ -f "${OUT_DIR}/imu_odom_trace.csv" ]]; then
  echo "  ${OUT_DIR}/imu_odom_trace.csv"
fi
