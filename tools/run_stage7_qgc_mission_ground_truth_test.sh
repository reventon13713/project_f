#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-project_f}"
PX4_GAZEBO_ROOT="/home/reventon/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

docker start "${CONTAINER}" >/dev/null
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_worlds/vio_baylands.world" \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/worlds/vio_baylands.world"

# The generated qgc_stage7.plan is a normal QGroundControl plan and the node
# uploads it through the same MAVLink mission protocol. The evidence bag omits
# camera/IMU payloads but retains both global tracks and independent Gazebo GT.
RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-420}" \
RECORD_BAGS=1 \
EVIDENCE_BAG_ONLY=1 \
PRESERVE_LAST_BAGS="${PRESERVE_LAST_BAGS:-1}" \
READY_HOLD_SECONDS="${READY_HOLD_SECONDS:-25}" \
EXCITATION_REPEATS="${EXCITATION_REPEATS:-2}" \
POST_METRICS_RECORD_SECONDS="${POST_METRICS_RECORD_SECONDS:-2}" \
STAGE7_QGC_MISSION_TEST=1 \
BRIDGE_MAX_OUTPUT_AGE_SECONDS="${BRIDGE_MAX_OUTPUT_AGE_SECONDS:-0.50}" \
BRIDGE_PX4_RECEIPT_TIMESTAMP="${BRIDGE_PX4_RECEIPT_TIMESTAMP:-true}" \
EKF2_EV_DELAY_MS="${EKF2_EV_DELAY_MS:-0}" \
EKF2_NOAID_TOUT_US="${EKF2_NOAID_TOUT_US:-10000000}" \
EXPERIMENT_LABEL="${EXPERIMENT_LABEL:-stage7_qgc_global_mission_stage8_gt}" \
"${PROJECT_ROOT}/tools/run_clean_preflight_repeats.sh" "${1:-1}"
