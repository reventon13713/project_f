#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-project_f}"
PX4_GAZEBO_ROOT="/home/reventon/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

docker start "${CONTAINER}" >/dev/null
# Keep the installed world definition synchronized without re-downloading the
# pinned 150 MB Baylands asset on every stage-6 run.
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_worlds/vio_baylands.world" \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/worlds/vio_baylands.world"

# Stage 6 is a live estimator/fault-injection gate. Full camera bags are left
# to stage 8; compressing them live can starve the 0.20 s EV freshness gate.
RECORD_BAGS="${RECORD_BAGS:-0}" \
PRESERVE_LAST_BAGS="${PRESERVE_LAST_BAGS:-0}" \
READY_HOLD_SECONDS="${READY_HOLD_SECONDS:-25}" \
EXCITATION_REPEATS="${EXCITATION_REPEATS:-2}" \
POST_METRICS_RECORD_SECONDS="${POST_METRICS_RECORD_SECONDS:-10}" \
STAGE6_GPS_DROPOUT_TEST=1 \
BRIDGE_MAX_OUTPUT_AGE_SECONDS="${BRIDGE_MAX_OUTPUT_AGE_SECONDS:-0.50}" \
BRIDGE_PX4_RECEIPT_TIMESTAMP="${BRIDGE_PX4_RECEIPT_TIMESTAMP:-true}" \
EKF2_EV_DELAY_MS="${EKF2_EV_DELAY_MS:-0}" \
EKF2_NOAID_TOUT_US="${EKF2_NOAID_TOUT_US:-10000000}" \
EXPERIMENT_LABEL="${EXPERIMENT_LABEL:-stage6_gazebo_gps_dropout_ev_flight}" \
"${PROJECT_ROOT}/tools/run_clean_preflight_repeats.sh" "${1:-1}"
