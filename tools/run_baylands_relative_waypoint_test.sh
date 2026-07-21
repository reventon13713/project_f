#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Stage-2 acceptance contract: public Baylands, raw VINS only (no PX4 EV),
# one complete 12-waypoint relative cross, independent Gazebo P3D truth, and
# a persistent compressed rosbag.  The evaluator's fixed limits are 0.30 m
# trajectory RMSE and 0.50 m trajectory/waypoint maxima.
PX4_SITL_WORLD=vio_baylands \
RECORD_BAGS=1 \
GROUND_TRUTH_EVALUATION=1 \
PX4_VIO_FAULT_TEST=0 \
READY_HOLD_SECONDS="${READY_HOLD_SECONDS:-5}" \
RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-900}" \
EXCITATION_REPEATS="${EXCITATION_REPEATS:-1}" \
EXPERIMENT_LABEL="${EXPERIMENT_LABEL:-baylands_relative_waypoint_gt}" \
"${PROJECT_ROOT}/tools/run_clean_preflight_repeats.sh" "${1:-1}"
