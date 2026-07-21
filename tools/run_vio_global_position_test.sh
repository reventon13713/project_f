#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-project_f}"

PX4_SITL_WORLD=vio_baylands \
RECORD_BAGS=1 \
GROUND_TRUTH_EVALUATION=1 \
PX4_VIO_FAULT_TEST=0 \
READY_HOLD_SECONDS="${READY_HOLD_SECONDS:-8}" \
RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-900}" \
EXCITATION_REPEATS="${EXCITATION_REPEATS:-1}" \
EXPERIMENT_LABEL="${EXPERIMENT_LABEL:-vio_global_position}" \
"${PROJECT_ROOT}/tools/run_clean_preflight_repeats.sh" 1

batch_dir="$(find "${PROJECT_ROOT}/artifacts/clean_repeats" -mindepth 1 \
    -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
run_dir="$(find "${batch_dir}" -mindepth 1 -maxdepth 1 -type d -name '*_run*' \
    -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
bag="${run_dir}/sensor_ground_truth_bag"
output="${run_dir}/global_position_metrics.json"
container_bag="/home/reventon/ros2_ws/src/${bag#${PROJECT_ROOT}/}"
container_output="/home/reventon/ros2_ws/src/${output#${PROJECT_ROOT}/}"

docker exec "${CONTAINER}" bash -lc \
    "source /opt/ros/humble/setup.bash; \
     source /home/reventon/ros2_ws/install/setup.bash; \
     ros2 run VIO_pkg vio_global_position_metrics '${container_bag}' \
       --output '${container_output}'"

echo "Global-position acceptance: ${output}"
