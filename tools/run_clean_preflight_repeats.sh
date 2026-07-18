#!/usr/bin/env bash
set -Eeuo pipefail

# Repeat the live Gazebo/PX4 VIO preflight from a clean process state.
# Full bags are disabled by default: one successful uncompressed bag is about
# 1.4 GB.  Set RECORD_BAGS=1 to preserve the sensor/ground-truth topics needed
# for initialization-window analysis.  Those bags are compressed after each
# run so compression load cannot perturb the live estimator.

CONTAINER="${CONTAINER:-project_f}"
RUN_COUNT="${1:-1}"
RUN_TIMEOUT_SECONDS="${RUN_TIMEOUT_SECONDS:-300}"
RECORD_BAGS="${RECORD_BAGS:-0}"
STOP_ON_OUTCOME_CHANGE="${STOP_ON_OUTCOME_CHANGE:-0}"
PREFLIGHT_START_DELAY_SECONDS="${PREFLIGHT_START_DELAY_SECONDS:-0}"
EXPERIMENT_LABEL="${EXPERIMENT_LABEL:-default}"
PRESERVE_LAST_BAGS="${PRESERVE_LAST_BAGS:-0}"
POST_METRICS_RECORD_SECONDS="${POST_METRICS_RECORD_SECONDS:-5}"
READY_HOLD_SECONDS="${READY_HOLD_SECONDS:-60}"
EXCITATION_REPEATS="${EXCITATION_REPEATS:-1}"
EXCITATION_SPEED="${EXCITATION_SPEED:-0.55}"
PX4_VIO_FAULT_TEST="${PX4_VIO_FAULT_TEST:-0}"
PX4_DISABLE_GPS_BEFORE_FAULT="${PX4_DISABLE_GPS_BEFORE_FAULT:-0}"
PX4_DISABLE_GPS_BEFORE_FAULT_BOOL=false
if [[ "${PX4_DISABLE_GPS_BEFORE_FAULT}" == 1 ]]; then
    PX4_DISABLE_GPS_BEFORE_FAULT_BOOL=true
fi
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BATCH_ID="$(date +%Y%m%dT%H%M%SKST)"
BATCH_DIR="${PROJECT_ROOT}/artifacts/clean_repeats/${BATCH_ID}"
CONTAINER_PROJECT_ROOT="/home/reventon/ros2_ws/src"
SUMMARY="${BATCH_DIR}/summary.tsv"
ACTIVE_VINS_CONFIG="${PROJECT_ROOT}/vins_fusion_ros2/config/euroc/euroc_mono_imu_config.yaml"

if ! [[ "${RUN_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
    echo "usage: $0 [positive-run-count]" >&2
    exit 2
fi

if [[ "${RECORD_BAGS}" != 0 && "${RECORD_BAGS}" != 1 ]]; then
    echo "RECORD_BAGS must be 0 or 1" >&2
    exit 2
fi

if [[ "${STOP_ON_OUTCOME_CHANGE}" != 0 && "${STOP_ON_OUTCOME_CHANGE}" != 1 ]]; then
    echo "STOP_ON_OUTCOME_CHANGE must be 0 or 1" >&2
    exit 2
fi

if [[ "${PX4_VIO_FAULT_TEST}" != 0 && "${PX4_VIO_FAULT_TEST}" != 1 ]] ||
   [[ "${PX4_DISABLE_GPS_BEFORE_FAULT}" != 0 &&
      "${PX4_DISABLE_GPS_BEFORE_FAULT}" != 1 ]]; then
    echo "PX4_VIO_FAULT_TEST and PX4_DISABLE_GPS_BEFORE_FAULT must be 0 or 1" >&2
    exit 2
fi

if ! [[ "${PREFLIGHT_START_DELAY_SECONDS}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "PREFLIGHT_START_DELAY_SECONDS must be a non-negative number" >&2
    exit 2
fi

if ! [[ "${PRESERVE_LAST_BAGS}" =~ ^[0-9]+$ ]]; then
    echo "PRESERVE_LAST_BAGS must be a non-negative integer" >&2
    exit 2
fi
if ! [[ "${POST_METRICS_RECORD_SECONDS}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "POST_METRICS_RECORD_SECONDS must be a non-negative number" >&2
    exit 2
fi
if ! [[ "${READY_HOLD_SECONDS}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! [[ "${EXCITATION_REPEATS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "READY_HOLD_SECONDS must be non-negative and EXCITATION_REPEATS positive" >&2
    exit 2
fi
if [[ "${READY_HOLD_SECONDS}" != *.* ]]; then
    READY_HOLD_SECONDS="${READY_HOLD_SECONDS}.0"
fi
if ! [[ "${EXCITATION_SPEED}" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
   ! awk -v speed="${EXCITATION_SPEED}" 'BEGIN { exit !(speed > 0) }'; then
    echo "EXCITATION_SPEED must be a positive number" >&2
    exit 2
fi

mkdir -p "${BATCH_DIR}"
cp "${ACTIVE_VINS_CONFIG}" "${BATCH_DIR}/active_vins_config.yaml"
config_sha256="$(sha256sum "${ACTIVE_VINS_CONFIG}" | awk '{print $1}')"
printf 'run\trun_id\toutcome\tscale_path_ratio\tscale_pair_count\tvins_recovery_count\tvector_rmse_m\tvector_max_m\tmax_vio_step_m\tfeature_queue_max_depth\tfeature_queue_span_max_s\tfeature_queue_span_p95_s\tinitialization_candidate_rejection_count\tsfm_cheirality_rejection_count\tinitialization_probation_pass_count\tinitialization_completion_count\testimator_reset_count\tposition_jump_count\tnonlinear_geometry_rejection_count\tsolver_failure_count\n' > "${SUMMARY}"
printf 'run_count\t%s\nrun_timeout_seconds\t%s\nrecord_bags\t%s\nstop_on_outcome_change\t%s\npreflight_start_delay_seconds\t%s\nexperiment_label\t%s\npreserve_last_bags\t%s\npost_metrics_record_seconds\t%s\nready_hold_seconds\t%s\nexcitation_repeats\t%s\nexcitation_speed\t%s\npx4_vio_fault_test\t%s\npx4_disable_gps_before_fault\t%s\nactive_vins_config_sha256\t%s\n' \
    "${RUN_COUNT}" "${RUN_TIMEOUT_SECONDS}" "${RECORD_BAGS}" \
    "${STOP_ON_OUTCOME_CHANGE}" "${PREFLIGHT_START_DELAY_SECONDS}" \
    "${EXPERIMENT_LABEL}" "${PRESERVE_LAST_BAGS}" \
    "${POST_METRICS_RECORD_SECONDS}" "${READY_HOLD_SECONDS}" \
    "${EXCITATION_REPEATS}" "${EXCITATION_SPEED}" "${PX4_VIO_FAULT_TEST}" \
    "${PX4_DISABLE_GPS_BEFORE_FAULT}" "${config_sha256}" \
    > "${BATCH_DIR}/batch_config.tsv"

ros_env='source /opt/ros/humble/setup.bash; source /home/reventon/ros2_ws/install/setup.bash'

container_exec() {
    docker exec "${CONTAINER}" bash -lc "${ros_env}; $1"
}

cleanup_runtime() {
    docker exec "${CONTAINER}" bash -lc '
        patterns=(
            "[v]io_preflight_test"
            "[v]io_camera_relay"
            "[v]io_bridge"
            "[v]io_px4_fault_test"
            "[v]ins_fusion_ros2_node"
            "[s]tatic_transform_publisher"
            "[r]os2 launch VIO_pkg"
            "[r]os2 bag record"
            "[M]icroXRCEAgent"
            "[g]zserver"
            "[g]zclient"
            "[g]z model --verbose"
            "[P]X4-Autopilot/build/px4_sitl_default/bin/px4"
        )
        for signal in INT TERM KILL; do
            for pattern in "${patterns[@]}"; do
                pkill -"${signal}" -f "${pattern}" 2>/dev/null || true
            done
            sleep 1
        done
    ' >/dev/null 2>&1 || true
}

runtime_processes() {
    docker exec "${CONTAINER}" bash -lc '
        pgrep -af "[v]io_preflight_test|[v]io_camera_relay|[v]io_bridge|[v]io_px4_fault_test|[v]ins_fusion_ros2_node|[s]tatic_transform_publisher|[r]os2 launch VIO_pkg|[r]os2 bag record|[M]icroXRCEAgent|[g]zserver|[g]zclient|[g]z model --verbose|[P]X4-Autopilot/build/px4_sitl_default/bin/px4" || true
    '
}

publisher_count() {
    local topic="$1"
    # Query the live DDS graph directly.  The ros2 daemon can retain a just-
    # killed relay participant long enough to report a false duplicate.
    container_exec "ros2 topic info --no-daemon --spin-time 1 '${topic}' 2>/dev/null | awk '/Publisher count:/ {print \$3}'" | tail -n 1
}

wait_for_publisher_count() {
    local topic="$1"
    local expected="$2"
    local deadline=$((SECONDS + 45))
    local count
    while (( SECONDS < deadline )); do
        count="$(publisher_count "${topic}")"
        if [[ "${count:-0}" == "${expected}" ]]; then
            return 0
        fi
        sleep 1
    done
    echo "publisher check failed: ${topic} expected=${expected} actual=${count:-0}" >&2
    return 1
}

wait_for_message() {
    local topic="$1"
    container_exec "timeout 10 ros2 topic echo --once '${topic}' >/dev/null"
}

write_infra_metrics() {
    local path="$1"
    local outcome="$2"
    printf '{\n  "outcome": "%s"\n}\n' "${outcome}" > "${path}"
}

enrich_queue_metrics() {
    local log_path="$1"
    local metrics_path="$2"
    python3 - "${log_path}" "${metrics_path}" <<'PY'
import json
import math
import re
import sys

log_path, metrics_path = sys.argv[1:]
enqueue_depths = []
dequeue_depths = []
queue_spans = []
event_counts = {
    'initialization_candidate_rejection_count': 0,
    'sfm_cheirality_rejection_count': 0,
    'initialization_probation_pass_count': 0,
    'initialization_completion_count': 0,
    'estimator_reset_count': 0,
    'position_jump_count': 0,
    'nonlinear_geometry_rejection_count': 0,
    'solver_failure_count': 0,
}
enqueue_pattern = re.compile(r'enqueued=1.*queue_depth_after_push=(\d+)')
dequeue_pattern = re.compile(
    r'queue_depth_after_pop=(\d+), queue_span_s=([-+0-9.eE]+)')

with open(log_path, encoding='utf-8', errors='replace') as stream:
    for line in stream:
        if 'Rejected unhealthy initialization candidate:' in line:
            event_counts['initialization_candidate_rejection_count'] += 1
        if 'stage=post_ba_cheirality' in line:
            event_counts['sfm_cheirality_rejection_count'] += 1
        if 'Initialization probation passed;' in line:
            event_counts['initialization_probation_pass_count'] += 1
        if 'Initialization complete. Switching to NON_LINEAR mode.' in line:
            event_counts['initialization_completion_count'] += 1
        if 'reset state successfully' in line:
            event_counts['estimator_reset_count'] += 1
        if 'Position jump detected:' in line:
            event_counts['position_jump_count'] += 1
        if 'Nonlinear geometry health rejected update:' in line:
            event_counts['nonlinear_geometry_rejection_count'] += 1
        if ('Nonlinear optimization failed' in line or
                'Ceres rejected solution:' in line or
                'Cholesky failure' in line or
                'Linear solver failure.' in line or
                'dense Cholesky factorization' in line):
            event_counts['solver_failure_count'] += 1
        match = enqueue_pattern.search(line)
        if match:
            enqueue_depths.append(int(match.group(1)))
        match = dequeue_pattern.search(line)
        if match:
            dequeue_depths.append(int(match.group(1)))
            queue_spans.append(float(match.group(2)))

with open(metrics_path, encoding='utf-8') as stream:
    metrics = json.load(stream)

def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]

metrics.update({
    'feature_queue_enqueue_samples': len(enqueue_depths),
    'feature_queue_dequeue_samples': len(dequeue_depths),
    'feature_queue_max_depth': max(enqueue_depths + dequeue_depths, default=None),
    'feature_queue_span_max_s': max(queue_spans, default=None),
    'feature_queue_span_p95_s': percentile(queue_spans, 0.95),
    **event_counts,
})
replacement_path = metrics_path + '.queue-metrics.tmp'
with open(replacement_path, 'w', encoding='utf-8') as stream:
    json.dump(metrics, stream, indent=2, sort_keys=True)
    stream.write('\n')
import os
os.replace(replacement_path, metrics_path)
PY
}

append_summary() {
    local index="$1"
    local run_id="$2"
    local metrics="$3"
    python3 - "${index}" "${run_id}" "${metrics}" >> "${SUMMARY}" <<'PY'
import json
import sys

index, run_id, path = sys.argv[1:]
with open(path, encoding='utf-8') as stream:
    data = json.load(stream)
keys = (
    'outcome', 'scale_path_ratio', 'scale_pair_count',
    'vins_recovery_count', 'vector_position_rmse_m_no_scale_fit',
    'vector_position_max_error_m_no_scale_fit', 'max_vio_pose_step_m',
    'feature_queue_max_depth', 'feature_queue_span_max_s',
    'feature_queue_span_p95_s',
    'initialization_candidate_rejection_count',
    'sfm_cheirality_rejection_count',
    'initialization_probation_pass_count',
    'initialization_completion_count', 'estimator_reset_count',
    'position_jump_count', 'nonlinear_geometry_rejection_count',
    'solver_failure_count',
)
values = [index, run_id]
values.extend('' if data.get(key) is None else str(data.get(key)) for key in keys)
print('\t'.join(values))
PY
}

apply_px4_fault_result() {
    local metrics_path="$1"
    local fault_path="$2"
    python3 - "${metrics_path}" "${fault_path}" <<'PY'
import json
import os
import sys

metrics_path, fault_path = sys.argv[1:]
with open(metrics_path, encoding='utf-8') as stream:
    metrics = json.load(stream)

fault = None
failure_reason = ''
try:
    with open(fault_path, encoding='utf-8') as stream:
        fault = json.load(stream)
except FileNotFoundError:
    failure_reason = 'PX4 fault metrics missing'
except (OSError, json.JSONDecodeError) as exc:
    failure_reason = f'PX4 fault metrics unreadable: {exc}'

passed = isinstance(fault, dict) and fault.get('passed') is True
if not passed and not failure_reason:
    failure_reason = str(fault.get('reason', 'PX4 fault assertion failed'))

metrics['px4_fault_test_passed'] = passed
metrics['px4_fault_test_reason'] = (
    str(fault.get('reason', 'passed')) if passed else failure_reason)
if not passed:
    metrics['preflight_outcome'] = metrics.get('outcome')
    metrics['outcome'] = 'PX4_FAULT_TEST_FAILED'

replacement = metrics_path + '.px4-fault.tmp'
with open(replacement, 'w', encoding='utf-8') as stream:
    json.dump(metrics, stream, indent=2, sort_keys=True)
    stream.write('\n')
os.replace(replacement, metrics_path)
raise SystemExit(0 if passed else 1)
PY
}

prune_old_bags() {
    if [[ "${RECORD_BAGS}" != 1 || "${PRESERVE_LAST_BAGS}" == 0 ]]; then
        return
    fi
    mapfile -t bag_dirs < <(
        find "${BATCH_DIR}" -mindepth 2 -maxdepth 2 -type d \
            -name sensor_ground_truth_bag -printf '%T@ %p\n' \
            | sort -n | awk '{print $2}'
    )
    local excess=$((${#bag_dirs[@]} - PRESERVE_LAST_BAGS))
    for ((index = 0; index < excess; ++index)); do
        echo "Pruning rolling full bag: ${bag_dirs[index]}"
        rm -rf -- "${bag_dirs[index]}"
    done
}

trap cleanup_runtime EXIT INT TERM

docker start "${CONTAINER}" >/dev/null
cleanup_runtime
previous_outcome=''
batch_fault_failed=0

if [[ -n "$(runtime_processes)" ]]; then
    echo "Unable to establish a zero-process preflight state:" >&2
    runtime_processes >&2
    exit 1
fi

echo "Building VIO packages once for batch ${BATCH_ID}..."
container_exec 'cd /home/reventon/ros2_ws; colcon build --packages-select vins_fusion_ros2 VIO_pkg --symlink-install' \
    > "${BATCH_DIR}/colcon_build.log" 2>&1

for ((run_index = 1; run_index <= RUN_COUNT; run_index++)); do
    run_id="$(date +%Y%m%dT%H%M%SKST)_run$(printf '%02d' "${run_index}")"
    run_dir="${BATCH_DIR}/${run_id}"
    container_run_dir="${CONTAINER_PROJECT_ROOT}/artifacts/clean_repeats/${BATCH_ID}/${run_id}"
    metrics="${run_dir}/metrics.json"
    mkdir -p "${run_dir}"

    echo "[${run_index}/${RUN_COUNT}] ${run_id}: cleaning runtime..."
    cleanup_runtime
    runtime_processes > "${run_dir}/preflight_processes.txt"
    if [[ -s "${run_dir}/preflight_processes.txt" ]]; then
        write_infra_metrics "${metrics}" 'INFRA_STALE_PROCESS'
        append_summary "${run_index}" "${run_id}" "${metrics}"
        continue
    fi

    # Start the DDS bridge before PX4 so its ROS publishers are discoverable
    # before the flight controller is allowed to arm.
    container_exec 'MicroXRCEAgent udp4 -p 8888' \
        > "${run_dir}/microxrce_agent.log" 2>&1 &
    agent_host_pid=$!

    docker exec "${CONTAINER}" bash -lc "
        ${ros_env};
        cd /home/reventon/PX4-Autopilot;
        HEADLESS=1 PX4_SITL_WORLD=vio_feature \
        GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:\${GAZEBO_PLUGIN_PATH:-} \
        make px4_sitl gazebo-classic_iris_fpv_cam
    " 2>&1 \
        | tr '\r' '\n' \
        | sed -u -E $'s/\033\\[[0-9;]*[[:alpha:]]//g; /^[[:space:]]*pxh>[[:space:]]*$/d' \
        > "${run_dir}/px4_gazebo.log" &
    simulator_host_pid=$!

    infra_ok=true
    for topic in /clock /vio/imu /vio/camera/image_raw /vio/range /fmu/out/vehicle_odometry; do
        if ! wait_for_publisher_count "${topic}" 1 || ! wait_for_message "${topic}"; then
            echo "Missing live sensor stream: ${topic}" >> "${run_dir}/infrastructure_error.log"
            infra_ok=false
            break
        fi
    done

    if [[ "${infra_ok}" == true ]]; then
        clock_before="$(container_exec "ros2 topic echo --once /clock" | tr '\n' ' ')"
        sleep 1
        clock_after="$(container_exec "ros2 topic echo --once /clock" | tr '\n' ' ')"
        if [[ "${clock_before}" == "${clock_after}" ]]; then
            echo 'Simulation clock did not advance.' >> "${run_dir}/infrastructure_error.log"
            infra_ok=false
        fi
    fi

    # This test is estimator-only.  Any publisher feeding PX4 visual odometry
    # would invalidate the safety condition and the experimental comparison.
    if [[ "${infra_ok}" == true ]]; then
        visual_odom_publishers="$(publisher_count /fmu/in/vehicle_visual_odometry)"
        if [[ "${visual_odom_publishers:-0}" != 0 ]]; then
            echo "PX4 visual odometry has ${visual_odom_publishers} publisher(s)." \
                >> "${run_dir}/infrastructure_error.log"
            infra_ok=false
        fi
    fi

    if [[ "${infra_ok}" != true ]]; then
        write_infra_metrics "${metrics}" 'INFRA_SENSOR_CHECK_FAILED'
        append_summary "${run_index}" "${run_id}" "${metrics}"
        cleanup_runtime
        wait "${agent_host_pid}" "${simulator_host_pid}" 2>/dev/null || true
        continue
    fi

    recorder_host_pid=''
    relay_host_pid=''
    if [[ "${RECORD_BAGS}" == 1 ]]; then
        # Advertise and stream the relay topic before recording, then start the
        # estimator only after rosbag is subscribed.  The bag therefore
        # contains a superset of every image the live feature tracker saw.
        container_exec "ros2 run VIO_pkg vio_camera_relay --ros-args \
            -p use_sim_time:=true \
            -p input_topic:=/vio/camera/image_raw \
            -p output_topic:=/vio/camera/vins_image_raw \
            -p target_rate_hz:=25.0" \
            > "${run_dir}/vio_relay.log" 2>&1 &
        relay_host_pid=$!
        if ! wait_for_publisher_count /vio/camera/vins_image_raw 1; then
            write_infra_metrics "${metrics}" 'INFRA_RELAY_COUNT_FAILED'
            append_summary "${run_index}" "${run_id}" "${metrics}"
            cleanup_runtime
            wait "${agent_host_pid}" "${simulator_host_pid}" "${relay_host_pid}" 2>/dev/null || true
            continue
        fi
        container_exec "ros2 bag record \
            --compression-mode file --compression-format zstd \
            -o '${container_run_dir}/sensor_ground_truth_bag' \
            /clock /vio/camera/image_raw /vio/camera/vins_image_raw \
            /vio/imu /vio/range /fmu/out/vehicle_odometry \
            /odometry /vio/ready /vio/handover_ready /vio/preflight_event" \
            > "${run_dir}/rosbag_record.log" 2>&1 &
        recorder_host_pid=$!
        sleep 2
        if ! kill -0 "${recorder_host_pid}" 2>/dev/null; then
            write_infra_metrics "${metrics}" 'INFRA_RECORDER_EXITED'
            append_summary "${run_index}" "${run_id}" "${metrics}"
            cleanup_runtime
            wait "${agent_host_pid}" "${simulator_host_pid}" \
                "${relay_host_pid}" "${recorder_host_pid}" 2>/dev/null || true
            continue
        fi
    fi

    start_camera_relay=true
    if [[ "${RECORD_BAGS}" == 1 ]]; then
        start_camera_relay=false
    fi
    container_exec "ros2 launch VIO_pkg vio_vins.launch.py \
        start_camera_relay:=${start_camera_relay}" \
        > "${run_dir}/vio_vins.log" 2>&1 &
    vins_host_pid=$!

    if ! wait_for_publisher_count /vio/camera/vins_image_raw 1; then
        write_infra_metrics "${metrics}" 'INFRA_RELAY_COUNT_FAILED'
        append_summary "${run_index}" "${run_id}" "${metrics}"
        cleanup_runtime
        wait "${agent_host_pid}" "${simulator_host_pid}" "${vins_host_pid}" \
            ${relay_host_pid:+"${relay_host_pid}"} \
            ${recorder_host_pid:+"${recorder_host_pid}"} 2>/dev/null || true
        continue
    fi

    bridge_host_pid=''
    fault_test_host_pid=''
    fault_metrics="${run_dir}/px4_vio_fault_metrics.json"
    if [[ "${PX4_VIO_FAULT_TEST}" == 1 ]]; then
        container_exec "ros2 run VIO_pkg vio_bridge --ros-args \
            -p use_sim_time:=true -p publish_visual_odometry:=true" \
            > "${run_dir}/vio_bridge.log" 2>&1 &
        bridge_host_pid=$!
        if ! wait_for_publisher_count /fmu/in/vehicle_visual_odometry 1; then
            write_infra_metrics "${metrics}" 'INFRA_BRIDGE_COUNT_FAILED'
            append_summary "${run_index}" "${run_id}" "${metrics}"
            cleanup_runtime
            continue
        fi
        container_exec "ros2 run VIO_pkg vio_px4_fault_test --ros-args \
            -p use_sim_time:=true \
            -p disable_gps_before_fault:=${PX4_DISABLE_GPS_BEFORE_FAULT_BOOL} \
            -p fault_timeout_seconds:=25.0 \
            -p metrics_output:='${container_run_dir}/px4_vio_fault_metrics.json'" \
            > "${run_dir}/vio_px4_fault_test.log" 2>&1 &
        fault_test_host_pid=$!
    fi

    # Isolate the effect of allowing VINS/relay callbacks to accumulate before
    # the flight preflight begins.  This delay is independent of rosbag startup;
    # RECORD_BAGS=1 still keeps its separate recorder readiness wait above.
    if [[ "${PREFLIGHT_START_DELAY_SECONDS}" != 0 ]]; then
        sleep "${PREFLIGHT_START_DELAY_SECONDS}"
    fi

    container_exec "ros2 run VIO_pkg vio_preflight_test --ros-args \
        -p use_sim_time:=true -p warmup_seconds:=5.0 \
        -p ready_hold_seconds:=${READY_HOLD_SECONDS} \
        -p excitation_repeats:=${EXCITATION_REPEATS} \
        -p excitation_speed:=${EXCITATION_SPEED} \
        -p metrics_output:='${container_run_dir}/metrics.json'" \
        > "${run_dir}/vio_preflight.log" 2>&1 &
    preflight_host_pid=$!

    deadline=$((SECONDS + RUN_TIMEOUT_SECONDS))
    while [[ ! -s "${metrics}" ]] && (( SECONDS < deadline )); do
        if ! kill -0 "${preflight_host_pid}" 2>/dev/null; then
            break
        fi
        sleep 2
    done

    if [[ ! -s "${metrics}" ]]; then
        if kill -0 "${preflight_host_pid}" 2>/dev/null; then
            write_infra_metrics "${metrics}" 'INFRA_WALL_TIMEOUT'
        else
            write_infra_metrics "${metrics}" 'INFRA_PREFLIGHT_EXITED'
        fi
    fi

    if [[ "${PX4_VIO_FAULT_TEST}" == 1 && -s "${metrics}" ]]; then
        fault_deadline=$((SECONDS + 35))
        while [[ ! -s "${fault_metrics}" ]] && (( SECONDS < fault_deadline )); do
            sleep 1
        done
        if ! apply_px4_fault_result "${metrics}" "${fault_metrics}"; then
            batch_fault_failed=1
        fi
    fi

    if [[ -n "${recorder_host_pid}" ]]; then
        # Preserve events which race just behind terminal preflight metrics.
        # A live guard reset was previously logged about two simulation seconds
        # after VALIDATION_TIMEOUT while rosbag had already stopped.
        sleep "${POST_METRICS_RECORD_SECONDS}"
        # Give rosbag2 an INT and let it finish metadata/file compression before
        # the general cleanup escalates to TERM/KILL for the other processes.
        docker exec "${CONTAINER}" bash -lc \
            'pkill -INT -f "[r]os2 bag record" 2>/dev/null || true' || true
        wait "${recorder_host_pid}" 2>/dev/null || true
    fi
    cleanup_runtime
    wait "${agent_host_pid}" "${simulator_host_pid}" "${vins_host_pid}" "${preflight_host_pid}" \
        ${bridge_host_pid:+"${bridge_host_pid}"} \
        ${fault_test_host_pid:+"${fault_test_host_pid}"} \
        ${relay_host_pid:+"${relay_host_pid}"} \
        2>/dev/null || true
    runtime_processes > "${run_dir}/postflight_processes.txt"
    enrich_queue_metrics "${run_dir}/vio_vins.log" "${metrics}"
    append_summary "${run_index}" "${run_id}" "${metrics}"
    prune_old_bags

    outcome="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["outcome"])' "${metrics}")"
    echo "[${run_index}/${RUN_COUNT}] ${run_id}: ${outcome}"
    if [[ "${STOP_ON_OUTCOME_CHANGE}" == 1 && -n "${previous_outcome}" && \
          "${outcome}" != "${previous_outcome}" ]]; then
        echo "Adjacent outcome change captured: ${previous_outcome} -> ${outcome}"
        break
    fi
    previous_outcome="${outcome}"
done

cleanup_runtime
echo "Batch complete: ${SUMMARY}"
if [[ "${batch_fault_failed}" == 1 ]]; then
    echo "One or more PX4 fault assertions failed; see summary and fault logs." >&2
    exit 1
fi
