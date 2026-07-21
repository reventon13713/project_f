#!/usr/bin/env python3
"""Stage-5 GPS+EV global continuity acceptance from rosbag and PX4 ULog."""

import argparse
from bisect import bisect_left
import json
import math

import numpy as np

from VIO_pkg.vio_bag_metrics import read_topic
from VIO_pkg.vio_global_position import geodetic_to_ecef


def nearest(samples, timestamps, timestamp_ns, max_gap_ns=150_000_000):
    index = bisect_left(timestamps, timestamp_ns)
    candidates = samples[max(0, index - 1):index + 1]
    if not candidates:
        return None
    match = min(candidates, key=lambda sample: abs(sample[0] - timestamp_ns))
    return match if abs(match[0] - timestamp_ns) <= max_gap_ns else None


def step_statistics(samples):
    if len(samples) < 2:
        return {'maximum_3d': math.inf, 'p95_3d': math.inf,
                'maximum_gap_s': math.inf}
    steps = []
    gaps = []
    previous_time, previous_point = samples[0]
    previous_ecef = geodetic_to_ecef(*previous_point)
    for timestamp_ns, point in samples[1:]:
        current_ecef = geodetic_to_ecef(*point)
        steps.append(float(np.linalg.norm(current_ecef - previous_ecef)))
        gaps.append((timestamp_ns - previous_time) * 1e-9)
        previous_time = timestamp_ns
        previous_ecef = current_ecef
    return {
        'maximum_3d': max(steps),
        'p95_3d': float(np.percentile(steps, 95)),
        'maximum_gap_s': max(gaps),
    }


def analyze_ulog(path):
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise RuntimeError(
            'pyulog is required (validated version: 0.9.0)') from exc

    names = ['estimator_status_flags', 'estimator_aid_src_gnss_pos',
             'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel',
             'vehicle_visual_odometry', 'vehicle_global_position']
    ulog = ULog(path, message_name_filter_list=names)
    datasets = {dataset.name: dataset for dataset in ulog.data_list
                if dataset.multi_id == 0}
    missing = [name for name in names if name not in datasets]
    if missing:
        raise RuntimeError(f'ULog missing datasets: {missing}')

    flags = datasets['estimator_status_flags'].data
    simultaneous = (flags['cs_gps'].astype(bool) &
                    flags['cs_ev_pos'].astype(bool) &
                    flags['cs_ev_vel'].astype(bool))
    simultaneous_times = flags['timestamp'][simultaneous]
    start_us = int(simultaneous_times[0]) if len(simultaneous_times) else None

    aid = {}
    for name in ('estimator_aid_src_gnss_pos', 'estimator_aid_src_ev_pos',
                 'estimator_aid_src_ev_vel'):
        data = datasets[name].data
        aid[name] = {
            'samples': len(data['timestamp']),
            'fusion_enabled_samples': int(np.count_nonzero(data['fusion_enabled'])),
            'fused_samples': int(np.count_nonzero(data['fused'])),
            'rejected_samples': int(np.count_nonzero(data['innovation_rejected'])),
        }

    global_data = datasets['vehicle_global_position'].data
    if start_us is None:
        active_indices = np.array([], dtype=int)
    else:
        active_indices = np.flatnonzero(global_data['timestamp'] >= start_us)
    reset_changes = 0
    max_step = math.inf
    duration_s = 0.0
    if len(active_indices) >= 2:
        times = global_data['timestamp'][active_indices]
        duration_s = float(times[-1] - times[0]) * 1e-6
        reset_counter = global_data['lat_lon_reset_counter'][active_indices]
        reset_changes = int(np.count_nonzero(np.diff(reset_counter)))
        points = zip(global_data['lat'][active_indices],
                     global_data['lon'][active_indices],
                     global_data['alt_ellipsoid'][active_indices])
        ecef = np.asarray([geodetic_to_ecef(*point) for point in points])
        max_step = float(np.max(np.linalg.norm(np.diff(ecef, axis=0), axis=1)))

    visual_count = len(datasets['vehicle_visual_odometry'].data['timestamp'])
    passed = (len(simultaneous_times) > 0 and visual_count >= 10 and
              all(item['fusion_enabled_samples'] > 0 and item['fused_samples'] > 0
                  for item in aid.values()) and
              len(active_indices) >= 2 and reset_changes == 0 and max_step <= 0.50)
    return {
        'passed': bool(passed),
        'simultaneous_gps_ev_pos_ev_vel_samples': len(simultaneous_times),
        'first_simultaneous_fusion_timestamp_us': start_us,
        'vehicle_visual_odometry_samples': visual_count,
        'aid_sources': aid,
        'global_after_fusion': {
            'samples': len(active_indices), 'duration_s': duration_s,
            'lat_lon_reset_changes': reset_changes,
            'maximum_step_3d_m': max_step,
        },
    }


def analyze_bag(path, minimum_duration_s, minimum_pairs,
                maximum_rmse_m, maximum_error_m, maximum_step_m,
                maximum_output_gap_s):
    topics = {'/vio/global_position', '/fmu/out/vehicle_global_position',
              '/fmu/out/vehicle_gps_position', '/vio/global_alignment_ready',
              '/vio/handover_ready'}
    messages = read_topic(path, topics, include_topic=True)
    alignment_time = None
    alignment_ready = False
    handover_ready = False
    raw_vio = []
    raw_px4 = []
    raw_gps = []
    for topic, timestamp_ns, message in messages:
        if topic == '/vio/global_alignment_ready' and bool(message.data):
            alignment_ready = True
            alignment_time = timestamp_ns if alignment_time is None else alignment_time
        elif topic == '/vio/handover_ready' and bool(message.data):
            handover_ready = True
        elif topic == '/vio/global_position':
            point = (message.latitude, message.longitude, message.altitude)
            if np.isfinite(point).all():
                raw_vio.append((timestamp_ns, point))
        elif topic == '/fmu/out/vehicle_global_position':
            point = (message.lat, message.lon, message.alt_ellipsoid)
            if np.isfinite(point).all() and abs(message.lat) <= 90.0:
                raw_px4.append((timestamp_ns, point))
        elif topic == '/fmu/out/vehicle_gps_position':
            raw_gps.append((timestamp_ns, int(message.fix_type)))

    if alignment_time is None:
        alignment_time = 2**63 - 1
    vio = [sample for sample in raw_vio if sample[0] >= alignment_time]
    px4 = [sample for sample in raw_px4 if sample[0] >= alignment_time]
    gps = [sample for sample in raw_gps if sample[0] >= alignment_time]
    px4_times = [sample[0] for sample in px4]
    errors = []
    paired_times = []
    for timestamp_ns, point in vio:
        match = nearest(px4, px4_times, timestamp_ns)
        if match is not None:
            errors.append(float(np.linalg.norm(
                geodetic_to_ecef(*point) - geodetic_to_ecef(*match[1]))))
            paired_times.append(timestamp_ns)

    duration_s = ((paired_times[-1] - paired_times[0]) * 1e-9
                  if len(paired_times) >= 2 else 0.0)
    rmse = math.sqrt(float(np.mean(np.square(errors)))) if errors else math.inf
    maximum = max(errors) if errors else math.inf
    error_step = (max(abs(right - left) for left, right in zip(errors, errors[1:]))
                  if len(errors) >= 2 else math.inf)
    vio_steps = step_statistics(vio)
    px4_steps = step_statistics(px4)
    gps_3d = sum(fix_type >= 3 for _, fix_type in gps)
    passed = (alignment_ready and handover_ready and
              len(errors) >= minimum_pairs and duration_s >= minimum_duration_s and
              gps_3d > 0 and rmse <= maximum_rmse_m and maximum <= maximum_error_m and
              vio_steps['maximum_3d'] <= maximum_step_m and
              px4_steps['maximum_3d'] <= maximum_step_m and
              error_step <= maximum_step_m and
              vio_steps['maximum_gap_s'] <= maximum_output_gap_s)
    return {
        'passed': bool(passed),
        'alignment_ready': alignment_ready, 'handover_ready': handover_ready,
        'sample_counts': {'vio': len(vio), 'px4': len(px4),
                          'paired': len(errors), 'gps_3d_fix': gps_3d},
        'paired_duration_s': duration_s,
        'cross_source_error_m': {
            'rmse': rmse, 'p95': float(np.percentile(errors, 95)) if errors else math.inf,
            'maximum': maximum, 'maximum_consecutive_change': error_step,
        },
        'vio_global_continuity': vio_steps,
        'px4_global_continuity': px4_steps,
        'limits': {
            'minimum_duration_s': minimum_duration_s,
            'minimum_pairs': minimum_pairs,
            'maximum_rmse_m': maximum_rmse_m,
            'maximum_error_m': maximum_error_m,
            'maximum_step_m': maximum_step_m,
            'maximum_output_gap_s': maximum_output_gap_s,
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('ulog')
    parser.add_argument('--output', required=True)
    parser.add_argument('--minimum-duration-s', type=float, default=10.0)
    parser.add_argument('--minimum-pairs', type=int, default=50)
    parser.add_argument('--maximum-rmse-m', type=float, default=0.50)
    parser.add_argument('--maximum-error-m', type=float, default=1.00)
    parser.add_argument('--maximum-step-m', type=float, default=0.50)
    parser.add_argument('--maximum-output-gap-s', type=float, default=0.50)
    args = parser.parse_args()
    bag = analyze_bag(args.bag, args.minimum_duration_s, args.minimum_pairs,
                      args.maximum_rmse_m, args.maximum_error_m,
                      args.maximum_step_m, args.maximum_output_gap_s)
    ulog = analyze_ulog(args.ulog)
    result = {'passed': bag['passed'] and ulog['passed'],
              'rosbag': bag, 'ulog': ulog}
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
