#!/usr/bin/env python3
"""Evaluate a relative-waypoint VIO flight against independent Gazebo truth."""

import argparse
from bisect import bisect_left
import json
import math

import numpy as np

from VIO_pkg.vio_bag_metrics import read_topic


VIO_TOPIC = '/odometry'
GT_TOPIC = '/vio/ground_truth/odometry'
PX4_TOPIC = '/fmu/out/vehicle_odometry'
EVENT_TOPIC = '/vio/preflight_event'


def nearest(samples, times, timestamp_ns, max_gap_ns=200_000_000):
    index = bisect_left(times, timestamp_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    if not candidates:
        return None
    sample = min(candidates, key=lambda item: abs(item[0] - timestamp_ns))
    return sample if abs(sample[0] - timestamp_ns) <= max_gap_ns else None


def yaw_translation_fit(source, target, motion_threshold_m=0.02):
    """Fit only yaw and translation; metric scale remains observable evidence."""
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    ds = np.diff(source[:, :2], axis=0)
    dt = np.diff(target[:, :2], axis=0)
    moving = ((np.linalg.norm(ds, axis=1) > motion_threshold_m) &
              (np.linalg.norm(dt, axis=1) > motion_threshold_m))
    if int(np.count_nonzero(moving)) < 5:
        raise RuntimeError('insufficient common XY motion for yaw alignment')
    ds = ds[moving]
    dt = dt[moving]
    dot = float(np.sum(ds[:, 0] * dt[:, 0] + ds[:, 1] * dt[:, 1]))
    cross = float(np.sum(ds[:, 0] * dt[:, 1] - ds[:, 1] * dt[:, 0]))
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    translation = np.mean(target - (rotation @ source.T).T, axis=0)
    return yaw, rotation, translation


def position_from_odometry(message):
    p = message.pose.pose.position
    return np.array([p.x, p.y, p.z], dtype=float)


def select_complete_attempt(events):
    attempts = {}
    for timestamp_ns, message in events:
        try:
            event = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            continue
        kind = event.get('event')
        if kind not in ('RELATIVE_WAYPOINT_MISSION_START',
                        'RELATIVE_WAYPOINT_REACHED'):
            continue
        attempt = int(event.get('attempt', 0))
        record = attempts.setdefault(attempt, {'start': None, 'count': None, 'reached': []})
        if kind == 'RELATIVE_WAYPOINT_MISSION_START':
            record['start'] = timestamp_ns
            record['count'] = int(event['waypoint_count'])
            record['reached'] = []
        elif record['start'] is not None and timestamp_ns >= record['start']:
            record['reached'].append((timestamp_ns, event))
    complete = []
    for attempt, record in attempts.items():
        indices = [int(item[1]['index']) for item in record['reached']]
        expected = list(range(record['count'] or 0))
        if record['start'] is not None and indices == expected:
            complete.append((attempt, record))
    if not complete:
        detail = {key: len(value['reached']) for key, value in attempts.items()}
        raise RuntimeError(f'no complete relative-waypoint attempt in bag: {detail}')
    return max(complete, key=lambda item: item[0])


def evaluate(bag_path, limits):
    messages = read_topic(bag_path, {VIO_TOPIC, GT_TOPIC, PX4_TOPIC, EVENT_TOPIC},
                          include_topic=True)
    topics = {name: [] for name in (VIO_TOPIC, GT_TOPIC, PX4_TOPIC, EVENT_TOPIC)}
    for topic, timestamp_ns, message in messages:
        topics[topic].append((timestamp_ns, message))

    attempt, mission = select_complete_attempt(topics[EVENT_TOPIC])
    start_ns = mission['start']
    end_ns = mission['reached'][-1][0]
    vio = [(t, position_from_odometry(m)) for t, m in topics[VIO_TOPIC]
           if start_ns <= t <= end_ns]
    gt = [(t, position_from_odometry(m)) for t, m in topics[GT_TOPIC]
          if start_ns <= t <= end_ns]
    px4 = [(t, np.asarray(m.position, dtype=float)) for t, m in topics[PX4_TOPIC]
           if start_ns <= t <= end_ns]
    if min(len(vio), len(gt), len(px4)) < 10:
        raise RuntimeError(f'insufficient mission samples: vio={len(vio)}, gt={len(gt)}, px4={len(px4)}')

    gt_times = [item[0] for item in gt]
    px4_times = [item[0] for item in px4]
    vio_gt = []
    for timestamp_ns, position in vio:
        match = nearest(gt, gt_times, timestamp_ns)
        if match is not None:
            vio_gt.append((timestamp_ns, position, match[1]))
    if len(vio_gt) < 10:
        raise RuntimeError(f'insufficient VIO/GT synchronized samples: {len(vio_gt)}')

    vio_position = np.asarray([item[1] for item in vio_gt])
    gt_position = np.asarray([item[2] for item in vio_gt])
    vio_yaw, vio_rotation, vio_translation = yaw_translation_fit(vio_position, gt_position)
    aligned_vio = (vio_rotation @ vio_position.T).T + vio_translation
    trajectory_error = np.linalg.norm(aligned_vio - gt_position, axis=1)

    # Convert Gazebo ENU positions to the PX4 NED axis convention, then fit
    # only the local-origin yaw/translation needed to compare commanded NED
    # targets with the independent simulator pose.
    gt_px4_source = []
    gt_px4_target = []
    for timestamp_ns, position in gt:
        match = nearest(px4, px4_times, timestamp_ns)
        if match is not None:
            gt_px4_source.append(np.array([position[1], position[0], -position[2]]))
            gt_px4_target.append(match[1])
    # GT/PX4 are paired near 50 Hz, so a healthy 0.55 m/s leg moves only about
    # 1.1 cm per sample.  The 2 cm VINS threshold would discard every sample.
    gt_yaw, gt_rotation, gt_translation = yaw_translation_fit(
        gt_px4_source, gt_px4_target, motion_threshold_m=0.005)

    waypoint_results = []
    vio_gt_times = [item[0] for item in vio_gt]
    for timestamp_ns, event in mission['reached']:
        gt_match = nearest(gt, gt_times, timestamp_ns)
        vio_match = nearest(vio_gt, vio_gt_times, timestamp_ns)
        if gt_match is None or vio_match is None:
            raise RuntimeError(f'missing synchronized pose for waypoint {event["index"]}')
        target = np.asarray(event['target_ned_m'], dtype=float)
        gt_enu = gt_match[1]
        gt_ned = gt_rotation @ np.array([gt_enu[1], gt_enu[0], -gt_enu[2]]) + gt_translation
        aligned_vio_at_event = vio_rotation @ vio_match[1] + vio_translation
        waypoint_results.append({
            'index': int(event['index']),
            'target_ned_m': target.tolist(),
            'ground_truth_ned_m': gt_ned.tolist(),
            'ground_truth_reach_error_m': float(np.linalg.norm(gt_ned - target)),
            'vio_ground_truth_error_m': float(np.linalg.norm(aligned_vio_at_event - gt_enu)),
        })

    rmse = float(math.sqrt(np.mean(trajectory_error ** 2)))
    max_error = float(np.max(trajectory_error))
    reach_max = max(item['ground_truth_reach_error_m'] for item in waypoint_results)
    waypoint_vio_max = max(item['vio_ground_truth_error_m'] for item in waypoint_results)
    checks = {
        'trajectory_rmse': rmse <= limits['rmse_m'],
        'trajectory_max_error': max_error <= limits['max_error_m'],
        'ground_truth_waypoint_reach': reach_max <= limits['waypoint_reach_m'],
        'waypoint_vio_ground_truth': waypoint_vio_max <= limits['waypoint_vio_gt_m'],
    }
    return {
        'passed': all(checks.values()),
        'checks': checks,
        'limits_m': limits,
        'mission': {'attempt': attempt, 'waypoint_count': len(waypoint_results),
                    'start_time_s': start_ns * 1e-9, 'end_time_s': end_ns * 1e-9},
        'sample_counts': {'vio': len(vio), 'ground_truth': len(gt),
                          'px4': len(px4), 'synchronized_vio_gt': len(vio_gt)},
        'alignment_no_scale_fit': {
            'vio_to_ground_truth_yaw_rad': vio_yaw,
            'vio_to_ground_truth_translation_m': vio_translation.tolist(),
            'ground_truth_to_px4_ned_yaw_rad': gt_yaw,
            'ground_truth_to_px4_ned_translation_m': gt_translation.tolist(),
        },
        'trajectory_error_m_no_scale_fit': {
            'rmse_3d': rmse, 'max_3d': max_error,
            'final_3d': float(trajectory_error[-1]),
        },
        'waypoint_error_m': {
            'ground_truth_reach_max': reach_max,
            'vio_ground_truth_max': waypoint_vio_max,
        },
        'waypoints': waypoint_results,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--output', required=True)
    parser.add_argument('--rmse-limit-m', type=float, default=0.30)
    parser.add_argument('--max-error-limit-m', type=float, default=0.50)
    parser.add_argument('--waypoint-reach-limit-m', type=float, default=0.50)
    parser.add_argument('--waypoint-vio-gt-limit-m', type=float, default=0.50)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()
    limits = {'rmse_m': args.rmse_limit_m, 'max_error_m': args.max_error_limit_m,
              'waypoint_reach_m': args.waypoint_reach_limit_m,
              'waypoint_vio_gt_m': args.waypoint_vio_gt_limit_m}
    result = evaluate(args.bag, limits)
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    if args.require_pass and not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
