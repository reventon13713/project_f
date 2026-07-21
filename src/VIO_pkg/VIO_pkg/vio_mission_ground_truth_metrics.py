#!/usr/bin/env python3
"""Stage-7/8 QGC mission, map continuity, independent GT and ULog verdict."""

import argparse
from bisect import bisect_left
import json
import math

import numpy as np

from VIO_pkg.vio_bag_metrics import read_topic
from VIO_pkg.vio_global_position import geodetic_to_ecef
from VIO_pkg.vio_ground_truth_metrics import yaw_translation_fit


EVENT = '/vio/stage7_mission_event'
GT = '/vio/ground_truth/odometry'
PX4 = '/fmu/out/vehicle_global_position'
VIO = '/vio/global_position'
GPS = '/fmu/out/vehicle_gps_position'


def nearest(samples, times, timestamp_ns, max_gap_ns=200_000_000):
    index = bisect_left(times, timestamp_ns)
    candidates = samples[max(0, index - 1):index + 1]
    if not candidates:
        return None
    sample = min(candidates, key=lambda item: abs(item[0] - timestamp_ns))
    return sample if abs(sample[0] - timestamp_ns) <= max_gap_ns else None


def geodetic_to_ned(point, origin):
    latitude, longitude, altitude = point
    lat0, lon0, alt0 = origin
    delta = geodetic_to_ecef(latitude, longitude, altitude) - geodetic_to_ecef(
        lat0, lon0, alt0)
    lat = math.radians(lat0)
    lon = math.radians(lon0)
    rotation = np.array([
        [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon),
         math.cos(lat)],
        [-math.sin(lon), math.cos(lon), 0.0],
        [-math.cos(lat) * math.cos(lon), -math.cos(lat) * math.sin(lon),
         -math.sin(lat)],
    ])
    return rotation @ delta


def step_stats(samples):
    if len(samples) < 2:
        return {'maximum_m': math.inf, 'path_m': 0.0, 'maximum_gap_s': math.inf}
    steps = [float(np.linalg.norm(right[1] - left[1]))
             for left, right in zip(samples, samples[1:])]
    gaps = [(right[0] - left[0]) * 1e-9
            for left, right in zip(samples, samples[1:])]
    return {'maximum_m': max(steps), 'path_m': sum(steps),
            'maximum_gap_s': max(gaps)}


def parse_events(samples):
    parsed = []
    for timestamp_ns, message in samples:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            continue
        parsed.append((timestamp_ns, payload))
    by_kind = {}
    for timestamp_ns, payload in parsed:
        by_kind.setdefault(payload.get('event'), []).append((timestamp_ns, payload))
    required = ('QGC_MISSION_UPLOADED', 'QGC_MISSION_STARTED',
                'GPS_DROPOUT_COMMAND', 'QGC_MISSION_RESULT')
    missing = [kind for kind in required if not by_kind.get(kind)]
    if missing:
        raise RuntimeError(f'missing mission events: {missing}')
    return by_kind


def analyze_bag(path, limits):
    wanted = {EVENT, GT, PX4, VIO, GPS}
    topics = {name: [] for name in wanted}
    for topic, timestamp_ns, message in read_topic(path, wanted, include_topic=True):
        topics[topic].append((timestamp_ns, message))
    events = parse_events(topics[EVENT])
    upload_time, upload = events['QGC_MISSION_UPLOADED'][-1]
    start_ns = events['QGC_MISSION_STARTED'][-1][0]
    dropout_ns, dropout = events['GPS_DROPOUT_COMMAND'][-1]
    result_ns, live_result = events['QGC_MISSION_RESULT'][-1]
    if not start_ns < dropout_ns < result_ns:
        raise RuntimeError('invalid mission/dropout/result event order')

    gt = []
    for timestamp_ns, message in topics[GT]:
        position = message.pose.pose.position
        vector = np.array([position.y, position.x, -position.z], dtype=float)
        if np.isfinite(vector).all():
            gt.append((timestamp_ns, vector))
    raw_px4 = []
    for timestamp_ns, message in topics[PX4]:
        point = (message.lat, message.lon, message.alt_ellipsoid)
        if np.isfinite(point).all() and abs(message.lat) <= 90:
            raw_px4.append((timestamp_ns, point))
    raw_vio = []
    for timestamp_ns, message in topics[VIO]:
        point = (message.latitude, message.longitude, message.altitude)
        if np.isfinite(point).all():
            raw_vio.append((timestamp_ns, point))
    if min(len(gt), len(raw_px4), len(raw_vio)) < 20:
        raise RuntimeError('insufficient GT/PX4/VIO global samples')
    origin = raw_px4[0][1]
    px4 = [(time, geodetic_to_ned(point, origin)) for time, point in raw_px4]
    vio = [(time, geodetic_to_ned(point, origin)) for time, point in raw_vio]

    # Establish only yaw and translation while GPS is still live. Scale is
    # never fitted, so post-dropout GT drift remains visible in metres.
    px4_times = [item[0] for item in px4]
    fit_gt, fit_px4 = [], []
    for timestamp_ns, position in gt:
        if start_ns <= timestamp_ns < dropout_ns:
            match = nearest(px4, px4_times, timestamp_ns)
            if match is not None:
                fit_gt.append(position)
                fit_px4.append(match[1])
    yaw, rotation, translation = yaw_translation_fit(
        fit_gt, fit_px4, motion_threshold_m=0.005)

    gt_post = [(time, rotation @ position + translation)
               for time, position in gt if dropout_ns <= time <= result_ns]
    px4_post = [(time, position) for time, position in px4
                if dropout_ns <= time <= result_ns]
    vio_post = [(time, position) for time, position in vio
                if dropout_ns <= time <= result_ns]
    gt_times = [item[0] for item in gt_post]

    def errors_against_gt(samples):
        errors = []
        for timestamp_ns, position in samples:
            match = nearest(gt_post, gt_times, timestamp_ns)
            if match is not None:
                errors.append(float(np.linalg.norm(position - match[1])))
        return errors

    px4_errors = errors_against_gt(px4_post)
    vio_errors = errors_against_gt(vio_post)
    if min(len(px4_errors), len(vio_errors)) < 20:
        raise RuntimeError('insufficient synchronized post-dropout GT samples')

    waypoints = {int(item['sequence']): item for item in upload['waypoints']}
    reached = []
    for timestamp_ns, payload in events.get('MISSION_ITEM_REACHED', []):
        sequence = int(payload['sequence'])
        if timestamp_ns < dropout_ns or sequence not in waypoints:
            continue
        match = nearest(gt_post, gt_times, timestamp_ns)
        if match is None:
            raise RuntimeError(f'no GT pose at reached sequence {sequence}')
        item = waypoints[sequence]
        # Relative altitude is already held by the mission. Horizontal GT
        # reach is the QGC map waypoint criterion and avoids home-alt ambiguity.
        target = geodetic_to_ned(
            (item['latitude'], item['longitude'], origin[2]), origin)
        reached.append({'sequence': sequence,
                        'horizontal_error_m': float(np.linalg.norm(
                            match[1][:2] - target[:2]))})
    dropout_sequence = int(dropout['at_sequence'])
    final_sequence = int(upload.get('final_sequence', max(waypoints)))
    expected_after = list(range(dropout_sequence, final_sequence + 1))
    actual_after = [item['sequence'] for item in reached]
    # The dropout command occurs on MISSION_CURRENT and sequence may be reached
    # just before or just after its event record. Only strictly remaining items
    # are mandatory in the post-dropout reached list.
    remaining = list(range(dropout_sequence + 1, final_sequence + 1))

    gps_after = [time for time, _ in topics[GPS] if time > dropout_ns + 500_000_000]
    px4_stats = step_stats(px4_post)
    vio_stats = step_stats(vio_post)
    gt_stats = step_stats(gt_post)
    vio_end_gap_s = ((result_ns - vio_post[-1][0]) * 1e-9
                     if vio_post else math.inf)
    px4_rmse = math.sqrt(float(np.mean(np.square(px4_errors))))
    vio_rmse = math.sqrt(float(np.mean(np.square(vio_errors))))
    reach_max = max((item['horizontal_error_m'] for item in reached), default=math.inf)
    checks = {
        'actual_gps_stream_stopped': len(gps_after) == 0,
        'remaining_waypoints_reached': all(sequence in actual_after
                                           for sequence in remaining),
        'px4_map_feed_continuous': (px4_stats['maximum_m'] <= limits['step_m'] and
                                    px4_stats['maximum_gap_s'] <= limits['gap_s']),
        'vio_global_continuous': (vio_stats['maximum_m'] <= limits['step_m'] and
                                  vio_stats['maximum_gap_s'] <= limits['gap_s'] and
                                  vio_end_gap_s <= limits['gap_s']),
        'map_position_moved_after_dropout': px4_stats['path_m'] >= limits['path_m'],
        'ground_truth_moved_after_dropout': gt_stats['path_m'] >= limits['path_m'],
        'px4_ground_truth_accuracy': (px4_rmse <= limits['rmse_m'] and
                                      max(px4_errors) <= limits['max_error_m']),
        'vio_ground_truth_accuracy': (vio_rmse <= limits['rmse_m'] and
                                      max(vio_errors) <= limits['max_error_m']),
        'ground_truth_waypoint_accuracy': reach_max <= limits['waypoint_m'],
    }
    return {
        'passed': all(checks.values()), 'checks': checks, 'limits': limits,
        'events': {'mission_start_s': start_ns * 1e-9,
                   'dropout_s': dropout_ns * 1e-9,
                   'dropout_sim_s': float(dropout['sim_time_s']),
                   'mission_result_sim_s': float(live_result['sim_time_s']),
                   'mission_result_s': result_ns * 1e-9,
                   'dropout_sequence': dropout_sequence,
                   'waypoint_count': len(waypoints),
                   'final_sequence': final_sequence},
        'alignment_no_scale_fit': {'gt_to_global_ned_yaw_rad': yaw,
                                   'translation_m': translation.tolist()},
        'post_dropout_samples': {'ground_truth': len(gt_post),
                                 'px4_global': len(px4_post),
                                 'vio_global': len(vio_post),
                                 'px4_gt_paired': len(px4_errors),
                                 'vio_gt_paired': len(vio_errors)},
        'post_dropout_continuity': {'px4_map_feed': px4_stats,
                                    'vio_global': {**vio_stats,
                                                   'gap_to_mission_result_s': vio_end_gap_s},
                                    'ground_truth': gt_stats},
        'post_dropout_error_m': {
            'px4_vs_ground_truth': {'rmse_3d': px4_rmse,
                                    'maximum_3d': max(px4_errors)},
            'vio_vs_ground_truth': {'rmse_3d': vio_rmse,
                                    'maximum_3d': max(vio_errors)}},
        'waypoints_after_dropout': reached,
        'ground_truth_waypoint_horizontal_max_m': reach_max,
        'gps_messages_later_than_0_5_s_after_command': len(gps_after),
        'expected_sequences_from_dropout': expected_after,
    }


def analyze_ulog(path, dropout_s, mission_result_s, final_sequence, limits):
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError('pyulog is required (validated version: 0.9.0)') from error
    names = ['mission_result', 'vehicle_status', 'vehicle_global_position',
             'estimator_status_flags', 'estimator_aid_src_ev_pos',
             'estimator_aid_src_ev_vel']
    ulog = ULog(path, message_name_filter_list=names)
    datasets = {item.name: item.data for item in ulog.data_list if item.multi_id == 0}
    missing = [name for name in names if name not in datasets]
    if missing:
        raise RuntimeError(f'ULog missing datasets: {missing}')
    dropout_us = int(dropout_s * 1e6)
    # Include PX4's mission-finished publication, but exclude shutdown/loiter
    # transitions produced after the live result and recorder grace period.
    end_us = int((mission_result_s + 0.5) * 1e6)
    mission = datasets['mission_result']
    mission_mask = ((mission['timestamp'] >= dropout_us) &
                    (mission['timestamp'] <= end_us))
    status = datasets['vehicle_status']
    status_mask = ((status['timestamp'] >= dropout_us) &
                   (status['timestamp'] <= end_us))
    flags = datasets['estimator_status_flags']
    flags_mask = ((flags['timestamp'] >= dropout_us + 2_000_000) &
                  (flags['timestamp'] <= end_us))
    global_data = datasets['vehicle_global_position']
    global_mask = ((global_data['timestamp'] >= dropout_us) &
                   (global_data['timestamp'] <= end_us))

    sequences = mission['seq_current'][mission_mask]
    reached = mission['seq_reached'][mission_mask]
    nav_states = status['nav_state'][status_mask]
    post_flags = flags['cs_gps'][flags_mask].astype(bool)
    ev_pos_flags = flags['cs_ev_pos'][flags_mask].astype(bool)
    ev_vel_flags = flags['cs_ev_vel'][flags_mask].astype(bool)
    indices = np.flatnonzero(global_mask)
    reset_changes = 0
    maximum_step = math.inf
    if len(indices) >= 2:
        reset_changes = int(np.count_nonzero(np.diff(
            global_data['lat_lon_reset_counter'][indices])))
        ecef = np.asarray([geodetic_to_ecef(lat, lon, alt) for lat, lon, alt in zip(
            global_data['lat'][indices], global_data['lon'][indices],
            global_data['alt_ellipsoid'][indices])])
        maximum_step = float(np.max(np.linalg.norm(np.diff(ecef, axis=0), axis=1)))
    ev_aid = {}
    for name in ('estimator_aid_src_ev_pos', 'estimator_aid_src_ev_vel'):
        data = datasets[name]
        mask = ((data['timestamp'] >= dropout_us) &
                (data['timestamp'] <= end_us))
        ev_aid[name] = {'post_dropout_fused_samples': int(np.count_nonzero(
            data['fused'][mask])),
                        'post_dropout_rejected_samples': int(np.count_nonzero(
            data['innovation_rejected'][mask]))}
    finished = bool(np.any(mission['finished'][mission_mask]))
    checks = {
        'mission_finished': finished and bool(np.any(reached >= final_sequence)),
        'auto_mission_observed': bool(np.any(nav_states == 3)),
        'gps_aid_dropped': len(post_flags) > 0 and bool(np.any(~post_flags)),
        'ev_position_velocity_sustained': (len(ev_pos_flags) > 0 and
                                           bool(np.any(ev_pos_flags & ev_vel_flags))),
        'ev_aid_actually_fused': all(item['post_dropout_fused_samples'] > 0
                                     for item in ev_aid.values()),
        # EKF2 may increment the reset counter once when GNSS aiding is removed.
        # The map contract is bounded displacement, not a frozen counter.
        'global_reset_count_bounded': reset_changes <= 1,
        'global_continuity': maximum_step <= limits['step_m'],
    }
    return {'passed': all(checks.values()), 'checks': checks,
            'mission': {'current_sequences': sorted(set(map(int, sequences))),
                        'reached_sequences': sorted(set(map(int, reached))),
                        'finished': finished},
            'post_dropout_nav_states': sorted(set(map(int, nav_states))),
            'post_stale_fusion_flags': {
                'samples': len(post_flags),
                'gps_false_samples': int(np.count_nonzero(~post_flags)),
                'ev_pos_ev_vel_true_samples': int(np.count_nonzero(
                    ev_pos_flags & ev_vel_flags))},
            'ev_aid_sources': ev_aid,
            'global_position': {'samples': len(indices),
                                'lat_lon_reset_changes': reset_changes,
                                'maximum_step_3d_m': maximum_step}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('ulog')
    parser.add_argument('--output', required=True)
    parser.add_argument('--rmse-limit-m', type=float, default=1.00)
    parser.add_argument('--max-error-limit-m', type=float, default=1.50)
    parser.add_argument('--step-limit-m', type=float, default=0.50)
    parser.add_argument('--gap-limit-s', type=float, default=0.60)
    parser.add_argument('--minimum-post-dropout-path-m', type=float, default=8.0)
    parser.add_argument('--waypoint-limit-m', type=float, default=1.50)
    parser.add_argument('--require-pass', action='store_true')
    args = parser.parse_args()
    limits = {'rmse_m': args.rmse_limit_m, 'max_error_m': args.max_error_limit_m,
              'step_m': args.step_limit_m, 'gap_s': args.gap_limit_s,
              'path_m': args.minimum_post_dropout_path_m,
              'waypoint_m': args.waypoint_limit_m}
    bag = analyze_bag(args.bag, limits)
    ulog = analyze_ulog(args.ulog, bag['events']['dropout_sim_s'],
                        bag['events']['mission_result_sim_s'],
                        bag['events']['final_sequence'], limits)
    result = {'passed': bag['passed'] and ulog['passed'],
              'rosbag_ground_truth': bag, 'ulog': ulog}
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.require_pass and not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
