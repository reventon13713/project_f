#!/usr/bin/env python3
"""Compute no-scale VIO metrics from a recorded ROS 2 output bag.

The evaluator uses PX4/Gazebo VehicleOdometry only while VINS external
odometry publication is disabled.  It estimates yaw and translation, never a
scale factor, then reports the remaining vector error.
"""

import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from bisect import bisect_left

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@contextmanager
def readable_bag_path(bag_path):
    """Yield a rosbag2 URI, transparently expanding file-zstd bags."""
    source = Path(bag_path)
    compressed_files = sorted(source.glob('*.db3.zstd')) if source.is_dir() else []
    if not compressed_files:
        yield str(source)
        return
    if len(compressed_files) != 1:
        raise RuntimeError(
            f'expected one compressed sqlite file in {source}, found {len(compressed_files)}')
    if shutil.which('zstd') is None:
        raise RuntimeError('zstd executable is required to read a file-compressed bag')
    metadata_path = source / 'metadata.yaml'
    if not metadata_path.exists():
        raise RuntimeError(f'missing rosbag metadata: {metadata_path}')
    with tempfile.TemporaryDirectory(prefix='vio_bag_metrics_') as temporary:
        target = Path(temporary)
        compressed = compressed_files[0]
        database_name = compressed.name.removesuffix('.zstd')
        subprocess.run(
            ['zstd', '--decompress', '--quiet', str(compressed),
             '--output-dir-flat', str(target)], check=True)
        metadata = metadata_path.read_text(encoding='utf-8')
        metadata = metadata.replace(compressed.name, database_name)
        metadata = metadata.replace('compression_format: zstd', 'compression_format: ""')
        metadata = metadata.replace('compression_mode: file', 'compression_mode: ""')
        (target / 'metadata.yaml').write_text(metadata, encoding='utf-8')
        yield str(target)


def read_topic(bag_path, wanted):
    with readable_bag_path(bag_path) as readable_path:
        storage = rosbag2_py.StorageOptions(uri=readable_path, storage_id='sqlite3')
        converter = rosbag2_py.ConverterOptions('cdr', 'cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, converter)
        type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
        result = []
        while reader.has_next():
            topic, payload, timestamp_ns = reader.read_next()
            if topic in wanted:
                result.append(
                    (timestamp_ns, deserialize_message(payload, get_message(type_map[topic]))))
        return result


def nearest(samples, timestamps, timestamp_ns, max_gap_ns=150_000_000):
    index = bisect_left(timestamps, timestamp_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    if not candidates:
        return None
    candidate = min(candidates, key=lambda sample: abs(sample[0] - timestamp_ns))
    return candidate if abs(candidate[0] - timestamp_ns) <= max_gap_ns else None


def percentile(values, value):
    return float(np.percentile(np.asarray(values, dtype=float), value)) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='ros2 bag containing /odometry and PX4 ground truth')
    parser.add_argument('--output', required=True, help='metrics JSON output path')
    parser.add_argument('--last-contiguous-segment', action='store_true',
                        help='evaluate only the final VIO segment after a >0.5 s output gap')
    args = parser.parse_args()

    messages = read_topic(args.bag, {'/odometry', '/fmu/out/vehicle_odometry'})
    vio = []
    gt = []
    for timestamp_ns, message in messages:
        if hasattr(message, 'pose') and hasattr(message.pose, 'pose'):
            p = message.pose.pose.position
            v = message.twist.twist.linear
            position = np.array([p.x, p.y, p.z], dtype=float)
            velocity = np.array([v.x, v.y, v.z], dtype=float)
            if np.isfinite(position).all() and np.isfinite(velocity).all():
                vio.append((timestamp_ns, position, velocity))
        elif hasattr(message, 'position'):
            position = np.asarray(message.position, dtype=float)
            velocity = np.asarray(message.velocity, dtype=float)
            if np.isfinite(position).all() and np.isfinite(velocity).all():
                gt.append((timestamp_ns, position, velocity))
    if len(vio) < 10 or len(gt) < 10:
        raise RuntimeError(f'insufficient samples: vio={len(vio)}, gt={len(gt)}')
    segment_start_ns = vio[0][0]
    if args.last_contiguous_segment:
        gaps = [vio[index][0] - vio[index - 1][0] for index in range(1, len(vio))]
        boundaries = [index for index, gap in enumerate(gaps, start=1)
                      if gap > 500_000_000]
        if boundaries:
            vio = vio[boundaries[-1]:]
            segment_start_ns = vio[0][0]

    gt_times = [item[0] for item in gt]
    pairs = []
    for timestamp_ns, position, velocity in vio:
        matched = nearest(gt, gt_times, timestamp_ns)
        if matched is not None:
            pairs.append((timestamp_ns, position, velocity, matched[1], matched[2]))
    if len(pairs) < 10:
        raise RuntimeError(f'insufficient time-synchronised samples: {len(pairs)}')

    vio_position_enu = np.asarray([pair[1] for pair in pairs])
    # VINS world positions use ROS ENU; PX4 VehicleOdometry positions use
    # NED.  Apply the fixed axis convention before estimating arbitrary yaw
    # and translation.  This is not a fitted rotation or scale factor.
    vio_position = vio_position_enu[:, [1, 0, 2]].copy()
    vio_position[:, 2] *= -1.0
    gt_position = np.asarray([pair[3] for pair in pairs])
    dv = np.diff(vio_position[:, :2], axis=0)
    dg = np.diff(gt_position[:, :2], axis=0)
    motion = ((np.linalg.norm(dv, axis=1) > 0.02) &
              (np.linalg.norm(dg, axis=1) > 0.02))
    if np.count_nonzero(motion) < 5:
        raise RuntimeError('insufficient XY displacement for yaw alignment')
    dv, dg = dv[motion], dg[motion]
    dot = float(np.sum(dv[:, 0] * dg[:, 0] + dv[:, 1] * dg[:, 1]))
    cross = float(np.sum(dv[:, 0] * dg[:, 1] - dv[:, 1] * dg[:, 0]))
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    translation = np.mean(gt_position - (rotation @ vio_position.T).T, axis=0)
    aligned = (rotation @ vio_position.T).T + translation
    error = aligned - gt_position
    error_norm = np.linalg.norm(error, axis=1)

    pairwise_3d, pairwise_xy, pairwise_z = [], [], []
    # Downsample to roughly 10 Hz, then use all well-separated pose pairs.
    indices = range(0, len(pairs), max(1, len(pairs) // 250))
    indices = list(indices)
    for left, i in enumerate(indices):
        for j in indices[left + 1:]:
            gt_delta = gt_position[j] - gt_position[i]
            vio_delta = vio_position[j] - vio_position[i]
            gt_3d, vio_3d = np.linalg.norm(gt_delta), np.linalg.norm(vio_delta)
            gt_xy, vio_xy = np.linalg.norm(gt_delta[:2]), np.linalg.norm(vio_delta[:2])
            if gt_3d > 0.3 and vio_3d > 1e-4:
                pairwise_3d.append(float(vio_3d / gt_3d))
            if gt_xy > 0.3 and vio_xy > 1e-4:
                pairwise_xy.append(float(vio_xy / gt_xy))
            if abs(gt_delta[2]) > 0.3 and abs(vio_delta[2]) > 1e-4:
                pairwise_z.append(float(abs(vio_delta[2]) / abs(gt_delta[2])))

    step = np.linalg.norm(np.diff(vio_position, axis=0), axis=1)
    output = {
        'alignment': {'yaw_rad_no_scale_fit': yaw,
                      'translation_m_no_scale_fit': translation.tolist()},
        'sample_counts': {'vio': len(vio), 'ground_truth': len(gt), 'paired': len(pairs)},
        'segment_start_time_s': segment_start_ns * 1e-9,
        'position_error_m_no_scale_fit': {
            'rmse_3d': float(math.sqrt(np.mean(error_norm ** 2))),
            'rmse_xy': float(math.sqrt(np.mean(np.sum(error[:, :2] ** 2, axis=1)))),
            'rmse_z': float(math.sqrt(np.mean(error[:, 2] ** 2))),
            'max_3d': float(np.max(error_norm)),
            'final_3d': float(error_norm[-1]),
        },
        'vio': {'max_speed_mps': float(max(np.linalg.norm(pair[2]) for pair in pairs)),
                'max_pose_step_m': float(np.max(step))},
        'pairwise_scale_no_fit': {
            key: {'count': len(values), 'median': percentile(values, 50),
                  'mean': float(np.mean(values)) if values else None,
                  'stddev': float(np.std(values)) if values else None,
                  'p05': percentile(values, 5), 'p95': percentile(values, 95),
                  'min': float(np.min(values)) if values else None,
                  'max': float(np.max(values)) if values else None}
            for key, values in {'3d': pairwise_3d, 'xy': pairwise_xy, 'z': pairwise_z}.items()
        },
    }
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write('\n')


if __name__ == '__main__':
    main()
