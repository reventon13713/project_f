#!/usr/bin/env python3
"""Measure PX4 ground-truth motion over VINS initialization windows."""

import argparse
import json
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from VIO_pkg.vio_bag_metrics import readable_bag_path


def parse_window(value):
    try:
        label, start, end = value.split(':', 2)
        start, end = float(start), float(end)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            'window must have the form LABEL:START_SECONDS:END_SECONDS') from error
    if not label or not math.isfinite(start) or not math.isfinite(end) or end <= start:
        raise argparse.ArgumentTypeError('window label/times are invalid')
    return label, start, end


def read_ground_truth(bag_path):
    with readable_bag_path(bag_path) as readable_path:
        storage = rosbag2_py.StorageOptions(uri=readable_path, storage_id='sqlite3')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, rosbag2_py.ConverterOptions('cdr', 'cdr'))
        type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
        message_type = get_message(type_map['/fmu/out/vehicle_odometry'])
        clock_type = get_message(type_map['/clock'])
        recorded_samples = []
        clocks = []
        while reader.has_next():
            topic, payload, recorded_ns = reader.read_next()
            if topic == '/clock':
                message = deserialize_message(payload, clock_type)
                sim_time = message.clock.sec + message.clock.nanosec * 1e-9
                clocks.append((recorded_ns, sim_time))
                continue
            if topic != '/fmu/out/vehicle_odometry':
                continue
            message = deserialize_message(payload, message_type)
            position = np.asarray(message.position, dtype=float)
            quaternion = np.asarray(message.q, dtype=float)
            if np.isfinite(position).all() and np.isfinite(quaternion).all():
                norm = np.linalg.norm(quaternion)
                if norm > 0.0:
                    recorded_samples.append((recorded_ns, position, quaternion / norm))
        if len(clocks) < 2:
            raise RuntimeError('insufficient /clock samples for simulation-time mapping')
        clock_recorded = np.asarray([item[0] for item in clocks], dtype=np.float64)
        clock_sim = np.asarray([item[1] for item in clocks], dtype=np.float64)
        sample_recorded = np.asarray([item[0] for item in recorded_samples], dtype=np.float64)
        sample_sim = np.interp(sample_recorded, clock_recorded, clock_sim)
        return [(float(sim_time), sample[1], sample[2])
                for sim_time, sample in zip(sample_sim, recorded_samples)]


def summarize(samples, start, end):
    selected = [sample for sample in samples if start <= sample[0] <= end]
    if len(selected) < 2:
        raise RuntimeError(f'insufficient ground-truth samples in {start:.3f}~{end:.3f}s')
    positions = np.asarray([sample[1] for sample in selected])
    displacement = positions[-1] - positions[0]
    steps = np.diff(positions, axis=0)
    q_start, q_end = selected[0][2], selected[-1][2]
    rotation_deg = math.degrees(2.0 * math.acos(
        float(np.clip(abs(np.dot(q_start, q_end)), -1.0, 1.0))))
    return {
        'requested_start_s': start,
        'requested_end_s': end,
        'sample_start_s': selected[0][0],
        'sample_end_s': selected[-1][0],
        'sample_count': len(selected),
        'displacement_ned_m': displacement.tolist(),
        'displacement_norm_m': float(np.linalg.norm(displacement)),
        'path_length_m': float(np.sum(np.linalg.norm(steps, axis=1))),
        'endpoint_rotation_deg': rotation_deg,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='ros2 bag containing PX4 VehicleOdometry')
    parser.add_argument('--window', action='append', type=parse_window, required=True,
                        help='LABEL:START_SECONDS:END_SECONDS; may be repeated')
    parser.add_argument('--output', required=True, help='metrics JSON output path')
    args = parser.parse_args()
    samples = read_ground_truth(args.bag)
    output = {label: summarize(samples, start, end)
              for label, start, end in args.window}
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write('\n')


if __name__ == '__main__':
    main()
