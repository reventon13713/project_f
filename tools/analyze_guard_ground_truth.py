#!/usr/bin/env python3
"""Compare an estimator guard event in simulation time with recorded PX4 motion."""

import argparse
from bisect import bisect_left
import json

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from VIO_pkg.vio_bag_metrics import readable_bag_path


def nearest(samples, timestamp_ns):
    times = [sample[0] for sample in samples]
    index = bisect_left(times, timestamp_ns)
    candidates = samples[max(0, index - 1):index + 1]
    return min(candidates, key=lambda sample: abs(sample[0] - timestamp_ns))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--event-sim-time', required=True, type=float)
    parser.add_argument('--interval', type=float, default=0.1)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    clock = []
    ground_truth = []
    vio = []
    with readable_bag_path(args.bag) as bag:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, payload, storage_ns = reader.read_next()
            if topic not in {'/clock', '/odometry', '/fmu/out/vehicle_odometry'}:
                continue
            message = deserialize_message(payload, get_message(types[topic]))
            if topic == '/clock':
                sim_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
                clock.append((sim_ns, storage_ns))
            elif topic == '/odometry':
                p = message.pose.pose.position
                vio.append((storage_ns, np.array([p.x, p.y, p.z], dtype=float)))
            else:
                ground_truth.append(
                    (storage_ns, np.asarray(message.position, dtype=float)))

    clock.sort()
    ground_truth.sort()
    vio.sort()
    event_sim_ns = round(args.event_sim_time * 1e9)
    previous_sim_ns = round((args.event_sim_time - args.interval) * 1e9)
    event_storage_ns = nearest(clock, event_sim_ns)[1]
    previous_storage_ns = nearest(clock, previous_sim_ns)[1]
    gt_before = nearest(ground_truth, previous_storage_ns)
    gt_event = nearest(ground_truth, event_storage_ns)
    vio_before = nearest(vio, previous_storage_ns)
    vio_event = nearest(vio, event_storage_ns)

    output = {
        'event_sim_time_s': args.event_sim_time,
        'interval_s': args.interval,
        'clock_mapping_error_before_ms':
            abs(nearest(clock, previous_sim_ns)[0] - previous_sim_ns) / 1e6,
        'clock_mapping_error_event_ms':
            abs(nearest(clock, event_sim_ns)[0] - event_sim_ns) / 1e6,
        'px4_ground_truth_step_m': float(np.linalg.norm(gt_event[1] - gt_before[1])),
        'published_vio_step_m': float(np.linalg.norm(vio_event[1] - vio_before[1])),
        'px4_sample_storage_gap_ms': (gt_event[0] - gt_before[0]) / 1e6,
        'vio_sample_storage_gap_ms': (vio_event[0] - vio_before[0]) / 1e6,
        'guard_note': ('The rejected estimator state is not published; the VIO step only '
                       'measures the nearest safe outputs surrounding the event.'),
    }
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write('\n')


if __name__ == '__main__':
    main()
