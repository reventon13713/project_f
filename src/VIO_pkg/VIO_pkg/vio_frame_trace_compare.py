import argparse
import json
import re

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from VIO_pkg.vio_bag_metrics import readable_bag_path


TRACE_PATTERN = re.compile(
    r'Camera callback trace: sequence=(?P<sequence>\d+), '
    r'timestamp=(?P<timestamp>\d+(?:\.\d+)?), '
    r'fingerprint=(?P<fingerprint>0x[0-9a-fA-F]+)')


def timestamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def parse_trace(log_path):
    records = []
    with open(log_path, encoding='utf-8', errors='replace') as stream:
        for line in stream:
            match = TRACE_PATTERN.search(line)
            if not match:
                continue
            records.append({
                'sequence': int(match.group('sequence')),
                'timestamp_ns': round(float(match.group('timestamp')) * 1e9),
                'fingerprint': match.group('fingerprint').lower(),
            })
    if not records:
        raise RuntimeError(f'no camera callback trace records in {log_path}')
    return records


def read_bag_timestamps(bag_path, topic):
    timestamps = []
    with readable_bag_path(bag_path) as readable_path:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=readable_path, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        type_map = {item.name: item.type
                    for item in reader.get_all_topics_and_types()}
        if topic not in type_map:
            raise RuntimeError(f'{topic} is not present in {bag_path}')
        message_type = get_message(type_map[topic])
        while reader.has_next():
            current_topic, payload, _ = reader.read_next()
            if current_topic != topic:
                continue
            message = deserialize_message(payload, message_type)
            timestamps.append(timestamp_ns(message.header.stamp))
    return timestamps


def summarize(trace, bag_timestamps):
    trace_ns = [item['timestamp_ns'] for item in trace]
    if not bag_timestamps:
        raise RuntimeError('bag topic contains no messages')
    overlap_start = max(trace_ns[0], bag_timestamps[0])
    overlap_end = min(trace_ns[-1], bag_timestamps[-1])
    trace_overlap = [value for value in trace_ns
                     if overlap_start <= value <= overlap_end]
    bag_overlap = [value for value in bag_timestamps
                   if overlap_start <= value <= overlap_end]
    trace_set = set(trace_overlap)
    bag_set = set(bag_overlap)
    live_only = sorted(trace_set - bag_set)
    bag_only = sorted(bag_set - trace_set)
    return {
        'trace_frame_count': len(trace_ns),
        'bag_frame_count': len(bag_timestamps),
        'trace_sequence_gap_count': sum(
            current['sequence'] != previous['sequence'] + 1
            for previous, current in zip(trace, trace[1:])),
        'trace_nonmonotonic_count': sum(
            current <= previous for previous, current
            in zip(trace_ns, trace_ns[1:])),
        'bag_nonmonotonic_count': sum(
            current <= previous for previous, current
            in zip(bag_timestamps, bag_timestamps[1:])),
        'overlap_start_s': overlap_start / 1e9,
        'overlap_end_s': overlap_end / 1e9,
        'trace_overlap_count': len(trace_overlap),
        'bag_overlap_count': len(bag_overlap),
        'matched_unique_timestamp_count': len(trace_set & bag_set),
        'trace_only_unique_timestamp_count': len(live_only),
        'bag_only_unique_timestamp_count': len(bag_only),
        'first_trace_only_timestamps_s': [value / 1e9 for value in live_only[:20]],
        'first_bag_only_timestamps_s': [value / 1e9 for value in bag_only[:20]],
    }


def main():
    parser = argparse.ArgumentParser(
        description='Compare VINS camera callback timestamps with a rosbag topic.')
    parser.add_argument('bag')
    parser.add_argument('trace_log')
    parser.add_argument('--topic', default='/vio/camera/vins_image_raw')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = summarize(parse_trace(args.trace_log),
                       read_bag_timestamps(args.bag, args.topic))
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
