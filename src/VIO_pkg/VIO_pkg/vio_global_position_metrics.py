#!/usr/bin/env python3
"""Compare recorded VIO-derived WGS84 output with PX4 fused global pose."""

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--output', required=True)
    parser.add_argument('--maximum-rmse-m', type=float, default=0.50)
    parser.add_argument('--maximum-error-m', type=float, default=1.00)
    args = parser.parse_args()

    messages = read_topic(
        args.bag, {'/vio/global_position', '/fmu/out/vehicle_global_position'},
        include_topic=True)
    vio = []
    px4 = []
    for topic, timestamp_ns, message in messages:
        if topic == '/vio/global_position':
            point = (message.latitude, message.longitude, message.altitude)
            if np.isfinite(point).all():
                vio.append((timestamp_ns, point))
        else:
            point = (message.lat, message.lon, message.alt_ellipsoid)
            if np.isfinite(point).all() and abs(message.lat) <= 90.0:
                px4.append((timestamp_ns, point))
    px4_times = [sample[0] for sample in px4]
    errors = []
    for timestamp_ns, point in vio:
        match = nearest(px4, px4_times, timestamp_ns)
        if match is not None:
            errors.append(float(np.linalg.norm(
                geodetic_to_ecef(*point) - geodetic_to_ecef(*match[1]))))
    if len(errors) < 10:
        raise RuntimeError(
            f'insufficient global samples: vio={len(vio)}, px4={len(px4)}, '
            f'paired={len(errors)}')
    rmse = math.sqrt(float(np.mean(np.square(errors))))
    maximum = max(errors)
    output = {
        'passed': rmse <= args.maximum_rmse_m and maximum <= args.maximum_error_m,
        'sample_counts': {'vio': len(vio), 'px4': len(px4), 'paired': len(errors)},
        'error_m': {'rmse': rmse, 'maximum': maximum,
                    'p95': float(np.percentile(errors, 95))},
        'limits_m': {'rmse': args.maximum_rmse_m,
                     'maximum': args.maximum_error_m},
    }
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(output, indent=2, sort_keys=True))
    if not output['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
