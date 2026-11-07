#!/usr/bin/env python3
"""Publish a strictly monotonic camera stream for VINS-Fusion."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import math
from collections import deque


class VioCameraRelay(Node):
    def __init__(self):
        super().__init__('vio_camera_relay')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/vio/camera/image_raw')
        # Gazebo Classic publishes one extra uniquely stamped copy about 4 ms
        # after some camera renders.  A 25 Hz cap removes only those copies
        # (40 ms gate), while retaining the real 20 Hz renders whose stamp
        # interval is about 48--52 ms.
        self.declare_parameter('target_rate_hz', 25.0)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        target_rate_hz = float(self.get_parameter('target_rate_hz').value)
        self.minimum_period_ns = (
            int(1_000_000_000 / target_rate_hz) if target_rate_hz > 0.0 else 0)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.create_subscription(Image, input_topic, self.callback, 10)
        self.last_stamp_ns = -1
        self.last_forwarded_stamp_ns = -1
        self.raw_count = 0
        self.forwarded = 0
        self.dropped = 0
        self.rate_dropped = 0
        self.short_interval_count = 0
        self.output_intervals_ns = deque(maxlen=5000)
        self.create_timer(10.0, self.report_statistics)
        self.get_logger().info(
            f'Monotonic camera relay: {input_topic} -> {output_topic} '
            f'({"unlimited" if target_rate_hz <= 0.0 else f"max {target_rate_hz:g} Hz"})')

    def callback(self, message):
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.raw_count += 1
        if self.last_stamp_ns >= 0 and stamp_ns - self.last_stamp_ns <= 10_000_000:
            self.short_interval_count += 1
        if stamp_ns <= self.last_stamp_ns:
            self.dropped += 1
            self.get_logger().warning(
                f'Dropped duplicate/out-of-order camera frame ({self.dropped} total).',
                throttle_duration_sec=2.0)
            return
        self.last_stamp_ns = stamp_ns
        # The gate rejects Gazebo's shortly-after-render duplicate but does
        # not decimate the actual 20 Hz sensor stream.  Keep the original
        # simulation stamp so image/IMU integration uses one clock domain.
        if (self.last_forwarded_stamp_ns >= 0 and
                stamp_ns - self.last_forwarded_stamp_ns < self.minimum_period_ns):
            self.rate_dropped += 1
            return
        if self.last_forwarded_stamp_ns >= 0:
            self.output_intervals_ns.append(stamp_ns - self.last_forwarded_stamp_ns)
        self.last_forwarded_stamp_ns = stamp_ns
        self.forwarded += 1
        self.publisher.publish(message)

    def report_statistics(self):
        if not self.output_intervals_ns:
            return
        intervals_ms = sorted(value * 1e-6 for value in self.output_intervals_ns)
        mean_ms = sum(intervals_ms) / len(intervals_ms)
        variance = sum((value - mean_ms) ** 2 for value in intervals_ms) / len(intervals_ms)
        p95_index = min(len(intervals_ms) - 1, math.ceil(0.95 * len(intervals_ms)) - 1)
        self.get_logger().info(
            'camera relay stats: raw=%d forwarded=%d duplicate_or_ooo=%d '
            'rate_dropped=%d input_le_10ms=%d output_fps=%.2f '
            'interval_ms(mean=%.2f std=%.2f min=%.2f max=%.2f p95=%.2f)' % (
                self.raw_count, self.forwarded, self.dropped, self.rate_dropped,
                self.short_interval_count, 1000.0 / mean_ms, mean_ms,
                math.sqrt(variance), intervals_ms[0], intervals_ms[-1],
                intervals_ms[p95_index]))


def main(args=None):
    rclpy.init(args=args)
    node = VioCameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
