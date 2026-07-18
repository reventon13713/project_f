#!/usr/bin/env python3
"""Deterministic EuRoC mono+IMU player for validating the ROS 2 VINS node."""

import argparse
import csv
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, Imu


def read_csv(path):
    with open(path, newline='') as stream:
        return [row for row in csv.reader(stream) if row and not row[0].startswith('#')]


class EurocPlayer(Node):
    def __init__(self, dataset, speed):
        super().__init__('euroc_player')
        self.dataset = Path(dataset)
        self.speed = speed
        mav0 = self.dataset / 'mav0'
        self.images = read_csv(mav0 / 'cam0' / 'data.csv')
        self.imu = read_csv(mav0 / 'imu0' / 'data.csv')
        if not self.images or not self.imu:
            raise RuntimeError('EuRoC cam0/imu0 CSV data is empty')

        # The ROS 2 VINS port subscribes with the default RELIABLE QoS.
        # Use the matching publisher profile for this reference test.
        self.image_pub = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu0', 10)
        self.image_index = 0
        self.imu_index = 0
        self.start_stamp_ns = min(int(self.images[0][0]), int(self.imu[0][0]))
        self.start_wall_ns = time.monotonic_ns()
        self.timer = self.create_timer(0.001, self.publish_due)
        self.get_logger().info(
            f'Playing {len(self.images)} images and {len(self.imu)} IMU samples at {speed}x')

    @staticmethod
    def stamp(timestamp_ns):
        stamp = Time()
        stamp.sec = timestamp_ns // 1_000_000_000
        stamp.nanosec = timestamp_ns % 1_000_000_000
        return stamp

    def publish_due(self):
        elapsed_ns = int((time.monotonic_ns() - self.start_wall_ns) * self.speed)
        target_ns = self.start_stamp_ns + elapsed_ns

        while self.imu_index < len(self.imu) and int(self.imu[self.imu_index][0]) <= target_ns:
            row = self.imu[self.imu_index]
            msg = Imu()
            msg.header.stamp = self.stamp(int(row[0]))
            msg.header.frame_id = 'imu0'
            msg.angular_velocity.x = float(row[1])
            msg.angular_velocity.y = float(row[2])
            msg.angular_velocity.z = float(row[3])
            msg.linear_acceleration.x = float(row[4])
            msg.linear_acceleration.y = float(row[5])
            msg.linear_acceleration.z = float(row[6])
            self.imu_pub.publish(msg)
            self.imu_index += 1

        while self.image_index < len(self.images) and int(self.images[self.image_index][0]) <= target_ns:
            row = self.images[self.image_index]
            frame = cv2.imread(str(self.dataset / 'mav0' / 'cam0' / 'data' / row[1]),
                               cv2.IMREAD_GRAYSCALE)
            if frame is None:
                raise RuntimeError(f'Cannot read EuRoC image {row[1]}')
            msg = Image()
            msg.header.stamp = self.stamp(int(row[0]))
            msg.header.frame_id = 'cam0'
            msg.height, msg.width = frame.shape
            msg.encoding = 'mono8'
            msg.is_bigendian = False
            msg.step = msg.width
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)
            self.image_index += 1

        if self.image_index == len(self.images) and self.imu_index == len(self.imu):
            self.get_logger().info('EuRoC playback complete')
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--speed', type=float, default=1.0)
    parsed, _ = parser.parse_known_args(args=args)
    rclpy.init(args=args)
    node = EurocPlayer(parsed.dataset, parsed.speed)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
