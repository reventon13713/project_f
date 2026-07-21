#!/usr/bin/env python3
"""ROS contract regression: aligned NED odometry must reach PX4 unchanged."""

import time

import numpy as np

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleOdometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from VIO_pkg.vio_bridge import VioBridge


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def main():
    rclpy.init()
    bridge = VioBridge(parameter_overrides=[
        Parameter('vins_odom_topic', value='/vio/aligned_odometry'),
        Parameter('odom_frame_contract', value='ned_frd'),
        Parameter('readiness_topic', value='/vio/global_alignment_ready'),
        Parameter('publish_visual_odometry', value=True),
    ])
    probe = rclpy.create_node('vio_aligned_bridge_contract_probe')
    aligned_pub = probe.create_publisher(Odometry, '/vio/aligned_odometry', 10)
    ready_pub = probe.create_publisher(Bool, '/vio/global_alignment_ready', 10)
    clock_pub = probe.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    imu_pub = probe.create_publisher(SensorCombined, '/fmu/out/sensor_combined', qos)
    output = []
    probe.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                              output.append, qos)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    spin(executor, 0.15)

    clock = Clock(); clock.clock.sec = 100
    clock_pub.publish(clock)
    spin(executor, 0.05)
    imu = SensorCombined(); imu.timestamp = 50_000_000
    imu_pub.publish(imu)
    spin(executor, 0.05)
    ready = Bool(); ready.data = True
    ready_pub.publish(ready)
    spin(executor, 0.1)
    for _ in range(12):
        odom = Odometry()
        odom.header.stamp = probe.get_clock().now().to_msg()
        odom.pose.pose.position.x = 1.25
        odom.pose.pose.position.y = -2.5
        odom.pose.pose.position.z = 3.75
        odom.pose.pose.orientation.w = 1.0  # body-FRD -> NED identity
        odom.twist.twist.linear.x = 0.4
        odom.twist.twist.linear.y = -0.5
        odom.twist.twist.linear.z = 0.6
        aligned_pub.publish(odom)
        spin(executor, 0.02)

    assert output, 'aligned NED input emitted no PX4 VehicleOdometry'
    message = output[-1]
    assert np.allclose(message.position, [1.25, -2.5, 3.75])
    assert np.allclose(message.velocity, [0.4, -0.5, 0.6])
    assert np.allclose(message.q, [1.0, 0.0, 0.0, 0.0])
    assert message.pose_frame == VehicleOdometry.POSE_FRAME_NED
    assert message.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
    executor.remove_node(probe)
    executor.remove_node(bridge)
    probe.destroy_node()
    bridge.destroy_node()
    rclpy.shutdown()
    print('aligned bridge contract: NED position/velocity/FRD attitude preserved')


if __name__ == '__main__':
    main()
