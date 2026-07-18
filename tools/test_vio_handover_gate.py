#!/usr/bin/env python3
"""ROS integration smoke test for the PX4 handover gate and PX4 output."""

import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleOdometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from VIO_pkg.vio_bridge import VioBridge


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def main():
    rclpy.init()
    bridge = VioBridge(parameter_overrides=[
        Parameter('publish_visual_odometry', value=True),
        Parameter('allow_rehandover_after_fault', value=True),
    ])
    probe = rclpy.create_node('vio_handover_gate_probe')
    ready_pub = probe.create_publisher(Bool, '/vio/ready', 10)
    odom_pub = probe.create_publisher(Odometry, '/odometry', 10)
    clock_pub = probe.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    px4_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    imu_pub = probe.create_publisher(
        SensorCombined, '/fmu/out/sensor_combined', px4_qos)
    handover_states = []
    visual_messages = []
    probe.create_subscription(
        Bool, '/vio/handover_ready',
        lambda msg: handover_states.append(bool(msg.data)), 10)
    probe.create_subscription(
        VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
        lambda msg: visual_messages.append(msg), px4_qos)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    spin_for(executor, 0.2)

    clock = Clock()
    clock.clock.sec = 100
    clock_pub.publish(clock)
    spin_for(executor, 0.05)
    imu = SensorCombined()
    imu.timestamp = 50_000_000
    imu_pub.publish(imu)
    spin_for(executor, 0.1)

    ready = Bool(); ready.data = True
    ready_pub.publish(ready)
    spin_for(executor, 0.1)
    for index in range(12):
        odom = Odometry()
        odom.header.stamp = probe.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = index * 0.01
        odom_pub.publish(odom)
        spin_for(executor, 0.02)
    assert handover_states and handover_states[-1] is True
    assert visual_messages, 'open handover gate emitted no PX4 odometry'
    assert visual_messages[-1].timestamp_sample > 0
    first_reset_counter = visual_messages[-1].reset_counter
    assert first_reset_counter == 1
    messages_before_jump = len(visual_messages)

    jump = Odometry()
    jump.header.stamp = probe.get_clock().now().to_msg()
    jump.pose.pose.orientation.w = 1.0
    jump.pose.pose.position.x = 1.0
    odom_pub.publish(jump)
    spin_for(executor, 0.1)
    assert handover_states[-1] is False
    assert len(visual_messages) == messages_before_jump

    ready_pub.publish(ready)
    spin_for(executor, 0.05)
    for index in range(12):
        odom = Odometry()
        odom.header.stamp = probe.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = 2.0 + index * 0.01
        odom_pub.publish(odom)
        spin_for(executor, 0.02)
    assert handover_states[-1] is True
    assert len(visual_messages) > messages_before_jump
    assert visual_messages[-1].reset_counter == (first_reset_counter + 1) & 0xff
    messages_before_stale = len(visual_messages)
    spin_for(executor, 0.35)
    assert handover_states[-1] is False
    assert len(visual_messages) == messages_before_stale

    executor.remove_node(probe)
    executor.remove_node(bridge)
    probe.destroy_node()
    bridge.destroy_node()

    # Production default is fail-closed: once an already-open handover sees a
    # fault, a later readiness message cannot reconnect a potentially shifted
    # VINS frame without restarting the bridge.
    latched_bridge = VioBridge(parameter_overrides=[
        Parameter('publish_visual_odometry', value=True),
    ])
    latched_bridge._handover_is_open = True
    latched_bridge._invalidate_handover('test in-flight reset')
    generation_before = latched_bridge._ready_generation
    latched_bridge._ready_callback(ready)
    assert latched_bridge._fault_latched
    assert latched_bridge._ready_generation == generation_before
    latched_bridge.destroy_node()
    rclpy.shutdown()
    print('handover gate smoke: opt-in reopen works; production default latches closed')


if __name__ == '__main__':
    main()
