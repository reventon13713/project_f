#!/usr/bin/env python3
"""Fail-closed evidence gate for stage-4 GPS + external-vision fusion."""

import json
from pathlib import Path
import re
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import EstimatorStatusFlags, VehicleOdometry
from std_msgs.msg import Bool


class VioGpsEvFusionTest(Node):
    def __init__(self):
        super().__init__('vio_gps_ev_fusion_test')
        self.declare_parameter('metrics_output', '')
        self.declare_parameter('fusion_hold_samples', 20)
        # VIO readiness is deliberately emitted only after the independent
        # preflight validation/hold.  Do not time out before an alignment can
        # legally exist; the short fusion gate begins after alignment.
        self.declare_parameter('alignment_timeout_seconds', 180.0)
        self.declare_parameter('fusion_timeout_seconds', 30.0)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, '/vio/global_alignment_ready',
                                 self._alignment_cb, 10)
        self.create_subscription(Bool, '/vio/handover_ready',
                                 self._handover_cb, 10)
        self.create_subscription(VehicleOdometry,
                                 '/fmu/in/vehicle_visual_odometry',
                                 self._ev_cb, qos)
        self.create_subscription(EstimatorStatusFlags,
                                 '/fmu/out/estimator_status_flags',
                                 self._flags_cb, qos)
        self.alignment_ready = False
        self.handover_ready = False
        self.ev_messages = 0
        self.ros_flags = None
        self.native_flags = {}
        self.consecutive_fused = 0
        self.started_s = None
        self.alignment_started_s = None
        self.last_poll_s = -1.0
        self.done = False
        self.timer = self.create_timer(0.05, self._tick)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _alignment_cb(self, message):
        self.alignment_ready = bool(message.data)
        if self.alignment_ready and self.alignment_started_s is None:
            self.alignment_started_s = self._now_s()

    def _handover_cb(self, message):
        self.handover_ready = bool(message.data)

    def _ev_cb(self, _message):
        self.ev_messages += 1

    def _flags_cb(self, message):
        self.ros_flags = message

    def _poll_native_flags(self):
        now = self._now_s()
        if now - self.last_poll_s < 0.5:
            return
        self.last_poll_s = now
        command = ['/home/reventon/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener',
                   'estimator_status_flags', '-n', '1']
        try:
            output = subprocess.run(command, capture_output=True, text=True,
                                    timeout=2.0, check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            return
        flags = {}
        for name, value in re.findall(
                r'^\s*(cs_[a-z0-9_]+):\s+(True|False|true|false|0|1)\s*$',
                output, flags=re.MULTILINE):
            flags[name] = value.lower() in ('true', '1')
        if flags:
            self.native_flags = flags

    def _fusion_active(self):
        # Native uORB is authoritative; ROS mirrors it for rosbag evidence.
        return all(self.native_flags.get(name, False)
                   for name in ('cs_gps', 'cs_ev_pos', 'cs_ev_vel'))

    def _write(self, passed, reason):
        result = {
            'passed': bool(passed), 'reason': reason,
            'global_alignment_ready': self.alignment_ready,
            'handover_ready': self.handover_ready,
            'ev_input_messages': self.ev_messages,
            'px4_gps_fusion_active': bool(self.native_flags.get('cs_gps')),
            'px4_ev_position_fusion_active': bool(self.native_flags.get('cs_ev_pos')),
            'px4_ev_velocity_fusion_active': bool(self.native_flags.get('cs_ev_vel')),
            'native_estimator_flags': self.native_flags,
            'ros_estimator_flags_received': self.ros_flags is not None,
        }
        path_text = str(self.get_parameter('metrics_output').value)
        if path_text:
            path = Path(path_text)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        self.get_logger().info(f'GPS+EV fusion test: {result}')
        self.done = True

    def _tick(self):
        if self.done:
            return
        now = self._now_s()
        if self.started_s is None and now > 0:
            self.started_s = now
        self._poll_native_flags()
        if (self.alignment_ready and self.handover_ready and
                self.ev_messages > 0 and self._fusion_active()):
            self.consecutive_fused += 1
        else:
            self.consecutive_fused = 0
        if self.consecutive_fused >= int(self.get_parameter('fusion_hold_samples').value):
            self._write(True, 'PX4 concurrently fused GPS, EV position, and EV velocity')
            return
        if (self.alignment_started_s is None and self.started_s is not None and
                now - self.started_s > float(
                    self.get_parameter('alignment_timeout_seconds').value)):
            self._write(False, 'global VIO alignment did not become ready before timeout')
        elif (self.alignment_started_s is not None and
              now - self.alignment_started_s > float(
                  self.get_parameter('fusion_timeout_seconds').value)):
            self._write(False, 'GPS+EV fusion gates did not hold after alignment')


def main(args=None):
    rclpy.init(args=args)
    node = VioGpsEvFusionTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
