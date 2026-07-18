#!/usr/bin/env python3
"""Verify PX4 EV fusion and GPS fallback after a VINS reset."""

import json
from pathlib import Path
import re
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from px4_msgs.msg import (EstimatorStatusFlags, VehicleLocalPosition,
                          VehicleOdometry, VehicleStatus)


class VioPx4FaultTest(Node):
    def __init__(self):
        super().__init__('vio_px4_fault_test')
        self.declare_parameter('metrics_output', '')
        self.declare_parameter('fusion_hold_samples', 10)
        self.declare_parameter('fault_timeout_seconds', 8.0)
        self.declare_parameter('disable_gps_before_fault', False)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, '/vio/handover_ready',
                                 self._handover_cb, 10)
        self.create_subscription(VehicleOdometry,
                                 '/fmu/in/vehicle_visual_odometry',
                                 self._ev_input_cb, qos)
        self.create_subscription(EstimatorStatusFlags,
                                 '/fmu/out/estimator_status_flags',
                                 self._flags_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._status_cb, qos)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self._local_position_cb, qos)
        self.reset_client = self.create_client(Trigger, '/reset')
        self.timer = self.create_timer(0.05, self._tick)
        self.handover_ready = False
        self.flags = None
        self.status = None
        self.local_position = None
        self.fusion_samples = 0
        self.ev_input_count = 0
        self.ev_count_at_fault = None
        self.fault_time = None
        self.reset_future = None
        self.post_fault_positions = []
        self.gate_closed_after_fault = False
        self.ev_fusion_stopped = False
        self.completed = False
        self.cli_flags = {}
        self.last_cli_poll_s = 0.0
        self.gps_disable_requested = False

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _handover_cb(self, msg):
        self.handover_ready = bool(msg.data)
        if self.fault_time is not None and not self.handover_ready:
            self.gate_closed_after_fault = True

    def _ev_input_cb(self, _msg):
        self.ev_input_count += 1

    def _flags_cb(self, msg):
        self.flags = msg
        if self.fault_time is not None and not msg.cs_ev_pos and not msg.cs_ev_vel:
            self.ev_fusion_stopped = True

    def _status_cb(self, msg):
        self.status = msg

    def _local_position_cb(self, msg):
        self.local_position = msg
        if self.fault_time is not None:
            pos = np.array([msg.x, msg.y, msg.z], dtype=float)
            if np.isfinite(pos).all():
                self.post_fault_positions.append(pos)

    def _write_result(self, passed, reason):
        max_step = 0.0
        if len(self.post_fault_positions) > 1:
            positions = np.asarray(self.post_fault_positions)
            max_step = float(np.max(np.linalg.norm(np.diff(positions, axis=0),
                                                   axis=1)))
        result = {
            'passed': bool(passed),
            'reason': reason,
            'handover_gate_closed_after_fault': self.gate_closed_after_fault,
            'ev_fusion_stopped_after_fault': self.ev_fusion_stopped,
            'ev_input_count': self.ev_input_count,
            'ev_input_count_at_fault': self.ev_count_at_fault,
            'ev_messages_after_fault': (None if self.ev_count_at_fault is None
                                        else self.ev_input_count - self.ev_count_at_fault),
            'px4_gps_fusion_active': bool(self.cli_flags.get('cs_gps')),
            'vehicle_nav_state': (None if self.status is None
                                  else int(self.status.nav_state)),
            'vehicle_failsafe': (None if self.status is None
                                 else bool(self.status.failsafe)),
            'local_position_xy_valid': (None if self.local_position is None
                                        else bool(self.local_position.xy_valid)),
            'post_fault_local_position_max_step_m': max_step,
            'gps_disabled_before_fault': bool(
                self.get_parameter('disable_gps_before_fault').value),
        }
        output = str(self.get_parameter('metrics_output').value)
        if output:
            path = Path(output)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        self.get_logger().info(f'PX4 VIO fault test: {result}')
        self.completed = True

    @staticmethod
    def _set_px4_param(name, value):
        command = [
            '/home/reventon/PX4-Autopilot/build/px4_sitl_default/bin/px4-param',
            'set', name, str(value),
        ]
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=3.0, check=False)
        return result.returncode == 0

    def _poll_native_estimator_flags(self):
        now = self._now_s()
        if now - self.last_cli_poll_s < 0.5:
            return
        self.last_cli_poll_s = now
        command = [
            '/home/reventon/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener',
            'estimator_status_flags', '-n', '1',
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=2.0, check=False)
        except (OSError, subprocess.TimeoutExpired):
            return
        flags = {}
        for name, value in re.findall(r'^\s*(cs_[a-z0-9_]+):\s+(True|False|true|false|0|1)\s*$',
                                      result.stdout, flags=re.MULTILINE):
            flags[name] = value.lower() in ('true', '1')
        if flags:
            self.cli_flags = flags

    def _tick(self):
        if self.completed:
            return
        self._poll_native_estimator_flags()
        if self.fault_time is None:
            fusion_active = bool(self.cli_flags.get('cs_ev_pos') and
                                 self.cli_flags.get('cs_ev_vel'))
            disable_gps = bool(
                self.get_parameter('disable_gps_before_fault').value)
            if (disable_gps and self.handover_ready and fusion_active and
                    not self.gps_disable_requested):
                if (self._set_px4_param('COM_POSCTL_NAVL', 1) and
                        self._set_px4_param('EKF2_HGT_REF', 0) and
                        self._set_px4_param('EKF2_GPS_CTRL', 0)):
                    self.gps_disable_requested = True
                    self.fusion_samples = 0
                    self.get_logger().warning(
                        'Disabled PX4 GPS fusion; EV is now the required horizontal aid.')
                return
            gps_state_ready = (not self.cli_flags.get('cs_gps', True)
                               if disable_gps else
                               self.cli_flags.get('cs_gps', False))
            if self.handover_ready and fusion_active and gps_state_ready:
                self.fusion_samples += 1
            else:
                self.fusion_samples = 0
            if self.fusion_samples < int(
                    self.get_parameter('fusion_hold_samples').value):
                return
            if not self.reset_client.service_is_ready():
                return
            self.ev_count_at_fault = self.ev_input_count
            self.fault_time = self._now_s()
            self.reset_future = self.reset_client.call_async(Trigger.Request())
            self.get_logger().warning(
                'Injected VINS reset after confirmed PX4 EV fusion.')
            return

        elapsed = self._now_s() - self.fault_time
        if elapsed < 1.0:
            return
        ev_quiet = (self.ev_count_at_fault is not None and
                    self.ev_input_count - self.ev_count_at_fault <= 2)
        self.ev_fusion_stopped = bool(
            not self.cli_flags.get('cs_ev_pos', True) and
            not self.cli_flags.get('cs_ev_vel', True))
        disable_gps = bool(
            self.get_parameter('disable_gps_before_fault').value)
        gps_fallback = bool(self.cli_flags.get('cs_gps'))
        local_valid = bool(self.local_position and self.local_position.xy_valid)
        offboard_held = bool(
            self.status and
            self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            not self.status.failsafe)
        if self.gate_closed_after_fault and self.ev_fusion_stopped and ev_quiet:
            if disable_gps:
                safe_nav_state = bool(
                    self.status and self.status.failsafe and
                    self.status.nav_state in (
                        VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                        VehicleStatus.NAVIGATION_STATE_DESCEND))
                if safe_nav_state:
                    self._write_result(
                        True, 'PX4 entered Land/Descend after EV and GPS loss')
                    return
            elif gps_fallback and local_valid and offboard_held:
                self._write_result(
                    True, 'PX4 stopped EV fusion and held GPS offboard')
                return
        if elapsed > float(self.get_parameter('fault_timeout_seconds').value):
            self._write_result(False, 'fault response did not satisfy all gates')


def main(args=None):
    rclpy.init(args=args)
    node = VioPx4FaultTest()
    try:
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if bool(node.get_parameter('disable_gps_before_fault').value):
            node._set_px4_param('EKF2_GPS_CTRL', 7)
            node._set_px4_param('EKF2_HGT_REF', 1)
            node._set_px4_param('COM_POSCTL_NAVL', 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
