#!/usr/bin/env python3
"""Stage-6 gate: stop the Gazebo GPS sensor and prove EV-only flight persists."""

import json
import math
from pathlib import Path
import re
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (EstimatorStatusFlags, SensorGps, VehicleGlobalPosition,
                          VehicleLocalPosition, VehicleOdometry, VehicleStatus)
from std_msgs.msg import Bool, String


class VioGpsDropoutTest(Node):
    def __init__(self):
        super().__init__('vio_gps_dropout_test')
        self.declare_parameter('metrics_output', '')
        # Stage 5 already proves a sustained GPS+EV overlap window. Stage 6
        # injects on the first authoritative native overlap sample so a brief
        # scheduler gap cannot close the bridge before the intended fault.
        self.declare_parameter('prefusion_hold_samples', 1)
        self.declare_parameter('gps_stale_seconds', 2.0)
        self.declare_parameter('dropout_hold_seconds', 8.0)
        self.declare_parameter('min_post_dropout_ev_fused_polls', 3)
        self.declare_parameter('alignment_timeout_seconds', 180.0)
        self.declare_parameter('dropout_timeout_seconds', 35.0)
        self.declare_parameter('max_hover_drift_m', 1.0)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, '/vio/global_alignment_ready', self._alignment_cb, 10)
        self.create_subscription(Bool, '/vio/handover_ready', self._handover_cb, 10)
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                                 self._ev_cb, px4_qos)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position',
                                 self._gps_cb, px4_qos)
        self.create_subscription(EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
                                 self._flags_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._local_cb, px4_qos)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
                                 self._global_cb, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._status_cb, px4_qos)
        self.event_pub = self.create_publisher(String, '/vio/gps_dropout_event', 10)

        self.started_s = None
        self.alignment_ready = False
        self.handover_ready = False
        self.native_flags = {}
        self.native_ev_aid = {'pos': {}, 'vel': {}}
        self.last_counted_ev_fuse = {'pos': 0, 'vel': 0}
        self.ros_flags_received = False
        self.last_poll_s = -1.0
        self.prefusion_samples = 0
        self.gps_messages = 0
        self.gps_at_dropout = 0
        self.last_gps_s = None
        self.ev_messages = 0
        self.last_ev_s = None
        self.local_position = None
        self.global_messages = 0
        self.last_global_s = None
        self.status = None
        self.dropout_s = None
        self.ev_only_started_s = None
        self.gps_stale_started_s = None
        self.post_dropout_flag_polls = 0
        self.post_dropout_ev_fused_polls = 0
        self.dropout_origin = None
        self.max_hover_drift_m = 0.0
        self.post_dropout_disarmed = False
        self.post_dropout_failsafe = False
        self.failsafe_at_dropout = None
        self.nav_state_at_dropout = None
        self.post_dropout_nav_state_changed = False
        self.done = False
        self.timer = self.create_timer(0.05, self._tick)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _alignment_cb(self, message):
        self.alignment_ready = bool(message.data)

    def _handover_cb(self, message):
        self.handover_ready = bool(message.data)

    def _ev_cb(self, _message):
        self.ev_messages += 1
        self.last_ev_s = self._now_s()

    def _gps_cb(self, _message):
        self.gps_messages += 1
        self.last_gps_s = self._now_s()

    def _flags_cb(self, _message):
        self.ros_flags_received = True

    def _local_cb(self, message):
        self.local_position = message
        if self.dropout_origin is not None and all(math.isfinite(value) for value in
                                                   (message.x, message.y, message.z)):
            distance = math.sqrt(sum((value - origin) ** 2 for value, origin in
                                     zip((message.x, message.y, message.z),
                                         self.dropout_origin)))
            self.max_hover_drift_m = max(self.max_hover_drift_m, distance)

    def _global_cb(self, _message):
        self.global_messages += 1
        self.last_global_s = self._now_s()

    def _status_cb(self, message):
        self.status = message
        if self.dropout_s is not None:
            self.post_dropout_disarmed |= (
                message.arming_state != VehicleStatus.ARMING_STATE_ARMED)
            # SITL without QGC/RC can already report the generic failsafe bit
            # while safely remaining in Offboard. Judge dropout by a new
            # transition and by nav-state continuity, not that baseline bit.
            self.post_dropout_failsafe |= (
                not bool(self.failsafe_at_dropout) and bool(message.failsafe))
            self.post_dropout_nav_state_changed |= (
                self.nav_state_at_dropout is not None and
                message.nav_state != self.nav_state_at_dropout)

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
            if self.dropout_s is not None:
                self.post_dropout_flag_polls += 1
                if all(flags.get(name, False) for name in ('cs_ev_pos', 'cs_ev_vel')):
                    self.post_dropout_ev_fused_polls += 1
        for kind, topic in (('pos', 'estimator_aid_src_ev_pos'),
                            ('vel', 'estimator_aid_src_ev_vel')):
            try:
                aid_output = subprocess.run(
                    ['/home/reventon/PX4-Autopilot/build/px4_sitl_default/bin/'
                     'px4-listener', topic, '-n', '1'], capture_output=True,
                    text=True, timeout=2.0, check=False).stdout
            except (OSError, subprocess.TimeoutExpired):
                continue
            timestamp = re.search(r'^\s*timestamp:\s+(\d+)', aid_output,
                                  flags=re.MULTILINE)
            last_fuse = re.search(r'^\s*time_last_fuse:\s+(\d+)', aid_output,
                                  flags=re.MULTILINE)
            if timestamp and last_fuse:
                aid = {'timestamp': int(timestamp.group(1)),
                       'time_last_fuse': int(last_fuse.group(1))}
                self.native_ev_aid[kind] = aid
                if (self.dropout_s is not None and aid['time_last_fuse'] >
                        self.last_counted_ev_fuse[kind]):
                    self.last_counted_ev_fuse[kind] = aid['time_last_fuse']
                    self.post_dropout_ev_fused_polls += 1

    def _ev_aid_recent(self):
        return all(aid.get('time_last_fuse', 0) > 0 and
                   aid.get('timestamp', 0) - aid['time_last_fuse'] < 2_000_000
                   for aid in self.native_ev_aid.values())

    def _preconditions_hold(self):
        return (self.alignment_ready and self.handover_ready and self.gps_messages > 0 and
                self.ev_messages >= 20 and self.native_flags.get('cs_gps', False) and
                self.status is not None and
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                self.local_position is not None and self.local_position.xy_valid and
                self.local_position.z_valid)

    def _inject_dropout(self):
        command = [
            '/home/reventon/ros2_ws/install/vio_gazebo_plugins/lib/'
            'vio_gazebo_plugins/vio_gps_control_command', 'off']
        try:
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=10.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            self._write(False, f'failed to invoke Gazebo GPS control: {error}')
            return
        if result.returncode != 0:
            self._write(False, f'Gazebo GPS control exited {result.returncode}: '
                               f'{result.stderr.strip()}')
            return
        self.dropout_s = self._now_s()
        self.gps_at_dropout = self.gps_messages
        self.dropout_origin = (self.local_position.x, self.local_position.y,
                               self.local_position.z)
        self.failsafe_at_dropout = bool(self.status.failsafe)
        self.nav_state_at_dropout = int(self.status.nav_state)
        for kind, aid in self.native_ev_aid.items():
            self.last_counted_ev_fuse[kind] = aid.get('time_last_fuse', 0)
        event = String()
        event.data = 'GPS_DROPOUT_COMMAND sensor=all_gps state=off'
        self.event_pub.publish(event)
        self.get_logger().warn(event.data)

    def _write(self, passed, reason):
        now = self._now_s()
        result = {
            'passed': bool(passed),
            'reason': reason,
            'global_alignment_ready': self.alignment_ready,
            'handover_ready': self.handover_ready,
            'gps_messages_total': self.gps_messages,
            'gps_messages_after_dropout': max(0, self.gps_messages - self.gps_at_dropout),
            'gps_last_sample_age_s': None if self.last_gps_s is None else now - self.last_gps_s,
            'ev_input_messages': self.ev_messages,
            'ev_last_sample_age_s': None if self.last_ev_s is None else now - self.last_ev_s,
            'global_position_messages': self.global_messages,
            'global_position_last_sample_age_s': (
                None if self.last_global_s is None else now - self.last_global_s),
            'dropout_observed_seconds': (
                0.0 if self.gps_stale_started_s is None else
                now - self.gps_stale_started_s),
            'post_dropout_flag_polls': self.post_dropout_flag_polls,
            'post_dropout_ev_fused_polls': self.post_dropout_ev_fused_polls,
            'max_hover_drift_m': self.max_hover_drift_m,
            'post_dropout_disarmed': self.post_dropout_disarmed,
            'post_dropout_failsafe': self.post_dropout_failsafe,
            'failsafe_at_dropout': self.failsafe_at_dropout,
            'nav_state_at_dropout': self.nav_state_at_dropout,
            'post_dropout_nav_state_changed': self.post_dropout_nav_state_changed,
            'px4_gps_fusion_active_at_end': bool(self.native_flags.get('cs_gps')),
            'px4_ev_position_fusion_active_at_end': bool(self.native_flags.get('cs_ev_pos')),
            'px4_ev_velocity_fusion_active_at_end': bool(self.native_flags.get('cs_ev_vel')),
            'native_estimator_flags': self.native_flags,
            'native_ev_aid': self.native_ev_aid,
            'ros_estimator_flags_received': self.ros_flags_received,
        }
        path_text = str(self.get_parameter('metrics_output').value)
        if path_text:
            path = Path(path_text)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        event = String()
        event.data = f'GPS_DROPOUT_RESULT passed={str(bool(passed)).lower()} reason={reason}'
        self.event_pub.publish(event)
        self.get_logger().info(f'GPS dropout test: {result}')
        self.done = True

    def _tick(self):
        if self.done:
            return
        now = self._now_s()
        if self.started_s is None and now > 0:
            self.started_s = now
        self._poll_native_flags()

        if self.dropout_s is None:
            self.prefusion_samples = self.prefusion_samples + 1 if self._preconditions_hold() else 0
            if self.prefusion_samples >= int(
                    self.get_parameter('prefusion_hold_samples').value):
                self._inject_dropout()
                return
            if (self.started_s is not None and now - self.started_s > float(
                    self.get_parameter('alignment_timeout_seconds').value)):
                self._write(False, 'GPS+EV pre-dropout gates did not hold before timeout')
            return

        gps_stale = (self.last_gps_s is not None and now - self.last_gps_s >= float(
            self.get_parameter('gps_stale_seconds').value))
        local_valid = (self.local_position is not None and self.local_position.xy_valid and
                       self.local_position.z_valid)
        global_fresh = self.last_global_s is not None and now - self.last_global_s < 1.0
        ev_fresh = self.last_ev_s is not None and now - self.last_ev_s < 0.5
        armed = (self.status is not None and
                 self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        ev_only_fusion = (
            not self.native_flags.get('cs_gps', True) and
            self.native_flags.get('cs_ev_pos', False) and
            self.native_flags.get('cs_ev_vel', False) and
            not self.native_flags.get('cs_inertial_dead_reckoning', True) and
            self._ev_aid_recent())

        if gps_stale:
            if self.gps_stale_started_s is None:
                self.gps_stale_started_s = now
        else:
            self.gps_stale_started_s = None

        if (self.post_dropout_disarmed or self.post_dropout_failsafe or
                self.post_dropout_nav_state_changed):
            self._write(False, 'vehicle disarmed or changed failsafe/nav state after GPS dropout')
        elif self.max_hover_drift_m > float(self.get_parameter('max_hover_drift_m').value):
            self._write(False, 'vehicle drift exceeded the stage-6 hover bound')
        elif (self.gps_stale_started_s is not None and
              now - self.gps_stale_started_s >= float(
                  self.get_parameter('dropout_hold_seconds').value) and
              self.post_dropout_ev_fused_polls >= int(
                  self.get_parameter('min_post_dropout_ev_fused_polls').value) and
              ev_only_fusion and local_valid and global_fresh and ev_fresh and armed):
            self._write(True, 'Gazebo GPS stopped; PX4 sustained armed EV-only position flight')
        elif now - self.dropout_s > float(
                self.get_parameter('dropout_timeout_seconds').value):
            self._write(False, 'GPS dropout/EV-only flight gates did not hold before timeout')


def main(args=None):
    rclpy.init(args=args)
    node = VioGpsDropoutTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
