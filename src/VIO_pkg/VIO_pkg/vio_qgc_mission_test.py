#!/usr/bin/env python3
"""Upload a QGC-compatible global mission and stop Gazebo GPS mid-mission."""

import json
import math
from pathlib import Path
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (SensorGps, VehicleGlobalPosition, VehicleOdometry,
                          VehicleStatus)
from std_msgs.msg import Bool, String


def offset_wgs84(latitude, longitude, north_m, east_m):
    radius = 6378137.0
    latitude_out = latitude + math.degrees(north_m / radius)
    longitude_out = longitude + math.degrees(
        east_m / (radius * math.cos(math.radians(latitude))))
    return latitude_out, longitude_out


class VioQgcMissionTest(Node):
    def __init__(self):
        super().__init__('vio_qgc_mission_test')
        self.declare_parameter('metrics_output', '')
        self.declare_parameter('qgc_plan_output', '')
        self.declare_parameter('mavlink_endpoint', 'udpin:0.0.0.0:14550')
        self.declare_parameter('mission_altitude_m', 2.0)
        # Sequence 0 is QGC's DO_CHANGE_SPEED item; global waypoints are 1..7.
        self.declare_parameter('dropout_sequence', 5)
        self.declare_parameter('mission_speed_mps', 0.55)
        self.declare_parameter('mission_acceptance_radius_m', 0.50)
        self.declare_parameter('mission_timeout_seconds', 120.0)
        self.declare_parameter('gps_stale_seconds', 2.0)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Bool, '/vio/global_alignment_ready',
                                 self._alignment_cb, 10)
        self.create_subscription(Bool, '/vio/handover_ready', self._handover_cb, 10)
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                                 self._ev_cb, qos)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position',
                                 self._gps_cb, qos)
        self.create_subscription(VehicleGlobalPosition,
                                 '/fmu/out/vehicle_global_position',
                                 self._global_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._status_cb, qos)
        self.event_pub = self.create_publisher(String, '/vio/stage7_mission_event', 20)

        self.alignment_ready = False
        self.handover_ready = False
        self.ev_messages = 0
        self.last_ev_s = None
        self.mission_yaw_deg = 90.0
        self.gps_messages = 0
        self.last_gps_s = None
        self.global_position = None
        self.status = None
        self.connection = None
        self.mission_started_s = None
        self.dropout_s = None
        self.dropout_gps_count = 0
        self.current_sequences = []
        self.reached_sequences = []
        self.mission_mode_seen = False
        self.post_dropout_non_mission_before_finish = False
        self.waypoints = []
        self.final_sequence = 0
        self.done = False
        self.started_s = None
        self.last_heartbeat_s = -1.0
        self.timer = self.create_timer(0.05, self._tick)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _alignment_cb(self, message):
        self.alignment_ready = bool(message.data)

    def _handover_cb(self, message):
        self.handover_ready = bool(message.data)

    def _ev_cb(self, message):
        self.ev_messages += 1
        self.last_ev_s = self._now_s()
        q = list(message.q)
        if len(q) == 4 and all(math.isfinite(value) for value in q):
            w, x, y, z = q
            self.mission_yaw_deg = math.degrees(math.atan2(
                2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    def _gps_cb(self, _message):
        self.gps_messages += 1
        self.last_gps_s = self._now_s()

    def _global_cb(self, message):
        if all(math.isfinite(value) for value in
               (message.lat, message.lon, message.alt)):
            self.global_position = message

    def _status_cb(self, message):
        self.status = message
        if message.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
            self.mission_mode_seen = True
        elif (self.dropout_s is not None and self.reached_sequences and
              self.reached_sequences[-1] < self.final_sequence):
            self.post_dropout_non_mission_before_finish = True

    def _publish_event(self, event, **fields):
        payload = {'event': event, 'sim_time_s': self._now_s(), **fields}
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self.event_pub.publish(message)
        self.get_logger().info(message.data)

    def _write_plan(self):
        output = str(self.get_parameter('qgc_plan_output').value)
        if not output:
            return
        altitude = float(self.get_parameter('mission_altitude_m').value)
        speed = float(self.get_parameter('mission_speed_mps').value)
        acceptance = float(self.get_parameter('mission_acceptance_radius_m').value)
        items = [{
            'autoContinue': True, 'command': 178, 'doJumpId': 1, 'frame': 2,
            'params': [1, speed, -1, 0, 0, 0, 0], 'type': 'SimpleItem',
        }]
        for sequence, (latitude, longitude) in enumerate(self.waypoints, start=1):
            items.append({
                'AMSLAltAboveTerrain': None,
                'Altitude': altitude,
                'AltitudeMode': 1,
                'autoContinue': True,
                'command': 16,
                'doJumpId': sequence + 1,
                'frame': 3,
                'params': [0, acceptance, 0, self.mission_yaw_deg,
                           latitude, longitude, altitude],
                'type': 'SimpleItem',
            })
        plan = {
            'fileType': 'Plan',
            'geoFence': {'circles': [], 'polygons': [], 'version': 2},
            'groundStation': 'QGroundControl',
            'mission': {
                'cruiseSpeed': 5,
                'firmwareType': 12,
                'globalPlanAltitudeMode': 1,
                'hoverSpeed': speed,
                'items': items,
                'plannedHomePosition': [self.waypoints[0][0], self.waypoints[0][1],
                                        float(self.global_position.alt) - altitude],
                'vehicleType': 2,
                'version': 2,
            },
            'rallyPoints': {'points': [], 'version': 2},
            'version': 1,
        }
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(plan, indent=2, sort_keys=True) + '\n')

    def _upload_mission(self):
        try:
            from pymavlink import mavutil
        except ImportError as error:
            raise RuntimeError('pymavlink is required for QGC mission upload') from error

        endpoint = str(self.get_parameter('mavlink_endpoint').value)
        connection = mavutil.mavlink_connection(
            endpoint, source_system=255, source_component=190)
        heartbeat = connection.wait_heartbeat(timeout=15)
        if heartbeat is None:
            raise RuntimeError(f'no PX4 MAVLink heartbeat on {endpoint}')
        self.connection = connection
        target_system = connection.target_system
        target_component = connection.target_component

        latitude = float(self.global_position.lat)
        longitude = float(self.global_position.lon)
        # Seven QGC waypoints make the dropout happen early enough that four
        # independently observable global legs remain afterward.
        offsets = [(0, 0), (3.2, 0), (3.2, 3.2), (0, 3.2),
                   (-3.2, 3.2), (-3.2, 0), (0, 0)]
        self.waypoints = [offset_wgs84(latitude, longitude, north, east)
                          for north, east in offsets]
        self.final_sequence = len(self.waypoints)
        self._write_plan()

        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        connection.mav.mission_clear_all_send(target_system, target_component,
                                              mission_type)
        connection.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        connection.mav.mission_count_send(target_system, target_component,
                                          self.final_sequence + 1, mission_type)
        sent = set()
        deadline = self._now_s() + 20.0
        altitude = float(self.get_parameter('mission_altitude_m').value)
        speed = float(self.get_parameter('mission_speed_mps').value)
        acceptance = float(self.get_parameter('mission_acceptance_radius_m').value)
        while len(sent) < self.final_sequence + 1:
            request = connection.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True,
                timeout=2)
            if request is None:
                if self._now_s() > deadline:
                    raise RuntimeError('PX4 mission upload request timed out')
                continue
            sequence = int(request.seq)
            if not 0 <= sequence <= self.final_sequence:
                raise RuntimeError(f'PX4 requested invalid mission sequence {sequence}')
            if sequence == 0:
                connection.mav.mission_item_int_send(
                    target_system, target_component, sequence,
                    mavutil.mavlink.MAV_FRAME_MISSION,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    1, 1, 1.0, speed, -1.0, 0.0, 0, 0, 0.0, mission_type)
            else:
                lat, lon = self.waypoints[sequence - 1]
                connection.mav.mission_item_int_send(
                    target_system, target_component, sequence,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1, 0.0, acceptance, 0.0, self.mission_yaw_deg,
                    int(round(lat * 1e7)), int(round(lon * 1e7)), altitude,
                    mission_type)
            sent.add(sequence)
        acknowledgement = connection.recv_match(type='MISSION_ACK', blocking=True,
                                                timeout=5)
        if acknowledgement is None or int(acknowledgement.type) != 0:
            raise RuntimeError(f'PX4 rejected mission: {acknowledgement}')

        self._publish_event('QGC_MISSION_UPLOADED', waypoint_count=len(self.waypoints),
                            mission_item_count=self.final_sequence + 1,
                            final_sequence=self.final_sequence,
                            mission_speed_mps=speed,
                            acceptance_radius_m=acceptance,
                            fixed_yaw_deg=self.mission_yaw_deg,
                            dropout_sequence=int(
                                self.get_parameter('dropout_sequence').value),
                            waypoints=[{'sequence': index, 'latitude': lat,
                                        'longitude': lon, 'relative_altitude_m': altitude}
                                       for index, (lat, lon) in
                                       enumerate(self.waypoints, start=1)])
        # PX4 custom main/sub modes AUTO=4, MISSION=4. This is the same mode
        # transition QGC sends after uploading and starting a mission.
        connection.mav.command_long_send(
            target_system, target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4, 4, 0, 0, 0, 0)
        acknowledgement = connection.recv_match(type='COMMAND_ACK', blocking=True,
                                                timeout=5)
        if acknowledgement is None or int(acknowledgement.result) not in (0, 5):
            raise RuntimeError(f'PX4 rejected AUTO.MISSION: {acknowledgement}')
        self.mission_started_s = self._now_s()
        self._publish_event('QGC_MISSION_STARTED')

    def _inject_dropout(self):
        command = ['/home/reventon/ros2_ws/install/vio_gazebo_plugins/lib/'
                   'vio_gazebo_plugins/vio_gps_control_command', 'off']
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=10.0, check=False)
        if result.returncode != 0:
            raise RuntimeError(f'Gazebo GPS control failed: {result.stderr.strip()}')
        self.dropout_s = self._now_s()
        self.dropout_gps_count = self.gps_messages
        self._publish_event('GPS_DROPOUT_COMMAND', sensor='all_gps', state='off',
                            at_sequence=self.current_sequences[-1])

    def _handle_mavlink(self):
        if self.connection is None:
            return
        now = self._now_s()
        if now - self.last_heartbeat_s >= 1.0:
            from pymavlink import mavutil
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat_s = now
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            message_type = message.get_type()
            if message_type == 'MISSION_CURRENT':
                sequence = int(message.seq)
                if not self.current_sequences or sequence != self.current_sequences[-1]:
                    self.current_sequences.append(sequence)
                    self._publish_event('MISSION_CURRENT', sequence=sequence)
            elif message_type == 'MISSION_ITEM_REACHED':
                sequence = int(message.seq)
                if sequence not in self.reached_sequences:
                    self.reached_sequences.append(sequence)
                    self._publish_event('MISSION_ITEM_REACHED', sequence=sequence)

    def _write(self, passed, reason):
        now = self._now_s()
        result = {
            'passed': bool(passed), 'reason': reason,
            'global_alignment_ready': self.alignment_ready,
            'handover_ready': self.handover_ready,
            'mission_mode_seen': self.mission_mode_seen,
            'waypoint_count': len(self.waypoints),
            'current_sequences': self.current_sequences,
            'reached_sequences': self.reached_sequences,
            'dropout_sequence': int(self.get_parameter('dropout_sequence').value),
            'gps_messages_after_dropout': max(
                0, self.gps_messages - self.dropout_gps_count),
            'gps_last_sample_age_s': (
                None if self.last_gps_s is None else now - self.last_gps_s),
            'ev_last_sample_age_s': (
                None if self.last_ev_s is None else now - self.last_ev_s),
            'post_dropout_non_mission_before_finish':
                self.post_dropout_non_mission_before_finish,
        }
        output = str(self.get_parameter('metrics_output').value)
        if output:
            path = Path(output)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
        self._publish_event('QGC_MISSION_RESULT', passed=bool(passed), reason=reason)
        self.get_logger().info(f'Stage-7 mission result: {result}')
        self.done = True

    def _tick(self):
        if self.done:
            return
        now = self._now_s()
        if self.started_s is None and now > 0:
            self.started_s = now
        if self.mission_started_s is None:
            ready = (self.alignment_ready and self.handover_ready and
                     self.ev_messages >= 20 and self.gps_messages > 0 and
                     self.global_position is not None and self.status is not None and
                     self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            if ready:
                try:
                    self._upload_mission()
                except Exception as error:  # fail closed with a preserved reason
                    self._write(False, str(error))
            elif self.started_s is not None and now - self.started_s > 180.0:
                self._write(False, 'alignment/EV/armed mission preconditions timed out')
            return

        self._handle_mavlink()
        dropout_sequence = int(self.get_parameter('dropout_sequence').value)
        if (self.dropout_s is None and self.current_sequences and
                self.current_sequences[-1] >= dropout_sequence):
            try:
                self._inject_dropout()
            except Exception as error:
                self._write(False, str(error))
                return

        final_sequence = self.final_sequence
        if self.reached_sequences and self.reached_sequences[-1] >= final_sequence:
            gps_stale = (self.last_gps_s is not None and
                         now - self.last_gps_s >= float(
                             self.get_parameter('gps_stale_seconds').value))
            passed = (self.dropout_s is not None and gps_stale and
                      self.mission_mode_seen and
                      self.alignment_ready and self.handover_ready and
                      self.last_ev_s is not None and now - self.last_ev_s < 0.5 and
                      any(seq > dropout_sequence for seq in self.reached_sequences))
            self._write(passed, 'global mission completed after actual GPS dropout'
                        if passed else 'mission completion gates failed')
        elif now - self.mission_started_s > float(
                self.get_parameter('mission_timeout_seconds').value):
            self._write(False, 'global mission timed out')


def main(args=None):
    rclpy.init(args=args)
    node = VioQgcMissionTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
