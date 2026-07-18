#!/usr/bin/env python3
"""Safe Gazebo VIO initialization flight: warm up, slow takeoff, excite, validate."""

from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleOdometry, VehicleStatus)


class VioPreflightTest(Node):
    def __init__(self):
        super().__init__('vio_preflight_test')
        self.declare_parameter('warmup_seconds', 5.0)
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('takeoff_speed', 0.30)
        # At the 10 Hz rendered-camera rate a quick corner-to-corner command
        # loses too many tracks in a single VINS initialization window.  Use a
        # deliberately gentle, continuously observed translation instead.
        self.declare_parameter('excitation_speed', 0.55)
        self.declare_parameter('cross_xy_extent', 1.20)
        self.declare_parameter('cross_z_extent', 1.00)
        self.declare_parameter('ready_hold_seconds', 60.0)
        # A guarded estimator retry near the end of excitation can leave less
        # than the 8 s continuous-validation hold inside the old 30 s window.
        # Waiting is safe because PX4 remains on GPS hover and no handover has
        # occurred; allow a healthy late initialization to prove itself.
        self.declare_parameter('validation_timeout_seconds', 45.0)
        self.declare_parameter('excitation_repeats', 1)
        self.declare_parameter('max_vins_recoveries', 2)
        # Optional per-run evidence file.  Keep this outside the package so a
        # clean container restart never overwrites a previous run.
        self.declare_parameter('metrics_output', '')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.ready_pub = self.create_publisher(Bool, '/vio/ready', 1)
        # A rosbag records topics but not service calls.  Record this event so
        # an offline VINS replay can issue its /reset at the same simulation
        # instant as the original controlled flight.
        self.event_pub = self.create_publisher(String, '/vio/preflight_event', 10)
        self.reset_client = self.create_client(Trigger, '/reset')
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self._px4_odom_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._status_cb, qos)
        self.create_subscription(Odometry, '/odometry', self._vio_cb, 20)
        self.create_subscription(Range, '/vio/range', self._range_cb, 10)

        self.state = 'WARMUP'
        self.state_start = None
        self.status = VehicleStatus()
        self.px4_pos = None
        self.command = np.zeros(3)
        # Capture the simulator's actual initial heading before sending an
        # offboard yaw setpoint.  The iris model starts near +90 deg in this
        # world; commanding an absolute zero here previously mixed a 45 deg
        # yaw transient into the first 0.5 s VINS initialization window.
        self.yaw = math.nan
        self.initial_yaw = None
        self.arm_sent = False
        self.last_range_time = None
        self.vio_samples = deque(maxlen=100)
        self.last_vio_pos = None
        self.last_vio_receive_time = None
        self.vins_recovery_count = 0
        # PX4 and VINS do not become valid at the same instant: VINS publishes
        # only after its visual/IMU initialization window.  Accumulating two
        # independent paths from the reset (the old check) therefore compared
        # different time intervals and reported a fictitious scale error.
        # Keep a short PX4 history keyed by simulation time, then compare only
        # paired VIO/PX4 increments after the VINS initialization jump.
        self.px4_history = deque(maxlen=1200)
        self.scale_vio_path = 0.0
        self.scale_px4_path = 0.0
        self.last_scale_vio_pos = None
        self.last_scale_px4_pos = None
        self.scale_pair_count = 0
        # Paired positions collected only after VINS has an initialized pose.
        # They estimate an XY yaw/translation alignment for evaluation.  They
        # deliberately never estimate scale: scale error is evidence, not a
        # nuisance parameter to be hidden by a Sim(3) fit.
        self.alignment_pairs = []
        self.alignment_yaw = None
        self.alignment_translation = None
        self.last_vector_error = None
        self.max_vector_error = 0.0
        self.vector_error_squared_sum = 0.0
        self.vector_error_count = 0
        self.max_vio_speed = 0.0
        self.max_vio_step = 0.0
        self.excitation_index = 0
        self.reset_future = None
        self.valid_since = None
        self.ready_reference_vio = None
        self.ready_reference_px4 = None
        altitude = float(self.get_parameter('takeoff_altitude').value)
        # A repeated, axis-aligned cross is intentionally easy to inspect in
        # RViz: each return-to-centre leg retraces the same line.  Keep yaw
        # fixed so feature motion comes from translation, not a turn-in-place.
        xy_extent = float(self.get_parameter('cross_xy_extent').value)
        z_extent = float(self.get_parameter('cross_z_extent').value)
        centre = (0.0, 0.0, -altitude, None)
        self.excitation = [
            # X axis: +X -> centre -> -X -> centre.
            (xy_extent, 0.0, -altitude, None), centre,
            (-xy_extent, 0.0, -altitude, None), centre,
            # Y axis: +Y -> centre -> -Y -> centre.
            (0.0, xy_extent, -altitude, None), centre,
            (0.0, -xy_extent, -altitude, None), centre,
            # Z axis: down -> centre -> up -> centre.  NED altitude is -Z.
            (0.0, 0.0, -altitude - z_extent, None), centre,
            (0.0, 0.0, -altitude + z_extent, None), centre,
        ] * max(1, int(self.get_parameter('excitation_repeats').value))
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            'VIO is running before arming; collecting stationary IMU/camera/range data.')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self):
        if self.state_start is None:
            return 0.0
        return (self.get_clock().now() - self.state_start).nanoseconds * 1e-9

    def _transition(self, state):
        self.get_logger().info(f'[VIO preflight] {self.state} -> {state}')
        self.state = state
        self.state_start = self.get_clock().now()

    def _status_cb(self, msg):
        self.status = msg

    def _px4_odom_cb(self, msg):
        pos = np.asarray(msg.position, dtype=float)
        if not np.isfinite(pos).all():
            return
        self.px4_pos = pos
        if self.initial_yaw is None:
            q = np.asarray(msg.q, dtype=float)
            if np.isfinite(q).all() and np.linalg.norm(q) > 0.9:
                w, x, y, z = q
                self.initial_yaw = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z))
                self.yaw = self.initial_yaw
                self.get_logger().info(
                    f'Captured initial PX4 yaw: {math.degrees(self.yaw):.2f} deg.')
        self.px4_history.append((self._now_s(), pos.copy()))

    def _px4_at(self, timestamp_s):
        """Return the nearest PX4 ground-truth sample in the VINS timebase."""
        if not self.px4_history:
            return None
        time_s, pos = min(self.px4_history,
                          key=lambda item: abs(item[0] - timestamp_s))
        # VehicleOdometry callbacks are frequent; a farther sample means the
        # VINS image is outside our retained/common time interval.
        return pos.copy() if abs(time_s - timestamp_s) <= 0.15 else None

    def _append_scale_pair(self, vio_pos, vio_stamp_s):
        # Scale is observable from the commanded motion only.  Once the
        # trajectory enters VALIDATE the vehicle is stationary: continuing to
        # sum VINS jitter while PX4 path length is ~zero makes the cumulative
        # ratio grow forever even for a metrically correct estimator.
        if self.state != 'EXCITATION':
            return
        px4_pos = self._px4_at(vio_stamp_s)
        if px4_pos is None:
            return
        if self.last_scale_vio_pos is None:
            self.last_scale_vio_pos = vio_pos.copy()
            self.last_scale_px4_pos = px4_pos
            return
        vio_step = float(np.linalg.norm(vio_pos - self.last_scale_vio_pos))
        px4_step = float(np.linalg.norm(px4_pos - self.last_scale_px4_pos))
        # The first optimized VINS pose has an arbitrary local-origin jump.
        # It is not vehicle motion and must establish, rather than pollute,
        # the common trajectory reference.
        if vio_step > 0.5:
            self.last_scale_vio_pos = vio_pos.copy()
            self.last_scale_px4_pos = px4_pos
            # A VINS reset changes its local origin.  Do not fit one yaw and
            # translation across two unrelated VIO epochs; start the vector
            # evaluation history again while retaining the scale path already
            # accumulated from valid within-epoch increments.
            self.alignment_pairs.clear()
            self.alignment_yaw = None
            self.alignment_translation = None
            self.last_vector_error = None
            self.max_vector_error = 0.0
            self.vector_error_squared_sum = 0.0
            self.vector_error_count = 0
            self.get_logger().info('VINS initialization jump excluded from scale check.')
            return
        if vio_step > 1e-4 and px4_step < 0.5:
            self.scale_vio_path += vio_step
            self.scale_px4_path += px4_step
            self.scale_pair_count += 1
            self._append_alignment_pair(vio_pos, px4_pos)
        self.last_scale_vio_pos = vio_pos.copy()
        self.last_scale_px4_pos = px4_pos

    def _append_alignment_pair(self, vio_pos, px4_pos):
        """Update yaw-only, no-scale VINS-to-Gazebo evaluation alignment."""
        # VINS publishes its gravity-aligned world pose using ROS ENU axes,
        # while PX4 VehicleOdometry is NED.  Convert the known axis convention
        # before fitting the one genuinely arbitrary degree of freedom (the
        # VINS world yaw).  A yaw-only fit on raw ENU/NED vectors cannot model
        # the XY reflection or the opposite vertical axis and previously made
        # the full-trajectory RMSE appear several metres too large.
        vio_ned = np.array([vio_pos[1], vio_pos[0], -vio_pos[2]], dtype=float)
        self.alignment_pairs.append((vio_ned, px4_pos.copy()))
        if len(self.alignment_pairs) < 6:
            return
        vio = np.asarray([pair[0] for pair in self.alignment_pairs])
        px4 = np.asarray([pair[1] for pair in self.alignment_pairs])
        dv = np.diff(vio[:, :2], axis=0)
        dp = np.diff(px4[:, :2], axis=0)
        # Very small ground-truth displacements contain no direction signal.
        mask = ((np.linalg.norm(dv, axis=1) > 0.02) &
                (np.linalg.norm(dp, axis=1) > 0.02))
        if int(np.count_nonzero(mask)) < 5:
            return
        dv = dv[mask]
        dp = dp[mask]
        dot = float(np.sum(dv[:, 0] * dp[:, 0] + dv[:, 1] * dp[:, 1]))
        cross = float(np.sum(dv[:, 0] * dp[:, 1] - dv[:, 1] * dp[:, 0]))
        self.alignment_yaw = math.atan2(cross, dot)
        c, s = math.cos(self.alignment_yaw), math.sin(self.alignment_yaw)
        rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        # Translation is fitted only.  No scale fit is allowed here.
        self.alignment_translation = np.mean(px4 - (rotation @ vio.T).T, axis=0)
        aligned = rotation @ vio_ned + self.alignment_translation
        error = float(np.linalg.norm(aligned - px4_pos))
        self.last_vector_error = error
        self.max_vector_error = max(self.max_vector_error, error)
        self.vector_error_squared_sum += error * error
        self.vector_error_count += 1

    def _vio_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        vel = np.array([v.x, v.y, v.z], dtype=float)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        finite = (np.isfinite(pos).all() and np.isfinite(vel).all() and
                  np.isfinite(quat).all())
        if not finite or not 0.95 <= np.linalg.norm(quat) <= 1.05:
            self.get_logger().error('VINS emitted invalid odometry; sample rejected.')
            self.vio_samples.clear()
            return
        self.last_vio_receive_time = self._now_s()
        step = 0.0 if self.last_vio_pos is None else np.linalg.norm(pos - self.last_vio_pos)
        self.last_vio_pos = pos
        self.max_vio_speed = max(self.max_vio_speed, float(np.linalg.norm(vel)))
        self.max_vio_step = max(self.max_vio_step, float(step))
        vio_stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._append_scale_pair(pos, vio_stamp_s)
        self.vio_samples.append((self._now_s(), pos, np.linalg.norm(vel), step))

    def _range_cb(self, msg):
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            self.last_range_time = self._now_s()

    def _publish_control(self):
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(mode)
        sp = TrajectorySetpoint()
        sp.position = self.command.astype(float).tolist()
        sp.yaw = float(self.yaw)
        sp.timestamp = mode.timestamp
        self.sp_pub.publish(sp)

    def _command_vehicle(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _move_command(self, target, speed):
        target = np.asarray(target, dtype=float)
        delta = target - self.command
        limit = speed * 0.05
        distance = np.linalg.norm(delta)
        if distance > limit:
            self.command += delta / distance * limit
        else:
            self.command = target
        return distance < 0.15

    def _vio_valid(self):
        now = self._now_s()
        recent = [s for s in self.vio_samples if now - s[0] <= 3.0]
        range_fresh = (self.last_range_time is not None and
                       now - self.last_range_time < 0.25)
        if (len(recent) < 20 or not range_fresh or
                self.scale_px4_path < 2.0 or self.scale_pair_count < 20):
            return False
        max_speed = max(s[2] for s in recent)
        max_step = max(s[3] for s in recent)
        scale_ratio = self.scale_vio_path / self.scale_px4_path
        # A finite, smooth estimate is not sufficient for a GPS→VIO handover:
        # the local VIO metre must agree with PX4/Gazebo metres.  Keep this
        # deliberately tight so a 30% scale error cannot be labelled READY.
        # READY requires a settled hover, not merely a finite estimate.  The
        # former 5 m/s limit admitted a VINS state still moving at 0.8 m/s,
        # which then crossed the 0.30 m drift gate immediately.
        valid = max_speed < 0.20 and max_step < 0.10 and 0.95 < scale_ratio < 1.05
        self.get_logger().info(
            f'validation: samples={len(recent)}, paired={self.scale_pair_count}, '
            f'max_speed={max_speed:.2f}, max_step={max_step:.2f}, '
            f'path_scale={scale_ratio:.2f}, '
            f'vector_error={self.last_vector_error if self.last_vector_error is not None else float("nan"):.2f}',
            throttle_duration_sec=1.0)
        return valid

    def _write_metrics(self, outcome):
        output = str(self.get_parameter('metrics_output').value)
        if not output:
            return
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        scale = (self.scale_vio_path / self.scale_px4_path
                 if self.scale_px4_path > 0.0 else None)
        # During excitation the yaw estimate evolves as pairs arrive.  Report
        # every pair against the final yaw/translation, not a mixture of early
        # provisional alignments and the final one.
        vector_rmse = None
        vector_max = None
        if self.alignment_yaw is not None and self.alignment_pairs:
            c, s = math.cos(self.alignment_yaw), math.sin(self.alignment_yaw)
            rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            vio = np.asarray([pair[0] for pair in self.alignment_pairs])
            px4 = np.asarray([pair[1] for pair in self.alignment_pairs])
            translation = np.mean(px4 - (rotation @ vio.T).T, axis=0)
            errors = np.linalg.norm((rotation @ vio.T).T + translation - px4,
                                    axis=1)
            vector_rmse = float(math.sqrt(np.mean(errors ** 2)))
            vector_max = float(np.max(errors))
        document = {
            'outcome': outcome,
            'sim_time_s': self._now_s(),
            'scale_path_ratio': scale,
            'scale_pair_count': self.scale_pair_count,
            'vins_recovery_count': self.vins_recovery_count,
            'alignment_pair_count': len(self.alignment_pairs),
            'yaw_alignment_rad_no_scale_fit': self.alignment_yaw,
            'vector_position_rmse_m_no_scale_fit': vector_rmse,
            'vector_position_max_error_m_no_scale_fit': vector_max,
            'max_vio_speed_mps': self.max_vio_speed,
            'max_vio_pose_step_m': self.max_vio_step,
        }
        path.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
        self.get_logger().info(f'Wrote preflight metrics: {path}')

    def _tick(self):
        # use_sim_time can become active just after construction.  Establish
        # all state deadlines on the first timer tick in the simulation epoch.
        if self.state_start is None:
            self.state_start = self.get_clock().now()
            return
        self._publish_control()
        altitude = float(self.get_parameter('takeoff_altitude').value)
        if self.state == 'WARMUP':
            if self._elapsed() >= float(self.get_parameter('warmup_seconds').value):
                self._transition('ARMING')
        elif self.state == 'ARMING':
            if not self.arm_sent or self._elapsed() > 1.0:
                self._command_vehicle(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._command_vehicle(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.arm_sent = True
                self.state_start = self.get_clock().now()
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self._transition('SLOW_TAKEOFF')
        elif self.state == 'SLOW_TAKEOFF':
            reached = self._move_command(
                (0.0, 0.0, -altitude),
                float(self.get_parameter('takeoff_speed').value))
            if reached and self.px4_pos is not None and abs(self.px4_pos[2] + altitude) < 0.25:
                self._transition('HOVER')
        elif self.state == 'HOVER':
            if self._elapsed() >= 2.0:
                vio_is_fresh = (self.last_vio_receive_time is not None and
                                self._now_s() - self.last_vio_receive_time <= 1.0)
                if vio_is_fresh:
                    self.get_logger().info(
                        'Keeping the VINS state initialized during takeoff; '
                        'skipping the destructive hover reset.')
                    self._transition('EXCITATION')
                elif self.reset_client.service_is_ready():
                    event = String(); event.data = 'VINS_RESET_REQUEST'
                    self.event_pub.publish(event)
                    self.reset_future = self.reset_client.call_async(Trigger.Request())
                    self._transition('RESET_VINS')
                else:
                    self.get_logger().warning(
                        'Waiting for /reset service before hover initialization.',
                        throttle_duration_sec=1.0)
        elif self.state == 'RESET_VINS':
            if self.reset_future is not None and self.reset_future.done():
                response = self.reset_future.result()
                if response is not None and response.success:
                    self.vio_samples.clear()
                    self.last_vio_pos = None
                    self.px4_history.clear()
                    self.scale_vio_path = self.scale_px4_path = 0.0
                    self.last_scale_vio_pos = self.last_scale_px4_pos = None
                    self.scale_pair_count = 0
                    # A reset creates a new arbitrary VINS local frame.  The
                    # scale accumulators above were already epoch-local, but
                    # retaining the old yaw/translation fit mixed two frame
                    # origins and made an otherwise healthy recovery time out
                    # with a multi-metre vector error.
                    self.alignment_pairs.clear()
                    self.alignment_yaw = None
                    self.alignment_translation = None
                    self.last_vector_error = None
                    self.max_vector_error = 0.0
                    self.vector_error_squared_sum = 0.0
                    self.vector_error_count = 0
                    self.get_logger().info(
                        'VINS reset at GPS hover; starting stationary warm-up.')
                    self._transition('HOVER_VINS_WARMUP')
                else:
                    self.get_logger().error('VINS reset failed; holding hover.')
                    self._transition('FAILSAFE_HOVER')
        elif self.state == 'HOVER_VINS_WARMUP':
            if self._elapsed() >= 2.0:
                self._transition('EXCITATION')
        elif self.state == 'EXCITATION':
            target = self.excitation[self.excitation_index]
            reached = self._move_command(target[:3],
                                         float(self.get_parameter('excitation_speed').value))
            yaw_reached = True
            if target[3] is not None and self.initial_yaw is not None:
                target_yaw = self.initial_yaw + target[3]
                yaw_delta = math.atan2(math.sin(target_yaw - self.yaw),
                                       math.cos(target_yaw - self.yaw))
                self.yaw += max(-0.02, min(0.02, yaw_delta))
                yaw_reached = abs(yaw_delta) < 0.05
            actual_reached = (self.px4_pos is not None and
                              np.linalg.norm(self.px4_pos - np.asarray(target[:3])) < 0.25)
            if reached and actual_reached and yaw_reached:
                self.excitation_index += 1
                if self.excitation_index == len(self.excitation):
                    self._transition('VALIDATE')
        elif self.state == 'VALIDATE':
            # VINS stops publishing after an internal failure reset.  Holding
            # hover until the generic validation timeout leaves it stationary,
            # so mono VIO cannot obtain the motion needed to initialize again.
            # Restart the controlled warm-up/excitation sequence instead.
            if (self.last_vio_receive_time is None or
                    self._now_s() - self.last_vio_receive_time > 1.0):
                if self.vins_recovery_count >= int(self.get_parameter('max_vins_recoveries').value):
                    self.get_logger().error('VINS output lost after all recovery attempts; holding GPS hover.')
                    self._write_metrics('VINS_OUTPUT_LOST')
                    self._transition('FAILSAFE_HOVER')
                    return
                if self.reset_client.service_is_ready():
                    self.vins_recovery_count += 1
                    self.excitation_index = 0
                    self.valid_since = None
                    event = String(); event.data = 'VINS_RESET_REQUEST'
                    self.event_pub.publish(event)
                    self.reset_future = self.reset_client.call_async(Trigger.Request())
                    self.get_logger().warning(
                        f'VINS output lost; recovery {self.vins_recovery_count} of '
                        f'{int(self.get_parameter("max_vins_recoveries").value)}.')
                    self._transition('RESET_VINS')
                    return
            if self._vio_valid():
                if self.valid_since is None:
                    self.valid_since = self._now_s()
                elif self._now_s() - self.valid_since >= 8.0:
                    self.ready_reference_vio = (None if self.last_vio_pos is None
                                                else self.last_vio_pos.copy())
                    self.ready_reference_px4 = (None if self.px4_pos is None
                                                else self.px4_pos.copy())
                    self._transition('READY_HOLD')
            else:
                self.valid_since = None
            if (self.state == 'VALIDATE' and self._elapsed() >
                    float(self.get_parameter('validation_timeout_seconds').value)):
                self.get_logger().error('VIO validation timed out; holding GPS hover, no handover.')
                self._write_metrics('VALIDATION_TIMEOUT')
                self._transition('FAILSAFE_HOVER')
        elif self.state == 'READY_HOLD':
            if self.ready_reference_vio is None or self.ready_reference_px4 is None:
                self._transition('FAILSAFE_HOVER')
                return
            vio_drift = (0.0 if self.last_vio_pos is None else float(
                np.linalg.norm(self.last_vio_pos - self.ready_reference_vio)))
            px4_drift = (0.0 if self.px4_pos is None else float(
                np.linalg.norm(self.px4_pos - self.ready_reference_px4)))
            # The position controller itself wanders a few centimetres in
            # hover.  Absolute VINS displacement therefore contains both real
            # vehicle motion and estimator drift.  Compare displacement
            # magnitudes so only the unmatched component is charged to VIO;
            # the two local frames have an arbitrary yaw, so raw vectors are
            # not directly subtractable here.
            drift_error = abs(vio_drift - px4_drift)
            self.get_logger().info(
                f'ready hold: vio_motion={vio_drift:.2f}m '
                f'px4_motion={px4_drift:.2f}m drift_error={drift_error:.2f}m',
                throttle_duration_sec=1.0)
            if drift_error > 0.30 or px4_drift > 0.25:
                self.get_logger().error('VIO hover drift exceeds acceptance limit; no handover.')
                self._write_metrics('FAILSAFE_HOVER')
                self._transition('FAILSAFE_HOVER')
            elif self._elapsed() >= float(self.get_parameter('ready_hold_seconds').value):
                ready = Bool(); ready.data = True; self.ready_pub.publish(ready)
                self.get_logger().info(
                    f'VIO READY: {self._elapsed():.0f} s hover drift check passed.')
                self._write_metrics('HOVER_READY')
                self._transition('HOVER_READY')
        elif self.state in ('HOVER_READY', 'FAILSAFE_HOVER'):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VioPreflightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
