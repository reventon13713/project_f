#!/usr/bin/env python3
"""Lock a VIO local frame to PX4 NED and publish a WGS84 position.

Alignment is learned only while PX4 has a valid georeferenced local position
and a healthy 3-D GPS fix.  Once locked, a VIO epoch fault is fail-closed: a
new arbitrary VIO frame is never joined to the old global trajectory in
flight.  Restarting this node is the explicit recovery action.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorGps, VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class SimilarityYaw:
    """A scale, yaw and translation mapping VIO ENU coordinates to NED."""

    scale: float
    yaw_rad: float
    translation_ned: np.ndarray
    rmse_m: float
    max_error_m: float

    def apply(self, vio_enu):
        vio_ned = enu_to_ned(vio_enu)
        return self.scale * (self.rotation_ned() @ vio_ned) + self.translation_ned

    def rotation_ned(self):
        c = math.cos(self.yaw_rad)
        s = math.sin(self.yaw_rad)
        return np.array([[c, -s, 0.0],
                         [s, c, 0.0],
                         [0.0, 0.0, 1.0]])

    def apply_vector(self, vio_enu):
        return self.scale * (self.rotation_ned() @ enu_to_ned(vio_enu))


def enu_to_ned(vector):
    vector = np.asarray(vector, dtype=float)
    return np.array([vector[1], vector[0], -vector[2]], dtype=float)


def fit_similarity_yaw(vio_enu_samples, ned_samples):
    """Least-squares no-tilt Sim(3), with a shared XYZ scale."""
    vio = np.asarray([enu_to_ned(p) for p in vio_enu_samples], dtype=float)
    ned = np.asarray(ned_samples, dtype=float)
    if vio.shape != ned.shape or vio.ndim != 2 or vio.shape[1] != 3:
        raise ValueError('alignment samples must be matching Nx3 arrays')
    if len(vio) < 3 or not np.isfinite(vio).all() or not np.isfinite(ned).all():
        raise ValueError('alignment needs at least three finite samples')

    vio_center = vio.mean(axis=0)
    ned_center = ned.mean(axis=0)
    v = vio - vio_center
    n = ned - ned_center
    # R(yaw) maximizes trace(R H) in horizontal NED.  Vertical is deliberately
    # not allowed to tilt the frame, but it contributes to the shared scale.
    a = float(np.sum(v[:, 0] * n[:, 0] + v[:, 1] * n[:, 1]))
    b = float(np.sum(v[:, 0] * n[:, 1] - v[:, 1] * n[:, 0]))
    yaw = math.atan2(b, a)
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotated = np.column_stack((c * v[:, 0] - s * v[:, 1],
                               s * v[:, 0] + c * v[:, 1],
                               v[:, 2]))
    denominator = float(np.sum(rotated * rotated))
    if denominator < 1e-6:
        raise ValueError('alignment motion is degenerate')
    scale = float(np.sum(rotated * n) / denominator)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    translation = ned_center - scale * (rotation @ vio_center)
    predicted = scale * (vio @ rotation.T) + translation
    error = np.linalg.norm(predicted - ned, axis=1)
    return SimilarityYaw(scale, yaw, translation,
                         float(np.sqrt(np.mean(error ** 2))),
                         float(np.max(error)))


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    prime_vertical = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat ** 2)
    return np.array([
        (prime_vertical + altitude_m) * math.cos(lat) * math.cos(lon),
        (prime_vertical + altitude_m) * math.cos(lat) * math.sin(lon),
        (prime_vertical * (1.0 - WGS84_E2) + altitude_m) * sin_lat,
    ])


def ecef_to_geodetic(ecef):
    x, y, z = np.asarray(ecef, dtype=float)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    altitude = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        prime_vertical = WGS84_A_M / math.sqrt(
            1.0 - WGS84_E2 * sin_lat ** 2)
        altitude = p / max(math.cos(lat), 1e-12) - prime_vertical
        updated = math.atan2(
            z, p * (1.0 - WGS84_E2 * prime_vertical /
                    (prime_vertical + altitude)))
        if abs(updated - lat) < 1e-13:
            lat = updated
            break
        lat = updated
    return math.degrees(lat), math.degrees(lon), altitude


def ned_to_geodetic(ned, origin):
    """Convert local tangent NED metres to WGS84 geodetic coordinates."""
    lat_deg, lon_deg, alt_m = origin
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    north, east, down = np.asarray(ned, dtype=float)
    enu = np.array([east, north, -down])
    ecef_from_enu = np.array([
        [-math.sin(lon), -math.sin(lat) * math.cos(lon),
         math.cos(lat) * math.cos(lon)],
        [math.cos(lon), -math.sin(lat) * math.sin(lon),
         math.cos(lat) * math.sin(lon)],
        [0.0, math.cos(lat), math.sin(lat)],
    ])
    return ecef_to_geodetic(
        geodetic_to_ecef(lat_deg, lon_deg, alt_m) + ecef_from_enu @ enu)


class VioGlobalPosition(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('vio_global_position',
                         parameter_overrides=parameter_overrides)
        self.declare_parameter('vins_odom_topic', '/odometry')
        self.declare_parameter('minimum_alignment_samples', 30)
        self.declare_parameter('minimum_horizontal_span_m', 0.8)
        self.declare_parameter('minimum_vertical_span_m', 0.3)
        self.declare_parameter('minimum_scale', 0.8)
        self.declare_parameter('maximum_scale', 1.2)
        self.declare_parameter('maximum_alignment_rmse_m', 0.25)
        self.declare_parameter('maximum_alignment_error_m', 0.50)
        self.declare_parameter('maximum_source_age_seconds', 0.25)
        self.declare_parameter('maximum_vio_gap_seconds', 0.50)
        self.declare_parameter('maximum_vio_step_m', 0.50)
        self.declare_parameter('minimum_gps_fix_type', 3)
        self.declare_parameter('maximum_gps_eph_m', 3.0)
        self.declare_parameter('maximum_gps_epv_m', 5.0)
        self.declare_parameter('alignment_history_size', 2400)

        get = lambda name: self.get_parameter(name).value
        self.min_samples = int(get('minimum_alignment_samples'))
        self.min_horizontal_span = float(get('minimum_horizontal_span_m'))
        self.min_vertical_span = float(get('minimum_vertical_span_m'))
        self.min_scale = float(get('minimum_scale'))
        self.max_scale = float(get('maximum_scale'))
        self.max_rmse = float(get('maximum_alignment_rmse_m'))
        self.max_error = float(get('maximum_alignment_error_m'))
        self.max_source_age_ns = int(float(get('maximum_source_age_seconds')) * 1e9)
        self.max_vio_gap_ns = int(float(get('maximum_vio_gap_seconds')) * 1e9)
        self.max_vio_step = float(get('maximum_vio_step_m'))
        self.min_gps_fix = int(get('minimum_gps_fix_type'))
        self.max_gps_eph = float(get('maximum_gps_eph_m'))
        self.max_gps_epv = float(get('maximum_gps_epv_m'))

        self.samples = deque(maxlen=int(get('alignment_history_size')))
        self.local_position = None
        self.local_receive_ns = 0
        self.gps = None
        self.gps_receive_ns = 0
        self.ready_generation = 0
        self.alignment = None
        self.origin = None
        self.alignment_eph = math.nan
        self.alignment_epv = math.nan
        self.fault_latched = False
        self.last_vio_position = None
        self.last_vio_sample_ns = 0
        self.last_vio_receive_ns = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            Odometry, str(get('vins_odom_topic')), self._vio_callback, 20)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_callback, px4_qos)
        self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self._gps_callback,
            px4_qos)
        self.create_subscription(Bool, '/vio/ready', self._ready_callback, 10)
        self.global_pub = self.create_publisher(
            NavSatFix, '/vio/global_position', 10)
        self.aligned_odom_pub = self.create_publisher(
            Odometry, '/vio/aligned_odometry', 10)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.status_pub = self.create_publisher(
            Bool, '/vio/global_alignment_ready', status_qos)
        self.create_timer(0.1, self._health_timer)
        self._publish_status(False)
        self.get_logger().info(
            'VIO global position started; waiting for synchronized '
            'VIO/GPS/PX4 alignment samples.')

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _publish_status(self, ready):
        msg = Bool()
        msg.data = bool(ready)
        self.status_pub.publish(msg)

    def _local_callback(self, msg):
        self.local_position = msg
        self.local_receive_ns = self._now_ns()

    def _gps_callback(self, msg):
        self.gps = msg
        self.gps_receive_ns = self._now_ns()

    def _ready_callback(self, msg):
        if not msg.data:
            return
        self.ready_generation += 1
        if self.alignment is not None or self.fault_latched:
            self._latch_fault('new VIO readiness generation after alignment')
            return
        self._try_lock_alignment()

    def _healthy_reference(self, now_ns):
        local = self.local_position
        gps = self.gps
        if local is None or gps is None:
            return False
        if (now_ns - self.local_receive_ns > self.max_source_age_ns or
                now_ns - self.gps_receive_ns > self.max_source_age_ns):
            return False
        if not (local.xy_valid and local.z_valid and local.xy_global and
                local.z_global):
            return False
        if (gps.fix_type < self.min_gps_fix or not np.isfinite(gps.eph) or
                not np.isfinite(gps.epv) or gps.eph > self.max_gps_eph or
                gps.epv > self.max_gps_epv):
            return False
        values = (local.x, local.y, local.z, local.ref_lat, local.ref_lon,
                  local.ref_alt)
        return np.isfinite(values).all()

    def _start_new_prelock_epoch(self, reason):
        self.samples.clear()
        self.last_vio_position = None
        self.last_vio_sample_ns = 0
        self.get_logger().warning(
            f'Cleared pre-alignment VIO epoch: {reason}.',
            throttle_duration_sec=1.0)

    def _latch_fault(self, reason):
        if self.fault_latched:
            return
        self.fault_latched = True
        self.alignment = None
        self._publish_status(False)
        self.get_logger().error(
            f'VIO global alignment latched closed: {reason}; restart the '
            'node for an explicit realignment.')

    def _health_timer(self):
        if self.last_vio_receive_ns == 0:
            return
        age = self._now_ns() - self.last_vio_receive_ns
        if age <= self.max_vio_gap_ns:
            return
        if self.alignment is not None:
            self._latch_fault(f'VIO output gap {age * 1e-9:.3f} s')
        elif self.last_vio_position is not None:
            self._start_new_prelock_epoch(
                f'VIO output gap {age * 1e-9:.3f} s')
        self.last_vio_receive_ns = 0

    @staticmethod
    def _stamp_ns(msg):
        return (int(msg.header.stamp.sec) * 1_000_000_000 +
                int(msg.header.stamp.nanosec))

    def _vio_callback(self, msg):
        if self.fault_latched:
            return
        now_ns = self._now_ns()
        sample_ns = self._stamp_ns(msg)
        position = np.array([msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z], dtype=float)
        if sample_ns <= 0 or not np.isfinite(position).all():
            if self.alignment is not None:
                self._latch_fault('invalid VIO sample')
            else:
                self._start_new_prelock_epoch('invalid VIO sample')
            return
        if self.last_vio_position is not None:
            reason = None
            if sample_ns <= self.last_vio_sample_ns:
                reason = 'non-monotonic VIO timestamp'
            elif (sample_ns - self.last_vio_sample_ns > self.max_vio_gap_ns or
                  (self.last_vio_receive_ns and
                   now_ns - self.last_vio_receive_ns > self.max_vio_gap_ns)):
                reason = 'VIO output gap'
            elif np.linalg.norm(position - self.last_vio_position) > self.max_vio_step:
                reason = 'VIO position discontinuity'
            if reason:
                if self.alignment is not None:
                    self._latch_fault(reason)
                    return
                else:
                    self._start_new_prelock_epoch(reason)
        self.last_vio_position = position
        self.last_vio_sample_ns = sample_ns
        self.last_vio_receive_ns = now_ns

        if self.alignment is None:
            if self._healthy_reference(now_ns):
                local = self.local_position
                # SensorGps altitude fields provide the geoid separation needed
                # to turn PX4's AMSL ref_alt into NavSatFix ellipsoid altitude.
                geoid_separation = ((float(self.gps.alt_ellipsoid) -
                                     float(self.gps.alt)) * 1e-3)
                self.samples.append((
                    position.copy(),
                    np.array([local.x, local.y, local.z], dtype=float),
                    float(local.ref_lat), float(local.ref_lon),
                    float(local.ref_alt), geoid_separation,
                    float(self.gps.eph), float(self.gps.epv)))
            if self.ready_generation > 0:
                self._try_lock_alignment()
            return

        ned = self.alignment.apply(position)
        self._publish_outputs(msg, ned)

    def _try_lock_alignment(self):
        if self.fault_latched or self.alignment is not None:
            return
        if len(self.samples) < self.min_samples:
            return
        vio = np.asarray([sample[0] for sample in self.samples])
        ned = np.asarray([sample[1] for sample in self.samples])
        horizontal_span = float(np.linalg.norm(
            np.ptp(ned[:, :2], axis=0)))
        vertical_span = float(np.ptp(ned[:, 2]))
        if (horizontal_span < self.min_horizontal_span or
                vertical_span < self.min_vertical_span):
            return
        try:
            candidate = fit_similarity_yaw(vio, ned)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return
        if not (self.min_scale <= candidate.scale <= self.max_scale):
            self.get_logger().error(
                f'Rejected global alignment scale {candidate.scale:.4f}.',
                throttle_duration_sec=2.0)
            return
        if (candidate.rmse_m > self.max_rmse or
                candidate.max_error_m > self.max_error):
            self.get_logger().error(
                f'Rejected global alignment error RMSE/max '
                f'{candidate.rmse_m:.3f}/{candidate.max_error_m:.3f} m.',
                throttle_duration_sec=2.0)
            return

        refs = np.asarray([[s[2], s[3], s[4], s[5]] for s in self.samples])
        ref_lat, ref_lon, ref_alt_msl, geoid = np.median(refs, axis=0)
        # Ref changes during sample collection indicate a PX4 local/global
        # reset.  Do not average two distinct global frames.
        if (np.ptp(refs[:, 0]) > 1e-8 or np.ptp(refs[:, 1]) > 1e-8 or
                np.ptp(refs[:, 2]) > 0.05):
            self._start_new_prelock_epoch('PX4 global reference changed')
            return
        self.origin = (float(ref_lat), float(ref_lon),
                       float(ref_alt_msl + geoid))
        self.alignment_eph = max(candidate.rmse_m,
                                 float(np.median([s[6] for s in self.samples])))
        self.alignment_epv = max(candidate.rmse_m,
                                 float(np.median([s[7] for s in self.samples])))
        self.alignment = candidate
        self._publish_status(True)
        self.get_logger().info(
            f'Locked VIO global alignment: samples={len(self.samples)}, '
            f'scale={candidate.scale:.5f}, yaw={math.degrees(candidate.yaw_rad):.2f} deg, '
            f'RMSE/max={candidate.rmse_m:.3f}/{candidate.max_error_m:.3f} m, '
            f'origin=({self.origin[0]:.8f}, {self.origin[1]:.8f}, '
            f'{self.origin[2]:.2f} m ellipsoid).')

    def _publish_outputs(self, source, ned):
        latitude, longitude, altitude = ned_to_geodetic(ned, self.origin)
        fix = NavSatFix()
        fix.header = source.header
        fix.header.frame_id = 'base_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude
        fix.position_covariance = [
            self.alignment_eph ** 2, 0.0, 0.0,
            0.0, self.alignment_eph ** 2, 0.0,
            0.0, 0.0, self.alignment_epv ** 2]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.global_pub.publish(fix)

        aligned = Odometry()
        aligned.header = source.header
        aligned.header.frame_id = 'map_ned'
        aligned.child_frame_id = 'base_link_frd'
        aligned.pose.pose.position.x = float(ned[0])
        aligned.pose.pose.position.y = float(ned[1])
        aligned.pose.pose.position.z = float(ned[2])
        source_q = source.pose.pose.orientation
        quaternion = np.array(
            [source_q.x, source_q.y, source_q.z, source_q.w], dtype=float)
        if np.isfinite(quaternion).all() and np.linalg.norm(quaternion) > 1e-6:
            enu_to_ned_rotation = np.array(
                [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
            flu_to_frd = np.diag([1.0, -1.0, -1.0])
            body_to_aligned_ned = (
                self.alignment.rotation_ned() @ enu_to_ned_rotation @
                Rotation.from_quat(quaternion).as_matrix() @ flu_to_frd)
            q = Rotation.from_matrix(body_to_aligned_ned).as_quat()
            aligned.pose.pose.orientation.x = float(q[0])
            aligned.pose.pose.orientation.y = float(q[1])
            aligned.pose.pose.orientation.z = float(q[2])
            aligned.pose.pose.orientation.w = float(q[3])
        else:
            aligned.pose.pose.orientation.w = 1.0
            aligned.pose.covariance[21] = math.inf
            aligned.pose.covariance[28] = math.inf
            aligned.pose.covariance[35] = math.inf
        source_velocity = np.array([
            source.twist.twist.linear.x, source.twist.twist.linear.y,
            source.twist.twist.linear.z], dtype=float)
        if np.isfinite(source_velocity).all():
            velocity_ned = self.alignment.apply_vector(source_velocity)
            aligned.twist.twist.linear.x = float(velocity_ned[0])
            aligned.twist.twist.linear.y = float(velocity_ned[1])
            aligned.twist.twist.linear.z = float(velocity_ned[2])
        aligned.pose.covariance[0] = self.alignment_eph ** 2
        aligned.pose.covariance[7] = self.alignment_eph ** 2
        aligned.pose.covariance[14] = self.alignment_epv ** 2
        aligned.pose.covariance[21] = max(aligned.pose.covariance[21], 0.01)
        aligned.pose.covariance[28] = max(aligned.pose.covariance[28], 0.01)
        aligned.pose.covariance[35] = max(aligned.pose.covariance[35], 0.01)
        aligned.twist.covariance[0] = 0.04
        aligned.twist.covariance[7] = 0.04
        aligned.twist.covariance[14] = 0.0625
        self.aligned_odom_pub.publish(aligned)


def main(args=None):
    rclpy.init(args=args)
    node = VioGlobalPosition()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
