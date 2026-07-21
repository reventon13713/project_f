#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleOdometry, SensorCombined
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class VioBridge(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('vio_bridge', parameter_overrides=parameter_overrides)

        # VINS publishes a root-level "odometry" topic unless it is explicitly
        # placed in a namespace.  Make the topic configurable, but keep the
        # default aligned with vins_fusion_ros2.launch.py.
        self.declare_parameter('vins_odom_topic', '/odometry')
        # ``enu_flu`` is the legacy/raw VINS contract.  Stage 4 consumes the
        # global-position node's already aligned PX4 local-frame output:
        # position/velocity NED and attitude body-FRD -> NED.
        self.declare_parameter('odom_frame_contract', 'enu_flu')
        self.declare_parameter('readiness_topic', '/vio/ready')
        # Keep estimator evaluation independent of PX4's EKF.  Feeding an
        # unvalidated VINS pose back into PX4 makes the "ground truth" path
        # used by the preflight test circular and can destabilize the flight.
        # Enable this only after the preflight scale check reports READY.
        self.declare_parameter('publish_visual_odometry', False)
        self.declare_parameter('max_output_age_seconds', 0.20)
        self.declare_parameter('px4_timestamp_sample_from_receipt', False)
        self.declare_parameter('max_position_step_m', 0.15)
        self.declare_parameter('max_implied_speed_mps', 1.5)
        self.declare_parameter('required_healthy_samples', 10)
        # A reset can change the arbitrary VINS origin/yaw. Reconnecting that
        # frame during flight can move PX4's local state and controller
        # setpoints even when reset_counter is correct. Require a bridge/node
        # restart by default; controlled tests may explicitly opt in.
        self.declare_parameter('allow_rehandover_after_fault', False)
        vins_odom_topic = self.get_parameter('vins_odom_topic').value
        self.odom_frame_contract = str(
            self.get_parameter('odom_frame_contract').value)
        if self.odom_frame_contract not in ('enu_flu', 'ned_frd'):
            raise ValueError(
                "odom_frame_contract must be 'enu_flu' or 'ned_frd'")
        readiness_topic = str(self.get_parameter('readiness_topic').value)
        self.publish_visual_odometry = bool(
            self.get_parameter('publish_visual_odometry').value)
        self.max_output_age_seconds = float(
            self.get_parameter('max_output_age_seconds').value)
        self.px4_timestamp_sample_from_receipt = bool(
            self.get_parameter('px4_timestamp_sample_from_receipt').value)
        self.max_position_step_m = float(
            self.get_parameter('max_position_step_m').value)
        self.max_implied_speed_mps = float(
            self.get_parameter('max_implied_speed_mps').value)
        self.required_healthy_samples = int(
            self.get_parameter('required_healthy_samples').value)
        self.allow_rehandover_after_fault = bool(
            self.get_parameter('allow_rehandover_after_fault').value)
        self._ready_generation = 0
        self._required_ready_generation = 1
        # PX4 uses this counter to distinguish a discontinuous external-
        # vision reference frame from ordinary motion. Every newly approved
        # VIO generation may have a different origin after estimator reset.
        self._active_reset_counter = 0
        self._handover_is_open = False
        self._fault_latched = False
        self._healthy_sample_count = 0
        self._last_vio_position = None
        self._last_vio_sample_ns = 0
        self._last_vio_receive_ns = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            vins_odom_topic,
            self.odom_callback,
            10
        )
        self.create_subscription(Bool, readiness_topic, self._ready_callback, 10)
        self.create_timer(0.1, self._health_timer)

        self.imu_pub = self.create_publisher(
            Imu, 
            '/imu/data', 
            10
        )
        
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            px4_qos
        )
        self.handover_ready_pub = self.create_publisher(
            Bool, '/vio/handover_ready', 10)
        
        self.px4_imu_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.px4_imu_callback,
            px4_qos
        )
        
        # Transformation matrices
        self.R_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.R_flu_to_frd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        # PX4 SITL's uORB timestamp and Gazebo /clock have different epochs.
        # The first IMU sample establishes the epoch conversion; subsequent
        # samples retain the PX4 measurement-time spacing.
        self._px4_to_ros_offset_us = None
        self._last_px4_timestamp_us = 0
        self._last_ros_imu_timestamp_us = 0
        self._clock_us = 0
        self._clock_monotonic_ns = 0
        self._previous_clock_us = 0
        self._previous_clock_monotonic_ns = 0
        self._clock_sequence = 0
        self._offset_clock_sequence = -1
        self._sim_rate = 1.0
        self.create_subscription(Clock, '/clock', self._clock_callback,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f'VIO Bridge started: {vins_odom_topic} -> PX4 visual odometry, '
            f'PX4 SensorCombined -> /imu/data '
            f'(contract={self.odom_frame_contract}, ready={readiness_topic}, '
            f'visual odometry publish={self.publish_visual_odometry})')

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _ready_callback(self, msg: Bool):
        if msg.data:
            if self._fault_latched:
                self.get_logger().error(
                    'Ignoring /vio/ready after an in-flight VIO fault; '
                    'restart the bridge for an explicit new handover.',
                    throttle_duration_sec=2.0)
                return
            self._ready_generation += 1
            self._active_reset_counter = self._ready_generation & 0xff
            self._healthy_sample_count = 0
            self.get_logger().info(
                f'Received VIO readiness generation {self._ready_generation}.')

    def _invalidate_handover(self, reason: str):
        was_open = self._handover_is_open
        self._handover_is_open = False
        if was_open and not self.allow_rehandover_after_fault:
            self._fault_latched = True
        self._healthy_sample_count = 0
        self._last_vio_position = None
        self._last_vio_sample_ns = 0
        self._required_ready_generation = max(
            self._required_ready_generation, self._ready_generation + 1)
        recovery = ('bridge restart is required'
                    if self._fault_latched
                    else 'a new /vio/ready generation is required')
        self.get_logger().warning(
            f'VIO handover gate closed: {reason}; {recovery}.',
            throttle_duration_sec=1.0)
        status = Bool(); status.data = False
        self.handover_ready_pub.publish(status)

    def _health_timer(self):
        if self._last_vio_receive_ns == 0:
            return
        age = (self.get_clock().now().nanoseconds - self._last_vio_receive_ns) * 1e-9
        if age > self.max_output_age_seconds:
            self._invalidate_handover(f'output stale for {age:.3f} s')
            self._last_vio_receive_ns = 0

    def _clock_callback(self, msg: Clock):
        now_monotonic_ns = time.monotonic_ns()
        clock_us = (int(msg.clock.sec) * 1_000_000 +
                    int(msg.clock.nanosec) // 1_000)
        if self._previous_clock_monotonic_ns:
            wall_delta_us = ((now_monotonic_ns -
                              self._previous_clock_monotonic_ns) / 1_000)
            sim_delta_us = clock_us - self._previous_clock_us
            if wall_delta_us > 0:
                measured_rate = sim_delta_us / wall_delta_us
                if 0.1 <= measured_rate <= 5.0:
                    self._sim_rate = measured_rate
        self._previous_clock_us = clock_us
        self._previous_clock_monotonic_ns = now_monotonic_ns
        self._clock_us = clock_us
        self._clock_monotonic_ns = now_monotonic_ns
        self._clock_sequence += 1

    def _estimated_sim_time_us(self) -> int:
        """Interpolate between /clock updates for high-rate IMU time stamps."""
        if self._clock_monotonic_ns == 0:
            return 0
        elapsed_us = ((time.monotonic_ns() - self._clock_monotonic_ns) / 1_000)
        return int(self._clock_us + elapsed_us * self._sim_rate)

    @staticmethod
    def _time_msg_from_us(timestamp_us: int):
        """Convert a microsecond timestamp in the ROS/Gazebo clock to Time."""
        from builtin_interfaces.msg import Time

        stamp = Time()
        stamp.sec = int(timestamp_us // 1_000_000)
        stamp.nanosec = int((timestamp_us % 1_000_000) * 1_000)
        return stamp

    @staticmethod
    def _time_us_from_msg(stamp) -> int:
        return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000

    def px4_imu_callback(self, msg: SensorCombined):
        ros_imu = Imu()
        # PX4 SITL's SensorCombined timestamp has a different epoch from
        # Gazebo /clock.  Align epochs once, then preserve the PX4 measurement
        # time deltas so high-rate IMU samples do not all share one /clock tick.
        sample_timestamp_us = int(msg.timestamp)
        estimated_sim_us = self._estimated_sim_time_us()
        if sample_timestamp_us <= 0 or estimated_sim_us <= 0:
            self.get_logger().warning(
                'Waiting for valid PX4 IMU timestamps and Gazebo /clock.',
                throttle_duration_sec=5.0)
            return

        if self._px4_to_ros_offset_us is None:
            self._px4_to_ros_offset_us = estimated_sim_us - sample_timestamp_us
            self._offset_clock_sequence = self._clock_sequence
            self.get_logger().info(
                'Established PX4 IMU ↔ Gazebo clock offset for VIO synchronization.')

        # Refresh the epoch alignment just after each /clock update. This
        # removes the up-to-100 ms phase error caused by Gazebo publishing
        # /clock at a lower rate than the IMU stream, while preserving the
        # high-rate PX4 sample spacing between updates.
        if self._clock_sequence != self._offset_clock_sequence:
            candidate_offset = estimated_sim_us - sample_timestamp_us
            self._px4_to_ros_offset_us = (
                0.8 * self._px4_to_ros_offset_us + 0.2 * candidate_offset)
            self._offset_clock_sequence = self._clock_sequence

        ros_timestamp_us = sample_timestamp_us + self._px4_to_ros_offset_us
        if ros_timestamp_us <= self._last_ros_imu_timestamp_us:
            # Ignore duplicate/out-of-order samples rather than violating the
            # strictly increasing timestamp order expected by VINS.
            return

        self._last_px4_timestamp_us = sample_timestamp_us
        self._last_ros_imu_timestamp_us = ros_timestamp_us
        ros_imu.header.stamp = self._time_msg_from_us(ros_timestamp_us)
        ros_imu.header.frame_id = 'base_link'
        
        # PX4 FRD to ROS FLU
        ros_imu.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        ros_imu.linear_acceleration.y = float(-msg.accelerometer_m_s2[1])
        ros_imu.linear_acceleration.z = float(-msg.accelerometer_m_s2[2])
        
        ros_imu.angular_velocity.x = float(msg.gyro_rad[0])
        ros_imu.angular_velocity.y = float(-msg.gyro_rad[1])
        ros_imu.angular_velocity.z = float(-msg.gyro_rad[2])
        
        self.imu_pub.publish(ros_imu)

    def odom_callback(self, msg: Odometry):
        receive_ns = self.get_clock().now().nanoseconds
        sample_ns = (int(msg.header.stamp.sec) * 1_000_000_000 +
                     int(msg.header.stamp.nanosec))
        age_seconds = max(0.0, (receive_ns - sample_ns) * 1e-9)
        if sample_ns <= 0 or age_seconds > self.max_output_age_seconds:
            self._invalidate_handover(f'invalid output age {age_seconds:.3f} s')
            return
        position = np.array([msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z], dtype=float)
        if not np.isfinite(position).all():
            self._invalidate_handover('non-finite position')
            return
        if self._last_vio_position is not None:
            if sample_ns <= self._last_vio_sample_ns:
                self._invalidate_handover('non-monotonic sample timestamp')
                return
            position_step = float(np.linalg.norm(
                position - self._last_vio_position))
            sample_dt = (sample_ns - self._last_vio_sample_ns) * 1e-9
            if position_step > self.max_position_step_m:
                self._invalidate_handover('position discontinuity')
                return
            if position_step / sample_dt > self.max_implied_speed_mps:
                self._invalidate_handover('implausible position rate')
                return
        self._last_vio_position = position
        self._last_vio_sample_ns = sample_ns
        self._last_vio_receive_ns = receive_ns
        q_ros = msg.pose.pose.orientation
        quaternion = np.array([q_ros.x, q_ros.y, q_ros.z, q_ros.w])
        if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-6:
            self._invalidate_handover('invalid quaternion')
            return

        if self._ready_generation < self._required_ready_generation:
            return
        self._healthy_sample_count += 1
        if self._healthy_sample_count < self.required_healthy_samples:
            return
        self._handover_is_open = True
        status = Bool(); status.data = True
        self.handover_ready_pub.publish(status)
        if not self.publish_visual_odometry:
            return

        px4_msg = VehicleOdometry()
        ros_sample_timestamp_us = self._time_us_from_msg(msg.header.stamp)
        if self._px4_to_ros_offset_us is None or ros_sample_timestamp_us <= 0:
            self.get_logger().warning(
                'Ignoring VINS odometry until the PX4↔Gazebo clock mapping is ready.',
                throttle_duration_sec=1.0)
            return

        # Convert the VINS/Gazebo sample time back to PX4's local clock for
        # VehicleOdometry. PX4 requires both fields in its own time domain.
        px4_msg.timestamp_sample = int(max(
            0, ros_sample_timestamp_us - self._px4_to_ros_offset_us))
        if self.px4_timestamp_sample_from_receipt:
            # The aligned odometry pose is the estimator's latest propagated
            # state at publication. In high-latency SITL runs, using its camera
            # header makes PX4 treat a current pose as an old measurement and
            # repeatedly stop EV aid. Receipt mode is explicit and opt-in.
            px4_msg.timestamp_sample = int(self._last_px4_timestamp_us)
        px4_msg.timestamp = int(max(self._last_px4_timestamp_us,
                                    px4_msg.timestamp_sample))
        
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        
        if self.odom_frame_contract == 'enu_flu':
            # Raw VINS: position/velocity are ENU and attitude is body-FLU ->
            # ENU, so convert to PX4's NED/FRD convention.
            px4_msg.position[0] = float(msg.pose.pose.position.y)
            px4_msg.position[1] = float(msg.pose.pose.position.x)
            px4_msg.position[2] = float(-msg.pose.pose.position.z)
            r_ros = R.from_quat(quaternion).as_matrix()
            r_px4 = self.R_enu_to_ned @ r_ros @ self.R_flu_to_frd
            velocity = np.array([
                msg.twist.twist.linear.y, msg.twist.twist.linear.x,
                -msg.twist.twist.linear.z], dtype=float)
        else:
            # /vio/aligned_odometry is already PX4 local NED with body-FRD
            # attitude.  Applying ENU/FLU conversion again would swap axes
            # and invert Z, creating a false EV measurement.
            px4_msg.position = [float(position[0]), float(position[1]),
                                 float(position[2])]
            r_px4 = R.from_quat(quaternion).as_matrix()
            velocity = np.array([
                msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                msg.twist.twist.linear.z], dtype=float)
        if not np.isfinite(velocity).all():
            self._invalidate_handover('non-finite velocity')
            return
        q_px4 = R.from_matrix(r_px4).as_quat() # returns [x, y, z, w]
        
        px4_msg.q[0] = float(q_px4[3]) # w
        px4_msg.q[1] = float(q_px4[0]) # x
        px4_msg.q[2] = float(q_px4[1]) # y
        px4_msg.q[3] = float(q_px4[2]) # z
        
        # VehicleOdometry velocity is expressed in its selected world frame.
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        px4_msg.velocity[0] = float(velocity[0])
        px4_msg.velocity[1] = float(velocity[1])
        px4_msg.velocity[2] = float(velocity[2])

        # VINS does not estimate angular velocity in this Odometry output.
        # NaN explicitly tells PX4 that this field is unavailable.
        px4_msg.angular_velocity = [math.nan, math.nan, math.nan]

        # Conservative, non-zero uncertainty.  These should later be replaced
        # by VINS covariance once the estimator exposes it.
        px4_msg.position_variance = [0.10, 0.10, 0.15]
        px4_msg.orientation_variance = [0.10, 0.10, 0.15]
        px4_msg.velocity_variance = [0.20, 0.20, 0.25]
        px4_msg.reset_counter = self._active_reset_counter
        px4_msg.quality = 50
        
        self.visual_odom_pub.publish(px4_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
