#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from px4_msgs.msg import VehicleOdometry, SensorCombined
import numpy as np
from scipy.spatial.transform import Rotation as R

class VioBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/vins_estimator/odometry', 
            self.odom_callback, 
            10
        )
        
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
        
        self.px4_imu_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.px4_imu_callback,
            px4_qos
        )
        
        # Transformation matrices
        self.R_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.R_flu_to_frd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        
        self.get_logger().info('VIO Bridge Node Started: VINS -> PX4 Odometry & PX4 IMU -> VINS')

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def px4_imu_callback(self, msg: SensorCombined):
        ros_imu = Imu()
        ros_imu.header.stamp = self.get_clock().now().to_msg()
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
        px4_msg = VehicleOdometry()
        px4_msg.timestamp = self._timestamp_us()
        px4_msg.timestamp_sample = px4_msg.timestamp
        
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        
        # Position ENU -> NED
        px4_msg.position[0] = float(msg.pose.pose.position.y)
        px4_msg.position[1] = float(msg.pose.pose.position.x)
        px4_msg.position[2] = float(-msg.pose.pose.position.z)
        
        # Orientation ENU -> NED
        q_ros = msg.pose.pose.orientation
        r_ros = R.from_quat([q_ros.x, q_ros.y, q_ros.z, q_ros.w]).as_matrix()
        r_px4 = self.R_enu_to_ned @ r_ros @ self.R_flu_to_frd
        q_px4 = R.from_matrix(r_px4).as_quat() # returns [x, y, z, w]
        
        px4_msg.q[0] = float(q_px4[3]) # w
        px4_msg.q[1] = float(q_px4[0]) # x
        px4_msg.q[2] = float(q_px4[1]) # y
        px4_msg.q[3] = float(q_px4[2]) # z
        
        # Velocity FLU -> FRD (Twist is in child frame)
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        px4_msg.velocity[0] = float(msg.twist.twist.linear.x)
        px4_msg.velocity[1] = float(-msg.twist.twist.linear.y)
        px4_msg.velocity[2] = float(-msg.twist.twist.linear.z)
        
        # Angular Velocity FLU -> FRD
        px4_msg.angular_velocity[0] = float(msg.twist.twist.angular.x)
        px4_msg.angular_velocity[1] = float(-msg.twist.twist.angular.y)
        px4_msg.angular_velocity[2] = float(-msg.twist.twist.angular.z)
        
        # Dummy Variances
        px4_msg.position_variance = [0.01, 0.01, 0.01]
        px4_msg.orientation_variance = [0.01, 0.01, 0.01]
        px4_msg.velocity_variance = [0.01, 0.01, 0.01]
        
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
