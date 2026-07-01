#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from geometry_msgs.msg import Point
import math

class AutoMissionNode(Node):
    def __init__(self):
        super().__init__('auto_mission_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.state = 'INIT'
        self.offboard_setpoint_counter = 0
        self.current_pos = None

        # Mission Waypoints (NED)
        self.target_z_ned = -1.5
        self.waypoints = [
            Point(x=5.0, y=0.0, z=self.target_z_ned),
            Point(x=5.0, y=5.0, z=self.target_z_ned),
            Point(x=0.0, y=5.0, z=self.target_z_ned),
            Point(x=0.0, y=0.0, z=self.target_z_ned),
        ]
        self.current_wp_idx = 0
        self.reached_threshold = 0.5
        
        self.current_setpoint = None
        self.max_speed = 1.0
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('AutoMissionNode started.')

    def odom_callback(self, msg: VehicleOdometry):
        self.current_pos = msg.position

    def timer_callback(self):
        if self.state == 'INIT':
            self._pub_offboard_mode()
            self._pub_setpoint(0.0, 0.0, self.target_z_ned)
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info('Arming and switching to Offboard mode...')
                self._pub_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self._pub_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            self._pub_offboard_mode()
            self._pub_setpoint(0.0, 0.0, self.target_z_ned)
            if self.current_pos is not None and abs(self.current_pos[2] - self.target_z_ned) < 0.3:
                self.get_logger().info('Takeoff complete. Starting mission...')
                self.state = 'MISSION'
                self.current_setpoint = Point(x=float(self.current_pos[0]), 
                                              y=float(self.current_pos[1]), 
                                              z=float(self.current_pos[2]))

        elif self.state == 'MISSION':
            self._pub_offboard_mode()
            if self.current_pos is None:
                return

            target = self.waypoints[self.current_wp_idx]
            dist = math.sqrt((self.current_pos[0]-target.x)**2 + (self.current_pos[1]-target.y)**2 + (self.current_pos[2]-target.z)**2)
            
            if dist < self.reached_threshold:
                self.get_logger().info(f'Reached Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}')
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.get_logger().info('Mission Completed! Landing...')
                    self.state = 'LAND'
                    return

            dx = target.x - self.current_setpoint.x
            dy = target.y - self.current_setpoint.y
            dz = target.z - self.current_setpoint.z
            setpoint_dist = math.sqrt(dx**2 + dy**2 + dz**2)
            step_size = self.max_speed * self.dt

            if setpoint_dist > step_size:
                self.current_setpoint.x += (dx / setpoint_dist) * step_size
                self.current_setpoint.y += (dy / setpoint_dist) * step_size
                self.current_setpoint.z += (dz / setpoint_dist) * step_size
            else:
                self.current_setpoint.x = target.x
                self.current_setpoint.y = target.y
                self.current_setpoint.z = target.z
            
            self._pub_setpoint(self.current_setpoint.x, self.current_setpoint.y, self.current_setpoint.z)

        elif self.state == 'LAND':
            self._pub_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = 'DONE'

    def _pub_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def _pub_setpoint(self, x, y, z_ned):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z_ned)]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def _pub_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
