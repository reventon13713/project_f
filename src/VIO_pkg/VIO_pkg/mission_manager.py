#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import math

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # PX4 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.target_pub = self.create_publisher(Point, '/mission/target_waypoint', 10)
        self.completed_pub = self.create_publisher(Bool, '/mission/completed', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)

        # Waypoint list (NED coordinate: x: North, y: East, z: Down)
        # Using a square pattern of 5x5m at altitude 5m (z=-5)
        self.waypoints = [
            Point(x=5.0, y=0.0, z=-5.0),
            Point(x=5.0, y=5.0, z=-5.0),
            Point(x=0.0, y=5.0, z=-5.0),
            Point(x=0.0, y=0.0, z=-5.0),  # RTL waypoint
        ]
        self.current_wp_idx = 0
        self.mission_completed = False
        self.reached_threshold = 0.5  # 0.5m radius to consider reached

        self.current_pos = None
        self.current_setpoint = None
        self.max_speed = 0.5  # m/s (느리고 부드러운 이동 속도)
        self.dt = 0.1         # timer period

        self.timer = self.create_timer(self.dt, self.timer_callback) # 10Hz

        self.get_logger().info('MissionManager started. Waiting for odometry...')

    def odom_callback(self, msg: VehicleOdometry):
        self.current_pos = msg.position  # [x, y, z] in NED

        if self.mission_completed:
            return

        # Check distance to current waypoint
        target = self.waypoints[self.current_wp_idx]
        dist = math.sqrt(
            (self.current_pos[0] - target.x)**2 +
            (self.current_pos[1] - target.y)**2 +
            (self.current_pos[2] - target.z)**2
        )

        if dist < self.reached_threshold:
            self.get_logger().info(f'Reached Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}')
            self.current_wp_idx += 1

            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('Mission Completed! Sending RTL/Land command.')
                self.mission_completed = True
                
                msg_comp = Bool()
                msg_comp.data = True
                self.completed_pub.publish(msg_comp)

    def timer_callback(self):
        if self.current_pos is None:
            return

        # Do not advance the mission setpoint on the ground while the
        # offboard controller is still performing its takeoff phase.
        # All mission waypoints use z=-5 m in PX4 NED coordinates.
        if not self.mission_completed and self.current_pos[2] > -4.5:
            return
            
        if self.mission_completed:
            msg_comp = Bool()
            msg_comp.data = True
            self.completed_pub.publish(msg_comp)
        else:
            target = self.waypoints[self.current_wp_idx]
            
            # 처음 시작할 때 현재 위치를 기준점으로 삼음
            if self.current_setpoint is None:
                self.current_setpoint = Point(x=float(self.current_pos[0]), 
                                              y=float(self.current_pos[1]), 
                                              z=float(self.current_pos[2]))
            
            # 목표 지점을 향해 천천히 setpoint 이동
            dx = target.x - self.current_setpoint.x
            dy = target.y - self.current_setpoint.y
            dz = target.z - self.current_setpoint.z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            
            step_size = self.max_speed * self.dt
            if dist > step_size:
                self.current_setpoint.x += (dx / dist) * step_size
                self.current_setpoint.y += (dy / dist) * step_size
                self.current_setpoint.z += (dz / dist) * step_size
            else:
                self.current_setpoint.x = target.x
                self.current_setpoint.y = target.y
                self.current_setpoint.z = target.z
                
            self.target_pub.publish(self.current_setpoint)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
