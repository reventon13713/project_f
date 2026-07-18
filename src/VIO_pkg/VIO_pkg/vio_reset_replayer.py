#!/usr/bin/env python3
"""Re-issue recorded preflight VINS reset events during rosbag playback."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class VioResetReplayer(Node):
    def __init__(self):
        super().__init__('vio_reset_replayer')
        self.client = self.create_client(Trigger, '/reset')
        self.done = False
        self.create_subscription(String, '/vio/preflight_event', self.callback, 10)

    def callback(self, message):
        if self.done or message.data != 'VINS_RESET_REQUEST':
            return
        if not self.client.service_is_ready():
            self.get_logger().error('Recorded reset event arrived before /reset service.')
            return
        self.done = True
        self.client.call_async(Trigger.Request())
        self.get_logger().info('Replayed recorded VINS reset event.')


def main(args=None):
    rclpy.init(args=args)
    node = VioResetReplayer()
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
