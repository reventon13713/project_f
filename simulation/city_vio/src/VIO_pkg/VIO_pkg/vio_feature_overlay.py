#!/usr/bin/env python3
"""Render live camera features and optical-flow vectors for the VIO demo."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def select_valid_tracks(previous_points, next_points, status):
    """Return matched LK points as flat ``(N, 2)`` arrays.

    OpenCV represents points as ``(N, 1, 2)``.  Boolean indexing keeps the
    singleton middle dimension, so iterating the result directly yields
    ``(1, 2)`` rows that cannot be unpacked as ``x, y``.  Normalize the shape
    once at the callback boundary and also reject malformed LK output.
    """
    previous = np.asarray(previous_points, dtype=np.float32).reshape((-1, 2))
    following = np.asarray(next_points, dtype=np.float32).reshape((-1, 2))
    valid = np.asarray(status).reshape((-1,)).astype(bool)
    count = min(len(previous), len(following), len(valid))
    if count == 0:
        return previous[:0], following[:0]
    valid = valid[:count]
    return previous[:count][valid], following[:count][valid]


class VioFeatureOverlay(Node):
    def __init__(self):
        super().__init__('vio_feature_overlay')
        self.declare_parameter('input_topic', '/vio/camera/vins_image_raw')
        self.declare_parameter('output_topic', '/vio/feature_overlay')
        self.declare_parameter('max_features', 220)
        self.max_features = int(self.get_parameter('max_features').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data)
        self.previous_gray = None
        self.previous_points = None
        self.get_logger().info(
            f'Live feature overlay: {input_topic} -> {output_topic}')

    def _detect(self, gray, existing=None):
        corners = cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_features, qualityLevel=0.012,
            minDistance=7, blockSize=7)
        if corners is None:
            return existing
        if existing is None or len(existing) == 0:
            return corners
        return np.concatenate((existing, corners), axis=0)[:self.max_features]

    def image_callback(self, message):
        if message.encoding not in ('mono8', 'rgb8', 'bgr8') or message.height == 0:
            return
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = message.height * message.step
        channels = 1 if message.encoding == 'mono8' else 3
        if raw.size < expected or message.step < message.width * channels:
            return
        rows = raw[:expected].reshape((message.height, message.step))
        if message.encoding == 'mono8':
            gray = rows[:, :message.width].reshape(
                (message.height, message.width)).copy()
            image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            image = rows[:, :message.width * 3].reshape(
                (message.height, message.width, 3)).copy()
            gray = None
        if message.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if gray is None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        overlay = image.copy()

        if self.previous_gray is None or self.previous_points is None:
            points = self._detect(gray)
        else:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.previous_gray, gray, self.previous_points, None,
                winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                          20, 0.03))
            if next_points is None or status is None:
                points = self._detect(gray)
            else:
                old, new = select_valid_tracks(
                    self.previous_points, next_points, status)
                for old_point, new_point in zip(old, new):
                    x0, y0 = np.round(old_point).astype(int)
                    x1, y1 = np.round(new_point).astype(int)
                    cv2.line(overlay, (x0, y0), (x1, y1), (0, 220, 0), 1,
                             cv2.LINE_AA)
                    cv2.circle(overlay, (x1, y1), 2, (0, 0, 255), -1,
                               cv2.LINE_AA)
                points = new.reshape((-1, 1, 2))
                if len(points) < self.max_features // 2:
                    points = self._detect(gray, points)

        if points is not None and self.previous_gray is None:
            for point in points.reshape((-1, 2)):
                x, y = np.round(point).astype(int)
                cv2.circle(overlay, (x, y), 2, (0, 0, 255), -1,
                           cv2.LINE_AA)

        output = Image()
        output.header = message.header
        output.height, output.width = overlay.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = overlay.tobytes()
        self.publisher.publish(output)
        self.previous_gray = gray
        self.previous_points = points


def main(args=None):
    rclpy.init(args=args)
    node = VioFeatureOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
