#!/usr/bin/env python3
"""Deterministic math regression for vio_global_position."""

import math
import pathlib
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorGps, VehicleLocalPosition
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

repo = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(repo / 'src' / 'VIO_pkg'))

from VIO_pkg.vio_global_position import (  # noqa: E402
    VioGlobalPosition, fit_similarity_yaw, ned_to_geodetic)


def ros_policy_regression():
    """Lock once, publish, then prove a new generation stays latched closed."""
    rclpy.init()
    node = VioGlobalPosition(parameter_overrides=[
        Parameter('minimum_alignment_samples', value=12),
        Parameter('minimum_horizontal_span_m', value=0.5),
        Parameter('minimum_vertical_span_m', value=0.2),
    ])
    local = VehicleLocalPosition()
    local.xy_valid = local.z_valid = True
    local.xy_global = local.z_global = True
    local.ref_lat = 37.413534
    local.ref_lon = -121.996561
    local.ref_alt = 1.3
    gps = SensorGps()
    gps.fix_type = 3
    gps.eph = 0.4
    gps.epv = 0.7
    gps.alt = 1300
    gps.alt_ellipsoid = 31400
    node._local_callback(local)
    node._gps_callback(gps)

    start_ns = node.get_clock().now().nanoseconds - 1_000_000_000
    for index in range(20):
        odom = Odometry()
        stamp_ns = start_ns + index * 20_000_000
        odom.header.stamp.sec = stamp_ns // 1_000_000_000
        odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
        # ENU -> NED truth plus a constant NED translation.
        odom.pose.pose.position.x = index * 0.04
        odom.pose.pose.position.y = index * 0.06
        odom.pose.pose.position.z = index * 0.015
        local.x = odom.pose.pose.position.y + 2.0
        local.y = odom.pose.pose.position.x - 3.0
        local.z = -odom.pose.pose.position.z + 1.0
        node._local_callback(local)
        node._gps_callback(gps)
        node._vio_callback(odom)

    ready = Bool()
    ready.data = True
    node._ready_callback(ready)
    assert node.alignment is not None
    assert not node.fault_latched

    output = Odometry()
    stamp_ns = start_ns + 20 * 20_000_000
    output.header.stamp.sec = stamp_ns // 1_000_000_000
    output.header.stamp.nanosec = stamp_ns % 1_000_000_000
    output.pose.pose.position.x = 0.8
    output.pose.pose.position.y = 1.2
    output.pose.pose.position.z = 0.3
    node._vio_callback(output)

    node._ready_callback(ready)
    assert node.fault_latched
    assert node.alignment is None
    node.destroy_node()
    rclpy.shutdown()


def main():
    rng = np.random.default_rng(42)
    vio = rng.normal(size=(200, 3)) * np.array([8.0, 8.0, 2.0])
    scale = 1.037
    yaw = math.radians(31.0)
    translation = np.array([12.0, -7.0, 3.0])
    # Build truth independently from the fitter's implementation.
    raw_ned = np.column_stack((vio[:, 1], vio[:, 0], -vio[:, 2]))
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                         [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]])
    truth = scale * (raw_ned @ rotation.T) + translation
    fitted = fit_similarity_yaw(vio, truth)
    assert abs(fitted.scale - scale) < 1e-12
    assert abs(fitted.yaw_rad - yaw) < 1e-12
    assert np.linalg.norm(fitted.translation_ned - translation) < 1e-10
    assert fitted.max_error_m < 1e-10

    origin = (37.413534, -121.996561, 32.5)
    lat, lon, alt = ned_to_geodetic(np.zeros(3), origin)
    assert abs(lat - origin[0]) < 1e-10
    assert abs(lon - origin[1]) < 1e-10
    assert abs(alt - origin[2]) < 1e-6
    north = ned_to_geodetic(np.array([100.0, 0.0, 0.0]), origin)
    east = ned_to_geodetic(np.array([0.0, 100.0, 0.0]), origin)
    up = ned_to_geodetic(np.array([0.0, 0.0, -10.0]), origin)
    assert north[0] > origin[0] and abs(north[1] - origin[1]) < 1e-8
    assert east[1] > origin[1] and abs(east[0] - origin[0]) < 1e-8
    assert abs(up[2] - (origin[2] + 10.0)) < 1e-4
    ros_policy_regression()
    print('vio_global_position math regression: PASS')


if __name__ == '__main__':
    main()
