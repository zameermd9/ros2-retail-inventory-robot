#!/usr/bin/env python3
"""Observe live sensors and TF for up to 30 seconds; never move the robot."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, LaserScan, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = Node('amr_runtime_verifier')
    data = {}
    counts = {}
    clock_values = set()
    subscriptions = []
    for topic, typ in [('/scan', LaserScan), ('/camera/image_raw', Image),
                       ('/camera/camera_info', CameraInfo), ('/odom', Odometry),
                       ('/joint_states', JointState), ('/clock', Clock)]:
        def receive(msg, name=topic):
            data[name] = msg
            counts[name] = counts.get(name, 0) + 1
            if name == '/clock':
                clock_values.add((msg.clock.sec, msg.clock.nanosec))
        subscriptions.append(node.create_subscription(typ, topic, receive, qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def results():
        scan = data.get('/scan')
        image = data.get('/camera/image_raw')
        info = data.get('/camera/camera_info')
        odom = data.get('/odom')
        joints = data.get('/joint_states')
        clock = data.get('/clock')
        checks = {
            'LiDAR: 360 ranges, finite obstacle return, lidar_link frame': bool(
                scan and len(scan.ranges) == 360 and scan.header.frame_id == 'lidar_link'
                and any(math.isfinite(v) and scan.range_min <= v <= scan.range_max for v in scan.ranges)
                and counts.get('/scan', 0) >= 2),
            'Camera: RGB pixels and camera_optical_frame': bool(
                image and image.width == 640 and image.height == 360
                and len(image.data) == image.step * image.height
                and image.header.frame_id == 'camera_optical_frame'
                and counts.get('/camera/image_raw', 0) >= 2),
            'Camera calibration: matching resolution and positive focal length': bool(
                info and info.width == 640 and info.height == 360 and info.k[0] > 0
                and info.header.frame_id == 'camera_optical_frame'),
            'Odometry: odom to base_footprint': bool(
                odom and odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint'),
            'Both wheel joints are published': bool(joints and
                {'left_wheel_joint','right_wheel_joint'}.issubset(set(joints.name))),
            'Simulation clock is advancing': bool(clock and len(clock_values) >= 2
                and (clock.clock.sec > 0 or clock.clock.nanosec > 0)),
        }
        for frame in ['base_link','lidar_link','camera_optical_frame','left_wheel_link','right_wheel_link']:
            try:
                buffer.lookup_transform('odom', frame, Time())
                checks['TF: odom -> '+frame] = True
            except TransformException:
                checks['TF: odom -> '+frame] = False
        return checks

    deadline = time.monotonic() + 30
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(results().values()):
                break
        checks = results()
        for name, ok in checks.items():
            print(('PASS' if ok else 'FAIL') + '  ' + name)
        print('Message counts:', counts)
        print('Visual check still required: robot, scan overlay and live camera in RViz.')
        status = 0 if all(checks.values()) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(status)


if __name__ == '__main__':
    main()
