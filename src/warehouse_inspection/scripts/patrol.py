#!/usr/bin/env python3
"""Single-aisle inspection mission for the simulated warehouse."""

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pyzbar.pyzbar import ZBarSymbol, decode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool


STOPS = [
    (2.0, "STATION-A"),
    (0.0, "STATION-B"),
    (-2.0, "STATION-C"),
    (-4.0, "DOCK"),
]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def limit(value, maximum):
    return max(-maximum, min(maximum, value))


class Patrol(Node):
    def __init__(self):
        super().__init__("qr_inspection")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.battery = self.create_publisher(
            BatteryState, "/inspection/battery_state", 1
        )

        subscriptions = [
            (Odometry, "/odom", self.odom_cb),
            (LaserScan, "/scan", self.scan_cb),
            (Image, "/camera/image_raw", self.image_cb),
            (Bool, "/inspection_dock/touched", self.touch_cb),
        ]
        for message_type, topic, callback in subscriptions:
            self.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data
            )

        self.step = 0
        self.state = "READY"
        self.started = self.now()
        self.finished = False

        self.pose = None
        self.motion = (1.0, 1.0)
        self.scan = None
        self.last_odom = -math.inf
        self.last_scan = -math.inf

        self.last_img_stamp = None
        self.seen = 0
        self.marker_time = -math.inf
        self.pixel_error = 0.0
        self.still_since = None

        self.block_since = None
        self.last_block_report = -math.inf

        folder = Path.home() / "warehouse_missions"
        folder.mkdir(exist_ok=True)
        filename = datetime.now().strftime("patrol-%Y%m%d-%H%M%S-%f.csv")
        self.logpath = folder / filename
        self.file = self.logpath.open("x", newline="")
        self.csv = csv.writer(self.file)
        self.csv.writerow(
            ["sim_seconds", "event", "qr_or_detail", "odom_x_m", "odom_y_m"]
        )
        self.get_logger().info(f"Report: {self.logpath}")

        self.wall_start = time.monotonic()
        self.create_timer(0.08, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, event, detail=""):
        x, y, _ = self.pose or (0.0, 0.0, 0.0)
        self.csv.writerow(
            [round(self.now(), 2), event, detail, round(x, 3), round(y, 3)]
        )
        self.file.flush()
        self.get_logger().info(f"{event} {detail}")

    def set_state(self, state):
        self.state = state
        self.started = self.now()
        self.block_since = None
        self.seen = 0
        self.record(state, STOPS[self.step][1])

    def send(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def fail(self, detail):
        self.send()
        self.record("FAILED", detail)
        self.finished = True

    def odom_cb(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (pose.position.x, pose.position.y, yaw)
        self.motion = (
            abs(msg.twist.twist.linear.x),
            abs(msg.twist.twist.angular.z),
        )
        self.last_odom = self.now()

    def scan_cb(self, msg):
        if msg.header.frame_id == "lidar_link" and len(msg.ranges) >= 180:
            self.scan = msg
            self.last_scan = self.now()

    def image_cb(self, msg):
        if self.state not in ("READ", "APPROACH"):
            return

        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        image_time = stamp[0] + stamp[1] * 1e-9
        if stamp == self.last_img_stamp or self.now() - image_time > 0.8:
            return
        self.last_img_stamp = stamp

        channels = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }.get(msg.encoding.lower())

        if not channels or msg.step < msg.width * channels:
            return
        if len(msg.data) != msg.height * msg.step:
            return

        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step
        )
        gray = np.ascontiguousarray(pixels[:, :msg.width * channels:channels])
        expected = STOPS[self.step][1].encode()

        for qr in decode(gray, symbols=[ZBarSymbol.QRCODE]):
            if qr.data == expected:
                self.seen += 1
                self.marker_time = self.now()
                image_center = msg.width / 2
                qr_center = qr.rect.left + qr.rect.width / 2
                self.pixel_error = (qr_center - image_center) / image_center
                return

        self.seen = 0

    def touch_cb(self, msg):
        if self.state == "APPROACH" and msg.data:
            self.send()
            self.set_state("VERIFY_STOP")

    def obstacle_detail(self, turning):
        """Return the first blocking beam and reason, or None if clear."""
        scan = self.scan

        for i, distance in enumerate(scan.ranges):
            if distance == math.inf:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            degrees = math.degrees(angle)

            if not math.isfinite(distance) or distance < scan.range_min:
                return (
                    f"invalid_range beam={i} angle={degrees:.1f}deg "
                    f"range={distance:.3f}m minimum={scan.range_min:.3f}m"
                )

            if distance > scan.range_max:
                continue

            # Same LiDAR-to-base offset as in your original script.
            forward = distance * math.cos(angle) - 0.10
            left = distance * math.sin(angle)
            clearance = math.hypot(forward, left)

            if clearance < 0.62:
                reason = "inside_clearance_radius"
            elif not turning and 0 < forward < 1.03 and abs(left) < 0.50:
                reason = "forward_path_blocked"
            else:
                continue

            return (
                f"{reason} beam={i} angle={degrees:.1f}deg "
                f"range={distance:.3f}m forward={forward:.3f}m "
                f"left={left:.3f}m clearance={clearance:.3f}m"
            )

        return None

    def drive(self, v, w):
        turning = v == 0.0
        obstacle = self.obstacle_detail(turning)
        now = self.now()

        if obstacle is None:
            if self.block_since is not None:
                self.record(
                    "PATH_CLEAR",
                    f"{STOPS[self.step][1]} after {now - self.block_since:.1f}s",
                )
            self.block_since = None
            self.send(v, w)
            return

        self.send()
        motion = "turning" if turning else "forward"
        detail = f"{STOPS[self.step][1]} state={self.state} motion={motion} {obstacle}"

        if self.block_since is None:
            self.block_since = now
            self.last_block_report = now
            self.record("WAIT_OBSTACLE", detail)
        elif now - self.block_since > 12.0:
            self.fail(f"Route blocked for 12 simulated seconds. {detail}")
        elif now - self.last_block_report >= 2.0:
            self.last_block_report = now
            self.record("STILL_BLOCKED", detail)

    def tick(self):
        if self.finished:
            return

        t = self.now()
        if (
            self.pose is None
            or self.scan is None
            or t - self.last_odom > 0.8
            or t - self.last_scan > 0.8
        ):
            self.send()
            if self.state != "READY" and t - self.started > 90:
                self.fail("Sensor timeout")
            return

        x, y, yaw = self.pose
        target, code = STOPS[self.step]

        if self.state == "READY":
            if math.hypot(x, y) > 0.3 or abs(yaw) > 0.25:
                self.fail("Start from a fresh launch at the warehouse origin")
            elif self.count_publishers("/cmd_vel") > 1:
                self.fail("Stop all other /cmd_vel publishers before starting")
            else:
                self.set_state("TRAVEL")

        elif self.state == "TRAVEL":
            dx, dy = target - x, -y
            distance = math.hypot(dx, dy)

            if distance < 0.045:
                self.send()
                self.set_state("FACE")
            elif t - self.started > 100:
                self.fail("Travel timeout")
            else:
                error = wrap(math.atan2(dy, dx) - yaw)
                v = min(0.38, 0.9 * distance) if abs(error) < 0.28 else 0.0
                self.drive(v, limit(1.6 * error, 0.65))

        elif self.state == "FACE":
            target_yaw = math.pi if code == "DOCK" else math.pi / 2
            error = wrap(target_yaw - yaw)

            if abs(error) < 0.035:
                self.send()
                self.set_state("READ")
            elif t - self.started > 30:
                self.fail("Alignment timeout")
            else:
                self.drive(0.0, limit(1.8 * error, 0.6))

        elif self.state == "READ":
            self.send()
            if self.seen >= 2:
                self.record("QR_VERIFIED", code)
                if code == "DOCK":
                    self.set_state("APPROACH")
                else:
                    self.step += 1
                    self.set_state("TRAVEL")
            elif t - self.started > 12:
                self.fail(f"Camera did not decode {code} twice")

        elif self.state == "APPROACH":
            if t - self.started > 28 or x < -4.68 or abs(y) > 0.14:
                self.fail("Dock approach limit reached without contact")
            elif t - self.marker_time > 0.65:
                self.send()
                if t - self.marker_time > 3:
                    self.fail("Lost dock QR")
            else:
                v = 0.055 if abs(self.pixel_error) < 0.28 else 0.0
                self.drive(v, limit(-0.8 * self.pixel_error, 0.22))

        elif self.state == "VERIFY_STOP":
            self.send()
            if max(self.motion) < 0.025:
                if self.still_since is None:
                    self.still_since = t
                if t - self.still_since >= 0.4:
                    battery = BatteryState()
                    battery.header.stamp = self.get_clock().now().to_msg()
                    battery.present = True
                    battery.power_supply_status = (
                        BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    )
                    self.battery.publish(battery)
                    self.record(
                        "SIMULATED_CHARGING", "Dock contact and standstill confirmed"
                    )
                    self.finished = True
            else:
                self.still_since = None

            if not self.finished and t - self.started > 5:
                self.fail("Robot did not settle at dock")


def main():
    rclpy.init()
    node = Patrol()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.15)
            if node.state == "READY" and time.monotonic() - node.wall_start > 60:
                node.fail("No live odometry/scan/clock. Check the inspection launch.")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send()
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
