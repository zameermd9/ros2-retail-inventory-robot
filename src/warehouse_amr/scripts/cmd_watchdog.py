#!/usr/bin/env python3
"""Forward Twist commands; stop after 0.5 wall-seconds without new input."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Watchdog(Node):
    def __init__(self):
        super().__init__('amr_cmd_watchdog')
        self.publisher = self.create_publisher(Twist, '/amr/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.command = Twist()
        self.last_command = 0.0
        self.timer = self.create_timer(0.05, self.tick)

    def receive(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.command = Twist()
            self.last_command = 0.0
            return
        self.command = Twist()
        self.command.linear.x = max(-0.3, min(0.5, msg.linear.x))
        self.command.angular.z = max(-1.0, min(1.0, msg.angular.z))
        self.last_command = time.monotonic()

    def tick(self):
        self.publisher.publish(self.command if time.monotonic() - self.last_command < 0.5 else Twist())


def main():
    rclpy.init()
    node = Watchdog()
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
