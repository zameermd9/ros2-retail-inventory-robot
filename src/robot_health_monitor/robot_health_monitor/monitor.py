#!/usr/bin/env python3
"""Monitor robot streams and test missing-data detection."""

import argparse
import csv
import fcntl
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as TimerClock
from rclpy.clock_type import ClockType
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Prefer these names; otherwise find a unique topic of the same type.
STREAMS = {
    "LiDAR": (LaserScan, "sensor_msgs/msg/LaserScan", "/scan"),
    "Camera": (Image, "sensor_msgs/msg/Image", "/camera/image_raw"),
    "Odometry": (Odometry, "nav_msgs/msg/Odometry", "/odom"),
    "Clock": (Clock, "rosgraph_msgs/msg/Clock", "/clock"),
}

TIMEOUT = 3.0  # Real seconds without data.


class HealthMonitor(Node):
    def __init__(self, testing):
        super().__init__("robot_health_monitor")

        self.testing = testing
        self.done = False
        self.failed = False
        self.start = time.monotonic()

        self.topics = {}
        self.subs = {}
        self.last_seen = {}
        self.notes = {}
        self.previous = {}
        self.clock_value = None
        self.next_print = 0.0

        # Test state.
        self.hidden = None
        self.targets = list(STREAMS)
        self.index = 0
        self.phase = "BASELINE"
        self.phase_start = self.start
        self.healthy_since = None
        self.detection = None
        self.other_fault = False

        # Each run gets its own report folder.
        run = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        self.folder = (
            Path.home() / "robot_diagnostics_ws" / "reports" / run
        )
        self.folder.mkdir(parents=True, exist_ok=True)

        self.event_file = (self.folder / "events.csv").open("w", newline="")
        self.event_csv = csv.writer(self.event_file)
        self.event_csv.writerow([
            "elapsed_seconds", "component", "status", "detail", "mode"
        ])

        self.test_file = (self.folder / "tests.csv").open("w", newline="")
        self.test_csv = csv.writer(self.test_file)
        self.test_csv.writerow([
            "component", "result", "detection_seconds",
            "recovery_seconds", "detail"
        ])

        self.pub = self.create_publisher(
            DiagnosticArray, "/robot_health/diagnostics", 10
        )

        # Keep checking even if Gazebo's simulation clock stops.
        self.steady_clock = TimerClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(2.0, self.discover, clock=self.steady_clock)
        self.create_timer(0.25, self.tick, clock=self.steady_clock)

        print("\n=== ROBOT HEALTH MONITOR ===", flush=True)
        print("Mode:", "CONTROLLED TEST" if testing else "LIVE")
        print("Reports:", self.folder)
        print("Press Ctrl+C to stop.\n", flush=True)
        self.discover()

    def discover(self):
        """Look for topics with actual publishers."""
        available = self.get_topic_names_and_types()

        for name, (message_type, type_name, preferred) in STREAMS.items():
            if name in self.subs:
                continue

            candidates = sorted(
                topic for topic, types in available
                if type_name in types and self.count_publishers(topic) > 0
            )

            if preferred in candidates:
                topic = preferred
            elif len(candidates) == 1:
                topic = candidates[0]
            else:
                self.notes[name] = (
                    "No publisher found"
                    if not candidates else
                    "Ambiguous topics: " + ", ".join(candidates)
                )
                continue

            callback = lambda msg, key=name: self.receive(key, msg)
            self.subs[name] = self.create_subscription(
                message_type, topic, callback, qos_profile_sensor_data
            )
            self.topics[name] = topic
            print(f"Connected: {name} -> {topic}", flush=True)

    def receive(self, name, message):
        """Record incoming data, unless locally hidden during a test."""
        if name == self.hidden:
            return

        now = time.monotonic()

        if name == "Clock":
            value = message.clock.sec + message.clock.nanosec / 1e9
            if self.clock_value is None or value != self.clock_value:
                self.last_seen[name] = now
            self.clock_value = value
        else:
            self.last_seen[name] = now

    def status(self, name, now):
        if name not in self.topics:
            return 2, self.notes.get(name, "Searching for publisher")

        if name not in self.last_seen:
            return 2, "Topic found, but no data received"

        if now - self.last_seen[name] > TIMEOUT:
            if name == "Clock":
                return 2, "Clock missing or simulation time not changing"
            return 2, "No messages for more than 3 seconds"

        return 0, "Clock changing" if name == "Clock" else "Messages arriving"

    def tick(self):
        now = time.monotonic()
        states = {name: self.status(name, now) for name in STREAMS}
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        for name, (level, detail) in states.items():
            item = DiagnosticStatus()
            item.name = "Robot health: " + name
            item.hardware_id = "gazebo_robot"
            item.level = bytes([level])
            item.message = detail
            item.values = [
                KeyValue(key="topic", value=self.topics.get(name, "Not found")),
                KeyValue(key="mode", value="test" if self.testing else "live"),
                KeyValue(key="locally_hidden", value=str(name == self.hidden)),
            ]
            array.status.append(item)

            if self.previous.get(name) != (level, detail):
                self.event_csv.writerow([
                    round(now - self.start, 3), name,
                    "OK" if level == 0 else "ERROR", detail,
                    "test" if self.testing else "live"
                ])
                self.event_file.flush()
                self.previous[name] = (level, detail)

        self.pub.publish(array)

        if now >= self.next_print:
            print(" | ".join(
                f"{name}: {'OK' if level == 0 else 'ERROR'}"
                for name, (level, _) in states.items()
            ), flush=True)

            for name, (level, detail) in states.items():
                if level:
                    print(f"  {name}: {detail}", flush=True)

            self.next_print = now + 2.0

        if self.testing:
            self.test_step(now, states)

    def result(self, name, outcome, detection="", recovery="", detail=""):
        self.test_csv.writerow([
            name, outcome, detection, recovery, detail
        ])
        self.test_file.flush()
        print(f"\n{outcome}: {name} — {detail}\n", flush=True)

    def test_step(self, now, states):
        all_ok = all(level == 0 for level, _ in states.values())

        if self.phase == "BASELINE":
            if all_ok:
                if self.healthy_since is None:
                    self.healthy_since = now
                if now - self.healthy_since >= 3.0:
                    self.hidden = self.targets[self.index]
                    self.phase = "FAULT"
                    self.phase_start = now
                    self.detection = None
                    self.other_fault = False
                    print(
                        f"\nTEST: hiding {self.hidden} from this monitor "
                        "for 6 seconds.", flush=True
                    )
            else:
                self.healthy_since = None

            if self.phase == "BASELINE" and now - self.phase_start > 60:
                self.result(
                    "Prerequisites", "NOT_RUN",
                    detail="All four streams must be healthy before testing."
                )
                self.failed = True
                self.done = True

        elif self.phase == "FAULT":
            target = self.targets[self.index]

            if any(level != 0 for name, (level, _) in states.items()
                   if name != target):
                self.other_fault = True

            if states[target][0] == 2 and self.detection is None:
                self.detection = round(now - self.phase_start, 3)
                print(f"Detected missing {target} data.", flush=True)

            if now - self.phase_start >= 6:
                self.hidden = None
                self.phase = "RECOVERY"
                self.phase_start = now
                self.healthy_since = None
                print(f"Accepting {target} data again.", flush=True)

        elif self.phase == "RECOVERY":
            if all_ok:
                if self.healthy_since is None:
                    self.healthy_since = now
                if now - self.healthy_since >= 3:
                    self.finish_test(recovered=True)
            else:
                self.healthy_since = None

            if self.phase == "RECOVERY" and now - self.phase_start > 15:
                self.finish_test(recovered=False)

    def finish_test(self, recovered):
        target = self.targets[self.index]
        passed = (
            self.detection is not None
            and recovered
            and not self.other_fault
        )

        recovery = (
            round(self.healthy_since - self.phase_start, 3)
            if recovered else ""
        )
        detail = (
            "Dropout detected and stream recovered."
            if passed else
            f"detected={self.detection is not None}; "
            f"recovered={recovered}; other_fault={self.other_fault}"
        )

        self.result(
            target, "PASS" if passed else "FAIL",
            self.detection if self.detection is not None else "",
            recovery, detail
        )
        self.failed = self.failed or not passed
        self.index += 1

        if self.index == len(self.targets):
            self.phase = "DONE"
            self.done = True
            print(
                "TESTS FINISHED: " +
                ("some tests failed." if self.failed else "all four passed."),
                flush=True
            )
            print("Results:", self.folder / "tests.csv", flush=True)
        else:
            self.phase = "BASELINE"
            self.phase_start = time.monotonic()
            self.healthy_since = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", action="store_true")
    args, ros_args = parser.parse_known_args()

    # Prevent two copies from running accidentally.
    lock_path = Path.home() / ".ros" / "robot_health_monitor.lock"
    lock_path.parent.mkdir(exist_ok=True)
    lock = lock_path.open("w")

    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Monitor already running. Stop it with Ctrl+C first.")
        lock.close()
        return 1

    rclpy.init(args=ros_args)
    node = HealthMonitor(args.test)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if args.test and not node.done:
            node.result("Test run", "ABORTED", detail="Stopped by user.")
            node.failed = True
    finally:
        node.hidden = None
        node.event_file.close()
        node.test_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()

    return 1 if node.failed else 0
