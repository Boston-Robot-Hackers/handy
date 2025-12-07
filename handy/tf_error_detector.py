#!/usr/bin/env python3
"""
TF Error Detector - Monitor TF transforms and detect timing issues
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import statistics
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFTimingMonitor(Node):
    def __init__(self):
        super().__init__("tf_timing_monitor")

        # Subscribe to both dynamic and static transforms
        self.tf_sub = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)

        # /tf_static requires transient_local durability to receive historical messages
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self.tf_static_callback, static_qos
        )

        # Track timing data per transform pair
        self.timing_data = defaultdict(lambda: {
            "timestamps": [],
            "gaps": [],
            "is_static": False,
        })
        self.lookup_attempts = defaultdict(lambda: {"successes": 0, "failures": 0})
        self.error_report_count = defaultdict(int)
        self.error_details = defaultdict(list)
        self.max_reports = 3
        self.timestamp_warnings = defaultdict(list)
        self.timestamp_threshold = 0.010

        # Run duration configuration
        self.run_duration = 10.0  # Total seconds to run before exiting
        self.start_time = None  # Will be set after clock is available

        # Create a timer to periodically attempt lookups and detect issues
        self.timer = self.create_timer(0.1, self.attempt_lookups)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()

        print(
            f"TF Timing Monitor started - will run for {self.run_duration} seconds..."
        )

    def tf_callback(self, msg):
        self.analyze_transforms(msg, is_static=False)

    def tf_static_callback(self, msg):
        self.analyze_transforms(msg, is_static=True)

    def analyze_transforms(self, msg, is_static):
        current_ros_time = self.get_clock().now().nanoseconds / 1e9

        for transform in msg.transforms:
            frame_id = transform.header.frame_id
            child_frame_id = transform.child_frame_id
            pair = f"{frame_id}->{child_frame_id}"

            # Convert ROS time to seconds
            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec / 1e9

            # Check timestamp against ROS system time
            time_diff = abs(current_ros_time - timestamp)
            if time_diff > self.timestamp_threshold and len(self.timestamp_warnings[pair]) < 3:
                self.timestamp_warnings[pair].append({
                    "tf_timestamp": timestamp,
                    "ros_time": current_ros_time,
                    "diff": time_diff,
                    "ahead": current_ros_time < timestamp
                })

            data = self.timing_data[pair]
            data["is_static"] = is_static
            data["timestamps"].append(timestamp)

            # Keep only last 100 timestamps
            if len(data["timestamps"]) > 100:
                data["timestamps"].pop(0)

            # Calculate gaps between consecutive transforms
            if len(data["timestamps"]) > 1:
                gap = data["timestamps"][-1] - data["timestamps"][-2]
                data["gaps"].append(gap)
                if len(data["gaps"]) > 100:
                    data["gaps"].pop(0)

    def attempt_lookups(self):
        current_time = self.get_clock().now()

        for pair, data in self.timing_data.items():
            frame_id, child_frame_id = pair.split("->")

            try:
                # Attempt lookup at current time
                transform = self.tf_buffer.lookup_transform(
                    child_frame_id, frame_id, current_time
                )
                self.lookup_attempts[pair]["successes"] += 1

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.lookup_attempts[pair]["failures"] += 1

                # Check if this is the extrapolation error we're looking for
                if (
                    "extrapolation into the future" in str(e)
                    or "Lookup would require extrapolation" in str(e)
                ):
                    self.report_timing_issue(pair, str(e), current_time, data)

    def report_timing_issue(self, pair, error, current_time, data):
        """Store timing issue information for final report (no immediate printing)"""
        # Track how many times we've seen this error
        self.error_report_count[pair] += 1

        # Store up to 3 unique error details per pair
        if len(self.error_details[pair]) < 3:
            # Check if this error message already exists
            error_exists = any(e["error_message"] == error for e in self.error_details[pair])
            if not error_exists:
                frame_id, child_frame_id = pair.split("->")

                error_info = {
                    "error_message": error,
                    "frame_id": frame_id,
                    "child_frame_id": child_frame_id,
                    "lookup_time": current_time.nanoseconds / 1e9,
                }

                if data["timestamps"]:
                    error_info["latest_tf_time"] = data["timestamps"][-1]
                    error_info["time_diff"] = (current_time.nanoseconds / 1e9) - data["timestamps"][-1]

                    if error_info["time_diff"] < 0:
                        error_info["diagnosis"] = "future"
                        error_info["diagnosis_detail"] = (
                            f"Robot TF timestamp is {abs(error_info['time_diff'])*1000:.1f}ms IN THE FUTURE. "
                            f"Robot's clock is ahead of ROS system clock. "
                            f"TF lookup fails because requested time has not yet occurred on robot."
                        )
                    else:
                        error_info["diagnosis"] = "past"
                        error_info["diagnosis_detail"] = (
                            f"Robot TF timestamp is {error_info['time_diff']*1000:.1f}ms IN THE PAST. "
                            f"ROS system clock is ahead of robot's TF timestamp. "
                            f"Possible causes: clock desync, network delay, slow robot clock, or TF buffering."
                        )

                if data["gaps"]:
                    error_info["avg_gap"] = statistics.mean(data["gaps"])
                    error_info["publish_rate"] = 1 / error_info["avg_gap"]

                self.error_details[pair].append(error_info)

    def format_timestamp(self, unix_timestamp):
        dt = datetime.fromtimestamp(unix_timestamp)
        return dt.strftime("%H:%M:%S.%f")[:-3]

    def calc_success_rate(self, pair):
        attempts = self.lookup_attempts.get(pair, {"successes": 0, "failures": 0})
        total = attempts["successes"] + attempts["failures"]
        return (attempts["successes"] / total * 100) if total > 0 else 0

    def get_status_label(self, success_rate):
        if success_rate > 90:
            return "[OK]  "
        if success_rate > 50:
            return "[WARN]"
        return "[FAIL]"

    def print_transform_summary(self, pair, data):
        rate = 1 / statistics.mean(data["gaps"]) if data["gaps"] else 0
        success_rate = self.calc_success_rate(pair)
        status = self.get_status_label(success_rate)
        error_marker = " *ERROR*" if pair in self.error_details else ""
        print(
            f"{status} {pair:40s} | {rate:6.2f} Hz | Success: {success_rate:5.1f}%{error_marker}"
        )

    def print_timing_details(self, error_info):
        if "latest_tf_time" in error_info:
            print(
                f"    ROS: {self.format_timestamp(error_info['lookup_time'])} | "
                f"Robot: {self.format_timestamp(error_info['latest_tf_time'])} | "
                f"Diff: {error_info['time_diff']*1000:.1f}ms"
            )

    def print_error_report(self, pair, error_info):
        print(f"    {error_info['frame_id']} -> {error_info['child_frame_id']}")
        self.print_timing_details(error_info)
        if "diagnosis_detail" in error_info:
            print(f"    {error_info['diagnosis_detail']}")
        if "publish_rate" in error_info:
            rate_info = f"{error_info['publish_rate']:.1f} Hz"
            success = self.calc_success_rate(pair)
            print(f"    Rate: {rate_info} | Success: {success:.1f}%")

    def print_summary(self):
        if not self.timing_data:
            print("\nNo TF data collected.")
            return

        print("\n" + "=" * 70)
        print("TF MONITORING SUMMARY - ALL TRANSFORMS")
        print("=" * 70)

        for pair, data in sorted(self.timing_data.items()):
            if data["timestamps"]:
                self.print_transform_summary(pair, data)

        print("=" * 70)

        if self.timestamp_warnings:
            print("\n" + "=" * 70)
            print("TIMESTAMP WARNINGS - TF vs ROS SYSTEM TIME")
            print("=" * 70)
            for pair, warnings in sorted(self.timestamp_warnings.items()):
                print(f"\n{pair}: {len(warnings)} warnings")
                for w in warnings:
                    direction = "AHEAD" if w["ahead"] else "BEHIND"
                    print(
                        f"  TF: {self.format_timestamp(w['tf_timestamp'])} | "
                        f"ROS: {self.format_timestamp(w['ros_time'])} | "
                        f"{direction} by {w['diff']*1000:.1f}ms"
                    )
            print("=" * 70)

        if self.error_details:
            print("\n" + "=" * 70)
            print("ERROR REPORT - TRANSFORMS WITH TIMING ISSUES")
            print("=" * 70)
            for pair, error_list in sorted(self.error_details.items()):
                print(f"\n{pair}: {self.error_report_count.get(pair, 0)} errors, {len(error_list)} types")
                for idx, error_info in enumerate(error_list, 1):
                    if len(error_list) > 1:
                        print(f"  Type {idx}:")
                    self.print_error_report(pair, error_info)
            print(f"\nTotal transforms with errors: {len(self.error_details)}")
            print("=" * 70 + "\n")
        else:
            print("\nNo timing errors detected.")
            print("=" * 70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = TFTimingMonitor()

    try:
        # Run until duration expires
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if we've exceeded run duration
            current_time = node.get_clock().now()
            elapsed = (current_time.nanoseconds - node.start_time.nanoseconds) / 1e9
            if elapsed >= node.run_duration:
                print(f"\nRun duration ({node.run_duration}s) complete.")
                break
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Print final summary report
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
