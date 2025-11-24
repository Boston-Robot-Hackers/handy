#!/usr/bin/env python3
"""
Monitor TF transforms and detect timing issues that would cause
"Lookup would require extrapolation into the future" errors.

This subscribes to /tf and /tf_static topics and analyzes timestamp gaps.
"""

import statistics

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFTimingMonitor(Node):
    def __init__(self):
        super().__init__("tf_timing_monitor")

        # Subscribe to both dynamic and static transforms
        self.tf_sub = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)

        self.tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self.tf_static_callback, 10
        )

        # Track timing data per transform pair
        self.timing_data = {}
        self.lookup_attempts = {}
        self.error_report_count = {}  # Track how many times each error was reported
        self.error_details = {}  # Store detailed error information for final report
        self.max_reports = 3  # Exit after reporting same error this many times

        # Run duration configuration
        self.run_duration = 30.0  # Total seconds to run before exiting
        self.start_time = None  # Will be set after clock is available

        # Create a timer to periodically attempt lookups and detect issues
        self.timer = self.create_timer(0.1, self.attempt_lookups)

        from tf2_ros import Buffer, TransformListener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()

        print(f"TF Timing Monitor started - will run for {self.run_duration} seconds...")

    def tf_callback(self, msg):
        """Process dynamic TF transforms"""
        self.analyze_transforms(msg, is_static=False)

    def tf_static_callback(self, msg):
        """Process static TF transforms"""
        self.analyze_transforms(msg, is_static=True)

    def analyze_transforms(self, msg, is_static):
        """Analyze timestamp gaps in received transforms"""
        for transform in msg.transforms:
            frame_id = transform.header.frame_id
            child_frame_id = transform.child_frame_id
            pair = f"{frame_id}->{child_frame_id}"

            # Convert ROS time to seconds
            stamp = transform.header.stamp
            timestamp = stamp.sec + stamp.nanosec / 1e9

            if pair not in self.timing_data:
                self.timing_data[pair] = {
                    "timestamps": [],
                    "gaps": [],
                    "is_static": is_static,
                }

            data = self.timing_data[pair]
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
        """Attempt to look up all known transforms at current time and detect failures"""
        current_time = self.get_clock().now()

        for pair, data in self.timing_data.items():
            frame_id, child_frame_id = pair.split("->")

            if pair not in self.lookup_attempts:
                self.lookup_attempts[pair] = {"successes": 0, "failures": 0}

            try:
                # Attempt lookup at current time
                transform = self.tf_buffer.lookup_transform(
                    child_frame_id, frame_id, current_time
                )
                self.lookup_attempts[pair]["successes"] += 1

            except Exception as e:
                error_str = str(e)
                self.lookup_attempts[pair]["failures"] += 1

                # Check if this is the extrapolation error we're looking for
                if (
                    "extrapolation into the future" in error_str
                    or "Lookup would require extrapolation" in error_str
                ):
                    self.report_timing_issue(pair, error_str, current_time, data)

    def report_timing_issue(self, pair, error, current_time, data):
        """Store timing issue information for final report (no immediate printing)"""
        # Track how many times we've seen this error
        if pair not in self.error_report_count:
            self.error_report_count[pair] = 0

        self.error_report_count[pair] += 1

        # Store detailed error information (only store the first occurrence details)
        if pair not in self.error_details:
            frame_id, child_frame_id = pair.split("->")

            error_info = {
                "error_message": error,
                "frame_id": frame_id,
                "child_frame_id": child_frame_id,
                "lookup_time": current_time.nanoseconds / 1e9,
            }

            if data["timestamps"]:
                latest_tf_time = data["timestamps"][-1]
                time_diff = (current_time.nanoseconds / 1e9) - latest_tf_time
                error_info["latest_tf_time"] = latest_tf_time
                error_info["time_diff"] = time_diff

                if time_diff < 0:
                    error_info["diagnosis"] = "future"
                    error_info["diagnosis_detail"] = (
                        f"Robot TF timestamp is {abs(time_diff):.6f}s IN THE FUTURE. "
                        f"Robot's clock is ahead of this computer's clock. "
                        f"TF lookup fails because requested time has not yet occurred on robot."
                    )
                else:
                    error_info["diagnosis"] = "past"
                    error_info["diagnosis_detail"] = (
                        f"Robot TF timestamp is {time_diff:.6f}s IN THE PAST. "
                        f"This computer's clock is ahead of robot's TF timestamp. "
                        f"Possible causes: clock desync, network delay, slow robot clock, or TF buffering."
                    )

            if data["gaps"]:
                avg_gap = statistics.mean(data["gaps"])
                error_info["publish_rate"] = 1 / avg_gap
                error_info["avg_gap"] = avg_gap

            self.error_details[pair] = error_info

    def print_summary(self):
        """Print comprehensive final report of all monitored transforms and errors"""
        if not self.timing_data:
            print("\nNo TF data collected.")
            return

        print("\n" + "=" * 70)
        print("TF MONITORING SUMMARY - ALL TRANSFORMS")
        print("=" * 70)

        # Print table of all monitored transforms
        for pair, data in sorted(self.timing_data.items()):
            if data["timestamps"]:
                avg_gap = statistics.mean(data["gaps"]) if data["gaps"] else 0
                rate = 1 / avg_gap if avg_gap > 0 else 0
                attempts = self.lookup_attempts.get(
                    pair, {"successes": 0, "failures": 0}
                )
                success_rate = (
                    attempts["successes"]
                    / (attempts["successes"] + attempts["failures"])
                    * 100
                    if (attempts["successes"] + attempts["failures"]) > 0
                    else 0
                )

                status = (
                    "[OK]  " if success_rate > 90 else "[WARN]" if success_rate > 50 else "[FAIL]"
                )
                error_marker = " *ERROR*" if pair in self.error_details else ""
                print(
                    f"{status} {pair:40s} | {rate:6.2f} Hz | Success: {success_rate:5.1f}%{error_marker}"
                )

        print("=" * 70)

        # Print detailed error report for transforms with timing issues
        if self.error_details:
            print("\n" + "=" * 70)
            print("DETAILED ERROR REPORT - TRANSFORMS WITH TIMING ISSUES")
            print("=" * 70)

            for pair, error_info in sorted(self.error_details.items()):
                print(f"\n{'─' * 70}")
                print(f"Transform: {pair}")
                print(f"  Source frame: {error_info['frame_id']}")
                print(f"  Target frame: {error_info['child_frame_id']}")
                print(f"  Error count: {self.error_report_count.get(pair, 0)} occurrences")

                print(f"\nError message:")
                print(f"  {error_info['error_message']}")

                print(f"\nTiming details:")
                print(f"  Lookup attempted at: {error_info['lookup_time']:.9f} (local system time)")
                if "latest_tf_time" in error_info:
                    print(f"  Latest TF timestamp: {error_info['latest_tf_time']:.9f} (robot)")
                    print(f"  Time difference: {error_info['time_diff']:.9f} seconds (local - robot)")
                    print(f"  Absolute difference: {abs(error_info['time_diff']):.6f}s")

                print(f"\nDiagnosis:")
                if "diagnosis_detail" in error_info:
                    print(f"  {error_info['diagnosis_detail']}")

                if "publish_rate" in error_info:
                    print(f"\nTF publish rate: {error_info['publish_rate']:.2f} Hz (gap: {error_info['avg_gap']:.6f}s)")

                # Show success rate for this transform
                attempts = self.lookup_attempts.get(pair, {"successes": 0, "failures": 0})
                if attempts["successes"] + attempts["failures"] > 0:
                    success_rate = (
                        attempts["successes"]
                        / (attempts["successes"] + attempts["failures"])
                        * 100
                    )
                    print(
                        f"Transform lookup success rate: {success_rate:.1f}% "
                        f"({attempts['successes']}/{attempts['successes'] + attempts['failures']})"
                    )

            print(f"\n{'─' * 70}")
            print(f"\nTotal transforms with errors: {len(self.error_details)}")
            print("=" * 70 + "\n")
        else:
            print("\nNo timing errors detected during monitoring period.")
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
