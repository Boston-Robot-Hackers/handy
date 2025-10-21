#!/usr/bin/env python3
# net_latency: Network latency measurement tool for ROS2 communication
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class DelayMeasureNode(Node):
    def __init__(self, self_name, peer_name):
        super().__init__(f"{self_name}_delay_node")
        self.self_name = self_name
        self.peer_name = peer_name
        self.counter = 0

        self.publisher = self.create_publisher(
            Float64MultiArray,
            f"{peer_name}-topic",
            10
        )

        self.subscriber = self.create_subscription(
            Float64MultiArray,
            f"{self_name}-topic",
            self.message_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(
            f"Node started: subscribing to {self_name}-topic, "
            f"publishing to {peer_name}-topic"
        )

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [float(self.counter), time.time()]
        self.publisher.publish(msg)
        self.counter += 1

    def message_callback(self, msg):
        received_time = time.time()
        sent_time = msg.data[1]
        delay_ms = (received_time - sent_time) * 1000
        print(
            f"Time skew from {self.peer_name} to {self.self_name} "
            f"is {delay_ms:.2f} ms"
        )


def main(args=None):
    if len(sys.argv) != 3:
        print("Usage: ros2 run handy net_latency <node1> <node2>")
        print("Example: ros2 run handy net_latency node1 node2")
        sys.exit(1)

    self_name = sys.argv[1]
    peer_name = sys.argv[2]

    rclpy.init(args=args)
    node = DelayMeasureNode(self_name, peer_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
