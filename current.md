## Initial spec

* A program to determine the delays introduced when two ROS nodes are communicating via ros2->linux->vmware->macos->wifi->raspberry pi->ros2
* There is one node which is run on both computers at the same time
* ros2 run handy net_latency node1 node2 (on one computer)
* ros2 run handy net_latency node2 node1 (on the other computer)
* It's a simple Python program in a ROS package
* The program subscribes to a topic called <node1>-topic and publishes to a topic called <node2>-topic
* Each topic message contains two numbers: an integer that is incremented each time a pub happens, and the current time on this computer
* When the subscriber sees a message, it computes the difference between the time in the payload and the local time and prints a message saying:
* Time skew from node-1 to node-2 is <xxx> ms
* It does this once per second
* The loop is interrupted by Ctrl+C

## Current Status

* Implementation complete in handy/net_latency.py
* Uses DelayMeasureNode class that extends rclpy.node.Node
* Messages use std_msgs/Float64MultiArray with [counter, timestamp] format
* Entry point configured in setup.py as "net_latency"
* Ready to build with: colcon build --packages-select handy
* Awaiting testing on two computers to verify network latency measurements