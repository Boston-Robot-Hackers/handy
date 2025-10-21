# Handy

ROS2 utility package for network diagnostics and monitoring.

## Tools

### net_latency

Network latency measurement tool for ROS2 communication across distributed systems.

#### Purpose

Measures the time delays introduced when two ROS nodes communicate across a complex network path (e.g., ros2 → linux → vmware → macos → wifi → raspberry pi → ros2).

#### Usage

Run the same program on both computers with reversed node names:

**Computer 1:**
```bash
ros2 run handy net_latency node1 node2
```

**Computer 2:**
```bash
ros2 run handy net_latency node2 node1
```

#### How It Works

- Each instance subscribes to `<node1>-topic` and publishes to `<node2>-topic`
- Messages contain a counter and timestamp
- Published at 1 Hz
- On receiving a message, calculates time difference and displays:
  ```
  Time skew from node2 to node1 is 123.45 ms
  ```
- Press Ctrl+C to stop

#### Message Format

Uses `std_msgs/Float64MultiArray`:
- `data[0]`: Counter (incremented with each publish)
- `data[1]`: Timestamp (seconds since epoch)

## Building

```bash
cd /path/to/ros2_ws
colcon build --packages-select handy
source install/setup.bash
```

## Requirements

- ROS2 (Humble or later)
- Python 3
- Dependencies: rclpy, std_msgs

## Network Setup

Ensure both computers:
- Are on the same network
- Have the same ROS_DOMAIN_ID
- Can discover each other's topics (`ros2 topic list`)

## License

Apache-2.0

## Maintainer

pitosalas@gmail.com
