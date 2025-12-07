# Handy

ROS2 utility package for network diagnostics and monitoring.

## Tools

### ros2_params

ROS2 parameter search and discovery tool with parallel scanning and live progress tracking.

#### Purpose

Quickly scan all running ROS2 nodes and search for parameters by name or value. Useful for:
- Finding which nodes use a specific parameter
- Discovering parameters across a complex system
- Debugging parameter configuration issues

#### Usage

```bash
ros2 run handy ros2_params <search_string> [timeout_seconds] [max_threads]
```

**Examples:**
```bash
# Search for parameters containing "sim"
ros2 run handy ros2_params sim

# Search with 5 second timeout per parameter
ros2 run handy ros2_params frame_id 5.0

# Search with custom timeout and 16 threads
ros2 run handy ros2_params topic 3.0 16
```

#### Features

- **Parallel scanning**: Scans multiple nodes simultaneously (default: 8 threads)
- **Live progress**: Shows real-time status during scanning
- **Complete results**: Displays all parameters from all nodes
- **Filtered results**: Shows only parameters matching your search string
- **Timeout handling**: Configurable timeout to handle slow-responding nodes

#### Output

1. Live status line during scanning:
   ```
   Status: total nodes: 23, in progress: 5, completed: 18, matching query: 3
   ```

2. Complete parameter table after scanning

3. Filtered search results showing only matching parameters

### tf_error_detector

TF transform monitoring and exception detection tool that tracks lookup errors and timing issues.

#### Purpose

Monitors all TF transforms to detect and diagnose three types of lookup exceptions:
- **LookupException**: Transform not available (missing or disconnected frames)
- **ConnectivityException**: No path between frames in the TF tree
- **ExtrapolationException**: Requested time outside available transform data

Also tracks clock synchronization between TF timestamps and system time.

#### Usage

```bash
ros2 run handy tf_error_detector
```

Runs for 10 seconds (configurable via `RUN_DURATION_SEC` constant) and produces a summary report.

#### How It Works

- Subscribes to `/tf` and `/tf_static` topics to monitor all transform broadcasts
- Waits 1 second for static transforms to load before starting lookups
- Tracks timing data for each transform pair (frame_id → child_frame_id)
- Attempts transform lookups every 0.1 seconds using latest available transform
- Records exceptions by type and frame pair
- Detects clock sync issues when TF timestamps differ from system time by ≥25ms
- Prints comprehensive summary report at end

#### Output

The tool produces three sections in its final report:

**1. Transform Summary:**
```
================================================================================
TF MONITORING SUMMARY - ALL TRANSFORMS
================================================================================
[OK]   base_link->laser                         |  20.00 Hz | # exceptions on lookup: 0
[WARN] odom->base_footprint                     |  10.05 Hz | # exceptions on lookup: 5
[FAIL] map->odom                                 |   0.00 Hz | # exceptions on lookup: 23
================================================================================
```

**2. Clock Sync Info (if timestamp differences ≥25ms detected):**
```
================================================================================
CLOCK SYNC INFO
================================================================================
base_footprint->base_link: TF behind by 1226015.6ms
odom->base_footprint: TF behind by 33.2ms
================================================================================
```

**3. Error Report (if lookup exceptions occurred):**
```
================================================================================
ERROR REPORT - TRANSFORMS WITH TIMING ISSUES
================================================================================

map->odom: 23 errors, 1 types
    ExtrapolationException
    System: 14:32:15.123 | TF: 14:32:15.087 | Diff: 36.2ms
    Rate: 10.0 Hz | Success: 45.2%

Total transforms with errors: 1
================================================================================
```

If no exceptions occur, shows: "No timing errors detected."

#### Configuration Constants

Adjust these constants at the top of the file:
- `TIMESTAMP_THRESHOLD_SEC`: Clock sync reporting threshold (default: 25ms)
- `RUN_DURATION_SEC`: How long to monitor (default: 10 seconds)
- `TIMER_PERIOD_SEC`: Lookup attempt frequency (default: 0.1s)
- `STARTUP_DELAY_SEC`: Delay before lookups start (default: 1s)

#### Common Issues Detected

- **LookupException**: Missing transforms, frame not in TF tree
- **ConnectivityException**: Disconnected TF tree branches
- **ExtrapolationException**: TF buffer too small or publish rate too slow
- **Clock synchronization**: TF timestamps significantly different from system time

#### Use Cases

- Debugging TF tree connectivity issues
- Detecting timing problems before they cause navigation errors
- Monitoring multi-robot systems with distributed clocks
- Validating TF configuration in new robot setups

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
