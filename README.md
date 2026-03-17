# Mini Autoware Sensor Integration Pipeline

A minimal autonomous-driving-style pipeline that simulates **sensor → ROS topic → perception → control**, similar to [Autoware](https://autoware.org/). All data flows over ROS2 topics.

## Pipeline Overview

```
[Simulated LiDAR] → /raw_lidar → [Driver] → /points_raw → [Perception] → /detected_obstacles → [Control] → /vehicle_cmd
```

| Module            | Node           | Subscribes      | Publishes           |
|-------------------|----------------|-----------------|---------------------|
| Simulated Sensor  | `sensor_node`  | —               | `/raw_lidar`        |
| Driver / Adapter  | `driver_node`  | `/raw_lidar`    | `/points_raw`       |
| Consumer (Perception) | `consumer_node` | `/points_raw` | `/detected_obstacles` |
| Control           | `control_node` | `/detected_obstacles` | `/vehicle_cmd` |

## Requirements

- **Python 3.x**
- **ROS2** (Foxy or Humble), on WSL2 or native Linux
- **Optional:** `matplotlib` for 2D visualization (saves plot to `/tmp/mini_autoware_viz.png`)

## Workspace Layout

```
mini_autoware_sensor_integration_pipeline/
├── README.md
├── requirements.txt
└── src/
    └── mini_autoware_sensor_pipeline/
        ├── CMakeLists.txt
        ├── package.xml
        ├── msg/
        │   ├── RawLidar.msg
        │   ├── Obstacle.msg
        │   ├── DetectedObstacles.msg
        │   └── VehicleCmd.msg
        ├── nodes/
        │   ├── sensor_node.py    # Simulated LiDAR → /raw_lidar
        │   ├── driver_node.py    # Adapter → /points_raw (PointCloud2)
        │   ├── consumer_node.py  # Perception + optional plot
        │   └── control_node.py   # Commands → /vehicle_cmd
        └── launch/
            └── pipeline.launch.py
```

## Build

From the **project root** (one level above `src/`):

```bash
cd /path/to/mini_autoware_sensor_integration_pipeline
source /opt/ros/humble/setup.bash   # or foxy
colcon build --packages-select mini_autoware_sensor_pipeline
source install/setup.bash
```

## Run

**Option A – Launch all nodes:**

```bash
source install/setup.bash
ros2 launch mini_autoware_sensor_pipeline pipeline.launch.py
```

**Option B – Run nodes in separate terminals (good for watching pub/sub):**

```bash
# Terminal 1
ros2 run mini_autoware_sensor_pipeline sensor_node.py

# Terminal 2
ros2 run mini_autoware_sensor_pipeline driver_node.py

# Terminal 3
ros2 run mini_autoware_sensor_pipeline consumer_node.py

# Terminal 4
ros2 run mini_autoware_sensor_pipeline control_node.py
```

**Inspect topics:**

```bash
ros2 topic list
ros2 topic echo /raw_lidar
ros2 topic echo /detected_obstacles
ros2 topic echo /vehicle_cmd
```

## Visualization

- If **matplotlib** is installed, the consumer node periodically writes a 2D plot to **`/tmp/mini_autoware_viz.png`** (LiDAR points + detected obstacles).
- View with any image viewer or refresh the file in your file manager.

## Parameters (optional)

- **sensor_node:** `rate_hz` (default 10), `num_points`, `range_min`, `range_max`, `fov_deg`
- **consumer_node:** `cluster_dist` (clustering distance), `visualize` (true/false), `viz_interval` (seconds between plots)
- **control_node:** `max_velocity`, `safe_dist`, `min_velocity`

Example:

```bash
ros2 run mini_autoware_sensor_pipeline sensor_node.py --ros-args -p rate_hz:=5.0
```

## Optional Enhancements

- Simulate **moving obstacles** (e.g., time-varying point clusters).
- Add **timestamp synchronization** to messages (headers are already set).
- Add **multiple subscribers** to the same topic (e.g., a logging node + perception).

## License

Apache-2.0
