# Combined Launch Package

This package provides a single launch file to run both the `home_label_node` and `us100_sensor_node` together with proper logging configuration.

## Setup

1. Copy this `combined_launch` folder to your RDK-X5 workspace
2. Make sure all required packages are built in your workspace:
   - `home_label_package`
   - `us100_sensor_pkg`
   - `mqtt_navigation`
3. Build this package:
   ```bash
   cd combined_launch
   chmod +x build_combined.sh
   ./build_combined.sh
   ```
4. **Source the workspace** (required before using):
   ```bash
   source install/setup.bash
   ```
   
   **Optional**: Add to your `~/.bashrc` for permanent sourcing:
   ```bash
   echo 'source $(pwd)/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

## Usage

### Basic Usage
Launch both nodes with default settings:
```bash
ros2 launch combined_launch combined_nodes_launch.py
```

### Advanced Usage

#### Enable Debug Logging
```bash
ros2 launch combined_launch combined_nodes_launch.py debug_mode:=true
```

#### Custom Log Interval
Set status logging interval to 5 seconds:
```bash
ros2 launch combined_launch combined_nodes_launch.py log_interval:=5.0
```

#### Enable Detailed Sensor Logging
```bash
ros2 launch combined_launch combined_nodes_launch.py enable_detailed_logging:=true
```

#### Launch Only Specific Nodes
Launch only home labels (disable US100 sensor):
```bash
ros2 launch combined_launch combined_nodes_launch.py enable_us100_sensor:=false
```

Launch only US100 sensor (disable home labels):
```bash
ros2 launch combined_launch combined_nodes_launch.py enable_home_labels:=false
```

Launch only MQTT navigation (disable other nodes):
```bash
ros2 launch combined_launch combined_nodes_launch.py enable_home_labels:=false enable_us100_sensor:=false
```

#### MQTT Configuration
```bash
# Custom MQTT broker settings
ros2 launch combined_launch combined_nodes_launch.py \
  mqtt_broker_address:=192.168.1.100 \
  mqtt_broker_port:=1883 \
  mqtt_topic:=robot/commands \
  mqtt_status_topic:=robot/status
```

#### Combined Parameters
```bash
ros2 launch combined_launch combined_nodes_launch.py \
  debug_mode:=true \
  log_interval:=5.0 \
  enable_detailed_logging:=true
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `debug_mode` | `false` | Enable debug logging for detailed sensor information |
| `log_interval` | `10.0` | Interval in seconds for status logging (0.0 to disable) |
| `enable_detailed_logging` | `false` | Enable detailed logging of individual sensor readings |
| `enable_home_labels` | `true` | Enable home label publishing |
| `enable_us100_sensor` | `true` | Enable US100 sensor node |
| `enable_mqtt_navigation` | `true` | Enable MQTT navigation node |
| `mqtt_broker_address` | `192.168.0.224` | MQTT broker IP address |
| `mqtt_broker_port` | `1883` | MQTT broker port |
| `mqtt_topic` | `robot/control` | MQTT topic for receiving commands |
| `mqtt_status_topic` | `robot/status` | MQTT topic for publishing status |

## Expected Output

When launched successfully, you should see:

1. **Home Label Node**: Publishing room markers every 10 seconds
   ```
   [home_label_node] Published marker for Study_Room at (-0.725, 0.099)
   [home_label_node] Published marker for Washroom at (1.05, -0.187)
   [home_label_node] Published marker for Bedroom at (0.615, -2.81)
   [home_label_node] Published marker for Corridor at (0.922, 0.579)
   [home_label_node] Published marker for Dining_Room at (3.69, 1.87)
   [home_label_node] Published marker for Living_Room at (0.24, 1.78)
   ```

2. **US100 Sensor Node**: Status updates and sensor readings
   ```
   [us100_sensor_node] US100 sensor node started successfully
   [us100_sensor_node] Distance reading: 45.2 cm
   [us100_sensor_node] Status update: Sensor operational, readings stable
   ```

3. **MQTT Navigation Node**: MQTT connection and navigation status
   ```
   [mqtt_navigation_node] Connected to MQTT broker at 192.168.0.224:1883
   [mqtt_navigation_node] Service /controller_server/change_state is available
   [mqtt_navigation_node] Service /planner_server/change_state is available
   [mqtt_navigation_node] TF transform from map to base_link is available
   [mqtt_navigation_node] Published battery voltage: 12.3V to robot/status
   ```

## Topics

The launch will create these topics:
- `/marker` (visualization_msgs/Marker) - Room labels for RViz
- `/us100_distance` (std_msgs/Float32) - Distance readings from US100 sensor
- `/originbot_status` (originbot_msgs/OriginbotStatus) - Robot status including battery
- `/navigate_to_pose/_action/status` (action_msgs/GoalStatusArray) - Navigation status

## MQTT Topics

The MQTT navigation node will:
- **Subscribe to**: `robot/control` (or custom topic) for navigation commands
- **Publish to**: `robot/status` (or custom topic) for status updates

### Navigation Commands

Send these commands to the MQTT topic to control navigation:
- `study_room` or `studyroom` - Navigate to study room
- `washroom` - Navigate to washroom  
- `bed_room` or `bedroom` - Navigate to bedroom
- `dining_room` or `diningroom` - Navigate to dining room
- `living_room` or `livingroom` - Navigate to living room
- `pause` - Pause navigation
- `resume` - Resume navigation
- `cancel` - Cancel navigation

## Troubleshooting

1. **Package not found**: Make sure all required packages (`home_label_package`, `us100_sensor_pkg`, `mqtt_navigation`) are built
2. **Permission denied**: Run `chmod +x build_combined.sh` before building
3. **No output**: Check if all packages are properly installed and sourced
4. **Command not found**: Make sure you've sourced the workspace with `source install/setup.bash`
5. **Launch file not found**: Verify the package was built successfully and workspace is sourced
6. **MQTT connection failed**: Check MQTT broker address and port, ensure broker is running
7. **Navigation services not available**: Make sure Nav2 is running and services are available
8. **TF transform errors**: Ensure map server and localization are running 