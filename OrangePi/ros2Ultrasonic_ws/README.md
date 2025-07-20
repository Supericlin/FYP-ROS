# OrangePi Ultrasonic Sensor Node

## Overview

This is an optimized ultrasonic sensor node for OrangePi systems, featuring the same professional style and monitoring capabilities as the RDK-X5 version while maintaining low CPU usage.

## Features

- **Low CPU Usage**: Optimized for resource-constrained systems
- **Professional Monitoring**: Performance statistics and error tracking
- **Configurable Logging**: Adjustable status reports and debug logging
- **Stuck Sensor Detection**: Automatic detection and recovery from stuck sensors
- **Parameterized**: Launch-time configuration via ROS2 parameters

## Hardware Setup

### GPIO Pin Configuration

| Sensor | Trigger Pin | Echo Pin | Name |
|--------|-------------|----------|------|
| Sensor 4 | 24 | 25 | Sensor 4 |
| Sensor 5 | 10 | 13 | Sensor 5 |

### Wiring

Connect your ultrasonic sensors (HC-SR04 or similar) to the GPIO pins as follows:
- **VCC**: 5V power
- **GND**: Ground
- **TRIG**: Trigger pin (as specified above)
- **ECHO**: Echo pin (as specified above)

## Installation

```bash
# Clone the repository
cd OrangePi/ros2Ultrasonic_ws

# Build the package
colcon build --packages-select ultrasonic_sensors

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Launch

```bash
# Launch with default settings (30s status reports, no debug)
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py
```

### Advanced Configuration

```bash
# Disable status logging for maximum performance
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py log_interval:=0.0

# Enable debug mode for detailed logging
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py debug_mode:=true

# Enable detailed sensor readings
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py enable_detailed_logging:=true

# Custom status interval (e.g., every 60 seconds)
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py log_interval:=60.0
```

### Performance Testing

```bash
# Run performance test
python3 test_performance.py
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug_mode` | `false` | Enable debug logging for detailed sensor information |
| `log_interval` | `30.0` | Interval in seconds for status logging (0.0 to disable) |
| `enable_detailed_logging` | `false` | Enable detailed logging of individual sensor readings |

## Topics

### Published Topics

- `/ultrasonic_sensor_4` (sensor_msgs/Range): Distance readings from Sensor 4
- `/ultrasonic_sensor_5` (sensor_msgs/Range): Distance readings from Sensor 5

### Message Format

```yaml
header:
  stamp: <timestamp>
  frame_id: "ultrasonic_sensor_X"
radiation_type: 0  # ULTRASOUND
field_of_view: 0.26  # ~15 degrees
min_range: 0.02  # 2 cm
max_range: 4.0   # 4 m
range: <distance_in_meters>
```

## Performance Characteristics

- **Update Rate**: 10 Hz (100ms intervals)
- **CPU Usage**: ~2-3% (typical)
- **Memory Usage**: ~15-20 MB
- **Status Reports**: Every 30 seconds (configurable)

## Troubleshooting

### Common Issues

1. **Permission Denied**: Ensure your user has GPIO access
   ```bash
   sudo usermod -a -G gpio $USER
   ```

2. **No Sensor Response**: Check wiring and power supply
   ```bash
   # Test GPIO pins manually
   gpio readall
   ```

3. **Stuck Sensor Warnings**: The node automatically detects and resets stuck sensors

### Debug Mode

Enable debug mode to see detailed sensor information:
```bash
ros2 launch ultrasonic_sensors orangepi_ultrasonic_launch.py debug_mode:=true
```

## Comparison with RDK-X5 Version

| Feature | OrangePi | RDK-X5 |
|---------|----------|--------|
| **GPIO Library** | wiringpi | Hobot.GPIO |
| **CPU Usage** | ~2-3% | ~3-4% |
| **Memory Usage** | ~15-20 MB | ~20-25 MB |
| **Monitoring** | ✅ Yes | ✅ Yes |
| **Parameterization** | ✅ Yes | ✅ Yes |
| **Stuck Detection** | ✅ Yes | ✅ Yes |

## License

This project is part of the FYP-ROS system. 