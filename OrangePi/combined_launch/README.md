# OrangePi Combined Launch Package

This package provides a single launch file to run both the fall detection system (HF867 camera) and ultrasonic sensors together on the OrangePi.

## Setup

1. Make sure both workspaces are built:
   - `FallDetect2_ws` (contains `fall_detection_pkg`)
   - `ros2Ultrasonic_ws` (contains `ultrasonic_sensors`)
2. Copy this `combined_launch` folder to your OrangePi root directory
3. Build this package:
   ```bash
   cd combined_launch
   chmod +x build_orangepi.sh
   ./build_orangepi.sh
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
Launch both fall detection and ultrasonic sensors with default settings:
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py
```

### Advanced Usage

#### Enable Debug Logging
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py debug_mode:=true
```

#### Launch Only Specific Systems
Launch only fall detection (disable ultrasonic):
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py enable_ultrasonic:=false
```

Launch only ultrasonic sensors (disable fall detection):
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py enable_fall_detection:=false
```

#### Camera Configuration
Custom camera settings for HF867:
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py \
  exposure_time:=100 \
  brightness:=40 \
  contrast:=60 \
  saturation:=70
```

#### Preview Control
Disable preview window (headless mode):
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py show_preview:=false
```

Custom preview size:
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py \
  preview_width:=640 \
  preview_height:=480
```

#### Fall Detection Tuning
Adjust fall detection sensitivity:
```bash
ros2 launch orangepi_combined_launch orangepi_combined_launch.py \
  fall_threshold:=0.6 \
  confidence_threshold:=0.5
```

## Launch Arguments

### Camera Parameters
| Argument | Default | Description |
|----------|---------|-------------|
| `frame_width` | `320` | Camera frame width for HF867 |
| `frame_height` | `240` | Camera frame height for HF867 |
| `fps` | `30` | Camera FPS target |
| `exposure_time` | `166` | Manual exposure time (lower = less overexposure) |
| `gain` | `0` | Manual gain setting |
| `brightness` | `50` | Brightness (0-100, lower for HF867) |
| `contrast` | `50` | Contrast (0-100, higher for HF867) |
| `saturation` | `50` | Saturation (0-100, higher for wide-angle) |
| `enable_wide_angle_correction` | `false` | Enable fisheye correction for 120° FOV |
| `enable_color_correction` | `false` | Enable color correction for HF867 |

### Fall Detection Parameters
| Argument | Default | Description |
|----------|---------|-------------|
| `fall_threshold` | `0.7` | Aspect ratio threshold for fall detection |
| `confidence_threshold` | `0.4` | Minimum confidence for person detection |
| `debug_mode` | `false` | Enable debug logging |
| `mqtt_broker` | `192.168.0.224` | MQTT broker IP address |

### Preview Parameters
| Argument | Default | Description |
|----------|---------|-------------|
| `show_preview` | `true` | Show image preview window |
| `preview_width` | `320` | Preview window width |
| `preview_height` | `240` | Preview window height |

### Node Control
| Argument | Default | Description |
|----------|---------|-------------|
| `enable_fall_detection` | `true` | Enable fall detection system |
| `enable_ultrasonic` | `true` | Enable ultrasonic sensors |

## Expected Output

When launched successfully, you should see:

1. **HF867 Camera Node**: Camera initialization and frame publishing
   ```
   [hf867_camera] HF867 camera initialized successfully
   [hf867_camera] Publishing frames at 30 FPS
   [hf867_camera] Manual exposure set to 166
   ```

2. **Fall Detection Node**: Person detection and fall analysis
   ```
   [fall_detector] Fall detection node started
   [fall_detector] Person detected with confidence: 0.85
   [fall_detector] Fall detected! Aspect ratio: 0.45
   ```

3. **Image Preview Node**: Preview window display
   ```
   [hf867_preview] Preview window opened: HF867 Fall Detection Preview
   [hf867_preview] FPS: 28.5 | Timestamp: 2025-01-13 10:30:15
   ```

4. **Ultrasonic Sensors Node**: Distance readings
   ```
   [ultrasonic_sensors] Ultrasonic sensors initialized
   [ultrasonic_sensors] Distance reading: 45.2 cm
   [ultrasonic_sensors] Sensor status: OK
   ```

## Topics

The launch will create these topics:
- `/camera_frames` (sensor_msgs/Image) - Camera frames from HF867
- `/fall_detection_result` (std_msgs/String) - Fall detection results
- `/ultrasonic_distance` (std_msgs/Float32) - Distance readings from ultrasonic sensors

## MQTT Integration

The fall detection system will:
- **Publish to**: `fall_detection/alert` - Fall detection alerts
- **Publish to**: `fall_detection/status` - System status updates

## Troubleshooting

1. **Package not found**: Make sure both `fall_detection_pkg` and `ultrasonic_sensors` are built
2. **Permission denied**: Run `chmod +x build_orangepi.sh` before building
3. **No output**: Check if all packages are properly installed and sourced
4. **Command not found**: Make sure you've sourced the workspace with `source install/setup.bash`
5. **Launch file not found**: Verify the package was built successfully and workspace is sourced
6. **Camera not found**: Check USB connection and permissions for video device
7. **Preview not showing**: Ensure X11 forwarding is enabled for remote connections
8. **Ultrasonic sensors not working**: Check GPIO connections and permissions
9. **Preview window not closing**: Use the cleanup utility or manual commands

## Preview Window Issues

If the camera preview window doesn't close properly after using Ctrl+C:

### Quick Fix
Run the cleanup utility:
```bash
# Python version
python3 close_preview_windows.py

# Or bash version (make executable first)
chmod +x cleanup_preview.sh
./cleanup_preview.sh
```

### Manual Commands
```bash
# Close all OpenCV windows
python3 -c "import cv2; cv2.destroyAllWindows()"

# Kill any hanging image preview processes
pkill -f 'image_preview'

# Force kill if needed
pkill -9 -f 'image_preview'
```

### Prevention
- Always use Ctrl+C to stop the launch
- Wait a few seconds for graceful shutdown
- If using SSH, ensure X11 forwarding is properly configured 