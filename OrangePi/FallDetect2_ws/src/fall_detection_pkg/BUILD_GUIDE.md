# Orange Pi Camera Package Build Guide

## 🏗️ Package Structure Setup

### Step 1: Run the Setup Script
```bash
cd OrangePi
chmod +x build_package.sh
./build_package.sh
```

This will create the proper ROS2 package structure:
```
OrangePi/
├── orange_pi_camera/          # Python package directory
│   ├── __init__.py
│   ├── camera_node.py
│   └── fall_detection_node.py
├── launch/                    # Launch files
│   ├── hf867_system_launch.py
│   ├── camera_launch.py
│   └── fall_detection_launch.py
├── config/                    # Configuration files
│   └── hf867_camera_config.yaml
├── models/                    # TFLite models
│   └── mobilenet_ssd.tflite
├── calibration_data/          # Camera calibration files
├── resource/                  # ROS2 resource marker
├── package.xml               # Package manifest
├── setup.py                  # Package setup
└── build_package.sh          # Setup script
```

### Step 2: Add Your TFLite Model
```bash
# Copy your TFLite model to the models directory
cp /path/to/your/mobilenet_ssd.tflite models/
```

## 🔨 Building the Package

### Option 1: Build from OrangePi Directory
```bash
cd OrangePi
colcon build --packages-select orange_pi_camera
```

### Option 2: Build from Parent Directory
```bash
# From the parent directory (FYP-ROS)
colcon build --packages-select orange_pi_camera
```

### Option 3: Build All Packages
```bash
# Build all packages in the workspace
colcon build
```

## 🚀 After Building

### Step 1: Source the Workspace
```bash
# Source the workspace to make the package available
source install/setup.bash

# Or if you're in the OrangePi directory
source ../install/setup.bash
```

### Step 2: Verify Package Installation
```bash
# Check if the package is recognized
ros2 pkg list | grep orange_pi_camera

# Check available launch files
ros2 launch orange_pi_camera --help
```

### Step 3: Launch the System
```bash
# Launch the complete HF867 system
ros2 launch orange_pi_camera hf867_system_launch.py

# Or launch individual components
ros2 launch orange_pi_camera camera_launch.py
ros2 launch orange_pi_camera fall_detection_launch.py
```

## 🔧 Troubleshooting

### Common Build Issues:

#### 1. Missing Dependencies
```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs

# Install Python dependencies
pip3 install opencv-python numpy tflite-runtime mediapipe paho-mqtt
```

#### 2. Permission Issues
```bash
# Make Python files executable
chmod +x orange_pi_camera/*.py
chmod +x *.py
```

#### 3. Package Not Found
```bash
# Make sure you're in the right directory
pwd  # Should show path to OrangePi

# Check package structure
ls -la orange_pi_camera/
ls -la launch/
ls -la config/
```

#### 4. Model File Missing
```bash
# Check if model file exists
ls -la models/mobilenet_ssd.tflite

# If missing, you'll need to add it
echo "Please add your TFLite model file to models/mobilenet_ssd.tflite"
```

### Build Verification:
```bash
# Check build status
colcon build --packages-select orange_pi_camera --event-handlers console_direct+

# Check for errors
colcon build --packages-select orange_pi_camera --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## 📋 Complete Build Checklist

- [ ] Run `./build_package.sh`
- [ ] Add TFLite model to `models/` directory
- [ ] Run `colcon build --packages-select orange_pi_camera`
- [ ] Source workspace: `source install/setup.bash`
- [ ] Verify package: `ros2 pkg list | grep orange_pi_camera`
- [ ] Test launch: `ros2 launch orange_pi_camera hf867_system_launch.py`

## 🎯 Quick Commands Summary

```bash
# Setup and build
cd OrangePi
./build_package.sh
colcon build --packages-select orange_pi_camera
source install/setup.bash

# Launch system
ros2 launch orange_pi_camera hf867_system_launch.py

# Test individual components
ros2 run orange_pi_camera camera_node
ros2 run orange_pi_camera fall_detection_node
```

## 📝 Notes

1. **Always source the workspace** after building
2. **Check dependencies** if build fails
3. **Verify model file** exists before running
4. **Use debug mode** for troubleshooting
5. **Monitor system resources** during operation 