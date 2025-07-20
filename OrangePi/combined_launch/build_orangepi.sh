#!/bin/bash

# Build script for OrangePi combined launch package
echo "Building OrangePi combined launch package..."

# Navigate to the combined_launch directory
cd "$(dirname "$0")"

# Build the package
colcon build --packages-select orangepi_combined_launch

echo "OrangePi combined launch package built successfully!"
echo ""
echo "IMPORTANT: You need to source the workspace before using the launch file:"
echo "  source install/setup.bash"
echo ""
echo "Or add this to your ~/.bashrc for permanent sourcing:"
echo "  echo 'source $(pwd)/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Usage examples (after sourcing):"
echo "  # Launch both fall detection and ultrasonic sensors with default settings"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py"
echo ""
echo "  # Launch with debug logging enabled"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py debug_mode:=true"
echo ""
echo "  # Launch only fall detection (disable ultrasonic)"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py enable_ultrasonic:=false"
echo ""
echo "  # Launch only ultrasonic sensors (disable fall detection)"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py enable_fall_detection:=false"
echo ""
echo "  # Custom camera settings"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py exposure_time:=100 brightness:=40"
echo ""
echo "  # Disable preview window (headless mode)"
echo "  ros2 launch orangepi_combined_launch orangepi_combined_launch.py show_preview:=false" 