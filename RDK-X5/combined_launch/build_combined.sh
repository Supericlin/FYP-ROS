#!/bin/bash

# Build script for combined launch package
echo "Building combined launch package..."

# Navigate to the combined_launch directory
cd "$(dirname "$0")"

# Build the package
colcon build --packages-select combined_launch

echo "Combined launch package built successfully!"
echo ""
echo "IMPORTANT: You need to source the workspace before using the launch file:"
echo "  source install/setup.bash"
echo ""
echo "Or add this to your ~/.bashrc for permanent sourcing:"
echo "  echo 'source $(pwd)/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Usage examples (after sourcing):"
echo "  # Launch all three nodes with default settings"
echo "  ros2 launch combined_launch combined_nodes_launch.py"
echo ""
echo "  # Launch with debug logging enabled"
echo "  ros2 launch combined_launch combined_nodes_launch.py debug_mode:=true"
echo ""
echo "  # Launch with custom log interval (5 seconds)"
echo "  ros2 launch combined_launch combined_nodes_launch.py log_interval:=5.0"
echo ""
echo "  # Launch only home labels (disable other nodes)"
echo "  ros2 launch combined_launch combined_nodes_launch.py enable_us100_sensor:=false enable_mqtt_navigation:=false"
echo ""
echo "  # Launch only US100 sensor (disable other nodes)"
echo "  ros2 launch combined_launch combined_nodes_launch.py enable_home_labels:=false enable_mqtt_navigation:=false"
echo ""
echo "  # Launch only MQTT navigation (disable other nodes)"
echo "  ros2 launch combined_launch combined_nodes_launch.py enable_home_labels:=false enable_us100_sensor:=false"
echo ""
echo "  # Custom MQTT broker settings"
echo "  ros2 launch combined_launch combined_nodes_launch.py mqtt_broker_address:=192.168.1.100" 