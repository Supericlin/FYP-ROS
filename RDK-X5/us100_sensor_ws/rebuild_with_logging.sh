#!/bin/bash

echo "=== Rebuilding US100 Sensor Package with Enhanced Logging ==="
echo ""
echo "📊 New Logging Features Added:"
echo "1. Comprehensive status reporting every 10 seconds"
echo "2. Individual sensor performance tracking"
echo "3. Error rate monitoring and statistics"
echo "4. Debug mode for detailed sensor readings"
echo "5. Uptime and performance metrics"
echo "6. Clean shutdown with final statistics"
echo "7. Configurable logging parameters"
echo ""

# Clean build
echo "Cleaning previous build..."
rm -rf build/ install/ log/

# Build the package
echo "Building US100 sensor package..."
colcon build --packages-select us100_sensor_pkg

if [ $? -eq 0 ]; then
    echo ""
    echo "=== Build successful! ==="
    echo ""
    
    # Fix executables if needed
    if [ -d "install/us100_sensor_pkg/bin" ] && [ ! -d "install/us100_sensor_pkg/lib/us100_sensor_pkg" ]; then
        echo "Fixing executable locations..."
        mkdir -p install/us100_sensor_pkg/lib/us100_sensor_pkg/
        ln -sf ../../bin/us100_sensor_node install/us100_sensor_pkg/lib/us100_sensor_pkg/us100_sensor_node
        echo "✅ Executables fixed"
    fi
    
    echo ""
    echo "=== US100 Sensor Package Ready! ==="
    echo ""
    echo "🎯 Usage Options:"
    echo ""
    echo "1. Launch with default logging (status every 10s):"
    echo "   source install/setup.bash"
    echo "   ros2 launch us100_sensor_pkg us100_sensor_launch.py"
    echo ""
    echo "2. Launch with debug mode (detailed sensor readings):"
    echo "   ros2 launch us100_sensor_pkg us100_sensor_launch.py debug_mode:=true"
    echo ""
    echo "3. Launch with detailed logging (every sensor reading):"
    echo "   ros2 launch us100_sensor_pkg us100_sensor_launch.py \\"
    echo "       debug_mode:=true enable_detailed_logging:=true"
    echo ""
    echo "4. Launch with custom status interval (every 5 seconds):"
    echo "   ros2 launch us100_sensor_pkg us100_sensor_launch.py log_interval:=5.0"
    echo ""
    echo "5. Disable status logging:"
    echo "   ros2 launch us100_sensor_pkg us100_sensor_launch.py log_interval:=0.0"
    echo ""
    echo "6. Run node directly:"
    echo "   source install/setup.bash"
    echo "   ros2 run us100_sensor_pkg us100_sensor_node"
    echo ""
    echo "📊 Expected Log Output:"
    echo "- 🚀 Startup messages with sensor initialization"
    echo "- 📡 Publisher creation confirmations"
    echo "- 🎯 Ready status message"
    echo "- 📊 Status reports every 10 seconds (configurable)"
    echo "- ❌ Error messages for failed readings"
    echo "- 🛑 Clean shutdown with final statistics"
    echo ""
    echo "📈 Status Report Includes:"
    echo "- Uptime and total reads"
    echo "- Error counts and rates per sensor"
    echo "- Last distance readings"
    echo "- GPIO and node status"
    echo "- Individual sensor performance"
else
    echo "❌ Build failed! Please check the error messages above."
    exit 1
fi 