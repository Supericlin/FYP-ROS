@echo off
REM Build script for combined launch package
echo Building combined launch package...

REM Navigate to the combined_launch directory
cd /d "%~dp0"

REM Build the package
colcon build --packages-select combined_launch

REM Source the workspace (for Windows)
call install\setup.bat

echo.
echo Combined launch package built successfully!
echo.
echo Usage examples:
echo   # Launch both nodes with default settings
echo   ros2 launch combined_launch combined_nodes_launch.py
echo.
echo   # Launch with debug logging enabled
echo   ros2 launch combined_launch combined_nodes_launch.py debug_mode:=true
echo.
echo   # Launch with custom log interval (5 seconds)
echo   ros2 launch combined_launch combined_nodes_launch.py log_interval:=5.0
echo.
echo   # Launch only home labels (disable US100 sensor)
echo   ros2 launch combined_launch combined_nodes_launch.py enable_us100_sensor:=false
echo.
echo   # Launch only US100 sensor (disable home labels)
echo   ros2 launch combined_launch combined_nodes_launch.py enable_home_labels:=false 