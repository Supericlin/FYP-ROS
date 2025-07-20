# OrangePi Camera Troubleshooting Guide

## 🚨 Problem: Camera Stops Working After Few Minutes

### **Symptoms:**
- Camera works initially for 2-5 minutes
- Then shows: `[WARN] Failed to capture frame`
- Continuous frame capture failures
- No recovery without restart

## 🔧 Root Causes & Solutions

### **1. Resource Exhaustion**
**Cause:** Memory leaks, buffer overflow, or resource conflicts

**Solutions:**
```bash
# Use improved camera node with auto-reconnect
ros2 launch orange_pi_camera camera_launch.py enable_auto_reconnect:=true

# Or run with reduced buffer size
ros2 run orange_pi_camera camera_node --ros-args -p buffer_size:=1
```

### **2. Camera Driver Issues**
**Cause:** V4L2 driver instability or conflicts

**Solutions:**
```bash
# Reset camera system
sudo v4l2-ctl -d /dev/video0 --reset

# Reload camera modules
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo

# Or use the troubleshooting script
python3 camera_troubleshooter.py
```

### **3. USB Power Issues**
**Cause:** Insufficient power for USB camera

**Solutions:**
- Use powered USB hub
- Try different USB port
- Check USB cable quality
- Reduce camera resolution/FPS

### **4. System Resource Constraints**
**Cause:** High CPU/memory usage

**Solutions:**
```bash
# Monitor system resources
htop
free -h

# Reduce camera load
ros2 run orange_pi_camera camera_node --ros-args -p fps:=10 -p frame_width:=160 -p frame_height:=120
```

## 🚀 Quick Fixes

### **Immediate Solution (Recommended):**
```bash
cd OrangePi/fallDetect_ws

# Build the improved camera node
colcon build --packages-select orange_pi_camera

# Source the workspace
source install/setup.bash

# Run with auto-reconnect enabled
ros2 launch orange_pi_camera camera_launch.py enable_auto_reconnect:=true debug_mode:=true
```

### **Alternative: Manual Run with Parameters**
```bash
ros2 run orange_pi_camera camera_node --ros-args \
  -p enable_auto_reconnect:=true \
  -p reconnect_delay:=2.0 \
  -p max_reconnect_attempts:=5 \
  -p buffer_size:=1 \
  -p fps:=10 \
  -p debug_mode:=true
```

## 🔍 Diagnostic Tools

### **1. Camera Troubleshooter Script**
```bash
# Run comprehensive diagnostics
python3 camera_troubleshooter.py

# Test specific camera device
python3 camera_troubleshooter.py 0
```

### **2. Manual V4L2 Testing**
```bash
# Check camera devices
ls -la /dev/video*
v4l2-ctl --list-devices

# Test camera capture
ffmpeg -f v4l2 -i /dev/video0 -t 10 -f null -

# Check camera capabilities
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-ctrls
```

### **3. System Monitoring**
```bash
# Monitor CPU and memory
htop
free -h

# Check USB devices
lsusb
dmesg | grep -i usb

# Monitor camera processes
ps aux | grep camera
```

## ⚙️ Optimized Settings

### **For Stability (Recommended):**
```bash
ros2 launch orange_pi_camera camera_launch.py \
  frame_width:=320 \
  frame_height:=240 \
  fps:=10 \
  buffer_size:=1 \
  enable_auto_reconnect:=true \
  reconnect_delay:=2.0 \
  max_reconnect_attempts:=5
```

### **For Performance:**
```bash
ros2 launch orange_pi_camera camera_launch.py \
  frame_width:=640 \
  frame_height:=480 \
  fps:=15 \
  buffer_size:=2 \
  enable_auto_reconnect:=true
```

### **For Low Resource Systems:**
```bash
ros2 launch orange_pi_camera camera_launch.py \
  frame_width:=160 \
  frame_height:=120 \
  fps:=5 \
  buffer_size:=1 \
  enable_auto_reconnect:=true
```

## 🛠️ Advanced Troubleshooting

### **1. Camera Module Issues**
```bash
# Check loaded modules
lsmod | grep uvc

# Reload modules
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo timeout=5000

# Check kernel messages
dmesg | tail -20
```

### **2. USB Power Management**
```bash
# Disable USB autosuspend
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend

# Check USB power
cat /sys/bus/usb/devices/*/power/runtime_status
```

### **3. V4L2 Buffer Issues**
```bash
# Set optimal buffer settings
v4l2-ctl -d /dev/video0 --set-fmt-video=width=320,height=240,pixelformat=MJPG
v4l2-ctl -d /dev/video0 --set-parm=10

# Check buffer status
v4l2-ctl -d /dev/video0 --get-fmt-video
```

## 📊 Monitoring & Logging

### **Enable Debug Mode:**
```bash
ros2 launch orange_pi_camera camera_launch.py debug_mode:=true
```

### **Watch for These Log Messages:**
- `✅ Camera initialized successfully` - Camera working
- `🔄 Attempting camera reconnection` - Auto-recovery in progress
- `✅ Camera reconnection successful` - Recovery successful
- `❌ Max reconnection attempts reached` - Need manual intervention

### **Performance Monitoring:**
```bash
# Monitor camera FPS
ros2 topic echo /camera/image --once

# Check topic frequency
ros2 topic hz /camera/image
```

## 🔄 Recovery Procedures

### **When Camera Fails:**

1. **Try Auto-Reconnect** (if enabled):
   - Wait 10-30 seconds for automatic recovery
   - Check logs for reconnection attempts

2. **Manual Reset:**
   ```bash
   # Stop the node
   Ctrl+C
   
   # Reset camera
   sudo v4l2-ctl -d /dev/video0 --reset
   
   # Restart
   ros2 run orange_pi_camera camera_node
   ```

3. **System Reset:**
   ```bash
   # Reload modules
   sudo modprobe -r uvcvideo
   sudo modprobe uvcvideo
   
   # Or reboot if necessary
   sudo reboot
   ```

## 💡 Prevention Tips

### **1. Use Improved Camera Node**
- Always use the updated camera node with auto-reconnect
- Enable debug mode during testing
- Use appropriate buffer sizes

### **2. System Optimization**
- Keep system updated
- Monitor resource usage
- Use stable USB ports
- Avoid USB hubs if possible

### **3. Camera Settings**
- Start with lower resolution/FPS
- Gradually increase if stable
- Use MJPG format for better compatibility
- Set appropriate buffer sizes

### **4. Monitoring**
- Enable debug logging
- Monitor system resources
- Check camera health regularly
- Set up automatic restart scripts if needed

## 🆘 Emergency Recovery

If nothing works:

1. **Hard Reset:**
   ```bash
   sudo reboot
   ```

2. **Check Hardware:**
   - Try different USB cable
   - Test camera on different computer
   - Check USB port functionality

3. **Reinstall Dependencies:**
   ```bash
   sudo apt update
   sudo apt install v4l-utils ffmpeg
   ```

4. **Contact Support:**
   - Check camera manufacturer support
   - Verify OrangePi compatibility
   - Consider alternative camera models 