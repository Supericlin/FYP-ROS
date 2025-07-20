#!/usr/bin/env python3
"""
Camera Troubleshooter for OrangePi
This script helps diagnose and fix camera issues.
"""

import subprocess
import time
import os
import sys

def run_command(cmd, capture_output=True, timeout=10):
    """Run a command and return the result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=capture_output, 
                              text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)

def check_camera_devices():
    """Check available camera devices"""
    print("🔍 Checking camera devices...")
    
    # Check /dev/video* devices
    success, stdout, stderr = run_command("ls -la /dev/video*")
    if success:
        print("✅ Available video devices:")
        print(stdout)
    else:
        print("❌ No video devices found")
        print(stderr)
    
    # Check v4l2 devices
    success, stdout, stderr = run_command("v4l2-ctl --list-devices")
    if success:
        print("✅ V4L2 devices:")
        print(stdout)
    else:
        print("❌ V4L2 command failed")
        print(stderr)

def check_camera_capabilities(device_index=0):
    """Check camera capabilities"""
    print(f"\n🔍 Checking camera capabilities for /dev/video{device_index}...")
    
    # Check formats
    success, stdout, stderr = run_command(f"v4l2-ctl -d /dev/video{device_index} --list-formats-ext")
    if success:
        print("✅ Supported formats:")
        print(stdout)
    else:
        print("❌ Failed to get formats")
        print(stderr)
    
    # Check controls
    success, stdout, stderr = run_command(f"v4l2-ctl -d /dev/video{device_index} --list-ctrls")
    if success:
        print("✅ Available controls:")
        print(stdout)
    else:
        print("❌ Failed to get controls")
        print(stderr)

def test_camera_capture(device_index=0, duration=5):
    """Test camera capture"""
    print(f"\n📹 Testing camera capture for {duration} seconds...")
    
    # Test with ffmpeg
    cmd = f"timeout {duration} ffmpeg -f v4l2 -i /dev/video{device_index} -t {duration} -f null -"
    success, stdout, stderr = run_command(cmd, capture_output=False, timeout=duration+5)
    
    if success:
        print("✅ Camera capture test successful")
    else:
        print("❌ Camera capture test failed")
        print(stderr)

def reset_camera_system(device_index=0):
    """Reset camera system"""
    print(f"\n🔄 Resetting camera system...")
    
    # Reset camera
    success, stdout, stderr = run_command(f"v4l2-ctl -d /dev/video{device_index} --reset")
    if success:
        print("✅ Camera reset successful")
    else:
        print("❌ Camera reset failed")
        print(stderr)
    
    # Wait a bit
    time.sleep(2)
    
    # Unload and reload modules
    print("🔄 Reloading camera modules...")
    run_command("sudo modprobe -r uvcvideo", capture_output=False)
    time.sleep(1)
    run_command("sudo modprobe uvcvideo", capture_output=False)
    time.sleep(2)

def check_system_resources():
    """Check system resources"""
    print("\n💻 Checking system resources...")
    
    # Check CPU usage
    success, stdout, stderr = run_command("top -bn1 | grep 'Cpu(s)'")
    if success:
        print("✅ CPU usage:")
        print(stdout)
    
    # Check memory usage
    success, stdout, stderr = run_command("free -h")
    if success:
        print("✅ Memory usage:")
        print(stdout)
    
    # Check USB devices
    success, stdout, stderr = run_command("lsusb")
    if success:
        print("✅ USB devices:")
        print(stdout)

def optimize_camera_settings(device_index=0):
    """Optimize camera settings"""
    print(f"\n⚙️ Optimizing camera settings...")
    
    # Set buffer size
    run_command(f"v4l2-ctl -d /dev/video{device_index} --set-fmt-video=width=320,height=240,pixelformat=MJPG")
    
    # Set FPS
    run_command(f"v4l2-ctl -d /dev/video{device_index} --set-parm=15")
    
    # Disable auto exposure if available
    run_command(f"v4l2-ctl -d /dev/video{device_index} --set-ctrl=exposure_auto=1")
    
    print("✅ Camera settings optimized")

def main():
    """Main troubleshooting function"""
    print("🔧 OrangePi Camera Troubleshooter")
    print("=" * 40)
    
    if len(sys.argv) > 1:
        device_index = int(sys.argv[1])
    else:
        device_index = 0
    
    print(f"Using camera device: /dev/video{device_index}")
    
    # Run diagnostics
    check_camera_devices()
    check_camera_capabilities(device_index)
    check_system_resources()
    
    # Ask user what to do
    print("\n🎯 Choose an action:")
    print("1. Test camera capture")
    print("2. Reset camera system")
    print("3. Optimize camera settings")
    print("4. Run all tests")
    print("5. Exit")
    
    choice = input("\nEnter your choice (1-5): ").strip()
    
    if choice == "1":
        test_camera_capture(device_index)
    elif choice == "2":
        reset_camera_system(device_index)
    elif choice == "3":
        optimize_camera_settings(device_index)
    elif choice == "4":
        test_camera_capture(device_index)
        reset_camera_system(device_index)
        optimize_camera_settings(device_index)
    elif choice == "5":
        print("👋 Goodbye!")
        return
    else:
        print("❌ Invalid choice")
        return
    
    print("\n✅ Troubleshooting complete!")
    print("\n💡 Tips:")
    print("- If camera still fails, try rebooting the system")
    print("- Check if camera is properly connected")
    print("- Try different USB ports")
    print("- Use the improved camera node with auto-reconnect")

if __name__ == "__main__":
    main() 