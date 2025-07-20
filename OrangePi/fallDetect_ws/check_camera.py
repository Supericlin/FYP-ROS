#!/usr/bin/env python3
"""
Simple USB Camera Checker
This script helps identify your USB camera and its capabilities.
"""

import subprocess
import sys

def run_command(cmd):
    """Run a command and return the result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

def main():
    print("🔍 USB Camera Checker")
    print("=" * 30)
    
    # Check USB devices
    print("📱 USB Devices:")
    success, stdout, stderr = run_command("lsusb")
    if success:
        print(stdout)
    else:
        print("❌ Failed to list USB devices")
    
    # Check video devices
    print("\n📹 Video Devices:")
    success, stdout, stderr = run_command("ls -la /dev/video*")
    if success:
        print(stdout)
    else:
        print("❌ No video devices found")
    
    # Check V4L2 devices
    print("\n🎥 V4L2 Devices:")
    success, stdout, stderr = run_command("v4l2-ctl --list-devices")
    if success:
        print(stdout)
    else:
        print("❌ V4L2 command failed")
    
    # Check camera capabilities (if video0 exists)
    print("\n⚙️ Camera Capabilities (/dev/video0):")
    success, stdout, stderr = run_command("v4l2-ctl -d /dev/video0 --list-formats-ext")
    if success:
        print(stdout)
    else:
        print("❌ Failed to get camera capabilities")
    
    print("\n✅ Camera check complete!")
    print("\n💡 If you see your camera listed above, the improved camera node will work with it!")

if __name__ == "__main__":
    main() 