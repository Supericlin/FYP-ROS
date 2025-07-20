#!/usr/bin/env python3

import cv2
import subprocess
import time
import sys

def list_camera_controls():
    """List all available camera controls"""
    try:
        result = subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--list-ctrls'], 
                              capture_output=True, text=True)
        print("Available camera controls:")
        print(result.stdout)
    except Exception as e:
        print(f"Error listing controls: {e}")

def set_camera_control(control, value):
    """Set a camera control"""
    try:
        result = subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c', f'{control}={value}'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ Set {control} to {value}")
        else:
            print(f"✗ Failed to set {control} to {value}: {result.stderr}")
    except Exception as e:
        print(f"Error setting {control}: {e}")

def get_camera_control(control):
    """Get current camera control value"""
    try:
        result = subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-C', control], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            output = result.stdout.strip()
            # Extract just the value from "control: value" format
            if ':' in output:
                return output.split(':')[1].strip()
            return output
        else:
            return None
    except Exception as e:
        print(f"Error getting {control}: {e}")
        return None

def show_camera_preview():
    """Show camera preview for tuning"""
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    # Set basic camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("Camera preview started. Press 'q' to quit, 's' to save settings")
    print("Controls: 1=exposure, 2=gain, 3=brightness, 4=contrast, 5=saturation")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame")
            break
        
        # Add current settings to frame
        exposure = get_camera_control('exposure_time_absolute')
        gain = get_camera_control('gain')
        brightness = get_camera_control('brightness')
        contrast = get_camera_control('contrast')
        saturation = get_camera_control('saturation')
        
        # Draw settings on frame
        cv2.putText(frame, f"Exposure: {exposure}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Gain: {gain}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Brightness: {brightness}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Contrast: {contrast}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, f"Saturation: {saturation}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow('Camera Tuning', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_settings()
        elif key == ord('1'):
            # Adjust exposure
            current = int(get_camera_control('exposure_time_absolute') or '50')
            new_value = max(1, current - 10)
            set_camera_control('exposure_time_absolute', new_value)
        elif key == ord('2'):
            # Adjust gain
            current = int(get_camera_control('gain') or '0')
            new_value = max(0, current + 5)
            set_camera_control('gain', new_value)
        elif key == ord('3'):
            # Adjust brightness
            current = int(get_camera_control('brightness') or '50')
            new_value = max(0, current - 5)
            set_camera_control('brightness', new_value)
        elif key == ord('4'):
            # Adjust contrast
            current = int(get_camera_control('contrast') or '50')
            new_value = min(100, current + 5)
            set_camera_control('contrast', new_value)
        elif key == ord('5'):
            # Adjust saturation
            current = int(get_camera_control('saturation') or '50')
            new_value = min(100, current + 5)
            set_camera_control('saturation', new_value)
    
    cap.release()
    cv2.destroyAllWindows()

def save_settings():
    """Save current camera settings"""
    settings = {
        'exposure_time_absolute': get_camera_control('exposure_time_absolute'),
        'gain': get_camera_control('gain'),
        'brightness': get_camera_control('brightness'),
        'contrast': get_camera_control('contrast'),
        'saturation': get_camera_control('saturation'),
    }
    
    print("\nCurrent camera settings:")
    for key, value in settings.items():
        print(f"  {key}: {value}")
    
    # Save to file
    with open('camera_settings.txt', 'w') as f:
        for key, value in settings.items():
            f.write(f"{key}={value}\n")
    
    print("Settings saved to camera_settings.txt")

def apply_optimized_settings():
    """Apply optimized settings for HF867"""
    print("Applying optimized HF867 settings...")
    
    settings = [
        ('exposure_time_absolute', '30'),  # Very low exposure
        ('gain', '0'),                     # No gain
        ('brightness', '25'),              # Low brightness
        ('contrast', '85'),                # High contrast
        ('saturation', '65'),              # High saturation
        ('white_balance_automatic', '1'),  # Auto white balance
        ('gamma', '120'),                  # Gamma correction
        ('sharpness', '50'),               # Medium sharpness
        ('backlight_compensation', '0'),   # No backlight compensation
    ]
    
    for control, value in settings:
        set_camera_control(control, value)
        time.sleep(0.1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "list":
            list_camera_controls()
        elif command == "optimize":
            apply_optimized_settings()
        elif command == "preview":
            show_camera_preview()
        else:
            print("Usage: python3 camera_tuning.py [list|optimize|preview]")
    else:
        print("Camera Tuning Tool for HF867")
        print("Usage:")
        print("  python3 camera_tuning.py list     - List all camera controls")
        print("  python3 camera_tuning.py optimize - Apply optimized settings")
        print("  python3 camera_tuning.py preview  - Interactive camera preview") 