#!/usr/bin/env python3

import cv2
import subprocess
import time
import sys

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

def simple_camera_viewer():
    """Simple camera viewer with basic controls"""
    print("Starting simple camera viewer...")
    print("Controls:")
    print("  q - Quit")
    print("  s - Save current settings")
    print("  r - Reset to default settings")
    print("  e - Decrease exposure")
    print("  E - Increase exposure")
    print("  b - Decrease brightness")
    print("  B - Increase brightness")
    print("  c - Decrease contrast")
    print("  C - Increase contrast")
    print("  g - Decrease gain")
    print("  G - Increase gain")
    print("  a - Decrease saturation")
    print("  A - Increase saturation")
    
    # Initialize camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    # Set basic properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # Apply initial settings
    set_camera_control('exposure_time_absolute', '30')
    set_camera_control('gain', '0')
    set_camera_control('brightness', '25')
    set_camera_control('contrast', '85')
    set_camera_control('saturation', '65')
    
    # Small delay to let settings take effect
    time.sleep(0.5)
    
    print("Camera viewer started. Press 'q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame")
            break
        
        # Get current settings
        exposure = get_camera_control('exposure_time_absolute') or 'N/A'
        gain = get_camera_control('gain') or 'N/A'
        brightness = get_camera_control('brightness') or 'N/A'
        contrast = get_camera_control('contrast') or 'N/A'
        saturation = get_camera_control('saturation') or 'N/A'
        
        # Draw settings on frame
        cv2.putText(frame, f"Exposure: {exposure}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Gain: {gain}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Brightness: {brightness}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Contrast: {contrast}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Saturation: {saturation}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add instructions
        cv2.putText(frame, "q=quit, s=save, r=reset, e/E=exposure, b/B=brightness, a/A=saturation", (10, 480-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('HF867 Camera Viewer', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_current_settings()
        elif key == ord('r'):
            reset_to_defaults()
        elif key == ord('e'):
            adjust_exposure(-10)
        elif key == ord('E'):
            adjust_exposure(10)
        elif key == ord('b'):
            adjust_brightness(-5)
        elif key == ord('B'):
            adjust_brightness(5)
        elif key == ord('c'):
            adjust_contrast(-5)
        elif key == ord('C'):
            adjust_contrast(5)
        elif key == ord('g'):
            adjust_gain(-5)
        elif key == ord('G'):
            adjust_gain(5)
        elif key == ord('a'):
            adjust_saturation(-5)
        elif key == ord('A'):
            adjust_saturation(5)
    
    cap.release()
    cv2.destroyAllWindows()

def adjust_exposure(delta):
    """Adjust exposure by delta"""
    try:
        current_str = get_camera_control('exposure_time_absolute')
        if current_str and current_str.isdigit():
            current = int(current_str)
            new_value = max(1, current + delta)
            set_camera_control('exposure_time_absolute', str(new_value))
        else:
            print("Could not get current exposure value")
    except (ValueError, TypeError) as e:
        print(f"Error adjusting exposure: {e}")

def adjust_brightness(delta):
    """Adjust brightness by delta"""
    try:
        current_str = get_camera_control('brightness')
        if current_str and current_str.isdigit():
            current = int(current_str)
            new_value = max(0, min(100, current + delta))
            set_camera_control('brightness', str(new_value))
        else:
            print("Could not get current brightness value")
    except (ValueError, TypeError) as e:
        print(f"Error adjusting brightness: {e}")

def adjust_contrast(delta):
    """Adjust contrast by delta"""
    try:
        current_str = get_camera_control('contrast')
        if current_str and current_str.isdigit():
            current = int(current_str)
            new_value = max(0, min(100, current + delta))
            set_camera_control('contrast', str(new_value))
        else:
            print("Could not get current contrast value")
    except (ValueError, TypeError) as e:
        print(f"Error adjusting contrast: {e}")

def adjust_gain(delta):
    """Adjust gain by delta"""
    try:
        current_str = get_camera_control('gain')
        if current_str and current_str.isdigit():
            current = int(current_str)
            new_value = max(0, current + delta)
            set_camera_control('gain', str(new_value))
        else:
            print("Could not get current gain value")
    except (ValueError, TypeError) as e:
        print(f"Error adjusting gain: {e}")

def adjust_saturation(delta):
    """Adjust saturation by delta"""
    try:
        current_str = get_camera_control('saturation')
        if current_str and current_str.isdigit():
            current = int(current_str)
            new_value = max(0, min(100, current + delta))
            set_camera_control('saturation', str(new_value))
        else:
            print("Could not get current saturation value")
    except (ValueError, TypeError) as e:
        print(f"Error adjusting saturation: {e}")

def save_current_settings():
    """Save current camera settings"""
    settings = {
        'exposure_time_absolute': get_camera_control('exposure_time_absolute'),
        'gain': get_camera_control('gain'),
        'brightness': get_camera_control('brightness'),
        'contrast': get_camera_control('contrast'),
        'saturation': get_camera_control('saturation'),
    }
    
    print("\n=== Current Camera Settings ===")
    for key, value in settings.items():
        print(f"  {key}: {value}")
    
    # Save to file
    with open('optimal_camera_settings.txt', 'w') as f:
        for key, value in settings.items():
            f.write(f"{key}={value}\n")
    
    print("Settings saved to optimal_camera_settings.txt")
    print("You can copy these values to your launch file!")

def reset_to_defaults():
    """Reset to default settings"""
    print("Resetting to default settings...")
    set_camera_control('exposure_time_absolute', '50')
    set_camera_control('gain', '0')
    set_camera_control('brightness', '30')
    set_camera_control('contrast', '80')
    set_camera_control('saturation', '60')

if __name__ == "__main__":
    simple_camera_viewer() 