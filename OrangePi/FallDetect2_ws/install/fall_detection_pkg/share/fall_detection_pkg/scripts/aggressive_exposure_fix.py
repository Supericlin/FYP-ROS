#!/usr/bin/env python3
"""
Aggressive HF867 Exposure Fix
Uses manual exposure and additional camera controls to fix severe overexposure
"""

import cv2
import numpy as np
import time
import subprocess
import os

def check_v4l2_controls(camera_device=0):
    """Check available V4L2 controls for the camera"""
    try:
        result = subprocess.run(['v4l2-ctl', '-d', str(camera_device), '--list-ctrls'], 
                              capture_output=True, text=True)
        print("=== Available V4L2 Controls ===")
        print(result.stdout)
        return result.stdout
    except FileNotFoundError:
        print("v4l2-ctl not found. Install with: sudo apt install v4l-utils")
        return None

def set_v4l2_controls(camera_device=0):
    """Set V4L2 controls directly for aggressive exposure fix"""
    controls = [
        # Disable auto exposure completely
        ['--set-ctrl=exposure_auto=1'],  # Manual mode
        ['--set-ctrl=exposure_time_absolute=1'],  # Minimum exposure
        ['--set-ctrl=exposure_auto_priority=0'],  # Disable auto priority
        
        # Set gain to minimum
        ['--set-ctrl=gain=0'],
        
        # Set brightness to minimum
        ['--set-ctrl=brightness=0'],
        
        # Adjust other controls
        ['--set-ctrl=contrast=50'],
        ['--set-ctrl=saturation=50'],
        ['--set-ctrl=hue=0'],
        ['--set-ctrl=gamma=100'],
        ['--set-ctrl=white_balance_automatic=0'],  # Disable auto white balance
        ['--set-ctrl=white_balance_temperature=4000'],  # Cooler temperature
    ]
    
    print("Setting V4L2 controls for aggressive exposure fix...")
    
    for control in controls:
        try:
            cmd = ['v4l2-ctl', '-d', str(camera_device)] + control
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✅ Set: {' '.join(control)}")
            else:
                print(f"❌ Failed: {' '.join(control)} - {result.stderr}")
        except Exception as e:
            print(f"❌ Error setting {' '.join(control)}: {e}")

def test_aggressive_exposure_fix(camera_device=0):
    """Test camera with aggressive exposure settings"""
    
    # First, set V4L2 controls
    set_v4l2_controls(camera_device)
    
    cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"Failed to open camera device {camera_device}")
        return
    
    # Set basic properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    
    # Aggressive manual exposure settings
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
    cap.set(cv2.CAP_PROP_EXPOSURE, 1)  # Minimum exposure
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)  # Minimum brightness
    cap.set(cv2.CAP_PROP_GAIN, 0)  # Minimum gain
    cap.set(cv2.CAP_PROP_CONTRAST, 50)
    cap.set(cv2.CAP_PROP_SATURATION, 50)
    
    print("=== Aggressive Exposure Fix Test ===")
    print("Controls:")
    print("  '1-9' - Set exposure (1=min, 9=max)")
    print("  'b' - Decrease brightness")
    print("  'B' - Increase brightness")
    print("  'g' - Decrease gain")
    print("  'G' - Increase gain")
    print("  'c' - Decrease contrast")
    print("  'C' - Increase contrast")
    print("  's' - Save settings")
    print("  'q' - Quit")
    print()
    
    current_exposure = 1
    current_brightness = 0
    current_gain = 0
    current_contrast = 50
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        # Get current camera settings
        actual_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
        actual_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        actual_gain = cap.get(cv2.CAP_PROP_GAIN)
        actual_contrast = cap.get(cv2.CAP_PROP_CONTRAST)
        
        # Add settings info to frame
        cv2.putText(frame, f"Exposure: {actual_exposure:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Brightness: {actual_brightness:.1f}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Gain: {actual_gain:.1f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Contrast: {actual_contrast:.1f}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Mode: Manual (Aggressive)", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Add histogram for exposure analysis
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist_norm = hist.flatten() / hist.sum()
        
        # Create histogram visualization
        hist_img = np.zeros((100, 256, 3), dtype=np.uint8)
        for i in range(256):
            height = int(hist_norm[i] * 100)
            cv2.line(hist_img, (i, 99), (i, 99-height), (255, 255, 255), 1)
        
        # Add histogram to frame
        hist_resized = cv2.resize(hist_img, (256, 100))
        frame[480-110:480-10, 10:266] = hist_resized
        
        # Add histogram labels
        cv2.putText(frame, "Histogram (Dark)", (10, 480-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Bright", (220, 480-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add overexposure warning
        if np.mean(gray) > 200:
            cv2.putText(frame, "OVEREXPOSED!", (10, 480-130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        cv2.imshow('Aggressive Exposure Fix', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8'), ord('9')]:
            # Set exposure based on key
            exposure_map = {ord('1'): 1, ord('2'): 10, ord('3'): 20, ord('4'): 30, 
                           ord('5'): 50, ord('6'): 75, ord('7'): 100, ord('8'): 150, ord('9'): 200}
            current_exposure = exposure_map[key]
            cap.set(cv2.CAP_PROP_EXPOSURE, current_exposure)
            print(f"Exposure set to: {current_exposure}")
        elif key == ord('b'):
            current_brightness = max(-100, current_brightness - 5)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
            print(f"Brightness: {current_brightness}")
        elif key == ord('B'):
            current_brightness = min(100, current_brightness + 5)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
            print(f"Brightness: {current_brightness}")
        elif key == ord('g'):
            current_gain = max(0, current_gain - 5)
            cap.set(cv2.CAP_PROP_GAIN, current_gain)
            print(f"Gain: {current_gain}")
        elif key == ord('G'):
            current_gain = min(100, current_gain + 5)
            cap.set(cv2.CAP_PROP_GAIN, current_gain)
            print(f"Gain: {current_gain}")
        elif key == ord('c'):
            current_contrast = max(0, current_contrast - 5)
            cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
            print(f"Contrast: {current_contrast}")
        elif key == ord('C'):
            current_contrast = min(100, current_contrast + 5)
            cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
            print(f"Contrast: {current_contrast}")
        elif key == ord('s'):
            # Save current settings
            settings = {
                'auto_exposure': False,
                'exposure_time': cap.get(cv2.CAP_PROP_EXPOSURE),
                'brightness': cap.get(cv2.CAP_PROP_BRIGHTNESS),
                'gain': cap.get(cv2.CAP_PROP_GAIN),
                'contrast': cap.get(cv2.CAP_PROP_CONTRAST)
            }
            print("\n=== Aggressive Settings ===")
            for key, value in settings.items():
                print(f"{key}: {value}")
            print("\nUse these values in your launch file:")
            print(f"auto_exposure:=false")
            print(f"exposure_time:={settings['exposure_time']:.0f}")
            print(f"brightness:={settings['brightness']:.0f}")
            print(f"gain:={settings['gain']:.0f}")
            print(f"contrast:={settings['contrast']:.0f}")
    
    cap.release()
    cv2.destroyAllWindows()

def main():
    print("=== HF867 Aggressive Exposure Fix ===")
    print("This tool uses manual exposure and V4L2 controls to fix severe overexposure")
    print()
    
    # Check V4L2 controls first
    check_v4l2_controls()
    print()
    
    # Test aggressive exposure fix
    test_aggressive_exposure_fix()

if __name__ == "__main__":
    main() 