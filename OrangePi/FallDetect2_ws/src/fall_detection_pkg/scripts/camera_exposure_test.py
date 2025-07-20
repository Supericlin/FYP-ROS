#!/usr/bin/env python3
"""
HF867 Camera Exposure Test Script
Helps adjust camera settings to fix overexposure issues
"""

import cv2
import numpy as np
import argparse
import time

def test_camera_exposure(camera_device=0, width=640, height=480):
    """Test camera with different exposure settings"""
    
    cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"Failed to open camera device {camera_device}")
        return
    
    # Set basic properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    
    print("=== HF867 Camera Exposure Test ===")
    print("Controls:")
    print("  '1' - Auto exposure")
    print("  '2' - Manual exposure (low)")
    print("  '3' - Manual exposure (medium)")
    print("  '4' - Manual exposure (high)")
    print("  'b' - Decrease brightness")
    print("  'B' - Increase brightness")
    print("  'g' - Decrease gain")
    print("  'G' - Increase gain")
    print("  'c' - Decrease contrast")
    print("  'C' - Increase contrast")
    print("  's' - Save current settings")
    print("  'q' - Quit")
    print()
    
    # Current settings
    current_exposure = 100
    current_brightness = 30
    current_gain = 0
    current_contrast = 60
    auto_exposure = True
    
    # Set initial settings
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto
    cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
    cap.set(cv2.CAP_PROP_GAIN, current_gain)
    
    print("Starting with auto exposure and reduced brightness...")
    
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
        cv2.putText(frame, f"Mode: {'Auto' if auto_exposure else 'Manual'}", (10, 150), 
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
        frame[height-110:height-10, 10:266] = hist_resized
        
        # Add histogram labels
        cv2.putText(frame, "Histogram (Dark)", (10, height-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, "Bright", (220, height-5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('HF867 Exposure Test', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('1'):
            # Auto exposure
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            auto_exposure = True
            print("Switched to auto exposure")
        elif key == ord('2'):
            # Manual exposure (low)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, 50)
            auto_exposure = False
            print("Manual exposure: Low (50)")
        elif key == ord('3'):
            # Manual exposure (medium)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, 100)
            auto_exposure = False
            print("Manual exposure: Medium (100)")
        elif key == ord('4'):
            # Manual exposure (high)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, 200)
            auto_exposure = False
            print("Manual exposure: High (200)")
        elif key == ord('b'):
            # Decrease brightness
            current_brightness = max(0, current_brightness - 5)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
            print(f"Brightness: {current_brightness}")
        elif key == ord('B'):
            # Increase brightness
            current_brightness = min(100, current_brightness + 5)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
            print(f"Brightness: {current_brightness}")
        elif key == ord('g'):
            # Decrease gain
            current_gain = max(0, current_gain - 5)
            cap.set(cv2.CAP_PROP_GAIN, current_gain)
            print(f"Gain: {current_gain}")
        elif key == ord('G'):
            # Increase gain
            current_gain = min(100, current_gain + 5)
            cap.set(cv2.CAP_PROP_GAIN, current_gain)
            print(f"Gain: {current_gain}")
        elif key == ord('c'):
            # Decrease contrast
            current_contrast = max(0, current_contrast - 5)
            cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
            print(f"Contrast: {current_contrast}")
        elif key == ord('C'):
            # Increase contrast
            current_contrast = min(100, current_contrast + 5)
            cap.set(cv2.CAP_PROP_CONTRAST, current_contrast)
            print(f"Contrast: {current_contrast}")
        elif key == ord('s'):
            # Save current settings
            settings = {
                'auto_exposure': auto_exposure,
                'exposure_time': cap.get(cv2.CAP_PROP_EXPOSURE),
                'brightness': cap.get(cv2.CAP_PROP_BRIGHTNESS),
                'gain': cap.get(cv2.CAP_PROP_GAIN),
                'contrast': cap.get(cv2.CAP_PROP_CONTRAST)
            }
            print("\n=== Current Settings ===")
            for key, value in settings.items():
                print(f"{key}: {value}")
            print("\nUse these values in your launch file:")
            print(f"auto_exposure:={str(auto_exposure).lower()}")
            print(f"exposure_time:={settings['exposure_time']:.0f}")
            print(f"brightness:={settings['brightness']:.0f}")
            print(f"gain:={settings['gain']:.0f}")
            print(f"contrast:={settings['contrast']:.0f}")
    
    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='HF867 Camera Exposure Test')
    parser.add_argument('--camera', type=int, default=0, help='Camera device number')
    parser.add_argument('--width', type=int, default=640, help='Frame width')
    parser.add_argument('--height', type=int, default=480, help='Frame height')
    
    args = parser.parse_args()
    
    test_camera_exposure(args.camera, args.width, args.height)

if __name__ == "__main__":
    main() 