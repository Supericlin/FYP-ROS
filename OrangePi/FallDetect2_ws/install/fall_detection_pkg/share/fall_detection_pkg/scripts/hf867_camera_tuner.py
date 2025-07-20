#!/usr/bin/env python3

import cv2
import numpy as np
import subprocess
import time
import sys

class HF867CameraTuner:
    def __init__(self, device=0):
        self.device = device
        self.cap = None
        self.current_settings = {
            'exposure_time': 100,
            'gain': 0,
            'brightness': 30,
            'contrast': 60,
            'saturation': 70,
            'hue': 0
        }
        
    def initialize_camera(self):
        """Initialize HF867 camera with V4L2"""
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            print(f"❌ Failed to open camera device {self.device}")
            return False
        
        # Set basic properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        
        # Set manual exposure
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        
        print("✅ HF867 camera initialized")
        return True
    
    def set_v4l2_control(self, control, value):
        """Set V4L2 control using v4l2-ctl"""
        try:
            device = f'/dev/video{self.device}'
            result = subprocess.run(['v4l2-ctl', '-d', device, '-c', f'{control}={value}'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                return True
            else:
                print(f"⚠️ Failed to set {control}={value}: {result.stderr}")
                return False
        except Exception as e:
            print(f"⚠️ V4L2 control failed: {str(e)}")
            return False
    
    def apply_settings(self):
        """Apply current settings to camera"""
        print(f"\n🔧 Applying settings:")
        print(f"   Exposure: {self.current_settings['exposure_time']}")
        print(f"   Gain: {self.current_settings['gain']}")
        print(f"   Brightness: {self.current_settings['brightness']}")
        print(f"   Contrast: {self.current_settings['contrast']}")
        print(f"   Saturation: {self.current_settings['saturation']}")
        
        # Set OpenCV properties
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.current_settings['exposure_time'])
        self.cap.set(cv2.CAP_PROP_GAIN, self.current_settings['gain'])
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.current_settings['brightness'])
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.current_settings['contrast'])
        self.cap.set(cv2.CAP_PROP_SATURATION, self.current_settings['saturation'])
        self.cap.set(cv2.CAP_PROP_HUE, self.current_settings['hue'])
        
        # Set V4L2 controls
        self.set_v4l2_control('exposure_time_absolute', self.current_settings['exposure_time'])
        self.set_v4l2_control('gain', self.current_settings['gain'])
        self.set_v4l2_control('brightness', self.current_settings['brightness'])
        self.set_v4l2_control('contrast', self.current_settings['contrast'])
        self.set_v4l2_control('saturation', self.current_settings['saturation'])
        
        time.sleep(0.5)  # Allow settings to take effect
    
    def show_preview(self):
        """Show camera preview with current settings"""
        cv2.namedWindow('HF867 Camera Tuner', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('HF867 Camera Tuner', 800, 600)
        
        print("\n🎮 Controls:")
        print("   E/D - Increase/Decrease Exposure")
        print("   G/F - Increase/Decrease Gain")
        print("   B/V - Increase/Decrease Brightness")
        print("   C/X - Increase/Decrease Contrast")
        print("   S/A - Increase/Decrease Saturation")
        print("   R - Reset to defaults")
        print("   Q - Quit and save settings")
        print("   SPACE - Save current frame")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Failed to read frame")
                break
            
            # Add settings overlay
            overlay = frame.copy()
            cv2.putText(overlay, f"Exposure: {self.current_settings['exposure_time']}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Gain: {self.current_settings['gain']}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Brightness: {self.current_settings['brightness']}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Contrast: {self.current_settings['contrast']}", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, f"Saturation: {self.current_settings['saturation']}", 
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('HF867 Camera Tuner', overlay)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('e'):
                self.current_settings['exposure_time'] = max(1, self.current_settings['exposure_time'] - 10)
                self.apply_settings()
            elif key == ord('d'):
                self.current_settings['exposure_time'] = min(1000, self.current_settings['exposure_time'] + 10)
                self.apply_settings()
            elif key == ord('g'):
                self.current_settings['gain'] = max(0, self.current_settings['gain'] - 1)
                self.apply_settings()
            elif key == ord('f'):
                self.current_settings['gain'] = min(100, self.current_settings['gain'] + 1)
                self.apply_settings()
            elif key == ord('b'):
                self.current_settings['brightness'] = max(0, self.current_settings['brightness'] - 5)
                self.apply_settings()
            elif key == ord('v'):
                self.current_settings['brightness'] = min(100, self.current_settings['brightness'] + 5)
                self.apply_settings()
            elif key == ord('c'):
                self.current_settings['contrast'] = max(0, self.current_settings['contrast'] - 5)
                self.apply_settings()
            elif key == ord('x'):
                self.current_settings['contrast'] = min(100, self.current_settings['contrast'] + 5)
                self.apply_settings()
            elif key == ord('s'):
                self.current_settings['saturation'] = max(0, self.current_settings['saturation'] - 5)
                self.apply_settings()
            elif key == ord('a'):
                self.current_settings['saturation'] = min(100, self.current_settings['saturation'] + 5)
                self.apply_settings()
            elif key == ord('r'):
                self.current_settings = {
                    'exposure_time': 100,
                    'gain': 0,
                    'brightness': 30,
                    'contrast': 60,
                    'saturation': 70,
                    'hue': 0
                }
                self.apply_settings()
            elif key == ord(' '):  # Spacebar
                timestamp = int(time.time())
                filename = f'hf867_tuned_{timestamp}.jpg'
                cv2.imwrite(filename, frame)
                print(f"💾 Saved: {filename}")
        
        cv2.destroyAllWindows()
    
    def save_settings(self):
        """Save current settings to a file"""
        timestamp = int(time.time())
        filename = f'hf867_settings_{timestamp}.txt'
        
        with open(filename, 'w') as f:
            f.write("# HF867 Camera Settings\n")
            f.write(f"# Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"exposure_time: {self.current_settings['exposure_time']}\n")
            f.write(f"gain: {self.current_settings['gain']}\n")
            f.write(f"brightness: {self.current_settings['brightness']}\n")
            f.write(f"contrast: {self.current_settings['contrast']}\n")
            f.write(f"saturation: {self.current_settings['saturation']}\n")
            f.write(f"hue: {self.current_settings['hue']}\n")
        
        print(f"💾 Settings saved to: {filename}")
        
        # Also generate ROS2 launch parameters
        launch_params = f"""
# ROS2 Launch Parameters for HF867
ros2 launch fall_detection_pkg hf867_optimized_launch.py \\
    exposure_time:={self.current_settings['exposure_time']} \\
    gain:={self.current_settings['gain']} \\
    brightness:={self.current_settings['brightness']} \\
    contrast:={self.current_settings['contrast']} \\
    saturation:={self.current_settings['saturation']}
"""
        
        launch_filename = f'hf867_launch_command_{timestamp}.txt'
        with open(launch_filename, 'w') as f:
            f.write(launch_params)
        
        print(f"🚀 Launch command saved to: {launch_filename}")
    
    def cleanup(self):
        """Clean up camera resources"""
        if self.cap:
            self.cap.release()

def main():
    if len(sys.argv) > 1:
        device = int(sys.argv[1])
    else:
        device = 0
    
    print("🎥 HF867 Camera Tuner")
    print("=====================")
    print(f"Using camera device: /dev/video{device}")
    
    tuner = HF867CameraTuner(device)
    
    if not tuner.initialize_camera():
        print("❌ Failed to initialize camera")
        return
    
    try:
        tuner.apply_settings()
        tuner.show_preview()
        tuner.save_settings()
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
    finally:
        tuner.cleanup()
        print("✅ Camera tuner closed")

if __name__ == '__main__':
    main() 