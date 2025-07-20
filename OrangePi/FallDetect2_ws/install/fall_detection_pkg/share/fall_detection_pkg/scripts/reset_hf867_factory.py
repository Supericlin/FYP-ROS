#!/usr/bin/env python3

import cv2
import subprocess
import time
import sys

class HF867FactoryReset:
    def __init__(self, device=0):
        self.device = device
        self.cap = None
        
        # Factory default settings for HF867
        self.factory_settings = {
            'exposure_time_absolute': 166,  # Factory default exposure
            'gain': 0,                      # Factory default gain
            'brightness': 50,               # Factory default brightness
            'contrast': 50,                 # Factory default contrast
            'saturation': 50,               # Factory default saturation
            'hue': 0,                       # Factory default hue
            'white_balance_automatic': 1,   # Auto white balance
            'auto_exposure': 3,             # Auto exposure mode
            'focus_automatic_continuous': 0, # Manual focus
        }
        
    def initialize_camera(self):
        """Initialize HF867 camera"""
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            print(f"❌ Failed to open camera device {self.device}")
            return False
        
        print("✅ HF867 camera initialized")
        return True
    
    def reset_v4l2_controls(self):
        """Reset all V4L2 controls to factory defaults"""
        device = f'/dev/video{self.device}'
        
        print("🔄 Resetting V4L2 controls to factory defaults...")
        
        # List of controls to reset
        controls = [
            ('exposure_time_absolute', 166),
            ('gain', 0),
            ('brightness', 50),
            ('contrast', 50),
            ('saturation', 50),
            ('hue', 0),
            ('white_balance_automatic', 1),
            ('auto_exposure', 3),
            ('focus_automatic_continuous', 0),
        ]
        
        success_count = 0
        total_count = len(controls)
        
        for control, value in controls:
            try:
                result = subprocess.run(['v4l2-ctl', '-d', device, '-c', f'{control}={value}'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    print(f"   ✅ {control} = {value}")
                    success_count += 1
                else:
                    print(f"   ⚠️ {control} = {value} (failed: {result.stderr.strip()})")
            except subprocess.TimeoutExpired:
                print(f"   ⚠️ {control} = {value} (timeout)")
            except Exception as e:
                print(f"   ❌ {control} = {value} (error: {str(e)})")
        
        print(f"\n📊 Reset Results: {success_count}/{total_count} controls set successfully")
        return success_count == total_count
    
    def reset_opencv_properties(self):
        """Reset OpenCV camera properties to factory defaults"""
        print("🔄 Resetting OpenCV properties to factory defaults...")
        
        try:
            # Set auto exposure
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto mode
            
            # Set factory default values
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)
            self.cap.set(cv2.CAP_PROP_CONTRAST, 50)
            self.cap.set(cv2.CAP_PROP_SATURATION, 50)
            self.cap.set(cv2.CAP_PROP_HUE, 0)
            self.cap.set(cv2.CAP_PROP_GAIN, 0)
            
            # Set autofocus off (fixed focus for HF867)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            
            # Set white balance to auto
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1.0)
            
            print("   ✅ OpenCV properties reset successfully")
            return True
            
        except Exception as e:
            print(f"   ❌ OpenCV reset failed: {str(e)}")
            return False
    
    def verify_settings(self):
        """Verify current camera settings"""
        print("\n🔍 Verifying current settings...")
        
        try:
            # Get current OpenCV properties
            brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
            contrast = self.cap.get(cv2.CAP_PROP_CONTRAST)
            saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
            hue = self.cap.get(cv2.CAP_PROP_HUE)
            gain = self.cap.get(cv2.CAP_PROP_GAIN)
            exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            
            print(f"   Brightness: {brightness:.1f}")
            print(f"   Contrast: {contrast:.1f}")
            print(f"   Saturation: {saturation:.1f}")
            print(f"   Hue: {hue:.1f}")
            print(f"   Gain: {gain:.1f}")
            print(f"   Exposure: {exposure:.1f}")
            
            # Get V4L2 current values
            device = f'/dev/video{self.device}'
            try:
                result = subprocess.run(['v4l2-ctl', '-d', device, '--list-ctrls'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    print("\n📋 V4L2 Current Values:")
                    for line in result.stdout.split('\n'):
                        if any(control in line for control in ['brightness', 'contrast', 'saturation', 'gain', 'exposure']):
                            print(f"   {line.strip()}")
            except:
                print("   ⚠️ Could not read V4L2 values")
                
        except Exception as e:
            print(f"   ❌ Verification failed: {str(e)}")
    
    def show_preview(self, duration=10):
        """Show camera preview for specified duration"""
        print(f"\n👁️ Showing camera preview for {duration} seconds...")
        print("Press 'q' to quit early")
        
        cv2.namedWindow('HF867 Factory Reset Preview', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('HF867 Factory Reset Preview', 800, 600)
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            ret, frame = self.cap.read()
            if not ret:
                print("❌ Failed to read frame")
                break
            
            # Add overlay
            overlay = frame.copy()
            cv2.putText(overlay, "HF867 Factory Reset", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(overlay, "Press 'q' to quit", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('HF867 Factory Reset Preview', overlay)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        print("✅ Preview completed")
    
    def save_factory_settings(self):
        """Save factory settings to a file"""
        timestamp = int(time.time())
        filename = f'hf867_factory_settings_{timestamp}.txt'
        
        with open(filename, 'w') as f:
            f.write("# HF867 Factory Default Settings\n")
            f.write(f"# Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write("# V4L2 Controls\n")
            for control, value in self.factory_settings.items():
                f.write(f"{control}: {value}\n")
            
            f.write("\n# ROS2 Launch Parameters\n")
            f.write("ros2 launch fall_detection_pkg hf867_optimized_launch.py \\\n")
            f.write("    auto_exposure:=true \\\n")
            f.write("    exposure_time:=166 \\\n")
            f.write("    gain:=0 \\\n")
            f.write("    brightness:=50 \\\n")
            f.write("    contrast:=50 \\\n")
            f.write("    saturation:=50 \\\n")
            f.write("    enable_color_correction:=false \\\n")
            f.write("    enable_wide_angle_correction:=false\n")
        
        print(f"💾 Factory settings saved to: {filename}")
    
    def cleanup(self):
        """Clean up camera resources"""
        if self.cap:
            self.cap.release()

def main():
    if len(sys.argv) > 1:
        device = int(sys.argv[1])
    else:
        device = 0
    
    print("🔄 HF867 Factory Reset Tool")
    print("===========================")
    print(f"Using camera device: /dev/video{device}")
    print("This will reset all camera settings to factory defaults.")
    
    # Ask for confirmation
    response = input("\n⚠️ Are you sure you want to reset to factory settings? (y/N): ")
    if response.lower() != 'y':
        print("❌ Reset cancelled")
        return
    
    reset_tool = HF867FactoryReset(device)
    
    if not reset_tool.initialize_camera():
        print("❌ Failed to initialize camera")
        return
    
    try:
        print("\n🔄 Starting factory reset...")
        
        # Reset V4L2 controls
        v4l2_success = reset_tool.reset_v4l2_controls()
        
        # Reset OpenCV properties
        opencv_success = reset_tool.reset_opencv_properties()
        
        # Wait for settings to take effect
        print("\n⏳ Waiting for settings to take effect...")
        time.sleep(2)
        
        # Verify settings
        reset_tool.verify_settings()
        
        # Save factory settings
        reset_tool.save_factory_settings()
        
        # Show preview
        preview_response = input("\n👁️ Show camera preview? (y/N): ")
        if preview_response.lower() == 'y':
            reset_tool.show_preview()
        
        print("\n✅ Factory reset completed!")
        print("📋 Factory settings have been saved to a file.")
        print("🚀 You can now launch with factory settings using the saved command.")
        
    except KeyboardInterrupt:
        print("\n⏹️ Reset interrupted by user")
    finally:
        reset_tool.cleanup()
        print("✅ Camera reset tool closed")

if __name__ == '__main__':
    main() 