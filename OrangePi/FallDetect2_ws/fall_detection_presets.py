#!/usr/bin/env python3
"""
Fall Detection Sensitivity Presets
This script provides different sensitivity levels for the fall detection system.
"""

import subprocess
import sys
import os

def run_launch_with_preset(preset_name, fall_threshold, confidence_threshold, description):
    """Run the launch file with specific sensitivity settings"""
    print(f"\n🎯 {preset_name}")
    print(f"   {description}")
    print(f"   Fall Threshold: {fall_threshold} (aspect ratio)")
    print(f"   Confidence Threshold: {confidence_threshold} (person detection)")
    
    # Build the command
    cmd = [
        "ros2", "launch", "fall_detection_pkg", "hf867_optimized_launch.py",
        f"fall_threshold:={fall_threshold}",
        f"confidence_threshold:={confidence_threshold}",
        "debug_mode:=true"
    ]
    
    print(f"\n🚀 Running command:")
    print(f"   {' '.join(cmd)}")
    
    try:
        # Run the launch command
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n⏹️ Launch interrupted by user")
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Error running launch: {e}")
    except FileNotFoundError:
        print("\n❌ Error: ros2 command not found. Make sure ROS2 is installed and sourced.")

def show_presets():
    """Show available sensitivity presets"""
    print("🎯 FALL DETECTION SENSITIVITY PRESETS")
    print("=" * 50)
    print("Choose a sensitivity level:")
    print()
    print("1. 🔴 HIGH SENSITIVITY (Very Sensitive)")
    print("   - Triggers on slight tilts and low confidence")
    print("   - Good for early warning systems")
    print("   - May have false positives")
    print()
    print("2. 🟡 MEDIUM SENSITIVITY (Balanced)")
    print("   - Current default settings")
    print("   - Good balance between detection and accuracy")
    print("   - Recommended for most use cases")
    print()
    print("3. 🟢 LOW SENSITIVITY (Less Sensitive)")
    print("   - Only triggers on clear fall poses")
    print("   - Fewer false positives")
    print("   - May miss subtle falls")
    print()
    print("4. 🔵 VERY LOW SENSITIVITY (Conservative)")
    print("   - Only triggers on obvious falls")
    print("   - Minimal false positives")
    print("   - May miss some falls")
    print()
    print("5. ⚙️  CUSTOM SETTINGS")
    print("   - Set your own fall_threshold and confidence_threshold")
    print()

def get_custom_settings():
    """Get custom sensitivity settings from user"""
    print("⚙️  CUSTOM SENSITIVITY SETTINGS")
    print("=" * 30)
    
    try:
        fall_threshold = float(input("Enter fall threshold (0.3-1.0, default 0.7): ") or "0.7")
        confidence_threshold = float(input("Enter confidence threshold (0.1-0.9, default 0.2): ") or "0.2")
        
        if not (0.3 <= fall_threshold <= 1.0):
            print("❌ Fall threshold must be between 0.3 and 1.0")
            return None, None
            
        if not (0.1 <= confidence_threshold <= 0.9):
            print("❌ Confidence threshold must be between 0.1 and 0.9")
            return None, None
            
        return fall_threshold, confidence_threshold
        
    except ValueError:
        print("❌ Invalid input. Please enter numeric values.")
        return None, None

def main():
    """Main function to handle preset selection"""
    if len(sys.argv) > 1:
        # Command line mode
        preset = sys.argv[1].lower()
    else:
        # Interactive mode
        show_presets()
        preset = input("Enter your choice (1-5): ").strip()
    
    # Define presets
    presets = {
        "1": {
            "name": "HIGH SENSITIVITY",
            "fall_threshold": 0.5,
            "confidence_threshold": 0.15,
            "description": "Very sensitive - triggers on slight tilts and low confidence"
        },
        "2": {
            "name": "MEDIUM SENSITIVITY",
            "fall_threshold": 0.7,
            "confidence_threshold": 0.2,
            "description": "Balanced - current default settings"
        },
        "3": {
            "name": "LOW SENSITIVITY",
            "fall_threshold": 0.8,
            "confidence_threshold": 0.3,
            "description": "Less sensitive - only triggers on clear fall poses"
        },
        "4": {
            "name": "VERY LOW SENSITIVITY",
            "fall_threshold": 0.9,
            "confidence_threshold": 0.4,
            "description": "Conservative - only triggers on obvious falls"
        }
    }
    
    # Handle preset selection
    if preset in presets:
        p = presets[preset]
        run_launch_with_preset(p["name"], p["fall_threshold"], p["confidence_threshold"], p["description"])
    elif preset == "5":
        # Custom settings
        fall_threshold, confidence_threshold = get_custom_settings()
        if fall_threshold is not None and confidence_threshold is not None:
            run_launch_with_preset("CUSTOM", fall_threshold, confidence_threshold, "User-defined sensitivity settings")
    else:
        print("❌ Invalid choice. Please run the script again and select 1-5.")

if __name__ == "__main__":
    main() 