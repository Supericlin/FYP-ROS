#!/usr/bin/env python3

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

def set_camera_control(control, value):
    """Set a camera control"""
    try:
        result = subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c', f'{control}={value}'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ Set {control} to {value}")
            return True
        else:
            print(f"✗ Failed to set {control} to {value}: {result.stderr}")
            return False
    except Exception as e:
        print(f"Error setting {control}: {e}")
        return False

def show_current_settings():
    """Show current camera settings"""
    controls = [
        'exposure_time_absolute',
        'gain',
        'brightness',
        'contrast',
        'saturation',
        'white_balance_automatic',
        'gamma',
        'sharpness',
        'backlight_compensation'
    ]
    
    print("\n=== Current Camera Settings ===")
    for control in controls:
        value = get_camera_control(control)
        if value:
            print(f"  {control}: {value}")
        else:
            print(f"  {control}: Not available")

def interactive_tuning():
    """Interactive command-line camera tuning"""
    print("Interactive Camera Tuning")
    print("Commands:")
    print("  show - Show current settings")
    print("  exp <value> - Set exposure (1-1000)")
    print("  gain <value> - Set gain (0-100)")
    print("  bright <value> - Set brightness (0-100)")
    print("  contrast <value> - Set contrast (0-100)")
    print("  sat <value> - Set saturation (0-100)")
    print("  save - Save current settings")
    print("  reset - Reset to defaults")
    print("  quit - Exit")
    
    while True:
        try:
            command = input("\nEnter command: ").strip().split()
            if not command:
                continue
                
            cmd = command[0].lower()
            
            if cmd == 'quit' or cmd == 'q':
                break
            elif cmd == 'show':
                show_current_settings()
            elif cmd == 'save':
                save_settings()
            elif cmd == 'reset':
                reset_to_defaults()
            elif cmd == 'exp' and len(command) > 1:
                set_camera_control('exposure_time_absolute', command[1])
            elif cmd == 'gain' and len(command) > 1:
                set_camera_control('gain', command[1])
            elif cmd == 'bright' and len(command) > 1:
                set_camera_control('brightness', command[1])
            elif cmd == 'contrast' and len(command) > 1:
                set_camera_control('contrast', command[1])
            elif cmd == 'sat' and len(command) > 1:
                set_camera_control('saturation', command[1])
            else:
                print("Invalid command. Type 'show' for current settings.")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def save_settings():
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
        if value:
            print(f"  {key}: {value}")
    
    # Save to file
    with open('optimal_camera_settings.txt', 'w') as f:
        for key, value in settings.items():
            if value:
                f.write(f"{key}={value}\n")
    
    print("Settings saved to optimal_camera_settings.txt")
    
    # Also show launch file format
    print("\n=== For Launch File ===")
    print("Copy these values to your launch file:")
    for key, value in settings.items():
        if value:
            if key == 'exposure_time_absolute':
                print(f"  'exposure_time': '{value}',")
            elif key == 'gain':
                print(f"  'gain': '{value}',")
            elif key == 'brightness':
                print(f"  'brightness': '{value}',")
            elif key == 'contrast':
                print(f"  'contrast': '{value}',")
            elif key == 'saturation':
                print(f"  'saturation': '{value}',")

def reset_to_defaults():
    """Reset to default settings"""
    print("Resetting to default settings...")
    defaults = [
        ('exposure_time_absolute', '30'),
        ('gain', '0'),
        ('brightness', '25'),
        ('contrast', '85'),
        ('saturation', '65'),
        ('white_balance_automatic', '1'),
        ('gamma', '120'),
        ('sharpness', '50'),
        ('backlight_compensation', '0'),
    ]
    
    for control, value in defaults:
        set_camera_control(control, value)
        time.sleep(0.1)

def apply_very_low_exposure():
    """Apply very low exposure settings for overexposure issues"""
    print("Applying very low exposure settings...")
    settings = [
        ('exposure_time_absolute', '10'),  # Very low exposure
        ('gain', '0'),                     # No gain
        ('brightness', '20'),              # Very low brightness
        ('contrast', '90'),                # High contrast
        ('saturation', '70'),              # High saturation
        ('white_balance_automatic', '1'),  # Auto white balance
        ('gamma', '130'),                  # Higher gamma
        ('sharpness', '60'),               # Higher sharpness
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
        elif command == "show":
            show_current_settings()
        elif command == "reset":
            reset_to_defaults()
        elif command == "very-low":
            apply_very_low_exposure()
        elif command == "interactive":
            interactive_tuning()
        else:
            print("Usage: python3 cli_camera_tuner.py [list|show|reset|very-low|interactive]")
    else:
        print("CLI Camera Tuning Tool for HF867")
        print("Usage:")
        print("  python3 cli_camera_tuner.py list        - List all camera controls")
        print("  python3 cli_camera_tuner.py show        - Show current settings")
        print("  python3 cli_camera_tuner.py reset       - Reset to default settings")
        print("  python3 cli_camera_tuner.py very-low    - Apply very low exposure settings")
        print("  python3 cli_camera_tuner.py interactive - Interactive tuning mode") 