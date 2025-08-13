#!/usr/bin/env python3
"""
Utility script to close any remaining OpenCV windows.
Run this if preview windows don't close properly after Ctrl+C.
"""

import cv2
import os
import subprocess
import signal

def close_opencv_windows():
    """Close all OpenCV windows"""
    try:
        print("Closing all OpenCV windows...")
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        print("OpenCV windows closed successfully")
    except Exception as e:
        print(f"Error closing OpenCV windows: {e}")

def kill_opencv_processes():
    """Kill any remaining OpenCV-related processes"""
    try:
        # Find processes with 'python' and 'opencv' or 'cv2' in their command line
        result = subprocess.run(['pgrep', '-f', 'python.*opencv'], capture_output=True, text=True)
        if result.stdout:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    print(f"Killing process {pid}")
                    os.kill(int(pid), signal.SIGTERM)
        
        # Also check for any remaining python processes that might be hanging
        result = subprocess.run(['pgrep', '-f', 'image_preview'], capture_output=True, text=True)
        if result.stdout:
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    print(f"Killing image preview process {pid}")
                    os.kill(int(pid), signal.SIGKILL)
                    
    except Exception as e:
        print(f"Error killing processes: {e}")

def main():
    print("=== OpenCV Window Cleanup Utility ===")
    print("This script will close any remaining OpenCV windows")
    print("and kill any hanging image preview processes.")
    print()
    
    # Close OpenCV windows
    close_opencv_windows()
    
    # Kill any hanging processes
    kill_opencv_processes()
    
    print()
    print("Cleanup complete!")
    print("If windows still persist, try:")
    print("1. xkill (click on the window)")
    print("2. pkill -f 'image_preview'")
    print("3. Restart your terminal session")

if __name__ == "__main__":
    main() 