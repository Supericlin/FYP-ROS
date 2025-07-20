#!/bin/bash

# Cleanup script for OpenCV preview windows
echo "=== Cleaning up OpenCV preview windows ==="

# Close OpenCV windows using Python
echo "Closing OpenCV windows..."
python3 -c "import cv2; cv2.destroyAllWindows(); print('OpenCV windows closed')" 2>/dev/null || echo "No OpenCV windows to close"

# Kill any hanging image preview processes
echo "Checking for hanging image preview processes..."
if pgrep -f "image_preview" > /dev/null; then
    echo "Found hanging processes, killing them..."
    pkill -f "image_preview"
    sleep 1
    if pgrep -f "image_preview" > /dev/null; then
        echo "Force killing remaining processes..."
        pkill -9 -f "image_preview"
    fi
else
    echo "No hanging image preview processes found"
fi

# Kill any other OpenCV-related processes
echo "Checking for other OpenCV processes..."
if pgrep -f "python.*opencv" > /dev/null; then
    echo "Found OpenCV processes, killing them..."
    pkill -f "python.*opencv"
fi

echo "Cleanup complete!"
echo ""
echo "If windows still persist, try:"
echo "1. xkill (click on the window)"
echo "2. Restart your terminal session"
echo "3. Log out and log back in" 