# How to Launch with Saved Camera Settings

## Problem
When you run `ros2 launch fall_detection_pkg hf867_system_launch.py` without parameters, it uses the default settings from the launch file, not your saved settings.

## Solution
You must pass your saved settings as parameters to the launch command.

## Step 1: Get Your Saved Settings
When you pressed 's' in the test tool, it showed you settings like:
```
auto_exposure: False
exposure_time: 10.0
brightness: -20.0
gain: 0.0
contrast: 50.0
```

## Step 2: Use the Correct Launch Command Format

### For Manual Exposure (Recommended for overexposure):
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=false \
    exposure_time:=10 \
    brightness:=-20 \
    gain:=0 \
    contrast:=50
```

### For Auto Exposure:
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=true \
    brightness:=-20 \
    gain:=0 \
    contrast:=50
```

## Step 3: Use the Helper Script
```bash
cd ~/FallDetect2_ws
chmod +x apply_saved_settings.sh
./apply_saved_settings.sh
```

This script will ask for your saved settings and create the correct launch command.

## Common Settings for Overexposure Fix

### Most Aggressive (Very Dark):
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=false \
    exposure_time:=1 \
    brightness:=0 \
    gain:=0 \
    contrast:=50
```

### Moderate Fix:
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=false \
    exposure_time:=10 \
    brightness:=-10 \
    gain:=0 \
    contrast:=60
```

### Light Fix:
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=false \
    exposure_time:=20 \
    brightness:=0 \
    gain:=0 \
    contrast:=50
```

## Important Notes

1. **Always use `auto_exposure:=false`** for manual control
2. **Lower exposure_time** = less exposure = darker image
3. **Negative brightness** values are possible and often needed
4. **Zero gain** prevents digital amplification
5. **Higher contrast** helps with image definition

## Troubleshooting

### If still overexposed:
- Decrease `exposure_time` further (try 1, 5, 10)
- Use negative `brightness` values (try -10, -20, -30)
- Keep `gain:=0`

### If too dark:
- Increase `exposure_time` gradually (20, 30, 50)
- Increase `brightness` slightly (0, 10, 20)
- Keep `gain:=0` to avoid noise

## Example with Your Settings
If your saved settings were:
- auto_exposure: False
- exposure_time: 15.0
- brightness: -15.0
- gain: 0.0
- contrast: 55.0

Use this command:
```bash
ros2 launch fall_detection_pkg hf867_system_launch.py \
    auto_exposure:=false \
    exposure_time:=15 \
    brightness:=-15 \
    gain:=0 \
    contrast:=55
``` 