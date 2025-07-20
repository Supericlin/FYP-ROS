# Fall Detection Sensitivity Guide

## 🎯 Overview

The fall detection system uses two main parameters to control sensitivity:

1. **`fall_threshold`** - Aspect ratio threshold (0.3-1.0)
2. **`confidence_threshold`** - Person detection confidence (0.1-0.9)

## 📊 Sensitivity Parameters

### Fall Threshold (Aspect Ratio)
- **Range**: 0.3 - 1.0
- **How it works**: `height / width` of detected person
- **Lower values** = More sensitive (triggers on less flat poses)
- **Higher values** = Less sensitive (only triggers on very flat poses)

| Value | Sensitivity | Description |
|-------|-------------|-------------|
| 0.3-0.5 | 🔴 Very High | Triggers on slight tilts |
| 0.6-0.7 | 🟡 High | Triggers on moderate tilts |
| 0.8-0.9 | 🟢 Low | Only triggers on clear falls |
| 0.9-1.0 | 🔵 Very Low | Only triggers on obvious falls |

### Confidence Threshold
- **Range**: 0.1 - 0.9
- **How it works**: Minimum confidence for person detection
- **Lower values** = More sensitive (detects people more easily)
- **Higher values** = Less sensitive (requires higher confidence)

| Value | Sensitivity | Description |
|-------|-------------|-------------|
| 0.1-0.2 | 🔴 Very High | Detects people with low confidence |
| 0.3-0.4 | 🟡 High | Balanced person detection |
| 0.5-0.7 | 🟢 Low | Requires clear person detection |
| 0.8-0.9 | 🔵 Very Low | Requires very clear person detection |

## 🚀 Quick Setup

### Option 1: Use Preset Script (Recommended)
```bash
cd OrangePi/FallDetect2_ws
python3 fall_detection_presets.py
```

### Option 2: Manual Launch Commands

#### Less Sensitive (Recommended for reducing false positives)
```bash
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.8 confidence_threshold:=0.3
```

#### Even Less Sensitive
```bash
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.9 confidence_threshold:=0.4
```

#### Very Conservative
```bash
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.95 confidence_threshold:=0.5
```

#### More Sensitive (if you want to detect subtle falls)
```bash
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.6 confidence_threshold:=0.2
```

## 🎯 Recommended Settings

### For Reducing False Positives (Current Issue)
```bash
# Less sensitive - fewer false alarms
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.8 confidence_threshold:=0.3
```

### For Different Use Cases

| Use Case | Fall Threshold | Confidence Threshold | Command |
|----------|----------------|---------------------|---------|
| **Home Monitoring** | 0.8 | 0.3 | `fall_threshold:=0.8 confidence_threshold:=0.3` |
| **Hospital/Elderly Care** | 0.7 | 0.25 | `fall_threshold:=0.7 confidence_threshold:=0.25` |
| **Industrial Safety** | 0.9 | 0.4 | `fall_threshold:=0.9 confidence_threshold:=0.4` |
| **Research/Testing** | 0.6 | 0.2 | `fall_threshold:=0.6 confidence_threshold:=0.2` |

## 🔧 Fine-Tuning

### If Still Too Sensitive
1. **Increase fall_threshold** by 0.1 increments
2. **Increase confidence_threshold** by 0.1 increments

```bash
# Example: Make it even less sensitive
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.85 confidence_threshold:=0.35
```

### If Not Sensitive Enough
1. **Decrease fall_threshold** by 0.1 increments
2. **Decrease confidence_threshold** by 0.1 increments

```bash
# Example: Make it more sensitive
ros2 launch fall_detection_pkg hf867_optimized_launch.py fall_threshold:=0.75 confidence_threshold:=0.25
```

## 📈 Testing Your Settings

1. **Enable debug mode** to see detection details:
```bash
ros2 launch fall_detection_pkg hf867_optimized_launch.py debug_mode:=true
```

2. **Watch the logs** for:
   - `Person detected - Score: X.XXX, Aspect ratio: X.XXX`
   - `Fall detected by aspect ratio: X.XXX < Y.YYY`

3. **Test different poses**:
   - Standing normally (should NOT trigger)
   - Sitting (should NOT trigger)
   - Lying down (should trigger)
   - Bending over (may trigger depending on sensitivity)

## 🎯 Troubleshooting

### Too Many False Positives
- Increase `fall_threshold` (try 0.8, 0.85, 0.9)
- Increase `confidence_threshold` (try 0.3, 0.4, 0.5)

### Missing Real Falls
- Decrease `fall_threshold` (try 0.7, 0.6, 0.5)
- Decrease `confidence_threshold` (try 0.25, 0.2, 0.15)

### Person Detection Issues
- Adjust `confidence_threshold` only
- Keep `fall_threshold` at 0.7-0.8

## 📝 Example Commands

```bash
# Build and source
cd OrangePi/FallDetect2_ws
colcon build --packages-select fall_detection_pkg
source install/setup.bash

# Test different sensitivities
python3 fall_detection_presets.py 3  # Low sensitivity
python3 fall_detection_presets.py 4  # Very low sensitivity
python3 fall_detection_presets.py 5  # Custom settings
```

## 🔍 Understanding the Detection

The system detects falls by:
1. **Detecting a person** (confidence > confidence_threshold)
2. **Calculating aspect ratio** (height/width)
3. **Checking if ratio < fall_threshold** (person is "flat")
4. **Optional**: Checking if person is near bottom of frame

A **fall_threshold of 0.8** means the person must be very flat (height < 80% of width) to trigger a fall detection. 