# Models Directory

This directory should contain the TensorFlow Lite model files for fall detection.

## Required Files:
- `mobilenet_ssd.tflite` - MobileNet SSD model for person detection

## How to add models:
1. Copy your TFLite model file to this directory
2. Ensure the model path in launch files points to the correct file
3. The model should be trained for person detection with COCO dataset classes

## Example:
```bash
cp /path/to/your/mobilenet_ssd.tflite src/fall_detection_pkg/models/
``` 