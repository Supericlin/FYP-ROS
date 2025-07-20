# Calibration Data Directory

This directory should contain camera calibration files for the HF867 wide-angle camera.

## Required Files:
- `hf867_camera_matrix.npy` - Camera intrinsic matrix
- `hf867_dist_coeffs.npy` - Distortion coefficients

## How to generate calibration data:
1. Run the calibration script: `python3 scripts/hf867_calibration.py`
2. The script will generate the required .npy files
3. Copy the generated files to this directory

## Example:
```bash
python3 scripts/hf867_calibration.py
cp /path/to/generated/hf867_camera_matrix.npy src/fall_detection_pkg/calibration_data/
cp /path/to/generated/hf867_dist_coeffs.npy src/fall_detection_pkg/calibration_data/
``` 