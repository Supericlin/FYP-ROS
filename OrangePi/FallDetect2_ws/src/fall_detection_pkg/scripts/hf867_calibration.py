#!/usr/bin/env python3
"""
HF867 2.8mm 120° Wide-Angle Camera Calibration Script
Optimized specifically for this camera module
"""

import cv2
import numpy as np
import os
import glob
import argparse
from pathlib import Path

def calibrate_hf867_camera(images_path, output_path, chessboard_size=(9, 6), square_size=0.025):
    """
    Calibrate HF867 camera using chessboard pattern
    Optimized for 120° wide-angle lens
    """
    
    # Prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []
    imgpoints = []
    
    # Find calibration images
    images = glob.glob(os.path.join(images_path, '*.jpg')) + glob.glob(os.path.join(images_path, '*.png'))
    
    if not images:
        print(f"No images found in {images_path}")
        return False
    
    print(f"Found {len(images)} calibration images for HF867")
    
    # Process each image
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"Failed to load image: {fname}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners (more lenient for wide-angle)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, 
                                                cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                cv2.CALIB_CB_FAST_CHECK + 
                                                cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            print(f"Processing image {i+1}/{len(images)}: {os.path.basename(fname)}")
            
            # Refine corner detection
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners2)
            
            # Draw and display corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('HF867 Calibration', img)
            cv2.waitKey(500)
        else:
            print(f"No chessboard found in: {fname}")
    
    cv2.destroyAllWindows()
    
    if len(objpoints) < 5:
        print("Not enough valid calibration images. Need at least 5.")
        return False
    
    print(f"Using {len(objpoints)} images for HF867 calibration")
    
    # Calibrate camera with HF867 specific flags
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None,
        flags=cv2.CALIB_RATIONAL_MODEL  # Better for wide-angle lenses
    )
    
    if ret:
        print("HF867 calibration successful!")
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients:\n{dist_coeffs}")
        
        # Calculate reprojection error
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(f"Total reprojection error: {mean_error/len(objpoints)}")
        
        # Save calibration data
        os.makedirs(output_path, exist_ok=True)
        np.save(os.path.join(output_path, 'hf867_camera_matrix.npy'), camera_matrix)
        np.save(os.path.join(output_path, 'hf867_dist_coeffs.npy'), dist_coeffs)
        
        print(f"HF867 calibration data saved to: {output_path}")
        return True
    else:
        print("HF867 calibration failed!")
        return False

def capture_hf867_calibration_images(output_path, camera_device=0, num_images=25):
    """
    Capture calibration images for HF867 camera
    Optimized for 120° wide-angle lens
    """
    cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
    
    # HF867 optimized settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    
    os.makedirs(output_path, exist_ok=True)
    
    print("=== HF867 Camera Calibration ===")
    print("Hold a chessboard pattern in front of the camera")
    print("For 120° wide-angle lens, try different angles and distances")
    print("Press 'c' to capture image, 'q' to quit")
    print(f"Target: {num_images} images")
    print("Tip: Capture images at different distances and angles")
    
    captured = 0
    
    while captured < num_images:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Add instructions
        cv2.putText(frame, f"Captured: {captured}/{num_images}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "Hold chessboard at different angles", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, "Press 'c' to capture, 'q' to quit", (10, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow('HF867 Calibration Capture', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            filename = os.path.join(output_path, f'hf867_calibration_{captured:02d}.jpg')
            cv2.imwrite(filename, frame)
            captured += 1
            print(f"Captured image {captured}: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"Captured {captured} HF867 calibration images")

def test_hf867_calibration(camera_matrix_path, dist_coeffs_path, camera_device=0):
    """
    Test HF867 calibration with side-by-side comparison
    """
    camera_matrix = np.load(camera_matrix_path)
    dist_coeffs = np.load(dist_coeffs_path)
    
    cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("HF867 Calibration Test")
    print("Left: Original (distorted), Right: Undistorted")
    print("Press 'q' to quit, 's' to save test image")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Undistort frame
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # Create side-by-side comparison
        combined = np.hstack((frame, undistorted))
        cv2.putText(combined, "HF867 Original (120°)", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "HF867 Undistorted", (frame.shape[1] + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('HF867 Calibration Test', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('hf867_calibration_test.jpg', combined)
            print("HF867 test image saved as 'hf867_calibration_test.jpg'")
    
    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='HF867 2.8mm 120° Camera Calibration')
    parser.add_argument('--mode', choices=['capture', 'calibrate', 'test'], required=True,
                       help='Mode: capture images, calibrate camera, or test calibration')
    parser.add_argument('--images', default='hf867_calibration_images',
                       help='Path to calibration images directory')
    parser.add_argument('--output', default='calibration_data',
                       help='Path to save calibration data')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera device number')
    parser.add_argument('--chessboard', nargs=2, type=int, default=[9, 6],
                       help='Chessboard internal corners (width height)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='Square size in meters')
    parser.add_argument('--num-images', type=int, default=25,
                       help='Number of calibration images to capture')
    
    args = parser.parse_args()
    
    if args.mode == 'capture':
        capture_hf867_calibration_images(args.images, args.camera, args.num_images)
    elif args.mode == 'calibrate':
        calibrate_hf867_camera(args.images, args.output, tuple(args.chessboard), args.square_size)
    elif args.mode == 'test':
        camera_matrix_path = os.path.join(args.output, 'hf867_camera_matrix.npy')
        dist_coeffs_path = os.path.join(args.output, 'hf867_dist_coeffs.npy')
        if os.path.exists(camera_matrix_path) and os.path.exists(dist_coeffs_path):
            test_hf867_calibration(camera_matrix_path, dist_coeffs_path, args.camera)
        else:
            print("HF867 calibration files not found. Run calibration first.")

if __name__ == "__main__":
    main() 