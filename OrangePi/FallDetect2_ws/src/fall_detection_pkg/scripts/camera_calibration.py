#!/usr/bin/env python3
"""
Camera calibration script for wide-angle lens
This script helps you calibrate your camera to correct lens distortion
"""

import cv2
import numpy as np
import os
import glob
import argparse

def calibrate_camera(images_path, output_path, chessboard_size=(9, 6), square_size=0.025):
    """
    Calibrate camera using chessboard pattern
    
    Args:
        images_path: Path to directory containing calibration images
        output_path: Path to save calibration files
        chessboard_size: Number of internal corners (width, height)
        square_size: Size of square in meters
    """
    
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    # Find calibration images
    images = glob.glob(os.path.join(images_path, '*.jpg')) + glob.glob(os.path.join(images_path, '*.png'))
    
    if not images:
        print(f"No images found in {images_path}")
        return False
    
    print(f"Found {len(images)} calibration images")
    
    # Process each image
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"Failed to load image: {fname}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        if ret:
            print(f"Processing image {i+1}/{len(images)}: {os.path.basename(fname)}")
            
            # Refine corner detection
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners2)
            
            # Draw and display corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('Calibration', img)
            cv2.waitKey(500)
        else:
            print(f"No chessboard found in: {fname}")
    
    cv2.destroyAllWindows()
    
    if len(objpoints) < 5:
        print("Not enough valid calibration images. Need at least 5.")
        return False
    
    print(f"Using {len(objpoints)} images for calibration")
    
    # Calibrate camera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    if ret:
        print("Calibration successful!")
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients:\n{dist_coeffs}")
        
        # Save calibration data
        os.makedirs(output_path, exist_ok=True)
        np.save(os.path.join(output_path, 'camera_matrix.npy'), camera_matrix)
        np.save(os.path.join(output_path, 'dist_coeffs.npy'), dist_coeffs)
        
        print(f"Calibration data saved to: {output_path}")
        return True
    else:
        print("Calibration failed!")
        return False

def test_calibration(camera_matrix_path, dist_coeffs_path, camera_device=0):
    """
    Test calibration by showing undistorted video feed
    """
    # Load calibration data
    camera_matrix = np.load(camera_matrix_path)
    dist_coeffs = np.load(dist_coeffs_path)
    
    # Open camera
    cap = cv2.VideoCapture(camera_device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Press 'q' to quit, 's' to save test image")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Undistort frame
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # Create side-by-side comparison
        combined = np.hstack((frame, undistorted))
        cv2.putText(combined, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Undistorted", (frame.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Calibration Test', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('calibration_test.jpg', combined)
            print("Test image saved as 'calibration_test.jpg'")
    
    cap.release()
    cv2.destroyAllWindows()

def capture_calibration_images(output_path, camera_device=0, num_images=20):
    """
    Capture calibration images using chessboard pattern
    """
    cap = cv2.VideoCapture(camera_device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    os.makedirs(output_path, exist_ok=True)
    
    print("Hold a chessboard pattern in front of the camera")
    print("Press 'c' to capture image, 'q' to quit")
    print(f"Target: {num_images} images")
    
    captured = 0
    
    while captured < num_images:
        ret, frame = cap.read()
        if not ret:
            break
        
        cv2.putText(frame, f"Captured: {captured}/{num_images}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Capture Calibration Images', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            filename = os.path.join(output_path, f'calibration_{captured:02d}.jpg')
            cv2.imwrite(filename, frame)
            captured += 1
            print(f"Captured image {captured}: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"Captured {captured} calibration images")

def main():
    parser = argparse.ArgumentParser(description='Camera calibration for wide-angle lens')
    parser.add_argument('--mode', choices=['capture', 'calibrate', 'test'], required=True,
                       help='Mode: capture images, calibrate camera, or test calibration')
    parser.add_argument('--images', default='calibration_images',
                       help='Path to calibration images directory')
    parser.add_argument('--output', default='calibration_data',
                       help='Path to save calibration data')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera device number')
    parser.add_argument('--chessboard', nargs=2, type=int, default=[9, 6],
                       help='Chessboard internal corners (width height)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='Square size in meters')
    parser.add_argument('--num-images', type=int, default=20,
                       help='Number of calibration images to capture')
    
    args = parser.parse_args()
    
    if args.mode == 'capture':
        capture_calibration_images(args.images, args.camera, args.num_images)
    elif args.mode == 'calibrate':
        calibrate_camera(args.images, args.output, tuple(args.chessboard), args.square_size)
    elif args.mode == 'test':
        camera_matrix_path = os.path.join(args.output, 'camera_matrix.npy')
        dist_coeffs_path = os.path.join(args.output, 'dist_coeffs.npy')
        if os.path.exists(camera_matrix_path) and os.path.exists(dist_coeffs_path):
            test_calibration(camera_matrix_path, dist_coeffs_path, args.camera)
        else:
            print("Calibration files not found. Run calibration first.")

if __name__ == "__main__":
    main() 