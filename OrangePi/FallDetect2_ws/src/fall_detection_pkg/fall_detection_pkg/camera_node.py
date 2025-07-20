#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import subprocess
import os

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters for HF867 wide-angle camera
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('frame_width', 640)  # Higher resolution for better quality
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_undistortion', True)  # Enable for wide-angle
        self.declare_parameter('camera_matrix_path', '')
        self.declare_parameter('distortion_coeffs_path', '')
        self.declare_parameter('buffer_size', 1)
        
        # HF867 specific exposure and color parameters
        self.declare_parameter('auto_exposure', True)  # Manual exposure for HF867
        self.declare_parameter('exposure_time', 166)  # Low exposure to prevent overexposure
        self.declare_parameter('gain', 0)  # Low gain for better quality
        self.declare_parameter('brightness', 50)  # Lower brightness
        self.declare_parameter('contrast', 50)  # Higher contrast
        self.declare_parameter('saturation', 50)  # Higher saturation for wide-angle
        self.declare_parameter('white_balance', True)
        self.declare_parameter('enable_color_correction', False)
        self.declare_parameter('enable_wide_angle_correction', True)
        
        # Get parameters
        self.camera_device = self.get_parameter('camera_device').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.enable_undistortion = self.get_parameter('enable_undistortion').value
        self.camera_matrix_path = self.get_parameter('camera_matrix_path').value
        self.distortion_coeffs_path = self.get_parameter('distortion_coeffs_path').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.auto_exposure = self.get_parameter('auto_exposure').value
        self.exposure_time = self.get_parameter('exposure_time').value
        self.gain = self.get_parameter('gain').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        self.saturation = self.get_parameter('saturation').value
        self.white_balance = self.get_parameter('white_balance').value
        self.enable_color_correction = self.get_parameter('enable_color_correction').value
        self.enable_wide_angle_correction = self.get_parameter('enable_wide_angle_correction').value
        
        self.publisher_ = self.create_publisher(Image, 'camera/image', 5)
        
        # Initialize camera with V4L2
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.camera_device}')
            raise RuntimeError(f'Camera device {self.camera_device} not available')
        
        # Configure HF867 camera settings
        self._configure_hf867_camera()
        
        # Load camera calibration if available
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.enable_undistortion:
            self._load_camera_calibration()
        
        self.br = CvBridge()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.last_fps_log = 0
        
        # Calculate timer interval based on FPS
        timer_interval = max(0.01, 1.0 / self.fps)
        self.timer = self.create_timer(timer_interval, self.timer_callback)
        
        self.get_logger().info(f'HF867 camera initialized: {self.frame_width}x{self.frame_height} @ {self.fps}fps')

    def _configure_hf867_camera(self):
        """Configure HF867 wide-angle camera with optimal settings"""
        try:
            # Set codec for better performance
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            
            # Set FPS
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Set buffer size
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
            
            # HF867 specific exposure settings
            if not self.auto_exposure:
                # Manual exposure mode - critical for HF867
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
                self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_time)
                self.get_logger().info(f'Manual exposure set to: {self.exposure_time}')
            else:
                # Auto exposure with limits
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto mode
                # Set exposure limits to prevent overexposure
                self.cap.set(cv2.CAP_PROP_EXPOSURE, 200)  # Limit maximum exposure
            
            # Gain settings
            if self.gain >= 0:
                self.cap.set(cv2.CAP_PROP_GAIN, self.gain)
                self.get_logger().info(f'Manual gain set to: {self.gain}')
            else:
                self.cap.set(cv2.CAP_PROP_GAIN, 0)  # Auto gain
            
            # Color settings for wide-angle lens
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
            self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
            
            # White balance for HF867
            if self.white_balance:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 1.0)  # Auto white balance
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)  # Manual white balance
            
            # Additional HF867 settings
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus for fixed focus
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1.0)  # Ensure RGB conversion
            
            # Try to set additional V4L2 controls for HF867
            self._set_v4l2_controls()
            
            # Verify settings
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            actual_gain = self.cap.get(cv2.CAP_PROP_GAIN)
            actual_brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
            actual_saturation = self.cap.get(cv2.CAP_PROP_SATURATION)
            
            self.get_logger().info(f'HF867 configured: {actual_width}x{actual_height} @ {actual_fps}fps')
            self.get_logger().info(f'Exposure: {actual_exposure}, Gain: {actual_gain}')
            self.get_logger().info(f'Brightness: {actual_brightness}, Saturation: {actual_saturation}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure HF867 camera: {str(e)}')
            raise

    def _set_v4l2_controls(self):
        """Set additional V4L2 controls for HF867"""
        try:
            # Use v4l2-ctl to set additional controls
            device = f'/dev/video{self.camera_device}'
            
            # Set exposure time (in microseconds)
            subprocess.run(['v4l2-ctl', '-d', device, '-c', 'exposure_time_absolute=100'], 
                         capture_output=True, text=True)
            
            # Set gain
            subprocess.run(['v4l2-ctl', '-d', device, '-c', f'gain={self.gain}'], 
                         capture_output=True, text=True)
            
            # Set brightness
            subprocess.run(['v4l2-ctl', '-d', device, '-c', f'brightness={self.brightness}'], 
                         capture_output=True, text=True)
            
            # Set contrast
            subprocess.run(['v4l2-ctl', '-d', device, '-c', f'contrast={self.contrast}'], 
                         capture_output=True, text=True)
            
            # Set saturation
            subprocess.run(['v4l2-ctl', '-d', device, '-c', f'saturation={self.saturation}'], 
                         capture_output=True, text=True)
            
            self.get_logger().info('V4L2 controls set successfully')
            
        except Exception as e:
            self.get_logger().warn(f'V4L2 controls failed: {str(e)}')

    def _load_camera_calibration(self):
        """Load camera calibration data for undistortion"""
        try:
            if self.camera_matrix_path and self.distortion_coeffs_path:
                self.camera_matrix = np.load(self.camera_matrix_path)
                self.dist_coeffs = np.load(self.distortion_coeffs_path)
                self.get_logger().info('HF867 calibration loaded')
            else:
                self.enable_undistortion = False
        except Exception as e:
            self.get_logger().error(f'Failed to load HF867 calibration: {str(e)}')
            self.enable_undistortion = False

    def _undistort_frame(self, frame):
        """Apply camera undistortion for wide-angle lens"""
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            try:
                undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
                return undistorted
            except Exception as e:
                return frame
        return frame

    def _apply_wide_angle_correction(self, frame):
        """Apply wide-angle lens correction for HF867"""
        if not self.enable_wide_angle_correction:
            return frame
        
        try:
            # Get frame dimensions
            height, width = frame.shape[:2]
            
            # Create fisheye correction map for 120° FOV
            # This helps reduce the fisheye effect
            center_x, center_y = width // 2, height // 2
            max_radius = min(center_x, center_y)
            
            # Create correction map
            map_x = np.zeros((height, width), dtype=np.float32)
            map_y = np.zeros((height, width), dtype=np.float32)
            
            for y in range(height):
                for x in range(width):
                    # Calculate distance from center
                    dx = x - center_x
                    dy = y - center_y
                    distance = np.sqrt(dx*dx + dy*dy)
                    
                    if distance > 0:
                        # Apply fisheye correction
                        theta = distance / max_radius
                        corrected_distance = max_radius * np.sin(theta)
                        
                        # Calculate corrected position
                        if distance > 0:
                            map_x[y, x] = center_x + (dx / distance) * corrected_distance
                            map_y[y, x] = center_y + (dy / distance) * corrected_distance
                        else:
                            map_x[y, x] = x
                            map_y[y, x] = y
                    else:
                        map_x[y, x] = x
                        map_y[y, x] = y
            
            # Apply the correction
            corrected_frame = cv2.remap(frame, map_x, map_y, cv2.INTER_LINEAR)
            return corrected_frame
            
        except Exception as e:
            self.get_logger().warn(f'Wide-angle correction failed: {str(e)}')
            return frame

    def _apply_color_correction(self, frame):
        """Apply color correction for HF867 wide-angle camera"""
        if not self.enable_color_correction:
            return frame
        
        try:
            # Convert to LAB color space for better color correction
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            
            # Enhance saturation in LAB space
            l, a, b = cv2.split(lab)
            
            # Enhance color channels (a and b) for wide-angle
            a = cv2.multiply(a, 1.3)  # Increase green-red channel
            b = cv2.multiply(b, 1.3)  # Increase blue-yellow channel
            
            # Enhance lightness slightly
            l = cv2.multiply(l, 1.1)
            
            # Merge channels back
            enhanced_lab = cv2.merge([l, a, b])
            
            # Convert back to BGR
            enhanced_frame = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
            
            # Apply additional color enhancement
            enhanced_frame = cv2.convertScaleAbs(enhanced_frame, alpha=1.2, beta=10)
            
            # Apply gamma correction for better contrast
            gamma = 0.8
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            enhanced_frame = cv2.LUT(enhanced_frame, table)
            
            return enhanced_frame
            
        except Exception as e:
            self.get_logger().warn(f'Color correction failed: {str(e)}')
            return frame

    def _calculate_fps(self):
        """Calculate and log FPS only when it changes significantly"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 10.0:
            fps = self.frame_count / (current_time - self.last_fps_time)
            
            if abs(fps - self.last_fps_log) > 5:
                self.get_logger().info(f'HF867 FPS: {fps:.1f}')
                self.last_fps_log = fps
            
            self.frame_count = 0
            self.last_fps_time = current_time

    def timer_callback(self):
        """Timer callback for capturing and publishing frames"""
        try:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Failed to read frame from HF867 camera')
                self._recover_camera()
                return
            
            # Apply wide-angle correction
            if self.enable_wide_angle_correction:
                frame = self._apply_wide_angle_correction(frame)
            
            # Apply undistortion if enabled
            if self.enable_undistortion:
                frame = self._undistort_frame(frame)
            
            # Apply color correction
            if self.enable_color_correction:
                frame = self._apply_color_correction(frame)
            
            # Convert to ROS message and publish
            try:
                msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
                self._calculate_fps()
            except Exception as e:
                self.get_logger().error(f'Failed to publish frame: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'HF867 camera error: {str(e)}')
            self._recover_camera()

    def _recover_camera(self):
        """Attempt to recover HF867 camera connection"""
        try:
            self.get_logger().warn('Attempting HF867 camera recovery...')
            self.cap.release()
            time.sleep(1)
            self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            if self.cap.isOpened():
                self._configure_hf867_camera()
                self.get_logger().info('HF867 camera recovery successful')
            else:
                self.get_logger().error('HF867 camera recovery failed')
        except Exception as e:
            self.get_logger().error(f'HF867 camera recovery error: {str(e)}')

    def destroy_node(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()