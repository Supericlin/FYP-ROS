#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import threading
import subprocess
import os

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('fps', 15)
        self.declare_parameter('buffer_size', 2)
        self.declare_parameter('enable_auto_reconnect', True)
        self.declare_parameter('reconnect_delay', 2.0)
        self.declare_parameter('max_reconnect_attempts', 5)
        self.declare_parameter('debug_mode', False)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.enable_auto_reconnect = self.get_parameter('enable_auto_reconnect').value
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Initialize variables
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.br = CvBridge()
        self.cap = None
        self.is_running = True
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10
        self.reconnect_attempts = 0
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Camera lock for thread safety
        self.camera_lock = threading.Lock()
        
        # Initialize camera
        if not self._initialize_camera():
            self.get_logger().error("Failed to initialize camera on startup")
            return
        
        # Create timer
        timer_interval = 1.0 / self.fps
        self.timer = self.create_timer(timer_interval, self.timer_callback)
        
        self.get_logger().info(f"Camera node initialized with index={self.camera_index}, "
                             f"resolution={self.frame_width}x{self.frame_height}, fps={self.fps}")

    def _initialize_camera(self):
        """Initialize camera with proper error handling"""
        try:
            # Release existing camera if any
            self._release_camera()
            
            # Try different camera backends
            backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
            
            for backend in backends:
                try:
                    self.get_logger().info(f"Trying camera backend: {backend}")
                    self.cap = cv2.VideoCapture(self.camera_index, backend)
                    
                    if not self.cap.isOpened():
                        self.get_logger().warn(f"Failed to open camera with backend {backend}")
                        continue
                    
                    # Set camera properties in recommended order
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                    
                    # Try different codecs
                    codecs = [
                        cv2.VideoWriter_fourcc('M','J','P','G'),
                        cv2.VideoWriter_fourcc('Y','U','Y','V'),
                        cv2.VideoWriter_fourcc('Y','U','Y','2')
                    ]
                    
                    for codec in codecs:
                        try:
                            self.cap.set(cv2.CAP_PROP_FOURCC, codec)
                            
                            # Test if we can read a frame
                            ret, test_frame = self.cap.read()
                            if ret and test_frame is not None:
                                self.get_logger().info(f"Camera initialized successfully with backend {backend} and codec {codec}")
                                self.consecutive_failures = 0
                                self.reconnect_attempts = 0
                                return True
                            else:
                                self.get_logger().warn(f"Failed to read test frame with codec {codec}")
                        except Exception as e:
                            if self.debug_mode:
                                self.get_logger().warn(f"Codec {codec} failed: {str(e)}")
                            continue
                    
                    # If we get here, no codec worked
                    self.cap.release()
                    self.cap = None
                    
                except Exception as e:
                    self.get_logger().warn(f"Backend {backend} failed: {str(e)}")
                    continue
            
            self.get_logger().error("All camera backends and codecs failed")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {str(e)}")
            return False

    def _release_camera(self):
        """Safely release camera resources"""
        try:
            if self.cap is not None and self.cap.isOpened():
                self.cap.release()
                time.sleep(0.1)  # Small delay to ensure proper release
            self.cap = None
        except Exception as e:
            if self.debug_mode:
                self.get_logger().warn(f"Error releasing camera: {str(e)}")

    def _reset_camera_system(self):
        """Reset camera system using v4l2-ctl"""
        try:
            # Reset camera using v4l2-ctl
            subprocess.run(['v4l2-ctl', '-d', f'/dev/video{self.camera_index}', '--reset'], 
                         capture_output=True, timeout=5)
            time.sleep(1.0)  # Wait for reset to complete
            self.get_logger().info("Camera system reset completed")
            return True
        except Exception as e:
            if self.debug_mode:
                self.get_logger().warn(f"Camera reset failed: {str(e)}")
            return False

    def _attempt_reconnect(self):
        """Attempt to reconnect to camera"""
        if not self.enable_auto_reconnect:
            return False
            
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self.get_logger().error(f"Max reconnection attempts ({self.max_reconnect_attempts}) reached")
            return False
        
        self.reconnect_attempts += 1
        self.get_logger().warn(f"Attempting camera reconnection ({self.reconnect_attempts}/{self.max_reconnect_attempts})")
        
        # Reset camera system first
        self._reset_camera_system()
        
        # Wait before reconnecting
        time.sleep(self.reconnect_delay)
        
        # Try to reinitialize
        if self._initialize_camera():
            self.get_logger().info("Camera reconnection successful")
            return True
        else:
            self.get_logger().warn("Camera reconnection failed")
            return False

    def _calculate_fps(self):
        """Calculate and log actual FPS"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 10.0:  # Log every 10 seconds
            actual_fps = self.frame_count / (current_time - self.last_fps_time)
            self.get_logger().info(f'Camera FPS: {actual_fps:.1f}')
            self.frame_count = 0
            self.last_fps_time = current_time

    def timer_callback(self):
        """Timer callback for frame capture"""
        if not self.is_running or self.cap is None:
            return
        
        try:
            with self.camera_lock:
                ret, frame = self.cap.read()
                
                if ret and frame is not None:
                    # Reset failure counter on success
                    self.consecutive_failures = 0
                    self.last_frame_time = time.time()
                    
                    try:
                        # Publish frame
                        msg = self.br.cv2_to_imgmsg(frame, "bgr8")
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_.publish(msg)
                        
                        # Calculate FPS
                        self._calculate_fps()
                        
                    except Exception as e:
                        self.get_logger().error(f'CVBridge error: {str(e)}')
                        self.consecutive_failures += 1
                else:
                    self.consecutive_failures += 1
                    self.get_logger().warn(f'Failed to capture frame (consecutive failures: {self.consecutive_failures})')
                    
                    # Check if we need to attempt reconnection
                    if self.consecutive_failures >= self.max_consecutive_failures:
                        self.get_logger().error(f"Too many consecutive failures ({self.consecutive_failures}), attempting reconnection")
                        if not self._attempt_reconnect():
                            self.get_logger().error("Reconnection failed, stopping camera node")
                            self.is_running = False
                        else:
                            self.consecutive_failures = 0
                            
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {str(e)}')
            self.consecutive_failures += 1

    def destroy_node(self):
        """Clean shutdown of camera node"""
        self.get_logger().info("Shutting down camera node...")
        self.is_running = False
        
        # Wait a bit for timer to stop
        time.sleep(0.5)
        
        # Release camera
        self._release_camera()
        
        # Call parent destroy
        super().destroy_node()
        self.get_logger().info("Camera node shutdown complete")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error during node destruction: {str(e)}")
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {str(e)}")

if __name__ == '__main__':
    main()