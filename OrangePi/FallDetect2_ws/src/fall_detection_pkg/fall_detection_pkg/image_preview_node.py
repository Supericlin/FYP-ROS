#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import signal
import sys

class ImagePreviewNode(Node):
    def __init__(self):
        super().__init__('image_preview')
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Declare parameters
        self.declare_parameter('window_name', 'Fall Detection Preview')
        self.declare_parameter('window_width', 320)
        self.declare_parameter('window_height', 240)
        self.declare_parameter('show_fps', True)
        self.declare_parameter('show_timestamp', True)
        self.declare_parameter('enable_display', True)
        
        # Get parameters
        self.window_name = self.get_parameter('window_name').value
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.show_fps = self.get_parameter('show_fps').value
        self.show_timestamp = self.get_parameter('show_timestamp').value
        self.enable_display = self.get_parameter('enable_display').value
        
        # Initialize CV bridge
        self.br = CvBridge()
        
        # Check if display is available
        self.display_available = False
        if self.enable_display:
            try:
                # Check if we have a display
                display = os.environ.get('DISPLAY', '')
                if display:
                    # Create window
                    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
                    self.display_available = True
                    self.get_logger().info(f'Image preview initialized: {self.window_name}')
                    self.get_logger().info('Press "q" to quit, "s" to save image')
                else:
                    self.get_logger().warn('No display available, running in headless mode')
            except Exception as e:
                self.get_logger().warn(f'Display initialization failed: {str(e)}, running in headless mode')
        
        # FPS tracking
        self.frame_count = 0
        self.last_fps_time = self.get_clock().now().nanoseconds
        self.current_fps = 0.0
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        
        if not self.display_available:
            self.get_logger().info('Image preview running in headless mode - no display window')
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info(f'Received signal {signum}, shutting down gracefully...')
        self.cleanup_and_exit()
    
    def cleanup_and_exit(self):
        """Clean up resources and exit"""
        try:
            if self.display_available:
                self.get_logger().info('Closing preview window...')
                cv2.destroyAllWindows()
                cv2.waitKey(1)  # Ensure windows are closed
                self.get_logger().info('Preview window closed successfully')
        except Exception as e:
            self.get_logger().warn(f'Error closing preview window: {str(e)}')
        
        # Force exit if needed
        sys.exit(0)
        
    def calculate_fps(self):
        """Calculate current FPS"""
        self.frame_count += 1
        current_time = self.get_clock().now().nanoseconds
        
        if current_time - self.last_fps_time >= 1_000_000_000:  # 1 second in nanoseconds
            self.current_fps = self.frame_count
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def image_callback(self, msg):
        try:
            # Convert ROS message to OpenCV image
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Calculate FPS
            if self.show_fps:
                self.calculate_fps()
            
            # Create display frame
            display_frame = frame.copy()
            
            # Add FPS overlay
            if self.show_fps:
                cv2.putText(display_frame, f'FPS: {self.current_fps}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add timestamp overlay
            if self.show_timestamp:
                timestamp = self.get_clock().now().to_msg()
                time_str = f'{timestamp.sec}.{timestamp.nanosec//1000000:03d}'
                cv2.putText(display_frame, f'Time: {time_str}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add resolution info
            cv2.putText(display_frame, f'Res: {frame.shape[1]}x{frame.shape[0]}', 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display the frame if display is available
            if self.display_available:
                cv2.imshow(self.window_name, display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Quit requested by user')
                    rclpy.shutdown()
                elif key == ord('s'):
                    # Save current frame
                    timestamp = self.get_clock().now().to_msg()
                    filename = f'fall_detection_preview_{timestamp.sec}_{timestamp.nanosec//1000000:03d}.jpg'
                    cv2.imwrite(filename, frame)
                    self.get_logger().info(f'Image saved: {filename}')
                elif key == ord('h'):
                    # Show help
                    self.show_help()
            else:
                # Log FPS periodically in headless mode
                if self.frame_count % 30 == 0:  # Every 30 frames
                    self.get_logger().info(f'Headless mode - FPS: {self.current_fps}, Res: {frame.shape[1]}x{frame.shape[0]}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def show_help(self):
        """Show help information"""
        help_text = [
            "=== Image Preview Controls ===",
            "q - Quit application",
            "s - Save current frame",
            "h - Show this help",
            "Mouse - Resize window",
            "========================"
        ]
        for line in help_text:
            self.get_logger().info(line)
    
    def destroy_node(self):
        """Clean up resources"""
        self.cleanup_and_exit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePreviewNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal, shutting down...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.get_logger().info('Cleaning up resources...')
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Image preview node shutdown complete')

if __name__ == '__main__':
    main() 