#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

def main():
    rclpy.init()
    
    # Create a simple preview node
    node = Node('simple_preview')
    bridge = CvBridge()
    
    # Create window
    window_name = 'Camera Preview'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)
    
    def image_callback(msg):
        try:
            # Convert ROS message to OpenCV image
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the frame
            cv2.imshow(window_name, frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit requested")
                rclpy.shutdown()
                
        except Exception as e:
            node.get_logger().error(f'Error: {str(e)}')
    
    # Subscribe to camera feed
    subscription = node.create_subscription(
        Image,
        'camera/image',
        image_callback,
        10
    )
    
    print("Camera preview started. Press 'q' to quit.")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 