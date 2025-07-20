import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import warnings
import numpy as np
import paho.mqtt.client as mqtt
import cv2
import os
import tflite_runtime.interpreter as tflite
import math
import tensorflow as tf
import time
import absl.logging

warnings.filterwarnings("ignore", category=UserWarning, module='numpy')
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
os.environ["BPLOG_SUPPRESS"] = "1"
tf.get_logger().setLevel('ERROR')
absl.logging.set_verbosity(absl.logging.ERROR)

class FallDetector(Node):
    def __init__(self):
        super().__init__('fall_detector')
        
        # Debug and tuning parameters
        self.debug_mode = False  # Reduced logging by default
        self.fall_detection_count = 0
        self.last_fall_time = 0
        self.fall_cooldown = 3.0  # Reduced cooldown for faster response
        
        # Detection stability parameters
        self.person_detection_history = []  # Track recent detections
        self.history_size = 5  # Number of frames to consider
        self.stable_detection_threshold = 3  # Minimum detections in history to be "stable"
                
        package_dir = get_package_share_directory('fall_detection_pkg')
       
        # Declare parameters     
        self.declare_parameter('model_path', 'models/mobilenet_ssd.tflite')
        self.declare_parameter('mqtt_broker', '192.168.0.224')
        self.declare_parameter('fall_threshold', 0.7)  # Aspect ratio threshold for fall detection
        self.declare_parameter('confidence_threshold', 0.4)  # Minimum confidence for person detection
        self.declare_parameter('debug_mode', False)  # Disabled by default for cleaner logs
        
        model_rel_path = self.get_parameter('model_path').value
        mqtt_broker = self.get_parameter('mqtt_broker').value
        self.fall_threshold = self.get_parameter('fall_threshold').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Validate parameters first
        if not mqtt_broker:
            raise ValueError("MQTT broker IP required")
            
        model_path = os.path.join(package_dir, model_rel_path)
    
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            raise FileNotFoundError("Missing TFLite model")
            
        # Initialize TFLite interpreter
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_connected = False
        self._connect_to_mqtt()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Verify input dimensions
        if len(self.input_details[0]['shape']) != 4:
            raise ValueError("Invalid model input dimensions")
            
        self.input_height = self.input_details[0]['shape'][1]
        self.input_width = self.input_details[0]['shape'][2]
        
        # Log model details for debugging
        if self.debug_mode:
            self.get_logger().info(f"Model input size: {self.input_width}x{self.input_height}")
            self.get_logger().info(f"Fall threshold: {self.fall_threshold}")
            self.get_logger().info(f"Confidence threshold: {self.confidence_threshold}")
        
        # Subscription
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            5  # Reduced queue size for better performance
        )
        self.br = CvBridge()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.last_fps_log = 0
        self.last_person_detected = False
        
        self.get_logger().info("Fall detection system initialized")
        
    def _calculate_fps(self):
        """Calculate and log processing FPS only when it changes significantly"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 15.0:  # Log every 15 seconds
            fps = self.frame_count / (current_time - self.last_fps_time)
            
            # Only log if FPS changed significantly (more than 3 FPS difference)
            if abs(fps - self.last_fps_log) > 3:
                self.get_logger().info(f'Processing FPS: {fps:.1f}')
                self.last_fps_log = fps
            
            self.frame_count = 0
            self.last_fps_time = current_time

    def image_callback(self, msg):
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize and preprocess
            input_img = cv2.resize(frame, (self.input_width, self.input_height))
            input_data = np.expand_dims(input_img, axis=0)
            
            if self.input_details[0]['dtype'] == np.float32:
                input_data = (input_data.astype(np.float32) / 127.5) - 1.0
                
            # Inference
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            self.interpreter.invoke()
            
            # Get outputs
            boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
            num_detections = int(self.interpreter.get_tensor(self.output_details[3]['index'])[0])
            
            # Process detections - only look for persons (class 0)
            person_detected = False
            max_confidence = 0.0
            
            for i in range(num_detections):
                if classes[i] == 0:  # Class 0 = person
                    max_confidence = max(max_confidence, scores[i])
                    if scores[i] > self.confidence_threshold:
                        person_detected = True
                    ymin = int(boxes[i][0] * frame.shape[0])
                    xmin = int(boxes[i][1] * frame.shape[1])
                    ymax = int(boxes[i][2] * frame.shape[0])
                    xmax = int(boxes[i][3] * frame.shape[1])
                    
                    # Calculate bounding box properties
                    bbox_height = ymax - ymin
                    bbox_width = xmax - xmin
                    aspect_ratio = bbox_height / bbox_width if bbox_width > 0 else 0
                    bbox_area = bbox_width * bbox_height
                    frame_area = frame.shape[0] * frame.shape[1]
                    relative_area = bbox_area / frame_area
                    
                    # Debug logging for person detection (only when status changes)
                    if self.debug_mode and not self.last_person_detected:
                        self.get_logger().info(f"Person detected - Score: {scores[i]:.3f}, "
                                             f"Aspect ratio: {aspect_ratio:.3f}")
                    
                    # Simplified fall detection logic
                    current_time = time.time()
                    
                    # Check if enough time has passed since last fall detection
                    if (current_time - self.last_fall_time) < self.fall_cooldown:
                        continue
                    
                    # Fall detection conditions
                    fall_detected = False
                    
                    # Primary condition: Low aspect ratio (person is lying down)
                    if bbox_width > 0 and aspect_ratio < self.fall_threshold:
                        fall_detected = True
                        if self.debug_mode:
                            self.get_logger().info(f"Fall detected by aspect ratio: {aspect_ratio:.3f} < {self.fall_threshold}")
                    
                    # Secondary condition: Person near bottom of frame with low aspect ratio
                    elif (ymax > frame.shape[0] * 0.8 and  # Near bottom 80% of frame
                          aspect_ratio < 0.8):  # Low aspect ratio
                        fall_detected = True
                        if self.debug_mode:
                            self.get_logger().info(f"Fall detected by position: ymax={ymax}, ratio={aspect_ratio:.3f}")
                    
                    if fall_detected:
                        self.fall_detection_count += 1
                        self.last_fall_time = current_time
                        self.get_logger().warn(f"🚨 FALL DETECTED! (Count: {self.fall_detection_count})")
                        self.mqtt_client.publish("robot/status", "FALL_DETECTED")
                        self.mqtt_client.publish("robot/control", "navStop")
                        break  # Only detect one fall per frame
            
            # Update detection history for stability
            self.person_detection_history.append(person_detected)
            if len(self.person_detection_history) > self.history_size:
                self.person_detection_history.pop(0)
            
            # Calculate stable detection
            recent_detections = sum(self.person_detection_history)
            stable_person_detected = recent_detections >= self.stable_detection_threshold
            
            # Log person detection status changes (only when stable)
            if stable_person_detected != self.last_person_detected:
                if stable_person_detected:
                    self.get_logger().info(f"👤 Person detected (stable: {recent_detections}/{self.history_size}, max_conf: {max_confidence:.3f})")
                else:
                    self.get_logger().info(f"❌ No person detected (stable: {recent_detections}/{self.history_size}, max_conf: {max_confidence:.3f})")
                self.last_person_detected = stable_person_detected
            
            # Calculate processing FPS
            self._calculate_fps()
    
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def _connect_to_mqtt(self):
        for _ in range(3):  # Retry 3 times
            try:
                self.mqtt_client.connect(
                    self.get_parameter('mqtt_broker').value, 
                    1883, 
                    60
                )
                self.mqtt_client.loop_start()
                return
            except Exception as e:
                self.get_logger().error(f"MQTT connection failed: {str(e)}")
                time.sleep(1)

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("✅ Connected to MQTT broker")
            self.mqtt_client.publish("robot/status", "fallDetect online")
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"❌ MQTT connection failed with code {rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn("⚠️ Disconnected from MQTT broker")

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except:
            pass
        super().destroy_node()
        self.get_logger().info("Node shut down cleanly")

def main(args=None):
    rclpy.init(args=args)
    node = FallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
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