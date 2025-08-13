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
                
        package_dir = get_package_share_directory('orange_pi_camera')
       
        # Declare parameters     
        self.declare_parameter('model_path', 'models/mobilenet_ssd.tflite')
        self.declare_parameter('mqtt_broker', '192.168.0.224')
        
        model_rel_path = self.get_parameter('model_path').value
        mqtt_broker = self.get_parameter('mqtt_broker').value

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

        self.mqtt_client = mqtt.Client()  # Moved up
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_connected = False
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Verify input dimensions
        if len(self.input_details[0]['shape']) != 4:
            raise ValueError("Invalid model input dimensions")
            
        self.input_height = self.input_details[0]['shape'][1]
        self.input_width = self.input_details[0]['shape'][2]
        
        # Connect to MQTT (only once)
        self._connect_to_mqtt()
        
        # Subscription
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            10
        )
        self.br = CvBridge()
        

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
            boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]    # Shape: [1, N, 4]
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Shape: [1, N]
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]   # Shape: [1, N]
            num_detections = int(self.interpreter.get_tensor(self.output_details[3]['index'])[0])  # Shape: [1]
            
            # Process detections
            for i in range(num_detections):
                if scores[i] > 0.5 and classes[i] == 0:  # Class 0 = person
                    ymin = int(boxes[i][0] * frame.shape[0])
                    xmin = int(boxes[i][1] * frame.shape[1])
                    ymax = int(boxes[i][2] * frame.shape[0])
                    xmax = int(boxes[i][3] * frame.shape[1])
                    
                    # Fall detection logic
                    bbox_height = ymax - ymin
                    bbox_width = xmax - xmin
                    if bbox_width > 0 and (bbox_height / bbox_width) < 0.5:
                        self.get_logger().info("FALL_DETECTED")
                        
                        # Check MQTT connection status before publishing
                        if not self._check_mqtt_connection():
                            self.get_logger().error("MQTT not connected, attempting to reconnect...")
                            self._connect_to_mqtt()
                            time.sleep(0.5)
                        
                        # Publish to userBodyStatus topic
                        try:
                            result = self.mqtt_client.publish("userBodyStatus", "FALL_DETECTED")
                            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                                self.get_logger().info("Successfully published FALL_DETECTED to userBodyStatus")
                            else:
                                self.get_logger().error(f"Failed to publish to userBodyStatus, rc: {result.rc}")
                        except Exception as e:
                            self.get_logger().error(f"Error publishing to userBodyStatus: {str(e)}")
                        
                        # Publish to robot/control topic to trigger navigation stop
                        try:
                            result = self.mqtt_client.publish("robot/control", "navStop")
                            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                                self.get_logger().info("Successfully published navStop to robot/control")
                            else:
                                self.get_logger().error(f"Failed to publish to robot/control, rc: {result.rc}")
                        except Exception as e:
                            self.get_logger().error(f"Error publishing to robot/control: {str(e)}")
    
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")

    def _connect_to_mqtt(self):
        for attempt in range(3):
            try:
                self.get_logger().info(f"Attempting MQTT connection (attempt {attempt + 1}/3)...")
                self.mqtt_client.connect(
                    self.get_parameter('mqtt_broker').value, 
                    1883, 
                    60
                )
                self.mqtt_client.loop_start()
                self.get_logger().info("MQTT connection attempt completed")
                return
            except Exception as e:
                self.get_logger().error(f"MQTT connection failed (attempt {attempt + 1}/3): {str(e)}")
                if attempt < 2:
                    time.sleep(1)
        
        self.get_logger().error("Failed to connect to MQTT broker after 3 attempts")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("Connected to MQTT broker")
            self.mqtt_client.publish("robot/status", "fallDetect online")
        else:
            self.mqtt_connected = False
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn("Disconnected from MQTT broker")
    
    def _check_mqtt_connection(self):
        """Check if MQTT client is properly connected"""
        if not self.mqtt_connected:
            self.get_logger().warning("MQTT connection status: DISCONNECTED")
            return False
        else:
            self.get_logger().debug("MQTT connection status: CONNECTED")
            return True

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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()