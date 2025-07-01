#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import Hobot.GPIO as GPIO
import time
import threading


class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # Sensor GPIO pins
        self.sensors = [
            {'trigger': 29, 'echo': 31},  # Sensor 1
            {'trigger': 37, 'echo': 36},  # Sensor 2
            {'trigger': 18, 'echo': 22},  # Sensor 3
        ]

        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        for sensor in self.sensors:
            GPIO.setup(sensor['trigger'], GPIO.OUT)
            GPIO.setup(sensor['echo'], GPIO.IN)
            GPIO.output(sensor['trigger'], GPIO.LOW)  # Ensure triggers start LOW

        # Create publishers for each sensor
        self.sensor_publishers = []
        for i, _ in enumerate(self.sensors):
            topic_name = f'ultrasonic_sensor_{i + 1}'
            publisher = self.create_publisher(Range, topic_name, 10)
            self.sensor_publishers.append(publisher)

        # Use a lock to prevent overlapping sensor readings
        self.sensor_lock = threading.Lock()
        
        # Create a timer to read sensors
        self.timer = self.create_timer(0.1, self.read_sensors)  # 10 Hz

    def read_sensors(self):
        # Skip if we're already processing
        if not self.sensor_lock.acquire(blocking=False):
            self.get_logger().debug("Skipping sensor read - previous read still in progress")
            return
            
        try:
            for i, sensor in enumerate(self.sensors):
                try:
                    distance = self.get_distance(sensor['trigger'], sensor['echo'])
                    if distance == float('inf'):
                        self.get_logger().debug(f"Sensor {i + 1}: No echo received (object out of range)")
                    elif distance <= 0.02:
                        self.get_logger().debug(f"Sensor {i + 1}: Object too close, reporting minimum range")
                    
                    self.publish_range_msg(i, distance)
                except Exception as e:
                    self.get_logger().error(f"Error reading sensor {i + 1}: {e}")
                    # Publish an error reading
                    self.publish_range_msg(i, float('inf'))
                
                # Small delay between sensors to reduce interference
                time.sleep(0.01)
        finally:
            self.sensor_lock.release()

    def get_distance(self, trigger_pin, echo_pin):
        # Send trigger pulse
        GPIO.output(trigger_pin, GPIO.LOW)
        time.sleep(0.00002)  # Short delay to ensure trigger is LOW
        GPIO.output(trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)  # Trigger pulse (10 microseconds)
        GPIO.output(trigger_pin, GPIO.LOW)

        # Wait for echo with timeout
        pulse_start = time.time()
        timeout = pulse_start + 0.03  # 30ms timeout
        
        # Wait for echo to start (go HIGH)
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return float('inf')  # Timeout waiting for start
                
        # Wait for echo to end (go LOW)
        pulse_end = time.time()
        timeout = pulse_end + 0.03  # 30ms timeout
        
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return float('inf')  # Timeout waiting for end

        # Calculate time elapsed and distance
        pulse_duration = pulse_end - pulse_start
        distance = (pulse_duration * 34300) / 2  # Speed of sound = 34300 cm/s

        return distance

    def publish_range_msg(self, sensor_index, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'ultrasonic_sensor_{sensor_index + 1}'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # Approx. 15 degrees in radians
        msg.min_range = 0.02  # 2 cm
        msg.max_range = 4.0   # 4 m

        if distance < msg.min_range:
            msg.range = msg.min_range  # Report minimum range
        elif distance == float('inf'):
            msg.range = msg.max_range + 1.0  # Out of range
        else:
            msg.range = distance / 100.0  # Convert to meters

        self.sensor_publishers[sensor_index].publish(msg)

    def destroy_node(self):
        # Ensure we clean up GPIO on shutdown
        try:
            GPIO.cleanup()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()