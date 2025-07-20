import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import wiringpi
import time
import sys

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node2')

        # GPIO pin assignments
        self.sensors = [
            {"trig": 19, "echo": 20},
            {"trig": 24, "echo": 25},
            {"trig": 10, "echo": 13},
        ]

        # Initialize WiringOP
        wiringpi.wiringPiSetup()

        # Initialize GPIO pins
        #for sensor in self.sensors:
        #    self.setup_gpio(sensor["trig"], sensor["echo"])
        
        # Initialize GPIO pins and log once
        for i, sensor in enumerate(self.sensors):
          self.setup_gpio(sensor["trig"], sensor["echo"])
          self.get_logger().info(
              f"Sensor {i + 1}: Initialized with TRIG pin {sensor['trig']} and ECHO pin {sensor['echo']}"
          )

        # Publishers for each sensor
        self.sensor_publishers = []
        for i, _ in enumerate(self.sensors):
            topic_name = f'ultrasonic_sensor_{i + 4}'
            publisher = self.create_publisher(Range, topic_name, 10)
            self.sensor_publishers.append(publisher)

        self.timer = self.create_timer(0.05, self.read_sensors)

    def setup_gpio(self, trig, echo):
        wiringpi.pinMode(trig, 1)  # Output
        wiringpi.pinMode(echo, 0)  # Input

    def read_sensors(self):
        for i, sensor in enumerate(self.sensors):
            distance = self.measure_distance(sensor["trig"], sensor["echo"])
            if distance == float('inf'):
                self.get_logger().warn(f"Sensor {i + 4}: No echo received (object out of range)")
            elif distance < 0.02:
                self.get_logger().info(f"Sensor {i + 4}: Object too close, reporting minimum range")
            else:
                self.publish_range_msg(i, distance)
            time.sleep(0.1)  # Add delay to avoid interference

    def measure_distance(self, trig, echo):
        # Send trigger pulse
        wiringpi.digitalWrite(trig, 1)
        time.sleep(0.00001)  # 10 microseconds
        wiringpi.digitalWrite(trig, 0)

        # Wait for echo to go HIGH (start of pulse)
        pulse_start = time.time()
        timeout_start = time.time()
        while wiringpi.digitalRead(echo) == 0:
            if time.time() - timeout_start > 0.03:  # Timeout after 30ms
                return float('inf')

        pulse_start = time.time()

        # Wait for echo to go LOW (end of pulse)
        timeout_end = time.time()
        while wiringpi.digitalRead(echo) == 1:
            if time.time() - timeout_end > 0.03:  # Timeout after 30ms
                return float('inf')

        pulse_end = time.time()

        # Calculate pulse duration
        pulse_duration = pulse_end - pulse_start

        # Log timing for debugging
        #self.get_logger().info(f"Sensor Timing: TRIG={trig}, ECHO={echo}, Pulse Duration={pulse_duration:.6f}")

        # Calculate distance
        distance = (pulse_duration * 34300) / 2  # Speed of sound: 343 m/s
        return distance

    def publish_range_msg(self, sensor_index, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'ultrasonic_sensor_{sensor_index + 4}'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # Approx. 15 degrees in radians
        msg.min_range = 0.02  # 2 cm
        msg.max_range = 4.0   # 4 m

        if distance < msg.min_range:
            msg.range = msg.min_range
        elif distance == float('inf'):
            msg.range = msg.max_range + 1.0  # Out of range
        else:
            msg.range = distance / 100.0  # Convert to meters

        self.sensor_publishers[sensor_index].publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()