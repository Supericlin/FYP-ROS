import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import wiringpi
import time
import sys
import threading

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node2')

        # GPIO pin assignments
        self.sensors = [
            #{"trig": 19, "echo": 20},  # Sensor 3 (commented out)
            {"trig": 24, "echo": 25},   # Sensor 4
            {"trig": 10, "echo": 13},   # Sensor 5
        ]

        # Initialize WiringOP
        wiringpi.wiringPiSetup()

        # Initialize GPIO pins and log once
        for i, sensor in enumerate(self.sensors):
            self.setup_gpio(sensor["trig"], sensor["echo"])
            self.get_logger().info(
                f"Sensor {i + 4}: Initialized with TRIG pin {sensor['trig']} and ECHO pin {sensor['echo']}"
            )
            
            # Test the sensor with a quick reading
            try:
                test_distance = self.measure_distance(sensor["trig"], sensor["echo"])
                if test_distance == float('inf'):
                    self.get_logger().warn(f"Sensor {i + 4}: No response during initialization test")
                else:
                    self.get_logger().info(f"Sensor {i + 4}: Initial test reading = {test_distance:.2f} cm")
            except Exception as e:
                self.get_logger().error(f"Sensor {i + 4}: Error during initialization test: {str(e)}")

        # Publishers for each sensor
        self.sensor_publishers = []
        for i, _ in enumerate(self.sensors):
            topic_name = f'ultrasonic_sensor_{i + 4}'
            publisher = self.create_publisher(Range, topic_name, 10)
            self.sensor_publishers.append(publisher)

        # Add a lock to prevent overlapping sensor readings
        self.sensor_lock = threading.Lock()
        
        # Last readings to detect stuck sensors
        self.last_readings = [-1.0] * len(self.sensors)  # Initialize to -1 to indicate no previous reading
        self.same_reading_count = [0] * len(self.sensors)
        self.stuck_warning_sent = [False] * len(self.sensors)  # Track if warning was already sent
        self.last_warning_time = [0.0] * len(self.sensors)  # Track when last warning was sent
        
        # Create a timer to read sensors
        self.timer = self.create_timer(0.1, self.read_sensors)

    def setup_gpio(self, trig, echo):
        wiringpi.pinMode(trig, 1)  # Output
        wiringpi.digitalWrite(trig, 0)  # Ensure trigger starts LOW
        wiringpi.pinMode(echo, 0)  # Input

    def reset_sensor(self, trig, echo):
        """Reset a sensor by reconfiguring its GPIO pins"""
        try:
            # Reset pins to safe state
            wiringpi.pinMode(trig, 0)  # Set as input temporarily
            wiringpi.pinMode(echo, 0)  # Set as input
            time.sleep(0.1)  # Wait for pins to settle
            
            # Reconfigure pins
            wiringpi.pinMode(trig, 1)  # Set trigger as output
            wiringpi.digitalWrite(trig, 0)  # Ensure trigger is LOW
            wiringpi.pinMode(echo, 0)  # Set echo as input
            
            self.get_logger().info(f"Reset sensor with TRIG pin {trig} and ECHO pin {echo}")
        except Exception as e:
            self.get_logger().error(f"Error resetting sensor: {str(e)}")

    def read_sensors(self):
        # Skip if already processing to prevent overlap
        if not self.sensor_lock.acquire(blocking=False):
            self.get_logger().debug("Skipping sensor read - previous read still in progress")
            return
        
        try:
            for i, sensor in enumerate(self.sensors):
                try:
                    # Reset trig pin - helps with stability
                    wiringpi.digitalWrite(sensor["trig"], 0)
                    time.sleep(0.001)  # Short delay
                    
                    # Measure distance
                    distance = self.measure_distance(sensor["trig"], sensor["echo"])
                    
                    # Check for stuck readings (same value multiple times)
                    if self.last_readings[i] >= 0:  # Only check if we have a previous reading
                        if abs(distance - self.last_readings[i]) < 1.0:  # Increased threshold to 1.0 cm
                            self.same_reading_count[i] += 1
                            current_time = time.time()
                            
                            # Only warn if we have many consecutive readings AND haven't warned recently
                            if (self.same_reading_count[i] > 200 and  # 20 seconds of same reading
                                current_time - self.last_warning_time[i] > 60.0):  # Only warn every 60 seconds
                                
                                self.get_logger().warn(f"Sensor {i + 4} might be stuck, value: {distance}")
                                self.last_warning_time[i] = current_time
                                
                                # Try to reset the sensor
                                self.reset_sensor(sensor["trig"], sensor["echo"])
                                self.same_reading_count[i] = 0
                                self.last_readings[i] = -1.0  # Reset last reading to force a fresh start
                        else:
                            # Reading changed, reset stuck detection
                            self.same_reading_count[i] = 0
                    
                    self.last_readings[i] = distance
                    
                    # Log and publish
                    if distance == float('inf'):
                        self.get_logger().debug(f"Sensor {i + 4}: No echo received (object out of range)")
                    elif distance < 0.02:
                        self.get_logger().debug(f"Sensor {i + 4}: Object too close, reporting minimum range")
                    else:
                        # Log sensor 4 readings more frequently for debugging
                        if i == 0:  # Sensor 4 (first sensor in the list)
                            self.get_logger().debug(f"Sensor {i + 4}: Distance = {distance:.2f} cm")
                    
                    self.publish_range_msg(i, distance)
                    
                except Exception as e:
                    self.get_logger().error(f"Error reading sensor {i + 4}: {str(e)}")
                    # Always publish something, even on error
                    self.publish_range_msg(i, float('inf'))
                
                # Small delay between sensors to reduce interference
                time.sleep(0.01)
        finally:
            self.sensor_lock.release()

    def measure_distance(self, trig, echo):
        # Send trigger pulse
        wiringpi.digitalWrite(trig, 0)
        time.sleep(0.000002)  # 2 microseconds to ensure LOW
        wiringpi.digitalWrite(trig, 1)
        time.sleep(0.00001)  # 10 microseconds
        wiringpi.digitalWrite(trig, 0)

        # Wait for echo to go HIGH (start of pulse)
        start_time = time.time()
        timeout = start_time + 0.03  # 30ms timeout
        
        # Wait for echo to start (rise)
        while wiringpi.digitalRead(echo) == 0:
            if time.time() > timeout:
                return float('inf')
            start_time = time.time()

        # Wait for echo to go LOW (end of pulse)
        stop_time = time.time()
        timeout = stop_time + 0.03  # 30ms timeout
        
        while wiringpi.digitalRead(echo) == 1:
            if time.time() > timeout:
                return float('inf')
            stop_time = time.time()

        # Calculate pulse duration and distance
        pulse_duration = stop_time - start_time
        distance = (pulse_duration * 34300) / 2  # Speed of sound: 343 m/s
        
        # Sanity check for very large values
        if distance > 400:  # More than 4m
            return float('inf')
        
        # Check if the reading is reasonable (not exactly 63cm repeatedly)
        if 62.5 < distance < 63.5:
            # This might be a stuck sensor reading, log it
            return distance  # Still return the value but it will be flagged as stuck
        
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

    def cleanup_gpio(self):
        """Clean up GPIO (as much as possible with wiringpi)"""
        for sensor in self.sensors:
            # Reset pins to inputs (safer state)
            wiringpi.pinMode(sensor["trig"], 0)
            wiringpi.pinMode(sensor["echo"], 0)

    def destroy_node(self):
        self.cleanup_gpio()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()