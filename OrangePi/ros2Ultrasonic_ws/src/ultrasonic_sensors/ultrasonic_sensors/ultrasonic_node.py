#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import wiringpi
import time
import sys
import threading

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # Declare parameters for logging control (matching RDK-X5 style)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('log_interval', 30.0)  # Log status every 30 seconds
        self.declare_parameter('enable_detailed_logging', False)
        
        # Get parameters
        self.debug_mode = self.get_parameter('debug_mode').value
        self.log_interval = self.get_parameter('log_interval').value
        self.enable_detailed_logging = self.get_parameter('enable_detailed_logging').value

        # GPIO pin assignments
        self.sensors = [
            #{"trig": 19, "echo": 20, "name": "Sensor 3"},  # Sensor 3 (commented out)
            {"trig": 24, "echo": 25, "name": "Sensor 4"},   # Sensor 4
            {"trig": 10, "echo": 13, "name": "Sensor 5"},   # Sensor 5
        ]

        # Performance tracking (matching RDK-X5 style)
        self.read_count = 0
        self.error_count = 0
        self.sensor_stats = {i: {'reads': 0, 'errors': 0, 'last_distance': 0.0} for i in range(len(self.sensors))}
        
        # Status tracking
        self.node_start_time = time.time()
        self.is_initialized = False
        self.gpio_initialized = False

        self.get_logger().info("OrangePi Ultrasonic Sensor Node starting...")
        self.get_logger().info(f"Configuration: {len(self.sensors)} sensors, Debug: {self.debug_mode}")

        # Initialize WiringOP
        try:
            wiringpi.wiringPiSetup()
            
            # Initialize GPIO pins and log once
            for i, sensor in enumerate(self.sensors):
                self.setup_gpio(sensor["trig"], sensor["echo"])
                self.get_logger().info(f"Sensor {i + 4} ({sensor['name']}) initialized - Trigger: {sensor['trig']}, Echo: {sensor['echo']}")
                
                # Test the sensor with a quick reading
                try:
                    test_distance = self.measure_distance(sensor["trig"], sensor["echo"])
                    if test_distance == float('inf'):
                        self.get_logger().warn(f"Sensor {i + 4}: No response during initialization test")
                    else:
                        self.get_logger().info(f"Sensor {i + 4}: Initial test reading = {test_distance:.2f} cm")
                except Exception as e:
                    self.get_logger().error(f"Sensor {i + 4}: Error during initialization test: {str(e)}")
            
            self.gpio_initialized = True
            self.get_logger().info("GPIO initialization completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {str(e)}")
            raise

        # Publishers for each sensor
        self.sensor_publishers = []
        for i, _ in enumerate(self.sensors):
            topic_name = f'ultrasonic_sensor_{i + 4}'
            publisher = self.create_publisher(Range, topic_name, 10)
            self.sensor_publishers.append(publisher)
            self.get_logger().info(f"Publisher created: {topic_name}")

        # Add a lock to prevent overlapping sensor readings
        self.sensor_lock = threading.Lock()
        
        # Last readings to detect stuck sensors (enhanced)
        self.last_readings = [-1.0] * len(self.sensors)
        self.same_reading_count = [0] * len(self.sensors)
        self.stuck_warning_sent = [False] * len(self.sensors)
        self.last_warning_time = [0.0] * len(self.sensors)
        
        # Create a timer to read sensors
        self.timer = self.create_timer(0.1, self.read_sensors)  # 10 Hz
        
        # Create a timer for status logging (only if interval > 0)
        if self.log_interval > 0:
            self.status_timer = self.create_timer(self.log_interval, self.log_status)
        
        self.is_initialized = True
        self.get_logger().info("OrangePi Ultrasonic Sensor Node fully initialized and ready")
        self.get_logger().info(f"Reading sensors at 10Hz, status logs every {self.log_interval}s")

    def log_status(self):
        """Simplified status logging (matching RDK-X5 style)"""
        current_time = time.time()
        uptime = current_time - self.node_start_time
        
        # Calculate basic statistics
        total_reads = sum(stats['reads'] for stats in self.sensor_stats.values())
        total_errors = sum(stats['errors'] for stats in self.sensor_stats.values())
        error_rate = (total_errors / max(total_reads, 1)) * 100
        
        # Simplified status log
        self.get_logger().info(f"Status: Uptime={uptime:.0f}s, Reads={total_reads}, Errors={total_errors} ({error_rate:.1f}%)")
        
        # Only log individual sensors if debug mode is enabled
        if self.debug_mode:
            for i, sensor in enumerate(self.sensors):
                stats = self.sensor_stats[i]
                sensor_error_rate = (stats['errors'] / max(stats['reads'], 1)) * 100
                self.get_logger().info(f"Sensor {i+4} ({sensor['name']}): Reads={stats['reads']}, Errors={stats['errors']} ({sensor_error_rate:.1f}%), Last={stats['last_distance']:.3f}m")

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
            return
        
        try:
            self.read_count += 1
            
            for i, sensor in enumerate(self.sensors):
                try:
                    # Reset trig pin - helps with stability
                    wiringpi.digitalWrite(sensor["trig"], 0)
                    time.sleep(0.001)  # Short delay
                    
                    # Measure distance
                    distance = self.measure_distance(sensor["trig"], sensor["echo"])
                    
                    # Update statistics (matching RDK-X5 style)
                    self.sensor_stats[i]['reads'] += 1
                    self.sensor_stats[i]['last_distance'] = distance / 100.0 if distance != float('inf') else 0.0
                    
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
                    
                    # Log detailed information only if explicitly enabled
                    if self.enable_detailed_logging:
                        if distance == float('inf'):
                            self.get_logger().debug(f"Sensor {i + 4}: No echo received (object out of range)")
                        elif distance < 0.02:
                            self.get_logger().debug(f"Sensor {i + 4}: Object too close, reporting minimum range")
                        else:
                            self.get_logger().debug(f"Sensor {i + 4}: Distance = {distance:.2f} cm")
                    
                    self.publish_range_msg(i, distance)
                    
                except Exception as e:
                    self.error_count += 1
                    self.sensor_stats[i]['errors'] += 1
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
        """Clean up GPIO pins"""
        try:
            for sensor in self.sensors:
                wiringpi.pinMode(sensor["trig"], 0)  # Set as input
                wiringpi.pinMode(sensor["echo"], 0)  # Set as input
            self.get_logger().info("GPIO cleanup completed")
        except Exception as e:
            self.get_logger().error(f"GPIO cleanup failed: {str(e)}")

    def destroy_node(self):
        """Simplified shutdown (matching RDK-X5 style)"""
        self.get_logger().info("OrangePi Ultrasonic Sensor Node shutting down...")
        
        # Log final statistics (simplified)
        total_reads = sum(stats['reads'] for stats in self.sensor_stats.values())
        total_errors = sum(stats['errors'] for stats in self.sensor_stats.values())
        uptime = time.time() - self.node_start_time
        
        self.get_logger().info(f"Final stats: Uptime={uptime:.0f}s, Reads={total_reads}, Errors={total_errors}")
        
        # Clean up GPIO
        self.cleanup_gpio()
        
        self.get_logger().info("OrangePi Ultrasonic Sensor Node shutdown complete")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by user (Ctrl+C)")
    except Exception as e:
        node.get_logger().error(f"Unexpected error occurred: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()