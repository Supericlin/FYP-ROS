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

        # Declare parameters for logging control (optimized)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('log_interval', 30.0)  # Log status every 30 seconds (optimized)
        self.declare_parameter('enable_detailed_logging', False)
        
        # Get parameters
        self.debug_mode = self.get_parameter('debug_mode').value
        self.log_interval = self.get_parameter('log_interval').value
        self.enable_detailed_logging = self.get_parameter('enable_detailed_logging').value

        # Sensor GPIO pins
        self.sensors = [
            {'trigger': 29, 'echo': 31, 'name': 'Front'},  # Sensor 1
            {'trigger': 37, 'echo': 36, 'name': 'Left'},   # Sensor 2
            {'trigger': 18, 'echo': 22, 'name': 'Right'},  # Sensor 3
        ]

        # Simplified performance tracking (reduced overhead)
        self.read_count = 0
        self.error_count = 0
        self.sensor_stats = {i: {'reads': 0, 'errors': 0, 'last_distance': 0.0} for i in range(len(self.sensors))}
        
        # Status tracking
        self.node_start_time = time.time()
        self.is_initialized = False
        self.gpio_initialized = False

        self.get_logger().info("US100 Ultrasonic Sensor Node starting...")
        self.get_logger().info(f"Configuration: {len(self.sensors)} sensors, Debug: {self.debug_mode}")

        # Initialize GPIO
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            for i, sensor in enumerate(self.sensors):
                GPIO.setup(sensor['trigger'], GPIO.OUT)
                GPIO.setup(sensor['echo'], GPIO.IN)
                GPIO.output(sensor['trigger'], GPIO.LOW)  # Ensure triggers start LOW
                self.get_logger().info(f"Sensor {i+1} ({sensor['name']}) initialized - Trigger: {sensor['trigger']}, Echo: {sensor['echo']}")
            
            self.gpio_initialized = True
            self.get_logger().info("GPIO initialization completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {str(e)}")
            raise

        # Create publishers for each sensor
        self.sensor_publishers = []
        for i, sensor in enumerate(self.sensors):
            topic_name = f'ultrasonic_sensor_{i + 1}'
            publisher = self.create_publisher(Range, topic_name, 10)
            self.sensor_publishers.append(publisher)
            self.get_logger().info(f"Publisher created: {topic_name}")

        # Use a lock to prevent overlapping sensor readings
        self.sensor_lock = threading.Lock()
        
        # Create a timer to read sensors
        self.timer = self.create_timer(0.1, self.read_sensors)  # 10 Hz
        
        # Create a timer for status logging (only if interval > 0)
        if self.log_interval > 0:
            self.status_timer = self.create_timer(self.log_interval, self.log_status)
        
        self.is_initialized = True
        self.get_logger().info("US100 Sensor Node fully initialized and ready")
        self.get_logger().info(f"Reading sensors at 10Hz, status logs every {self.log_interval}s")

    def log_status(self):
        """Simplified status logging (optimized for CPU)"""
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
                self.get_logger().info(f"Sensor {i+1} ({sensor['name']}): Reads={stats['reads']}, Errors={stats['errors']} ({sensor_error_rate:.1f}%), Last={stats['last_distance']:.3f}m")

    def read_sensors(self):
        # Skip if we're already processing
        if not self.sensor_lock.acquire(blocking=False):
            return
            
        try:
            self.read_count += 1
            
            for i, sensor in enumerate(self.sensors):
                try:
                    distance = self.get_distance(sensor['trigger'], sensor['echo'])
                    
                    # Update statistics (simplified)
                    self.sensor_stats[i]['reads'] += 1
                    self.sensor_stats[i]['last_distance'] = distance / 100.0 if distance != float('inf') else 0.0
                    
                    # Log detailed information only if explicitly enabled
                    if self.enable_detailed_logging:
                        if distance == float('inf'):
                            self.get_logger().debug(f"Sensor {i+1} ({sensor['name']}): No echo received")
                        elif distance <= 0.02:
                            self.get_logger().debug(f"Sensor {i+1} ({sensor['name']}): Object too close")
                        else:
                            self.get_logger().debug(f"Sensor {i+1} ({sensor['name']}): Distance = {distance/100.0:.3f}m")
                    
                    self.publish_range_msg(i, distance)
                    
                except Exception as e:
                    self.error_count += 1
                    self.sensor_stats[i]['errors'] += 1
                    self.get_logger().error(f"Error reading sensor {i+1} ({sensor['name']}): {str(e)}")
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

        # Validate distance reading
        if distance < 0 or distance > 400:  # 4m max range
            return float('inf')

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
        """Simplified shutdown"""
        self.get_logger().info("US100 Sensor Node shutting down...")
        
        # Log final statistics (simplified)
        total_reads = sum(stats['reads'] for stats in self.sensor_stats.values())
        total_errors = sum(stats['errors'] for stats in self.sensor_stats.values())
        uptime = time.time() - self.node_start_time
        
        self.get_logger().info(f"Final stats: Uptime={uptime:.0f}s, Reads={total_reads}, Errors={total_errors}")
        
        # Ensure we clean up GPIO on shutdown
        try:
            GPIO.cleanup()
            self.get_logger().info("GPIO cleanup completed")
        except Exception as e:
            self.get_logger().error(f"GPIO cleanup failed: {str(e)}")
        
        self.get_logger().info("US100 Sensor Node shutdown complete")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    
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