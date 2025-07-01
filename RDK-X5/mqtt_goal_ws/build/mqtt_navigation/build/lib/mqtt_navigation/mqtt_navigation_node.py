#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from threading import Lock
from action_msgs.msg import GoalStatus
import paho.mqtt.client as mqtt
import math
import time
import tf2_ros
from tf2_msgs.msg import TFMessage

class MqttNavigationNode(Node):
    def __init__(self):
        super().__init__('mqtt_navigation_node')

        # Action client for Nav2's NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_goal_handle = None
        self.goal_lock = Lock()
        self.last_goal = None

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Service clients for lifecycle management
        self.lifecycle_cb_group = MutuallyExclusiveCallbackGroup()
        self.controller_lifecycle_client = self.create_client(
            ChangeState, '/controller_server/change_state', callback_group=self.lifecycle_cb_group)
        self.planner_lifecycle_client = self.create_client(
            ChangeState, '/planner_server/change_state', callback_group=self.lifecycle_cb_group)
        self.controller_state_client = self.create_client(
            GetState, '/controller_server/get_state', callback_group=self.lifecycle_cb_group)
        self.planner_state_client = self.create_client(
            GetState, '/planner_server/get_state', callback_group=self.lifecycle_cb_group)

        # Dictionary of predefined locations
        self.locations = {
            "study_room": {"x": -0.725, "y": 0.099, "theta": 0.0},
            "studyroom": {"x": -0.725, "y": 0.099, "theta": 0.0},
            "washroom": {"x": 1.05, "y": -0.187, "theta": 1.57},
            "bed_room": {"x": 0.615, "y": -2.81, "theta": 3.14},
            "bedroom": {"x": 0.615, "y": -2.81, "theta": 3.14},
            "dining_room": {"x": 3.69, "y": 1.87, "theta": -1.57},
            "diningroom": {"x": 3.69, "y": 1.87, "theta": -1.57},
            "living_room": {"x": 0.24, "y": 1.78, "theta": 0.0},
            "livingroom": {"x": 0.24, "y": 1.78, "theta": 0.0}
        }

        # MQTT client setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # MQTT parameters
        self.declare_parameter('mqtt_broker_address', '192.168.0.224')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/control')
        
        mqtt_broker = self.get_parameter('mqtt_broker_address').value
        mqtt_port = self.get_parameter('mqtt_broker_port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        
        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connected to MQTT broker at {mqtt_broker}:{mqtt_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")

        # Check lifecycle services and TF
        self.check_lifecycle_services()
        self.check_tf_transform()

    def check_lifecycle_services(self, timeout_sec=10.0):
        """Check if lifecycle services are available."""
        services = [
            (self.controller_lifecycle_client, '/controller_server/change_state'),
            (self.planner_lifecycle_client, '/planner_server/change_state'),
            (self.controller_state_client, '/controller_server/get_state'),
            (self.planner_state_client, '/planner_server/get_state')
        ]
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f"Service {name} not available after {timeout_sec}s. Ensure Nav2 is running.")
            else:
                self.get_logger().info(f"Service {name} is available")

    def check_tf_transform(self, timeout_sec=5.0):
        """Check if map -> base_link transform is available."""
        try:
            self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=timeout_sec))
            self.get_logger().info("TF transform from map to base_link is available")
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"TF transform from map to base_link not available: {str(e)}. Check map server and localization.")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT connection established, subscribing to {self.mqtt_topic}")
        client.subscribe(self.mqtt_topic)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8').strip()
            self.get_logger().info(f"Received MQTT message: {payload}")
            
            # Handle navigation commands
            if payload in self.locations:
                self.check_tf_transform()  # Check TF before sending goal
                self.process_location_command(payload)
            elif payload == "navStop":
                self.cancel_navigation()
            elif payload == "navPause":  # Fixed command match
                self.pause_navigation()
            elif payload == "navCon":
                self.resume_navigation()
            else:
                self.get_logger().warning(f"Unknown MQTT command: {payload}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def process_location_command(self, location_name):
        if location_name in self.locations:
            location = self.locations[location_name]
            self.get_logger().info(f"Navigating to {location_name}")
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = location['x']
            pose_msg.pose.position.y = location['y']
            pose_msg.pose.position.z = 0.0
            
            theta = location['theta']
            pose_msg.pose.orientation.z = math.sin(theta / 2.0)
            pose_msg.pose.orientation.w = math.cos(theta / 2.0)
            
            self.send_navigation_goal(pose_msg)
        else:
            self.get_logger().error(f"Unknown location: {location_name}")

    def send_navigation_goal(self, pose_msg):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        # Store the last goal
        self.last_goal = pose_msg

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.current_goal_handle = None
            return
        self.get_logger().info("Goal accepted!")
        self.result_future = self.current_goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def cancel_navigation(self):
        with self.goal_lock:
            if self.current_goal_handle is not None:
                future = self.current_goal_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)
                self.get_logger().info("Canceling current goal...")
            else:
                self.get_logger().warning("No active goal to cancel")

    def cancel_done(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                with self.goal_lock:
                    self.current_goal_handle = None
                self.get_logger().info("Goal canceled successfully")
            else:
                self.get_logger().warning("No active goals were canceled")
        except Exception as e:
            self.get_logger().error(f"Cancel error: {str(e)}")

    def get_lifecycle_state(self, client, timeout_sec=10.0, retries=3):
        for attempt in range(retries):
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"Lifecycle state service {client.srv_name} not ready, waiting... (Attempt {attempt + 1}/{retries})")
                if not client.wait_for_service(timeout_sec=timeout_sec):
                    if attempt < retries - 1:
                        time.sleep(1.0)
                        continue
                    return None

            req = GetState.Request()
            try:
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
                if future.result() is not None:
                    state = future.result().current_state
                    return state
                else:
                    self.get_logger().error(
                        f"Failed to get lifecycle state for {client.srv_name}: No response")
            except Exception as e:
                self.get_logger().error(
                    f"Error getting lifecycle state for {client.srv_name}: {str(e)}")

            if attempt < retries - 1:
                time.sleep(1.0)
        return None

    def change_lifecycle_state(self, transition, client, timeout_sec=10.0, retries=3):
        valid_transitions = [Transition.TRANSITION_ACTIVATE,
                             Transition.TRANSITION_DEACTIVATE]
        if transition not in valid_transitions:
            self.get_logger().error(f"Invalid transition ID: {transition}")
            return False

        for attempt in range(retries):
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"Lifecycle service {client.srv_name} not ready, waiting... (Attempt {attempt + 1}/{retries})")
                if not client.wait_for_service(timeout_sec=timeout_sec):
                    if attempt < retries - 1:
                        time.sleep(1.0)
                        continue
                    return False

            req = ChangeState.Request()
            req.transition.id = transition

            try:
                # Get current state for debugging
                current_state = self.get_lifecycle_state(client)
                self.get_logger().info(
                    f"Current state of {client.srv_name}: {current_state}")

                future = client.call_async(req)
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=timeout_sec)
                if future.result() is not None and future.result().success:
                    return True
                else:
                    self.get_logger().error(
                        f"Failed to perform transition {transition} for {client.srv_name}")
            except Exception as e:
                self.get_logger().error(
                    f"Error during transition {transition}: {str(e)}")

            if attempt < retries - 1:
                time.sleep(1.0)
                time.sleep(1.0)  # Add a delay before retrying
        return False

    def pause_navigation(self):
        self.get_logger().info("Attempting to pause navigation...")

        # Cancel the current goal
        self.cancel_navigation()

        # Deactivate components, but only if they are active
        controller_state = self.get_lifecycle_state(
            self.controller_lifecycle_client)
        planner_state = self.get_lifecycle_state(
            self.planner_lifecycle_client)
        success = True
        if controller_state == State.PRIMARY_STATE_ACTIVE:
            success &= self.change_lifecycle_state(
                Transition.TRANSITION_DEACTIVATE, self.controller_lifecycle_client)
        if planner_state == State.PRIMARY_STATE_ACTIVE:
            success &= self.change_lifecycle_state(
                Transition.TRANSITION_DEACTIVATE, self.planner_lifecycle_client)

        if success:
            self.get_logger().info("Navigation paused successfully")
        else:
            self.get_logger().error("Failed to pause navigation")

    def resume_navigation(self):
        self.get_logger().info("Attempting to resume navigation...")

        # Activate components, but only if they are inactive
        controller_state = self.get_lifecycle_state(
            self.controller_lifecycle_client)
        planner_state = self.get_lifecycle_state(
            self.planner_lifecycle_client)
        success = True
        if controller_state == State.PRIMARY_STATE_INACTIVE:
            success &= self.change_lifecycle_state(
                Transition.TRANSITION_ACTIVATE, self.controller_lifecycle_client)
        if planner_state == State.PRIMARY_STATE_INACTIVE:
            success &= self.change_lifecycle_state(
                Transition.TRANSITION_ACTIVATE, self.planner_lifecycle_client)

        if success:
            self.get_logger().info("Navigation components resumed successfully")
            # Reissue the last goal if it exists
            if self.last_goal is not None:
                self.send_navigation_goal(self.last_goal)
                self.get_logger().info("Resent the previous navigation goal.")
            else:
                self.get_logger().warning(
                    "No previous goal to resume.  Send a new goal.")
        else:
            self.get_logger().error("Failed to resume navigation")

    def goal_result_callback(self, future):
        try:
            result = future.result()
            with self.goal_lock:
                if self.current_goal_handle:
                    status = result.status
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info("Navigation completed successfully!")
                    elif status == GoalStatus.STATUS_CANCELED:
                        self.get_logger().info("Navigation canceled!")
                    else:
                        self.get_logger().error(f"Navigation failed with status: {status}")
                    self.current_goal_handle = None
        except Exception as e:
            self.get_logger().error(f"Result error: {str(e)}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_x = feedback.current_pose.pose.position.x
        current_y = feedback.current_pose.pose.position.y
        self.get_logger().info(f"Current position: {current_x:.2f}, {current_y:.2f}")

    def __del__(self):
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = MqttNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()