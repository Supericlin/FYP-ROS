import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math

class AndroidNavigationNode(Node):
    def __init__(self):
        super().__init__('android_navigation_node')

        # Action client for Nav2's NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscriber to listen for goals from the Android app
        self.subscription = self.create_subscription(
            PoseStamped,
            '/android_goal',  # Topic where the Android app will publish goals
            self.goal_callback,
            10
        )

        self.get_logger().info("Android Navigation Node is ready and listening on /android_goal")

    def goal_callback(self, msg):
        """Callback function when a goal is received from the Android app."""
        self.get_logger().info(f"Received navigation goal from Android at ({msg.pose.position.x}, {msg.pose.position.y})")

        # Create the action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg  # Use the PoseStamped directly

        # Send the goal to the action server
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the action server!")
            return

        self.get_logger().info("Goal accepted, navigating to the target...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle the result of the action."""
        result = future.result().result
        self.get_logger().info(f"Navigation completed with result: {result}")

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current position: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    node = AndroidNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()