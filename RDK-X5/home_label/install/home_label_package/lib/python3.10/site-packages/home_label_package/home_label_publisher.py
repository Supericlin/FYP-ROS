import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class RoomLabelPublisher(Node):
    def __init__(self):
        super().__init__("home_label_publisher")
        self.publisher = self.create_publisher(Marker, "/marker", 10)

        # Create a timer to call publish_markers every 1 second
        self.timer = self.create_timer(10.0, self.publish_markers)

    def publish_markers(self):
        marker_list = [
            {"name": "Study_Room", "x": -0.725, "y": 0.099, "z": 0.00226},
            {"name": "Washroom", "x": 1.05, "y": -0.187, "z": 0.000485},
            {"name": "Bedroom", "x": 0.615, "y": -2.81, "z": -0.0018},
            {"name": "Corridor", "x": 0.922, "y": 0.579, "z": 0.000775},
            {"name": "Dining_Room", "x": 3.69, "y": 1.87, "z": 0.000647},
            {"name": "Living_Room", "x": 0.24, "y": 1.78, "z": 0.00119},
        ]

        for idx, room in enumerate(marker_list):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "room_labels"
            marker.id = idx
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = room["x"]
            marker.pose.position.y = room["y"]
            marker.pose.position.z = room["z"]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4  # Text size
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 0.4  # Red
            marker.color.g = 0.4  # Green
            marker.color.b = 0.4  # Blue
            marker.text = room["name"]

            self.publisher.publish(marker)
            self.get_logger().info(f"Published marker for {room['name']} at ({room['x']}, {room['y']})")

def main(args=None):
    rclpy.init(args=args)
    node = RoomLabelPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()