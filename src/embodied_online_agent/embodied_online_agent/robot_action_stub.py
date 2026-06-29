import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotActionStub(Node):
    """Development-only stand-in for the future UART/SPI control node."""

    def __init__(self):
        super().__init__("robot_action_stub")
        self.ack_pub = self.create_publisher(String, "/robot/action_ack", 10)
        self.create_subscription(String, "/robot/action_command", self._on_action, 10)
        self.get_logger().info("robot action stub ready")

    def _on_action(self, message: String):
        try:
            action = json.loads(message.data)
            name = action["name"]
            self.get_logger().info(f"simulating action: {message.data}")
            ack = {"name": name, "status": "accepted"}
        except (ValueError, KeyError, TypeError) as exc:
            ack = {"status": "rejected", "error": str(exc)}
        self.ack_pub.publish(String(data=json.dumps(ack, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

