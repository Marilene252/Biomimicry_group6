import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestListener(Node):

    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.callback,
            10
        )
        self.get_logger().info("Listener ready...")

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TestListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
