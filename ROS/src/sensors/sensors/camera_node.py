import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import time

class RPICamNode(Node):
    def __init__(self):
        super().__init__('rpicam_node')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/image_raw', 10)

        # Subscribe to capture command
        self.create_subscription(Empty, '/capture_command', self.capture_cb, 10)

        self.temp_file = '/tmp/rpicam_image.jpg'
        self.get_logger().info("RPICam node ready")

    def capture_cb(self, msg):
        """Triggered when /capture_command is received"""
        self.get_logger().info("Capture command received")

        cmd = ['rpicam-still', '-o', self.temp_file]
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"rpicam-still failed: {e}")
            return

        time.sleep(0.1)

        if not os.path.exists(self.temp_file):
            self.get_logger().error("Image file not found")
            return

        img = cv2.imread(self.temp_file)
        if img is None:
            self.get_logger().error("Failed to read image")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.pub.publish(msg)
            self.get_logger().info("Image published to /image_raw")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RPICamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
