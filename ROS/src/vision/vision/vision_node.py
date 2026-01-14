import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
from pipeline_vision import run_segmentation, compute_grain_size_pixels  


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_distance = None

        self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.create_subscription(Float32, '/distance_mm', self.distance_cb, 10)
        self.create_subscription(Empty, '/capture_command', self.capture_cb, 10)
        
        self.areas_pub = self.create_publisher(Float32MultiArray, '/grain_areas_mm2', 10)
        self.m50_pub = self.create_publisher(Float32, '/m50_result', 10)
        self.debug_pub = self.create_publisher(Image, '/segment_image', 10)

        self.get_logger().info("Vision node ready")

    def image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def distance_cb(self, msg):
        self.latest_distance = msg.data

    def capture_cb(self, msg):
        """Triggered when /capture_command is received"""
        if self.latest_image is None or self.latest_distance is None:
            self.get_logger().warn("Missing image or distance")
            return

        debug_img, m50, areas_mm, px_to_mm = run_segmentation(
            self.latest_image,
            self.latest_distance
        )

        self.m50_pub.publish(Float32(data=m50))

        try:
            img_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

        areas_msg = Float32MultiArray()
        areas_msg.data = areas_mm.tolist()
        self.areas_pub.publish(areas_msg)

        self.get_logger().info(f"Published M50: {m50:.3f}, Grain count: {len(areas_mm)}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
