import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from datetime import datetime
import matplotlib.pyplot as plt

class ResultsNode(Node):
    def __init__(self):
        super().__init__('results_node')

        self.bridge = CvBridge()

        # Latest messages storage
        self.latest_raw = None
        self.latest_segmented = None
        self.latest_areas = None
        self.latest_m50 = None

        # Subscribers
        self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.create_subscription(Image, '/segment_image', self.segment_cb, 10)
        self.create_subscription(Float32, '/m50_result', self.m50_cb, 10)
        self.create_subscription(Float32MultiArray, '/grain_areas_mm2', self.areas_cb, 10)

    # -------- Callbacks --------
    def image_cb(self, msg):
        self.latest_raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def segment_cb(self, msg):
        self.latest_segmented = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def m50_cb(self, msg):
        self.latest_m50 = msg.data
        self.try_save_results()

    def areas_cb(self, msg):
        self.latest_areas = np.array(msg.data)

    # -------- Save results --------
    def try_save_results(self):
        if self.latest_raw is None or self.latest_segmented is None \
           or self.latest_areas is None or self.latest_m50 is None:
            return  # Wait until all data is ready

        # Create timestamped folder
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        folder = os.path.join(os.path.dirname(__file__), 'results', timestamp)
        os.makedirs(folder, exist_ok=True)

        # --- Save images ---
        cv2.imwrite(os.path.join(folder, 'raw_image.png'), self.latest_raw)
        cv2.imwrite(os.path.join(folder, 'segmented.png'), self.latest_segmented)

        # --- Compute D10/D50/D90 ---
        areas = self.latest_areas
        D10 = np.percentile(areas, 10)
        D50 = np.percentile(areas, 50)
        D90 = np.percentile(areas, 90)

        # --- Save stats.txt ---
        with open(os.path.join(folder, 'stats.txt'), 'w') as f:
            f.write(f"M50: {self.latest_m50:.2f} mm²\n")
            f.write(f"D10: {D10:.2f} mm²\n")
            f.write(f"D50: {D50:.2f} mm²\n")
            f.write(f"D90: {D90:.2f} mm²\n")
            f.write(f"Number of grains: {len(areas)}\n")

        # --- Histogram ---
        bins = np.logspace(np.log10(areas.min()), np.log10(areas.max()), 20)
        hist, bin_edges = np.histogram(areas, bins=bins)

        plt.figure()
        plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor='black', align='edge')
        plt.xscale('log')
        plt.xlabel('Grain area (mm²)')
        plt.ylabel('Count')
        plt.title('Grain Size Histogram')
        plt.savefig(os.path.join(folder, 'grain_histogram.png'))
        plt.close()

        # --- Save CSV of bins ---
        np.savetxt(
            os.path.join(folder, 'bins.csv'),
            np.column_stack((bin_edges[:-1], hist)),
            delimiter=',',
            header='bin_start_mm2,count',
            comments=''
        )

        self.get_logger().info(f"Saved results in {folder}")

        # Reset latest data for next measurement
        self.latest_raw = None
        self.latest_segmented = None
        self.latest_areas = None
        self.latest_m50 = None


def main(args=None):
    rclpy.init(args=args)
    node = ResultsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
