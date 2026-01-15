import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DistanceSensor
import numpy as np
from time import sleep, time

class DistanceSensorNode(Node):
    def __init__(self):
        super().__init__('distance_sensor')

        self.sensor = DistanceSensor(echo=24, trigger=23, max_distance=4)
        self.publisher = self.create_publisher(Float32, '/distance_mm', 10)
        self.timer = self.create_timer(1.0, self.publish_distance)

    def get_stable_distance_mm(self, duration=1.0, tol=2.0):
        readings = []
        start = time()

        while time() - start < duration:
            d = self.sensor.distance * 1000  

            if 0 < d <= 400:
                readings.append(d)

            sleep(0.05)

        if len(readings) < 5:
            return None

        readings = np.array(readings)
        median = np.median(readings)
        stable = readings[np.abs(readings - median) < tol]

        return float(np.mean(stable)) if len(stable) else None

    def publish_distance(self):
        distance = self.get_stable_distance_mm()
        if distance is not None:
            msg = Float32()
            msg.data = distance
            self.publisher.publish(msg)

def main():
    rclpy.init()
    node = DistanceSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
