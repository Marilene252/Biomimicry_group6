import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TempHumidityPublisher(Node):
    def __init__(self):
        super().__init__('temp_humidity_pub_node')

        self.temp_publisher = self.create_publisher(
            Float32,
            'temperature',
            10
        )

        self.humidity_publisher = self.create_publisher(
            Float32,
            'humidity',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        temperature = 25.0
        humidity = 60.0

        self.temp_publisher.publish(Float32(data=temperature))
        self.humidity_publisher.publish(Float32(data=humidity))

        self.get_logger().info(
            f'Temp: {temperature} C | Humidity: {humidity} %'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TempHumidityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
