import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import board
import busio
import adafruit_lsm9ds1
import math
import time


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # topics
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
        self.timer = self.create_timer(0.02, self.publish_imu)


    def publish_imu(self):
        imu_msg = Imu()
        mag_msg = MagneticField()

        accel_x, accel_y, accel_z = self.sensor.acceleration
        gyro_x, gyro_y, gyro_z = self.sensor.gyro
        mag_x, mag_y, mag_z = self.sensor.magnetic

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = "imu_link"

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

        self.get_logger().info(
            f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f} | "
            f"Gyro: {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f} | "
            f"Mag: {mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()