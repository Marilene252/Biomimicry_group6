import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty
from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO

IN1, IN2, ENA = 20, 21, 26
IN3, IN4, ENB = 6, 13, 12

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

pwm_ena = GPIO.PWM(ENA, 1000)
pwm_enb = GPIO.PWM(ENB, 1000)
pwm_ena.start(0)
pwm_enb.start(0)

def set_motor_left(speed):
    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_ena.ChangeDutyCycle(abs(speed))
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(0)

def set_motor_right(speed):
    if speed > 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_enb.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_enb.ChangeDutyCycle(abs(speed))
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_enb.ChangeDutyCycle(0)

class PS4ControllerNode(Node):

    def __init__(self):
        super().__init__('ps4_controller_node')

        # Publishers
        self.capture_pub = self.create_publisher(Empty, '/capture_command', 10)
        self.save_pub = self.create_publisher(Empty, '/save_results', 10)

        # Motor control
        self.throttle = 0
        self.steering = 0

        # Start PS4 Controller
        self.controller = MyController(interface="/dev/input/js0",
                                       connecting_using_ds4drv=False,
                                       node=self)
        self.get_logger().info("PS4 controller ready. Use L3 to drive.")

        try:
            self.controller.listen(timeout=60)
        except KeyboardInterrupt:
            self.get_logger().info("Controller stopped by user.")
        finally:
            pwm_ena.stop()
            pwm_enb.stop()
            GPIO.cleanup()

class MyController(Controller):
    def __init__(self, node: Node, **kwargs):
        super().__init__(**kwargs)
        self.node = node
        self.throttle = 0
        self.steering = 0

    def update_motors(self):
        left = max(min(self.throttle + self.steering, 100), -100)
        right = max(min(self.throttle - self.steering, 100), -100)
        set_motor_left(left)
        set_motor_right(right)

    def on_L3_up(self, value):
        self.throttle = int(abs(value)/32767*100)
        self.update_motors()

    def on_L3_down(self, value):
        self.throttle = -int(value/32767*100)
        self.update_motors()

    def on_L3_y_at_rest(self):
        self.throttle = 0
        self.update_motors()

    def on_L3_left(self, value):
        self.steering = -int(abs(value)/32767*100)
        self.update_motors()

    def on_L3_right(self, value):
        self.steering = int(value/32767*100)
        self.update_motors()

    def on_L3_x_at_rest(self):
        self.steering = 0
        self.update_motors()

    def on_L3_press(self):
        self.throttle = 0
        self.steering = 0
        self.update_motors()

    def on_x_press(self):
        self.capture_pub.publish(Empty())
        print("Capture command sent!")

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
