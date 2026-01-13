from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

# Pin setup
IN1 = 5
IN2 = 4
ENA = 6

IN3 = 8
IN4 = 7
ENB = 9

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)  # or GPIO.BOARD depending on your wiring

# Setup pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Create PWM objects for ENA and ENB for motor speed control
pwm_ena = GPIO.PWM(ENA, 1000)  # Set PWM frequency to 1kHz
pwm_enb = GPIO.PWM(ENB, 1000)

pwm_ena.start(0)  # Start PWM with 0% duty cycle (motor stopped initially)
pwm_enb.start(0)


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")

    def on_triangle_press(self):
        print("on_triangle_press")

    def on_triangle_release(self):
        print("on_triangle_release")

    def on_circle_press(self):
        print("on_circle_press")

    def on_circle_release(self):
        print("on_circle_release")

    def on_square_press(self):
        print("on_square_press")

    def on_square_release(self):
        print("on_square_release")

    def on_L1_press(self):
        print("on_L1_press")

    def on_L1_release(self):
        print("on_L1_release")

    def on_L2_press(self, value):
        print("on_L2_press: {}".format(value))

    def on_L2_release(self):
        print("on_L2_release")

    def on_R1_press(self):
        print("on_R1_press")

    def on_R1_release(self):
        print("on_R1_release")

    def on_R2_press(self, value):
        print("on_R2_press: {}".format(value))

    def on_R2_release(self):
        print("on_R2_release")

    def on_up_arrow_press(self):
        print("on_up_arrow_press")

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")

    def on_down_arrow_press(self):
        print("on_down_arrow_press")

    def on_left_arrow_press(self):
        print("on_left_arrow_press")

    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")

    def on_right_arrow_press(self):
        print("on_right_arrow_press")

    def on_L3_up(self, value):
        print("on_L3_up: {}".format(value))

    def on_L3_down(self, value):
        print("on_L3_down: {}".format(value))

    def on_L3_left(self, value):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        pwm_ena.ChangeDutyCycle(speed)

    def on_L3_right(self, value):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        pwm_ena.ChangeDutyCycle(speed)  # Control motor speed with PWM

    def on_L3_y_at_rest(self):
        print("on_L3_y_at_rest")

    def on_L3_x_at_rest(self):
        print("on_L3_x_at_rest")

    def on_L3_press(self):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    def on_L3_release(self):
        print("on_L3_release")

    def on_R3_up(self, value):
        print("on_R3_up: {}".format(value))

    def on_R3_down(self, value):
        print("on_R3_down: {}".format(value))

    def on_R3_left(self, value):
        print("on_R3_left: {}".format(value))

    def on_R3_right(self, value):
        print("on_R3_right: {}".format(value))

    def on_R3_y_at_rest(self):
        print("on_R3_y_at_rest")

    def on_R3_x_at_rest(self):
        print("on_R3_x_at_rest")

    def on_R3_press(self):
        print("on_R3_press")

    def on_R3_release(self):
        print("on_R3_release")

    def on_options_press(self):
        print("on_options_press")

    def on_options_release(self):
        print("on_options_release")

    def on_share_press(self):
        print("on_share_press")

    def on_share_release(self):
        print("on_share_release")

    def on_playstation_button_press(self):
        print("on_playstation_button_press")

    def on_playstation_button_release(self):
        print("on_playstation_button_release")

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)