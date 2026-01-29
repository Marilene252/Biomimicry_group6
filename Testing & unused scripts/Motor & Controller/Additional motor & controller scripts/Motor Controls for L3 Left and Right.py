"""
This Script exlains how to put the Motors on and how the PS4 controller can control the motors,
so the robot can drive forwards and backwards.
"""

from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

IN1 = 20
IN2 = 21
ENA = 26   # PWM pins for motor 1

IN3 = 6
IN4 = 13
ENB = 12   # PWM pins for motor 2 (unused for now)`

GPIO.setmode(GPIO.BCM)

# Motors 1 and 2 pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Motors 3 and 4 pins
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM setup
pwm_ena = GPIO.PWM(ENA, 1000)
pwm_enb = GPIO.PWM(ENB, 1000)

pwm_ena.start(0)
pwm_enb.start(0)

def motor1_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_ena.ChangeDutyCycle(speed)

def motor1_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_ena.ChangeDutyCycle(speed)

def motor1_brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm_ena.ChangeDutyCycle(0)

class MyController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.speed = 0   # store current speed for smooth updates

    # L3 LEFT/RIGHT → Motor control
    # Left:  negative value  (-32768..0)
    # Right: positive value   (0..32767)

    def on_L3_left(self, value):
        """
        Stick moved left.
        value ranges from -32768 to 0, we convert to a positive speed.
        """
        speed = int(abs(value) / 32767 * 100)  # Convert stick value to 0-100%
        motor1_backward(speed)

    def on_L3_right(self, value):
        """
        Stick moved right.
        value ranges from 0 to 32767 → convert to 0-100%.
        """
        speed = int(value / 32767 * 100)
        motor1_forward(speed)

    def on_L3_x_at_rest(self):
        """Joystick released: stop motor"""
        motor1_brake()

    # Pressing L3 fully stops motor
    def on_L3_press(self):
        motor1_brake()

try:
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    pwm_ena.stop()
    pwm_enb.stop()
    GPIO.cleanup()

