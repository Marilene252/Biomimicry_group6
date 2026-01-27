#In this script everything from the example codes is implemented. Now, the BTS7960 motor drivers are working and the robot drives because of this complete script.

from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time

L_RPWM = 5
L_LPWM = 17 #  PWM pins for motordriver 1

# Enable pins for motordriver 1

M1_L_EN = 6
M1_R_EN = 27

R_RPWM = 19
R_LPWM = 16  # PWM pin for motordriver 2

# Enable pins for motordriver 2

M2_L_EN = 26
M2_R_EN = 13

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(L_RPWM, GPIO.OUT)
GPIO.setup(L_LPWM, GPIO.OUT)
GPIO.setup(R_RPWM, GPIO.OUT)
GPIO.setup(R_LPWM, GPIO.OUT)

# Setup pins as output
GPIO.setup(M1_L_EN, GPIO.OUT)
GPIO.setup(M1_R_EN, GPIO.OUT)
GPIO.setup(M2_L_EN, GPIO.OUT)
GPIO.setup(M2_R_EN, GPIO.OUT)

#Enable motor drivers
GPIO.output(M1_L_EN, GPIO.HIGH)
GPIO.output(M1_R_EN, GPIO.HIGH)
GPIO.output(M2_L_EN, GPIO.HIGH)
GPIO.output(M2_R_EN, GPIO.HIGH)

# PWM setup
pwm_left_fwd = GPIO.PWM(L_RPWM, 1000) # Frequency is set to 1000 Hz
pwm_left_bwd = GPIO.PWM(L_LPWM, 1000)

pwm_right_fwd = GPIO.PWM(R_RPWM, 1000)
pwm_right_bwd = GPIO.PWM(R_LPWM, 1000)

#Start everything on 0
pwm_left_fwd.start(0)
pwm_left_bwd.start(0)
pwm_right_fwd.start(0)
pwm_right_bwd .start(0)

def set_motor_left(speed):
    """
    Controls left BTS7960 Motor Driver
    speed: value between -100 and 100
    """
    if speed > 0: 
        #Forwards
        pwm_left_fwd.ChangeDutyCycle(speed)
        pwm_left_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Backwards
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(0)

def set_motor_right(speed):
    """
    Controls right BTS7960 Motor Driver
    speed: value between -100 and 100
    """
    if speed > 0: 
        #Forwards
        pwm_right_fwd.ChangeDutyCycle(speed)
        pwm_right_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Backwards
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(0)

class MyController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        #Monitors gas levels and maek adjustments
        self.throttle = 0
        self.steering = 0

    def update_motors(self):
        """
        Calculates the speed for left on right based on Throttle and Steering
        """
        #Links = Throttle + Steering
        #Rechts = Throttle - Steering
        left_speed = self.throttle + self.steering
        right_speed = self.throttle - self.steering

        #Values not above 100 or under -100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        #Control the Motors
        set_motor_left(left_speed)
        set_motor_right(right_speed)
        
    def on_L3_up(self, value):
        #Value is -32767 (full up) to 0. Converted to 0-100
        self.throttle = int(abs(value) / 32767 * 100)
        self.update_motors()

    def on_L3_down(self, value):
        #The same applies on this as to on_L3_up
        self.throttle = -int(value / 32767 * 100)
        self.update_motors()

    def on_L3_y_at_rest(self):
        #Joystick loose (no throttle)
        self.throttle = 0
        self.update_motors()

    #L3 Left / Right (Steering)

    def on_L3_left(self, value):
        #Steering to the left: Left slower, right faster
        #Negative value converterd to -100 to 0
        steering_val = int(abs(value) / 32767 * 100)
        self.steering = -steering_val #Left is a negevative value
        self.update_motors()

    def on_L3_right(self, value):
        #Steering to the right
        steering_val = int(value / 32767 * 100)
        self.steering = steering_val
        self.update_motors()

    def on_L3_x_at_rest(self):
        #Joystick loose (No steering)
        self.steering = 0
        self.update_motors()

    def on_square_press(self):
        #Emergency stop when the square button is pressed
        self.throttle = 0
        self.steering = 0
        self.update_motors()

try:
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    print("Controller connected. Use L3 to drive.")
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    pwm_left_fwd.stop()
    pwm_left_bwd.stop()
    pwm_right_fwd.stop()
    pwm_right_bwd.stop()
    GPIO.cleanup()

