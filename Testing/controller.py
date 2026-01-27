from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO
import time
import subprocess 
import sys
from imu_logger import IMULogger
import threading

TEMP_HUM_SCRIPT = "/home/rapi6/Biomimicry_group6/System/hum_temp.py"
IMAGE_CAPTURE_SCRIPT = "/home/rapi6/Biomimicry_group6/System/pipeline_image_capture.py"


L_RPWM = 4
L_LPWM = 17 #  PWM pin for motor 1 (Brown Wheels)
#Enable pins
M1_L_EN = 27
M1_R_EN = 22

R_RPWM = 5
R_LPWM = 6  # PWM pin for motor 2 (Pink wheels)
# Enable pins
M2_L_EN = 13
M2_R_EN = 19


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(L_RPWM, GPIO.OUT)
GPIO.setup(L_LPWM, GPIO.OUT)
GPIO.setup(R_RPWM, GPIO.OUT)
GPIO.setup(R_LPWM, GPIO.OUT)

# Alles instellen op OUT
GPIO.setup(M1_L_EN, GPIO.OUT)
GPIO.setup(M1_R_EN, GPIO.OUT)
GPIO.setup(M2_L_EN, GPIO.OUT)
GPIO.setup(M2_R_EN, GPIO.OUT)

# Alles instellen op HIGH
GPIO.output(M1_L_EN, GPIO.HIGH)
GPIO.output(M1_R_EN, GPIO.HIGH)
GPIO.output(M2_L_EN, GPIO.HIGH)
GPIO.output(M2_R_EN, GPIO.HIGH)


# PWM setup
pwm_left_fwd = GPIO.PWM(L_RPWM, 1000)
pwm_left_bwd = GPIO.PWM(L_LPWM, 1000)

pwm_right_fwd = GPIO.PWM(R_RPWM, 1000)
pwm_right_bwd = GPIO.PWM(R_LPWM, 1000)

#Start alles op 0
pwm_left_fwd.start(0)
pwm_left_bwd.start(0)
pwm_right_fwd.start(0)
pwm_right_bwd .start(0)

def set_motor_left(speed):
    """
    Stuurt linker BTS7960 aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        pwm_left_fwd.ChangeDutyCycle(speed)
        pwm_left_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Achteruit
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(0)

def set_motor_right(speed):
    """
    Stuurt rechter BTS7960 driver aan
    speed: waarde tussen -100 en 100
    """
    if speed > 0: 
        #Vooruit
        pwm_right_fwd.ChangeDutyCycle(speed)
        pwm_right_bwd.ChangeDutyCycle(0)
    elif speed < 0:
        #Achteruit
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(abs(speed))
    else:
        #Stop
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(0)

class MyController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        #We houden de stand van gas en sturen bij
        self.throttle = 0
        self.steering = 0
        self.imu_logger = IMULogger()

    def update_motors(self):
        """
        Berekent de snelheid voor links en rechts op basis van throttle en steering.
        """
        #Links = Gas + Stuur
        #Rechts = Gas - Stuur
        left_speed = self.throttle + self.steering
        right_speed = self.throttle - self.steering

        #Waarden niet boven 100 of onder -100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        #Stuur de motoren aan
        set_motor_left(left_speed)
        set_motor_right(right_speed)
        
    def on_L3_up(self, value):
        #Value is -32767 (vol omhoog) tot 0. Omgezet tot 0-100
        self.throttle = int(abs(value) / 32767 * 100)
        self.update_motors()

    def on_L3_down(self, value):
        #Voor deze value geldt hetzelfde als bij on_L3_up
        self.throttle = -int(value / 32767 * 100)
        self.update_motors()

    def on_L3_y_at_rest(self):
        #Joystick lostaten (geen gas)
        self.throttle = 0
        self.update_motors()

    #L3 Links / Rechts (Sturen)

    def on_L3_left(self, value):
        #Naar links sturen: Links langzamer, rechts sneller
        #Negatieve waarde geconverteerd naar -100 tot 0
        steering_val = int(abs(value) / 32767 * 100)
        self.steering = -steering_val #Links is negatieve waarde
        self.update_motors()

    def on_L3_right(self, value):
        #Naar rechts sturen
        steering_val = int(value / 32767 * 100)
        self.steering = steering_val
        self.update_motors()

    def on_L3_x_at_rest(self):
        #Joystick losgelaten (Niet sturen)
        self.steering = 0
        self.update_motors()
        
    def on_triangle_press(self):
        print("Triangle START IMU LOGGING")
        self.imu_logger.start_logging()

    def on_square_press(self):
        print("square STOP IMU LOGGING")
        self.imu_logger.stop_logging()

    def on_options_press(self):
        print("OPTIONS EMERGENCY STOP")

        self.throttle = 0
        self.steering = 0
        self.update_motors()


    def on_circle_press(self):
        print("O button pressed, Reading temperature & humidity.")
        try:
            subprocess.run([sys.executable, TEMP_HUM_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running temp/humidity script: {e}")

    def on_x_press(self):
        print("X button pressed, Running image capture pipeline.")
        try:
            subprocess.run([sys.executable, IMAGE_CAPTURE_SCRIPT], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running image capture script: {e}")


def imu_background(controller):

    while True:
        controller.imu_logger.update()
        time.sleep(0.05)


try:
    controller = MyController(
        interface="/dev/input/js0",
        connecting_using_ds4drv=False
    )
    print("Controller verbonden. Gebruik L3 om te rijden.")

    threading.Thread(
        target=imu_background,
        args=(controller,),
        daemon=True
    ).start()

    controller.listen(timeout=999999)

except KeyboardInterrupt:
    print("Onderbroken door gebruiker.")
finally:
    pwm_left_fwd.stop()
    pwm_left_bwd.stop()
    pwm_right_fwd.stop()
    pwm_right_bwd.stop()
    GPIO.cleanup()

